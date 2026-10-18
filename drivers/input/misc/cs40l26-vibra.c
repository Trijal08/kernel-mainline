// SPDX-License-Identifier: GPL-2.0
/*
 * CS40L26 Boosted Haptic Driver with Integrated DSP and Waveform Memory
 *
 * Copyright 2024 Cirrus Logic, Inc.
 * Copyright 2026 Trijal Saha <trijalsaha2012@gmail.com>
 *
 * Modelled on the CS40L50 driver by James Ogletree <james.ogletree@cirrus.com>.
 */

#include <linux/input.h>
#include <linux/mfd/cs40l26.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

/*
 * Waveform banks.  An effect is played by writing its index to the mailbox:
 * RAM holds the waveforms the wavetable file supplied, ROM the ones built into
 * the part.
 */
#define CS40L26_RAM_INDEX_START		0x1000000
#define CS40L26_RAM_INDEX_END		0x100007F
#define CS40L26_ROM_INDEX_START		0x1800000
#define CS40L26_ROM_INDEX_END		0x1800026

#define CS40L26_CUSTOM_DATA_SIZE	2
#define CS40L26_CUSTOM_DATA_MASK	0xFFFFU

/*
 * The part plays one waveform at a time, but an effect here is only a
 * reference into the wavetable and costs no device memory, so userspace may
 * keep several uploaded and choose between them.
 */
#define CS40L26_EFFECTS_MAX		16

enum cs40l26_bank_type {
	CS40L26_WVFRM_BANK_RAM,
	CS40L26_WVFRM_BANK_ROM,
	CS40L26_WVFRM_BANK_NUM,
};

struct cs40l26_bank {
	u32 base_index;
	u32 max_index;
};

static const struct cs40l26_bank cs40l26_banks[CS40L26_WVFRM_BANK_NUM] = {
	[CS40L26_WVFRM_BANK_RAM] = {
		.base_index =	CS40L26_RAM_INDEX_START,
		.max_index =	CS40L26_RAM_INDEX_END,
	},
	[CS40L26_WVFRM_BANK_ROM] = {
		.base_index =	CS40L26_ROM_INDEX_START,
		.max_index =	CS40L26_ROM_INDEX_END,
	},
};

struct cs40l26_effect {
	struct list_head list;
	u32 index;
	int id;
};

struct cs40l26_vibra {
	struct cs40l26 *core;
	struct device *dev;
	struct regmap *regmap;
	struct input_dev *input;
	struct workqueue_struct *vib_wq;
	struct list_head effect_head;
};

struct cs40l26_work {
	struct cs40l26_vibra *vib;
	struct ff_effect *effect;
	struct work_struct work;
	s16 *custom_data;
	int custom_len;
	int count;
	int error;
};

static struct cs40l26_effect *cs40l26_find_effect(int id, struct list_head *effect_head)
{
	struct cs40l26_effect *effect;

	list_for_each_entry(effect, effect_head, list)
		if (effect->id == id)
			return effect;

	return NULL;
}

static int cs40l26_effect_index_set(struct cs40l26_work *work_data,
				    struct cs40l26_effect *effect)
{
	u32 bank = work_data->custom_data[0] & CS40L26_CUSTOM_DATA_MASK;
	u32 index = work_data->custom_data[1] & CS40L26_CUSTOM_DATA_MASK;
	struct cs40l26_vibra *vib = work_data->vib;

	if (bank >= CS40L26_WVFRM_BANK_NUM) {
		dev_err(vib->dev, "Invalid bank (%u)\n", bank);
		return -EINVAL;
	}

	effect->index = cs40l26_banks[bank].base_index + index;

	if (effect->index > cs40l26_banks[bank].max_index) {
		dev_err(vib->dev, "Index out of bounds: %u\n", index);
		return -ENOSPC;
	}

	return 0;
}

/*
 * FF_RUMBLE asks for a buzz of a given strength for a given time, which is
 * what the firmware's buzz generator produces.  Program it and hand back its
 * waveform index, so playing it is no different from playing a wavetable
 * entry.
 */
static int cs40l26_buzz_set(struct cs40l26_work *work_data,
			    struct cs40l26_effect *effect)
{
	struct ff_periodic_effect *periodic = &work_data->effect->u.periodic;
	struct cs40l26 *core = work_data->vib->core;
	u16 length = work_data->effect->replay.length;
	unsigned int period;
	__be32 val;
	int ret;

	/*
	 * The generator wants a frequency where the effect carries a period,
	 * and only a narrow range of them.  An effect converted from
	 * FF_RUMBLE arrives with 50 ms, which clamps to the lowest tone the
	 * generator will produce.
	 */
	period = clamp_t(u16, periodic->period, CS40L26_BUZZGEN_PERIOD_MIN,
			 CS40L26_BUZZGEN_PERIOD_MAX);
	val = cpu_to_be32(MSEC_PER_SEC / period);
	ret = cs_dsp_coeff_lock_and_write_ctrl(core->buzz_freq, 0, &val, sizeof(val));
	if (ret < 0)
		return ret;

	/*
	 * Scale the magnitude into the byte-wide level rather than clamping
	 * it: force feedback states a magnitude as a proportion of full
	 * scale, and clamping would play every request above a thirtieth of
	 * it at full strength.
	 */
	val = cpu_to_be32(clamp_t(int, periodic->magnitude >> 7, 0,
				  CS40L26_BUZZGEN_LEVEL_MAX));
	ret = cs_dsp_coeff_lock_and_write_ctrl(core->buzz_level, 0, &val, sizeof(val));
	if (ret < 0)
		return ret;

	if (!length)
		length = CS40L26_BUZZGEN_MS_DFL;

	val = cpu_to_be32(DIV_ROUND_UP(length, CS40L26_BUZZGEN_MS_PER_UNIT));
	ret = cs_dsp_coeff_lock_and_write_ctrl(core->buzz_duration, 0, &val, sizeof(val));
	if (ret < 0)
		return ret;

	effect->index = CS40L26_BUZZGEN_INDEX;

	return 0;
}

static void cs40l26_add_worker(struct work_struct *work)
{
	struct cs40l26_work *work_data = container_of(work, struct cs40l26_work, work);
	struct cs40l26_vibra *vib = work_data->vib;
	struct cs40l26_effect *effect;
	bool is_new = false;
	int error;

	error = pm_runtime_resume_and_get(vib->dev);
	if (error)
		goto err_exit;

	/* Update the effect if already uploaded, otherwise create a new one */
	effect = cs40l26_find_effect(work_data->effect->id, &vib->effect_head);
	if (!effect) {
		effect = kzalloc_obj(*effect);
		if (!effect) {
			error = -ENOMEM;
			goto err_pm;
		}

		effect->id = work_data->effect->id;
		is_new = true;
	}

	if (work_data->effect->u.periodic.waveform == FF_SINE)
		error = cs40l26_buzz_set(work_data, effect);
	else
		error = cs40l26_effect_index_set(work_data, effect);

	if (is_new) {
		if (error)
			kfree(effect);
		else
			list_add(&effect->list, &vib->effect_head);
	}
err_pm:
	pm_runtime_put_autosuspend(vib->dev);
err_exit:
	work_data->error = error;
}

static int cs40l26_add(struct input_dev *dev, struct ff_effect *effect,
		       struct ff_effect *old)
{
	struct ff_periodic_effect *periodic = &effect->u.periodic;
	struct cs40l26_vibra *vib = input_get_drvdata(dev);
	struct cs40l26_work work_data = {};

	if (effect->type != FF_PERIODIC) {
		dev_err(vib->dev, "Effect type %#X unsupported\n", effect->type);
		return -EINVAL;
	}

	switch (periodic->waveform) {
	case FF_SINE:
		/*
		 * Fully described by the effect itself.  This is also how an
		 * FF_RUMBLE request arrives: the input core rewrites it to a
		 * sine whose magnitude averages the two rumble strengths.
		 */
		break;
	case FF_CUSTOM:
		if (periodic->custom_len < CS40L26_CUSTOM_DATA_SIZE) {
			dev_err(vib->dev, "Invalid custom data length (%u)\n",
				periodic->custom_len);
			return -EINVAL;
		}

		work_data.custom_data = memdup_array_user(periodic->custom_data,
							  CS40L26_CUSTOM_DATA_SIZE,
							  sizeof(s16));
		if (IS_ERR(work_data.custom_data))
			return PTR_ERR(work_data.custom_data);

		work_data.custom_len = CS40L26_CUSTOM_DATA_SIZE;
		break;
	default:
		dev_err(vib->dev, "Waveform %#X unsupported\n", periodic->waveform);
		return -EINVAL;
	}

	work_data.vib = vib;
	work_data.effect = effect;
	INIT_WORK_ONSTACK(&work_data.work, cs40l26_add_worker);

	/* Push to the workqueue to serialize with playbacks */
	queue_work(vib->vib_wq, &work_data.work);
	flush_work(&work_data.work);
	destroy_work_on_stack(&work_data.work);

	kfree(work_data.custom_data);

	return work_data.error;
}

static void cs40l26_start_worker(struct work_struct *work)
{
	struct cs40l26_work *work_data = container_of(work, struct cs40l26_work, work);
	struct cs40l26_vibra *vib = work_data->vib;
	struct cs40l26_effect *start_effect;

	if (pm_runtime_resume_and_get(vib->dev) < 0)
		goto err_free;

	start_effect = cs40l26_find_effect(work_data->effect->id, &vib->effect_head);
	if (start_effect) {
		while (--work_data->count >= 0) {
			cs40l26_dsp_write(vib->dev, vib->regmap, start_effect->index);
			usleep_range(work_data->effect->replay.length,
				     work_data->effect->replay.length + 100);
		}
	} else {
		dev_err(vib->dev, "Effect to play not found\n");
	}

	pm_runtime_put_autosuspend(vib->dev);
err_free:
	kfree(work_data);
}

static void cs40l26_stop_worker(struct work_struct *work)
{
	struct cs40l26_work *work_data = container_of(work, struct cs40l26_work, work);
	struct cs40l26_vibra *vib = work_data->vib;

	if (pm_runtime_resume_and_get(vib->dev) < 0)
		goto err_free;

	cs40l26_dsp_write(vib->dev, vib->regmap, CS40L26_STOP_PLAYBACK);

	pm_runtime_put_autosuspend(vib->dev);
err_free:
	kfree(work_data);
}

static int cs40l26_playback(struct input_dev *dev, int effect_id, int val)
{
	struct cs40l26_vibra *vib = input_get_drvdata(dev);
	struct cs40l26_work *work_data;

	work_data = kzalloc_obj(*work_data, GFP_ATOMIC);
	if (!work_data)
		return -ENOMEM;

	work_data->vib = vib;

	if (val > 0) {
		work_data->effect = &dev->ff->effects[effect_id];
		work_data->count = val;
		INIT_WORK(&work_data->work, cs40l26_start_worker);
	} else {
		/* Stop the amplifier as the device drives only one effect */
		INIT_WORK(&work_data->work, cs40l26_stop_worker);
	}

	queue_work(vib->vib_wq, &work_data->work);

	return 0;
}

static void cs40l26_erase_worker(struct work_struct *work)
{
	struct cs40l26_work *work_data = container_of(work, struct cs40l26_work, work);
	struct cs40l26_vibra *vib = work_data->vib;
	struct cs40l26_effect *erase_effect;

	erase_effect = cs40l26_find_effect(work_data->effect->id, &vib->effect_head);
	if (!erase_effect) {
		dev_err(vib->dev, "Effect to erase not found\n");
		work_data->error = -EINVAL;
		return;
	}

	list_del(&erase_effect->list);
	kfree(erase_effect);
}

static int cs40l26_erase(struct input_dev *dev, int effect_id)
{
	struct cs40l26_vibra *vib = input_get_drvdata(dev);
	struct cs40l26_work work_data = {};

	work_data.vib = vib;
	work_data.effect = &dev->ff->effects[effect_id];
	INIT_WORK_ONSTACK(&work_data.work, cs40l26_erase_worker);

	/* Push to the workqueue to serialize with playbacks */
	queue_work(vib->vib_wq, &work_data.work);
	flush_work(&work_data.work);
	destroy_work_on_stack(&work_data.work);

	return work_data.error;
}

static void cs40l26_remove_wq(void *data)
{
	flush_workqueue(data);
	destroy_workqueue(data);
}

static int cs40l26_vibra_probe(struct platform_device *pdev)
{
	struct cs40l26 *cs40l26 = dev_get_drvdata(pdev->dev.parent);
	struct cs40l26_vibra *vib;
	int error;

	vib = devm_kzalloc(&pdev->dev, sizeof(*vib), GFP_KERNEL);
	if (!vib)
		return -ENOMEM;

	vib->core = cs40l26;
	vib->dev = cs40l26->dev;
	vib->regmap = cs40l26->regmap;

	vib->input = devm_input_allocate_device(vib->dev);
	if (!vib->input)
		return -ENOMEM;

	vib->input->id.product = cs40l26->devid;
	vib->input->id.version = cs40l26->revid;
	vib->input->name = "cs40l26_vibra";

	input_set_drvdata(vib->input, vib);
	input_set_capability(vib->input, EV_FF, FF_PERIODIC);
	input_set_capability(vib->input, EV_FF, FF_CUSTOM);
	input_set_capability(vib->input, EV_FF, FF_SINE);

	error = input_ff_create(vib->input, CS40L26_EFFECTS_MAX);
	if (error) {
		dev_err(vib->dev, "Failed to create FF device: %d\n", error);
		return error;
	}

	/*
	 * input_ff_create() offers FF_RUMBLE on the strength of FF_PERIODIC,
	 * expecting the memless emulation to serve it.  This driver serves it
	 * from the buzz generator instead, so withdraw the offer when the
	 * firmware did not bring one.
	 */
	if (!cs40l26->buzz_freq || !cs40l26->buzz_level || !cs40l26->buzz_duration) {
		dev_warn(vib->dev, "No buzz generator, FF_RUMBLE unavailable\n");
		__clear_bit(FF_RUMBLE, vib->input->ffbit);
		__clear_bit(FF_SINE, vib->input->ffbit);
	}

	vib->input->ff->upload = cs40l26_add;
	vib->input->ff->playback = cs40l26_playback;
	vib->input->ff->erase = cs40l26_erase;

	INIT_LIST_HEAD(&vib->effect_head);

	vib->vib_wq = alloc_ordered_workqueue("vib_wq", WQ_HIGHPRI);
	if (!vib->vib_wq)
		return -ENOMEM;

	error = devm_add_action_or_reset(vib->dev, cs40l26_remove_wq, vib->vib_wq);
	if (error)
		return error;

	return input_register_device(vib->input);
}

static const struct platform_device_id cs40l26_vibra_id_match[] = {
	{ "cs40l26-vibra", },
	{}
};
MODULE_DEVICE_TABLE(platform, cs40l26_vibra_id_match);

static struct platform_driver cs40l26_vibra_driver = {
	.probe		= cs40l26_vibra_probe,
	.id_table	= cs40l26_vibra_id_match,
	.driver		= {
		.name	= "cs40l26-vibra",
	},
};
module_platform_driver(cs40l26_vibra_driver);

MODULE_DESCRIPTION("CS40L26 Boosted Haptic Driver");
MODULE_AUTHOR("Trijal Saha <trijalsaha2012@gmail.com>");
MODULE_LICENSE("GPL");
