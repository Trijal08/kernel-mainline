// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Exynos Bus Traffic Shaper interconnect provider
 *
 * The IP-BTS register operations and Zumapro policy data are ported from the
 * downstream Google/Samsung GPL driver.  The integration uses the mainline
 * interconnect and common-clock frameworks instead of Exynos-private PM QoS.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/interconnect-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/slab.h>

#include <dt-bindings/interconnect/google,zumapro.h>

#define BTS_CON			0x00
#define BTS_TIMEOUT_R0		0x08
#define BTS_TIMEOUT_W0		0x10
#define BTS_RCON		0x20
#define BTS_RBLK_UPPER		0x24
#define BTS_WCON		0x40
#define BTS_WBLK_UPPER		0x44

#define BTS_CON_QOS_EN		BIT(0)
#define BTS_CON_EX_QURGENT_EN	BIT(7)
#define BTS_CON_QURGENT_EN	BIT(23)
#define BTS_XCON_BLOCKING_EN	BIT(0)
#define BTS_XCON_QOS_OVERRIDE	BIT(8)
#define BTS_XCON_QOS		GENMASK(15, 12)
#define BTS_BLOCK_UPPER		GENMASK(15, 0)

#define BTS_NUM_CHANNELS	4U
#define BTS_MIF_BUS_WIDTH	16U
#define BTS_INT_BUS_WIDTH	32U
#define BTS_NOCL2A_CHANNELS	2U
#define BTS_MIF_UTIL		65U
#define BTS_INT_UTIL		65U
#define BTS_RT_UTIL		40U

struct exynos_bts_profile {
	u16 rmo;
	u16 wmo;
	u8 arqos;
	u8 awqos;
	u8 qurgent_th_r;
	u8 qurgent_th_w;
	bool qurgent;
	bool ex_qurgent;
};

struct exynos_bts_block_data {
	const char *name;
	const struct exynos_bts_profile *default_profile;
};

struct exynos_bts_override {
	unsigned int block;
	const struct exynos_bts_profile *profile;
};

/* A zero tag applies whenever the master has a nonzero bandwidth vote. */
struct exynos_bts_policy {
	u32 tag;
	u32 mif_floor;
	u32 int_floor;
	unsigned int priority;
	const struct exynos_bts_override *overrides;
	unsigned int num_overrides;
};

struct exynos_bts_master_data {
	unsigned int id;
	const char *name;
	u32 bus_width;
	bool realtime;
	u32 mif_floor;
	u32 int_floor;
	const struct exynos_bts_policy *policies;
	unsigned int num_policies;
};

struct exynos_bts_block {
	const struct exynos_bts_block_data *data;
	void __iomem *base;
};

struct exynos_bts_master {
	const struct exynos_bts_master_data *data;
	struct icc_node *node;
	u32 tags;
	u32 applied_tags;
	bool active;
};

struct exynos_bts {
	struct device *dev;
	/* Serializes shared clock floors and register profiles. */
	struct mutex lock;
	struct clk *mif_clk;
	struct clk *int_clk;
	unsigned long saved_mif_rate;
	unsigned long saved_int_rate;
	unsigned long target_mif_rate;
	unsigned long target_int_rate;
	bool rates_saved;
	bool active;

	struct exynos_bts_block *blocks;
	unsigned int num_blocks;
	struct exynos_bts_master *masters;
	unsigned int num_masters;

	struct icc_provider provider;
	struct icc_onecell_data *data;
};

/* Profiles from the downstream Zuma BTS description. */
static const struct exynos_bts_profile zumapro_mfc_default = {
	.rmo = 0x10,
	.wmo = 0x10,
	.arqos = 0x4,
	.awqos = 0x4,
	.qurgent_th_r = 0xff,
	.qurgent_th_w = 0xff,
};

static const struct exynos_bts_profile zumapro_mfc_uhd = {
	.rmo = 0x14,
	.wmo = 0x14,
	.arqos = 0x4,
	.awqos = 0x4,
	.qurgent_th_r = 0xff,
	.qurgent_th_w = 0xff,
};

static const struct exynos_bts_block_data zumapro_blocks[] = {
	{ "mfc0", &zumapro_mfc_default },
	{ "mfc1", &zumapro_mfc_default },
};

static const struct exynos_bts_override zumapro_mfc_uhd_overrides[] = {
	{ 0, &zumapro_mfc_uhd },
	{ 1, &zumapro_mfc_uhd },
};

/*
 * Upper MIF/INT floors of the downstream entries sharing each of the five
 * MFC clock rungs. The combined read/write ICC path also accounts for the
 * uncompressed traffic; these floors cover the workload's latency demand.
 */
static const struct exynos_bts_policy zumapro_mfc_policies[] = {
	{ .tag = ZUMAPRO_BTS_MFC_310, .mif_floor = 676000, .int_floor = 200000 },
	{ .tag = ZUMAPRO_BTS_MFC_400, .mif_floor = 1014000, .int_floor = 356000 },
	{ .tag = ZUMAPRO_BTS_MFC_465, .mif_floor = 1539000, .int_floor = 356000 },
	{
		.tag = ZUMAPRO_BTS_MFC_664,
		.mif_floor = 1539000,
		.int_floor = 533000,
		.priority = 1,
		.overrides = zumapro_mfc_uhd_overrides,
		.num_overrides = ARRAY_SIZE(zumapro_mfc_uhd_overrides),
	},
};

static const struct exynos_bts_master_data zumapro_masters[] = {
	{
		.id = ZUMAPRO_BTS_MASTER_MFC,
		.name = "mfc",
		.bus_width = 16,
		.mif_floor = 421000,
		.int_floor = 178000,
		.policies = zumapro_mfc_policies,
		.num_policies = ARRAY_SIZE(zumapro_mfc_policies),
	},
};

/* Live Android devfreq OPP ladders, in kHz, highest first. */
static const u32 zumapro_mif_rates[] = {
	3744000, 3172000, 2730000, 2288000, 2028000, 1716000, 1539000,
	1352000, 1014000, 845000, 676000, 546000, 421000,
};

static const u32 zumapro_int_rates[] = {
	1066000, 799000, 664000, 533000, 356000, 200000, 178000,
};

struct exynos_bts_bus1_map {
	u32 bus1;
	u32 int_freq;
};

static const struct exynos_bts_bus1_map zumapro_bus1_int_map[] = {
	{ 620000, 1066000 },
	{ 533000, 664000 },
	{ 400000, 533000 },
	{ 267000, 356000 },
	{ 200000, 200000 },
	{ 134000, 178000 },
};

static void exynos_bts_program_block(void __iomem *base,
				     const struct exynos_bts_profile *profile)
{
	u32 val;

	/* AxQoS override and value. */
	val = readl(base + BTS_RCON);
	val &= ~(BTS_XCON_QOS_OVERRIDE | BTS_XCON_QOS);
	val |= BTS_XCON_QOS_OVERRIDE |
	       FIELD_PREP(BTS_XCON_QOS, profile->arqos);
	writel(val, base + BTS_RCON);

	val = readl(base + BTS_WCON);
	val &= ~(BTS_XCON_QOS_OVERRIDE | BTS_XCON_QOS);
	val |= BTS_XCON_QOS_OVERRIDE |
	       FIELD_PREP(BTS_XCON_QOS, profile->awqos);
	writel(val, base + BTS_WCON);

	val = readl(base + BTS_CON);
	val |= BTS_CON_QOS_EN;
	writel(val, base + BTS_CON);

	/* Maximum outstanding read and write transactions. */
	val = readl(base + BTS_RBLK_UPPER);
	val &= ~BTS_BLOCK_UPPER;
	val |= FIELD_PREP(BTS_BLOCK_UPPER, profile->rmo);
	writel(val, base + BTS_RBLK_UPPER);

	val = readl(base + BTS_WBLK_UPPER);
	val &= ~BTS_BLOCK_UPPER;
	val |= FIELD_PREP(BTS_BLOCK_UPPER, profile->wmo);
	writel(val, base + BTS_WBLK_UPPER);

	/* The same urgent threshold applies to all four virtual channels. */
	val = profile->qurgent_th_r * 0x01010101U;
	writel(val, base + BTS_TIMEOUT_R0);
	val = profile->qurgent_th_w * 0x01010101U;
	writel(val, base + BTS_TIMEOUT_W0);

	val = readl(base + BTS_CON);
	val &= ~(BTS_CON_QURGENT_EN | BTS_CON_EX_QURGENT_EN);
	if (profile->qurgent)
		val |= BTS_CON_QURGENT_EN;
	if (profile->ex_qurgent)
		val |= BTS_CON_EX_QURGENT_EN;
	writel(val, base + BTS_CON);

	/* None of the scoped Zumapro profiles enables transaction blocking. */
	val = readl(base + BTS_RCON);
	val &= ~BTS_XCON_BLOCKING_EN;
	writel(val, base + BTS_RCON);
	val = readl(base + BTS_WCON);
	val &= ~BTS_XCON_BLOCKING_EN;
	writel(val, base + BTS_WCON);
}

static void exynos_bts_program_profiles(struct exynos_bts *bts)
{
	unsigned int i, j, k, n;

	for (i = 0; i < bts->num_blocks; i++) {
		const struct exynos_bts_profile *profile =
			bts->blocks[i].data->default_profile;
		unsigned int priority = 0;

		for (j = 0; j < bts->num_masters; j++) {
			struct exynos_bts_master *master = &bts->masters[j];

			if (!master->active)
				continue;
			for (k = 0; k < master->data->num_policies; k++) {
				const struct exynos_bts_policy *policy =
					&master->data->policies[k];

				if (policy->tag && !(master->applied_tags & policy->tag))
					continue;
				if (policy->priority < priority)
					continue;
				for (n = 0; n < policy->num_overrides; n++) {
					if (policy->overrides[n].block != i)
						continue;
					profile = policy->overrides[n].profile;
					priority = policy->priority;
				}
			}
		}
		exynos_bts_program_block(bts->blocks[i].base, profile);
	}
}

static u32 exynos_bts_ceil_rate(const u32 *rates, size_t count, u32 floor)
{
	size_t i;

	for (i = count; i-- > 0;)
		if (floor <= rates[i])
			return rates[i];

	return rates[0];
}

static u32 exynos_bts_bus1_to_int(u32 freq)
{
	size_t i = ARRAY_SIZE(zumapro_bus1_int_map);

	while (--i > 0)
		if (freq <= zumapro_bus1_int_map[i].bus1)
			break;

	return zumapro_bus1_int_map[i].int_freq;
}

static u32 exynos_bts_mif_floor(u32 total_bw, u32 rt_bw)
{
	u64 mif_freq;
	u64 rt_freq;

	/* Preserve downstream truncation without its u32 multiplication wrap. */
	mif_freq = (u64)(total_bw / BTS_MIF_BUS_WIDTH) * 100 / BTS_MIF_UTIL;
	rt_freq = (u64)(rt_bw / BTS_MIF_BUS_WIDTH) * 100 / BTS_RT_UTIL;

	return max(mif_freq, rt_freq);
}

static u32 exynos_bts_int_floor(u32 total_bw, u32 peak_freq)
{
	u64 bus1_freq;
	u64 int_freq;
	u64 freq;

	/*
	 * A combined read/write path bounds each direction by total bandwidth.
	 * This is exact for a unidirectional client and conservative otherwise.
	 */
	bus1_freq = peak_freq;
	freq = (total_bw / BTS_INT_BUS_WIDTH) / BTS_NOCL2A_CHANNELS;
	freq = freq * 100 / BTS_INT_UTIL;
	bus1_freq = max(bus1_freq, freq);

	/* NOCL1A sees the aggregate traffic over four channels. */
	int_freq = (total_bw / BTS_INT_BUS_WIDTH) / BTS_NUM_CHANNELS;
	int_freq = int_freq * 100 / BTS_INT_UTIL;

	return max_t(u64, int_freq, exynos_bts_bus1_to_int(bus1_freq));
}

/* CCF can discard the provider's set_rate error; check the recalculated rate. */
static int exynos_bts_set_rate(struct clk *clk, unsigned long rate)
{
	int ret = clk_set_rate(clk, rate);

	if (ret)
		return ret;
	return clk_get_rate(clk) == rate ? 0 : -EIO;
}

static int exynos_bts_set_active(struct exynos_bts *bts, u32 total_bw,
				 u32 rt_bw, u32 peak_freq,
				 u32 mif_floor, u32 int_floor)
{
	unsigned long old_mif, mif_rate, int_rate;
	int rollback_ret, ret;

	mif_floor = max(mif_floor, exynos_bts_mif_floor(total_bw, rt_bw));
	int_floor = max(int_floor, exynos_bts_int_floor(total_bw, peak_freq));
	mif_rate = (unsigned long)exynos_bts_ceil_rate(zumapro_mif_rates,
				ARRAY_SIZE(zumapro_mif_rates), mif_floor) * 1000;
	int_rate = (unsigned long)exynos_bts_ceil_rate(zumapro_int_rates,
				ARRAY_SIZE(zumapro_int_rates), int_floor) * 1000;

	if (!bts->rates_saved) {
		bts->saved_mif_rate = clk_get_rate(bts->mif_clk);
		bts->saved_int_rate = clk_get_rate(bts->int_clk);
		if (!bts->saved_mif_rate || !bts->saved_int_rate)
			return -EIO;
		bts->rates_saved = true;
	}
	mif_rate = max(mif_rate, bts->saved_mif_rate);
	int_rate = max(int_rate, bts->saved_int_rate);
	old_mif = bts->target_mif_rate ?: bts->saved_mif_rate;

	ret = exynos_bts_set_rate(bts->mif_clk, mif_rate);
	if (ret)
		return ret;
	ret = exynos_bts_set_rate(bts->int_clk, int_rate);
	if (ret) {
		rollback_ret = exynos_bts_set_rate(bts->mif_clk, old_mif);
		if (rollback_ret) {
			dev_err(bts->dev, "cannot roll MIF back to %lu Hz: %d\n",
				old_mif, rollback_ret);
		}
		return ret;
	}
	bts->target_mif_rate = mif_rate;
	bts->target_int_rate = int_rate;
	bts->active = true;
	dev_dbg(bts->dev, "%u KB/s, peak %u kHz -> MIF %lu INT %lu Hz\n",
		total_bw, peak_freq, mif_rate, int_rate);
	return 0;
}

static int exynos_bts_set_idle(struct exynos_bts *bts)
{
	unsigned int i;
	int rollback_ret, ret;

	if (bts->rates_saved) {
		ret = exynos_bts_set_rate(bts->mif_clk, bts->saved_mif_rate);
		if (ret)
			return ret;
		ret = exynos_bts_set_rate(bts->int_clk, bts->saved_int_rate);
		if (ret) {
			if (bts->target_mif_rate) {
				rollback_ret = exynos_bts_set_rate(bts->mif_clk,
							bts->target_mif_rate);
				if (rollback_ret)
					dev_err(bts->dev, "cannot restore MIF: %d\n",
						rollback_ret);
			}
			return ret;
		}
	}

	for (i = 0; i < bts->num_masters; i++)
		bts->masters[i].active = false;
	if (bts->active)
		exynos_bts_program_profiles(bts);
	bts->rates_saved = false;
	bts->active = false;
	bts->saved_mif_rate = 0;
	bts->saved_int_rate = 0;
	bts->target_mif_rate = 0;
	bts->target_int_rate = 0;
	return 0;
}

static int exynos_bts_icc_set(struct icc_node *src, struct icc_node *dst)
{
	struct exynos_bts *bts = container_of(src->provider, struct exynos_bts,
					    provider);
	u64 total_bw = 0, rt_bw = 0;
	u32 peak_freq = 0, mif_floor = 0, int_floor = 0;
	unsigned int i, j;
	int ret;

	if (dst->id != ZUMAPRO_BTS_SLAVE_MIF)
		return 0;

	mutex_lock(&bts->lock);
	for (i = 0; i < bts->num_masters; i++) {
		struct exynos_bts_master *master = &bts->masters[i];
		struct icc_node *node = master->node;

		/* Downstream requires both total and peak, or neither. */
		if (!node->avg_bw != !node->peak_bw) {
			ret = -EINVAL;
			goto out;
		}
		if (!node->avg_bw)
			continue;
		total_bw += node->avg_bw;
		if (master->data->realtime)
			rt_bw += node->avg_bw;
		peak_freq = max_t(u64, peak_freq,
			(u64)(node->peak_bw / master->data->bus_width) * 100 / BTS_INT_UTIL);
		mif_floor = max(mif_floor, master->data->mif_floor);
		int_floor = max(int_floor, master->data->int_floor);
		for (j = 0; j < master->data->num_policies; j++) {
			const struct exynos_bts_policy *policy = &master->data->policies[j];

			if (policy->tag && !(master->tags & policy->tag))
				continue;
			mif_floor = max(mif_floor, policy->mif_floor);
			int_floor = max(int_floor, policy->int_floor);
		}
	}
	if (!total_bw) {
		ret = exynos_bts_set_idle(bts);
		goto out;
	}
	ret = exynos_bts_set_active(bts, min_t(u64, total_bw, U32_MAX),
				   min_t(u64, rt_bw, U32_MAX), peak_freq,
				   mif_floor, int_floor);
	if (!ret) {
		bool changed = false;

		for (i = 0; i < bts->num_masters; i++) {
			struct exynos_bts_master *master = &bts->masters[i];
			bool active = master->node->avg_bw != 0;

			changed |= master->active != active || master->applied_tags != master->tags;
			master->active = active;
			master->applied_tags = master->tags;
		}
		if (changed)
			exynos_bts_program_profiles(bts);
	}
out:
	mutex_unlock(&bts->lock);
	return ret;
}

static void exynos_bts_pre_aggregate(struct icc_node *node)
{
	struct exynos_bts_master *master = node->data;

	if (master)
		master->tags = 0;
}

static int exynos_bts_aggregate(struct icc_node *node, u32 tag,
			      u32 avg_bw, u32 peak_bw, u32 *agg_avg, u32 *agg_peak)
{
	struct exynos_bts_master *master = node->data;

	if (master && (avg_bw || peak_bw))
		master->tags |= tag;
	return icc_std_aggregate(node, tag, avg_bw, peak_bw, agg_avg, agg_peak);
}

static int exynos_bts_icc_get_bw(struct icc_node *node, u32 *avg, u32 *peak)
{
	*avg = 0;
	*peak = 0;

	return 0;
}

static void exynos_bts_unregister(void *data)
{
	struct exynos_bts *bts = data;
	unsigned int i;
	int ret;

	icc_provider_deregister(&bts->provider);
	mutex_lock(&bts->lock);
	ret = exynos_bts_set_idle(bts);
	if (ret)
		dev_warn(bts->dev,
			 "cannot restore idle clock rates while removing: %d\n",
			 ret);
	for (i = 0; i < bts->num_masters; i++)
		bts->masters[i].active = false;
	exynos_bts_program_profiles(bts);
	mutex_unlock(&bts->lock);
	icc_nodes_remove(&bts->provider);
}

static int exynos_bts_register(struct exynos_bts *bts)
{
	struct icc_provider *provider = &bts->provider;
	struct icc_node *node, *slave;
	unsigned int i, count = ZUMAPRO_BTS_SLAVE_MIF + 1;
	int ret;

	for (i = 0; i < bts->num_masters; i++)
		count = max(count, bts->masters[i].data->id + 1);
	bts->data = devm_kzalloc(bts->dev, struct_size(bts->data, nodes, count), GFP_KERNEL);
	if (!bts->data)
		return -ENOMEM;
	bts->data->num_nodes = count;
	provider->dev = bts->dev;
	provider->set = exynos_bts_icc_set;
	provider->get_bw = exynos_bts_icc_get_bw;
	provider->pre_aggregate = exynos_bts_pre_aggregate;
	provider->aggregate = exynos_bts_aggregate;
	provider->xlate = of_icc_xlate_onecell;
	provider->data = bts->data;
	icc_provider_init(provider);

	slave = icc_node_create(ZUMAPRO_BTS_SLAVE_MIF);
	if (IS_ERR(slave))
		return PTR_ERR(slave);
	slave->name = "mif";
	icc_node_add(slave, provider);
	bts->data->nodes[ZUMAPRO_BTS_SLAVE_MIF] = slave;

	for (i = 0; i < bts->num_masters; i++) {
		struct exynos_bts_master *master = &bts->masters[i];

		node = icc_node_create(master->data->id);
		if (IS_ERR(node)) {
			ret = PTR_ERR(node);
			goto err_nodes;
		}
		master->node = node;
		node->name = master->data->name;
		node->data = master;
		icc_node_add(node, provider);
		bts->data->nodes[master->data->id] = node;
		ret = icc_link_create(node, slave->id);
		if (ret)
			goto err_nodes;
	}
	ret = icc_provider_register(provider);
	if (ret)
		goto err_nodes;
	return devm_add_action_or_reset(bts->dev, exynos_bts_unregister, bts);
err_nodes:
	icc_nodes_remove(provider);
	return ret;
}

static int exynos_bts_resume(struct device *dev)
{
	struct exynos_bts *bts = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&bts->lock);
	/* Retry an ICC rollback which failed before the system suspended. */
	if (bts->rates_saved && !bts->active)
		ret = exynos_bts_set_idle(bts);

	/* The BTS register state is lost independently of ACPM clock errors. */
	exynos_bts_program_profiles(bts);
	if (!ret && bts->active) {
		ret = exynos_bts_set_rate(bts->mif_clk, bts->target_mif_rate);
		if (!ret)
			ret = exynos_bts_set_rate(bts->int_clk, bts->target_int_rate);
	}
	mutex_unlock(&bts->lock);

	return ret;
}

static DEFINE_SIMPLE_DEV_PM_OPS(exynos_bts_pm_ops, NULL, exynos_bts_resume);

static int exynos_bts_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos_bts *bts;
	unsigned int i;
	int ret;

	bts = devm_kzalloc(dev, sizeof(*bts), GFP_KERNEL);
	if (!bts)
		return -ENOMEM;

	bts->dev = dev;
	bts->num_blocks = ARRAY_SIZE(zumapro_blocks);
	bts->blocks = devm_kcalloc(dev, bts->num_blocks,
				   sizeof(*bts->blocks), GFP_KERNEL);
	if (!bts->blocks)
		return -ENOMEM;

	bts->num_masters = ARRAY_SIZE(zumapro_masters);
	bts->masters = devm_kcalloc(dev, bts->num_masters, sizeof(*bts->masters), GFP_KERNEL);
	if (!bts->masters)
		return -ENOMEM;
	for (i = 0; i < bts->num_masters; i++)
		bts->masters[i].data = &zumapro_masters[i];

	mutex_init(&bts->lock);
	platform_set_drvdata(pdev, bts);

	for (i = 0; i < bts->num_blocks; i++) {
		const char *name = zumapro_blocks[i].name;

		bts->blocks[i].data = &zumapro_blocks[i];
		bts->blocks[i].base =
			devm_platform_ioremap_resource_byname(pdev, name);
		if (IS_ERR(bts->blocks[i].base))
			return dev_err_probe(dev, PTR_ERR(bts->blocks[i].base),
					     "cannot map %s\n", name);
	}

	bts->mif_clk = devm_clk_get(dev, "mif");
	if (IS_ERR(bts->mif_clk))
		return dev_err_probe(dev, PTR_ERR(bts->mif_clk),
				     "cannot get MIF clock\n");
	bts->int_clk = devm_clk_get(dev, "int");
	if (IS_ERR(bts->int_clk))
		return dev_err_probe(dev, PTR_ERR(bts->int_clk),
				     "cannot get INT clock\n");

	ret = exynos_bts_register(bts);
	if (ret)
		return dev_err_probe(dev, ret, "cannot register interconnect\n");

	/* Downstream installs the default profile at BTS probe. */
	exynos_bts_program_profiles(bts);

	return 0;
}

static const struct of_device_id exynos_bts_of_match[] = {
	{ .compatible = "google,zumapro-bts" },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos_bts_of_match);

static struct platform_driver exynos_bts_driver = {
	.probe = exynos_bts_probe,
	.driver = {
		.name = "exynos-bts",
		.of_match_table = exynos_bts_of_match,
		.pm = pm_sleep_ptr(&exynos_bts_pm_ops),
		.sync_state = icc_sync_state,
	},
};
module_platform_driver(exynos_bts_driver);

MODULE_DESCRIPTION("Samsung Exynos Bus Traffic Shaper interconnect driver");
MODULE_LICENSE("GPL");
