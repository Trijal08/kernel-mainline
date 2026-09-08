/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef S5P_MFC_QOS_H_
#define S5P_MFC_QOS_H_

#include <linux/types.h>

#define S5P_MFC_QOS_SAMPLES 32

/* Queue-side fields are serialized by mfc_mutex; feature flags by irqlock. */
struct s5p_mfc_qos {
	u64 timestamps[S5P_MFC_QOS_SAMPLES];
	u32 bytes[S5P_MFC_QOS_SAMPLES];
	unsigned int count;
	unsigned int next;
	bool active;
	bool tiled;
	bool mbaff;
};

struct s5p_mfc_ctx;
struct s5p_mfc_dev;
struct vb2_buffer;
struct vb2_queue;

int s5p_mfc_qos_queue(struct vb2_buffer *vb);
void s5p_mfc_qos_stop(struct vb2_queue *q);
void s5p_mfc_qos_release(struct s5p_mfc_ctx *ctx);
int s5p_mfc_qos_restore(struct s5p_mfc_dev *dev);
void s5p_mfc_qos_cleanup(struct s5p_mfc_dev *dev);

#endif
