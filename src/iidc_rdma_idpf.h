/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2018-2026 Intel Corporation */

#ifndef _IIDC_RDMA_IDPF_H_
#define _IIDC_RDMA_IDPF_H_

#include "iidc.h"

/* struct to be populated by core LAN PCI driver */
struct iidc_rdma_vport_dev_info {
	struct auxiliary_device *adev;
	struct auxiliary_device *core_adev;
	struct net_device *netdev;
	u16 vport_id;
};

struct iidc_rdma_vport_auxiliary_dev {
	struct auxiliary_device adev;
	struct iidc_rdma_vport_dev_info *vdev_info;
};

struct iidc_rdma_vport_auxiliary_drv {
	struct auxiliary_driver adrv;
	void (*event_handler)(struct iidc_rdma_vport_dev_info *vdev,
			      struct iidc_event *event);
};

struct iidc_rdma_lan_mapped_mem_region {
	u8 __iomem *region_addr;
	__le64 size;
	__le64 start_offset;
};

struct iidc_rdma_priv_dev_info {
	struct msix_entry *msix_entries;
	u16 msix_count; /* How many vectors are reserved for this device */
	enum iidc_function_type ftype;
	__le16 num_memory_regions;
	struct iidc_rdma_lan_mapped_mem_region *mapped_mem_regions;
};

int idpf_idc_vport_dev_ctrl(struct iidc_core_dev_info *cdev_info, bool up);
int idpf_idc_request_reset(struct iidc_core_dev_info *cdev_info,
			   enum iidc_reset_type __always_unused reset_type);
int idpf_idc_rdma_vc_send_sync(struct iidc_core_dev_info *cdev_info,
			       u8 *send_msg, u16 msg_size,
			       u8 *recv_msg, u16 *recv_len);
#endif /* _IIDC_RDMA_IDPF_H_ */
