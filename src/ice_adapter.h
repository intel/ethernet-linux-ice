/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2018-2025 Intel Corporation */

#ifndef _ICE_ADAPTER_H_
#define _ICE_ADAPTER_H_

#include <linux/types.h>
#include <linux/spinlock_types.h>
#include "kcompat.h"

struct pci_dev;
struct ice_pf;

/**
 * struct ice_port_list - data used to store the list of adapter ports
 *
 * This structure contains data used to maintain a list of adapter ports
 *
 * @ports: list of ports
 * @lock: protect access to the ports list
 */
struct ice_port_list {
	struct list_head ports;
	/* To synchronize the ports list operations */
	struct mutex lock;
};

/**
 * struct ice_adapter - PCI adapter resources shared across PFs
 * @ptp_gltsyn_time_lock: Spinlock protecting access to the GLTSYN_TIME
 *                        register of the PTP clock.
 * @refcount: Reference count. struct ice_pf objects hold the references.
 * @ctrl_pf: Control PF of the adapter
 * @ports: Ports list
 */
struct ice_adapter {
	refcount_t refcount;
	/* For access to the GLTSYN_TIME register */
	spinlock_t ptp_gltsyn_time_lock;

	struct ice_pf *ctrl_pf;
	struct ice_port_list ports;
};

struct ice_adapter *ice_adapter_get(const struct pci_dev *pdev);
void ice_adapter_put(const struct pci_dev *pdev);

#endif /* _ICE_ADAPTER_H */
