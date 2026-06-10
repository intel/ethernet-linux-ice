/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2018-2026 Intel Corporation */

#ifndef _ICE_ADAPTER_H_
#define _ICE_ADAPTER_H_

#include <linux/types.h>
#include <linux/spinlock_types.h>
#ifdef HAVE_XARRAY_API
#include <linux/xarray.h>
#endif /* HAVE_XARRAY_API */
#include "kcompat.h"

struct pci_dev;
struct ice_pf;

/**
 * struct ice_adapter - PCI adapter resources shared across PFs
 * @ptp_gltsyn_time_lock: Spinlock protecting access to the GLTSYN_TIME
 *                        register of the PTP clock.
 * @refcount: Reference count. struct ice_pf objects hold the references.
 * @ctrl_pf: Control PF of the adapter
 * @ptp_ports: PTP ports array
 * @whole_dev_lock: the lock that should be used to serialize FW/HW
 *                  re/configuration, or any other operation that requires
 *                  to be serialized over whole device (all PFs on given card)
 */
struct ice_adapter {
	refcount_t refcount;
	/* For access to the GLTSYN_TIME register */
	spinlock_t ptp_gltsyn_time_lock;

	struct ice_pf *ctrl_pf;
	struct xarray ptp_ports;

	struct mutex whole_dev_lock;
};

struct ice_adapter *ice_adapter_get(const struct pci_dev *pdev);
void ice_adapter_put(const struct pci_dev *pdev);

#endif /* _ICE_ADAPTER_H */
