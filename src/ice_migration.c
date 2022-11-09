// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2018-2021, Intel Corporation. */

#include "ice.h"

/**
 * ice_migration_get_vf - Get ice vf structure pointer by pdev
 * @vf_pdev: pointer to ice vfio pci vf pdev structure
 *
 * Return nonzero for success, NULL for failure.
 */
void *ice_migration_get_vf(struct pci_dev *vf_pdev)
{
	struct pci_dev *pf_pdev = vf_pdev->physfn;
	int vf_id = pci_iov_vf_id(vf_pdev);
	struct ice_pf *pf;

	if (!pf_pdev || vf_id < 0)
		return NULL;

	pf = pci_get_drvdata(pf_pdev);
	return ice_get_vf_by_id(pf, vf_id);
}
EXPORT_SYMBOL(ice_migration_get_vf);

