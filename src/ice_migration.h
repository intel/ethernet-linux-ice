/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2018-2021, Intel Corporation. */

#ifndef _ICE_MIGRATION_H_
#define _ICE_MIGRATION_H_

#include "kcompat.h"

#if IS_ENABLED(CONFIG_VFIO_PCI_CORE) && defined(HAVE_LMV1_SUPPORT)
void *ice_migration_get_vf(struct pci_dev *vf_pdev);
#else
static inline void *ice_migration_get_vf(struct pci_dev *vf_pdev)
{
	return NULL;
}
#endif /* CONFIG_VFIO_PCI_CORE && HAVE_LMV1_SUPPORT */

#endif /* _ICE_MIGRATION_H_ */
