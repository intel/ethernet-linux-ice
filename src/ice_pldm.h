/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2018-2019, Intel Corporation. */

#ifndef _ICE_PLDM_H_
#define _ICE_PLDM_H_

int ice_flash_pldm_image(struct ice_pf *pf, const struct firmware *fw,
			 struct netlink_ext_ack *extack);

#endif
