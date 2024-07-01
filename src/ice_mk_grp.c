/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2018-2024 Intel Corporation */

#include "ice_common.h"
#include "ice_parser_util.h"

#define ICE_MK_GRP_TABLE_SIZE 128
#define ICE_MK_COUNT_PER_GRP 8

/**
 * ice_mk_grp_dump - dump an marker group item info
 * @hw: pointer to the hardware structure
 * @item: marker group item to dump
 */
void ice_mk_grp_dump(struct ice_hw *hw, struct ice_mk_grp_item *item)
{
	int i;

	dev_info(ice_hw_to_dev(hw), "index = %d\n", item->idx);
	dev_info(ice_hw_to_dev(hw), "markers: ");
	for (i = 0; i < ICE_MK_COUNT_PER_GRP; i++)
		dev_info(ice_hw_to_dev(hw), "%d ", item->markers[i]);
	dev_info(ice_hw_to_dev(hw), "\n");
}

static void _mk_grp_parse_item(struct ice_hw *hw, u16 idx, void *item,
			       void *data, int size)
{
	struct ice_mk_grp_item *grp = item;
	u8 *buf = data;
	int i;

	grp->idx = idx;

	for (i = 0; i < ICE_MK_COUNT_PER_GRP; i++)
		grp->markers[i] = buf[i];

	if (hw->debug_mask & ICE_DBG_PARSER)
		ice_mk_grp_dump(hw, grp);
}

/**
 * ice_mk_grp_table_get - create a marker group table
 * @hw: pointer to the hardware structure
 */
struct ice_mk_grp_item *ice_mk_grp_table_get(struct ice_hw *hw)
{
	return (struct ice_mk_grp_item *)
		ice_parser_create_table(hw, ICE_SID_RXPARSER_MARKER_GRP,
					sizeof(struct ice_mk_grp_item),
					ICE_MK_GRP_TABLE_SIZE,
					ice_parser_sect_item_get,
					_mk_grp_parse_item, false);
}
