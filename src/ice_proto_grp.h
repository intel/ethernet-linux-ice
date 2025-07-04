/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2018-2025 Intel Corporation */

#ifndef _ICE_PROTO_GRP_H_
#define _ICE_PROTO_GRP_H_

#define ICE_PROTO_COUNT_PER_GRP 8
#define ICE_PROTO_GRP_TABLE_SIZE 192

struct ice_proto_off {
	bool polarity; /* true: positive, false: nagtive */
	u8 proto_id;
	u16 offset;
};

struct ice_proto_grp_item {
	u16 idx;
	struct ice_proto_off po[ICE_PROTO_COUNT_PER_GRP];
};

void ice_proto_grp_dump(struct ice_hw *hw, struct ice_proto_grp_item *item);
struct ice_proto_grp_item *ice_proto_grp_table_get(struct ice_hw *hw);
#endif /* _ICE_PROTO_GRP_H_ */
