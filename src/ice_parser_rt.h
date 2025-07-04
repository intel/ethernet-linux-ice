/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2018-2025 Intel Corporation */

#ifndef _ICE_PARSER_RT_H_
#define _ICE_PARSER_RT_H_

struct ice_parser_ctx;

#define ICE_PARSER_MAX_PKT_LEN 504
#define ICE_PARSER_GPR_NUM 128

struct ice_gpr_pu {
	bool gpr_val_upd[128]; /* flag to indicate if GRP needs to be updated */
	u16 gpr_val[128];
	u64 flg_msk;
	u64 flg_val;
	u16 err_msk;
	u16 err_val;
};

struct ice_parser_rt {
	struct ice_parser *psr;
	u16 gpr[ICE_PARSER_GPR_NUM];
	u8 pkt_buf[ICE_PARSER_MAX_PKT_LEN + 32];
	u16 pkt_len;
	u16 po;
	u8 bst_key[20];
	struct ice_pg_cam_key pg_key;
	struct ice_alu *alu0;
	struct ice_alu *alu1;
	struct ice_alu *alu2;
	struct ice_pg_cam_action *action;
	u8 pg;
	struct ice_gpr_pu pu;
	u8 markers[9]; /* 8 * 9 = 72 bits*/
	bool protocols[256];
	u16 offsets[256];
};

void ice_parser_rt_reset(struct ice_parser_rt *rt);
void ice_parser_rt_pktbuf_set(struct ice_parser_rt *rt, const u8 *pkt_buf,
			      int pkt_len);

struct ice_parser_result;
int ice_parser_rt_execute(struct ice_parser_rt *rt,
			  struct ice_parser_result *rslt);
#endif /* _ICE_PARSER_RT_H_ */
