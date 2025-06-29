/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2018-2025 Intel Corporation */

#ifndef _ICE_FLEX_PIPE_H_
#define _ICE_FLEX_PIPE_H_

#include "ice_type.h"

int
ice_find_prot_off(struct ice_hw *hw, enum ice_block blk, u8 prof, u16 fv_idx,
		  u8 *prot, u16 *off);
int
ice_find_label_value(struct ice_seg *ice_seg, char const *name, u32 type,
		     u16 *value);
void
ice_get_sw_fv_bitmap(struct ice_hw *hw, enum ice_prof_type type,
		     unsigned long *bm);
void
ice_init_prof_result_bm(struct ice_hw *hw);
int
ice_aq_upload_section(struct ice_hw *hw, struct ice_buf_hdr *pkg_buf,
		      u16 buf_size, struct ice_sq_cd *cd);
bool
ice_get_open_tunnel_port(struct ice_hw *hw, enum ice_tunnel_type type,
			 u16 *port);
int
ice_is_create_tunnel_possible(struct ice_hw *hw, enum ice_tunnel_type type,
			      u16 port);
bool ice_is_tunnel_empty(struct ice_hw *hw);
int
ice_create_tunnel(struct ice_hw *hw, enum ice_tunnel_type type, u16 port);
int ice_set_dvm_boost_entries(struct ice_hw *hw);
int ice_destroy_tunnel(struct ice_hw *hw, u16 port, bool all);
bool ice_tunnel_port_in_use(struct ice_hw *hw, u16 port, u16 *index);
bool
ice_tunnel_get_type(struct ice_hw *hw, u16 port, enum ice_tunnel_type *type);
int ice_replay_tunnels(struct ice_hw *hw);

/* RX parser PType functions */
bool ice_hw_ptype_ena(struct ice_hw *hw, u16 ptype);

/* XLT1/PType group functions */
int ice_ptg_update_xlt1(struct ice_hw *hw, enum ice_block blk);
void ice_ptg_free(struct ice_hw *hw, enum ice_block blk, u8 ptg);

/* XLT2/VSI group functions */
int ice_vsig_update_xlt2(struct ice_hw *hw, enum ice_block blk);
int
ice_vsig_find_vsi(struct ice_hw *hw, enum ice_block blk, u16 vsi, u16 *vsig);
int
ice_add_prof(struct ice_hw *hw, enum ice_block blk, u64 id,
	     unsigned long *ptypes, const struct ice_ptype_attributes *attr,
	     u16 attr_cnt, struct ice_fv_word *es, u16 *masks, bool fd_swap);
void ice_init_all_prof_masks(struct ice_hw *hw);
void ice_shutdown_all_prof_masks(struct ice_hw *hw);
struct ice_prof_map *
ice_search_prof_id(struct ice_hw *hw, enum ice_block blk, u64 id);
int
ice_add_vsi_flow(struct ice_hw *hw, enum ice_block blk, u16 vsi, u16 vsig);
int
ice_add_prof_id_flow(struct ice_hw *hw, enum ice_block blk, u16 vsi, u64 hdl);
int
ice_rem_prof_id_flow(struct ice_hw *hw, enum ice_block blk, u16 vsi, u64 hdl);
int
ice_flow_assoc_hw_prof(struct ice_hw *hw, enum ice_block blk,
		       u16 dest_vsi_handle, u16 fdir_vsi_handle, int id);
int
ice_set_prof_context(struct ice_hw *hw, enum ice_block blk, u64 id, u64 cntxt);
int
ice_get_prof_context(struct ice_hw *hw, enum ice_block blk, u64 id, u64 *cntxt);
int ice_init_hw_tbls(struct ice_hw *hw);
void ice_fill_blk_tbls(struct ice_hw *hw);
void ice_clear_hw_tbls(struct ice_hw *hw);
void ice_free_hw_tbls(struct ice_hw *hw);
int
ice_add_flow(struct ice_hw *hw, enum ice_block blk, u16 vsi[], u8 count,
	     u64 id);
int
ice_rem_flow(struct ice_hw *hw, enum ice_block blk, u16 vsi[], u8 count,
	     u64 id);
int
ice_rem_prof(struct ice_hw *hw, enum ice_block blk, u64 id);

int
ice_set_key(u8 *key, u16 size, u8 *val, u8 *upd, u8 *dc, u8 *nm, u16 off,
	    u16 len);

void ice_fill_blk_tbls(struct ice_hw *hw);

/* To support tunneling entries by PF, the package will append the PF number to
 * the label; for example TNL_VXLAN_PF0, TNL_VXLAN_PF1, TNL_VXLAN_PF2, etc.
 */
#define ICE_TNL_PRE	"TNL_"
/* For supporting double VLAN mode, it is necessary to enable or disable certain
 * boost tcam entries. The metadata labels names that match the following
 * prefixes will be saved to allow enabling double VLAN mode.
 */
#define ICE_DVM_PRE	"BOOST_MAC_VLAN_DVM"	/* enable these entries */
#define ICE_SVM_PRE	"BOOST_MAC_VLAN_SVM"	/* disable these entries */

void ice_add_tunnel_hint(struct ice_hw *hw, char *label_name, u16 val);
void ice_add_dvm_hint(struct ice_hw *hw, u16 val, bool enable);

#endif /* _ICE_FLEX_PIPE_H_ */
