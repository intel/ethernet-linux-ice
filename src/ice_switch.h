/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2018-2024 Intel Corporation */

#ifndef _ICE_SWITCH_H_
#define _ICE_SWITCH_H_

#include "ice_type.h"
#include "ice_protocol_type.h"

#define ICE_SW_CFG_MAX_BUF_LEN 2048
#define ICE_MAX_SW 256
#define ICE_DFLT_VSI_INVAL 0xff

#define ICE_FLTR_RX	BIT(0)
#define ICE_FLTR_TX	BIT(1)
#define ICE_FLTR_RX_LB	BIT(2)
#define ICE_FLTR_TX_RX	(ICE_FLTR_RX | ICE_FLTR_TX)

/* Switch Profile IDs for Profile related switch rules */
#define ICE_PROFID_IPV4_TCP		4
#define ICE_PROFID_IPV4_UDP		5
#define ICE_PROFID_IPV6_TCP		7
#define ICE_PROFID_IPV6_UDP		8
#define ICE_PROFID_IPV4_GTPC_TEID	41
#define ICE_PROFID_IPV4_GTPC_NO_TEID		42
#define ICE_PROFID_IPV4_GTPU_TEID		43
#define ICE_PROFID_IPV6_GTPC_TEID		44
#define ICE_PROFID_IPV6_GTPC_NO_TEID		45
#define ICE_PROFID_IPV6_GTPU_TEID		46
#define ICE_PROFID_IPV6_GTPU_IPV6_TCP		70

#define DUMMY_ETH_HDR_LEN		16

/* Worst case buffer length for ice_aqc_opc_get_res_alloc */
#define ICE_MAX_RES_TYPES 0x80
#define ICE_AQ_GET_RES_ALLOC_BUF_LEN \
	(ICE_MAX_RES_TYPES * sizeof(struct ice_aqc_get_res_resp_elem))

#define ICE_VSI_INVAL_ID 0xFFFF
#define ICE_INVAL_Q_HANDLE 0xFFFF

/* VSI context structure for add/get/update/free operations */
struct ice_vsi_ctx {
	u16 vsi_num;
	u16 vsis_allocd;
	u16 vsis_unallocated;
	u16 flags;
	struct ice_aqc_vsi_props info;
	struct ice_sched_vsi_info sched;
	u8 alloc_from_pool;
	u8 vf_num;
	u16 num_lan_q_entries[ICE_MAX_TRAFFIC_CLASS];
	struct ice_q_ctx *lan_q_ctx[ICE_MAX_TRAFFIC_CLASS];
	u16 num_rdma_q_entries[ICE_MAX_TRAFFIC_CLASS];
	struct ice_q_ctx *rdma_q_ctx[ICE_MAX_TRAFFIC_CLASS];
};

/* This is to be used by add/update mirror rule Admin Queue command */
struct ice_mir_rule_buf {
	u16 vsi_idx; /* VSI index */

	/* For each VSI, user can specify whether corresponding VSI
	 * should be added/removed to/from mirror rule
	 *
	 * add mirror rule: this should always be TRUE.
	 * update mirror rule:  add(true) or remove(false) VSI to/from
	 * mirror rule
	 */
	u8 add;
};

/* Switch recipe ID enum values are specific to hardware */
enum ice_sw_lkup_type {
	ICE_SW_LKUP_ETHERTYPE = 0,
	ICE_SW_LKUP_MAC = 1,
	ICE_SW_LKUP_MAC_VLAN = 2,
	ICE_SW_LKUP_PROMISC = 3,
	ICE_SW_LKUP_VLAN = 4,
	ICE_SW_LKUP_DFLT = 5,
	ICE_SW_LKUP_ETHERTYPE_MAC = 8,
	ICE_SW_LKUP_PROMISC_VLAN = 9,
	ICE_SW_LKUP_LAST
};

/* type of filter src ID */
enum ice_src_id {
	ICE_SRC_ID_UNKNOWN = 0,
	ICE_SRC_ID_VSI,
	ICE_SRC_ID_QUEUE,
	ICE_SRC_ID_LPORT,
};

struct ice_fltr_info {
	/* Look up information: how to look up packet */
	enum ice_sw_lkup_type lkup_type;
	/* Forward action: filter action to do after lookup */
	enum ice_sw_fwd_act_type fltr_act;
	/* rule ID returned by firmware once filter rule is created */
	u16 fltr_rule_id;
	u16 flag;

	/* Source VSI for LOOKUP_TX or source port for LOOKUP_RX */
	u16 src;
	enum ice_src_id src_id;

	union {
		struct {
			u8 mac_addr[ETH_ALEN];
		} mac;
		struct {
			u8 mac_addr[ETH_ALEN];
			u16 vlan_id;
		} mac_vlan;
		struct {
			u16 vlan_id;
			u16 tpid;
			u8 tpid_valid;
		} vlan;
		/* Set lkup_type as ICE_SW_LKUP_ETHERTYPE
		 * if just using ethertype as filter. Set lkup_type as
		 * ICE_SW_LKUP_ETHERTYPE_MAC if MAC also needs to be
		 * passed in as filter.
		 */
		struct {
			u16 ethertype;
			u8 mac_addr[ETH_ALEN]; /* optional */
		} ethertype_mac;
	} l_data; /* Make sure to zero out the memory of l_data before using
		   * it or only set the data associated with lookup match
		   * rest everything should be zero
		   */

	/* Depending on filter action */
	union {
		/* queue ID in case of ICE_FWD_TO_Q and starting
		 * queue ID in case of ICE_FWD_TO_QGRP.
		 */
		u16 q_id:11;
		u16 hw_vsi_id:10;
		u16 vsi_list_id:10;
	} fwd_id;

	/* Sw VSI handle */
	u16 vsi_handle;

	/* Set to num_queues if action is ICE_FWD_TO_QGRP. This field
	 * determines the range of queues the packet needs to be forwarded to.
	 * Note that qgrp_size must be set to a power of 2.
	 */
	u8 qgrp_size;

	/* Rule creations populate these indicators basing on the switch type */
	u8 lb_en;	/* Indicate if packet can be looped back */
	u8 lan_en;	/* Indicate if packet can be forwarded to the uplink */
};

struct ice_update_recipe_lkup_idx_params {
	u16 rid;
	u16 fv_idx;
	bool ignore_valid;
	u16 mask;
	bool mask_valid;
	u8 lkup_idx;
};

struct ice_adv_lkup_elem {
	enum ice_protocol_type type;
	union {
		union ice_prot_hdr h_u;	/* Header values */
		/* Used to iterate over the headers */
		u16 h_raw[sizeof(union ice_prot_hdr) / sizeof(u16)];
	};
	union {
		union ice_prot_hdr m_u;	/* Mask of header values to match */
		/* Used to iterate over header mask */
		u16 m_raw[sizeof(union ice_prot_hdr) / sizeof(u16)];
	};
};

struct entry_vsi_fwd {
	u16 vsi_list;
	u8 list;
	u8 valid;
};

struct entry_to_q {
	u16 q_idx;
	u8 q_region_sz;
	u8 q_pri;
};

struct entry_prune {
	u16 vsi_list;
	u8 list;
	u8 egr;
	u8 ing;
	u8 prune_t;
};

struct entry_mirror {
	u16 mirror_vsi;
};

struct entry_generic_act {
	u16 generic_value;
	u8 offset;
	u8 priority;
};

struct entry_statistics {
	u8 counter_idx;
};

struct ice_sw_act_ctrl {
	/* Source VSI for LOOKUP_TX or source port for LOOKUP_RX */
	u16 src;
	u16 flag;
	enum ice_sw_fwd_act_type fltr_act;
	/* Depending on filter action */
	union {
		/* This is a queue ID in case of ICE_FWD_TO_Q and starting
		 * queue ID in case of ICE_FWD_TO_QGRP.
		 */
		u16 q_id:11;
		u16 vsi_id:10;
		u16 hw_vsi_id:10;
		u16 vsi_list_id:10;
	} fwd_id;
	/* software VSI handle */
	u16 vsi_handle;
	u8 qgrp_size;
};

struct ice_rule_query_data {
	/* Recipe ID for which the requested rule was added */
	u16 rid;
	/* Rule ID that was added or is supposed to be removed */
	u16 rule_id;
	/* vsi_handle for which Rule was added or is supposed to be removed */
	u16 vsi_handle;
};

/*
 * This structure allows to pass info about lb_en and lan_en
 * flags to ice_add_adv_rule. Values in act would be used
 * only if act_valid was set to true, otherwise dflt
 * values would be used.
 */
struct ice_adv_rule_flags_info {
	u32 act;
	u8 act_valid;		/* indicate if flags in act are valid */
};

struct ice_adv_rule_info {
	/* Store metadata values in rule info */
	enum ice_sw_tunnel_type tun_type;
	u16 vlan_type;
	u16 fltr_rule_id;
	u32 priority;
	u16 src_vsi;
	struct ice_sw_act_ctrl sw_act;
	u8 add_dir_lkup;
	u16 lg_id;
	struct ice_adv_rule_flags_info flags_info;
};

/* A collection of one or more four word recipe */
struct ice_sw_recipe {
	/* For a chained recipe the root recipe is what should be used for
	 * programming rules
	 */
	u8 is_root;
	u8 root_rid;
	u8 recp_created;

	/* Number of extraction words */
	u8 n_ext_words;
	/* Protocol ID and Offset pair (extraction word) to describe the
	 * recipe
	 */
	struct ice_fv_word ext_words[ICE_MAX_CHAIN_WORDS];
	u16 word_masks[ICE_MAX_CHAIN_WORDS];

	/* if this recipe is a collection of other recipe */
	u8 big_recp;

	/* if this recipe is part of another bigger recipe then chain index
	 * corresponding to this recipe
	 */
	u8 chain_idx;

	/* if this recipe is a collection of other recipe then count of other
	 * recipes and recipe IDs of those recipes
	 */
	u8 n_grp_count;

	/* Bit map specifying the IDs associated with this group of recipe */
	DECLARE_BITMAP(r_bitmap, ICE_MAX_NUM_RECIPES);

	enum ice_sw_tunnel_type tun_type;

	/* List of type ice_fltr_mgmt_list_entry or adv_rule */
	u8 adv_rule;
	struct list_head filt_rules;
	struct list_head filt_replay_rules;

	struct mutex filt_rule_lock;	/* protect filter rule structure */

	/* Profiles this recipe should be associated with */
	struct list_head fv_list;

	/* Profiles this recipe is associated with */
	u8 num_profs, *prof_ids;

	/* Bit map for possible result indexes */
	DECLARE_BITMAP(res_idxs, ICE_MAX_FV_WORDS);

	/* This allows user to specify the recipe priority.
	 * For now, this becomes 'fwd_priority' when recipe
	 * is created, usually recipes can have 'fwd' and 'join'
	 * priority.
	 */
	u8 priority;

	struct list_head rg_list;

	/* AQ buffer associated with this recipe */
	struct ice_aqc_recipe_data_elem *root_buf;
	/* This struct saves the fv_words for a given lookup */
	struct ice_prot_lkup_ext lkup_exts;
};

/* Bookkeeping structure to hold bitmap of VSIs corresponding to VSI list ID */
struct ice_vsi_list_map_info {
	struct list_head list_entry;
	DECLARE_BITMAP(vsi_map, ICE_MAX_VSI);
	u16 vsi_list_id;
	/* counter to track how many rules are reusing this VSI list */
	u16 ref_cnt;
};

struct ice_fltr_list_entry {
	struct list_head list_entry;
	int status;
	struct ice_fltr_info fltr_info;
};

/* This defines an entry in the list that maintains MAC or VLAN membership
 * to HW list mapping, since multiple VSIs can subscribe to the same MAC or
 * VLAN. As an optimization the VSI list should be created only when a
 * second VSI becomes a subscriber to the same MAC address. VSI lists are always
 * used for VLAN membership.
 */
struct ice_fltr_mgmt_list_entry {
	/* back pointer to VSI list ID to VSI list mapping */
	struct ice_vsi_list_map_info *vsi_list_info;
	u16 vsi_count;
#define ICE_INVAL_LG_ACT_INDEX 0xffff
	u16 lg_act_idx;
#define ICE_INVAL_SW_MARKER_ID 0xffff
	u16 sw_marker_id;
	struct list_head list_entry;
	struct ice_fltr_info fltr_info;
#define ICE_INVAL_COUNTER_ID 0xff
	u8 counter_index;
};

struct ice_adv_fltr_mgmt_list_entry {
	struct list_head list_entry;

	struct ice_adv_lkup_elem *lkups;
	struct ice_adv_rule_info rule_info;
	u16 lkups_cnt;
	struct ice_vsi_list_map_info *vsi_list_info;
	u16 vsi_count;
};

enum ice_promisc_flags {
	ICE_PROMISC_UCAST_RX = 0,
	ICE_PROMISC_UCAST_TX,
	ICE_PROMISC_MCAST_RX,
	ICE_PROMISC_MCAST_TX,
	ICE_PROMISC_BCAST_RX,
	ICE_PROMISC_BCAST_TX,
	ICE_PROMISC_VLAN_RX,
	ICE_PROMISC_VLAN_TX,
	ICE_PROMISC_UCAST_RX_LB,
	/* Max value */
	ICE_PROMISC_MAX,
};

struct ice_dummy_pkt_offsets {
	enum ice_protocol_type type;
	u16 offset; /* ICE_PROTOCOL_LAST indicates end of list */
};

struct ice_dummy_pkt_profile {
	const struct ice_dummy_pkt_offsets *offsets;
	const u8 *pkt;
	u32 match;
	u16 pkt_len;
};

const struct ice_dummy_pkt_profile *
ice_find_dummy_packet(struct ice_adv_lkup_elem *lkups, u16 lkups_cnt,
		      enum ice_sw_tunnel_type tun_type);

int
ice_fill_adv_dummy_packet(struct ice_adv_lkup_elem *lkups, u16 lkups_cnt,
			  struct ice_sw_rule_lkup_rx_tx *s_rule,
			  const struct ice_dummy_pkt_profile *profile);

int
ice_add_adv_recipe(struct ice_hw *hw, struct ice_adv_lkup_elem *lkups,
		   u16 lkups_cnt, struct ice_adv_rule_info *rinfo, u16 *rid);

struct ice_adv_fltr_mgmt_list_entry *
ice_find_adv_rule_entry(struct ice_hw *hw, struct ice_adv_lkup_elem *lkups,
			u16 lkups_cnt, u16 recp_id,
			struct ice_adv_rule_info *rinfo);

int
ice_adv_add_update_vsi_list(struct ice_hw *hw,
			    struct ice_adv_fltr_mgmt_list_entry *m_entry,
			    struct ice_adv_rule_info *cur_fltr,
			    struct ice_adv_rule_info *new_fltr);

struct ice_vsi_list_map_info *
ice_find_vsi_list_entry(struct ice_sw_recipe *recp_list, u16 vsi_handle,
			u16 *vsi_list_id);

/* VSI related commands */
int
ice_add_vsi(struct ice_hw *hw, u16 vsi_handle, struct ice_vsi_ctx *vsi_ctx,
	    struct ice_sq_cd *cd);
int
ice_free_vsi(struct ice_hw *hw, u16 vsi_handle, struct ice_vsi_ctx *vsi_ctx,
	     bool keep_vsi_alloc, struct ice_sq_cd *cd);
int
ice_update_vsi(struct ice_hw *hw, u16 vsi_handle, struct ice_vsi_ctx *vsi_ctx,
	       struct ice_sq_cd *cd);
struct ice_vsi_ctx *ice_get_vsi_ctx(struct ice_hw *hw, u16 vsi_handle);
void ice_clear_all_vsi_ctx(struct ice_hw *hw);
int
ice_aq_get_vsi_params(struct ice_hw *hw, struct ice_vsi_ctx *vsi_ctx,
		      struct ice_sq_cd *cd);
int
ice_aq_add_update_mir_rule(struct ice_hw *hw, u16 rule_type, u16 dest_vsi,
			   u16 count, struct ice_mir_rule_buf *mr_buf,
			   struct ice_sq_cd *cd, u16 *rule_id);
int
ice_aq_delete_mir_rule(struct ice_hw *hw, u16 rule_id, bool keep_allocd,
		       struct ice_sq_cd *cd);
int
ice_aq_get_storm_ctrl(struct ice_hw *hw, u32 *bcast_thresh, u32 *mcast_thresh,
		      u32 *ctl_bitmask);
int
ice_aq_set_storm_ctrl(struct ice_hw *hw, u32 bcast_thresh, u32 mcast_thresh,
		      u32 ctl_bitmask);
/* Switch config */
int
ice_aq_get_sw_cfg(struct ice_hw *hw, struct ice_aqc_get_sw_cfg_resp_elem *buf,
		  u16 buf_size, u16 *req_desc, u16 *num_elems,
		  struct ice_sq_cd *cd);
int ice_get_initial_sw_cfg(struct ice_hw *hw);

int
ice_alloc_vlan_res_counter(struct ice_hw *hw, u16 *counter_id);
int
ice_free_vlan_res_counter(struct ice_hw *hw, u16 counter_id);
int
ice_alloc_res_cntr(struct ice_hw *hw, u8 type, u8 alloc_shared, u16 num_items,
		   u16 *counter_id);
int
ice_free_res_cntr(struct ice_hw *hw, u8 type, u8 alloc_shared, u16 num_items,
		  u16 counter_id);

int ice_update_sw_rule_bridge_mode(struct ice_hw *hw);
int ice_alloc_rss_global_lut(struct ice_hw *hw, bool shared_res, u16 *global_lut_id);
int ice_free_rss_global_lut(struct ice_hw *hw, u16 global_lut_id);
int
ice_alloc_sw(struct ice_hw *hw, bool ena_stats, bool shared_res, u16 *sw_id,
	     u16 *counter_id);
int
ice_free_sw(struct ice_hw *hw, u16 sw_id, u16 counter_id);
int
ice_aq_get_res_alloc(struct ice_hw *hw, u16 *num_entries,
		     struct ice_aqc_get_res_resp_elem *buf, u16 buf_size,
		     struct ice_sq_cd *cd);
int
ice_aq_get_res_descs(struct ice_hw *hw, u16 num_entries,
		     struct ice_aqc_res_elem *buf, u16 buf_size, u16 res_type,
		     bool res_shared, u16 *desc_id, struct ice_sq_cd *cd);
int
ice_add_vlan(struct ice_hw *hw, struct list_head *m_list);
int ice_remove_vlan(struct ice_hw *hw, struct list_head *v_list);
void ice_rem_all_sw_rules_info(struct ice_hw *hw);
int ice_add_mac(struct ice_hw *hw, struct list_head *m_lst);
int ice_remove_mac(struct ice_hw *hw, struct list_head *m_lst);
bool ice_mac_fltr_exist(struct ice_hw *hw, u8 *mac, u16 vsi_handle);
bool ice_vlan_fltr_exist(struct ice_hw *hw, u16 vlan_id, u16 vsi_handle);
int
ice_add_eth_mac(struct ice_hw *hw, struct list_head *em_list);
int
ice_remove_eth_mac(struct ice_hw *hw, struct list_head *em_list);
void ice_dump_sw_rules(struct ice_hw *hw, enum ice_sw_lkup_type lookup);
int
ice_cfg_iwarp_fltr(struct ice_hw *hw, u16 vsi_handle, bool enable);
int
ice_add_mac_vlan(struct ice_hw *hw, struct list_head *m_list);
int
ice_remove_mac_vlan(struct ice_hw *hw, struct list_head *v_list);

int
ice_add_mac_with_sw_marker(struct ice_hw *hw, struct ice_fltr_info *f_info,
			   u16 sw_marker);
int
ice_add_mac_with_counter(struct ice_hw *hw, struct ice_fltr_info *f_info);
void ice_remove_vsi_fltr(struct ice_hw *hw, u16 vsi_handle);

/* Promisc/defport setup for VSIs */
int
ice_cfg_dflt_vsi(struct ice_port_info *pi, u16 vsi_handle, bool set,
		 u8 direction);
bool ice_check_if_dflt_vsi(struct ice_port_info *pi, u16 vsi_handle,
			   bool *rule_exists);
int
ice_set_vsi_promisc(struct ice_hw *hw, u16 vsi_handle,
		    unsigned long *promisc_mask, u16 vid);
int
ice_clear_vsi_promisc(struct ice_hw *hw, u16 vsi_handle,
		      unsigned long *promisc_mask, u16 vid);
int
ice_set_vlan_vsi_promisc(struct ice_hw *hw, u16 vsi_handle,
			 unsigned long *promisc_mask, bool rm_vlan_promisc);

/* Get VSIs Promisc/defport settings */
int
ice_get_vsi_promisc(struct ice_hw *hw, u16 vsi_handle,
		    unsigned long *promisc_mask, u16 *vid);
int
ice_get_vsi_vlan_promisc(struct ice_hw *hw, u16 vsi_handle,
			 unsigned long *promisc_mask, u16 *vid);

int
ice_aq_add_recipe(struct ice_hw *hw,
		  struct ice_aqc_recipe_data_elem *s_recipe_list,
		  u16 num_recipes, struct ice_sq_cd *cd);

int
ice_aq_get_recipe(struct ice_hw *hw,
		  struct ice_aqc_recipe_data_elem *s_recipe_list,
		  u16 *num_recipes, u16 recipe_root, struct ice_sq_cd *cd);
int
ice_aq_map_recipe_to_profile(struct ice_hw *hw, u32 profile_id, u8 *r_bitmap,
			     struct ice_sq_cd *cd);

int
ice_aq_get_recipe_to_profile(struct ice_hw *hw, u32 profile_id, u8 *r_bitmap,
			     struct ice_sq_cd *cd);

void ice_init_chk_subscribable_recipe_support(struct ice_hw *hw);

int ice_alloc_recipe(struct ice_hw *hw, u16 *recipe_id);
void ice_rule_add_tunnel_metadata(struct ice_adv_lkup_elem *lkup,
				  struct ice_adv_rule_info *rule_info,
				  enum ice_sw_tunnel_type tun_type);
void ice_rule_add_direction_metadata(struct ice_adv_lkup_elem *lkup);
#ifdef HAVE_TCF_VLAN_TPID
void ice_rule_add_vlan_metadata(struct ice_adv_lkup_elem *lkup);
#endif /* HAVE_TCF_VLAN_TPID */
void ice_rule_add_src_vsi_metadata(struct ice_adv_lkup_elem *lkup);
int
ice_add_adv_rule(struct ice_hw *hw, struct ice_adv_lkup_elem *lkups,
		 u16 lkups_cnt, struct ice_adv_rule_info *rinfo,
		 struct ice_rule_query_data *added_entry);
int
ice_rem_adv_rule_for_vsi(struct ice_hw *hw, u16 vsi_handle);
int
ice_rem_adv_rule_by_id(struct ice_hw *hw,
		       const struct ice_rule_query_data *remove_entry);
int
ice_rem_adv_rule(struct ice_hw *hw, struct ice_adv_lkup_elem *lkups,
		 u16 lkups_cnt, struct ice_adv_rule_info *rinfo);

int ice_dump_sw_cfg(struct ice_hw *hw);

int
ice_init_def_sw_recp(struct ice_hw *hw, struct ice_sw_recipe **recp_list);
u16 ice_get_hw_vsi_num(struct ice_hw *hw, u16 vsi_handle);
bool ice_is_vsi_valid(struct ice_hw *hw, u16 vsi_handle);

int
ice_replay_vsi_all_fltr(struct ice_hw *hw, struct ice_port_info *pi,
			u16 vsi_handle);
void ice_rm_sw_replay_rule_info(struct ice_hw *hw, struct ice_switch_info *sw);
void ice_rm_all_sw_replay_rule_info(struct ice_hw *hw);
bool ice_is_prof_rule(enum ice_sw_tunnel_type type);
int
ice_aq_sw_rules(struct ice_hw *hw, void *rule_list, u16 rule_list_sz,
		u8 num_rules, enum ice_adminq_opc opc, struct ice_sq_cd *cd);
int
ice_update_recipe_lkup_idx(struct ice_hw *hw,
			   struct ice_update_recipe_lkup_idx_params *params);
void ice_change_proto_id_to_dvm(void);
#endif /* _ICE_SWITCH_H_ */
