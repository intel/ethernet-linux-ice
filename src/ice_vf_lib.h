/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2018-2024 Intel Corporation */

#ifndef _ICE_VF_LIB_H_
#define _ICE_VF_LIB_H_

#include <linux/types.h>
#include <linux/hashtable.h>
#include <linux/bitmap.h>
#include <linux/mutex.h>
#include <linux/kref.h>
#include <linux/pci.h>
#include <linux/if_ether.h>
#if IS_ENABLED(CONFIG_NET_DEVLINK)
#include <net/devlink.h>
#endif /* CONFIG_NET_DEVLINK */
#include "virtchnl.h"
#include "ice_type.h"
#include "ice_flow.h"
#include "ice_virtchnl_fdir.h"
#include "ice_virtchnl_fsub.h"
#include "ice_dcf.h"
#include "ice_vsi_vlan_ops.h"

/* VF resource constraints */
#define ICE_MAX_QS_PER_VF	256
/* Maximum number of queue pairs to configure by default for a VF */
#define ICE_MAX_DFLT_QS_PER_VF         16

#define ICE_VFR_WAIT_COUNT 100

struct ice_pf;
struct ice_vf;
struct ice_virtchnl_ops;

/* VF capabilities */
enum ice_vf_cap {
	ICE_VF_CAP_TRUSTED = 0,
	ICE_VF_CAP_NBITS
};

/* Specific VF states */
enum ice_vf_states {
	ICE_VF_STATE_INIT = 0,		/* PF is initializing VF */
	ICE_VF_STATE_ACTIVE,		/* VF resources are allocated for use */
	ICE_VF_STATE_QS_ENA,		/* VF queue(s) enabled */
	ICE_VF_STATE_DIS,
	ICE_VF_STATE_MC_PROMISC,
	ICE_VF_STATE_UC_PROMISC,
	ICE_VF_STATE_REPLAY_VC,
	ICE_VF_STATE_IN_RESET,		/* VF reset in progress*/
	ICE_VF_STATES_NBITS
};

struct ice_time_mac {
	unsigned long time_modified;
	u8 addr[ETH_ALEN];
};

/* VF MDD events print structure */
struct ice_mdd_vf_events {
	u16 count;			/* total count of Rx|Tx events */
	/* count number of the last printed event */
	u16 last_printed;
};

#define ICE_HASH_IP_CTX_IP		0
#define ICE_HASH_IP_CTX_IP_ESP		1
#define ICE_HASH_IP_CTX_IP_UDP_ESP	2
#define ICE_HASH_IP_CTX_IP_AH		3
#define ICE_HASH_IP_CTX_IP_L2TPV3	4
#define ICE_HASH_IP_CTX_IP_PFCP		5
#define ICE_HASH_IP_CTX_IP_UDP		6
#define ICE_HASH_IP_CTX_IP_TCP		7
#define ICE_HASH_IP_CTX_IP_SCTP		8
#define ICE_HASH_IP_CTX_MAX		9

struct ice_vf_hash_ip_ctx {
	struct ice_rss_hash_cfg ctx[ICE_HASH_IP_CTX_MAX];
};

#define ICE_HASH_GTPU_CTX_EH_IP		0
#define ICE_HASH_GTPU_CTX_EH_IP_UDP	1
#define ICE_HASH_GTPU_CTX_EH_IP_TCP	2
#define ICE_HASH_GTPU_CTX_UP_IP		3
#define ICE_HASH_GTPU_CTX_UP_IP_UDP	4
#define ICE_HASH_GTPU_CTX_UP_IP_TCP	5
#define ICE_HASH_GTPU_CTX_DW_IP		6
#define ICE_HASH_GTPU_CTX_DW_IP_UDP	7
#define ICE_HASH_GTPU_CTX_DW_IP_TCP	8
#define ICE_HASH_GTPU_CTX_MAX		9

struct ice_vf_hash_gtpu_ctx {
	struct ice_rss_hash_cfg ctx[ICE_HASH_GTPU_CTX_MAX];
};

struct ice_vf_hash_ctx {
	struct ice_vf_hash_ip_ctx v4;
	struct ice_vf_hash_ip_ctx v6;
	struct ice_vf_hash_gtpu_ctx ipv4;
	struct ice_vf_hash_gtpu_ctx ipv6;
};

/* In ADQ, max 4 VSI's can be allocated per VF including primary VF VSI.
 * These variables are used to store indices, ID's and number of queues
 * for each VSI including that of primary VF VSI. Each Traffic class is
 * termed as channel and each channel can in-turn have 4 queues which
 * means max 16 queues overall per VF.
 */
struct ice_channel_vf {
	u16 vsi_idx; /* index in PF struct for all channel VSIs */
	u16 vsi_num; /* HW (absolute) index of this VSI */
	u16 num_qps; /* number of queue pairs requested by user */
	u16 offset;
	u64 max_tx_rate; /* Tx rate limiting for channels */
};

/* The VF VLAN information controlled by DCF */
struct ice_dcf_vlan_info {
	struct ice_vlan outer_port_vlan;
	u16 outer_stripping_tpid;
	u8 outer_stripping_ena:1;
	u8 applying:1;
};

/* Structure to store fdir fv entry */
struct ice_fdir_prof_info {
	struct ice_parser_profile prof;
	u64 fdir_active_cnt;
};

/* Structure to store RSS field vector entry */
struct ice_rss_prof_info {
	struct ice_parser_profile prof;
	bool symm;
};

/* VF operations */
struct ice_vf_ops {
	enum ice_disq_rst_src reset_type;
	void (*free)(struct ice_vf *vf);
	void (*clear_reset_state)(struct ice_vf *vf);
	void (*clear_mbx_register)(struct ice_vf *vf);
	void (*trigger_reset_register)(struct ice_vf *vf, bool is_vflr);
	bool (*poll_reset_status)(struct ice_vf *vf);
	void (*clear_reset_trigger)(struct ice_vf *vf);
	void (*irq_close)(struct ice_vf *vf);
	int (*create_vsi)(struct ice_vf *vf);
	void (*post_vsi_rebuild)(struct ice_vf *vf);
	struct ice_q_vector* (*get_q_vector)(struct ice_vsi *vsi,
					     u16 vector_id);
	void (*cfg_rdma_irq_map)(struct ice_vf *vf,
				 struct virtchnl_rdma_qv_info *qv_info);
	void (*clear_rdma_irq_map)(struct ice_vf *vf);
};

/* Virtchnl/SR-IOV config info */
struct ice_vfs {
	DECLARE_HASHTABLE(table, 8);	/* table of VF entries */
	struct mutex table_lock;	/* Lock for protecting the hash table */
	u16 num_supported;		/* max supported VFs on this PF */
	u16 num_qps_per;		/* number of queue pairs per VF */
	u16 num_msix_per;		/* number of MSI-X vectors per VF */
	unsigned long last_printed_mdd_jiffies;	/* MDD message rate limit */
};

struct ice_vf_qs_bw {
	u16 queue_id;
	u32 committed;
	u32 peak;
	u8 tc;
};

#define VIRTCHNL_MSG_MAX 1000
/* VF information structure */
struct ice_vf {
	struct hlist_node entry;
	struct rcu_head rcu;
	struct kref refcnt;
	struct ice_pf *pf;
	struct pci_dev *vfdev;
	/* Used during virtchnl message handling and NDO ops against the VF
	 * that will trigger a VFR
	 */
	struct mutex cfg_lock;

	u16 vf_id;			/* VF ID in the PF space */
	u16 lan_vsi_idx;		/* index into PF struct */
	u16 ctrl_vsi_idx;
	struct ice_vf_fdir fdir;
	struct ice_fdir_prof_info fdir_prof_info[ICE_MAX_PTGS];
	struct ice_vf_fsub fsub;
	struct device_attribute rss_lut_attr;
	struct device_attribute transmit_lldp_attr;
	struct ice_vf_hash_ctx hash_ctx;
	struct ice_rss_prof_info rss_prof_info[ICE_MAX_PTGS];
	struct ice_vf_qs_bw qs_bw[ICE_MAX_QS_PER_VF];
	/* first vector index of this VF in the PF space */
	int first_vector_idx;
	struct ice_sw *vf_sw_id;	/* switch ID the VF VSIs connect to */
	struct virtchnl_version_info vf_ver;
	u32 driver_caps;		/* reported by VF driver */
	u16 stag;			/* VF Port Extender (PE) stag if used */
	struct virtchnl_ether_addr dev_lan_addr;
	struct virtchnl_ether_addr hw_lan_addr;
	struct ice_time_mac legacy_last_added_umac;
	DECLARE_BITMAP(txq_ena, ICE_MAX_QS_PER_VF);
	DECLARE_BITMAP(rxq_ena, ICE_MAX_QS_PER_VF);
	struct ice_vlan port_vlan_info;	/* Port VLAN ID, QoS, and TPID */
	struct virtchnl_vlan_caps vlan_v2_caps;
	struct ice_dcf_vlan_info dcf_vlan_info;
	struct ice_mbx_vf_info mbx_info;
	u8 pf_set_mac:1;		/* VF MAC address set by VMM admin */
	u8 trusted:1;
	u8 spoofchk:1;
	u8 transmit_lldp:1;
#ifdef HAVE_NDO_SET_VF_LINK_STATE
	u8 link_forced:1;
	u8 link_up:1;			/* only valid if VF link is forced */
#endif
	unsigned int min_tx_rate;	/* Minimum Tx bandwidth limit in Mbps */
	unsigned int max_tx_rate;	/* Maximum Tx bandwidth limit in Mbps */
	DECLARE_BITMAP(vf_states, ICE_VF_STATES_NBITS);	/* VF runtime states */
	/* VF's adv. capabilities */
	DECLARE_BITMAP(vf_caps, ICE_VF_CAP_NBITS);
	u16 num_req_qs;			/* num of queue pairs requested by VF */
	u16 num_mac;
	u16 num_vf_qs;			/* num of queue configured per VF */
	u8 vlan_strip_ena;		/* Outer and Inner VLAN strip enable */
#define ICE_INNER_VLAN_STRIP_ENA	BIT(0)
#define ICE_OUTER_VLAN_STRIP_ENA	BIT(1)
	/* ADQ related variables */
	u8 adq_enabled; /* flag to enable ADQ */
	u8 adq_fltr_ena; /* flag to denote that ADQ filters are applied */
	u8 num_tc;
	u16 num_dmac_chnl_fltrs;
	struct ice_channel_vf ch[VIRTCHNL_MAX_ADQ_V2_CHANNELS];
	struct hlist_head tc_flower_fltr_list;
	struct ice_mdd_vf_events mdd_rx_events;
	struct ice_mdd_vf_events mdd_tx_events;
	DECLARE_BITMAP(opcodes_allowlist, VIRTCHNL_OP_MAX);

	struct ice_repr *repr;
	const struct ice_virtchnl_ops *virtchnl_ops;
	const struct ice_vf_ops *vf_ops;
	struct virtchnl_ptp_caps ptp_caps;

#if IS_ENABLED(CONFIG_NET_DEVLINK)
	/* devlink port data */
	struct devlink_port devlink_port;
#endif /* CONFIG_NET_DEVLINK */
	bool migration_active;
	struct list_head virtchnl_msg_list;
	u64 virtchnl_msg_num;
	u16 vm_vsi_num;

	u16 num_msix;			/* num of MSI-X configured on this VF */
};

/* Flags for controlling behavior of ice_reset_vf */
enum ice_vf_reset_flags {
	ICE_VF_RESET_VFLR = BIT(0), /* Indicate a VFLR reset */
	ICE_VF_RESET_NOTIFY = BIT(1), /* Notify VF prior to reset */
	ICE_VF_RESET_LOCK = BIT(2), /* Acquire the VF cfg_lock */
};

static inline u16 ice_vf_get_port_vlan_id(struct ice_vf *vf)
{
	return vf->port_vlan_info.vid;
}

static inline u8 ice_vf_get_port_vlan_prio(struct ice_vf *vf)
{
	return vf->port_vlan_info.prio;
}

static inline bool ice_vf_is_port_vlan_ena(struct ice_vf *vf)
{
	return (ice_vf_get_port_vlan_id(vf) || ice_vf_get_port_vlan_prio(vf));
}

static inline u16 ice_vf_get_port_vlan_tpid(struct ice_vf *vf)
{
	return vf->port_vlan_info.tpid;
}

/* VF Hash Table access functions
 *
 * These functions provide abstraction for interacting with the VF hash table.
 * In general, direct access to the hash table should be avoided outside of
 * these functions where possible.
 *
 * The VF entries in the hash table are protected by reference counting to
 * track lifetime of accesses from the table. The ice_get_vf_by_id() function
 * obtains a reference to the VF structure which must be dropped by using
 * ice_put_vf().
 */

/**
 * ice_for_each_vf - Iterate over each VF entry
 * @pf: pointer to the PF private structure
 * @bkt: bucket index used for iteration
 * @vf: pointer to the VF entry currently being processed in the loop
 *
 * The bkt variable is an unsigned integer iterator used to traverse the VF
 * entries. It is *not* guaranteed to be the VF's vf_id. Do not assume it is.
 * Use vf->vf_id to get the id number if needed.
 *
 * The caller is expected to be under the table_lock mutex for the entire
 * loop. Use this iterator if your loop is long or if it might sleep.
 */
#define ice_for_each_vf(pf, bkt, vf) \
	hash_for_each((pf)->vfs.table, (bkt), (vf), entry)

/**
 * ice_for_each_vf_rcu - Iterate over each VF entry protected by RCU
 * @pf: pointer to the PF private structure
 * @bkt: bucket index used for iteration
 * @vf: pointer to the VF entry currently being processed in the loop
 *
 * The bkt variable is an unsigned integer iterator used to traverse the VF
 * entries. It is *not* guaranteed to be the VF's vf_id. Do not assume it is.
 * Use vf->vf_id to get the id number if needed.
 *
 * The caller is expected to be under rcu_read_lock() for the entire loop.
 * Only use this iterator if your loop is short and you can guarantee it does
 * not sleep.
 */
#define ice_for_each_vf_rcu(pf, bkt, vf) \
	hash_for_each_rcu((pf)->vfs.table, (bkt), (vf), entry)

#ifdef CONFIG_PCI_IOV
/* The vf_id parameter is a u32 in order to handle IDs stored as u32 values
 * without implicit truncation.
 */
struct ice_vf *ice_get_vf_by_id(struct ice_pf *pf, u32 vf_id);
struct ice_vf *ice_get_vf_by_dev(struct device *dev);
void ice_put_vf(struct ice_vf *vf);
bool ice_is_valid_vf_id(struct ice_pf *pf, u32 vf_id);
bool ice_has_vfs(struct ice_pf *pf);
u16 ice_get_num_vfs(struct ice_pf *pf);
struct ice_vsi *ice_get_vf_vsi(struct ice_vf *vf);
bool ice_is_vf_disabled(struct ice_vf *vf);
int ice_check_vf_ready_for_cfg(struct ice_vf *vf);
void ice_set_vf_state_qs_dis(struct ice_vf *vf);
bool ice_is_any_vf_in_unicast_promisc(struct ice_pf *pf);
void ice_vf_get_promisc_masks(struct ice_vf *vf, struct ice_vsi *vsi,
			      unsigned long *ucast_m, unsigned long *mcast_m);
int ice_vf_set_vsi_promisc(struct ice_vf *vf, struct ice_vsi *vsi,
			   unsigned long *promisc_m);
int ice_vf_clear_vsi_promisc(struct ice_vf *vf, struct ice_vsi *vsi,
			     unsigned long *promisc_m);
int ice_reset_vf(struct ice_vf *vf, u32 flags);
void ice_reset_all_vfs(struct ice_pf *pf);
int ice_init_vf_sysfs(struct ice_vf *vf);
int ice_handle_vf_tx_lldp(struct ice_vf *vf, bool ena);
void ice_ena_all_vfs_rx_lldp(struct ice_pf *pf);
int ice_ena_vf_rx_lldp(struct ice_vf *vf);
int ice_vf_reconfig_vsi(struct ice_vf *vf);
#else /* CONFIG_PCI_IOV */
static inline void ice_ena_all_vfs_rx_lldp(struct ice_pf *pf)
{
}

static inline int ice_handle_vf_tx_lldp(struct ice_vf *vf, bool ena)
{
	return -EOPNOTSUPP;
}

static inline int ice_init_vf_sysfs(struct ice_vf *vf)
{
	return -EOPNOTSUPP;
}

static inline int ice_vf_reconfig_vsi(struct ice_vf *vf)
{
	return -EOPNOTSUPP;
}

static inline struct ice_vf *ice_get_vf_by_id(struct ice_pf *pf, u32 vf_id)
{
	return NULL;
}

static inline struct ice_vf *ice_get_vf_by_dev(struct device *dev)
{
	return NULL;
}

static inline void ice_put_vf(struct ice_vf *vf)
{
}

static inline bool ice_is_valid_vf_id(struct ice_pf *pf, u32 vf_id)
{
	return false;
}

static inline bool ice_has_vfs(struct ice_pf *pf)
{
	return false;
}

static inline u16 ice_get_num_vfs(struct ice_pf *pf)
{
	return 0;
}

static inline struct ice_vsi *ice_get_vf_vsi(struct ice_vf *vf)
{
	return NULL;
}

static inline bool ice_is_vf_disabled(struct ice_vf *vf)
{
	return true;
}

static inline int ice_check_vf_ready_for_cfg(struct ice_vf *vf)
{
	return -EOPNOTSUPP;
}

static inline void ice_set_vf_state_qs_dis(struct ice_vf *vf)
{
}

static inline bool ice_is_any_vf_in_unicast_promisc(struct ice_pf *pf)
{
	return false;
}

static inline int
ice_vf_set_vsi_promisc(struct ice_vf *vf, struct ice_vsi *vsi,
		       unsigned long *promisc_m)
{
	return -EOPNOTSUPP;
}

static inline int
ice_vf_clear_vsi_promisc(struct ice_vf *vf, struct ice_vsi *vsi,
			 unsigned long *promisc_m)
{
	return -EOPNOTSUPP;
}

static inline int ice_reset_vf(struct ice_vf *vf, bool is_vflr)
{
	return 0;
}

static inline void ice_reset_all_vfs(struct ice_pf *pf)
{
}
#endif /* !CONFIG_PCI_IOV */

#endif /* _ICE_VF_LIB_H_ */
