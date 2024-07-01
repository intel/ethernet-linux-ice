/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2018-2024 Intel Corporation */

#ifndef _ICE_SRIOV_H_
#define _ICE_SRIOV_H_
#include "ice_vf_lib.h"
#include "ice_virtchnl.h"

#define ICE_MAX_SRIOV_VFS	256

/* Static VF transaction/status register def */
#define VF_DEVICE_STATUS		0xAA
#define VF_TRANS_PENDING_M		0x20

/* wait defines for polling PF_PCI_CIAD register status */
#define ICE_PCI_CIAD_WAIT_COUNT		100
#define ICE_PCI_CIAD_WAIT_DELAY_US	1

#define ICE_MIN_QS_PER_VF		1
#define ICE_NONQ_VECS_VF		1

#define ICE_NUM_VF_MSIX_MAX		65
#define ICE_NUM_VF_MSIX_LARGE		33
#define ICE_NUM_VF_MSIX_MED		17
#define ICE_NUM_VF_MSIX_SMALL		5
#define ICE_NUM_VF_MSIX_MULTIQ_MIN	3
#define ICE_MIN_INTR_PER_VF		(ICE_MIN_QS_PER_VF + 1)

#define ICE_MAX_VF_RESET_TRIES		40
#define ICE_MAX_VF_RESET_SLEEP_MS	20

/**
 * ice_vf_chnl_dmac_fltr_cnt - number of dmac based channel filters
 * @vf: pointer to the VF info
 */
static inline u16 ice_vf_chnl_dmac_fltr_cnt(struct ice_vf *vf)
{
	return vf->num_dmac_chnl_fltrs;
}

#ifdef CONFIG_PCI_IOV
int
ice_get_vf_port_info(struct ice_pf *pf, u16 vf_id,
		     struct iidc_vf_port_info *vf_port_info);
void ice_dump_all_vfs(struct ice_pf *pf);
void ice_process_vflr_event(struct ice_pf *pf);
#ifdef HAVE_DEVLINK_RELOAD_ACTION_AND_LIMIT
u64 ice_sriov_get_vf_used_msix(struct ice_pf *pf);
#endif /* HAVE_DEVLINK_RELOAD_ACTION_AND_LIMIT */
#ifdef HAVE_PER_VF_MSIX_SYSFS
u32 ice_sriov_get_vf_total_msix(struct pci_dev *pdev);
int ice_sriov_set_msix_vec_count(struct pci_dev *vf_dev, int msix_vec_count);
#endif /* HAVE_PER_VF_MSIX_SYSFS */
int ice_sriov_configure(struct pci_dev *pdev, int num_vfs);
int ice_set_vf_mac(struct net_device *netdev, int vf_id, u8 *mac);
int
ice_get_vf_cfg(struct net_device *netdev, int vf_id, struct ifla_vf_info *ivi);
void ice_free_vfs(struct ice_pf *pf);

/* VF configuration related iplink handlers */
void ice_restore_all_vfs_msi_state(struct pci_dev *pdev);

#ifdef IFLA_VF_VLAN_INFO_MAX
int
ice_set_vf_port_vlan(struct net_device *netdev, int vf_id, u16 vlan_id, u8 qos,
		     __be16 vlan_proto);
#else
int
ice_set_vf_port_vlan(struct net_device *netdev, int vf_id, u16 vlan_id, u8 qos);
#endif

#ifdef HAVE_NDO_SET_VF_MIN_MAX_TX_RATE
int
ice_set_vf_bw(struct net_device *netdev, int vf_id, int min_tx_rate,
	      int max_tx_rate);
#else
int ice_set_vf_bw(struct net_device *netdev, int vf_id, int tx_rate);
#endif

#ifdef HAVE_NDO_SET_VF_TRUST
int ice_set_vf_trust(struct net_device *netdev, int vf_id, bool trusted);
#endif

#ifdef HAVE_NDO_SET_VF_LINK_STATE
int ice_set_vf_link_state(struct net_device *netdev, int vf_id, int link_state);
#endif

int ice_set_vf_spoofchk(struct net_device *netdev, int vf_id, bool ena);

int ice_calc_vf_reg_idx(struct ice_vf *vf, struct ice_q_vector *q_vector,
			u8 tc);

#ifdef HAVE_VF_STATS
int
ice_get_vf_stats(struct net_device *netdev, int vf_id,
		 struct ifla_vf_stats *vf_stats);
#endif /* HAVE_VF_STATS */
void
ice_vf_lan_overflow_event(struct ice_pf *pf, struct ice_rq_event_info *event);
void ice_print_vfs_mdd_events(struct ice_pf *pf);
void ice_print_vf_rx_mdd_event(struct ice_vf *vf);
bool ice_vc_validate_pattern(struct ice_vf *vf,
			     struct virtchnl_proto_hdrs *proto);
bool ice_vc_isvalid_vsi_id(struct ice_vf *vf, u16 vsi_id);
#else /* CONFIG_PCI_IOV */
static inline void ice_dump_all_vfs(struct ice_pf *pf) { }
static inline void ice_process_vflr_event(struct ice_pf *pf) { }
static inline void ice_free_vfs(struct ice_pf *pf) { }
static inline void
ice_vf_lan_overflow_event(struct ice_pf *pf,
			  struct ice_rq_event_info *event) { }
static inline void ice_print_vfs_mdd_events(struct ice_pf *pf) { }
static inline void ice_print_vf_rx_mdd_event(struct ice_vf *vf) { }
static inline void ice_restore_all_vfs_msi_state(struct pci_dev *pdev) { }
static inline int
ice_get_vf_port_info(struct ice_pf __always_unused *pf,
		     u16 __always_unused vf_id,
		     struct iidc_vf_port_info __always_unused *vf_port_info)
{
	return -EOPNOTSUPP;
}

#ifdef HAVE_DEVLINK_RELOAD_ACTION_AND_LIMIT
static inline u64 ice_sriov_get_vf_used_msix(struct ice_pf *pf)
{
	return 0;
}
#endif /* HAVE_DEVLINK_RELOAD_ACTION_AND_LIMIT */

#ifdef HAVE_PER_VF_MSIX_SYSFS
static inline u32 ice_sriov_get_vf_total_msix(struct pci_dev *pdev)
{
	return 0;
}

static inline int
ice_sriov_set_msix_vec_count(struct pci_dev *vf_dev, int msix_vec_count)
{
	return -EOPNOTSUPP;
}
#endif /* !HAVE_PER_VF_MSIX_SYSFS */

static inline int
ice_sriov_configure(struct pci_dev __always_unused *pdev,
		    int __always_unused num_vfs)
{
	return -EOPNOTSUPP;
}

static inline int
ice_set_vf_mac(struct net_device __always_unused *netdev,
	       int __always_unused vf_id, u8 __always_unused *mac)
{
	return -EOPNOTSUPP;
}

static inline int
ice_get_vf_cfg(struct net_device __always_unused *netdev,
	       int __always_unused vf_id,
	       struct ifla_vf_info __always_unused *ivi)
{
	return -EOPNOTSUPP;
}

#ifdef HAVE_NDO_SET_VF_TRUST
static inline int
ice_set_vf_trust(struct net_device __always_unused *netdev,
		 int __always_unused vf_id, bool __always_unused trusted)
{
	return -EOPNOTSUPP;
}
#endif /* HAVE_NDO_SET_VF_TRUST */

#ifdef IFLA_VF_VLAN_INFO_MAX
static inline int
ice_set_vf_port_vlan(struct net_device __always_unused *netdev,
		     int __always_unused vf_id, u16 __always_unused vid,
		     u8 __always_unused qos, __be16 __always_unused v_proto)
{
	return -EOPNOTSUPP;
}
#else
static inline int
ice_set_vf_port_vlan(struct net_device __always_unused *netdev,
		     int __always_unused vf_id, u16 __always_unused vid,
		     u8 __always_unused qos)
{
	return -EOPNOTSUPP;
}
#endif /* IFLA_VF_VLAN_INFO_MAX */

static inline int
ice_set_vf_spoofchk(struct net_device __always_unused *netdev,
		    int __always_unused vf_id, bool __always_unused ena)
{
	return -EOPNOTSUPP;
}

#ifdef HAVE_NDO_SET_VF_LINK_STATE
static inline int
ice_set_vf_link_state(struct net_device __always_unused *netdev,
		      int __always_unused vf_id, int __always_unused link_state)
{
	return -EOPNOTSUPP;
}
#endif /* HAVE_NDO_SET_VF_LINK_STATE */

#ifdef HAVE_NDO_SET_VF_MIN_MAX_TX_RATE
static inline int
ice_set_vf_bw(struct net_device __always_unused *netdev,
	      int __always_unused vf_id, int __always_unused min_tx_rate,
	      int __always_unused max_tx_rate)
#else
static inline int
ice_set_vf_bw(struct net_device __always_unused *netdev,
	      int __always_unused vf_id, int __always_unused max_tx_rate)
#endif
{
	return -EOPNOTSUPP;
}

static inline int
ice_calc_vf_reg_idx(struct ice_vf __always_unused *vf,
		    struct ice_q_vector __always_unused *q_vector,
		    u8 __always_unused tc)
{
	return 0;
}

#ifdef HAVE_VF_STATS
static inline int
ice_get_vf_stats(struct net_device __always_unused *netdev,
		 int __always_unused vf_id,
		 struct ifla_vf_stats __always_unused *vf_stats)
{
	return -EOPNOTSUPP;
}
#endif /* HAVE_VF_STATS */
#endif /* CONFIG_PCI_IOV */

static inline u16 ice_abs_vf_id(struct ice_hw *hw, u16 rel_vf_id)
{
	return rel_vf_id + hw->func_caps.vf_base_id;
}

static inline u16 ice_rel_vf_id(struct ice_hw *hw, u16 abs_vf_id)
{
	return abs_vf_id - hw->func_caps.vf_base_id;
}
#endif /* _ICE_SRIOV_H_ */
