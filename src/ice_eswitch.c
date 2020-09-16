// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2018-2019, Intel Corporation. */

#if IS_ENABLED(CONFIG_NET_DEVLINK)
#include "ice.h"
#include "ice_lib.h"
#include "ice_eswitch.h"
#include "ice_fltr.h"
#include "ice_repr.h"

#include <net/dst_metadata.h>

/**
 * ice_eswitch_vsi_change_lan_en - change default LB/LAN enable bit
 * @vsi: pointer to VSI given rule belongs to
 * @rule_id: rule identifier
 * @lan_en: bool, false to prevent packet from being sent to network
 *
 * Used in switchdev mode to prevent packets from being sent to LAN.
 */
static int
ice_eswitch_vsi_change_lan_en(struct ice_vsi *vsi, u16 rule_id, bool lan_en)
{
	struct ice_aqc_sw_rules_elem *s_rule;
	struct ice_hw *hw = &vsi->back->hw;
	u32 act = 0;
	int err;

	s_rule = devm_kzalloc(ice_hw_to_dev(hw),
			      ICE_SW_RULE_RX_TX_NO_HDR_SIZE, GFP_KERNEL);
	if (!s_rule)
		return -ENOMEM;

	act = (vsi->vsi_num << ICE_SINGLE_ACT_VSI_ID_S) &
	       ICE_SINGLE_ACT_VSI_ID_M;
	act |= ICE_SINGLE_ACT_VALID_BIT;

	if (lan_en)
		act |= ICE_SINGLE_ACT_LAN_ENABLE;
	else
		act |= ICE_SINGLE_ACT_LB_ENABLE;

	s_rule->pdata.lkup_tx_rx.recipe_id = cpu_to_le16(ICE_SW_LKUP_DFLT);
	s_rule->pdata.lkup_tx_rx.index = cpu_to_le16(rule_id);
	s_rule->pdata.lkup_tx_rx.act = cpu_to_le32(act);

	s_rule->type = cpu_to_le16(ICE_AQC_SW_RULES_T_LKUP_TX);
	s_rule->pdata.lkup_tx_rx.src = cpu_to_le16(vsi->vsi_num);

	err = ice_aq_sw_rules(hw, s_rule, ICE_SW_RULE_RX_TX_ETH_HDR_SIZE, 1,
			      ice_aqc_opc_update_sw_rules, NULL);

	devm_kfree(ice_hw_to_dev(hw), s_rule);
	return err;
}

/**
 * ice_eswitch_setup_env - configure switchdev HW filters
 * @pf: pointer to PF struct
 *
 * This function adds HW filters configuration specific for switchdev
 * mode.
 */
static int
ice_eswitch_setup_env(struct ice_pf *pf)
{
	struct ice_vsi *uplink_vsi = pf->switchdev.uplink_vsi;
	struct ice_vsi *ctrl_vsi = pf->switchdev.control_vsi;
	struct ice_port_info *pi = pf->hw.port_info;

	ice_remove_vsi_fltr(&pf->hw, uplink_vsi->idx);

	if (ice_vsi_add_vlan(uplink_vsi, 0, ICE_FWD_TO_VSI))
		goto err_def_rx;

	if (ice_cfg_dflt_vsi(pi, uplink_vsi->idx, true, ICE_FLTR_RX))
		goto err_def_rx;
	uplink_vsi->vsw->dflt_vsi = uplink_vsi;
	uplink_vsi->vsw->dflt_vsi_ena = true;

	if (ice_cfg_dflt_vsi(pi, ctrl_vsi->idx, true, ICE_FLTR_TX))
		goto err_def_tx;

	if (ice_vsi_update_security(uplink_vsi, ice_vsi_ctx_set_allow_override))
		goto err_override_uplink;

	if (ice_vsi_update_security(ctrl_vsi, ice_vsi_ctx_set_allow_override))
		goto err_override_control;

	if (ice_eswitch_vsi_change_lan_en(ctrl_vsi, pi->dflt_tx_vsi_rule_id,
					  false))
		goto err_update_action;

	return 0;

err_update_action:
	ice_vsi_update_security(ctrl_vsi, ice_vsi_ctx_clear_allow_override);
err_override_control:
	ice_vsi_update_security(uplink_vsi, ice_vsi_ctx_clear_allow_override);
err_override_uplink:
	ice_cfg_dflt_vsi(pi, ctrl_vsi->idx, false, ICE_FLTR_TX);
err_def_tx:
	ice_cfg_dflt_vsi(pi, uplink_vsi->idx, false, ICE_FLTR_RX);
	uplink_vsi->vsw->dflt_vsi = NULL;
	uplink_vsi->vsw->dflt_vsi_ena = false;
err_def_rx:
	ice_fltr_add_mac_and_broadcast(uplink_vsi,
				       uplink_vsi->port_info->mac.perm_addr,
				       ICE_FWD_TO_VSI);
	return -ENODEV;
}

/**
 * ice_eswitch_release_env - clear switchdev HW filters
 * @pf: pointer to PF struct
 *
 * This function removes HW filters configuration specific for switchdev
 * mode and restores default legacy mode settings.
 */
static void
ice_eswitch_release_env(struct ice_pf *pf)
{
	struct ice_vsi *uplink_vsi = pf->switchdev.uplink_vsi;
	struct ice_vsi *ctrl_vsi = pf->switchdev.control_vsi;
	struct ice_port_info *pi = pf->hw.port_info;

	ice_vsi_update_security(ctrl_vsi, ice_vsi_ctx_clear_allow_override);
	ice_vsi_update_security(uplink_vsi, ice_vsi_ctx_clear_allow_override);
	ice_cfg_dflt_vsi(pi, ctrl_vsi->idx, false, ICE_FLTR_TX);
	ice_cfg_dflt_vsi(pi, uplink_vsi->idx, false, ICE_FLTR_RX);
	uplink_vsi->vsw->dflt_vsi = NULL;
	uplink_vsi->vsw->dflt_vsi_ena = false;
	ice_fltr_add_mac_and_broadcast(uplink_vsi,
				       uplink_vsi->port_info->mac.perm_addr,
				       ICE_FWD_TO_VSI);
}

#ifdef HAVE_METADATA_PORT_INFO
/**
 * ice_eswitch_setup_reprs - configure port reprs to run in switchdev mode
 * @pf: pointer to PF struct
 * @ctrl_vsi: pointer to switchdev control VSI
 */
static int ice_eswitch_setup_reprs(struct ice_pf *pf, struct ice_vsi *ctrl_vsi)
{
	int max_vsi_num = 0;
	int i;

	ice_for_each_vf(pf, i) {
		struct ice_vsi *vsi = pf->vf[i].repr->src_vsi;
		struct ice_vf *vf = &pf->vf[i];

		ice_remove_vsi_fltr(&pf->hw, vsi->idx);
		vf->repr->dst = metadata_dst_alloc(0, METADATA_HW_PORT_MUX,
						   GFP_KERNEL);
		if (!vf->repr->dst) {
			ice_fltr_add_mac_and_broadcast(vsi,
						       vf->hw_lan_addr.addr,
						       ICE_FWD_TO_VSI);
			goto err;
		}

		if (ice_vsi_update_security(vsi, ice_vsi_ctx_clear_antispoof)) {
			ice_fltr_add_mac_and_broadcast(vsi,
						       vf->hw_lan_addr.addr,
						       ICE_FWD_TO_VSI);
			metadata_dst_free(vf->repr->dst);
			goto err;
		}

		if (max_vsi_num < vsi->vsi_num)
			max_vsi_num = vsi->vsi_num;
	}

	ctrl_vsi->target_netdevs = kcalloc(max_vsi_num,
					   sizeof(*ctrl_vsi->target_netdevs),
					   GFP_KERNEL);
	if (!ctrl_vsi->target_netdevs)
		goto err;

	i = 0;
	ice_for_each_vf(pf, i) {
		struct ice_repr *repr = pf->vf[i].repr;
		struct ice_vsi *vsi = repr->src_vsi;
		struct metadata_dst *dst;

		ctrl_vsi->target_netdevs[vsi->vsi_num] = repr->netdev;

		dst = repr->dst;
		dst->u.port_info.port_id = vsi->vsi_num;
		dst->u.port_info.lower_dev = repr->netdev;
		ice_repr_set_traffic_vsi(repr, ctrl_vsi);
	}

	return 0;

err:
	for (i = i - 1; i >= 0; i--) {
		struct ice_vsi *vsi = pf->vf[i].repr->src_vsi;
		struct ice_vf *vf = &pf->vf[i];

		ice_vsi_update_security(vsi, ice_vsi_ctx_set_antispoof);
		metadata_dst_free(vf->repr->dst);
		ice_fltr_add_mac_and_broadcast(vsi, vf->hw_lan_addr.addr,
					       ICE_FWD_TO_VSI);
	}

	return -ENODEV;
}

/**
 * ice_eswitch_release_reprs - clear PR VSIs configuration
 * @pf: poiner to PF struct
 * @ctrl_vsi: pointer to switchdev control VSI
 */
static void ice_eswitch_release_reprs(struct ice_pf *pf,
				      struct ice_vsi *ctrl_vsi)
{
	int i;

	kfree(ctrl_vsi->target_netdevs);
	ice_for_each_vf(pf, i) {
		struct ice_vsi *vsi = pf->vf[i].repr->src_vsi;
		struct ice_vf *vf = &pf->vf[i];

		ice_vsi_update_security(vsi, ice_vsi_ctx_set_antispoof);
		metadata_dst_free(vf->repr->dst);
		ice_fltr_add_mac_and_broadcast(vsi, vf->hw_lan_addr.addr,
					       ICE_FWD_TO_VSI);
	}
}

/**
 * ice_eswitch_port_start_xmit - callback for packets transmit
 * @skb: send buffer
 * @netdev: network interface device structure
 *
 * Returns NETDEV_TX_OK if sent, else an error code
 */
netdev_tx_t
ice_eswitch_port_start_xmit(struct sk_buff *skb, struct net_device *netdev)
{
	struct ice_repr *repr = ice_netdev_to_repr(netdev);

	skb_dst_drop(skb);
	dst_hold((struct dst_entry *)repr->dst);
	skb_dst_set(skb, (struct dst_entry *)repr->dst);

	return ice_start_xmit(skb, netdev);
}

/**
 * ice_eswitch_set_target_vsi - set switchdev context in Tx context descriptor
 * @skb: pointer to send buffer
 * @off: pointer to offload struct
 */
void ice_eswitch_set_target_vsi(struct sk_buff *skb,
				struct ice_tx_offload_params *off)
{
	struct metadata_dst *dst = skb_metadata_dst(skb);
	u64 cd_cmd, dst_vsi;

	if (!dst) {
		cd_cmd = ICE_TX_CTX_DESC_SWTCH_UPLINK << ICE_TXD_CTX_QW1_CMD_S;
		off->cd_qw1 |= (cd_cmd | ICE_TX_DESC_DTYPE_CTX);
	} else {
		cd_cmd = ICE_TX_CTX_DESC_SWTCH_VSI << ICE_TXD_CTX_QW1_CMD_S;
		dst_vsi = ((u64)dst->u.port_info.port_id <<
			   ICE_TXD_CTX_QW1_VSI_S) & ICE_TXD_CTX_QW1_VSI_M;
		off->cd_qw1 = cd_cmd | dst_vsi | ICE_TX_DESC_DTYPE_CTX;
	}
}
#else
static void
ice_eswitch_release_reprs(struct ice_pf __always_unused *pf,
			  struct ice_vsi __always_unused *ctrl_vsi)
{
}

static int ice_eswitch_setup_reprs(struct ice_pf __always_unused *pf,
				   struct ice_vsi __always_unused *ctrl_vsi)
{
	return -ENODEV;
}

netdev_tx_t
ice_eswitch_port_start_xmit(struct sk_buff __always_unused *skb,
			    struct net_device __always_unused *netdev)
{
	return -EOPNOTSUPP;
}
#endif /* HAVE_METADATA_PORT_INFO */

/**
 * ice_eswitch_vsi_setup - configure switchdev control VSI
 * @pf: pointer to PF structure
 * @pi: pointer to port_info structure
 */
static struct ice_vsi *
ice_eswitch_vsi_setup(struct ice_pf *pf, struct ice_port_info *pi)
{
	return ice_vsi_setup(pf, pi, ICE_VSI_SWITCHDEV_CTRL, ICE_INVAL_VFID, NULL, 0);
}

/**
 * ice_eswitch_set_rxdid - configure rxdid on all rx queues from VSI
 * @vsi: vsi to setup rxdid on
 * @rxdid: flex descriptor id
 */
static void ice_eswitch_set_rxdid(struct ice_vsi *vsi, u32 rxdid)
{
	struct ice_hw *hw = &vsi->back->hw;
	int i;

	ice_for_each_rxq(vsi, i) {
		struct ice_ring *ring = vsi->rx_rings[i];
		u16 pf_q = vsi->rxq_map[ring->q_index];

		ice_write_qrxflxp_cntxt(hw, pf_q, rxdid, 0x3, true);
	}
}

/**
 * ice_eswitch_enable_switchdev - configure eswitch in switchdev mode
 * @pf: pointer to PF structure
 */
static int
ice_eswitch_enable_switchdev(struct ice_pf *pf)
{
	struct ice_vsi *control_vsi;
	struct ice_vsi *uplink_vsi;

#ifdef NETIF_F_HW_TC
	if (ice_is_adq_active(pf)) {
		dev_err(ice_pf_to_dev(pf), "switchdev cannot be configured - ADQ is active. Delete ADQ configs using TC and try again\n");
		return -EOPNOTSUPP;
	}

#endif /* NETIF_F_HW_TC */
	if (!pf->num_alloc_vfs) {
		dev_info(ice_pf_to_dev(pf), "No vfs created");
		return -EOPNOTSUPP;
	}

	pf->switchdev.control_vsi = ice_eswitch_vsi_setup(pf, pf->hw.port_info);
	if (!pf->switchdev.control_vsi)
		return -ENODEV;

	control_vsi = pf->switchdev.control_vsi;
	pf->switchdev.uplink_vsi = ice_get_main_vsi(pf);
	if (!pf->switchdev.uplink_vsi)
		goto err_vsi;

	uplink_vsi = pf->switchdev.uplink_vsi;

	control_vsi->netdev = uplink_vsi->netdev;
	ice_napi_add(control_vsi);

	if (ice_vsi_open(control_vsi))
		goto err_vsi;

	ice_eswitch_set_rxdid(control_vsi, ICE_RXDID_FLEX_NIC_2);
	netif_keep_dst(control_vsi->netdev);

	if (ice_eswitch_setup_env(pf))
		goto err_vsi;

	if (ice_repr_add_for_all_vfs(pf))
		goto err_repr_add;

	if (ice_eswitch_setup_reprs(pf, control_vsi))
		goto err_setup_reprs;

	return 0;

err_setup_reprs:
	ice_repr_rem_from_all_vfs(pf);
err_repr_add:
	ice_eswitch_release_env(pf);
err_vsi:
	ice_vsi_release(control_vsi);
	return -ENODEV;
}

/**
 * ice_eswitch_disable_switchdev - disable switchdev resources
 * @pf: pointer to PF structure
 */
static void ice_eswitch_disable_switchdev(struct ice_pf *pf)
{
	struct ice_vsi *control_vsi = pf->switchdev.control_vsi;

	ice_eswitch_release_reprs(pf, control_vsi);
	ice_repr_rem_from_all_vfs(pf);
	ice_eswitch_release_env(pf);
	ice_vsi_release(control_vsi);
}

#ifdef HAVE_METADATA_PORT_INFO
#ifdef HAVE_DEVLINK_ESWITCH_OPS_EXTACK
/**
 * ice_eswitch_mode_set - set new eswitch mode
 * @devlink: pointer to devlink structure
 * @mode: eswitch mode to switch to
 * @extack: pointer to extack structure
 */
int ice_eswitch_mode_set(struct devlink *devlink, u16 mode,
			 struct netlink_ext_ack *extack)
#else
int ice_eswitch_mode_set(struct devlink *devlink, u16 mode)
#endif /* HAVE_DEVLINK_ESWITCH_OPS_EXTACK */
{
	struct ice_pf *pf = devlink_priv(devlink);

	if (pf->eswitch_mode == mode)
		return 0;

	switch (mode) {
	case DEVLINK_ESWITCH_MODE_LEGACY:
		ice_eswitch_disable_switchdev(pf);
		break;
	case DEVLINK_ESWITCH_MODE_SWITCHDEV:
		if (ice_eswitch_enable_switchdev(pf)) {
#ifdef HAVE_DEVLINK_ESWITCH_OPS_EXTACK
			NL_SET_ERR_MSG_MOD(extack, "Unable to go to switchdev");
#else
			dev_err(ice_pf_to_dev(pf), "Unable to go to switchdev");
#endif /* HAVE_DEVLINK_ESWITCH_OPS_EXTACK */
			return -EINVAL;
		}
		break;
	default:
#ifdef HAVE_DEVLINK_ESWITCH_OPS_EXTACK
		NL_SET_ERR_MSG_MOD(extack, "Unknown eswitch mode");
#else
		dev_err(ice_pf_to_dev(pf), "Unknown eswitch_mode");
#endif /* HAVE_DEVLINK_ESWITCH_OPS_EXTACK */
		return -EINVAL;
	}

	pf->eswitch_mode = mode;
	return 0;
}
#endif /* HAVE_METADATA_PORT_INFO */

/**
 * ice_eswitch_get_target_netdev - return port representor netdev
 * @rx_ring: pointer to rx ring
 * @rx_desc: pointer to rx descriptor
 *
 * When working in switchdev mode context (when control vsi is used), this
 * function returns netdev of appropriate port representor. For non-switchdev
 * context, regular netdev associated with rx ring is returned.
 */
struct net_device *
ice_eswitch_get_target_netdev(struct ice_ring *rx_ring,
			      union ice_32b_rx_flex_desc *rx_desc)
{
	struct ice_32b_rx_flex_desc_nic_2 *desc;
	struct ice_vsi *vsi = rx_ring->vsi;
	struct ice_vsi *control_vsi;
	u16 target_vsi_id;

	control_vsi = vsi->back->switchdev.control_vsi;
	if (vsi != control_vsi)
		return rx_ring->netdev;

	desc = (struct ice_32b_rx_flex_desc_nic_2 *)rx_desc;
	target_vsi_id = le16_to_cpu(desc->src_vsi);

	return vsi->target_netdevs[target_vsi_id];
}

/**
 * ice_eswitch_mode_get - get current eswitch mode
 * @devlink: pointer to devlink structure
 * @mode: output parameter for current eswitch mode
 */
int ice_eswitch_mode_get(struct devlink *devlink, u16 *mode)
{
	struct ice_pf *pf = devlink_priv(devlink);

	*mode = pf->eswitch_mode;
	return 0;
}

/**
 * ice_is_eswitch_mode_switchdev - check if eswitch mode is set to switchdev
 * @pf: pointer to PF structure
 *
 * Returns true if eswitch mode is set to DEVLINK_ESWITCH_MODE_SWITCHDEV,
 * false otherwise.
 */
bool ice_is_eswitch_mode_switchdev(struct ice_pf *pf)
{
	return pf->eswitch_mode == DEVLINK_ESWITCH_MODE_SWITCHDEV;
}

/**
 * ice_eswitch_release - cleanup eswitch
 * @pf: pointer to PF structure
 */
void ice_eswitch_release(struct ice_pf *pf)
{
	if (pf->eswitch_mode == DEVLINK_ESWITCH_MODE_LEGACY)
		return;

	ice_eswitch_disable_switchdev(pf);
	pf->eswitch_mode = DEVLINK_ESWITCH_MODE_LEGACY;
}

/**
 * ice_eswitch_close - cleanup eswitch before rebuild
 * @pf: pointer to PF structure
 *
 * This function is called in rebuild path. If eswitch mode is set to
 * DEVLINK_ESWITCH_MODE_SWITCHDEV, it frees switchdev resources. Current
 * eswitch mode remains unchanged and ice_switch_rebuild function should be
 * called to restore original eswitch configuration.
 */
void ice_eswitch_close(struct ice_pf *pf)
{
	if (pf->eswitch_mode == DEVLINK_ESWITCH_MODE_LEGACY)
		return;

	ice_eswitch_disable_switchdev(pf);
}

/**
 * ice_eswitch_rebuild - rebuild eswitch configuration
 * @pf: pointer to PF structure
 *
 * This function is called in rebuild path to restore original eswitch
 * configuration after reset. If the eswitch mode was set to
 * DEVLINK_ESWITCH_MODE_SWITCHDEV before the reset, all switchdev configuration
 * is restored. No action is taken otherwise.
 */
void ice_eswitch_rebuild(struct ice_pf *pf)
{
	if (pf->eswitch_mode == DEVLINK_ESWITCH_MODE_LEGACY)
		return;

	if (ice_eswitch_enable_switchdev(pf))
		return;
#ifdef HAVE_METADATA_PORT_INFO
	pf->eswitch_mode = DEVLINK_ESWITCH_MODE_SWITCHDEV;
#endif /* HAVE_METADATA_PORT_INFO */
}
#endif /* CONFIG_NET_DEVLINK */
