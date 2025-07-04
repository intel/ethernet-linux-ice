/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2018-2025 Intel Corporation */

#if IS_ENABLED(CONFIG_NET_DEVLINK)
#include "ice.h"
#include "ice_lib.h"
#include "ice_eswitch.h"
#include "ice_fltr.h"
#include "ice_repr.h"
#include "ice_devlink.h"
#include "ice_tc_lib.h"

/**
 * ice_eswitch_setup_env - configure eswitch HW filters
 * @pf: pointer to PF struct
 *
 * This function adds HW filters configuration specific for switchdev
 * mode.
 */
static int ice_eswitch_setup_env(struct ice_pf *pf)
{
	struct ice_vsi *uplink_vsi = pf->eswitch.uplink_vsi;
	struct net_device *netdev = uplink_vsi->netdev;
	bool if_running = netif_running(netdev);
	struct ice_vsi_vlan_ops *vlan_ops;

	if (if_running && !test_and_set_bit(ICE_VSI_DOWN, uplink_vsi->state) &&
	    ice_down(uplink_vsi))
		return -ENODEV;

	ice_remove_vsi_fltr(&pf->hw, uplink_vsi->idx);

	netif_addr_lock_bh(netdev);
	__dev_uc_unsync(netdev, NULL);
	__dev_mc_unsync(netdev, NULL);
	netif_addr_unlock_bh(netdev);

	if (ice_vsi_add_vlan_zero(uplink_vsi))
		goto err_vlan_zero;

	if (ice_set_dflt_vsi(uplink_vsi))
		goto err_def_rx;

	if (ice_cfg_dflt_vsi(uplink_vsi->port_info, uplink_vsi->idx, true,
			     ICE_FLTR_TX))
		goto err_def_tx;

	vlan_ops = ice_get_compat_vsi_vlan_ops(uplink_vsi);
	if (vlan_ops->dis_rx_filtering(uplink_vsi))
		goto err_vlan_filtering;

	if (ice_vsi_update_security(uplink_vsi, ice_vsi_ctx_set_allow_override))
		goto err_override_uplink;

	if (ice_vsi_update_local_lb(uplink_vsi, true))
		goto err_override_local_lb;

	if (if_running && ice_up(uplink_vsi))
		goto err_up;

	return 0;

err_up:
	ice_vsi_update_local_lb(uplink_vsi, false);
err_override_local_lb:
	ice_vsi_update_security(uplink_vsi, ice_vsi_ctx_clear_allow_override);
err_override_uplink:
	vlan_ops->ena_rx_filtering(uplink_vsi);
err_vlan_filtering:
	ice_cfg_dflt_vsi(uplink_vsi->port_info, uplink_vsi->idx, false,
			 ICE_FLTR_TX);
err_def_tx:
	ice_cfg_dflt_vsi(uplink_vsi->port_info, uplink_vsi->idx, false,
			 ICE_FLTR_RX);
err_def_rx:
	ice_vsi_del_vlan_zero(uplink_vsi);
err_vlan_zero:
	ice_fltr_add_mac_and_broadcast(uplink_vsi,
				       uplink_vsi->port_info->mac.perm_addr,
				       ICE_FWD_TO_VSI);
	if (if_running)
		ice_up(uplink_vsi);

	return -ENODEV;
}

#ifdef HAVE_METADATA_PORT_INFO
/**
 * ice_eswitch_release_repr - clear PR VSI configuration
 * @pf: poiner to PF struct
 * @repr: pointer to PR struct
 */
static void
ice_eswitch_release_repr(struct ice_pf *pf, struct ice_repr *repr)
{
	struct ice_vsi *vsi = repr->src_vsi;

	/* Skip representors that aren't configured */
	if (!repr->dst)
		return;

	ice_vsi_update_security(vsi, ice_vsi_ctx_set_antispoof);
	metadata_dst_free(repr->dst);
	repr->dst = NULL;
	ice_fltr_add_mac_and_broadcast(vsi, repr->parent_mac,
				       ICE_FWD_TO_VSI);
}

/**
 * ice_eswitch_setup_repr - configure PR to run in switchdev mode
 * @pf: pointer to PF struct
 * @repr: pointer to PR struct
 */
static int ice_eswitch_setup_repr(struct ice_pf *pf, struct ice_repr *repr)
{
	struct ice_vsi *uplink_vsi = pf->eswitch.uplink_vsi;
	struct ice_vsi *vsi = repr->src_vsi;
	struct metadata_dst *dst;

	ice_remove_vsi_fltr(&pf->hw, vsi->idx);
	repr->dst = metadata_dst_alloc(0, METADATA_HW_PORT_MUX,
				       GFP_KERNEL);
	if (!repr->dst)
		goto err_add_mac_fltr;

	if (ice_vsi_update_security(vsi, ice_vsi_ctx_clear_antispoof))
		goto err_update_security;

	if (ice_vsi_add_vlan_zero(vsi))
		goto err_add_vlan;

	netif_keep_dst(uplink_vsi->netdev);

	dst = repr->dst;
	dst->u.port_info.port_id = vsi->vsi_num;
	dst->u.port_info.lower_dev = uplink_vsi->netdev;

	return 0;

err_add_vlan:
	ice_vsi_update_security(vsi, ice_vsi_ctx_set_antispoof);
err_update_security:
	metadata_dst_free(repr->dst);
	repr->dst = NULL;
err_add_mac_fltr:
	ice_fltr_add_mac_and_broadcast(vsi, repr->parent_mac, ICE_FWD_TO_VSI);

	return -ENODEV;
}

/**
 * ice_eswitch_update_repr - reconfigure port representor
 * @repr_id: representor ID
 * @vsi: VF VSI for which port representor is configured
 */
void ice_eswitch_update_repr(unsigned long repr_id, struct ice_vsi *vsi)
{
	struct ice_pf *pf = vsi->back;
	struct ice_repr *repr;
	int ret;

	if (!ice_is_switchdev_running(pf))
		return;

	repr = (struct ice_repr *)
		radix_tree_lookup(&pf->eswitch.reprs, repr_id);
	if (!repr)
		return;

	repr->src_vsi = vsi;
	repr->dst->u.port_info.port_id = vsi->vsi_num;

	ret = ice_vsi_update_security(vsi, ice_vsi_ctx_clear_antispoof);
	if (ret) {
		ice_fltr_add_mac_and_broadcast(vsi, repr->parent_mac,
					       ICE_FWD_TO_VSI);
		dev_err(ice_pf_to_dev(pf), "Failed to update VSI of port representor %d",
			repr->id);
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
	unsigned int len = skb->len;
	int ret;

	skb_dst_drop(skb);
	dst_hold((struct dst_entry *)repr->dst);
	skb_dst_set(skb, (struct dst_entry *)repr->dst);
	skb->dev = repr->dst->u.port_info.lower_dev;

	ret = dev_queue_xmit(skb);
	ice_repr_inc_tx_stats(repr, len, ret);

	return (netdev_tx_t)ret;
}

/**
 * ice_eswitch_set_target_vsi - set eswitch context in Tx context descriptor
 * @skb: pointer to send buffer
 * @off: pointer to offload struct
 */
void
ice_eswitch_set_target_vsi(struct sk_buff *skb,
			   struct ice_tx_offload_params *off)
{
	struct metadata_dst *dst = skb_metadata_dst(skb);
	u64 cd_cmd, dst_vsi;

	if (!dst) {
		cd_cmd = ICE_TX_CTX_DESC_SWTCH_UPLINK << ICE_TXD_CTX_QW1_CMD_S;
		off->cd_qw1 |= (cd_cmd | ICE_TX_DESC_DTYPE_CTX);
	} else {
		cd_cmd = ICE_TX_CTX_DESC_SWTCH_VSI << ICE_TXD_CTX_QW1_CMD_S;
		dst_vsi = FIELD_PREP(ICE_TXD_CTX_QW1_VSI_M,
				     dst->u.port_info.port_id);
		off->cd_qw1 = cd_cmd | dst_vsi | ICE_TX_DESC_DTYPE_CTX;
	}
}
#else
static void
ice_eswitch_release_repr(struct ice_pf __always_unused *pf,
			 struct ice_repr __always_unused *repr)
{
}

static int
ice_eswitch_setup_repr(struct ice_pf __always_unused *pf,
		       struct ice_repr __always_unused *repr)
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
 * ice_eswitch_release_env - clear eswitch HW filters
 * @pf: pointer to PF struct
 *
 * This function removes HW filters configuration specific for switchdev
 * mode and restores default legacy mode settings.
 */
static void ice_eswitch_release_env(struct ice_pf *pf)
{
	struct ice_vsi *uplink_vsi = pf->eswitch.uplink_vsi;
	struct ice_vsi_vlan_ops *vlan_ops;

	vlan_ops = ice_get_compat_vsi_vlan_ops(uplink_vsi);

	ice_vsi_update_local_lb(uplink_vsi, false);
	ice_vsi_update_security(uplink_vsi, ice_vsi_ctx_clear_allow_override);
	vlan_ops->ena_rx_filtering(uplink_vsi);
	ice_cfg_dflt_vsi(uplink_vsi->port_info, uplink_vsi->idx, false,
			 ICE_FLTR_TX);
	ice_cfg_dflt_vsi(uplink_vsi->port_info, uplink_vsi->idx, false,
			 ICE_FLTR_RX);
	ice_fltr_add_mac_and_broadcast(uplink_vsi,
				       uplink_vsi->port_info->mac.perm_addr,
				       ICE_FWD_TO_VSI);
}

/**
 * ice_eswitch_enable_switchdev - configure eswitch in switchdev mode
 * @pf: pointer to PF structure
 */
static int ice_eswitch_enable_switchdev(struct ice_pf *pf)
{
	struct ice_vsi *uplink_vsi = ice_get_main_vsi(pf);

	if (!uplink_vsi)
		return -EINVAL;
	pf->eswitch.uplink_vsi = uplink_vsi;

	if (ice_eswitch_setup_env(pf))
		return -ENODEV;

	pf->eswitch.is_running = true;

	return 0;
}

/**
 * ice_eswitch_disable_switchdev - disable eswitch resources
 * @pf: pointer to PF structure
 */
static void ice_eswitch_disable_switchdev(struct ice_pf *pf)
{

	ice_eswitch_release_env(pf);

	pf->eswitch.is_running = false;
}

#ifdef HAVE_METADATA_PORT_INFO
#ifdef HAVE_DEVLINK_ESWITCH_OPS_EXTACK
/**
 * ice_eswitch_mode_set - set new eswitch mode
 * @devlink: pointer to devlink structure
 * @mode: eswitch mode to switch to
 * @extack: pointer to extack structure
 */
int
ice_eswitch_mode_set(struct devlink *devlink, u16 mode,
		     struct netlink_ext_ack *extack)
#else
int ice_eswitch_mode_set(struct devlink *devlink, u16 mode)
#endif /* HAVE_DEVLINK_ESWITCH_OPS_EXTACK */
{
	struct ice_pf *pf = devlink_priv(devlink);

	if (pf->eswitch_mode == mode)
		return 0;

	if (ice_has_vfs(pf)) {
		dev_info(ice_pf_to_dev(pf), "Changing eswitch mode is allowed only if there is no VFs created");
		return -EOPNOTSUPP;
	}

	switch (mode) {
	case DEVLINK_ESWITCH_MODE_LEGACY:
		dev_info(ice_pf_to_dev(pf), "PF %d changed eswitch mode to legacy",
			 pf->hw.pf_id);
		break;
	case DEVLINK_ESWITCH_MODE_SWITCHDEV:
	{
#ifdef NETIF_F_HW_TC
		if (ice_is_adq_active(pf)) {
			dev_err(ice_pf_to_dev(pf), "switchdev cannot be configured - ADQ is active. Delete ADQ configs using TC and try again\n");
			return -EOPNOTSUPP;
		}
#endif /* NETIF_F_HW_TC */

#ifdef HAVE_NDO_DFWD_OPS
		if (ice_is_offloaded_macvlan_ena(pf)) {
			dev_err(ice_pf_to_dev(pf), "switchdev cannot be configured -  L2 Forwarding Offload is currently enabled.\n");
			return -EOPNOTSUPP;
		}
#endif /* HAVE_NDO_DFWD_OPS */

		if (!test_bit(ICE_FLAG_ESWITCH_CAPABLE, pf->flags)) {
			dev_err(ice_pf_to_dev(pf), "switchdev cannot be configured - eswitch isn't supported in hw or there was not enough msix\n");
			return -EOPNOTSUPP;
		}

		dev_info(ice_pf_to_dev(pf), "PF %d changed eswitch mode to switchdev",
			 pf->hw.pf_id);
		INIT_RADIX_TREE(&pf->eswitch.reprs, GFP_KERNEL);
		break;
	}
	default:
#ifdef HAVE_DEVLINK_ESWITCH_OPS_EXTACK
		NL_SET_ERR_MSG_MOD(extack, "Unknown eswitch mode");
#else
		dev_err(ice_pf_to_dev(pf), "Unknown eswitch mode");
#endif /* HAVE_DEVLINK_ESWITCH_OPS_EXTACK */
		return -EINVAL;
	}

	pf->eswitch_mode = mode;
	return 0;
}
#endif /* HAVE_METADATA_PORT_INFO */

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
 * ice_eswitch_start_all_tx_queues - start Tx queues of all port representors
 * @pf: pointer to PF structure
 */
void ice_eswitch_start_all_tx_queues(struct ice_pf *pf)
{
	struct radix_tree_iter id;
	struct ice_repr *repr;
	void __rcu **slot;

	if (test_bit(ICE_DOWN, pf->state))
		return;

	rcu_read_lock();
	radix_tree_for_each_slot(slot, &pf->eswitch.reprs, &id, 0) {
		repr = (struct ice_repr *)radix_tree_deref_slot(slot);
		ice_repr_start_tx_queues(repr);
	}
	rcu_read_unlock();
}

/**
 * ice_eswitch_stop_all_tx_queues - stop Tx queues of all port representors
 * @pf: pointer to PF structure
 */
void ice_eswitch_stop_all_tx_queues(struct ice_pf *pf)
{
	struct radix_tree_iter id;
	struct ice_repr *repr;
	void __rcu **slot;

	if (test_bit(ICE_DOWN, pf->state))
		return;

	rcu_read_lock();
	radix_tree_for_each_slot(slot, &pf->eswitch.reprs, &id, 0) {
		repr = (struct ice_repr *)radix_tree_deref_slot(slot);
		ice_repr_stop_tx_queues(repr);
	}
	rcu_read_unlock();
}

static void ice_eswitch_stop_reprs(struct ice_pf *pf)
{
	ice_eswitch_stop_all_tx_queues(pf);
}

static void ice_eswitch_start_reprs(struct ice_pf *pf)
{
	ice_eswitch_start_all_tx_queues(pf);
}

int ice_eswitch_attach(struct ice_pf *pf, struct ice_vf *vf)
{
	struct ice_repr *repr;
	int err;

	if (pf->eswitch_mode == DEVLINK_ESWITCH_MODE_LEGACY)
		return 0;

	if (radix_tree_empty(&pf->eswitch.reprs)) {
		err = ice_eswitch_enable_switchdev(pf);
		if (err)
			return err;
	}

	ice_eswitch_stop_reprs(pf);

	repr = ice_repr_add_vf(vf);
	if (IS_ERR(repr)) {
		err = PTR_ERR(repr);
		goto err_create_repr;
	}

	err = ice_eswitch_setup_repr(pf, repr);
	if (err)
		goto err_setup_repr;

	err = radix_tree_insert(&pf->eswitch.reprs, repr->id, repr);
	if (err)
		goto err_xa_alloc;

	vf->repr_id = repr->id;

	ice_eswitch_start_reprs(pf);

	return 0;

err_xa_alloc:
	ice_eswitch_release_repr(pf, repr);
err_setup_repr:
	ice_repr_rem_vf(repr);
err_create_repr:
	if (radix_tree_empty(&pf->eswitch.reprs))
		ice_eswitch_disable_switchdev(pf);
	ice_eswitch_start_reprs(pf);

	return err;
}

void ice_eswitch_detach(struct ice_pf *pf, struct ice_vf *vf)
{
	struct ice_repr *repr = (struct ice_repr *)
		radix_tree_lookup(&pf->eswitch.reprs, vf->repr_id);
#ifdef HAVE_DEVLINK_RATE_NODE_CREATE
	struct devlink *devlink = priv_to_devlink(pf);
#endif /* HAVE_DEVLINK_RATE_NODE_CREATE */

	if (!repr)
		return;

	ice_eswitch_stop_reprs(pf);
	radix_tree_delete(&pf->eswitch.reprs, repr->id);

	if (radix_tree_empty(&pf->eswitch.reprs))
		ice_eswitch_disable_switchdev(pf);

	ice_eswitch_release_repr(pf, repr);
	ice_repr_rem_vf(repr);

#ifdef HAVE_DEVLINK_RATE_NODE_CREATE
	if (radix_tree_empty(&pf->eswitch.reprs)) {
		/* since all port representors are destroyed, there is
		 * no point in keeping the nodes
		 */
		ice_devlink_rate_clear_tx_topology(ice_get_main_vsi(pf));
		devl_lock(devlink);
		devl_rate_nodes_destroy(devlink);
		devl_unlock(devlink);
	} else {
		ice_eswitch_start_reprs(pf);
	}
#else
	if (!radix_tree_empty(&pf->eswitch.reprs))
		ice_eswitch_start_reprs(pf);
#endif /* HAVE_DEVLINK_RATE_NODE_CREATE */
}

/**
 * ice_eswitch_get_target - get netdev based on src_vsi from descriptor
 * @rx_ring: ring used to receive the packet
 * @rx_desc: descriptor used to get src_vsi value
 *
 * Get src_vsi value from descriptor and load correct representor. If it isn't
 * found return rx_ring->netdev.
 */
struct net_device *ice_eswitch_get_target(const struct ice_rx_ring *rx_ring,
					  union ice_32b_rx_flex_desc *rx_desc)
{
	struct ice_eswitch *eswitch = &rx_ring->vsi->back->eswitch;
	struct ice_32b_rx_flex_desc_nic_2 *desc;
	struct ice_repr *repr;

	desc = (struct ice_32b_rx_flex_desc_nic_2 *)rx_desc;
	repr = (struct ice_repr *)
		radix_tree_lookup(&eswitch->reprs, le16_to_cpu(desc->src_vsi));
	if (!repr)
		return rx_ring->netdev;

	return repr->netdev;
}
#endif /* CONFIG_NET_DEVLINK */
