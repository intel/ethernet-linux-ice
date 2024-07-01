/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2018-2024 Intel Corporation */

#include "ice.h"
#include "ice_eswitch.h"
#if IS_ENABLED(CONFIG_NET_DEVLINK)
#include "ice_devlink.h"
#endif /* CONFIG_NET_DEVLINK */
#include "ice_sriov.h"
#include "ice_tc_lib.h"
#include "ice_lib.h"
#ifdef HAVE_DEVLINK_RATE_NODE_CREATE
#include "ice_dcb_lib.h"
#endif /* HAVE_DEVLINK_RATE_NODE_CREATE */

#ifdef HAVE_NDO_GET_PHYS_PORT_NAME
/**
 * ice_repr_get_sw_port_id - get port ID associated with representor
 * @repr: pointer to port representor
 */
static int ice_repr_get_sw_port_id(struct ice_repr *repr)
{
	return repr->vf->pf->hw.port_info->lport;
}

/**
 * ice_repr_get_phys_port_name - get phys port name
 * @netdev: pointer to port representor netdev
 * @buf: write here port name
 * @len: max length of buf
 */
static int
ice_repr_get_phys_port_name(struct net_device *netdev, char *buf, size_t len)
{
	struct ice_netdev_priv *np = netdev_priv(netdev);
	struct ice_repr *repr = np->repr;
	int res;

#if IS_ENABLED(CONFIG_NET_DEVLINK)
	/* Devlink port is registered and devlink core is taking care of name formatting. */
	if (repr->vf->devlink_port.devlink)
		return -EOPNOTSUPP;
#endif /* CONFIG_NET_DEVLINK */

	res = snprintf(buf, len, "pf%dvfr%d", ice_repr_get_sw_port_id(repr),
		       repr->vf->vf_id);
	if (res <= 0)
		return -EOPNOTSUPP;
	return 0;
}
#endif /* HAVE_NDO_GET_PHYS_PORT_NAME */

/**
 * ice_repr_get_stats64 - get VF stats for VFPR use
 * @netdev: pointer to port representor netdev
 * @stats: pointer to struct where stats can be stored
 */
#ifdef HAVE_VOID_NDO_GET_STATS64
static void
#else
static struct rtnl_link_stats64 *
#endif
ice_repr_get_stats64(struct net_device *netdev, struct rtnl_link_stats64 *stats)
{
	struct ice_netdev_priv *np = netdev_priv(netdev);
	struct ice_eth_stats *eth_stats;
	struct ice_vsi *vsi;

	if (ice_is_vf_disabled(np->repr->vf))
#ifdef HAVE_VOID_NDO_GET_STATS64
		return;
#else
		return stats;
#endif
	vsi = np->repr->src_vsi;

	ice_update_vsi_stats(vsi);
	eth_stats = &vsi->eth_stats;

	stats->tx_packets = eth_stats->tx_unicast + eth_stats->tx_broadcast +
			    eth_stats->tx_multicast;
	stats->rx_packets = eth_stats->rx_unicast + eth_stats->rx_broadcast +
			    eth_stats->rx_multicast;
	stats->tx_bytes = eth_stats->tx_bytes;
	stats->rx_bytes = eth_stats->rx_bytes;
	stats->multicast = eth_stats->rx_multicast;
	stats->tx_errors = eth_stats->tx_errors;
	stats->tx_dropped = eth_stats->tx_discards;
	stats->rx_dropped = eth_stats->rx_discards;
#ifndef HAVE_VOID_NDO_GET_STATS64

	return stats;
#endif
}

/**
 * ice_netdev_to_repr - Get port representor for given netdevice
 * @netdev: pointer to port representor netdev
 */
struct ice_repr *ice_netdev_to_repr(struct net_device *netdev)
{
	struct ice_netdev_priv *np = netdev_priv(netdev);

	return np->repr;
}

/**
 * ice_repr_open - Enable port representor's network interface
 * @netdev: network interface device structure
 *
 * The open entry point is called when a port representor's network
 * interface is made active by the system (IFF_UP). Corresponding
 * VF is notified about link status change.
 *
 * Returns 0 on success
 */
static int ice_repr_open(struct net_device *netdev)
{
	struct ice_repr *repr = ice_netdev_to_repr(netdev);
	struct ice_vf *vf;

	vf = repr->vf;
	vf->link_forced = true;
	vf->link_up = true;
	ice_vc_notify_vf_link_state(vf);

	netif_carrier_on(netdev);
	netif_tx_start_all_queues(netdev);

	return 0;
}

/**
 * ice_repr_stop - Disable port representor's network interface
 * @netdev: network interface device structure
 *
 * The stop entry point is called when a port representor's network
 * interface is de-activated by the system. Corresponding
 * VF is notified about link status change.
 *
 * Returns 0 on success
 */
static int ice_repr_stop(struct net_device *netdev)
{
	struct ice_repr *repr = ice_netdev_to_repr(netdev);
	struct ice_vf *vf;

	vf = repr->vf;
	vf->link_forced = true;
	vf->link_up = false;
	ice_vc_notify_vf_link_state(vf);

	netif_carrier_off(netdev);
	netif_tx_stop_all_queues(netdev);

	return 0;
}

#if IS_ENABLED(CONFIG_NET_DEVLINK) && defined(HAVE_DEVLINK_PORT_ATTR_PCI_VF)
#ifdef HAVE_NDO_GET_DEVLINK_PORT
#ifndef HAVE_SET_NETDEV_DEVLINK_PORT
static struct devlink_port *
ice_repr_get_devlink_port(struct net_device *netdev)
{
	struct ice_repr *repr = ice_netdev_to_repr(netdev);

	return &repr->vf->devlink_port;
}
#endif /* !HAVE_SET_NETDEV_DEVLINK_PORT */
#endif /* HAVE_NDO_GET_DEVLINK_PORT */
#endif /* CONFIG_NET_DEVLINK && HAVE_DEVLINK_PORT_ATTR_PCI_VF */
#if defined(HAVE_NDO_OFFLOAD_STATS) || defined(HAVE_RHEL7_EXTENDED_OFFLOAD_STATS)
/**
 * ice_repr_sp_stats64 - get slow path stats for port representor
 * @dev: network interface device structure
 * @stats: netlink stats structure
 *
 * RX/TX stats are being swapped here to be consistent with VF stats. In slow
 * path, port representor receives data when the corresponding VF is sending it
 * (and vice versa), TX and RX bytes/packets are effectively swapped on port
 * representor.
 */
static int
ice_repr_sp_stats64(const struct net_device *dev,
		    struct rtnl_link_stats64 *stats)
{
	struct ice_netdev_priv *np = netdev_priv(dev);
	int vf_id = np->repr->vf->vf_id;
	struct ice_tx_ring *tx_ring;
	struct ice_rx_ring *rx_ring;
	u64 pkts, bytes;

	tx_ring = np->vsi->tx_rings[vf_id];
	ice_fetch_u64_stats_per_ring(&tx_ring->ring_stats->syncp,
				     tx_ring->ring_stats->stats,
				     &pkts, &bytes);
	stats->rx_packets = pkts;
	stats->rx_bytes = bytes;

	rx_ring = np->vsi->rx_rings[vf_id];
	ice_fetch_u64_stats_per_ring(&rx_ring->ring_stats->syncp,
				     rx_ring->ring_stats->stats,
				     &pkts, &bytes);
	stats->tx_packets = pkts;
	stats->tx_bytes = bytes;
	stats->tx_dropped = rx_ring->ring_stats->rx_stats.alloc_page_failed +
			    rx_ring->ring_stats->rx_stats.alloc_buf_failed;

	return 0;
}

static bool
ice_repr_ndo_has_offload_stats(const struct net_device *dev, int attr_id)
{
	return attr_id == IFLA_OFFLOAD_XSTATS_CPU_HIT;
}

static int
ice_repr_ndo_get_offload_stats(int attr_id, const struct net_device *dev,
			       void *sp)
{
	if (attr_id == IFLA_OFFLOAD_XSTATS_CPU_HIT)
		return ice_repr_sp_stats64(dev, (struct rtnl_link_stats64 *)sp);

	return -EINVAL;
}
#endif /* HAVE_NDO_OFFLOAD_STATS || HAVE_RHEL7_EXTENDED_OFFLOAD_STATS */

#ifdef HAVE_TC_SETUP_CLSFLOWER
static int
ice_repr_setup_tc_cls_flower(struct ice_repr *repr,
			     struct flow_cls_offload *flower)
{
	switch (flower->command) {
	case FLOW_CLS_REPLACE:
		return ice_add_cls_flower(repr->netdev, repr->src_vsi, flower);
	case FLOW_CLS_DESTROY:
		return ice_del_cls_flower(repr->src_vsi, flower);
	default:
		return -EINVAL;
	}
}
#ifdef HAVE_TC_CB_AND_SETUP_QDISC_MQPRIO
static int
ice_repr_setup_tc_block_cb(enum tc_setup_type type, void *type_data,
			   void *cb_priv)
{
	struct flow_cls_offload *flower = type_data;
	struct ice_netdev_priv *np = cb_priv;

	switch (type) {
	case TC_SETUP_CLSFLOWER:
		return ice_repr_setup_tc_cls_flower(np->repr, flower);
	default:
		return -EOPNOTSUPP;
	}
}

static LIST_HEAD(ice_repr_block_cb_list);
#endif /* HAVE_TC_CB_AND_SETUP_QDISC_MQPRIO */

static int
#ifdef HAVE_NDO_SETUP_TC_REMOVE_TC_TO_NETDEV
ice_repr_setup_tc(struct net_device *netdev, enum tc_setup_type type,
		  void *type_data)
#elif defined(HAVE_NDO_SETUP_TC_CHAIN_INDEX)
ice_repr_setup_tc(struct net_device *netdev, u32 __always_unused handle,
		  __always_unused chain_index, __be16 proto,
		  struct tc_to_netdev *tc)
#else
ice_repr_setup_tc(struct net_device *netdev, u32 __always_unused handle,
		  __be16 __always_unused proto, struct tc_to_netdev *tc)
#endif
{
#ifndef HAVE_NDO_SETUP_TC_REMOVE_TC_TO_NETDEV
	struct tc_cls_flower_offload *cls_flower = tc->cls_flower;
	unsigned int type = tc->type;
#elif !defined(HAVE_TC_CB_AND_SETUP_QDISC_MQPRIO)
	struct tc_cls_flower_offload *cls_flower = type_data;
#endif /* HAVE_NDO_SETUP_TC_REMOVE_TC_TO_NETDEV */
	struct ice_netdev_priv *np = netdev_priv(netdev);

	switch (type) {
#ifdef HAVE_TC_CB_AND_SETUP_QDISC_MQPRIO
	case TC_SETUP_BLOCK:
		return flow_block_cb_setup_simple((struct flow_block_offload *)
						  type_data,
						  &ice_repr_block_cb_list,
						  ice_repr_setup_tc_block_cb,
						  np, np, true);
#elif !defined(HAVE_NDO_SETUP_TC_REMOVE_TC_TO_NETDEV) || !defined(HAVE_TC_CB_AND_SETUP_QDISC_MQPRIO)
	case TC_SETUP_CLSFLOWER:
		return ice_repr_setup_tc_cls_flower(np->repr, cls_flower);
#endif /* HAVE_TC_CB_AND_SETUP_QDISC_MQPRIO */
	default:
		return -EOPNOTSUPP;
	}
}
#endif /* HAVE_TC_SETUP_CLSFLOWER */

#ifndef HAVE_NETDEV_MIN_MAX_MTU
/**
 * ice_repr_change_mtu - NDO callback to change the MTU on port representor
 * @netdev: network interface device structure
 * @new_mtu: new value for MTU
 *
 * Returns 0 on success, negative on failure
 */
static int ice_repr_change_mtu(struct net_device *netdev, int new_mtu)
{
	int err;

	err = ice_check_mtu_valid(netdev, new_mtu);
	if (err)
		return err;

	netdev->mtu = (unsigned int)new_mtu;

	return 0;
}
#endif /* !HAVE_NETDEV_MIN_MAX_MTU */

static const struct net_device_ops ice_repr_netdev_ops = {
#ifdef HAVE_NDO_GET_PHYS_PORT_NAME
	.ndo_get_phys_port_name = ice_repr_get_phys_port_name,
#endif /* HAVE_NDO_GET_PHYS_PORT_NAME */
	.ndo_get_stats64 = ice_repr_get_stats64,
	.ndo_open = ice_repr_open,
	.ndo_stop = ice_repr_stop,
#if IS_ENABLED(CONFIG_NET_DEVLINK)
	.ndo_start_xmit = ice_eswitch_port_start_xmit,
#ifndef HAVE_NETDEV_MIN_MAX_MTU
#ifdef HAVE_NETDEV_EXTENDED_MIN_MAX_MTU
	.extended.ndo_change_mtu = ice_repr_change_mtu,
#else
	.ndo_change_mtu = ice_repr_change_mtu,
#endif /* HAVE_NETDEV_EXTENDED_MIN_MAX_MTU */
#endif /* HAVE_NETDEV_MIN_MAX_MTU */
#ifdef HAVE_DEVLINK_PORT_ATTR_PCI_VF
#ifdef HAVE_NDO_GET_DEVLINK_PORT
#ifndef HAVE_SET_NETDEV_DEVLINK_PORT
	.ndo_get_devlink_port = ice_repr_get_devlink_port,
#endif /* !HAVE_SET_NETDEV_DEVLINK_PORT */
#endif /* HAVE_NDO_GET_DEVLINK_PORT */
#endif /* HAVE_DEVLINK_PORT_ATTR_PCI_VF */
#ifdef HAVE_TC_SETUP_CLSFLOWER
#ifdef HAVE_RHEL7_NETDEV_OPS_EXT_NDO_SETUP_TC
	.extended.ndo_setup_tc_rh = ice_repr_setup_tc,
#else
	.ndo_setup_tc = ice_repr_setup_tc,
#endif /* HAVE_RHEL7_NETDEV_OPS_EXT_NDO_SETUP_TC */
#endif /* HAVE_TC_SETUP_CLSFLOWER */
#endif /* CONFIG_NET_DEVLINK */
#ifdef HAVE_NDO_OFFLOAD_STATS
	.ndo_has_offload_stats = ice_repr_ndo_has_offload_stats,
	.ndo_get_offload_stats = ice_repr_ndo_get_offload_stats,
#elif defined(HAVE_RHEL7_EXTENDED_OFFLOAD_STATS)
	.extended.ndo_has_offload_stats = ice_repr_ndo_has_offload_stats,
	.extended.ndo_get_offload_stats = ice_repr_ndo_get_offload_stats,
#endif
};

/**
 * ice_is_port_repr_netdev - Check if a given netdevice is a port representor
 * netdev
 * @netdev: pointer to netdev
 */
bool ice_is_port_repr_netdev(struct net_device *netdev)
{
	return netdev && (netdev->netdev_ops == &ice_repr_netdev_ops);
}

/**
 * ice_repr_reg_netdev - register port representor netdev
 * @netdev: pointer to port representor netdev
 */
static int
ice_repr_reg_netdev(struct net_device *netdev)
{
	eth_hw_addr_random(netdev);
	netdev->netdev_ops = &ice_repr_netdev_ops;
	ice_set_ethtool_repr_ops(netdev);

#ifdef NETIF_F_HW_TC
	netdev->hw_features |= NETIF_F_HW_TC;
#endif /* NETIF_F_HW_TC */

	netif_carrier_off(netdev);
	netif_tx_stop_all_queues(netdev);

	return register_netdev(netdev);
}

/**
 * ice_repr_add - add representor for VF
 * @vf: pointer to VF structure
 */
static int ice_repr_add(struct ice_vf *vf)
{
	struct ice_q_vector *q_vector;
	struct ice_netdev_priv *np;
	struct ice_repr *repr;
	struct ice_vsi *vsi;
	int err;

	vsi = ice_get_vf_vsi(vf);
	if (!vsi)
		return -EINVAL;

	repr = kzalloc(sizeof(*repr), GFP_KERNEL);
	if (!repr)
		return -ENOMEM;

	repr->netdev = alloc_etherdev(sizeof(struct ice_netdev_priv));
	if (!repr->netdev) {
		err =  -ENOMEM;
		goto err_alloc;
	}

	repr->src_vsi = vsi;
	repr->vf = vf;
	vf->repr = repr;
	np = netdev_priv(repr->netdev);
	np->repr = repr;

	q_vector = kzalloc(sizeof(*q_vector), GFP_KERNEL);
	if (!q_vector) {
		err = -ENOMEM;
		goto err_alloc_q_vector;
	}
	repr->q_vector = q_vector;

#if IS_ENABLED(CONFIG_NET_DEVLINK)
#ifdef HAVE_DEVLINK_PORT_ATTR_PCI_VF
	err = ice_devlink_create_vf_port(vf);
	if (err)
		goto err_devlink;
#endif /* HAVE_DEVLINK_PORT_ATTR_PCI_VF */

#ifdef HAVE_NETDEV_EXTENDED_MIN_MAX_MTU
	repr->netdev->extended->min_mtu = ETH_MIN_MTU;
	repr->netdev->extended->max_mtu = ICE_MAX_MTU;
#endif /* HAVE_NETDEV_EXTENDED_MIN_MAX_MTU */
#ifdef HAVE_NETDEV_MIN_MAX_MTU
	repr->netdev->min_mtu = ETH_MIN_MTU;
	repr->netdev->max_mtu = ICE_MAX_MTU;
#endif /* HAVE_NETDEV_MIN_MAX_MTU */
#endif /* CONFIG_NET_DEVLINK */

	SET_NETDEV_DEV(repr->netdev, ice_pf_to_dev(vf->pf));
#if IS_ENABLED(CONFIG_NET_DEVLINK)
#ifdef HAVE_DEVLINK_PORT_ATTR_PCI_VF
#ifdef HAVE_SET_NETDEV_DEVLINK_PORT
	SET_NETDEV_DEVLINK_PORT(repr->netdev, &vf->devlink_port);
#endif /* HAVE_SET_NETDEV_DEVLINK_PORT */
#endif /* HAVE_DEVLINK_PORT_ATTR_PCI_VF */
#endif /* CONFIG_NET_DEVLINK */
	err = ice_repr_reg_netdev(repr->netdev);
	if (err)
		goto err_netdev;

#if IS_ENABLED(CONFIG_NET_DEVLINK)
#ifdef HAVE_DEVLINK_PORT_ATTR_PCI_VF
#ifndef HAVE_SET_NETDEV_DEVLINK_PORT
	devlink_port_type_eth_set(&vf->devlink_port, repr->netdev);
#endif /* !HAVE_SET_NETDEV_DEVLINK_PORT */
#endif /* HAVE_DEVLINK_PORT_ATTR_PCI_VF */
#endif /* CONFIG_NET_DEVLINK */

	ice_virtchnl_set_repr_ops(vf);

	return 0;

err_netdev:
#if IS_ENABLED(CONFIG_NET_DEVLINK)
#ifdef HAVE_DEVLINK_PORT_ATTR_PCI_VF
	ice_devlink_destroy_vf_port(vf);
err_devlink:
#endif /* HAVE_DEVLINK_PORT_ATTR_PCI_VF */
#endif /* CONFIG_NET_DEVLINK */
	kfree(repr->q_vector);
	vf->repr->q_vector = NULL;
err_alloc_q_vector:
	free_netdev(repr->netdev);
	repr->netdev = NULL;
err_alloc:
	kfree(repr);
	vf->repr = NULL;
	return err;
}

/**
 * ice_repr_rem - remove representor from VF
 * @vf: pointer to VF structure
 */
static void ice_repr_rem(struct ice_vf *vf)
{
	if (!vf->repr)
		return;

	kfree(vf->repr->q_vector);
	vf->repr->q_vector = NULL;
	unregister_netdev(vf->repr->netdev);
#if IS_ENABLED(CONFIG_NET_DEVLINK)
#ifdef HAVE_DEVLINK_PORT_ATTR_PCI_VF
	ice_devlink_destroy_vf_port(vf);
#endif /* HAVE_DEVLINK_PORT_ATTR_PCI_VF */
#endif /* CONFIG_NET_DEVLINK */
	free_netdev(vf->repr->netdev);
	vf->repr->netdev = NULL;
	kfree(vf->repr);
	vf->repr = NULL;

	ice_virtchnl_set_dflt_ops(vf);
}

/**
 * ice_repr_rem_from_all_vfs - remove port representor for all VFs
 * @pf: pointer to PF structure
 */
void ice_repr_rem_from_all_vfs(struct ice_pf *pf)
{
#ifdef HAVE_DEVLINK_RATE_NODE_CREATE
	struct devlink *devlink;
#endif /* HAVE_DEVLINK_RATE_NODE_CREATE */
	struct ice_vf *vf;
	unsigned int bkt;

	lockdep_assert_held(&pf->vfs.table_lock);

	ice_for_each_vf(pf, bkt, vf)
		ice_repr_rem(vf);

#ifdef HAVE_DEVLINK_RATE_NODE_CREATE
	/* since all port representors are destroyed, there is
	 * no point in keeping the nodes
	 */
	devlink = priv_to_devlink(pf);
	devl_lock(devlink);
	devl_rate_nodes_destroy(devlink);
	devl_unlock(devlink);
#endif /* HAVE_DEVLINK_RATE_NODE_CREATE */
}

/**
 * ice_repr_add_for_all_vfs - add port representor for all VFs
 * @pf: pointer to PF structure
 */
int ice_repr_add_for_all_vfs(struct ice_pf *pf)
{
#if IS_ENABLED(CONFIG_NET_DEVLINK)
#ifdef HAVE_DEVLINK_RATE_NODE_CREATE
	struct devlink *devlink;
#endif /* HAVE_DEVLINK_RATE_NODE_CREATE */
#endif /* CONFIG_NET_DEVLINK */
	struct ice_vf *vf;
	unsigned int bkt;
	int err;

	lockdep_assert_held(&pf->vfs.table_lock);

	ice_for_each_vf(pf, bkt, vf) {
		err = ice_repr_add(vf);
		if (err)
			goto err;
	}

#if IS_ENABLED(CONFIG_NET_DEVLINK)
#ifdef HAVE_DEVLINK_RATE_NODE_CREATE
	/* only export if RDMA and DCB disabled */
	if (ice_is_aux_ena(pf) &&
	    ice_is_rdma_aux_loaded(pf))
		return 0;
	if (ice_is_dcb_active(pf))
		return 0;
	devlink = priv_to_devlink(pf);
	ice_devlink_rate_init_tx_topology(devlink, ice_get_main_vsi(pf));
#endif /* HAVE_DEVLINK_RATE_NODE_CREATE */
#endif /* CONFIG_NET_DEVLINK */

	return 0;

err:
	ice_repr_rem_from_all_vfs(pf);

	return err;
}

/**
 * ice_repr_start_tx_queues - start Tx queues of port representor
 * @repr: pointer to repr structure
 */
void ice_repr_start_tx_queues(struct ice_repr *repr)
{
	netif_carrier_on(repr->netdev);
	netif_tx_start_all_queues(repr->netdev);
}

/**
 * ice_repr_stop_tx_queues - stop Tx queues of port representor
 * @repr: pointer to repr structure
 */
void ice_repr_stop_tx_queues(struct ice_repr *repr)
{
	netif_carrier_off(repr->netdev);
	netif_tx_stop_all_queues(repr->netdev);
}

#ifdef HAVE_METADATA_PORT_INFO
/**
 * ice_repr_set_traffic_vsi - set traffic VSI for port representor
 * @repr: repr on with VSI will be set
 * @vsi: pointer to VSI that will be used by port representor to pass traffic
 */
void ice_repr_set_traffic_vsi(struct ice_repr *repr, struct ice_vsi *vsi)
{
	struct ice_netdev_priv *np = netdev_priv(repr->netdev);

	np->vsi = vsi;
}
#endif /* HAVE_METADATA_PORT_INFO */
