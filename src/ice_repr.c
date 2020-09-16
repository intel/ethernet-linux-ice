// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2018-2019, Intel Corporation. */

#include "ice_repr.h"
#include "ice_eswitch.h"
#include "ice_virtchnl_pf.h"

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
	struct ice_vsi *vsi = np->repr->src_vsi;
	struct ice_eth_stats *eth_stats;

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
#ifdef HAVE_METADATA_PORT_INFO
struct ice_repr *ice_netdev_to_repr(struct net_device *netdev)
#else
static struct ice_repr *ice_netdev_to_repr(struct net_device *netdev)
#endif /* HAVE_METADATA_PORT_INFO */
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
 * Returns 0 on success, negative value on failure
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
 * Returns 0 on success, negative value on failure
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

static const struct net_device_ops ice_repr_netdev_ops = {
#ifdef HAVE_NDO_GET_PHYS_PORT_NAME
	.ndo_get_phys_port_name = ice_repr_get_phys_port_name,
#endif /* HAVE_NDO_GET_PHYS_PORT_NAME */
	.ndo_get_stats64 = ice_repr_get_stats64,
	.ndo_open = ice_repr_open,
	.ndo_stop = ice_repr_stop,
#if IS_ENABLED(CONFIG_NET_DEVLINK)
	.ndo_start_xmit = ice_eswitch_port_start_xmit,
#endif /* CONFIG_NET_DEVLINK */
};

/**
 * ice_repr_reg_netdev - register port representor netdev
 * @netdev: pointer to port representor netdev
 */
static int
ice_repr_reg_netdev(struct net_device *netdev)
{
	eth_hw_addr_random(netdev);
	netdev->netdev_ops = &ice_repr_netdev_ops;

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
	struct ice_netdev_priv *np;
	struct ice_repr *repr;
	int err;

	repr = kzalloc(sizeof(*repr), GFP_KERNEL);
	if (!repr)
		return -ENOMEM;

	repr->netdev = alloc_etherdev(sizeof(struct ice_netdev_priv));
	if (!repr->netdev) {
		err =  -ENOMEM;
		goto err_alloc;
	}

	repr->src_vsi = vf->pf->vsi[vf->lan_vsi_idx];
	repr->vf = vf;
	vf->repr = repr;
	np = netdev_priv(repr->netdev);
	np->repr = repr;

	err = ice_repr_reg_netdev(repr->netdev);
	if (err)
		goto err_netdev;

	return 0;

err_netdev:
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
	unregister_netdev(vf->repr->netdev);
	free_netdev(vf->repr->netdev);
	vf->repr->netdev = NULL;
	kfree(vf->repr);
	vf->repr = NULL;
}


/**
 * ice_repr_add_for_all_vfs - add port representor for all VFs
 * @pf: pointer to PF structure
 */
int ice_repr_add_for_all_vfs(struct ice_pf *pf)
{
	int err;
	int i;


	ice_vc_change_ops_to_repr();
	ice_for_each_vf(pf, i) {
		err = ice_repr_add(&pf->vf[i]);
		if (err)
			goto err;
	}
	return 0;

err:
	for (i = i - 1; i >= 0; i--)
		ice_repr_rem(&pf->vf[i]);

	return err;
}

/**
 * ice_repr_rem_from_all_vfs - remove port representor for all VFs
 * @pf: pointer to PF structure
 */
void ice_repr_rem_from_all_vfs(struct ice_pf *pf)
{
	int i;


	ice_for_each_vf(pf, i)
		ice_repr_rem(&pf->vf[i]);

	ice_vc_change_ops_to_default();
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
