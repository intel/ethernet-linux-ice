// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2018-2019, Intel Corporation. */

/* ACL support for ice */


#include "ice.h"
#include "ice_lib.h"
#include "ice_flow.h"
#include "ice_fdir.h"

/* Default ACL Action priority */
#define ICE_ACL_ACT_PRIO	3

/* Number of action */
#define ICE_ACL_NUM_ACT		1

/**
 * ice_acl_set_input_set - Set the input set for ACL
 * @vsi: pointer to target VSI
 * @fsp: pointer to ethtool Rx flow specification
 * @input: filter structure
 *
 * Return error value or 0 on success
 */
static int
ice_acl_set_input_set(struct ice_vsi *vsi, struct ethtool_rx_flow_spec *fsp,
		      struct ice_fdir_fltr *input)
{
	u16 dest_vsi, q_index = 0;
	u16 orig_q_index = 0;
	struct ice_pf *pf;
	struct ice_hw *hw;
	int flow_type;
	u8 dest_ctl;
#ifdef HAVE_ETHTOOL_FLOW_UNION_IP6_SPEC
	int idx;
#endif /* HAVE_ETHTOOL_FLOW_UNION_IP6_SPEC */

	if (!fsp || !input)
		return -EINVAL;


	pf = vsi->back;
	hw = &pf->hw;

	dest_vsi = vsi->idx;
	if (fsp->ring_cookie == RX_CLS_FLOW_DISC) {
		dest_ctl = ICE_FLTR_PRGM_DESC_DEST_DROP_PKT;
	} else {
		u32 ring = ethtool_get_flow_spec_ring(fsp->ring_cookie);
		u8 vf = ethtool_get_flow_spec_ring_vf(fsp->ring_cookie);

		if (!vf) {
			if (ring >= vsi->num_rxq)
				return -EINVAL;
			orig_q_index = ring;
			ice_update_ring_dest_vsi(vsi, &dest_vsi, &ring);
		} else {
			dev_err(ice_pf_to_dev(pf), "Failed to add filter. Flow director filters are not supported on VF queues.\n");
			return -EINVAL;
		}
		dest_ctl = ICE_FLTR_PRGM_DESC_DEST_DIRECT_PKT_QINDEX;
		q_index = ring;
	}

	input->fltr_id = fsp->location;
	input->q_index = q_index;

	/* Record the original queue as specified by user, because
	 * due to channel, configuration 'q_index' gets adjusted
	 * accordingly, but to keep user experience same - queue of
	 * flow-director filter shall report original queue number
	 * as specified by user, hence record it and use it later
	 */
	input->orig_q_index = orig_q_index;
	input->dest_vsi = dest_vsi;
	input->dest_ctl = dest_ctl;
	input->fltr_status = ICE_FLTR_PRGM_DESC_FD_STATUS_FD_ID;
	input->cnt_index = ICE_FD_SB_STAT_IDX(hw->fd_ctr_base);
	flow_type = fsp->flow_type & ~FLOW_MAC_EXT;
	input->flow_type = ice_ethtool_flow_to_fltr(flow_type);

	if (fsp->flow_type & FLOW_EXT) {
		memcpy(input->ext_data.usr_def, fsp->h_ext.data,
		       sizeof(input->ext_data.usr_def));
		input->ext_data.vlan_type = fsp->h_ext.vlan_etype;
		input->ext_data.vlan_tag = fsp->h_ext.vlan_tci;
		memcpy(input->ext_mask.usr_def, fsp->m_ext.data,
		       sizeof(input->ext_mask.usr_def));
		input->ext_mask.vlan_type = fsp->m_ext.vlan_etype;
		input->ext_mask.vlan_tag = fsp->m_ext.vlan_tci;
	}

	switch (flow_type) {
	case TCP_V4_FLOW:
	case UDP_V4_FLOW:
	case SCTP_V4_FLOW:
		input->ip.v4.dst_port = fsp->h_u.tcp_ip4_spec.pdst;
		input->ip.v4.src_port = fsp->h_u.tcp_ip4_spec.psrc;
		input->ip.v4.dst_ip = fsp->h_u.tcp_ip4_spec.ip4dst;
		input->ip.v4.src_ip = fsp->h_u.tcp_ip4_spec.ip4src;
		input->mask.v4.dst_port = fsp->m_u.tcp_ip4_spec.pdst;
		input->mask.v4.src_port = fsp->m_u.tcp_ip4_spec.psrc;
		input->mask.v4.dst_ip = fsp->m_u.tcp_ip4_spec.ip4dst;
		input->mask.v4.src_ip = fsp->m_u.tcp_ip4_spec.ip4src;
		break;
	case IPV4_USER_FLOW:
		input->ip.v4.dst_ip = fsp->h_u.usr_ip4_spec.ip4dst;
		input->ip.v4.src_ip = fsp->h_u.usr_ip4_spec.ip4src;
		input->ip.v4.l4_header = fsp->h_u.usr_ip4_spec.l4_4_bytes;
		input->ip.v4.proto = fsp->h_u.usr_ip4_spec.proto;
		input->ip.v4.ip_ver = fsp->h_u.usr_ip4_spec.ip_ver;
		input->ip.v4.tos = fsp->h_u.usr_ip4_spec.tos;
		input->mask.v4.dst_ip = fsp->m_u.usr_ip4_spec.ip4dst;
		input->mask.v4.src_ip = fsp->m_u.usr_ip4_spec.ip4src;
		input->mask.v4.l4_header = fsp->m_u.usr_ip4_spec.l4_4_bytes;
		input->mask.v4.proto = fsp->m_u.usr_ip4_spec.proto;
		input->mask.v4.ip_ver = fsp->m_u.usr_ip4_spec.ip_ver;
		input->mask.v4.tos = fsp->m_u.usr_ip4_spec.tos;
		break;
#ifdef HAVE_ETHTOOL_FLOW_UNION_IP6_SPEC
	case TCP_V6_FLOW:
	case UDP_V6_FLOW:
	case SCTP_V6_FLOW:
		for (idx = 0; idx < ICE_IPV6_ADDR_LEN_AS_U32; idx++) {
			input->ip.v6.dst_ip[idx] =
					fsp->h_u.tcp_ip6_spec.ip6dst[idx];
			input->ip.v6.src_ip[idx] =
					fsp->h_u.tcp_ip6_spec.ip6src[idx];
		}
		input->ip.v6.dst_port = fsp->h_u.tcp_ip6_spec.pdst;
		input->ip.v6.src_port = fsp->h_u.tcp_ip6_spec.psrc;
		input->ip.v6.tc = fsp->h_u.tcp_ip6_spec.tclass;
		memcpy(input->mask.v6.dst_ip, fsp->m_u.tcp_ip6_spec.ip6dst,
		       sizeof(input->mask.v6.dst_ip));
		memcpy(input->mask.v6.src_ip, fsp->m_u.tcp_ip6_spec.ip6src,
		       sizeof(input->mask.v6.src_ip));
		input->mask.v6.dst_port = fsp->m_u.tcp_ip6_spec.pdst;
		input->mask.v6.src_port = fsp->m_u.tcp_ip6_spec.psrc;
		input->mask.v6.tc = fsp->m_u.tcp_ip6_spec.tclass;
		break;
	case IPV6_USER_FLOW:
		memcpy(input->ip.v6.dst_ip, fsp->h_u.usr_ip6_spec.ip6dst,
		       sizeof(input->ip.v6.dst_ip));
		memcpy(input->ip.v6.src_ip, fsp->h_u.usr_ip6_spec.ip6src,
		       sizeof(input->ip.v6.src_ip));
		input->ip.v6.l4_header = fsp->h_u.usr_ip6_spec.l4_4_bytes;
		input->ip.v6.tc = fsp->h_u.usr_ip6_spec.tclass;
		input->ip.v6.proto = fsp->h_u.usr_ip6_spec.l4_proto;
		memcpy(input->mask.v6.dst_ip, fsp->m_u.usr_ip6_spec.ip6dst,
		       sizeof(input->mask.v6.dst_ip));
		memcpy(input->mask.v6.src_ip, fsp->m_u.usr_ip6_spec.ip6src,
		       sizeof(input->mask.v6.src_ip));
		input->mask.v6.l4_header = fsp->m_u.usr_ip6_spec.l4_4_bytes;
		input->mask.v6.tc = fsp->m_u.usr_ip6_spec.tclass;
		input->mask.v6.proto = fsp->m_u.usr_ip6_spec.l4_proto;
		break;
#endif /* HAVE_ETHTOOL_FLOW_UNION_IP6_SPEC */
	default:
		return -EINVAL;
	}

	return 0;
}

/**
 * ice_acl_check_input_set - Checks that a given ACL input set is valid
 * @pf: ice PF structure
 * @fsp: pointer to ethtool Rx flow specification
 *
 * Returns 0 on success and negative values for failure
 */
static int
ice_acl_check_input_set(struct ice_pf *pf, struct ethtool_rx_flow_spec *fsp)
{
	struct ethtool_tcpip4_spec *tcp_ip4_spec;
	struct ethtool_usrip4_spec *usr_ip4_spec;
#ifdef HAVE_ETHTOOL_FLOW_UNION_IP6_SPEC
	struct ethtool_tcpip6_spec *tcp_ip6_spec;
	struct ethtool_usrip6_spec *usr_ip6_spec;
#endif
	struct ice_fd_hw_prof *hw_prof = NULL;
	struct ice_flow_prof *prof = NULL;
	struct ice_flow_seg_info *old_seg;
	struct ice_flow_seg_info *seg;
	enum ice_fltr_ptype fltr_type;
	struct ice_hw *hw = &pf->hw;
	enum ice_status status;
	u16 val_loc, mask_loc;
	struct device *dev;
	int err;


	if (!fsp)
		return -EINVAL;

	dev = ice_pf_to_dev(pf);
	seg = devm_kzalloc(dev, sizeof(*seg), GFP_KERNEL);
	if (!seg)
		return -ENOMEM;

	switch (fsp->flow_type & ~FLOW_EXT) {
	case TCP_V4_FLOW:
	case UDP_V4_FLOW:
	case SCTP_V4_FLOW:
		tcp_ip4_spec = &fsp->m_u.tcp_ip4_spec;

		err = ice_check_ip4_seg_fltr(tcp_ip4_spec);
		if (err)
			goto err_exit;

		/* IP source address */
		val_loc = offsetof(struct ice_fdir_fltr, ip.v4.src_ip);
		mask_loc = offsetof(struct ice_fdir_fltr, mask.v4.src_ip);

		ice_flow_set_fld(seg, ICE_FLOW_FIELD_IDX_IPV4_SA, val_loc,
				 mask_loc, ICE_FLOW_FLD_OFF_INVAL, false);

		/* IP destination address */
		val_loc = offsetof(struct ice_fdir_fltr, ip.v4.dst_ip);
		mask_loc = offsetof(struct ice_fdir_fltr, mask.v4.dst_ip);

		ice_flow_set_fld(seg, ICE_FLOW_FIELD_IDX_IPV4_DA, val_loc,
				 mask_loc, ICE_FLOW_FLD_OFF_INVAL, false);

		break;
	case IPV4_USER_FLOW:
		usr_ip4_spec = &fsp->m_u.usr_ip4_spec;

		err = ice_check_ip4_usr_seg_fltr(usr_ip4_spec);
		if (err)
			goto err_exit;

		/* IP source address */
		val_loc = offsetof(struct ice_fdir_fltr, ip.v4.src_ip);
		mask_loc = offsetof(struct ice_fdir_fltr, mask.v4.src_ip);

		ice_flow_set_fld(seg, ICE_FLOW_FIELD_IDX_IPV4_SA, val_loc,
				 mask_loc, ICE_FLOW_FLD_OFF_INVAL, false);

		/* IP destination address */
		val_loc = offsetof(struct ice_fdir_fltr, ip.v4.dst_ip);
		mask_loc = offsetof(struct ice_fdir_fltr, mask.v4.dst_ip);

		ice_flow_set_fld(seg, ICE_FLOW_FIELD_IDX_IPV4_DA, val_loc,
				 mask_loc, ICE_FLOW_FLD_OFF_INVAL, false);

		break;
#ifdef HAVE_ETHTOOL_FLOW_UNION_IP6_SPEC
	case TCP_V6_FLOW:
	case UDP_V6_FLOW:
	case SCTP_V6_FLOW:
		/* IPv6 not supported yet by ACL */
		if (pf->hw.acl_tbl)
			goto err_exit;

		tcp_ip6_spec = &fsp->m_u.tcp_ip6_spec;

		err = ice_check_ip6_seg_fltr(tcp_ip6_spec);
		if (err)
			goto err_exit;

		/* IP source address */
		val_loc = offsetof(struct ice_fdir_fltr, ip.v6.src_ip);
		mask_loc = offsetof(struct ice_fdir_fltr, mask.v6.src_ip);

		ice_flow_set_fld(seg, ICE_FLOW_FIELD_IDX_IPV6_SA, val_loc,
				 mask_loc, ICE_FLOW_FLD_OFF_INVAL, false);

		/* IP destination address */
		val_loc = offsetof(struct ice_fdir_fltr, ip.v6.dst_ip);
		mask_loc = offsetof(struct ice_fdir_fltr, mask.v6.dst_ip);

		ice_flow_set_fld(seg, ICE_FLOW_FIELD_IDX_IPV6_DA, val_loc,
				 mask_loc, ICE_FLOW_FLD_OFF_INVAL, false);

		break;
	case IPV6_USER_FLOW:
		/* IPv6 not supported yet by ACL */
		if (pf->hw.acl_tbl)
			goto err_exit;

		usr_ip6_spec = &fsp->m_u.usr_ip6_spec;

		err = ice_check_ip6_usr_seg_fltr(usr_ip6_spec);
		if (err)
			goto err_exit;

		/* IP source address */
		val_loc = offsetof(struct ice_fdir_fltr, ip.v6.src_ip);
		mask_loc = offsetof(struct ice_fdir_fltr, mask.v6.src_ip);

		ice_flow_set_fld(seg, ICE_FLOW_FIELD_IDX_IPV6_SA, val_loc,
				 mask_loc, ICE_FLOW_FLD_OFF_INVAL, false);

		/* IP destination address */
		val_loc = offsetof(struct ice_fdir_fltr, ip.v6.dst_ip);
		mask_loc = offsetof(struct ice_fdir_fltr, mask.v6.dst_ip);

		ice_flow_set_fld(seg, ICE_FLOW_FIELD_IDX_IPV6_DA, val_loc,
				 mask_loc, ICE_FLOW_FLD_OFF_INVAL, false);

		break;
#endif /* HAVE_ETHTOOL_FLOW_UNION_IP6_SPEC */
	default:
		goto err_exit;
	}

	switch (fsp->flow_type & ~FLOW_EXT) {
	case TCP_V4_FLOW:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_TCP |
				  ICE_FLOW_SEG_HDR_IPV4);

		/* Layer 4 source port */
		val_loc = offsetof(struct ice_fdir_fltr, ip.v4.src_port);
		mask_loc = offsetof(struct ice_fdir_fltr, mask.v4.src_port);

		ice_flow_set_fld(seg, ICE_FLOW_FIELD_IDX_TCP_SRC_PORT, val_loc,
				 mask_loc, ICE_FLOW_FLD_OFF_INVAL, false);

		/* Layer 4 destination port */
		val_loc = offsetof(struct ice_fdir_fltr, ip.v4.dst_port);
		mask_loc = offsetof(struct ice_fdir_fltr, mask.v4.dst_port);

		ice_flow_set_fld(seg, ICE_FLOW_FIELD_IDX_TCP_DST_PORT, val_loc,
				 mask_loc, ICE_FLOW_FLD_OFF_INVAL, false);

		break;
	case UDP_V4_FLOW:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_UDP |
				  ICE_FLOW_SEG_HDR_IPV4);

		/* Layer 4 source port */
		val_loc = offsetof(struct ice_fdir_fltr, ip.v4.src_port);
		mask_loc = offsetof(struct ice_fdir_fltr, mask.v4.src_port);

		ice_flow_set_fld(seg, ICE_FLOW_FIELD_IDX_UDP_SRC_PORT, val_loc,
				 mask_loc, ICE_FLOW_FLD_OFF_INVAL, false);

		/* Layer 4 destination port */
		val_loc = offsetof(struct ice_fdir_fltr, ip.v4.dst_port);
		mask_loc = offsetof(struct ice_fdir_fltr, mask.v4.dst_port);

		ice_flow_set_fld(seg, ICE_FLOW_FIELD_IDX_UDP_DST_PORT, val_loc,
				 mask_loc, ICE_FLOW_FLD_OFF_INVAL, false);

		break;
	case SCTP_V4_FLOW:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_SCTP |
				  ICE_FLOW_SEG_HDR_IPV4);

		/* Layer 4 source port */
		val_loc = offsetof(struct ice_fdir_fltr, ip.v4.src_port);
		mask_loc = offsetof(struct ice_fdir_fltr, mask.v4.src_port);

		ice_flow_set_fld(seg, ICE_FLOW_FIELD_IDX_SCTP_SRC_PORT, val_loc,
				 mask_loc, ICE_FLOW_FLD_OFF_INVAL, false);

		/* Layer 4 destination port */
		val_loc = offsetof(struct ice_fdir_fltr, ip.v4.dst_port);
		mask_loc = offsetof(struct ice_fdir_fltr, mask.v4.dst_port);

		ice_flow_set_fld(seg, ICE_FLOW_FIELD_IDX_SCTP_DST_PORT, val_loc,
				 mask_loc, ICE_FLOW_FLD_OFF_INVAL, false);

		break;
	case IPV4_USER_FLOW:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_IPV4);
		break;
#ifdef HAVE_ETHTOOL_FLOW_UNION_IP6_SPEC
	case TCP_V6_FLOW:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_IPV6 |
				  ICE_FLOW_SEG_HDR_TCP);

		/* Layer 4 source port */
		val_loc = offsetof(struct ice_fdir_fltr, ip.v6.src_port);
		mask_loc = offsetof(struct ice_fdir_fltr, mask.v6.src_port);

		ice_flow_set_fld(seg, ICE_FLOW_FIELD_IDX_TCP_SRC_PORT, val_loc,
				 mask_loc, ICE_FLOW_FLD_OFF_INVAL, false);

		/* Layer 4 destination port */
		val_loc = offsetof(struct ice_fdir_fltr, ip.v6.dst_port);
		mask_loc = offsetof(struct ice_fdir_fltr, mask.v6.dst_port);

		ice_flow_set_fld(seg, ICE_FLOW_FIELD_IDX_TCP_DST_PORT, val_loc,
				 mask_loc, ICE_FLOW_FLD_OFF_INVAL, false);

		break;
	case UDP_V6_FLOW:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_IPV6 |
				  ICE_FLOW_SEG_HDR_UDP);

		/* Layer 4 source port */
		val_loc = offsetof(struct ice_fdir_fltr, ip.v6.src_port);
		mask_loc = offsetof(struct ice_fdir_fltr, mask.v6.src_port);

		ice_flow_set_fld(seg, ICE_FLOW_FIELD_IDX_UDP_SRC_PORT, val_loc,
				 mask_loc, ICE_FLOW_FLD_OFF_INVAL, false);

		/* Layer 4 destination port */
		val_loc = offsetof(struct ice_fdir_fltr, ip.v6.dst_port);
		mask_loc = offsetof(struct ice_fdir_fltr, mask.v6.dst_port);

		ice_flow_set_fld(seg, ICE_FLOW_FIELD_IDX_UDP_DST_PORT, val_loc,
				 mask_loc, ICE_FLOW_FLD_OFF_INVAL, false);

		break;
	case SCTP_V6_FLOW:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_IPV6 |
				  ICE_FLOW_SEG_HDR_SCTP);

		/* Layer 4 source port */
		val_loc = offsetof(struct ice_fdir_fltr, ip.v6.src_port);
		mask_loc = offsetof(struct ice_fdir_fltr, mask.v6.src_port);

		ice_flow_set_fld(seg, ICE_FLOW_FIELD_IDX_SCTP_SRC_PORT, val_loc,
				 mask_loc, ICE_FLOW_FLD_OFF_INVAL, false);

		/* Layer 4 destination port */
		val_loc = offsetof(struct ice_fdir_fltr, ip.v6.dst_port);
		mask_loc = offsetof(struct ice_fdir_fltr, mask.v6.dst_port);

		ice_flow_set_fld(seg, ICE_FLOW_FIELD_IDX_SCTP_DST_PORT, val_loc,
				 mask_loc, ICE_FLOW_FLD_OFF_INVAL, false);

		break;
	case IPV6_USER_FLOW:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_IPV6);
		break;
#endif /* HAVE_ETHTOOL_FLOW_UNION_IP6_SPEC */
	default:
		goto err_exit;
	}

	fltr_type = ice_ethtool_flow_to_fltr(fsp->flow_type & ~FLOW_EXT);

	if (!hw->acl_prof) {
		hw->acl_prof = devm_kcalloc(dev, ICE_FLTR_PTYPE_MAX,
					    sizeof(*hw->acl_prof), GFP_KERNEL);
		if (!hw->acl_prof)
			goto err_exit;
	}
	if (!hw->acl_prof[fltr_type]) {
		hw->acl_prof[fltr_type] = devm_kzalloc(dev,
						       sizeof(**hw->acl_prof),
						       GFP_KERNEL);
		if (!hw->acl_prof[fltr_type])
			goto err_acl_prof_exit;
		hw->acl_prof[fltr_type]->cnt = 0;
	}

	hw_prof = hw->acl_prof[fltr_type];
	old_seg = hw_prof->fdir_seg[0];
	if (old_seg) {
		/* This flow_type already has an input set.
		 * If it matches the requested input set then we are
		 * done. If it's different then it's an error.
		 */
		if (!memcmp(old_seg, seg, sizeof(*seg))) {
			devm_kfree(dev, seg);
			return 0;
		}

		goto err_acl_prof_flow_exit;
	}

	/* Adding a profile for the given flow specification with no
	 * actions (NULL) and zero actions 0.
	 */
	status = ice_flow_add_prof(hw, ICE_BLK_ACL, ICE_FLOW_RX, fltr_type,
				   seg, 1, NULL, 0, &prof);
	err = ice_status_to_errno(status);
	if (err)
		goto err_exit;

	hw_prof->fdir_seg[0] = seg;
	return 0;

err_acl_prof_flow_exit:
	devm_kfree(dev, hw->acl_prof[fltr_type]);
err_acl_prof_exit:
	devm_kfree(dev, hw->acl_prof);
err_exit:
	devm_kfree(dev, seg);

	return -EOPNOTSUPP;
}

/**
 * ice_add_acl_rule_ethtool - Adds an ACL rule
 * @vsi: pointer to target VSI
 * @cmd: command to add or delete ACL rule
 *
 * Returns 0 on success and negative values for failure
 */
int ice_acl_add_rule_ethtool(struct ice_vsi *vsi, struct ethtool_rxnfc *cmd)
{
	struct ice_flow_action acts[ICE_ACL_NUM_ACT];
	struct ethtool_rx_flow_spec *fsp;
	struct ice_fd_hw_prof *hw_prof;
	struct ice_fdir_fltr *input;
	enum ice_fltr_ptype flow;
	enum ice_status status;
	struct device *dev;
	struct ice_pf *pf;
	struct ice_hw *hw;
	u64 entry_h = 0;
	int act_cnt;
	int ret;

	if (!vsi || !cmd)
		return -EINVAL;

	pf = vsi->back;
	hw = &pf->hw;
	dev = ice_pf_to_dev(pf);

	fsp = (struct ethtool_rx_flow_spec *)&cmd->fs;

	ret = ice_acl_check_input_set(pf, fsp);
	if (ret)
		return ret;

	/* Add new rule */
	input = devm_kzalloc(dev, sizeof(*input), GFP_KERNEL);
	if (!input)
		return -ENOMEM;

	ret = ice_acl_set_input_set(vsi, fsp, input);
	if (ret)
		goto free_input;

	memset(&acts, 0, sizeof(acts));
	act_cnt = 1;
	if (fsp->ring_cookie == RX_CLS_FLOW_DISC) {
		acts[0].type = ICE_FLOW_ACT_DROP;
		acts[0].data.acl_act.mdid = ICE_MDID_RX_PKT_DROP;
		acts[0].data.acl_act.prio = ICE_ACL_ACT_PRIO;
		acts[0].data.acl_act.value = cpu_to_le16(0x1);
	} else {
		acts[0].type = ICE_FLOW_ACT_FWD_QUEUE;
		acts[0].data.acl_act.mdid = ICE_MDID_RX_DST_Q;
		acts[0].data.acl_act.prio = ICE_ACL_ACT_PRIO;
		acts[0].data.acl_act.value = cpu_to_le16(input->q_index);
	}

	flow = ice_ethtool_flow_to_fltr(fsp->flow_type & ~FLOW_EXT);
	hw_prof = hw->acl_prof[flow];

	status = ice_flow_add_entry(hw, ICE_BLK_ACL, flow, fsp->location,
				    vsi->idx, ICE_FLOW_PRIO_NORMAL, input, acts,
				    act_cnt, &entry_h);
	ret = ice_status_to_errno(status);
	if (ret) {
		dev_err(dev, "Could not add flow entry %d\n", flow);
		goto free_input;
	}

	if (!hw_prof->cnt || vsi->idx != hw_prof->vsi_h[hw_prof->cnt - 1]) {
		hw_prof->vsi_h[hw_prof->cnt] = vsi->idx;
		hw_prof->entry_h[hw_prof->cnt++][0] = entry_h;
	}

	input->acl_fltr = true;
	/* input struct is added to the HW filter list */
	ice_fdir_update_list_entry(pf, input, fsp->location);

	return 0;

free_input:
	devm_kfree(dev, input);

	return ret;
}
