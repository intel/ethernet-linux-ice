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

	ret = ice_ntuple_set_input_set(vsi, ICE_BLK_ACL, fsp, input);
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
	ice_ntuple_update_list_entry(pf, input, fsp->location);

	return 0;

free_input:
	devm_kfree(dev, input);

	return ret;
}
