/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2018-2024 Intel Corporation */

#include "ice.h"
#include "ice_base.h"
#include "ice_lib.h"
#include "ice_vf_lib_private.h"

#define to_fltr_conf_from_desc(p) \
	container_of(p, struct virtchnl_fdir_fltr_conf, input)

#define ICE_FLOW_PROF_TYPE_S	0
#define ICE_FLOW_PROF_TYPE_M	(0xFFFFFFFFULL << ICE_FLOW_PROF_TYPE_S)
#define ICE_FLOW_PROF_VSI_S	32
#define ICE_FLOW_PROF_VSI_M	(0xFFFFFFFFULL << ICE_FLOW_PROF_VSI_S)

/* Flow profile ID format:
 * [0:31] - flow type, flow + tun_offs
 * [32:63] - VSI index
 */
#define ICE_FLOW_PROF_FD(vsi, flow, tun_offs) \
	(u64)(((((flow) + (tun_offs)) & ICE_FLOW_PROF_TYPE_M)) | \
	      FIELD_PREP(ICE_FLOW_PROF_VSI_M, vsi))

#define GTPU_TEID_OFFSET 4
#define GTPU_EH_QFI_OFFSET 1
#define GTPU_EH_QFI_MASK 0x3F
#define PFCP_S_OFFSET 0
#define PFCP_S_MASK 0x1
#define PFCP_PORT_NR 8805

#define FDIR_INSET_FLAG_ESP_S 0
#define FDIR_INSET_FLAG_ESP_M BIT_ULL(FDIR_INSET_FLAG_ESP_S)
#define FDIR_INSET_FLAG_ESP_UDP BIT_ULL(FDIR_INSET_FLAG_ESP_S)
#define FDIR_INSET_FLAG_ESP_IPSEC (0ULL << FDIR_INSET_FLAG_ESP_S)

#define FDIR_INSET_FLAG_ECPRI_S 1
#define FDIR_INSET_FLAG_ECPRI_M BIT_ULL(FDIR_INSET_FLAG_ECPRI_S)
#define FDIR_INSET_FLAG_ECPRI_UDP BIT_ULL(FDIR_INSET_FLAG_ECPRI_S)
#define FDIR_INSET_FLAG_ECPRI_MAC (0ULL << FDIR_INSET_FLAG_ECPRI_S)

/* These macros are used to set/check flow/tunnel type.
 * @param input: a struct of ice_fdir_fltr
 * @param conf: a struct of virtchnl_fdir_fltr_conf
 * @param f_type: NONF_IPV4_GTPU/NONF_IPV4_GTPU_IPV4, etc
 * @param tun_type: GTPU/ECPRI, etc
 */
#define FDIR_SET_FTYPE(f_type) \
	((input)->flow_type = ICE_FLTR_PTYPE_NONF_ ## f_type)
#define FDIR_CHK_FTYPE(f_type) \
	((input)->flow_type == ICE_FLTR_PTYPE_NONF_ ## f_type)
#define FDIR_SET_TTYPE(tun_type) \
	((conf)->ttype = ICE_FDIR_TUNNEL_TYPE_ ## tun_type)
#define FDIR_CHK_TTYPE(tun_type) \
	((conf)->ttype == ICE_FDIR_TUNNEL_TYPE_ ## tun_type)
#define FDIR_REPLACE_FTYPE(old_ftype, new_ftype) \
do { \
	if (FDIR_CHK_FTYPE(old_ftype)) \
		FDIR_SET_FTYPE(new_ftype); \
} while (0)

enum ice_fdir_tunnel_type {
	ICE_FDIR_TUNNEL_TYPE_NONE = 0,
	ICE_FDIR_TUNNEL_TYPE_GTPU,
	ICE_FDIR_TUNNEL_TYPE_GTPU_EH,
	ICE_FDIR_TUNNEL_TYPE_ECPRI,
	ICE_FDIR_TUNNEL_TYPE_GTPU_INNER,
	ICE_FDIR_TUNNEL_TYPE_GTPU_EH_INNER,
	ICE_FDIR_TUNNEL_TYPE_GRE,
	ICE_FDIR_TUNNEL_TYPE_GTPOGRE,
	ICE_FDIR_TUNNEL_TYPE_GTPOGRE_INNER,
	ICE_FDIR_TUNNEL_TYPE_GRE_INNER,
	ICE_FDIR_TUNNEL_TYPE_L2TPV2,
	ICE_FDIR_TUNNEL_TYPE_L2TPV2_INNER,
};

struct virtchnl_fdir_fltr_conf {
	struct ice_fdir_fltr input;
	enum ice_fdir_tunnel_type ttype;
	u64 inset_flag;
	u32 flow_id;

	struct ice_parser_profile *prof;
	bool parser_ena;
	u8 *pkt_buf;
	u8 pkt_len;
};

struct virtchnl_fdir_inset_map {
	enum virtchnl_proto_hdr_field field;
	enum ice_flow_field fld;
	u64 flag;
	u64 mask;
};

static const struct virtchnl_fdir_inset_map fdir_inset_map[] = {
	{VIRTCHNL_PROTO_HDR_ETH_SRC, ICE_FLOW_FIELD_IDX_ETH_SA,
		0, 0},
	{VIRTCHNL_PROTO_HDR_ETH_DST, ICE_FLOW_FIELD_IDX_ETH_DA,
		0, 0},
	{VIRTCHNL_PROTO_HDR_ETH_ETHERTYPE, ICE_FLOW_FIELD_IDX_ETH_TYPE,
		0, 0},
	{VIRTCHNL_PROTO_HDR_IPV4_SRC, ICE_FLOW_FIELD_IDX_IPV4_SA,
		0, 0},
	{VIRTCHNL_PROTO_HDR_IPV4_DST, ICE_FLOW_FIELD_IDX_IPV4_DA,
		0, 0},
	{VIRTCHNL_PROTO_HDR_IPV4_DSCP, ICE_FLOW_FIELD_IDX_IPV4_DSCP,
		0, 0},
	{VIRTCHNL_PROTO_HDR_IPV4_TTL, ICE_FLOW_FIELD_IDX_IPV4_TTL,
		0, 0},
	{VIRTCHNL_PROTO_HDR_IPV4_PROT, ICE_FLOW_FIELD_IDX_IPV4_PROT,
		0, 0},
	{VIRTCHNL_PROTO_HDR_IPV6_SRC, ICE_FLOW_FIELD_IDX_IPV6_SA,
		0, 0},
	{VIRTCHNL_PROTO_HDR_IPV6_DST, ICE_FLOW_FIELD_IDX_IPV6_DA,
		0, 0},
	{VIRTCHNL_PROTO_HDR_IPV6_TC, ICE_FLOW_FIELD_IDX_IPV6_DSCP,
		0, 0},
	{VIRTCHNL_PROTO_HDR_IPV6_HOP_LIMIT, ICE_FLOW_FIELD_IDX_IPV6_TTL,
		0, 0},
	{VIRTCHNL_PROTO_HDR_IPV6_PROT, ICE_FLOW_FIELD_IDX_IPV6_PROT,
		0, 0},
	{VIRTCHNL_PROTO_HDR_UDP_SRC_PORT, ICE_FLOW_FIELD_IDX_UDP_SRC_PORT,
		0, 0},
	{VIRTCHNL_PROTO_HDR_UDP_DST_PORT, ICE_FLOW_FIELD_IDX_UDP_DST_PORT,
		0, 0},
	{VIRTCHNL_PROTO_HDR_TCP_SRC_PORT, ICE_FLOW_FIELD_IDX_TCP_SRC_PORT,
		0, 0},
	{VIRTCHNL_PROTO_HDR_TCP_DST_PORT, ICE_FLOW_FIELD_IDX_TCP_DST_PORT,
		0, 0},
	{VIRTCHNL_PROTO_HDR_SCTP_SRC_PORT, ICE_FLOW_FIELD_IDX_SCTP_SRC_PORT,
		0, 0},
	{VIRTCHNL_PROTO_HDR_SCTP_DST_PORT, ICE_FLOW_FIELD_IDX_SCTP_DST_PORT,
		0, 0},
	{VIRTCHNL_PROTO_HDR_GTPU_IP_TEID, ICE_FLOW_FIELD_IDX_GTPU_IP_TEID,
		0, 0},
	{VIRTCHNL_PROTO_HDR_GTPU_EH_QFI, ICE_FLOW_FIELD_IDX_GTPU_EH_QFI,
		0, 0},
	{VIRTCHNL_PROTO_HDR_GTPU_UP_QFI, ICE_FLOW_FIELD_IDX_GTPU_UP_QFI,
		0, 0},
	{VIRTCHNL_PROTO_HDR_GTPU_DWN_QFI, ICE_FLOW_FIELD_IDX_GTPU_DWN_QFI,
		0, 0},
	{VIRTCHNL_PROTO_HDR_ESP_SPI, ICE_FLOW_FIELD_IDX_ESP_SPI,
		FDIR_INSET_FLAG_ESP_IPSEC, FDIR_INSET_FLAG_ESP_M},
	{VIRTCHNL_PROTO_HDR_ESP_SPI, ICE_FLOW_FIELD_IDX_NAT_T_ESP_SPI,
		FDIR_INSET_FLAG_ESP_UDP, FDIR_INSET_FLAG_ESP_M},
	{VIRTCHNL_PROTO_HDR_AH_SPI, ICE_FLOW_FIELD_IDX_AH_SPI,
		0, 0},
	{VIRTCHNL_PROTO_HDR_L2TPV3_SESS_ID, ICE_FLOW_FIELD_IDX_L2TPV3_SESS_ID,
		0, 0},
	{VIRTCHNL_PROTO_HDR_PFCP_S_FIELD, ICE_FLOW_FIELD_IDX_UDP_DST_PORT,
		0, 0},
	{
		VIRTCHNL_PROTO_HDR_ECPRI_PC_RTC_ID,
		ICE_FLOW_FIELD_IDX_ECPRI_TP0_PC_ID,
		FDIR_INSET_FLAG_ECPRI_MAC,
		FDIR_INSET_FLAG_ECPRI_M
	},
	{
		VIRTCHNL_PROTO_HDR_ECPRI_PC_RTC_ID,
		ICE_FLOW_FIELD_IDX_UDP_ECPRI_TP0_PC_ID,
		FDIR_INSET_FLAG_ECPRI_UDP,
		FDIR_INSET_FLAG_ECPRI_M
	},
	{VIRTCHNL_PROTO_HDR_IPV4_FRAG_PKID, ICE_FLOW_FIELD_IDX_IPV4_ID,
		0, 0},
	{VIRTCHNL_PROTO_HDR_IPV6_EH_FRAG_PKID, ICE_FLOW_FIELD_IDX_IPV6_ID,
		0, 0},
	{VIRTCHNL_PROTO_HDR_L2TPV2_SESS_ID, ICE_FLOW_FIELD_IDX_L2TPV2_SESS_ID,
		0, 0},
	{VIRTCHNL_PROTO_HDR_L2TPV2_LEN_SESS_ID,
		ICE_FLOW_FIELD_IDX_L2TPV2_LEN_SESS_ID,
		0, 0},
};

/**
 * ice_vc_fdir_param_check
 * @vf: pointer to the VF structure
 * @vsi_id: VF relative VSI ID
 *
 * Check for the valid VSI ID, PF's state and VF's state
 *
 * Return: 0 on success, and -EINVAL on error.
 */
static int
ice_vc_fdir_param_check(struct ice_vf *vf, u16 vsi_id)
{
	struct ice_pf *pf = vf->pf;

	if (!test_bit(ICE_FLAG_FD_ENA, pf->flags))
		return -EINVAL;

	if (!test_bit(ICE_VF_STATE_ACTIVE, vf->vf_states))
		return -EINVAL;

	if (!(vf->driver_caps & VIRTCHNL_VF_OFFLOAD_FDIR_PF))
		return -EINVAL;

	if (!ice_vc_isvalid_vsi_id(vf, vsi_id))
		return -EINVAL;

	if (!ice_get_vf_vsi(vf))
		return -EINVAL;

	return 0;
}

/**
 * ice_vf_start_ctrl_vsi
 * @vf: pointer to the VF structure
 *
 * Allocate ctrl_vsi for the first time and open the ctrl_vsi port for VF
 *
 * Return: 0 on success, and other on error.
 */
static int ice_vf_start_ctrl_vsi(struct ice_vf *vf)
{
	struct ice_pf *pf = vf->pf;
	struct ice_vsi *ctrl_vsi;
	struct device *dev;
	int err;

	dev = ice_pf_to_dev(pf);
	if (vf->ctrl_vsi_idx != ICE_NO_VSI)
		return -EEXIST;

	ctrl_vsi = ice_vf_ctrl_vsi_setup(vf);
	if (!ctrl_vsi) {
		dev_dbg(dev, "Could not setup control VSI for VF %d\n",
			vf->vf_id);
		return -ENOMEM;
	}

	err = ice_vsi_open_ctrl(ctrl_vsi);
	if (err) {
		dev_dbg(dev, "Could not open control VSI for VF %d\n",
			vf->vf_id);
		goto err_vsi_open;
	}

	return 0;

err_vsi_open:
	ice_vsi_release(ctrl_vsi);
	if (vf->ctrl_vsi_idx != ICE_NO_VSI) {
		pf->vsi[vf->ctrl_vsi_idx] = NULL;
		vf->ctrl_vsi_idx = ICE_NO_VSI;
	}
	return err;
}

/**
 * ice_vc_fdir_alloc_prof - allocate profile for this filter flow type
 * @vf: pointer to the VF structure
 * @flow: filter flow type
 *
 * Return: 0 on success, and other on error.
 */
static int
ice_vc_fdir_alloc_prof(struct ice_vf *vf, enum ice_fltr_ptype flow)
{
	struct ice_vf_fdir *fdir = &vf->fdir;

	if (!fdir->fdir_prof) {
		fdir->fdir_prof = kcalloc(ICE_FLTR_PTYPE_MAX,
					  sizeof(*fdir->fdir_prof),
					  GFP_KERNEL);
		if (!fdir->fdir_prof)
			return -ENOMEM;
	}

	if (!fdir->fdir_prof[flow]) {
		fdir->fdir_prof[flow] = kzalloc(sizeof(**fdir->fdir_prof),
						GFP_KERNEL);
		if (!fdir->fdir_prof[flow])
			return -ENOMEM;
	}

	return 0;
}

/**
 * ice_vc_fdir_free_prof - free profile for this filter flow type
 * @vf: pointer to the VF structure
 * @flow: filter flow type
 */
static void
ice_vc_fdir_free_prof(struct ice_vf *vf, enum ice_fltr_ptype flow)
{
	struct ice_vf_fdir *fdir = &vf->fdir;

	if (!fdir->fdir_prof)
		return;

	if (!fdir->fdir_prof[flow])
		return;

	kfree(fdir->fdir_prof[flow]);
	fdir->fdir_prof[flow] = NULL;
}

/**
 * ice_vc_fdir_free_prof_all - free all the profile for this VF
 * @vf: pointer to the VF structure
 */
void ice_vc_fdir_free_prof_all(struct ice_vf *vf)
{
	struct ice_vf_fdir *fdir = &vf->fdir;
	enum ice_fltr_ptype flow;

	if (!fdir->fdir_prof)
		return;

	for (flow = ICE_FLTR_PTYPE_NONF_NONE; flow < ICE_FLTR_PTYPE_MAX; flow++)
		ice_vc_fdir_free_prof(vf, flow);

	kfree(fdir->fdir_prof);
	fdir->fdir_prof = NULL;
}

/**
 * ice_vc_fdir_parse_flow_fld
 * @vf: pointer to the VF structure
 * @proto_hdr: virtual channel protocol filter header
 * @conf: FDIR configuration for each filter
 * @fld: field type array
 * @fld_cnt: field counter
 *
 * Parse the virtual channel filter header and store them into field type array
 *
 * Return: 0 on success, and other on error.
 */
static int
ice_vc_fdir_parse_flow_fld(struct ice_vf *vf,
			   struct virtchnl_proto_hdr *proto_hdr,
			   struct virtchnl_fdir_fltr_conf *conf,
			   enum ice_flow_field *fld,
			   int *fld_cnt)
{
	struct virtchnl_proto_hdr hdr;
	u32 i;

	memcpy(&hdr, proto_hdr, sizeof(hdr));

	for (i = 0; (i < ARRAY_SIZE(fdir_inset_map)) &&
	     VIRTCHNL_GET_PROTO_HDR_FIELD(&hdr); i++) {
		if (VIRTCHNL_TEST_PROTO_HDR(&hdr, fdir_inset_map[i].field)) {
			if (fdir_inset_map[i].mask &&
			    ((fdir_inset_map[i].mask & conf->inset_flag)
			    != fdir_inset_map[i].flag))
				continue;

			fld[*fld_cnt] = fdir_inset_map[i].fld;
			*fld_cnt += 1;
			if (*fld_cnt >= ICE_FLOW_FIELD_IDX_MAX)
				return -EINVAL;
			VIRTCHNL_DEL_PROTO_HDR_FIELD(&hdr,
						     fdir_inset_map[i].field);
		}
	}

	return 0;
}

/**
 * ice_vc_fdir_set_flow_fld
 * @vf: pointer to the VF structure
 * @fltr: virtual channel add cmd buffer
 * @conf: FDIR configuration for each filter
 * @segs: array of one or more packet segments that describe the flow
 * @tun: 0 implies non-tunnel type filter, 1 implies tunnel type filter
 *
 * Parse the virtual channel add msg buffer's field vector and store them into
 * flow's packet segment field
 *
 * Return: 0 on success, and other on error.
 */
static int
ice_vc_fdir_set_flow_fld(struct ice_vf *vf,
			 struct virtchnl_fdir_add *fltr,
			 struct virtchnl_fdir_fltr_conf *conf,
			 struct ice_flow_seg_info *segs,
			 int tun)
{
	struct virtchnl_fdir_rule *rule = &fltr->rule_cfg;
	enum ice_flow_field fld[ICE_FLOW_FIELD_IDX_MAX];
	struct device *dev = ice_pf_to_dev(vf->pf);
	struct virtchnl_proto_hdrs *proto;
	struct ice_flow_seg_info *seg;
	int fld_cnt = 0;
	int i;

	proto = &rule->proto_hdrs;
	for (i = 0; i < proto->count; i++) {
		struct virtchnl_proto_hdr *hdr = &proto->proto_hdr[i];
		int ret;

		ret = ice_vc_fdir_parse_flow_fld(vf, hdr, conf, fld, &fld_cnt);
		if (ret)
			return ret;
	}

	if (fld_cnt == 0) {
		dev_dbg(dev, "Empty input set for VF %d\n", vf->vf_id);
		return -EINVAL;
	}

	seg = (tun) ? &segs[tun] : segs;

	for (i = 0; i < fld_cnt; i++) {
		ice_flow_set_fld(seg, fld[i],
				 ICE_FLOW_FLD_OFF_INVAL,
				 ICE_FLOW_FLD_OFF_INVAL,
				 ICE_FLOW_FLD_OFF_INVAL, false);
	}

	return 0;
}

/**
 * ice_vc_fdir_set_flow_hdr - config the flow's packet segment header
 * @vf: pointer to the VF structure
 * @conf: FDIR configuration for each filter
 * @segs: array of one or more packet segments that describe the flow
 * @tun: 0 implies non-tunnel type filter, 1 implies tunnel type filter
 *
 * Return: 0 on success, and other on error.
 */
static int
ice_vc_fdir_set_flow_hdr(struct ice_vf *vf,
			 struct virtchnl_fdir_fltr_conf *conf,
			 struct ice_flow_seg_info *segs,
			 int tun)
{
	enum ice_fltr_ptype flow = conf->input.flow_type;
	enum ice_fdir_tunnel_type ttype = conf->ttype;
	struct device *dev = ice_pf_to_dev(vf->pf);
	struct ice_flow_seg_info *seg = NULL;

	if (ttype == ICE_FDIR_TUNNEL_TYPE_GTPU_INNER ||
	    ttype == ICE_FDIR_TUNNEL_TYPE_GTPU_EH_INNER ||
	    ttype == ICE_FDIR_TUNNEL_TYPE_GTPOGRE_INNER ||
	    ttype == ICE_FDIR_TUNNEL_TYPE_GRE_INNER ||
	    ttype == ICE_FDIR_TUNNEL_TYPE_L2TPV2_INNER) {
		seg = &segs[0];
		switch (flow) {
		case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_IPV4:
		case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_IPV4_UDP:
		case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_IPV4_TCP:
		case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_IPV6:
		case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_IPV6_UDP:
		case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_IPV6_TCP:
		case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_IPV4:
		case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_IPV4_UDP:
		case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_IPV4_TCP:
		case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_IPV6:
		case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_IPV6_UDP:
		case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_IPV6_TCP:
		case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_DW_IPV4:
		case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_DW_IPV4_UDP:
		case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_DW_IPV4_TCP:
		case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_DW_IPV6:
		case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_DW_IPV6_UDP:
		case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_DW_IPV6_TCP:
		case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_UP_IPV4:
		case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_UP_IPV4_UDP:
		case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_UP_IPV4_TCP:
		case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_UP_IPV6:
		case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_UP_IPV6_UDP:
		case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_UP_IPV6_TCP:
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_IPV4 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER |
					  ICE_FLOW_SEG_HDR_GRE);
			break;
		case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_IPV4:
		case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_IPV4_UDP:
		case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_IPV4_TCP:
		case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_IPV6:
		case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_IPV6_UDP:
		case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_IPV6_TCP:
		case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_IPV4:
		case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_IPV4_UDP:
		case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_IPV4_TCP:
		case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_IPV6:
		case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_IPV6_UDP:
		case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_IPV6_TCP:
		case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_DW_IPV4:
		case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_DW_IPV4_UDP:
		case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_DW_IPV4_TCP:
		case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_DW_IPV6:
		case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_DW_IPV6_UDP:
		case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_DW_IPV6_TCP:
		case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_UP_IPV4:
		case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_UP_IPV4_UDP:
		case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_UP_IPV4_TCP:
		case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_UP_IPV6:
		case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_UP_IPV6_UDP:
		case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_UP_IPV6_TCP:
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_IPV6 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER |
					  ICE_FLOW_SEG_HDR_GRE);
			break;
		case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4:
		case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_UDP:
		case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_TCP:
		case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV6:
		case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV6_UDP:
		case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV6_TCP:
		case ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_PPP_IPV4:
		case ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_PPP_IPV4_UDP:
		case ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_PPP_IPV4_TCP:
		case ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_PPP_IPV6:
		case ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_PPP_IPV6_UDP:
		case ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_PPP_IPV6_TCP:
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_IPV4 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
			break;
		case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4:
		case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_UDP:
		case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_TCP:
		case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV6:
		case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV6_UDP:
		case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV6_TCP:
		case ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_PPP_IPV4:
		case ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_PPP_IPV4_UDP:
		case ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_PPP_IPV4_TCP:
		case ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_PPP_IPV6:
		case ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_PPP_IPV6_UDP:
		case ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_PPP_IPV6_TCP:
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_IPV6 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
			break;
		default:
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_IPV4 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
			break;
		}
	}

	seg = (tun) ? &segs[tun] : segs;

	switch (flow) {
	case ICE_FLTR_PTYPE_NON_IP_L2:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_ETH_NON_IP);
		break;
	case ICE_FLTR_PTYPE_NONF_ECPRI_TP0:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_ECPRI_TP0);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_UDP_ECPRI_TP0:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_UDP_ECPRI_TP0 |
				  ICE_FLOW_SEG_HDR_IPV4 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_L2TPV3:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_L2TPV3 |
				  ICE_FLOW_SEG_HDR_IPV4 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_ESP:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_ESP |
				  ICE_FLOW_SEG_HDR_IPV4 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_AH:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_AH |
				  ICE_FLOW_SEG_HDR_IPV4 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_NAT_T_ESP:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_NAT_T_ESP |
				  ICE_FLOW_SEG_HDR_IPV4 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_PFCP_NODE:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_PFCP_NODE |
				  ICE_FLOW_SEG_HDR_IPV4 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_PFCP_SESSION:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_PFCP_SESSION |
				  ICE_FLOW_SEG_HDR_IPV4 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_OTHER:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_IPV4 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_FRAG_IPV4:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_IPV4 |
				  ICE_FLOW_SEG_HDR_IPV_FRAG);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_TCP:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_TCP |
				  ICE_FLOW_SEG_HDR_IPV4 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_UDP:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_UDP |
				  ICE_FLOW_SEG_HDR_IPV4 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GTPU:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GTPU) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_IP |
					  ICE_FLOW_SEG_HDR_IPV4 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_IPV4:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GTPU_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_IP |
					  ICE_FLOW_SEG_HDR_IPV4 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_IPV4_UDP:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GTPU_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_IP |
					  ICE_FLOW_SEG_HDR_UDP |
					  ICE_FLOW_SEG_HDR_IPV4 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_IPV4_TCP:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GTPU_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_IP |
					  ICE_FLOW_SEG_HDR_TCP |
					  ICE_FLOW_SEG_HDR_IPV4 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_IPV6:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GTPU_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_IP |
					  ICE_FLOW_SEG_HDR_IPV6 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_IPV6_UDP:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GTPU_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_IP |
					  ICE_FLOW_SEG_HDR_UDP |
					  ICE_FLOW_SEG_HDR_IPV6 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_IPV6_TCP:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GTPU_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_IP |
					  ICE_FLOW_SEG_HDR_TCP |
					  ICE_FLOW_SEG_HDR_IPV6 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GTPU_EH) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_EH |
					  ICE_FLOW_SEG_HDR_GTPU_IP |
					  ICE_FLOW_SEG_HDR_IPV4 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV4:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GTPU_EH_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_EH |
					  ICE_FLOW_SEG_HDR_GTPU_IP |
					  ICE_FLOW_SEG_HDR_IPV4 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV4_UDP:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GTPU_EH_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_EH |
					  ICE_FLOW_SEG_HDR_GTPU_IP |
					  ICE_FLOW_SEG_HDR_UDP |
					  ICE_FLOW_SEG_HDR_IPV4 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV4_TCP:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GTPU_EH_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_EH |
					  ICE_FLOW_SEG_HDR_GTPU_IP |
					  ICE_FLOW_SEG_HDR_TCP |
					  ICE_FLOW_SEG_HDR_IPV4 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV6:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GTPU_EH_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_EH |
					  ICE_FLOW_SEG_HDR_GTPU_IP |
					  ICE_FLOW_SEG_HDR_IPV6 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV6_UDP:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GTPU_EH_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_EH |
					  ICE_FLOW_SEG_HDR_GTPU_IP |
					  ICE_FLOW_SEG_HDR_UDP |
					  ICE_FLOW_SEG_HDR_IPV6 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV6_TCP:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GTPU_EH_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_EH |
					  ICE_FLOW_SEG_HDR_GTPU_IP |
					  ICE_FLOW_SEG_HDR_TCP |
					  ICE_FLOW_SEG_HDR_IPV6 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GTPU_EH) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_DWN |
					  ICE_FLOW_SEG_HDR_GTPU_IP |
					  ICE_FLOW_SEG_HDR_IPV4 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW_IPV4:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GTPU_EH_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_DWN |
					  ICE_FLOW_SEG_HDR_GTPU_IP |
					  ICE_FLOW_SEG_HDR_IPV4 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW_IPV4_UDP:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GTPU_EH_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_DWN |
					  ICE_FLOW_SEG_HDR_GTPU_IP |
					  ICE_FLOW_SEG_HDR_UDP |
					  ICE_FLOW_SEG_HDR_IPV4 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW_IPV4_TCP:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GTPU_EH_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_DWN |
					  ICE_FLOW_SEG_HDR_GTPU_IP |
					  ICE_FLOW_SEG_HDR_TCP |
					  ICE_FLOW_SEG_HDR_IPV4 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW_IPV6:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GTPU_EH_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_DWN |
					  ICE_FLOW_SEG_HDR_GTPU_IP |
					  ICE_FLOW_SEG_HDR_IPV6 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW_IPV6_UDP:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GTPU_EH_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_DWN |
					  ICE_FLOW_SEG_HDR_GTPU_IP |
					  ICE_FLOW_SEG_HDR_UDP |
					  ICE_FLOW_SEG_HDR_IPV6 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW_IPV6_TCP:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GTPU_EH_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_DWN |
					  ICE_FLOW_SEG_HDR_GTPU_IP |
					  ICE_FLOW_SEG_HDR_TCP |
					  ICE_FLOW_SEG_HDR_IPV6 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GTPU_EH) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_UP |
					  ICE_FLOW_SEG_HDR_GTPU_IP |
					  ICE_FLOW_SEG_HDR_IPV4 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP_IPV4:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GTPU_EH_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_UP |
					  ICE_FLOW_SEG_HDR_GTPU_IP |
					  ICE_FLOW_SEG_HDR_IPV4 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP_IPV4_UDP:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GTPU_EH_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_UP |
					  ICE_FLOW_SEG_HDR_GTPU_IP |
					  ICE_FLOW_SEG_HDR_UDP |
					  ICE_FLOW_SEG_HDR_IPV4 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP_IPV4_TCP:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GTPU_EH_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_UP |
					  ICE_FLOW_SEG_HDR_GTPU_IP |
					  ICE_FLOW_SEG_HDR_TCP |
					  ICE_FLOW_SEG_HDR_IPV4 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP_IPV6:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GTPU_EH_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_UP |
					  ICE_FLOW_SEG_HDR_GTPU_IP |
					  ICE_FLOW_SEG_HDR_IPV6 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP_IPV6_UDP:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GTPU_EH_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_UP |
					  ICE_FLOW_SEG_HDR_GTPU_IP |
					  ICE_FLOW_SEG_HDR_UDP |
					  ICE_FLOW_SEG_HDR_IPV6 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP_IPV6_TCP:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GTPU_EH_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_UP |
					  ICE_FLOW_SEG_HDR_GTPU_IP |
					  ICE_FLOW_SEG_HDR_TCP |
					  ICE_FLOW_SEG_HDR_IPV6 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV6_GTPU:
	case ICE_FLTR_PTYPE_NONF_IPV6_GTPU_IPV6_OTHER:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GTPU) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_IP |
					  ICE_FLOW_SEG_HDR_IPV6 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV6_GTPU_EH:
	case ICE_FLTR_PTYPE_NONF_IPV6_GTPU_EH_IPV6_OTHER:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GTPU_EH) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_EH |
					  ICE_FLOW_SEG_HDR_GTPU_IP |
					  ICE_FLOW_SEG_HDR_IPV6 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV6_GTPU_EH_DW:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GTPU_EH) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_DWN |
					  ICE_FLOW_SEG_HDR_GTPU_IP |
					  ICE_FLOW_SEG_HDR_IPV6 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV6_GTPU_EH_UP:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GTPU_EH) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_UP |
					  ICE_FLOW_SEG_HDR_GTPU_IP |
					  ICE_FLOW_SEG_HDR_IPV6 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_IP |
				  ICE_FLOW_SEG_HDR_IPV4 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_EH |
				  ICE_FLOW_SEG_HDR_GTPU_IP |
				  ICE_FLOW_SEG_HDR_IPV4 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_DW:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_DWN |
				  ICE_FLOW_SEG_HDR_GTPU_IP |
				  ICE_FLOW_SEG_HDR_IPV4 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_UP:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_UP |
				  ICE_FLOW_SEG_HDR_GTPU_IP |
				  ICE_FLOW_SEG_HDR_IPV4 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_IP |
				  ICE_FLOW_SEG_HDR_IPV6 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_EH |
				  ICE_FLOW_SEG_HDR_GTPU_IP |
				  ICE_FLOW_SEG_HDR_IPV6 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_DW:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_DWN |
				  ICE_FLOW_SEG_HDR_GTPU_IP |
				  ICE_FLOW_SEG_HDR_IPV6 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_UP:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_UP |
				  ICE_FLOW_SEG_HDR_GTPU_IP |
				  ICE_FLOW_SEG_HDR_IPV6 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_IPV4:
	case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_IPV4:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_IP |
				  ICE_FLOW_SEG_HDR_IPV4 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_IPV4_UDP:
	case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_IPV4_UDP:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_IP |
				  ICE_FLOW_SEG_HDR_UDP |
				  ICE_FLOW_SEG_HDR_IPV4 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_IPV4_TCP:
	case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_IPV4_TCP:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_IP |
				  ICE_FLOW_SEG_HDR_TCP |
				  ICE_FLOW_SEG_HDR_IPV4 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_IPV4:
	case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_IPV4:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_EH |
				  ICE_FLOW_SEG_HDR_GTPU_IP |
				  ICE_FLOW_SEG_HDR_IPV4 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_IPV4_UDP:
	case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_IPV4_UDP:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_EH |
				  ICE_FLOW_SEG_HDR_GTPU_IP |
				  ICE_FLOW_SEG_HDR_UDP |
				  ICE_FLOW_SEG_HDR_IPV4 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_IPV4_TCP:
	case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_IPV4_TCP:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_EH |
				  ICE_FLOW_SEG_HDR_GTPU_IP |
				  ICE_FLOW_SEG_HDR_TCP |
				  ICE_FLOW_SEG_HDR_IPV4 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_DW_IPV4:
	case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_DW_IPV4:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_DWN |
				  ICE_FLOW_SEG_HDR_GTPU_IP |
				  ICE_FLOW_SEG_HDR_IPV4 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_DW_IPV4_UDP:
	case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_DW_IPV4_UDP:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_DWN |
				  ICE_FLOW_SEG_HDR_GTPU_IP |
				  ICE_FLOW_SEG_HDR_UDP |
				  ICE_FLOW_SEG_HDR_IPV4 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_DW_IPV4_TCP:
	case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_DW_IPV4_TCP:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_DWN |
				  ICE_FLOW_SEG_HDR_GTPU_IP |
				  ICE_FLOW_SEG_HDR_TCP |
				  ICE_FLOW_SEG_HDR_IPV4 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_UP_IPV4:
	case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_UP_IPV4:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_UP |
				  ICE_FLOW_SEG_HDR_GTPU_IP |
				  ICE_FLOW_SEG_HDR_IPV4 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_UP_IPV4_UDP:
	case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_UP_IPV4_UDP:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_UP |
				  ICE_FLOW_SEG_HDR_GTPU_IP |
				  ICE_FLOW_SEG_HDR_UDP |
				  ICE_FLOW_SEG_HDR_IPV4 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_UP_IPV4_TCP:
	case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_UP_IPV4_TCP:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_UP |
				  ICE_FLOW_SEG_HDR_GTPU_IP |
				  ICE_FLOW_SEG_HDR_TCP |
				  ICE_FLOW_SEG_HDR_IPV4 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_IPV6:
	case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_IPV6:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_IP |
				  ICE_FLOW_SEG_HDR_IPV6 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_IPV6_UDP:
	case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_IPV6_UDP:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_IP |
				  ICE_FLOW_SEG_HDR_UDP |
				  ICE_FLOW_SEG_HDR_IPV6 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_IPV6_TCP:
	case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_IPV6_TCP:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_IP |
				  ICE_FLOW_SEG_HDR_TCP |
				  ICE_FLOW_SEG_HDR_IPV6 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_IPV6:
	case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_IPV6:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_EH |
				  ICE_FLOW_SEG_HDR_GTPU_IP |
				  ICE_FLOW_SEG_HDR_IPV6 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_IPV6_UDP:
	case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_IPV6_UDP:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_EH |
				  ICE_FLOW_SEG_HDR_GTPU_IP |
				  ICE_FLOW_SEG_HDR_UDP |
				  ICE_FLOW_SEG_HDR_IPV6 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_IPV6_TCP:
	case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_IPV6_TCP:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_EH |
				  ICE_FLOW_SEG_HDR_GTPU_IP |
				  ICE_FLOW_SEG_HDR_TCP |
				  ICE_FLOW_SEG_HDR_IPV6 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_DW_IPV6:
	case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_DW_IPV6:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_DWN |
				  ICE_FLOW_SEG_HDR_GTPU_IP |
				  ICE_FLOW_SEG_HDR_IPV6 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_DW_IPV6_UDP:
	case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_DW_IPV6_UDP:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_DWN |
				  ICE_FLOW_SEG_HDR_GTPU_IP |
				  ICE_FLOW_SEG_HDR_UDP |
				  ICE_FLOW_SEG_HDR_IPV6 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_DW_IPV6_TCP:
	case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_DW_IPV6_TCP:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_DWN |
				  ICE_FLOW_SEG_HDR_GTPU_IP |
				  ICE_FLOW_SEG_HDR_TCP |
				  ICE_FLOW_SEG_HDR_IPV6 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_UP_IPV6:
	case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_UP_IPV6:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_UP |
				  ICE_FLOW_SEG_HDR_GTPU_IP |
				  ICE_FLOW_SEG_HDR_IPV6 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_UP_IPV6_UDP:
	case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_UP_IPV6_UDP:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_UP |
				  ICE_FLOW_SEG_HDR_GTPU_IP |
				  ICE_FLOW_SEG_HDR_UDP |
				  ICE_FLOW_SEG_HDR_IPV6 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_GTPU_EH_UP_IPV6_TCP:
	case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_GTPU_EH_UP_IPV6_TCP:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GTPU_UP |
				  ICE_FLOW_SEG_HDR_GTPU_IP |
				  ICE_FLOW_SEG_HDR_TCP |
				  ICE_FLOW_SEG_HDR_IPV6 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_SCTP:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_SCTP |
				  ICE_FLOW_SEG_HDR_IPV4 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV6_L2TPV3:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_L2TPV3 |
				  ICE_FLOW_SEG_HDR_IPV6 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV6_ESP:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_ESP |
				  ICE_FLOW_SEG_HDR_IPV6 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV6_AH:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_AH |
				  ICE_FLOW_SEG_HDR_IPV6 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV6_NAT_T_ESP:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_NAT_T_ESP |
				  ICE_FLOW_SEG_HDR_IPV6 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV6_PFCP_NODE:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_PFCP_NODE |
				  ICE_FLOW_SEG_HDR_IPV6 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV6_PFCP_SESSION:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_PFCP_SESSION |
				  ICE_FLOW_SEG_HDR_IPV6 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV6_OTHER:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_IPV6 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_FRAG_IPV6:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_IPV6 |
				  ICE_FLOW_SEG_HDR_IPV_FRAG);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV6_TCP:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_TCP |
				  ICE_FLOW_SEG_HDR_IPV6 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV6_UDP:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_UDP |
				  ICE_FLOW_SEG_HDR_IPV6 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV6_SCTP:
		ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_SCTP |
				  ICE_FLOW_SEG_HDR_IPV6 |
				  ICE_FLOW_SEG_HDR_IPV_OTHER);
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4:
	case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GRE_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GRE |
					  ICE_FLOW_SEG_HDR_IPV4 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_UDP:
	case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_UDP:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GRE_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GRE |
					  ICE_FLOW_SEG_HDR_UDP |
					  ICE_FLOW_SEG_HDR_IPV4 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV4_TCP:
	case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV4_TCP:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GRE_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GRE |
					  ICE_FLOW_SEG_HDR_TCP |
					  ICE_FLOW_SEG_HDR_IPV4 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV6:
	case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV6:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GRE_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GRE |
					  ICE_FLOW_SEG_HDR_IPV6 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV6_UDP:
	case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV6_UDP:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GRE_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GRE |
					  ICE_FLOW_SEG_HDR_UDP |
					  ICE_FLOW_SEG_HDR_IPV6 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_GRE_IPV6_TCP:
	case ICE_FLTR_PTYPE_NONF_IPV6_GRE_IPV6_TCP:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_GRE_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_GRE |
					  ICE_FLOW_SEG_HDR_TCP |
					  ICE_FLOW_SEG_HDR_IPV6 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_CONTROL:
	case ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_L2TPV2) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_L2TPV2 |
					  ICE_FLOW_SEG_HDR_UDP |
					  ICE_FLOW_SEG_HDR_IPV4 |
					  ICE_FLOW_SEG_HDR_ETH);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_PPP:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_L2TPV2) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_L2TPV2 |
					  ICE_FLOW_SEG_HDR_PPP |
					  ICE_FLOW_SEG_HDR_UDP |
					  ICE_FLOW_SEG_HDR_IPV4 |
					  ICE_FLOW_SEG_HDR_ETH);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_CONTROL:
	case ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_L2TPV2) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_L2TPV2 |
					  ICE_FLOW_SEG_HDR_UDP |
					  ICE_FLOW_SEG_HDR_IPV6 |
					  ICE_FLOW_SEG_HDR_ETH);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_PPP:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_L2TPV2) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_L2TPV2 |
					  ICE_FLOW_SEG_HDR_PPP |
					  ICE_FLOW_SEG_HDR_UDP |
					  ICE_FLOW_SEG_HDR_IPV6 |
					  ICE_FLOW_SEG_HDR_ETH);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_PPP_IPV4:
	case ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_PPP_IPV4:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_L2TPV2_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_L2TPV2 |
					  ICE_FLOW_SEG_HDR_PPP |
					  ICE_FLOW_SEG_HDR_IPV4 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_PPP_IPV4_UDP:
	case ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_PPP_IPV4_UDP:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_L2TPV2_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_L2TPV2 |
					  ICE_FLOW_SEG_HDR_PPP |
					  ICE_FLOW_SEG_HDR_UDP |
					  ICE_FLOW_SEG_HDR_IPV4 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_PPP_IPV4_TCP:
	case ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_PPP_IPV4_TCP:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_L2TPV2_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_L2TPV2 |
					  ICE_FLOW_SEG_HDR_PPP |
					  ICE_FLOW_SEG_HDR_TCP |
					  ICE_FLOW_SEG_HDR_IPV4 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_PPP_IPV6:
	case ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_PPP_IPV6:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_L2TPV2_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_L2TPV2 |
					  ICE_FLOW_SEG_HDR_PPP |
					  ICE_FLOW_SEG_HDR_IPV6 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_PPP_IPV6_UDP:
	case ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_PPP_IPV6_UDP:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_L2TPV2_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_L2TPV2 |
					  ICE_FLOW_SEG_HDR_PPP |
					  ICE_FLOW_SEG_HDR_UDP |
					  ICE_FLOW_SEG_HDR_IPV6 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	case ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_PPP_IPV6_TCP:
	case ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_PPP_IPV6_TCP:
		if (ttype == ICE_FDIR_TUNNEL_TYPE_L2TPV2_INNER) {
			ICE_FLOW_SET_HDRS(seg, ICE_FLOW_SEG_HDR_L2TPV2 |
					  ICE_FLOW_SEG_HDR_PPP |
					  ICE_FLOW_SEG_HDR_TCP |
					  ICE_FLOW_SEG_HDR_IPV6 |
					  ICE_FLOW_SEG_HDR_IPV_OTHER);
		} else {
			dev_dbg(dev, "Invalid tunnel type 0x%x for VF %d\n",
				flow, vf->vf_id);
			return -EINVAL;
		}
		break;
	default:
		dev_dbg(dev, "Invalid flow type 0x%x for VF %d failed\n",
			flow, vf->vf_id);
		return -EINVAL;
	}

	return 0;
}

/**
 * ice_vc_fdir_rem_prof - remove profile for this filter flow type
 * @vf: pointer to the VF structure
 * @flow: filter flow type
 * @tun: 0 implies non-tunnel type filter, 1 implies tunnel type filter
 */
static void
ice_vc_fdir_rem_prof(struct ice_vf *vf, enum ice_fltr_ptype flow, int tun)
{
	struct ice_vf_fdir *fdir = &vf->fdir;
	struct ice_fd_hw_prof *vf_prof;
	struct ice_pf *pf = vf->pf;
	struct ice_vsi *vf_vsi;
	struct device *dev;
	struct ice_hw *hw;
	u64 prof_id;
	int i;

	dev = ice_pf_to_dev(pf);
	hw = &pf->hw;
	if (!fdir->fdir_prof || !fdir->fdir_prof[flow])
		return;

	vf_prof = fdir->fdir_prof[flow];

	vf_vsi = ice_get_vf_vsi(vf);
	if (!vf_vsi) {
		dev_dbg(dev, "NULL vf %d vsi pointer\n", vf->vf_id);
		return;
	}

	if (!fdir->prof_entry_cnt[flow][tun])
		return;

	prof_id = ICE_FLOW_PROF_FD(vf_vsi->vsi_num,
				   flow, tun ? ICE_FLTR_PTYPE_MAX : 0);

	for (i = 0; i < fdir->prof_entry_cnt[flow][tun]; i++) {
		if (vf_prof->entry_h[i][tun]) {
			u16 vsi_num = ice_get_hw_vsi_num(hw, vf_prof->vsi_h[i]);

			ice_rem_prof_id_flow(hw, ICE_BLK_FD, vsi_num, prof_id);
			ice_flow_rem_entry(hw, ICE_BLK_FD,
					   vf_prof->entry_h[i][tun]);
			vf_prof->entry_h[i][tun] = 0;
		}
	}

	ice_flow_rem_prof(hw, ICE_BLK_FD, prof_id);
	kfree(vf_prof->fdir_seg[tun]);
	vf_prof->fdir_seg[tun] = NULL;

	for (i = 0; i < vf_prof->cnt; i++)
		vf_prof->vsi_h[i] = 0;

	fdir->prof_entry_cnt[flow][tun] = 0;
}

/**
 * ice_vc_fdir_rem_prof_all - remove profile for this VF
 * @vf: pointer to the VF structure
 */
void ice_vc_fdir_rem_prof_all(struct ice_vf *vf)
{
	enum ice_fltr_ptype flow;

	for (flow = ICE_FLTR_PTYPE_NONF_NONE;
	     flow < ICE_FLTR_PTYPE_MAX; flow++) {
		ice_vc_fdir_rem_prof(vf, flow, 0);
		ice_vc_fdir_rem_prof(vf, flow, 1);
	}
}

/**
 * ice_vc_fdir_reset_cnt_all - reset all FDIR counters for this VF FDIR
 * @fdir: pointer to the VF FDIR structure
 */
static void ice_vc_fdir_reset_cnt_all(struct ice_vf_fdir *fdir)
{
	enum ice_fltr_ptype flow = ICE_FLTR_PTYPE_NONF_NONE;

	for (; flow < ICE_FLTR_PTYPE_MAX; flow++) {
		fdir->fdir_fltr_cnt[flow][0] = 0;
		fdir->fdir_fltr_cnt[flow][1] = 0;
	}
}

/**
 * ice_vc_fdir_write_flow_prof
 * @vf: pointer to the VF structure
 * @flow: filter flow type
 * @seg: array of one or more packet segments that describe the flow
 * @tun: 0 implies non-tunnel type filter, 1 implies tunnel type filter
 *
 * Write the flow's profile config and packet segment into the hardware
 *
 * Return: 0 on success, and other on error.
 */
static int
ice_vc_fdir_write_flow_prof(struct ice_vf *vf,
			    enum ice_fltr_ptype flow,
			    struct ice_flow_seg_info *seg,
			    int tun)
{
	struct ice_vf_fdir *fdir = &vf->fdir;
	struct ice_vsi *vf_vsi, *ctrl_vsi;
	struct ice_flow_seg_info *old_seg;
	struct ice_flow_prof *prof = NULL;
	struct ice_fd_hw_prof *vf_prof;
	struct device *dev;
	struct ice_pf *pf;
	struct ice_hw *hw;
	u64 entry1_h = 0;
	u64 entry2_h = 0;
	u64 prof_id;
	int ret;

	pf = vf->pf;
	dev = ice_pf_to_dev(pf);
	hw = &pf->hw;
	vf_vsi = ice_get_vf_vsi(vf);
	if (!vf_vsi)
		return -EINVAL;

	ctrl_vsi = pf->vsi[vf->ctrl_vsi_idx];
	if (!ctrl_vsi)
		return -EINVAL;

	vf_prof = fdir->fdir_prof[flow];
	old_seg = vf_prof->fdir_seg[tun];
	if (old_seg) {
		if (!memcmp(old_seg, seg, sizeof(*seg) * (tun + 1))) {
			dev_dbg(dev, "Duplicated profile for VF %d!\n",
				vf->vf_id);
			return -EEXIST;
		}

		if (fdir->fdir_fltr_cnt[flow][tun]) {
			ret = -EINVAL;
			dev_dbg(dev, "Input set conflicts for VF %d\n",
				vf->vf_id);
			goto err_exit;
		}

		/* remove previously allocated profile */
		ice_vc_fdir_rem_prof(vf, flow, tun);
	}

	prof_id = ICE_FLOW_PROF_FD(vf_vsi->vsi_num,
				   flow, tun ? ICE_FLTR_PTYPE_MAX : 0);

	ret = ice_flow_add_prof(hw, ICE_BLK_FD, ICE_FLOW_RX, prof_id, seg,
				tun + 1, NULL, 0, &prof);
	if (ret) {
		dev_dbg(dev, "Could not add VSI flow 0x%x for VF %d\n",
			flow, vf->vf_id);
		goto err_exit;
	}

	ret = ice_flow_add_entry(hw, ICE_BLK_FD, prof_id, vf_vsi->idx,
				 vf_vsi->idx, ICE_FLOW_PRIO_NORMAL,
				 seg, NULL, 0, &entry1_h);
	if (ret) {
		dev_dbg(dev, "Could not add flow 0x%x VSI entry for VF %d\n",
			flow, vf->vf_id);
		goto err_prof;
	}

	ret = ice_flow_add_entry(hw, ICE_BLK_FD, prof_id, vf_vsi->idx,
				 ctrl_vsi->idx, ICE_FLOW_PRIO_NORMAL,
				 seg, NULL, 0, &entry2_h);
	if (ret) {
		dev_dbg(dev,
			"Could not add flow 0x%x Ctrl VSI entry for VF %d\n",
			flow, vf->vf_id);
		goto err_entry_1;
	}

	vf_prof->fdir_seg[tun] = seg;
	vf_prof->cnt = 0;
	fdir->prof_entry_cnt[flow][tun] = 0;

	vf_prof->entry_h[vf_prof->cnt][tun] = entry1_h;
	vf_prof->vsi_h[vf_prof->cnt] = vf_vsi->idx;
	vf_prof->cnt++;
	fdir->prof_entry_cnt[flow][tun]++;

	vf_prof->entry_h[vf_prof->cnt][tun] = entry2_h;
	vf_prof->vsi_h[vf_prof->cnt] = ctrl_vsi->idx;
	vf_prof->cnt++;
	fdir->prof_entry_cnt[flow][tun]++;

	return 0;

err_entry_1:
	ice_rem_prof_id_flow(hw, ICE_BLK_FD,
			     ice_get_hw_vsi_num(hw, vf_vsi->idx), prof_id);
	ice_flow_rem_entry(hw, ICE_BLK_FD, entry1_h);
err_prof:
	ice_flow_rem_prof(hw, ICE_BLK_FD, prof_id);
err_exit:
	return ret;
}

/**
 * ice_vc_fdir_has_prof_conflict
 * @vf: pointer to the VF info
 * @conf: FDIR configuration for each filter
 *
 * Check if @conf has conflicting profile with existing profiles
 *
 * Return: true on success, and false on error.
 */
static bool
ice_vc_fdir_has_prof_conflict(struct ice_vf *vf,
			      struct virtchnl_fdir_fltr_conf *conf)
{
	struct ice_fdir_fltr *desc;

	list_for_each_entry(desc, &vf->fdir.fdir_rule_list, fltr_node) {
		struct virtchnl_fdir_fltr_conf *existing_conf =
				to_fltr_conf_from_desc(desc);
		struct ice_fdir_fltr *a = &existing_conf->input;
		struct ice_fdir_fltr *b = &conf->input;

		enum ice_fltr_ptype flow_type_a = a->flow_type;
		enum ice_fltr_ptype flow_type_b = b->flow_type;

		/* No need to compare two rules with different tunnel type */
		if (existing_conf->ttype != conf->ttype)
			continue;

		/* No need to compare two rules with same protocol */
		if (flow_type_a == flow_type_b)
			continue;

		switch (flow_type_a) {
		case ICE_FLTR_PTYPE_NONF_IPV4_UDP:
		case ICE_FLTR_PTYPE_NONF_IPV4_TCP:
		case ICE_FLTR_PTYPE_NONF_IPV4_SCTP:
			if (flow_type_b == ICE_FLTR_PTYPE_NONF_IPV4_OTHER)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV4_OTHER:
			if (flow_type_b == ICE_FLTR_PTYPE_NONF_IPV4_UDP ||
			    flow_type_b == ICE_FLTR_PTYPE_NONF_IPV4_TCP ||
			    flow_type_b == ICE_FLTR_PTYPE_NONF_IPV4_SCTP)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV6_UDP:
		case ICE_FLTR_PTYPE_NONF_IPV6_TCP:
		case ICE_FLTR_PTYPE_NONF_IPV6_SCTP:
			if (flow_type_b == ICE_FLTR_PTYPE_NONF_IPV6_OTHER)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV6_OTHER:
			if (flow_type_b == ICE_FLTR_PTYPE_NONF_IPV6_UDP ||
			    flow_type_b == ICE_FLTR_PTYPE_NONF_IPV6_TCP ||
			    flow_type_b == ICE_FLTR_PTYPE_NONF_IPV6_SCTP)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_IPV4:
			if (flow_type_b == ICE_FLTR_PTYPE_NONF_IPV4_GTPU ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_IPV4_UDP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_IPV4_TCP)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_IPV4_UDP:
		case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_IPV4_TCP:
			if (flow_type_b == ICE_FLTR_PTYPE_NONF_IPV4_GTPU ||
			    flow_type_b == ICE_FLTR_PTYPE_NONF_IPV4_GTPU_IPV4)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV4:
			if (flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV4_UDP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV4_TCP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW_IPV4 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW_IPV4_UDP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW_IPV4_TCP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP_IPV4 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP_IPV4_UDP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP_IPV4_TCP)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV4_UDP:
			if (flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV4 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW_IPV4 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW_IPV4_UDP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP_IPV4 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP_IPV4_UDP)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV4_TCP:
			if (flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV4 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW_IPV4 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW_IPV4_TCP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP_IPV4 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP_IPV4_TCP)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW_IPV4:
			if (flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV4 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV4_UDP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV4_TCP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW_IPV4_UDP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW_IPV4_TCP)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW_IPV4_UDP:
			if (flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV4 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV4_UDP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW_IPV4)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW_IPV4_TCP:
			if (flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV4 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV4_TCP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW_IPV4)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP_IPV4:
			if (flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV4 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV4_UDP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV4_TCP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP_IPV4_UDP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP_IPV4_TCP)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP_IPV4_UDP:
			if (flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV4 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV4_UDP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP_IPV4)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP_IPV4_TCP:
			if (flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV4 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV4_TCP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP_IPV4)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_IPV6:
			if (flow_type_b == ICE_FLTR_PTYPE_NONF_IPV4_GTPU ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_IPV6_UDP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_IPV6_TCP)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_IPV6_UDP:
		case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_IPV6_TCP:
			if (flow_type_b == ICE_FLTR_PTYPE_NONF_IPV4_GTPU ||
			    flow_type_b == ICE_FLTR_PTYPE_NONF_IPV4_GTPU_IPV6)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV6:
			if (flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV6_UDP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV6_TCP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW_IPV6 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW_IPV6_UDP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW_IPV6_TCP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP_IPV6 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP_IPV6_UDP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP_IPV6_TCP)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV6_UDP:
			if (flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV6 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW_IPV6 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW_IPV6_UDP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP_IPV6 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP_IPV6_UDP)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV6_TCP:
			if (flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV6 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW_IPV6 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW_IPV6_TCP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP_IPV6 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP_IPV6_TCP)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW_IPV6:
			if (flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV6 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV6_UDP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV6_TCP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW_IPV6_UDP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW_IPV6_TCP)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW_IPV6_UDP:
			if (flow_type_b ==
			ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV6 ||
			    flow_type_b ==
			ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV6_UDP ||
			    flow_type_b ==
			ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW_IPV6)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW_IPV6_TCP:
			if (flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV6 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV6_TCP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_DW_IPV6)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP_IPV6:
			if (flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV6 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV6_UDP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV6_TCP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP_IPV6_UDP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP_IPV6_TCP)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP_IPV6_UDP:
			if (flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV6 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV6_UDP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP_IPV6)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP_IPV6_TCP:
			if (flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV6 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_IPV6_TCP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_GTPU_EH_UP_IPV6)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_PPP:
			if (flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_CONTROL ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_PPP_IPV4 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_PPP_IPV4_UDP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_PPP_IPV4_TCP)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_PPP_IPV4:
			if (flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_CONTROL ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_PPP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_PPP_IPV4_UDP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_PPP_IPV4_TCP)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_PPP_IPV4_UDP:
		case ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_PPP_IPV4_TCP:
			if (flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_CONTROL ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_PPP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_PPP_IPV4)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_PPP_IPV6:
			if (flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_CONTROL ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_PPP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_PPP_IPV6_UDP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_PPP_IPV6_TCP)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_PPP_IPV6_UDP:
		case ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_PPP_IPV6_TCP:
			if (flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_CONTROL ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_PPP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV4_L2TPV2_PPP_IPV6)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_PPP:
			if (flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_CONTROL ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_PPP_IPV4 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_PPP_IPV4_UDP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_PPP_IPV4_TCP)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_PPP_IPV4:
			if (flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_CONTROL ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_PPP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_PPP_IPV4_UDP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_PPP_IPV4_TCP)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_PPP_IPV4_UDP:
		case ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_PPP_IPV4_TCP:
			if (flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_CONTROL ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_PPP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_PPP_IPV4)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_PPP_IPV6:
			if (flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_CONTROL ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_PPP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_PPP_IPV6_UDP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_PPP_IPV6_TCP)
				return true;
			break;
		case ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_PPP_IPV6_UDP:
		case ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_PPP_IPV6_TCP:
			if (flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_CONTROL ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2 ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_PPP ||
			    flow_type_b ==
				ICE_FLTR_PTYPE_NONF_IPV6_L2TPV2_PPP_IPV6)
				return true;
			break;
		default:
			break;
		}
	}

	return false;
}

/**
 * ice_vc_fdir_config_input_set
 * @vf: pointer to the VF structure
 * @fltr: virtual channel add cmd buffer
 * @conf: FDIR configuration for each filter
 * @tun: 0 implies non-tunnel type filter, 1 implies tunnel type filter
 *
 * Config the input set type and value for virtual channel add msg buffer
 *
 * Return: 0 on success, and other on error.
 */
static int
ice_vc_fdir_config_input_set(struct ice_vf *vf,
			     struct virtchnl_fdir_add *fltr,
			     struct virtchnl_fdir_fltr_conf *conf,
			     int tun)
{
	struct ice_fdir_fltr *input = &conf->input;
	struct device *dev = ice_pf_to_dev(vf->pf);
	struct ice_flow_seg_info *seg;
	enum ice_fltr_ptype flow;
	int ret;

	ret = ice_vc_fdir_has_prof_conflict(vf, conf);
	if (ret) {
		dev_dbg(dev, "Found flow prof conflict for VF %d\n", vf->vf_id);
		return ret;
	}

	flow = input->flow_type;
	ret = ice_vc_fdir_alloc_prof(vf, flow);
	if (ret) {
		dev_dbg(dev, "Alloc flow prof for VF %d failed\n", vf->vf_id);
		return ret;
	}

	seg = kcalloc((tun + 1), sizeof(*seg), GFP_KERNEL);
	if (!seg)
		return -ENOMEM;

	ret = ice_vc_fdir_set_flow_fld(vf, fltr, conf, seg, tun);
	if (ret) {
		dev_dbg(dev, "Set flow field for VF %d failed\n", vf->vf_id);
		goto err_exit;
	}

	ret = ice_vc_fdir_set_flow_hdr(vf, conf, seg, tun);
	if (ret) {
		dev_dbg(dev, "Set flow hdr for VF %d failed\n", vf->vf_id);
		goto err_exit;
	}

	ret = ice_vc_fdir_write_flow_prof(vf, flow, seg, tun);
	if (ret == -EEXIST) {
		kfree(seg);
	} else if (ret) {
		dev_dbg(dev, "Write flow profile for VF %d failed\n",
			vf->vf_id);
		goto err_exit;
	}

	return 0;

err_exit:
	kfree(seg);
	return ret;
}

/**
 * ice_vc_fdir_is_raw_flow
 * @proto: virtchnl protocol headers
 *
 * Check if the FDIR rule is raw flow (protocol agnostic flow) or not.
 * Note that common FDIR rule must have non-zero proto->count.
 * Thus, we choose the tunnel_level and count of proto as the indicators.
 * If both tunnel_level and count of proto are zeros, this FDIR rule will
 * be regarded as raw flow.
 *
 * Returns wheater headers describe raw flow or not.
 */
static bool
ice_vc_fdir_is_raw_flow(struct virtchnl_proto_hdrs *proto)
{
	return (proto->tunnel_level == 0 && proto->count == 0);
}

/**
 * ice_vc_fdir_parse_raw
 * @vf: pointer to the VF info
 * @proto: virtchnl protocol headers
 * @conf: FDIR configuration for each filter
 *
 * Parse the virtual channel filter's raw flow and store them into @conf
 *
 * Return: 0 on success, and other on error.
 */
static int
ice_vc_fdir_parse_raw(struct ice_vf *vf,
		      struct virtchnl_proto_hdrs *proto,
		      struct virtchnl_fdir_fltr_conf *conf)
{
	struct ice_parser_result rslt;
	struct ice_pf *pf = vf->pf;
	struct ice_parser *psr;
	u8 *pkt_buf, *msk_buf;
	int status = -ENOMEM;
	struct ice_hw *hw;
	u16 udp_port = 0;

	pkt_buf = kzalloc(proto->raw.pkt_len, GFP_KERNEL);
	msk_buf = kzalloc(proto->raw.pkt_len, GFP_KERNEL);
	if (!pkt_buf || !msk_buf)
		goto err_pkt_msk_buf_alloc;

	memcpy(pkt_buf, proto->raw.spec, proto->raw.pkt_len);
	memcpy(msk_buf, proto->raw.mask, proto->raw.pkt_len);

	hw = &pf->hw;
	/* Get raw profile info via Parser Lib */
	if (ice_parser_create(hw, &psr))
		goto err_parser_process;
	ice_parser_dvm_set(psr, ice_is_dvm_ena(hw));
	if (ice_get_open_tunnel_port(hw, TNL_VXLAN, &udp_port))
		ice_parser_vxlan_tunnel_set(psr, udp_port, true);
	if (ice_parser_run(psr, pkt_buf, proto->raw.pkt_len, &rslt))
		goto err_parser_process;
	ice_parser_destroy(psr);

	conf->prof = kzalloc(sizeof(*conf->prof), GFP_KERNEL);
	if (!conf->prof)
		goto err_conf_prof_alloc;

	status = ice_parser_profile_init(&rslt, pkt_buf, msk_buf,
					 proto->raw.pkt_len, ICE_BLK_FD, true,
					 conf->prof);
	if (status)
		goto err_parser_profile_init;

	/* Store raw flow info into @conf */
	conf->pkt_len = proto->raw.pkt_len;
	conf->pkt_buf = pkt_buf;
	kfree(msk_buf);

	conf->parser_ena = true;

	return 0;

err_parser_profile_init:
	kfree(conf->prof);
err_conf_prof_alloc:
err_parser_process:
	ice_parser_destroy(psr);
err_pkt_msk_buf_alloc:
	kfree(msk_buf);
	kfree(pkt_buf);
	return status;
}

/**
 * ice_vc_fdir_parse_pattern
 * @vf: pointer to the VF info
 * @fltr: virtual channel add cmd buffer
 * @conf: FDIR configuration for each filter
 *
 * Parse the virtual channel filter's pattern and store them into @conf
 *
 * Return: 0 on success, and other on error.
 */
static int
ice_vc_fdir_parse_pattern(struct ice_vf *vf,
			  struct virtchnl_fdir_add *fltr,
			  struct virtchnl_fdir_fltr_conf *conf)
{
	struct virtchnl_proto_hdrs *proto = &fltr->rule_cfg.proto_hdrs;
	enum virtchnl_proto_hdr_type l3 = VIRTCHNL_PROTO_HDR_NONE;
	enum virtchnl_proto_hdr_type l4 = VIRTCHNL_PROTO_HDR_NONE;
	struct device *dev = ice_pf_to_dev(vf->pf);
	struct ice_fdir_fltr *input = &conf->input;
	int i;

	if (proto->count > VIRTCHNL_MAX_NUM_PROTO_HDRS) {
		dev_dbg(dev, "Invalid protocol count:0x%x for VF %d\n",
			proto->count, vf->vf_id);
		return -EINVAL;
	}

	/* For Protocol Agnostic Flow Offloading case only */
	if (ice_vc_fdir_is_raw_flow(proto))
		return ice_vc_fdir_parse_raw(vf, proto, conf);

	for (i = 0; i < proto->count; i++) {
		struct virtchnl_proto_hdr *hdr = &proto->proto_hdr[i];
		struct frag_hdr *ip6h_ef;
		struct ip_esp_hdr *esph;
		struct ip_auth_hdr *ah;
		struct sctphdr *sctph;
		struct ipv6hdr *ip6h;
		struct udphdr *udph;
		struct tcphdr *tcph;
		struct ethhdr *eth;
		struct iphdr *iph;
		u16 frag_offset;
		u8 msg_type;
		u8 s_field;
		u8 *rawh;
		u16 flags_version;
		u16 pos;

		switch (hdr->type) {
		case VIRTCHNL_PROTO_HDR_ETH:
			eth = (struct ethhdr *)hdr->buffer;
			input->flow_type = ICE_FLTR_PTYPE_NON_IP_L2;

			if (hdr->field_selector) {
				ether_addr_copy(input->ext_data_outer.dst_mac,
						eth->h_dest);
				ether_addr_copy(input->ext_data_outer.src_mac,
						eth->h_source);
				input->ext_data.ether_type = eth->h_proto;
			}
			break;
		case VIRTCHNL_PROTO_HDR_IPV4:
			iph = (struct iphdr *)hdr->buffer;
			l3 = VIRTCHNL_PROTO_HDR_IPV4;

			if (FDIR_CHK_FTYPE(IPV4_GTPU))
				FDIR_SET_FTYPE(IPV4_GTPU_IPV4);
			else if (FDIR_CHK_FTYPE(IPV4_GTPU_EH))
				FDIR_SET_FTYPE(IPV4_GTPU_EH_IPV4);
			else if (FDIR_CHK_FTYPE(IPV4_GTPU_EH_DW))
				FDIR_SET_FTYPE(IPV4_GTPU_EH_DW_IPV4);
			else if (FDIR_CHK_FTYPE(IPV4_GTPU_EH_UP))
				FDIR_SET_FTYPE(IPV4_GTPU_EH_UP_IPV4);
			else if (FDIR_CHK_FTYPE(IPV4_GRE))
				FDIR_SET_FTYPE(IPV4_GRE_IPV4);
			else if (FDIR_CHK_FTYPE(IPV6_GRE))
				FDIR_SET_FTYPE(IPV6_GRE_IPV4);
			else if (FDIR_CHK_FTYPE(IPV4_GRE_IPV4_GTPU))
				FDIR_SET_FTYPE(IPV4_GRE_IPV4_GTPU_IPV4);
			else if (FDIR_CHK_FTYPE(IPV6_GRE_IPV4_GTPU))
				FDIR_SET_FTYPE(IPV6_GRE_IPV4_GTPU_IPV4);
			else if (FDIR_CHK_FTYPE(IPV4_GRE_IPV4_GTPU_EH))
				FDIR_SET_FTYPE(IPV4_GRE_IPV4_GTPU_EH_IPV4);
			else if (FDIR_CHK_FTYPE(IPV6_GRE_IPV4_GTPU_EH))
				FDIR_SET_FTYPE(IPV6_GRE_IPV4_GTPU_EH_IPV4);
			else if (FDIR_CHK_FTYPE(IPV4_GRE_IPV4_GTPU_EH_DW))
				FDIR_SET_FTYPE(IPV4_GRE_IPV4_GTPU_EH_DW_IPV4);
			else if (FDIR_CHK_FTYPE(IPV6_GRE_IPV4_GTPU_EH_DW))
				FDIR_SET_FTYPE(IPV6_GRE_IPV4_GTPU_EH_DW_IPV4);
			else if (FDIR_CHK_FTYPE(IPV4_GRE_IPV4_GTPU_EH_UP))
				FDIR_SET_FTYPE(IPV4_GRE_IPV4_GTPU_EH_UP_IPV4);
			else if (FDIR_CHK_FTYPE(IPV6_GRE_IPV4_GTPU_EH_UP))
				FDIR_SET_FTYPE(IPV6_GRE_IPV4_GTPU_EH_UP_IPV4);
			else if (FDIR_CHK_FTYPE(IPV4_L2TPV2_PPP))
				FDIR_SET_FTYPE(IPV4_L2TPV2_PPP_IPV4);
			else if (FDIR_CHK_FTYPE(IPV6_L2TPV2_PPP))
				FDIR_SET_FTYPE(IPV6_L2TPV2_PPP_IPV4);
			else
				FDIR_SET_FTYPE(IPV4_OTHER);

			if (FDIR_CHK_TTYPE(GTPU))
				FDIR_SET_TTYPE(GTPU_INNER);
			else if (FDIR_CHK_TTYPE(GTPU_EH))
				FDIR_SET_TTYPE(GTPU_EH_INNER);
			else if (FDIR_CHK_TTYPE(GRE))
				FDIR_SET_TTYPE(GRE_INNER);
			else if (FDIR_CHK_TTYPE(L2TPV2))
				FDIR_SET_TTYPE(L2TPV2_INNER);

			if (FDIR_CHK_TTYPE(GTPOGRE))
				FDIR_SET_TTYPE(GTPOGRE_INNER);

			if (hdr->field_selector) {
				input->ip.v4.src_ip = iph->saddr;
				input->ip.v4.dst_ip = iph->daddr;
				input->ip.v4.ttl = iph->ttl;
				input->ip.v4.tos = iph->tos;
				input->ip.v4.proto = iph->protocol;
			}
			break;
		case VIRTCHNL_PROTO_HDR_IPV4_FRAG:
			iph = (struct iphdr *)hdr->buffer;
			l3 = VIRTCHNL_PROTO_HDR_IPV4;
			frag_offset = be16_to_cpu(iph->frag_off);

			if (frag_offset >> ICE_FDIR_IPV4_PKT_FLAG_MF_SHIFT &
			    ICE_FDIR_IPV4_PKT_FLAG_MF) {
				input->flow_type = ICE_FLTR_PTYPE_FRAG_IPV4;
			} else {
				dev_err(dev, "Invalid fragment fdir for VF %d\n",
					vf->vf_id);
				return -EINVAL;
			}
			if (hdr->field_selector) {
				input->ip.v4.src_ip = iph->saddr;
				input->ip.v4.dst_ip = iph->daddr;
				input->ip.v4.ttl = iph->ttl;
				input->ip.v4.tos = iph->tos;
				input->ip.v4.proto = iph->protocol;
				input->ip.v4.packet_id = iph->id;
			}
			break;
		case VIRTCHNL_PROTO_HDR_IPV6:
			ip6h = (struct ipv6hdr *)hdr->buffer;
			l3 = VIRTCHNL_PROTO_HDR_IPV6;

			if (FDIR_CHK_FTYPE(IPV4_GTPU))
				FDIR_SET_FTYPE(IPV4_GTPU_IPV6);
			else if (FDIR_CHK_FTYPE(IPV4_GTPU_EH))
				FDIR_SET_FTYPE(IPV4_GTPU_EH_IPV6);
			else if (FDIR_CHK_FTYPE(IPV4_GTPU_EH_DW))
				FDIR_SET_FTYPE(IPV4_GTPU_EH_DW_IPV6);
			else if (FDIR_CHK_FTYPE(IPV4_GTPU_EH_UP))
				FDIR_SET_FTYPE(IPV4_GTPU_EH_UP_IPV6);
			else if (FDIR_CHK_FTYPE(IPV4_GRE))
				FDIR_SET_FTYPE(IPV4_GRE_IPV6);
			else if (FDIR_CHK_FTYPE(IPV6_GRE))
				FDIR_SET_FTYPE(IPV6_GRE_IPV6);
			else if (FDIR_CHK_FTYPE(IPV4_GRE_IPV4_GTPU))
				FDIR_SET_FTYPE(IPV4_GRE_IPV4_GTPU_IPV6);
			else if (FDIR_CHK_FTYPE(IPV6_GRE_IPV4_GTPU))
				FDIR_SET_FTYPE(IPV6_GRE_IPV4_GTPU_IPV6);
			else if (FDIR_CHK_FTYPE(IPV4_GRE_IPV4_GTPU_EH))
				FDIR_SET_FTYPE(IPV4_GRE_IPV4_GTPU_EH_IPV6);
			else if (FDIR_CHK_FTYPE(IPV6_GRE_IPV4_GTPU_EH))
				FDIR_SET_FTYPE(IPV6_GRE_IPV4_GTPU_EH_IPV6);
			else if (FDIR_CHK_FTYPE(IPV4_GRE_IPV4_GTPU_EH_DW))
				FDIR_SET_FTYPE(IPV4_GRE_IPV4_GTPU_EH_DW_IPV6);
			else if (FDIR_CHK_FTYPE(IPV6_GRE_IPV4_GTPU_EH_DW))
				FDIR_SET_FTYPE(IPV6_GRE_IPV4_GTPU_EH_DW_IPV6);
			else if (FDIR_CHK_FTYPE(IPV4_GRE_IPV4_GTPU_EH_UP))
				FDIR_SET_FTYPE(IPV4_GRE_IPV4_GTPU_EH_UP_IPV6);
			else if (FDIR_CHK_FTYPE(IPV6_GRE_IPV4_GTPU_EH_UP))
				FDIR_SET_FTYPE(IPV6_GRE_IPV4_GTPU_EH_UP_IPV6);
			else if (FDIR_CHK_FTYPE(IPV4_GRE))
				FDIR_SET_FTYPE(IPV4_GRE_IPV6);
			else if (FDIR_CHK_FTYPE(IPV6_GRE))
				FDIR_SET_FTYPE(IPV6_GRE_IPV6);
			else if (FDIR_CHK_FTYPE(IPV4_L2TPV2_PPP))
				FDIR_SET_FTYPE(IPV4_L2TPV2_PPP_IPV6);
			else if (FDIR_CHK_FTYPE(IPV6_L2TPV2_PPP))
				FDIR_SET_FTYPE(IPV6_L2TPV2_PPP_IPV6);
			else
				FDIR_SET_FTYPE(IPV6_OTHER);

			if (FDIR_CHK_TTYPE(GTPU))
				FDIR_SET_TTYPE(GTPU_INNER);
			else if (FDIR_CHK_TTYPE(GTPU_EH))
				FDIR_SET_TTYPE(GTPU_EH_INNER);
			else if (FDIR_CHK_TTYPE(GRE))
				FDIR_SET_TTYPE(GRE_INNER);
			else if (FDIR_CHK_TTYPE(L2TPV2))
				FDIR_SET_TTYPE(L2TPV2_INNER);

			if (FDIR_CHK_TTYPE(GTPOGRE))
				FDIR_SET_TTYPE(GTPOGRE_INNER);

			if (hdr->field_selector) {
				memcpy(input->ip.v6.src_ip,
				       ip6h->saddr.in6_u.u6_addr8,
				       sizeof(ip6h->saddr));
				memcpy(input->ip.v6.dst_ip,
				       ip6h->daddr.in6_u.u6_addr8,
				       sizeof(ip6h->daddr));
				input->ip.v6.hlim = ip6h->hop_limit;
				input->ip.v6.tc = ((u8)(ip6h->priority) << 4) |
						  (ip6h->flow_lbl[0] >> 4);
				input->ip.v6.proto = ip6h->nexthdr;
			}
			break;
		case VIRTCHNL_PROTO_HDR_IPV6_EH_FRAG:
			ip6h_ef = (struct frag_hdr *)hdr->buffer;
			frag_offset = be16_to_cpu(ip6h_ef->frag_off);
			if (FIELD_PREP(ICE_FDIR_IPV6_PKT_FLAG_MF, frag_offset)) {
				input->flow_type = ICE_FLTR_PTYPE_FRAG_IPV6;
			} else {
				dev_err(dev, "Invalid fragment fdir for VF %d\n",
					vf->vf_id);
				return -EINVAL;
			}

			if (hdr->field_selector)
				input->ip.v6.packet_id =
						ip6h_ef->identification;

			break;
		case VIRTCHNL_PROTO_HDR_TCP:
			tcph = (struct tcphdr *)hdr->buffer;
			if (l3 == VIRTCHNL_PROTO_HDR_IPV4 && !conf->ttype)
				input->flow_type = ICE_FLTR_PTYPE_NONF_IPV4_TCP;
			else if (l3 == VIRTCHNL_PROTO_HDR_IPV6 && !conf->ttype)
				input->flow_type = ICE_FLTR_PTYPE_NONF_IPV6_TCP;

			if (FDIR_CHK_FTYPE(IPV4_GTPU_IPV4))
				FDIR_SET_FTYPE(IPV4_GTPU_IPV4_TCP);
			else if (FDIR_CHK_FTYPE(IPV4_GTPU_EH_IPV4))
				FDIR_SET_FTYPE(IPV4_GTPU_EH_IPV4_TCP);
			else if (FDIR_CHK_FTYPE(IPV4_GTPU_EH_DW_IPV4))
				FDIR_SET_FTYPE(IPV4_GTPU_EH_DW_IPV4_TCP);
			else if (FDIR_CHK_FTYPE(IPV4_GTPU_EH_UP_IPV4))
				FDIR_SET_FTYPE(IPV4_GTPU_EH_UP_IPV4_TCP);

			if (FDIR_CHK_FTYPE(IPV4_GTPU_IPV6))
				FDIR_SET_FTYPE(IPV4_GTPU_IPV6_TCP);
			else if (FDIR_CHK_FTYPE(IPV4_GTPU_EH_IPV6))
				FDIR_SET_FTYPE(IPV4_GTPU_EH_IPV6_TCP);
			else if (FDIR_CHK_FTYPE(IPV4_GTPU_EH_DW_IPV6))
				FDIR_SET_FTYPE(IPV4_GTPU_EH_DW_IPV6_TCP);
			else if (FDIR_CHK_FTYPE(IPV4_GTPU_EH_UP_IPV6))
				FDIR_SET_FTYPE(IPV4_GTPU_EH_UP_IPV6_TCP);

			if (FDIR_CHK_FTYPE(IPV4_GRE_IPV4))
				FDIR_SET_FTYPE(IPV4_GRE_IPV4_TCP);
			else if (FDIR_CHK_FTYPE(IPV4_GRE_IPV6))
				FDIR_SET_FTYPE(IPV4_GRE_IPV6_TCP);
			else if (FDIR_CHK_FTYPE(IPV6_GRE_IPV4))
				FDIR_SET_FTYPE(IPV6_GRE_IPV4_TCP);
			else if (FDIR_CHK_FTYPE(IPV6_GRE_IPV6))
				FDIR_SET_FTYPE(IPV6_GRE_IPV6_TCP);

			FDIR_REPLACE_FTYPE(IPV4_GRE_IPV4_GTPU_IPV4,
					   IPV4_GRE_IPV4_GTPU_IPV4_TCP);
			FDIR_REPLACE_FTYPE(IPV6_GRE_IPV4_GTPU_IPV4,
					   IPV6_GRE_IPV4_GTPU_IPV4_TCP);
			FDIR_REPLACE_FTYPE(IPV4_GRE_IPV4_GTPU_IPV6,
					   IPV4_GRE_IPV4_GTPU_IPV6_TCP);
			FDIR_REPLACE_FTYPE(IPV6_GRE_IPV4_GTPU_IPV6,
					   IPV6_GRE_IPV4_GTPU_IPV6_TCP);
			FDIR_REPLACE_FTYPE(IPV4_GRE_IPV4_GTPU_EH_IPV4,
					   IPV4_GRE_IPV4_GTPU_EH_IPV4_TCP);
			FDIR_REPLACE_FTYPE(IPV6_GRE_IPV4_GTPU_EH_IPV4,
					   IPV6_GRE_IPV4_GTPU_EH_IPV4_TCP);
			FDIR_REPLACE_FTYPE(IPV4_GRE_IPV4_GTPU_EH_IPV6,
					   IPV4_GRE_IPV4_GTPU_EH_IPV6_TCP);
			FDIR_REPLACE_FTYPE(IPV6_GRE_IPV4_GTPU_EH_IPV6,
					   IPV6_GRE_IPV4_GTPU_EH_IPV6_TCP);
			FDIR_REPLACE_FTYPE(IPV4_GRE_IPV4_GTPU_EH_DW_IPV4,
					   IPV4_GRE_IPV4_GTPU_EH_DW_IPV4_TCP);
			FDIR_REPLACE_FTYPE(IPV6_GRE_IPV4_GTPU_EH_DW_IPV4,
					   IPV6_GRE_IPV4_GTPU_EH_DW_IPV4_TCP);
			FDIR_REPLACE_FTYPE(IPV4_GRE_IPV4_GTPU_EH_DW_IPV6,
					   IPV4_GRE_IPV4_GTPU_EH_DW_IPV6_TCP);
			FDIR_REPLACE_FTYPE(IPV6_GRE_IPV4_GTPU_EH_DW_IPV6,
					   IPV6_GRE_IPV4_GTPU_EH_DW_IPV6_TCP);
			FDIR_REPLACE_FTYPE(IPV4_GRE_IPV4_GTPU_EH_UP_IPV4,
					   IPV4_GRE_IPV4_GTPU_EH_UP_IPV4_TCP);
			FDIR_REPLACE_FTYPE(IPV6_GRE_IPV4_GTPU_EH_UP_IPV4,
					   IPV6_GRE_IPV4_GTPU_EH_UP_IPV4_TCP);
			FDIR_REPLACE_FTYPE(IPV4_GRE_IPV4_GTPU_EH_UP_IPV6,
					   IPV4_GRE_IPV4_GTPU_EH_UP_IPV6_TCP);
			FDIR_REPLACE_FTYPE(IPV6_GRE_IPV4_GTPU_EH_UP_IPV6,
					   IPV6_GRE_IPV4_GTPU_EH_UP_IPV6_TCP);

			if (FDIR_CHK_FTYPE(IPV4_L2TPV2_PPP_IPV4))
				FDIR_SET_FTYPE(IPV4_L2TPV2_PPP_IPV4_TCP);
			else if (FDIR_CHK_FTYPE(IPV4_L2TPV2_PPP_IPV6))
				FDIR_SET_FTYPE(IPV4_L2TPV2_PPP_IPV6_TCP);
			else if (FDIR_CHK_FTYPE(IPV6_L2TPV2_PPP_IPV4))
				FDIR_SET_FTYPE(IPV6_L2TPV2_PPP_IPV4_TCP);
			else if (FDIR_CHK_FTYPE(IPV6_L2TPV2_PPP_IPV6))
				FDIR_SET_FTYPE(IPV6_L2TPV2_PPP_IPV6_TCP);

			if (hdr->field_selector) {
				if (l3 == VIRTCHNL_PROTO_HDR_IPV4) {
					input->ip.v4.src_port = tcph->source;
					input->ip.v4.dst_port = tcph->dest;
				} else if (l3 == VIRTCHNL_PROTO_HDR_IPV6) {
					input->ip.v6.src_port = tcph->source;
					input->ip.v6.dst_port = tcph->dest;
				}
			}
			break;
		case VIRTCHNL_PROTO_HDR_UDP:
			udph = (struct udphdr *)hdr->buffer;
			l4 = VIRTCHNL_PROTO_HDR_UDP;
			if (l3 == VIRTCHNL_PROTO_HDR_IPV4 && !conf->ttype)
				input->flow_type = ICE_FLTR_PTYPE_NONF_IPV4_UDP;
			else if (l3 == VIRTCHNL_PROTO_HDR_IPV6 && !conf->ttype)
				input->flow_type = ICE_FLTR_PTYPE_NONF_IPV6_UDP;

			if (FDIR_CHK_FTYPE(IPV4_GTPU_IPV4))
				FDIR_SET_FTYPE(IPV4_GTPU_IPV4_UDP);
			else if (FDIR_CHK_FTYPE(IPV4_GTPU_EH_IPV4))
				FDIR_SET_FTYPE(IPV4_GTPU_EH_IPV4_UDP);
			else if (FDIR_CHK_FTYPE(IPV4_GTPU_EH_DW_IPV4))
				FDIR_SET_FTYPE(IPV4_GTPU_EH_DW_IPV4_UDP);
			else if (FDIR_CHK_FTYPE(IPV4_GTPU_EH_UP_IPV4))
				FDIR_SET_FTYPE(IPV4_GTPU_EH_UP_IPV4_UDP);

			if (FDIR_CHK_FTYPE(IPV4_GTPU_IPV6))
				FDIR_SET_FTYPE(IPV4_GTPU_IPV6_UDP);
			else if (FDIR_CHK_FTYPE(IPV4_GTPU_EH_IPV6))
				FDIR_SET_FTYPE(IPV4_GTPU_EH_IPV6_UDP);
			else if (FDIR_CHK_FTYPE(IPV4_GTPU_EH_DW_IPV6))
				FDIR_SET_FTYPE(IPV4_GTPU_EH_DW_IPV6_UDP);
			else if (FDIR_CHK_FTYPE(IPV4_GTPU_EH_UP_IPV6))
				FDIR_SET_FTYPE(IPV4_GTPU_EH_UP_IPV6_UDP);

			if (FDIR_CHK_FTYPE(IPV4_GRE_IPV4))
				FDIR_SET_FTYPE(IPV4_GRE_IPV4_UDP);
			else if (FDIR_CHK_FTYPE(IPV4_GRE_IPV6))
				FDIR_SET_FTYPE(IPV4_GRE_IPV6_UDP);
			else if (FDIR_CHK_FTYPE(IPV6_GRE_IPV4))
				FDIR_SET_FTYPE(IPV6_GRE_IPV4_UDP);
			else if (FDIR_CHK_FTYPE(IPV6_GRE_IPV6))
				FDIR_SET_FTYPE(IPV6_GRE_IPV6_UDP);

			FDIR_REPLACE_FTYPE(IPV4_GRE_IPV4_GTPU_IPV4,
					   IPV4_GRE_IPV4_GTPU_IPV4_UDP);
			FDIR_REPLACE_FTYPE(IPV6_GRE_IPV4_GTPU_IPV4,
					   IPV6_GRE_IPV4_GTPU_IPV4_UDP);
			FDIR_REPLACE_FTYPE(IPV4_GRE_IPV4_GTPU_IPV6,
					   IPV4_GRE_IPV4_GTPU_IPV6_UDP);
			FDIR_REPLACE_FTYPE(IPV6_GRE_IPV4_GTPU_IPV6,
					   IPV6_GRE_IPV4_GTPU_IPV6_UDP);
			FDIR_REPLACE_FTYPE(IPV4_GRE_IPV4_GTPU_EH_IPV4,
					   IPV4_GRE_IPV4_GTPU_EH_IPV4_UDP);
			FDIR_REPLACE_FTYPE(IPV6_GRE_IPV4_GTPU_EH_IPV4,
					   IPV6_GRE_IPV4_GTPU_EH_IPV4_UDP);
			FDIR_REPLACE_FTYPE(IPV4_GRE_IPV4_GTPU_EH_IPV6,
					   IPV4_GRE_IPV4_GTPU_EH_IPV6_UDP);
			FDIR_REPLACE_FTYPE(IPV6_GRE_IPV4_GTPU_EH_IPV6,
					   IPV6_GRE_IPV4_GTPU_EH_IPV6_UDP);
			FDIR_REPLACE_FTYPE(IPV4_GRE_IPV4_GTPU_EH_DW_IPV4,
					   IPV4_GRE_IPV4_GTPU_EH_DW_IPV4_UDP);
			FDIR_REPLACE_FTYPE(IPV6_GRE_IPV4_GTPU_EH_DW_IPV4,
					   IPV6_GRE_IPV4_GTPU_EH_DW_IPV4_UDP);
			FDIR_REPLACE_FTYPE(IPV4_GRE_IPV4_GTPU_EH_DW_IPV6,
					   IPV4_GRE_IPV4_GTPU_EH_DW_IPV6_UDP);
			FDIR_REPLACE_FTYPE(IPV6_GRE_IPV4_GTPU_EH_DW_IPV6,
					   IPV6_GRE_IPV4_GTPU_EH_DW_IPV6_UDP);
			FDIR_REPLACE_FTYPE(IPV4_GRE_IPV4_GTPU_EH_UP_IPV4,
					   IPV4_GRE_IPV4_GTPU_EH_UP_IPV4_UDP);
			FDIR_REPLACE_FTYPE(IPV6_GRE_IPV4_GTPU_EH_UP_IPV4,
					   IPV6_GRE_IPV4_GTPU_EH_UP_IPV4_UDP);
			FDIR_REPLACE_FTYPE(IPV4_GRE_IPV4_GTPU_EH_UP_IPV6,
					   IPV4_GRE_IPV4_GTPU_EH_UP_IPV6_UDP);
			FDIR_REPLACE_FTYPE(IPV6_GRE_IPV4_GTPU_EH_UP_IPV6,
					   IPV6_GRE_IPV4_GTPU_EH_UP_IPV6_UDP);

			if (FDIR_CHK_FTYPE(IPV4_L2TPV2_PPP_IPV4))
				FDIR_SET_FTYPE(IPV4_L2TPV2_PPP_IPV4_UDP);
			else if (FDIR_CHK_FTYPE(IPV4_L2TPV2_PPP_IPV6))
				FDIR_SET_FTYPE(IPV4_L2TPV2_PPP_IPV6_UDP);
			else if (FDIR_CHK_FTYPE(IPV6_L2TPV2_PPP_IPV4))
				FDIR_SET_FTYPE(IPV6_L2TPV2_PPP_IPV4_UDP);
			else if (FDIR_CHK_FTYPE(IPV6_L2TPV2_PPP_IPV6))
				FDIR_SET_FTYPE(IPV6_L2TPV2_PPP_IPV6_UDP);

			if (hdr->field_selector) {
				if (l3 == VIRTCHNL_PROTO_HDR_IPV4) {
					input->ip.v4.src_port = udph->source;
					input->ip.v4.dst_port = udph->dest;
				} else if (l3 == VIRTCHNL_PROTO_HDR_IPV6) {
					input->ip.v6.src_port = udph->source;
					input->ip.v6.dst_port = udph->dest;
				}
			}
			break;
		case VIRTCHNL_PROTO_HDR_SCTP:
			sctph = (struct sctphdr *)hdr->buffer;
			if (l3 == VIRTCHNL_PROTO_HDR_IPV4)
				input->flow_type =
					ICE_FLTR_PTYPE_NONF_IPV4_SCTP;
			else if (l3 == VIRTCHNL_PROTO_HDR_IPV6)
				input->flow_type =
					ICE_FLTR_PTYPE_NONF_IPV6_SCTP;

			if (hdr->field_selector) {
				if (l3 == VIRTCHNL_PROTO_HDR_IPV4) {
					input->ip.v4.src_port = sctph->source;
					input->ip.v4.dst_port = sctph->dest;
				} else if (l3 == VIRTCHNL_PROTO_HDR_IPV6) {
					input->ip.v6.src_port = sctph->source;
					input->ip.v6.dst_port = sctph->dest;
				}
			}
			break;
		case VIRTCHNL_PROTO_HDR_L2TPV3:
			if (l3 == VIRTCHNL_PROTO_HDR_IPV4)
				input->flow_type =
					ICE_FLTR_PTYPE_NONF_IPV4_L2TPV3;
			else if (l3 == VIRTCHNL_PROTO_HDR_IPV6)
				input->flow_type =
					ICE_FLTR_PTYPE_NONF_IPV6_L2TPV3;

			if (hdr->field_selector)
				input->l2tpv3_data.session_id =
					*((__force __be32 *)hdr->buffer);
			break;
		case VIRTCHNL_PROTO_HDR_ESP:
			esph = (struct ip_esp_hdr *)hdr->buffer;
			if (l3 == VIRTCHNL_PROTO_HDR_IPV4 &&
			    l4 == VIRTCHNL_PROTO_HDR_UDP)
				input->flow_type =
					ICE_FLTR_PTYPE_NONF_IPV4_NAT_T_ESP;
			else if (l3 == VIRTCHNL_PROTO_HDR_IPV6 &&
				 l4 == VIRTCHNL_PROTO_HDR_UDP)
				input->flow_type =
					ICE_FLTR_PTYPE_NONF_IPV6_NAT_T_ESP;
			else if (l3 == VIRTCHNL_PROTO_HDR_IPV4 &&
				 l4 == VIRTCHNL_PROTO_HDR_NONE)
				input->flow_type = ICE_FLTR_PTYPE_NONF_IPV4_ESP;
			else if (l3 == VIRTCHNL_PROTO_HDR_IPV6 &&
				 l4 == VIRTCHNL_PROTO_HDR_NONE)
				input->flow_type = ICE_FLTR_PTYPE_NONF_IPV6_ESP;

			if (l4 == VIRTCHNL_PROTO_HDR_UDP)
				conf->inset_flag |= FDIR_INSET_FLAG_ESP_UDP;
			else
				conf->inset_flag |= FDIR_INSET_FLAG_ESP_IPSEC;

			if (hdr->field_selector) {
				if (l3 == VIRTCHNL_PROTO_HDR_IPV4)
					input->ip.v4.sec_parm_idx = esph->spi;
				else if (l3 == VIRTCHNL_PROTO_HDR_IPV6)
					input->ip.v6.sec_parm_idx = esph->spi;
			}
			break;
		case VIRTCHNL_PROTO_HDR_AH:
			ah = (struct ip_auth_hdr *)hdr->buffer;
			if (l3 == VIRTCHNL_PROTO_HDR_IPV4)
				input->flow_type = ICE_FLTR_PTYPE_NONF_IPV4_AH;
			else if (l3 == VIRTCHNL_PROTO_HDR_IPV6)
				input->flow_type = ICE_FLTR_PTYPE_NONF_IPV6_AH;

			if (hdr->field_selector) {
				if (l3 == VIRTCHNL_PROTO_HDR_IPV4)
					input->ip.v4.sec_parm_idx = ah->spi;
				else if (l3 == VIRTCHNL_PROTO_HDR_IPV6)
					input->ip.v6.sec_parm_idx = ah->spi;
			}
			break;
		case VIRTCHNL_PROTO_HDR_PFCP:
			rawh = (u8 *)hdr->buffer;
			s_field = (rawh[0] >> PFCP_S_OFFSET) & PFCP_S_MASK;
			if (l3 == VIRTCHNL_PROTO_HDR_IPV4 && s_field == 0)
				input->flow_type =
					ICE_FLTR_PTYPE_NONF_IPV4_PFCP_NODE;
			else if (l3 == VIRTCHNL_PROTO_HDR_IPV4 && s_field == 1)
				input->flow_type =
					ICE_FLTR_PTYPE_NONF_IPV4_PFCP_SESSION;
			else if (l3 == VIRTCHNL_PROTO_HDR_IPV6 && s_field == 0)
				input->flow_type =
					ICE_FLTR_PTYPE_NONF_IPV6_PFCP_NODE;
			else if (l3 == VIRTCHNL_PROTO_HDR_IPV6 && s_field == 1)
				input->flow_type =
					ICE_FLTR_PTYPE_NONF_IPV6_PFCP_SESSION;

			if (hdr->field_selector) {
				if (l3 == VIRTCHNL_PROTO_HDR_IPV4)
					input->ip.v4.dst_port =
						cpu_to_be16(PFCP_PORT_NR);
				else if (l3 == VIRTCHNL_PROTO_HDR_IPV6)
					input->ip.v6.dst_port =
						cpu_to_be16(PFCP_PORT_NR);
			}
			break;
		case VIRTCHNL_PROTO_HDR_GRE:
			if (FDIR_CHK_FTYPE(IPV4_OTHER))
				FDIR_SET_FTYPE(IPV4_GRE);
			if (FDIR_CHK_FTYPE(IPV6_OTHER))
				FDIR_SET_FTYPE(IPV6_GRE);
			FDIR_SET_TTYPE(GRE);
			break;
		case VIRTCHNL_PROTO_HDR_GTPU_IP:
			rawh = (u8 *)hdr->buffer;
			if (FDIR_CHK_FTYPE(IPV4_UDP))
				FDIR_SET_FTYPE(IPV4_GTPU);
			else if (FDIR_CHK_FTYPE(IPV6_UDP))
				FDIR_SET_FTYPE(IPV6_GTPU);
			else if (FDIR_CHK_FTYPE(IPV4_GRE_IPV4_UDP))
				FDIR_SET_FTYPE(IPV4_GRE_IPV4_GTPU);
			else if (FDIR_CHK_FTYPE(IPV4_GRE_IPV6_UDP))
				FDIR_SET_FTYPE(IPV4_GRE_IPV4_GTPU);
			else if (FDIR_CHK_FTYPE(IPV6_GRE_IPV4_UDP))
				FDIR_SET_FTYPE(IPV6_GRE_IPV4_GTPU);
			else if (FDIR_CHK_FTYPE(IPV6_GRE_IPV6_UDP))
				FDIR_SET_FTYPE(IPV6_GRE_IPV4_GTPU);

			if (hdr->field_selector)
				input->gtpu_data.teid =
					*(__force __be32 *)(&rawh[GTPU_TEID_OFFSET]);
			conf->ttype = ICE_FDIR_TUNNEL_TYPE_GTPU;
			if (FDIR_CHK_FTYPE(IPV4_GRE_IPV4_GTPU))
				FDIR_SET_TTYPE(GTPOGRE);
			if (FDIR_CHK_FTYPE(IPV6_GRE_IPV4_GTPU))
				FDIR_SET_TTYPE(GTPOGRE);
			break;
		case VIRTCHNL_PROTO_HDR_GTPU_EH:
			rawh = (u8 *)hdr->buffer;
			if (FDIR_CHK_FTYPE(IPV4_GTPU))
				FDIR_SET_FTYPE(IPV4_GTPU_EH);
			else if (FDIR_CHK_FTYPE(IPV6_GTPU))
				FDIR_SET_FTYPE(IPV6_GTPU_EH);
			else if (FDIR_CHK_FTYPE(IPV4_GRE_IPV4_GTPU))
				FDIR_SET_FTYPE(IPV4_GRE_IPV4_GTPU_EH);
			else if (FDIR_CHK_FTYPE(IPV4_GRE_IPV6_GTPU))
				FDIR_SET_FTYPE(IPV4_GRE_IPV4_GTPU_EH);
			else if (FDIR_CHK_FTYPE(IPV6_GRE_IPV4_GTPU))
				FDIR_SET_FTYPE(IPV6_GRE_IPV4_GTPU_EH);
			else if (FDIR_CHK_FTYPE(IPV6_GRE_IPV6_GTPU))
				FDIR_SET_FTYPE(IPV6_GRE_IPV4_GTPU_EH);

			if (hdr->field_selector)
				input->gtpu_data.qfi =
					rawh[GTPU_EH_QFI_OFFSET] &
					GTPU_EH_QFI_MASK;
			if (conf->ttype != ICE_FDIR_TUNNEL_TYPE_GTPOGRE)
				FDIR_SET_TTYPE(GTPU_EH);
			break;
		case VIRTCHNL_PROTO_HDR_GTPU_EH_PDU_DWN:
			rawh = (u8 *)hdr->buffer;
			if (FDIR_CHK_FTYPE(IPV4_GTPU))
				FDIR_SET_FTYPE(IPV4_GTPU_EH_DW);
			else if (FDIR_CHK_FTYPE(IPV6_GTPU))
				FDIR_SET_FTYPE(IPV6_GTPU_EH_DW);
			else if (FDIR_CHK_FTYPE(IPV4_GRE_IPV4_GTPU))
				FDIR_SET_FTYPE(IPV4_GRE_IPV4_GTPU_EH_DW);
			else if (FDIR_CHK_FTYPE(IPV4_GRE_IPV6_GTPU))
				FDIR_SET_FTYPE(IPV4_GRE_IPV4_GTPU_EH_DW);
			else if (FDIR_CHK_FTYPE(IPV6_GRE_IPV4_GTPU))
				FDIR_SET_FTYPE(IPV6_GRE_IPV4_GTPU_EH_DW);
			else if (FDIR_CHK_FTYPE(IPV6_GRE_IPV6_GTPU))
				FDIR_SET_FTYPE(IPV6_GRE_IPV4_GTPU_EH_DW);

			if (hdr->field_selector)
				input->gtpu_data.qfi =
					rawh[GTPU_EH_QFI_OFFSET] &
					GTPU_EH_QFI_MASK;
			if (conf->ttype != ICE_FDIR_TUNNEL_TYPE_GTPOGRE)
				FDIR_SET_TTYPE(GTPU_EH);
			break;
		case VIRTCHNL_PROTO_HDR_GTPU_EH_PDU_UP:
			rawh = (u8 *)hdr->buffer;
			if (FDIR_CHK_FTYPE(IPV4_GTPU))
				FDIR_SET_FTYPE(IPV4_GTPU_EH_UP);
			else if (FDIR_CHK_FTYPE(IPV6_GTPU))
				FDIR_SET_FTYPE(IPV6_GTPU_EH_UP);
			else if (FDIR_CHK_FTYPE(IPV4_GRE_IPV4_GTPU))
				FDIR_SET_FTYPE(IPV4_GRE_IPV4_GTPU_EH_UP);
			else if (FDIR_CHK_FTYPE(IPV4_GRE_IPV6_GTPU))
				FDIR_SET_FTYPE(IPV4_GRE_IPV4_GTPU_EH_UP);
			else if (FDIR_CHK_FTYPE(IPV6_GRE_IPV4_GTPU))
				FDIR_SET_FTYPE(IPV6_GRE_IPV4_GTPU_EH_UP);
			else if (FDIR_CHK_FTYPE(IPV6_GRE_IPV6_GTPU))
				FDIR_SET_FTYPE(IPV6_GRE_IPV4_GTPU_EH_UP);

			if (hdr->field_selector)
				input->gtpu_data.qfi =
					rawh[GTPU_EH_QFI_OFFSET] &
					GTPU_EH_QFI_MASK;
			if (conf->ttype != ICE_FDIR_TUNNEL_TYPE_GTPOGRE)
				FDIR_SET_TTYPE(GTPU_EH);
			break;
		case VIRTCHNL_PROTO_HDR_ECPRI:
			rawh = (u8 *)hdr->buffer;
			msg_type = rawh[1];
			if (l3 == VIRTCHNL_PROTO_HDR_NONE &&
			    l4 == VIRTCHNL_PROTO_HDR_NONE &&
			    msg_type == 0) {
				input->flow_type =
					ICE_FLTR_PTYPE_NONF_ECPRI_TP0;
				conf->inset_flag |= FDIR_INSET_FLAG_ECPRI_MAC;
			} else if ((l3 == VIRTCHNL_PROTO_HDR_IPV4) &&
				   (l4 == VIRTCHNL_PROTO_HDR_UDP) &&
				   (msg_type == 0)) {
				input->flow_type =
					ICE_FLTR_PTYPE_NONF_IPV4_UDP_ECPRI_TP0;
				conf->inset_flag |= FDIR_INSET_FLAG_ECPRI_UDP;
				conf->ttype = ICE_FDIR_TUNNEL_TYPE_ECPRI;
			} else {
				return -EINVAL;
			}

			if (hdr->field_selector)
				input->ecpri_data.pc_id =
					*(__force __be16 *)(&rawh[4]);
			break;
		case VIRTCHNL_PROTO_HDR_L2TPV2:
			rawh = (u8 *)hdr->buffer;
			if (FDIR_CHK_FTYPE(IPV4_UDP))
				FDIR_SET_FTYPE(IPV4_L2TPV2);
			else if (FDIR_CHK_FTYPE(IPV6_UDP))
				FDIR_SET_FTYPE(IPV6_L2TPV2);

			pos = 0;
			input->l2tpv2_data.flags_version =
				*(__force __be16 *)(&rawh[pos]);
			pos += 2;

			flags_version =
				be16_to_cpu(input->l2tpv2_data.flags_version);
			if (flags_version & ICE_L2TPV2_FLAGS_CTRL) {
				if (FDIR_CHK_FTYPE(IPV4_L2TPV2))
					FDIR_SET_FTYPE(IPV4_L2TPV2_CONTROL);
				else if (FDIR_CHK_FTYPE(IPV6_L2TPV2))
					FDIR_SET_FTYPE(IPV6_L2TPV2_CONTROL);
			}

			if (flags_version & ICE_L2TPV2_FLAGS_LEN) {
				input->l2tpv2_data.length =
					*(__force __be16 *)(&rawh[pos]);
				pos += 2;
			}

			input->l2tpv2_data.tunnel_id =
				*(__force __be16 *)(&rawh[pos]);
			pos += 2;

			input->l2tpv2_data.session_id =
				*(__force __be16 *)(&rawh[pos]);
			pos += 2;

			if (flags_version & ICE_L2TPV2_FLAGS_SEQ) {
				input->l2tpv2_data.ns =
					*(__force __be16 *)(&rawh[pos]);
				pos += 2;

				input->l2tpv2_data.nr =
					*(__force __be16 *)(&rawh[pos]);
				pos += 2;
			}
			/* get l2tpv2 offset */
			if (flags_version & ICE_L2TPV2_FLAGS_OFF) {
				input->l2tpv2_data.offset_size =
					*(__force __be16 *)(&rawh[pos]);
			}

			conf->ttype = ICE_FDIR_TUNNEL_TYPE_L2TPV2;
			break;
		case VIRTCHNL_PROTO_HDR_PPP:
			if (FDIR_CHK_FTYPE(IPV4_L2TPV2))
				FDIR_SET_FTYPE(IPV4_L2TPV2_PPP);
			else if (FDIR_CHK_FTYPE(IPV6_L2TPV2))
				FDIR_SET_FTYPE(IPV6_L2TPV2_PPP);
			break;
		default:
			dev_dbg(dev, "Invalid header type 0x:%x for VF %d\n",
				hdr->type, vf->vf_id);
			return -EINVAL;
		}
	}

	return 0;
}

/**
 * ice_vc_fdir_parse_action
 * @vf: pointer to the VF info
 * @fltr: virtual channel add cmd buffer
 * @conf: FDIR configuration for each filter
 *
 * Parse the virtual channel filter's action and store them into @conf
 *
 * Return: 0 on success, and other on error.
 */
static int
ice_vc_fdir_parse_action(struct ice_vf *vf,
			 struct virtchnl_fdir_add *fltr,
			 struct virtchnl_fdir_fltr_conf *conf)
{
	struct virtchnl_filter_action_set *as = &fltr->rule_cfg.action_set;
	struct device *dev = ice_pf_to_dev(vf->pf);
	struct ice_fdir_fltr *input = &conf->input;
	u32 dest_num = 0;
	u32 mark_num = 0;
	int i;

	if (as->count > VIRTCHNL_MAX_NUM_ACTIONS) {
		dev_dbg(dev, "Invalid action numbers:0x%x for VF %d\n",
			as->count, vf->vf_id);
		return -EINVAL;
	}

	for (i = 0; i < as->count; i++) {
		struct virtchnl_filter_action *action = &as->actions[i];

		switch (action->type) {
		case VIRTCHNL_ACTION_PASSTHRU:
			dest_num++;
			input->dest_ctl =
				ICE_FLTR_PRGM_DESC_DEST_DIRECT_PKT_OTHER;
			break;
		case VIRTCHNL_ACTION_DROP:
			dest_num++;
			input->dest_ctl =
				ICE_FLTR_PRGM_DESC_DEST_DROP_PKT;
			break;
		case VIRTCHNL_ACTION_QUEUE:
			dest_num++;
			input->dest_ctl =
				ICE_FLTR_PRGM_DESC_DEST_DIRECT_PKT_QINDEX;
			input->q_index = action->act_conf.queue.index;
			break;
		case VIRTCHNL_ACTION_Q_REGION:
			dest_num++;
			input->dest_ctl =
				ICE_FLTR_PRGM_DESC_DEST_DIRECT_PKT_QGROUP;
			input->q_index = action->act_conf.queue.index;
			input->q_region = action->act_conf.queue.region;
			break;
		case VIRTCHNL_ACTION_MARK:
			mark_num++;
			input->fltr_id = action->act_conf.mark_id;
			input->fdid_prio = ICE_FXD_FLTR_QW1_FDID_PRI_THREE;
			break;
		default:
			dev_dbg(dev, "Invalid action type:0x%x for VF %d\n",
				action->type, vf->vf_id);
			return -EINVAL;
		}
	}

	if (dest_num == 0 || dest_num >= 2) {
		dev_dbg(dev, "Invalid destination action for VF %d\n",
			vf->vf_id);
		return -EINVAL;
	}

	if (mark_num >= 2) {
		dev_dbg(dev, "Too many mark actions for VF %d\n", vf->vf_id);
		return -EINVAL;
	}

	return 0;
}

/**
 * ice_vc_validate_fdir_fltr - validate the virtual channel filter
 * @vf: pointer to the VF info
 * @fltr: virtual channel add cmd buffer
 * @conf: FDIR configuration for each filter
 *
 * Return: 0 on success, and other on error.
 */
static int
ice_vc_validate_fdir_fltr(struct ice_vf *vf,
			  struct virtchnl_fdir_add *fltr,
			  struct virtchnl_fdir_fltr_conf *conf)
{
	struct virtchnl_proto_hdrs *proto = &fltr->rule_cfg.proto_hdrs;
	int ret;

	/* For Protocol Agnostic Flow Offloading case only */
	if (!ice_vc_fdir_is_raw_flow(proto))
		if (!ice_vc_validate_pattern(vf, proto))
			return -EINVAL;

	ret = ice_vc_fdir_parse_pattern(vf, fltr, conf);
	if (ret)
		return ret;

	ret = ice_vc_fdir_parse_action(vf, fltr, conf);
	if (ret)
		return ret;

	return 0;
}

/**
 * ice_vc_fdir_comp_rules - compare if two filter rules have the same value
 * @conf_a: FDIR configuration for filter a
 * @conf_b: FDIR configuration for filter b
 *
 * Return: 0 on success, and other on error.
 */
static bool
ice_vc_fdir_comp_rules(struct virtchnl_fdir_fltr_conf *conf_a,
		       struct virtchnl_fdir_fltr_conf *conf_b)
{
	struct ice_fdir_fltr *a = &conf_a->input;
	struct ice_fdir_fltr *b = &conf_b->input;

	if (conf_a->ttype != conf_b->ttype)
		return false;
	return ice_fdir_comp_rules_extended(a, b);
}

/**
 * ice_vc_fdir_is_dup_fltr
 * @vf: pointer to the VF info
 * @conf: FDIR configuration for each filter
 *
 * Check if there is duplicated rule with same @conf value
 *
 * Return: 0 true success, and false on error.
 */
static bool
ice_vc_fdir_is_dup_fltr(struct ice_vf *vf,
			struct virtchnl_fdir_fltr_conf *conf)
{
	struct ice_fdir_fltr *desc;

	list_for_each_entry(desc, &vf->fdir.fdir_rule_list, fltr_node) {
		struct virtchnl_fdir_fltr_conf *node =
				to_fltr_conf_from_desc(desc);

		if (ice_vc_fdir_comp_rules(node, conf))
			return true;
	}

	return false;
}

/**
 * ice_vc_fdir_insert_entry
 * @vf: pointer to the VF info
 * @conf: FDIR configuration for each filter
 * @id: pointer to ID value allocated by driver
 *
 * Insert FDIR conf entry into list and allocate ID for this filter
 *
 * Return: 0 true success, and other on error.
 */
static int
ice_vc_fdir_insert_entry(struct ice_vf *vf,
			 struct virtchnl_fdir_fltr_conf *conf,
			 u32 *id)
{
	struct ice_fdir_fltr *input = &conf->input;
	int i;

	/* alloc ID corresponding with conf */
	i = idr_alloc(&vf->fdir.fdir_rule_idr, conf, 0,
		      ICE_FDIR_MAX_FLTRS, GFP_KERNEL);
	if (i < 0)
		return -EINVAL;
	*id = i;

	list_add(&input->fltr_node, &vf->fdir.fdir_rule_list);
	return 0;
}

/**
 * ice_vc_fdir_remove_entry - remove FDIR conf entry by ID value
 * @vf: pointer to the VF info
 * @conf: FDIR configuration for each filter
 * @id: filter rule's ID
 */
static void
ice_vc_fdir_remove_entry(struct ice_vf *vf,
			 struct virtchnl_fdir_fltr_conf *conf,
			 u32 id)
{
	struct ice_fdir_fltr *input = &conf->input;

	idr_remove(&vf->fdir.fdir_rule_idr, id);
	list_del(&input->fltr_node);
}

/**
 * ice_vc_fdir_lookup_entry - lookup FDIR conf entry by ID value
 * @vf: pointer to the VF info
 * @id: filter rule's ID
 *
 * Return: NULL on error, and other on success.
 */
static struct virtchnl_fdir_fltr_conf *
ice_vc_fdir_lookup_entry(struct ice_vf *vf, u32 id)
{
	return idr_find(&vf->fdir.fdir_rule_idr, id);
}

/**
 * ice_vc_fdir_flush_entry - remove all FDIR conf entry
 * @vf: pointer to the VF info
 */
static void ice_vc_fdir_flush_entry(struct ice_vf *vf)
{
	struct ice_fdir_fltr *desc, *temp;

	list_for_each_entry_safe(desc, temp,
				 &vf->fdir.fdir_rule_list, fltr_node) {
		struct virtchnl_fdir_fltr_conf *conf =
				to_fltr_conf_from_desc(desc);

		list_del(&desc->fltr_node);
		kfree(conf);
	}
}

/**
 * ice_vc_fdir_add_del_raw - write raw flow filter rule into hardware
 * @vf: pointer to the VF info
 * @conf: FDIR configuration for each filter
 * @add: true implies add rule, false implies del rules
 *
 * Return: 0 on success, and other on error.
 */
static int ice_vc_fdir_add_del_raw(struct ice_vf *vf,
				   struct virtchnl_fdir_fltr_conf *conf,
				   bool add)
{
	struct ice_fdir_fltr *input = &conf->input;
	struct ice_vsi *vsi, *ctrl_vsi;
	struct ice_fltr_desc desc;
	struct device *dev;
	struct ice_pf *pf;
	struct ice_hw *hw;
	int ret;
	u8 *pkt;

	pf = vf->pf;
	dev = ice_pf_to_dev(pf);
	hw = &pf->hw;
	vsi = ice_get_vf_vsi(vf);
	if (!vsi) {
		dev_dbg(dev, "Invalid vsi for VF %d\n", vf->vf_id);
		return -EINVAL;
	}

	input->dest_vsi = vsi->idx;
	input->comp_report = ICE_FXD_FLTR_QW0_COMP_REPORT_SW;

	ctrl_vsi = pf->vsi[vf->ctrl_vsi_idx];
	if (!ctrl_vsi) {
		dev_dbg(dev, "Invalid ctrl_vsi for VF %d\n", vf->vf_id);
		return -EINVAL;
	}

	pkt = devm_kzalloc(dev, ICE_FDIR_MAX_RAW_PKT_SIZE, GFP_KERNEL);
	if (!pkt)
		return -ENOMEM;

	memcpy(pkt, conf->pkt_buf, conf->pkt_len);

	ice_fdir_get_prgm_desc(hw, input, &desc, add);

	ret = ice_prgm_fdir_fltr(ctrl_vsi, &desc, pkt);
	if (ret)
		goto err_free_pkt;

	return 0;

err_free_pkt:
	devm_kfree(dev, pkt);
	return ret;
}

/**
 * ice_vc_fdir_write_fltr - write filter rule into hardware
 * @vf: pointer to the VF info
 * @conf: FDIR configuration for each filter
 * @add: true implies add rule, false implies del rules
 * @is_tun: false implies non-tunnel type filter, true implies tunnel filter
 *
 * Return: 0 on success, and other on error.
 */
static int ice_vc_fdir_write_fltr(struct ice_vf *vf,
				  struct virtchnl_fdir_fltr_conf *conf,
				  bool add,
				  bool is_tun)
{
	struct ice_fdir_fltr *input = &conf->input;
	struct ice_vsi *vsi, *ctrl_vsi;
	struct ice_fltr_desc desc;
	struct device *dev;
	struct ice_pf *pf;
	struct ice_hw *hw;
	int ret;
	u8 *pkt;

	pf = vf->pf;
	dev = ice_pf_to_dev(pf);
	hw = &pf->hw;
	vsi = ice_get_vf_vsi(vf);
	if (!vsi) {
		dev_dbg(dev, "Invalid vsi for VF %d\n", vf->vf_id);
		return -EINVAL;
	}

	input->dest_vsi = vsi->idx;
	input->comp_report = ICE_FXD_FLTR_QW0_COMP_REPORT_SW;

	ctrl_vsi = pf->vsi[vf->ctrl_vsi_idx];
	if (!ctrl_vsi) {
		dev_dbg(dev, "Invalid ctrl_vsi for VF %d\n", vf->vf_id);
		return -EINVAL;
	}

	pkt = devm_kzalloc(dev, ICE_FDIR_MAX_RAW_PKT_SIZE, GFP_KERNEL);
	if (!pkt)
		return -ENOMEM;

	ice_fdir_get_prgm_desc(hw, input, &desc, add);
	ret = ice_fdir_get_gen_prgm_pkt(hw, input, pkt, false, is_tun);
	if (ret) {
		dev_dbg(dev, "Gen training pkt for VF %d ptype %d failed\n",
			vf->vf_id, input->flow_type);
		goto err_free_pkt;
	}

	ret = ice_prgm_fdir_fltr(ctrl_vsi, &desc, pkt);
	if (ret)
		goto err_free_pkt;

	return 0;

err_free_pkt:
	devm_kfree(dev, pkt);
	return ret;
}

/**
 * ice_vf_fdir_timer - FDIR program waiting timer interrupt handler
 * @t: pointer to timer_list
 */
static void ice_vf_fdir_timer(struct timer_list *t)
{
	struct ice_vf_fdir_ctx *ctx_irq = from_timer(ctx_irq, t, rx_tmr);
	struct ice_vf_fdir_ctx *ctx_done;
	struct ice_vf_fdir *fdir;
	unsigned long flags;
	struct ice_vf *vf;
	struct ice_pf *pf;

	fdir = container_of(ctx_irq, struct ice_vf_fdir, ctx_irq);
	vf = container_of(fdir, struct ice_vf, fdir);
	ctx_done = &fdir->ctx_done;
	pf = vf->pf;
	spin_lock_irqsave(&fdir->ctx_lock, flags);
	if (!(ctx_irq->flags & ICE_VF_FDIR_CTX_VALID)) {
		spin_unlock_irqrestore(&fdir->ctx_lock, flags);
		WARN_ON_ONCE(1);
		return;
	}

	ctx_irq->flags &= ~ICE_VF_FDIR_CTX_VALID;

	ctx_done->flags |= ICE_VF_FDIR_CTX_VALID;
	ctx_done->conf = ctx_irq->conf;
	ctx_done->stat = ICE_FDIR_CTX_TIMEOUT;
	ctx_done->v_opcode = ctx_irq->v_opcode;
	spin_unlock_irqrestore(&fdir->ctx_lock, flags);

	set_bit(ICE_FD_VF_FLUSH_CTX, pf->state);
	ice_service_task_schedule(pf);
}

/**
 * ice_vc_fdir_irq_handler - ctrl_vsi Rx queue interrupt handler
 * @ctrl_vsi: pointer to a VF's CTRL VSI
 * @rx_desc: pointer to FDIR Rx queue descriptor
 */
void
ice_vc_fdir_irq_handler(struct ice_vsi *ctrl_vsi,
			union ice_32b_rx_flex_desc *rx_desc)
{
	struct ice_pf *pf = ctrl_vsi->back;
	struct ice_vf *vf = ctrl_vsi->vf;
	struct ice_vf_fdir_ctx *ctx_done;
	struct ice_vf_fdir_ctx *ctx_irq;
	struct ice_vf_fdir *fdir;
	unsigned long flags;
	struct device *dev;
	int ret;

	if (WARN_ON(!vf))
		return;

	fdir = &vf->fdir;
	ctx_done = &fdir->ctx_done;
	ctx_irq = &fdir->ctx_irq;
	dev = ice_pf_to_dev(pf);
	spin_lock_irqsave(&fdir->ctx_lock, flags);
	if (!(ctx_irq->flags & ICE_VF_FDIR_CTX_VALID)) {
		spin_unlock_irqrestore(&fdir->ctx_lock, flags);
		WARN_ON_ONCE(1);
		return;
	}

	ctx_irq->flags &= ~ICE_VF_FDIR_CTX_VALID;

	ctx_done->flags |= ICE_VF_FDIR_CTX_VALID;
	ctx_done->conf = ctx_irq->conf;
	ctx_done->stat = ICE_FDIR_CTX_IRQ;
	ctx_done->v_opcode = ctx_irq->v_opcode;
	memcpy(&ctx_done->rx_desc, rx_desc, sizeof(*rx_desc));
	spin_unlock_irqrestore(&fdir->ctx_lock, flags);

	ret = del_timer(&ctx_irq->rx_tmr);
	if (!ret)
		dev_err(dev, "VF %d: Unexpected inactive timer!\n", vf->vf_id);

	set_bit(ICE_FD_VF_FLUSH_CTX, pf->state);
	ice_service_task_schedule(pf);
}

/**
 * ice_vf_fdir_dump_info - dump FDIR information for diagnosis
 * @vf: pointer to the VF info
 */
static void ice_vf_fdir_dump_info(struct ice_vf *vf)
{
	struct ice_vsi *vf_vsi;
	u32 fd_size, fd_cnt;
	struct device *dev;
	struct ice_pf *pf;
	struct ice_hw *hw;
	u16 vsi_num;

	pf = vf->pf;
	hw = &pf->hw;
	dev = ice_pf_to_dev(pf);
	vf_vsi = ice_get_vf_vsi(vf);
	if (!vf_vsi) {
		dev_dbg(dev, "VF %d: invalid VSI pointer\n", vf->vf_id);
		return;
	}

	vsi_num = ice_get_hw_vsi_num(hw, vf_vsi->idx);

	fd_size = rd32(hw, VSIQF_FD_SIZE(vsi_num));
	fd_cnt = rd32(hw, VSIQF_FD_CNT(vsi_num));
	dev_dbg(dev, "VF %d: space allocated: guar:0x%lx, be:0x%lx, space consumed: guar:0x%lx, be:0x%lx\n",
		vf->vf_id,
		FIELD_GET(VSIQF_FD_CNT_FD_GCNT_M, fd_size),
		FIELD_GET(VSIQF_FD_CNT_FD_BCNT_M, fd_size),
		FIELD_GET(VSIQF_FD_CNT_FD_GCNT_M, fd_cnt),
		FIELD_GET(VSIQF_FD_CNT_FD_BCNT_M, fd_cnt));
}

/**
 * ice_vf_verify_rx_desc - verify received FDIR programming status descriptor
 * @vf: pointer to the VF info
 * @ctx: FDIR context info for post processing
 * @status: virtchnl FDIR program status
 *
 * Return: 0 on success, and other on error.
 */
static int
ice_vf_verify_rx_desc(struct ice_vf *vf,
		      struct ice_vf_fdir_ctx *ctx,
		      enum virtchnl_fdir_prgm_status *status)
{
	struct device *dev = ice_pf_to_dev(vf->pf);
	u32 stat_err, error, prog_id;
	int ret;

	stat_err = le16_to_cpu(ctx->rx_desc.wb.status_error0);
	if (FIELD_GET(ICE_FXD_FLTR_WB_QW1_DD_M, stat_err) !=
	    ICE_FXD_FLTR_WB_QW1_DD_YES) {
		*status = VIRTCHNL_FDIR_FAILURE_RULE_NORESOURCE;
		dev_err(dev, "VF %d: Desc Done not set\n", vf->vf_id);
		ret = -EINVAL;
		goto err_exit;
	}

	prog_id = FIELD_GET(ICE_FXD_FLTR_WB_QW1_PROG_ID_M, stat_err);
	if (prog_id == ICE_FXD_FLTR_WB_QW1_PROG_ADD &&
	    ctx->v_opcode != VIRTCHNL_OP_ADD_FDIR_FILTER) {
		dev_err(dev, "VF %d: Desc show add, but ctx not",
			vf->vf_id);
		*status = VIRTCHNL_FDIR_FAILURE_RULE_INVALID;
		ret = -EINVAL;
		goto err_exit;
	}

	if (prog_id == ICE_FXD_FLTR_WB_QW1_PROG_DEL &&
	    ctx->v_opcode != VIRTCHNL_OP_DEL_FDIR_FILTER) {
		dev_err(dev, "VF %d: Desc show del, but ctx not",
			vf->vf_id);
		*status = VIRTCHNL_FDIR_FAILURE_RULE_INVALID;
		ret = -EINVAL;
		goto err_exit;
	}

	error = FIELD_GET(ICE_FXD_FLTR_WB_QW1_FAIL_M, stat_err);
	if (error == ICE_FXD_FLTR_WB_QW1_FAIL_YES) {
		if (prog_id == ICE_FXD_FLTR_WB_QW1_PROG_ADD) {
			dev_err(dev, "VF %d, Failed to add FDIR rule due to no space in the table",
				vf->vf_id);
			*status = VIRTCHNL_FDIR_FAILURE_RULE_NORESOURCE;
		} else {
			dev_err(dev, "VF %d, Failed to remove FDIR rule, attempt to remove non-existent entry",
				vf->vf_id);
			*status = VIRTCHNL_FDIR_FAILURE_RULE_NONEXIST;
		}
		ret = -EINVAL;
		goto err_exit;
	}

	error = FIELD_GET(ICE_FXD_FLTR_WB_QW1_FAIL_PROF_M, stat_err);
	if (error == ICE_FXD_FLTR_WB_QW1_FAIL_PROF_YES) {
		dev_err(dev, "VF %d: Profile matching error", vf->vf_id);
		*status = VIRTCHNL_FDIR_FAILURE_RULE_NORESOURCE;
		ret = -EINVAL;
		goto err_exit;
	}

	*status = VIRTCHNL_FDIR_SUCCESS;

	return 0;

err_exit:
	ice_vf_fdir_dump_info(vf);
	return ret;
}

static int ice_fdir_is_tunnel(enum ice_fdir_tunnel_type ttype)
{
	return (ttype == ICE_FDIR_TUNNEL_TYPE_GRE_INNER ||
		ttype == ICE_FDIR_TUNNEL_TYPE_GTPU_INNER ||
		ttype == ICE_FDIR_TUNNEL_TYPE_GTPU_EH_INNER ||
		ttype == ICE_FDIR_TUNNEL_TYPE_GTPOGRE_INNER ||
		ttype == ICE_FDIR_TUNNEL_TYPE_ECPRI ||
		ttype == ICE_FDIR_TUNNEL_TYPE_L2TPV2_INNER);
}

/**
 * ice_vc_add_fdir_fltr_post
 * @vf: pointer to the VF structure
 * @ctx: FDIR context info for post processing
 * @status: virtchnl FDIR program status
 * @success: true implies success, false implies failure
 *
 * Post process for flow director add command. If success, then do post process
 * and send back success msg by virtchnl. Otherwise, do context reversion and
 * send back failure msg by virtchnl.
 *
 * Return: 0 on success, and other on error.
 */
static int
ice_vc_add_fdir_fltr_post(struct ice_vf *vf,
			  struct ice_vf_fdir_ctx *ctx,
			  enum virtchnl_fdir_prgm_status status,
			  bool success)
{
	struct virtchnl_fdir_fltr_conf *conf = ctx->conf;
	struct device *dev = ice_pf_to_dev(vf->pf);
	enum virtchnl_status_code v_ret;
	struct virtchnl_fdir_add *resp;
	int ret, len, is_tun;

	v_ret = VIRTCHNL_STATUS_SUCCESS;
	len = sizeof(*resp);
	resp = kzalloc(sizeof(*resp), GFP_KERNEL);
	if (!resp) {
		len = 0;
		v_ret = VIRTCHNL_STATUS_ERR_NO_MEMORY;
		dev_dbg(dev, "VF %d: Alloc resp buf fail", vf->vf_id);
		goto err_exit;
	}

	if (!success)
		goto err_exit;

	is_tun = ice_fdir_is_tunnel(conf->ttype);
	resp->status = status;
	resp->flow_id = conf->flow_id;
	vf->fdir.fdir_fltr_cnt[conf->input.flow_type][is_tun]++;

	ret = ice_vc_send_msg_to_vf(vf, ctx->v_opcode, v_ret,
				    (u8 *)resp, len);
	kfree(resp);

	dev_dbg(dev, "VF %d: flow_id:0x%X, FDIR %s success!\n",
		vf->vf_id, conf->flow_id,
		(ctx->v_opcode == VIRTCHNL_OP_ADD_FDIR_FILTER) ?
		"add" : "del");
	return ret;

err_exit:
	if (resp)
		resp->status = status;
	ice_vc_fdir_remove_entry(vf, conf, conf->flow_id);
	kfree(conf);

	ret = ice_vc_send_msg_to_vf(vf, ctx->v_opcode, v_ret,
				    (u8 *)resp, len);
	kfree(resp);
	return ret;
}

/**
 * ice_vc_del_fdir_fltr_post
 * @vf: pointer to the VF structure
 * @ctx: FDIR context info for post processing
 * @status: virtchnl FDIR program status
 * @success: true implies success, false implies failure
 *
 * Post process for flow director del command. If success, then do post process
 * and send back success msg by virtchnl. Otherwise, do context reversion and
 * send back failure msg by virtchnl.
 *
 * Return: 0 on success, and other on error.
 */
static int
ice_vc_del_fdir_fltr_post(struct ice_vf *vf,
			  struct ice_vf_fdir_ctx *ctx,
			  enum virtchnl_fdir_prgm_status status,
			  bool success)
{
	struct virtchnl_fdir_fltr_conf *conf = ctx->conf;
	struct device *dev = ice_pf_to_dev(vf->pf);
	enum virtchnl_status_code v_ret;
	struct virtchnl_fdir_del *resp;
	int ret, len, is_tun;

	v_ret = VIRTCHNL_STATUS_SUCCESS;
	len = sizeof(*resp);
	resp = kzalloc(sizeof(*resp), GFP_KERNEL);
	if (!resp) {
		len = 0;
		v_ret = VIRTCHNL_STATUS_ERR_NO_MEMORY;
		dev_dbg(dev, "VF %d: Alloc resp buf fail", vf->vf_id);
		goto err_exit;
	}

	if (!success)
		goto err_exit;

	is_tun = ice_fdir_is_tunnel(conf->ttype);
	resp->status = status;
	ice_vc_fdir_remove_entry(vf, conf, conf->flow_id);
	vf->fdir.fdir_fltr_cnt[conf->input.flow_type][is_tun]--;

	ret = ice_vc_send_msg_to_vf(vf, ctx->v_opcode, v_ret,
				    (u8 *)resp, len);
	kfree(resp);

	dev_dbg(dev, "VF %d: flow_id:0x%X, FDIR %s success!\n",
		vf->vf_id, conf->flow_id,
		(ctx->v_opcode == VIRTCHNL_OP_ADD_FDIR_FILTER) ?
		"add" : "del");
	kfree(conf);
	return ret;

err_exit:
	if (resp)
		resp->status = status;
	if (success)
		kfree(conf);

	ret = ice_vc_send_msg_to_vf(vf, ctx->v_opcode, v_ret,
				    (u8 *)resp, len);
	kfree(resp);
	return ret;
}

/**
 * ice_flush_fdir_ctx
 * @pf: pointer to the PF structure
 *
 * Flush all the pending event on ctx_done list and process them.
 */
void ice_flush_fdir_ctx(struct ice_pf *pf)
{
	struct ice_vf *vf;
	unsigned int bkt;

	if (!test_and_clear_bit(ICE_FD_VF_FLUSH_CTX, pf->state))
		return;

	mutex_lock(&pf->vfs.table_lock);
	ice_for_each_vf(pf, bkt, vf) {
		struct device *dev = ice_pf_to_dev(pf);
		enum virtchnl_fdir_prgm_status status;
		struct ice_vf_fdir_ctx *ctx;
		unsigned long flags;
		int ret;

		if (!test_bit(ICE_VF_STATE_ACTIVE, vf->vf_states))
			continue;

		if (vf->ctrl_vsi_idx == ICE_NO_VSI)
			continue;

		ctx = &vf->fdir.ctx_done;
		spin_lock_irqsave(&vf->fdir.ctx_lock, flags);
		if (!(ctx->flags & ICE_VF_FDIR_CTX_VALID)) {
			spin_unlock_irqrestore(&vf->fdir.ctx_lock, flags);
			continue;
		}
		spin_unlock_irqrestore(&vf->fdir.ctx_lock, flags);

		WARN_ON(ctx->stat == ICE_FDIR_CTX_READY);
		if (ctx->stat == ICE_FDIR_CTX_TIMEOUT) {
			status = VIRTCHNL_FDIR_FAILURE_RULE_TIMEOUT;
			dev_err(dev, "VF %d: ctrl_vsi irq timeout\n",
				vf->vf_id);
			goto err_exit;
		}

		ret = ice_vf_verify_rx_desc(vf, ctx, &status);
		if (ret)
			goto err_exit;

		if (ctx->v_opcode == VIRTCHNL_OP_ADD_FDIR_FILTER)
			ice_vc_add_fdir_fltr_post(vf, ctx, status, true);
		else if (ctx->v_opcode == VIRTCHNL_OP_DEL_FDIR_FILTER)
			ice_vc_del_fdir_fltr_post(vf, ctx, status, true);
		else
			dev_err(dev, "VF %d: Unsupported opcode\n", vf->vf_id);

		spin_lock_irqsave(&vf->fdir.ctx_lock, flags);
		ctx->flags &= ~ICE_VF_FDIR_CTX_VALID;
		spin_unlock_irqrestore(&vf->fdir.ctx_lock, flags);
		continue;
err_exit:
		if (ctx->v_opcode == VIRTCHNL_OP_ADD_FDIR_FILTER)
			ice_vc_add_fdir_fltr_post(vf, ctx, status, false);
		else if (ctx->v_opcode == VIRTCHNL_OP_DEL_FDIR_FILTER)
			ice_vc_del_fdir_fltr_post(vf, ctx, status, false);
		else
			dev_err(dev, "VF %d: Unsupported opcode\n", vf->vf_id);

		spin_lock_irqsave(&vf->fdir.ctx_lock, flags);
		ctx->flags &= ~ICE_VF_FDIR_CTX_VALID;
		spin_unlock_irqrestore(&vf->fdir.ctx_lock, flags);
	}
	mutex_unlock(&pf->vfs.table_lock);
}

/**
 * ice_vc_fdir_set_irq_ctx - set FDIR context info for later irq handler
 * @vf: pointer to the VF structure
 * @conf: FDIR configuration for each filter
 * @v_opcode: virtual channel operation code
 *
 * Return: 0 on success, and other on error.
 */
static int
ice_vc_fdir_set_irq_ctx(struct ice_vf *vf,
			struct virtchnl_fdir_fltr_conf *conf,
			enum virtchnl_ops v_opcode)
{
	struct device *dev = ice_pf_to_dev(vf->pf);
	struct ice_vf_fdir_ctx *ctx;
	unsigned long flags;

	ctx = &vf->fdir.ctx_irq;
	spin_lock_irqsave(&vf->fdir.ctx_lock, flags);
	if ((vf->fdir.ctx_irq.flags & ICE_VF_FDIR_CTX_VALID) ||
	    (vf->fdir.ctx_done.flags & ICE_VF_FDIR_CTX_VALID)) {
		spin_unlock_irqrestore(&vf->fdir.ctx_lock, flags);
		dev_dbg(dev, "VF %d: Last request is still in progress\n",
			vf->vf_id);
		return -EBUSY;
	}
	ctx->flags |= ICE_VF_FDIR_CTX_VALID;
	spin_unlock_irqrestore(&vf->fdir.ctx_lock, flags);

	ctx->conf = conf;
	ctx->v_opcode = v_opcode;
	ctx->stat = ICE_FDIR_CTX_READY;
	timer_setup(&ctx->rx_tmr, ice_vf_fdir_timer, 0);

	mod_timer(&ctx->rx_tmr,
		  round_jiffies(msecs_to_jiffies(10) + jiffies));

	return 0;
}

/**
 * ice_vc_fdir_clear_irq_ctx - clear FDIR context info for irq handler
 * @vf: pointer to the VF structure
 *
 * Return: 0 on success, and other on error.
 */
static void ice_vc_fdir_clear_irq_ctx(struct ice_vf *vf)
{
	struct ice_vf_fdir_ctx *ctx = &vf->fdir.ctx_irq;
	unsigned long flags;

	del_timer(&ctx->rx_tmr);
	spin_lock_irqsave(&vf->fdir.ctx_lock, flags);
	ctx->flags &= ~ICE_VF_FDIR_CTX_VALID;
	spin_unlock_irqrestore(&vf->fdir.ctx_lock, flags);
}

/**
 * ice_vc_parser_fv_check_diff -  check two parsed FDIR profile fv context
 * @fv_a: struct of parsed FDIR profile field vector
 * @fv_b: struct of parsed FDIR profile field vector
 *
 * Check if the two parsed FDIR profile field vector context are different,
 * including proto_id, offset and mask.
 *
 * Return: true on differnet, false on otherwise.
 */
static bool ice_vc_parser_fv_check_diff(struct ice_parser_fv *fv_a,
					struct ice_parser_fv *fv_b)
{
	return (fv_a->proto_id	!= fv_b->proto_id ||
		fv_a->offset	!= fv_b->offset ||
		fv_a->msk	!= fv_b->msk);
}

/**
 * ice_vc_parser_fv_save -  save parsed FDIR profile fv context
 * @fv: struct of parsed FDIR profile field vector
 * @fv_src: parsed FDIR profile field vector context to save
 *
 * Save the parsed FDIR profile field vector context, including proto_id,
 * offset and mask.
 */
static void ice_vc_parser_fv_save(struct ice_parser_fv *fv,
				  struct ice_parser_fv *fv_src)
{
	fv->proto_id	= fv_src->proto_id;
	fv->offset	= fv_src->offset;
	fv->msk		= fv_src->msk;
}

/**
 * ice_vc_add_fdir_raw - add a raw FDIR filter for VF
 * @vf: pointer to the VF info
 * @conf: FDIR configuration for each filter
 * @stat: pointer to the VIRTCHNL_OP_ADD_FDIR_FILTER
 * @len: length of the stat
 *
 * Return: 0 on success, and other on error.
 */
static int
ice_vc_add_fdir_raw(struct ice_vf *vf,
		    struct virtchnl_fdir_fltr_conf *conf,
		    struct virtchnl_fdir_add *stat, int len)
{
	struct ice_vsi *vf_vsi, *ctrl_vsi;
	enum virtchnl_status_code v_ret;
	struct ice_fdir_prof_info *pi;
	struct ice_pf *pf = vf->pf;
	int ret, ptg, id, i;
	struct device *dev;
	struct ice_hw *hw;
	bool fv_found;

	dev = ice_pf_to_dev(pf);
	hw = &pf->hw;

	id = find_first_bit(conf->prof->ptypes, ICE_FLOW_PTYPE_MAX);
	ptg = hw->blk[ICE_BLK_FD].xlt1.t[id];

	v_ret = VIRTCHNL_STATUS_SUCCESS;
	vf_vsi = ice_get_vf_vsi(vf);
	if (!vf_vsi) {
		v_ret = VIRTCHNL_STATUS_ERR_PARAM;
		stat->status = VIRTCHNL_FDIR_FAILURE_RULE_NORESOURCE;
		dev_err(dev, "Can not get FDIR vf_vsi for VF %d\n", vf->vf_id);
		goto err_exit;
	}

	ctrl_vsi = pf->vsi[vf->ctrl_vsi_idx];
	if (!ctrl_vsi) {
		v_ret = VIRTCHNL_STATUS_ERR_PARAM;
		stat->status = VIRTCHNL_FDIR_FAILURE_RULE_NORESOURCE;
		dev_err(dev, "Can not get FDIR ctrl_vsi for VF %d\n",
			vf->vf_id);
		goto err_exit;
	}

	fv_found = false;

	/* Check if profile info already existed, then update the counter */
	pi = &vf->fdir_prof_info[ptg];
	if (pi->fdir_active_cnt != 0) {
		for (i = 0; i < ICE_MAX_FV_WORDS; i++)
			if (ice_vc_parser_fv_check_diff(&pi->prof.fv[i],
							&conf->prof->fv[i]))
				break;
		if (i == ICE_MAX_FV_WORDS) {
			fv_found = true;
			pi->fdir_active_cnt++;
		}
	}

	/* HW profile setting is only required for the first time */
	if (!fv_found) {
		ret = ice_flow_set_hw_prof(hw, vf_vsi->idx,
					   ctrl_vsi->idx, conf->prof,
					   ICE_BLK_FD);

		if (ret)
			goto err_free_conf;
	}

	ret = ice_vc_fdir_insert_entry(vf, conf, &conf->flow_id);
	if (ret) {
		v_ret = VIRTCHNL_STATUS_SUCCESS;
		stat->status = VIRTCHNL_FDIR_FAILURE_RULE_NORESOURCE;
		dev_dbg(dev, "VF %d: insert FDIR list failed\n",
			vf->vf_id);
		goto err_free_conf;
	}

	ret = ice_vc_fdir_set_irq_ctx(vf, conf,
				      VIRTCHNL_OP_ADD_FDIR_FILTER);
	if (ret) {
		v_ret = VIRTCHNL_STATUS_SUCCESS;
		stat->status = VIRTCHNL_FDIR_FAILURE_RULE_NORESOURCE;
		dev_dbg(dev, "VF %d: set FDIR context failed\n",
			vf->vf_id);
		goto err_rem_entry;
	}

	ret = ice_vc_fdir_add_del_raw(vf, conf, true);
	if (ret) {
		v_ret = VIRTCHNL_STATUS_SUCCESS;
		stat->status = VIRTCHNL_FDIR_FAILURE_RULE_NORESOURCE;
		ice_dev_err_errno(dev, ret, "VF %d: adding FDIR raw flow rule failed",
				  vf->vf_id);
		goto err_clr_irq;
	}

	/* Save parsed profile fv info of the FDIR rule for the first time */
	if (!fv_found) {
		for (i = 0; i < conf->prof->fv_num; i++)
			ice_vc_parser_fv_save(&pi->prof.fv[i],
					      &conf->prof->fv[i]);
		pi->fdir_active_cnt = 1;
	}

	return 0;

err_clr_irq:
	ice_vc_fdir_clear_irq_ctx(vf);
err_rem_entry:
	ice_vc_fdir_remove_entry(vf, conf, conf->flow_id);
err_free_conf:
	if (conf->parser_ena)
		conf->parser_ena = false;
	kfree(conf->prof);
	kfree(conf->pkt_buf);
	kfree(conf);
err_exit:
	ret = ice_vc_send_msg_to_vf(vf, VIRTCHNL_OP_ADD_FDIR_FILTER, v_ret,
				    (u8 *)stat, len);
	kfree(stat);
	return ret;
}

/**
 * ice_vc_add_fdir_fltr - add a FDIR filter for VF by the msg buffer
 * @vf: pointer to the VF info
 * @msg: pointer to the msg buffer
 *
 * Return: 0 on success, and other on error.
 */
int ice_vc_add_fdir_fltr(struct ice_vf *vf, u8 *msg)
{
	struct virtchnl_fdir_add *fltr = (struct virtchnl_fdir_add *)msg;
	struct virtchnl_fdir_add *stat = NULL;
	struct virtchnl_fdir_fltr_conf *conf;
	enum virtchnl_status_code v_ret;
	struct device *dev;
	struct ice_pf *pf;
	int is_tun = 0;
	int len = 0;
	int ret;

	pf = vf->pf;
	dev = ice_pf_to_dev(pf);
	ret = ice_vc_fdir_param_check(vf, fltr->vsi_id);
	if (ret) {
		v_ret = VIRTCHNL_STATUS_ERR_PARAM;
		dev_dbg(dev, "Parameter check for VF %d failed\n", vf->vf_id);
		goto err_exit;
	}

	ret = ice_vf_start_ctrl_vsi(vf);
	if (ret && (ret != -EEXIST)) {
		v_ret = VIRTCHNL_STATUS_ERR_PARAM;
		ice_dev_err_errno(dev, ret, "Init FDIR for VF %d failed",
				  vf->vf_id);
		goto err_exit;
	}

	stat = kzalloc(sizeof(*stat), GFP_KERNEL);
	if (!stat) {
		v_ret = VIRTCHNL_STATUS_ERR_NO_MEMORY;
		dev_dbg(dev, "Alloc stat for VF %d failed\n", vf->vf_id);
		goto err_exit;
	}

	conf = kzalloc(sizeof(*conf), GFP_KERNEL);
	if (!conf) {
		v_ret = VIRTCHNL_STATUS_ERR_NO_MEMORY;
		dev_dbg(dev, "Alloc conf for VF %d failed\n", vf->vf_id);
		goto err_exit;
	}

	len = sizeof(*stat);
	ret = ice_vc_validate_fdir_fltr(vf, fltr, conf);
	if (ret) {
		v_ret = VIRTCHNL_STATUS_SUCCESS;
		stat->status = VIRTCHNL_FDIR_FAILURE_RULE_INVALID;
		dev_dbg(dev, "Invalid FDIR filter from VF %d\n", vf->vf_id);
		goto err_free_conf;
	}

	if (fltr->validate_only) {
		v_ret = VIRTCHNL_STATUS_SUCCESS;
		stat->status = VIRTCHNL_FDIR_SUCCESS;
		kfree(conf);
		ret = ice_vc_send_msg_to_vf(vf, VIRTCHNL_OP_ADD_FDIR_FILTER,
					    v_ret, (u8 *)stat, len);
		goto exit;
	}

	/* For Protocol Agnostic Flow Offloading case only */
	if (conf->parser_ena)
		return ice_vc_add_fdir_raw(vf, conf, stat, len);

	is_tun = ice_fdir_is_tunnel(conf->ttype);
	ret = ice_vc_fdir_config_input_set(vf, fltr, conf, is_tun);
	if (ret) {
		v_ret = VIRTCHNL_STATUS_SUCCESS;
		stat->status = VIRTCHNL_FDIR_FAILURE_RULE_CONFLICT;
		ice_dev_err_errno(dev, ret,
				  "VF %d: FDIR input set configure failed",
				  vf->vf_id);
		goto err_free_conf;
	}

	ret = ice_vc_fdir_is_dup_fltr(vf, conf);
	if (ret) {
		v_ret = VIRTCHNL_STATUS_SUCCESS;
		stat->status = VIRTCHNL_FDIR_FAILURE_RULE_EXIST;
		dev_dbg(dev, "VF %d: duplicated FDIR rule detected\n",
			vf->vf_id);
		goto err_free_conf;
	}

	ret = ice_vc_fdir_insert_entry(vf, conf, &conf->flow_id);
	if (ret) {
		v_ret = VIRTCHNL_STATUS_SUCCESS;
		stat->status = VIRTCHNL_FDIR_FAILURE_RULE_NORESOURCE;
		dev_dbg(dev, "VF %d: insert FDIR list failed\n", vf->vf_id);
		goto err_free_conf;
	}

	ret = ice_vc_fdir_set_irq_ctx(vf, conf, VIRTCHNL_OP_ADD_FDIR_FILTER);
	if (ret) {
		v_ret = VIRTCHNL_STATUS_SUCCESS;
		stat->status = VIRTCHNL_FDIR_FAILURE_RULE_NORESOURCE;
		dev_dbg(dev, "VF %d: set FDIR context failed\n", vf->vf_id);
		goto err_rem_entry;
	}

	ret = ice_vc_fdir_write_fltr(vf, conf, true, is_tun);
	if (ret) {
		v_ret = VIRTCHNL_STATUS_SUCCESS;
		stat->status = VIRTCHNL_FDIR_FAILURE_RULE_NORESOURCE;
		ice_dev_err_errno(dev, ret, "VF %d: writing FDIR rule failed",
				  vf->vf_id);
		goto err_clr_irq;
	}

exit:
	kfree(stat);
	return ret;

err_clr_irq:
	ice_vc_fdir_clear_irq_ctx(vf);
err_rem_entry:
	ice_vc_fdir_remove_entry(vf, conf, conf->flow_id);
err_free_conf:
	kfree(conf);
err_exit:
	ret = ice_vc_send_msg_to_vf(vf, VIRTCHNL_OP_ADD_FDIR_FILTER, v_ret,
				    (u8 *)stat, len);
	kfree(stat);
	return ret;
}

/**
 * ice_vc_del_fdir_raw - delete a raw FDIR filter for VF
 * @vf: pointer to the VF info
 * @conf: FDIR configuration for each filter
 * @stat: pointer to the VIRTCHNL_OP_DEL_FDIR_FILTER
 * @len: length of the stat
 *
 * Return: 0 on success, and other on error.
 */
static int
ice_vc_del_fdir_raw(struct ice_vf *vf,
		    struct virtchnl_fdir_fltr_conf *conf,
		    struct virtchnl_fdir_del *stat, int len)
{
	struct ice_vsi *vf_vsi, *ctrl_vsi;
	enum ice_block blk = ICE_BLK_FD;
	enum virtchnl_status_code v_ret;
	struct ice_fdir_prof_info *pi;
	struct ice_pf *pf = vf->pf;
	struct device *dev;
	struct ice_hw *hw;
	u16 vsi_num;
	int ptg;
	int ret;
	int id;

	dev = ice_pf_to_dev(pf);
	hw = &pf->hw;

	id = find_first_bit(conf->prof->ptypes, ICE_FLOW_PTYPE_MAX);
	ptg = hw->blk[ICE_BLK_FD].xlt1.t[id];

	ret = ice_vc_fdir_add_del_raw(vf, conf, false);
	if (ret) {
		v_ret = VIRTCHNL_STATUS_SUCCESS;
		stat->status = VIRTCHNL_FDIR_FAILURE_RULE_NORESOURCE;
		ice_dev_err_errno(dev, ret, "VF %d: deleting FDIR raw flow rule failed",
				  vf->vf_id);
		goto err_del_tmr;
	}

	vf_vsi = ice_get_vf_vsi(vf);
	if (!vf_vsi) {
		v_ret = VIRTCHNL_STATUS_SUCCESS;
		stat->status = VIRTCHNL_FDIR_FAILURE_RULE_NORESOURCE;
		dev_err(dev, "Can not get FDIR vf_vsi for VF %d\n", vf->vf_id);
		goto err_exit;
	}

	ctrl_vsi = pf->vsi[vf->ctrl_vsi_idx];
	if (!ctrl_vsi) {
		v_ret = VIRTCHNL_STATUS_SUCCESS;
		stat->status = VIRTCHNL_FDIR_FAILURE_RULE_NORESOURCE;
		dev_err(dev, "Can not get FDIR ctrl_vsi for VF %d\n",
			vf->vf_id);
		goto err_exit;
	}

	pi = &vf->fdir_prof_info[ptg];
	if (pi->fdir_active_cnt != 0) {
		pi->fdir_active_cnt--;
		/* Remove the profile id flow if no active FDIR rule left */
		if (!pi->fdir_active_cnt) {
			vsi_num = ice_get_hw_vsi_num(hw, ctrl_vsi->idx);
			ice_rem_prof_id_flow(hw, blk, vsi_num, id);

			vsi_num = ice_get_hw_vsi_num(hw, vf_vsi->idx);
			ice_rem_prof_id_flow(hw, blk, vsi_num, id);
		}
	}

	kfree(conf->prof);
	kfree(conf->pkt_buf);
	conf->parser_ena = false;
	kfree(stat);

	return ret;

err_del_tmr:
	ice_vc_fdir_clear_irq_ctx(vf);
err_exit:
	ret = ice_vc_send_msg_to_vf(vf, VIRTCHNL_OP_DEL_FDIR_FILTER, v_ret,
				    (u8 *)stat, len);
	kfree(stat);
	return ret;
}

/**
 * ice_vc_del_fdir_fltr - delete a FDIR filter for VF by the msg buffer
 * @vf: pointer to the VF info
 * @msg: pointer to the msg buffer
 *
 * Return: 0 on success, and other on error.
 */
int ice_vc_del_fdir_fltr(struct ice_vf *vf, u8 *msg)
{
	struct virtchnl_fdir_del *fltr = (struct virtchnl_fdir_del *)msg;
	struct virtchnl_fdir_del *stat = NULL;
	struct virtchnl_fdir_fltr_conf *conf;
	struct ice_vf_fdir *fdir = &vf->fdir;
	enum virtchnl_status_code v_ret;
	struct ice_fdir_fltr *input;
	enum ice_fltr_ptype flow;
	struct device *dev;
	struct ice_pf *pf;
	int is_tun = 0;
	int len = 0;
	int ret;

	pf = vf->pf;
	dev = ice_pf_to_dev(pf);
	ret = ice_vc_fdir_param_check(vf, fltr->vsi_id);
	if (ret) {
		v_ret = VIRTCHNL_STATUS_ERR_PARAM;
		dev_dbg(dev, "Parameter check for VF %d failed\n", vf->vf_id);
		goto err_exit;
	}

	stat = kzalloc(sizeof(*stat), GFP_KERNEL);
	if (!stat) {
		v_ret = VIRTCHNL_STATUS_ERR_NO_MEMORY;
		dev_dbg(dev, "Alloc stat for VF %d failed\n", vf->vf_id);
		goto err_exit;
	}

	len = sizeof(*stat);

	conf = ice_vc_fdir_lookup_entry(vf, fltr->flow_id);
	if (!conf) {
		v_ret = VIRTCHNL_STATUS_SUCCESS;
		stat->status = VIRTCHNL_FDIR_FAILURE_RULE_NONEXIST;
		dev_dbg(dev, "VF %d: FDIR invalid flow_id:0x%X\n",
			vf->vf_id, fltr->flow_id);
		goto err_exit;
	}

	/* Just return failure when ctrl_vsi idx is invalid */
	if (vf->ctrl_vsi_idx == ICE_NO_VSI) {
		v_ret = VIRTCHNL_STATUS_SUCCESS;
		stat->status = VIRTCHNL_FDIR_FAILURE_RULE_NORESOURCE;
		dev_err(dev, "Invalid FDIR ctrl_vsi for VF %d\n", vf->vf_id);
		goto err_exit;
	}

	ret = ice_vc_fdir_set_irq_ctx(vf, conf, VIRTCHNL_OP_DEL_FDIR_FILTER);
	if (ret) {
		v_ret = VIRTCHNL_STATUS_SUCCESS;
		stat->status = VIRTCHNL_FDIR_FAILURE_RULE_NORESOURCE;
		dev_dbg(dev, "VF %d: set FDIR context failed\n", vf->vf_id);
		goto err_exit;
	}

	/* For Protocol Agnostic Flow Offloading case only */
	if (conf->parser_ena)
		return ice_vc_del_fdir_raw(vf, conf, stat, len);

	is_tun = ice_fdir_is_tunnel(conf->ttype);
	ret = ice_vc_fdir_write_fltr(vf, conf, false, is_tun);
	if (ret) {
		v_ret = VIRTCHNL_STATUS_SUCCESS;
		stat->status = VIRTCHNL_FDIR_FAILURE_RULE_NORESOURCE;
		ice_dev_err_errno(dev, ret, "VF %d: writing FDIR rule failed",
				  vf->vf_id);
		goto err_del_tmr;
	}

	/* Remove unused profiles to avoid unexpected behaviors */
	input = &conf->input;
	flow = input->flow_type;
	if (fdir->fdir_fltr_cnt[flow][is_tun] == 1)
		ice_vc_fdir_rem_prof(vf, flow, is_tun);

	kfree(stat);

	return ret;

err_del_tmr:
	ice_vc_fdir_clear_irq_ctx(vf);
err_exit:
	ret = ice_vc_send_msg_to_vf(vf, VIRTCHNL_OP_DEL_FDIR_FILTER, v_ret,
				    (u8 *)stat, len);
	kfree(stat);
	return ret;
}

/**
 * ice_vf_fdir_init - init FDIR resource for VF
 * @vf: pointer to the VF info
 */
void ice_vf_fdir_init(struct ice_vf *vf)
{
	struct ice_vf_fdir *fdir = &vf->fdir;

	idr_init(&fdir->fdir_rule_idr);
	INIT_LIST_HEAD(&fdir->fdir_rule_list);

	spin_lock_init(&fdir->ctx_lock);
	fdir->ctx_irq.flags = 0;
	fdir->ctx_done.flags = 0;
	ice_vc_fdir_reset_cnt_all(fdir);
}

/**
 * ice_vf_fdir_exit - destroy FDIR resource for VF
 * @vf: pointer to the VF info
 */
void ice_vf_fdir_exit(struct ice_vf *vf)
{
	ice_vc_fdir_flush_entry(vf);
	idr_destroy(&vf->fdir.fdir_rule_idr);
	ice_vc_fdir_rem_prof_all(vf);
	ice_vc_fdir_free_prof_all(vf);
}
