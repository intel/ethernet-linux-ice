/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2018-2023 Intel Corporation */

#include <linux/fs.h>
#include <linux/debugfs.h>
#include <linux/random.h>
#include "ice.h"
#include "ice_lib.h"
#include "ice_fltr.h"

static struct dentry *ice_debugfs_root;

#define ICE_FW_DUMP_BLK_MAX	0xFFFF
#define ICE_FW_DUMP_DATA_SIZE	4096
#define ICE_FW_DUMP_HDR_SIZE	24
/* dump blk = Cluster header + dump data */
#define ICE_FW_DUMP_BLK_SIZE	(ICE_FW_DUMP_DATA_SIZE + ICE_FW_DUMP_HDR_SIZE)
#define ICE_FW_DUMP_FILENAME	"debug_dump"
#define ICE_FW_DUMP_LAST_IDX	0xFFFFFFFF
#define ICE_FW_DUMP_LAST_ID	0xFF
#define ICE_FW_DUMP_LAST_ID2	0xFFFF

/* The ice_cluster_header structure needs to be added to each table dump
 * the header is contains 6 words each of 4byte(32 bits) in size
 * The structe contains following fields
 *
 * Cluster ID - The cluster ID of the tables
 * Table ID - The table ID of the data dump
 * Table Length - Size of the table
 * Current Offset - This value represents the total offset of bytes into
 *		the Table Length defined by Table ID. For the very
 *		first buffer returned for any table, the offset shall be 0.
 * Two (2) Reserved words for future use.
 */
struct ice_cluster_header {
	u32 cluster_id;
	u32 table_id;
	u32 table_len;
	u32 table_offset;
	u32 reserved[2];
};

/**
 * ice_debugfs_fw_dump - send request to FW to dump cluster and save to file
 * @pf: pointer to pf struct
 * @cluster_id: number or FW cluster to be dumped
 * @read_all_clusters: if true dump all clusters
 *
 * Create FW configuration binary snapshot. Repeatedly send AQ requests to dump
 * FW cluster, FW responds in 4KB blocks and sets new values for table_id
 * and table_idx. Repeat until all tables in given cluster were read.
 */
static int
ice_debugfs_fw_dump(struct ice_pf *pf, u16 cluster_id, bool read_all_clusters)
{
	u32 next_table_idx, table_idx = 0, ntw = 0, ctw = 0, offset = 0;
	u16 buf_len, next_table_id, next_cluster_id, table_id = 0;
	struct debugfs_blob_wrapper *desc_blob;
	struct device *dev = ice_pf_to_dev(pf);
	struct ice_cluster_header header = {};
	struct dentry *pfile;
	u8 *vblk;
	int i;

	desc_blob = devm_kzalloc(dev, sizeof(*desc_blob), GFP_KERNEL);
	if (!desc_blob)
		return -ENOMEM;

	vfree(pf->ice_cluster_blk);
	pf->ice_cluster_blk = NULL;

	vblk = vmalloc(ICE_FW_DUMP_BLK_MAX * ICE_FW_DUMP_BLK_SIZE);
	if (!vblk)
		return -ENOMEM;

	for (i = 0; i < ICE_FW_DUMP_BLK_MAX; i++) {
		int res;

		/* Skip the header bytes */
		ntw += sizeof(struct ice_cluster_header);

		res = ice_aq_get_internal_data(&pf->hw, cluster_id,
					       table_id, table_idx, vblk + ntw,
					       ICE_FW_DUMP_DATA_SIZE,
					       &buf_len, &next_cluster_id,
					       &next_table_id, &next_table_idx,
					       NULL);
		if (res) {
			dev_err(dev, "Internal FW error %d while dumping cluster %d\n",
				res, cluster_id);
			devm_kfree(dev, desc_blob);
			vfree(vblk);
			return -EINVAL;
		}
		ntw += buf_len;

		header.cluster_id = cluster_id;
		header.table_id = table_id;
		header.table_len = buf_len;
		header.table_offset = offset;

		memcpy(vblk + ctw, &header, sizeof(header));
		ctw = ntw;
		memset(&header, 0, sizeof(header));

		offset += buf_len;

		if (table_idx == next_table_idx)
			table_idx = ICE_FW_DUMP_LAST_IDX;
		else
			table_idx = next_table_idx;

		if (table_idx != ICE_FW_DUMP_LAST_IDX)
			continue;

		table_idx = 0;
		offset = 0;

		if (next_cluster_id == ICE_FW_DUMP_LAST_ID2)
			break;

		/* Some clusters return end of table as 0xFF
		 * and some are 0xFFFF
		 */
		if (!(next_table_id == ICE_FW_DUMP_LAST_ID ||
		      next_table_id == ICE_FW_DUMP_LAST_ID2))
			table_id = next_table_id;

		/* End of cluster */
		if (cluster_id != next_cluster_id) {
			if (read_all_clusters) {
				dev_info(dev, "All FW clusters dump - cluster %d appended",
					 cluster_id);
				cluster_id = next_cluster_id;
				table_id = 0;
			} else {
				break;
			}
		}
	}

	desc_blob->size = (unsigned long)ntw;
	desc_blob->data = vblk;

	pfile = debugfs_create_blob(ICE_FW_DUMP_FILENAME, 0400,
				    pf->ice_debugfs_fw, desc_blob);
	if (!pfile)
		return -ENODEV;

	pf->ice_cluster_blk = vblk;

	if (read_all_clusters)
		dev_info(dev, "Created FW dump of all available clusters in file %s\n",
			 ICE_FW_DUMP_FILENAME);
	else
		dev_info(dev, "Created FW dump of cluster %d in file %s\n",
			 cluster_id, ICE_FW_DUMP_FILENAME);

	return 0;
}

/**
 * dump_cluster_id_read - show currently set FW cluster id to dump
 * @file: kernel file struct
 * @buf: user space buffer to fill with correct data
 * @len: buf's length
 * @offset: current position in buf
 */
static ssize_t dump_cluster_id_read(struct file *file, char __user *buf,
				    size_t len, loff_t *offset)
{
	struct ice_pf *pf = file->private_data;
	char temp_buf[11];
	int ret;

	ret = snprintf(temp_buf, sizeof(temp_buf), "%u\n",
		       pf->fw_dump_cluster_id);

	return simple_read_from_buffer(buf, len, offset, temp_buf, ret);
}

/**
 * dump_cluster_id_write - set FW cluster id to dump
 * @file: kernel file struct
 * @buf: user space buffer containing data to read
 * @len: buf's length
 * @offset: current position in buf
 */
static ssize_t dump_cluster_id_write(struct file *file, const char __user *buf,
				     size_t len, loff_t *offset)
{
	struct ice_pf *pf = file->private_data;
	bool read_all_clusters = false;
	char kbuf[11] = { 0 };
	int bytes_read, err;
	u16 cluster_id;

	bytes_read = simple_write_to_buffer(kbuf, sizeof(kbuf), offset, buf,
					    len);
	if (bytes_read < 0)
		return -EINVAL;

	if (bytes_read == 1 && kbuf[0] == '\n') {
		cluster_id = 0;
		read_all_clusters = true;
	} else {
		err = kstrtou16(kbuf, 10, &cluster_id);
		if (err)
			return err;
	}

	debugfs_lookup_and_remove(ICE_FW_DUMP_FILENAME, pf->ice_debugfs_fw);
	err = ice_debugfs_fw_dump(pf, cluster_id, read_all_clusters);
	if (err)
		return err;

	/* Not all cluster IDs are supported in every FW version, save
	 * the value only when FW returned success
	 */
	pf->fw_dump_cluster_id = cluster_id;

	return bytes_read;
}

static const struct file_operations dump_cluster_id_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = dump_cluster_id_read,
	.write = dump_cluster_id_write,
};

static void ice_dump_pf(struct ice_pf *pf)
{
	struct device *dev = ice_pf_to_dev(pf);

	dev_info(dev, "pf struct:\n");
	dev_info(dev, "\tmax_pf_txqs = %d\n", pf->max_pf_txqs);
	dev_info(dev, "\tmax_pf_rxqs = %d\n", pf->max_pf_rxqs);
	dev_info(dev, "\tnum_alloc_vsi = %d\n", pf->num_alloc_vsi);
	dev_info(dev, "\tnum_lan_tx = %d\n", pf->num_lan_tx);
	dev_info(dev, "\tnum_lan_rx = %d\n", pf->num_lan_rx);
	dev_info(dev, "\tnum_avail_tx = %d\n", ice_get_avail_txq_count(pf));
	dev_info(dev, "\tnum_avail_rx = %d\n", ice_get_avail_rxq_count(pf));
	dev_info(dev, "\tmsix total = %d\n",
		 pf->msix.eth + pf->msix.rdma + pf->msix.misc + pf->msix.vf);
	dev_info(dev, "\tmsix.eth = %d\n", pf->msix.eth);
	dev_info(dev, "\tmsix.misc = %d\n", pf->msix.misc);
#ifdef HAVE_DEVLINK_RELOAD_ACTION_AND_LIMIT
	dev_info(dev, "\tmsix.vf occ = %lld\n", ice_sriov_get_vf_used_msix(pf));
	dev_info(dev, "\tmsix.vf free = %lld\n",
		 pf->msix.vf - ice_sriov_get_vf_used_msix(pf));
#endif
	dev_info(dev, "\tmsix.rdma = %d\n", pf->msix.rdma);
	dev_info(dev, "\trdma_base_vector = %d\n", pf->rdma_base_vector);
#ifdef HAVE_NDO_DFWD_OPS
	dev_info(dev, "\tnum_macvlan = %d\n", pf->num_macvlan);
	dev_info(dev, "\tmax_num_macvlan = %d\n", pf->max_num_macvlan);
#endif /* HAVE_NDO_DFWD_OPS */
	dev_info(dev, "\tirq_tracker->num_entries = %d\n",
		 pf->irq_tracker->num_entries);
	dev_info(dev, "\tirq_tracker->end = %d\n", pf->irq_tracker->end);
	dev_info(dev, "\tirq_tracker valid count = %d\n",
		 ice_get_valid_res_count(pf->irq_tracker));
	dev_info(dev, "\tsriov_base_vector = %d\n", pf->sriov_base_vector);
	dev_info(dev, "\tnum_alloc_vfs = %d\n", ice_get_num_vfs(pf));
	dev_info(dev, "\tnum_qps_per_vf = %d\n", pf->vfs.num_qps_per);
	dev_info(dev, "\tnum_msix_per_vf = %d\n", pf->vfs.num_msix_per);
}

static void ice_dump_pf_vsi_list(struct ice_pf *pf)
{
	struct device *dev = ice_pf_to_dev(pf);
	u16 i;

	ice_for_each_vsi(pf, i) {
		struct ice_vsi *vsi = pf->vsi[i];

		if (!vsi)
			continue;

		dev_info(dev, "vsi[%d]:\n", i);
		dev_info(dev, "\tvsi = %p\n", vsi);
		dev_info(dev, "\tvsi_num = %d\n", vsi->vsi_num);
		dev_info(dev, "\ttype = %s\n", ice_vsi_type_str(vsi->type));
		if (vsi->type == ICE_VSI_VF)
			dev_info(dev, "\tvf_id = %d\n", vsi->vf->vf_id);
		dev_info(dev, "\tback = %p\n", vsi->back);
		dev_info(dev, "\tnetdev = %p\n", vsi->netdev);
		dev_info(dev, "\tmax_frame = %d\n", vsi->max_frame);
		dev_info(dev, "\trx_buf_len = %d\n", vsi->rx_buf_len);
		dev_info(dev, "\tnum_txq = %d\n", vsi->num_txq);
		dev_info(dev, "\tnum_rxq = %d\n", vsi->num_rxq);
		dev_info(dev, "\treq_txq = %d\n", vsi->req_txq);
		dev_info(dev, "\treq_rxq = %d\n", vsi->req_rxq);
		dev_info(dev, "\talloc_txq = %d\n", vsi->alloc_txq);
		dev_info(dev, "\talloc_rxq = %d\n", vsi->alloc_rxq);
		dev_info(dev, "\tnum_rx_desc = %d\n", vsi->num_rx_desc);
		dev_info(dev, "\tnum_tx_desc = %d\n", vsi->num_tx_desc);
		dev_info(dev, "\tnum_vlan = %d\n", vsi->num_vlan);
	}
}

/**
 * ice_dump_pf_fdir - output Flow Director stats to dmesg log
 * @pf: pointer to PF to get Flow Director HW stats for.
 */
static void ice_dump_pf_fdir(struct ice_pf *pf)
{
	struct device *dev = ice_pf_to_dev(pf);
	struct ice_hw *hw = &pf->hw;
	u16 pf_guar_pool = 0;
	u32 dev_fltr_size;
	u32 dev_fltr_cnt;
	u32 pf_fltr_cnt;
	u16 i;

	pf_fltr_cnt = rd32(hw, PFQF_FD_CNT);
	dev_fltr_cnt = rd32(hw, GLQF_FD_CNT);
	dev_fltr_size = rd32(hw, GLQF_FD_SIZE);

	ice_for_each_vsi(pf, i) {
		struct ice_vsi *vsi = pf->vsi[i];

		if (!vsi)
			continue;

		pf_guar_pool += vsi->num_gfltr;
	}

	dev_info(dev, "Flow Director filter usage:\n");
	dev_info(dev, "\tPF guaranteed used = %ld\n",
		 (pf_fltr_cnt & PFQF_FD_CNT_FD_GCNT_M) >>
		 PFQF_FD_CNT_FD_GCNT_S);
	dev_info(dev, "\tPF best_effort used = %ld\n",
		 (pf_fltr_cnt & PFQF_FD_CNT_FD_BCNT_M) >>
		 PFQF_FD_CNT_FD_BCNT_S);
	dev_info(dev, "\tdevice guaranteed used = %ld\n",
		 (dev_fltr_cnt & GLQF_FD_CNT_FD_GCNT_M) >>
		 GLQF_FD_CNT_FD_GCNT_S);
	dev_info(dev, "\tdevice best_effort used = %ld\n",
		 (dev_fltr_cnt & GLQF_FD_CNT_FD_BCNT_M) >>
		 GLQF_FD_CNT_FD_BCNT_S);
	dev_info(dev, "\tPF guaranteed pool = %d\n", pf_guar_pool);
	dev_info(dev, "\tdevice guaranteed pool = %ld\n",
		 (dev_fltr_size & GLQF_FD_SIZE_FD_GSIZE_M) >>
		 GLQF_FD_SIZE_FD_GSIZE_S);
	dev_info(dev, "\tdevice best_effort pool = %d\n",
		 hw->func_caps.fd_fltr_best_effort);
}

/**
 * ice_dump_rclk_status - print the PHY recovered clock status
 * @pf: pointer to PF
 *
 * Print the PHY's recovered clock pin status.
 */
static void ice_dump_rclk_status(struct ice_pf *pf)
{
	struct device *dev = ice_pf_to_dev(pf);
	u8 phy, phy_pin, pin;
	int phy_pins;

	if (ice_is_e810(&pf->hw))
		phy_pins = ICE_C827_RCLK_PINS_NUM;
	else
		/* E822-based devices have only one RCLK pin */
		phy_pins = E822_CGU_RCLK_PHY_PINS_NUM;

	for (phy_pin = 0; phy_pin < phy_pins; phy_pin++) {
		const char *pin_name, *pin_state;
		u8 port_num, flags;
		u32 freq;

		port_num = ICE_AQC_SET_PHY_REC_CLK_OUT_CURR_PORT;
		if (ice_aq_get_phy_rec_clk_out(&pf->hw, phy_pin, &port_num,
					       &flags, &freq))
			return;

		if (ice_is_e810(&pf->hw)) {
			int status = ice_get_pf_c827_idx(&pf->hw, &phy);

			if (status) {
				dev_err(dev,
					"Could not find PF C827 PHY, status=%d\n",
					status);
				return;
			}

			pin = E810T_CGU_INPUT_C827(phy, phy_pin);
			pin_name = ice_zl_pin_idx_to_name_e810t(pin);
		} else {
			/* e822-based devices for now have only one phy
			 * available (from Rimmon) and only one DPLL RCLK input
			 * pin
			 */
			pin_name = E822_CGU_RCLK_PIN_NAME;
		}
		pin_state =
			flags & ICE_AQC_SET_PHY_REC_CLK_OUT_OUT_EN ?
			"Enabled" : "Disabled";

		dev_info(dev, "State for pin %s: %s\n", pin_name, pin_state);
	}
}

/**
 * ice_vsi_dump_ctxt - print the passed in VSI context structure
 * @dev: Device used for dev_info prints
 * @ctxt: VSI context structure to print
 */
static void ice_vsi_dump_ctxt(struct device *dev, struct ice_vsi_ctx *ctxt)
{
	struct ice_aqc_vsi_props *info;

	if (!ctxt)
		return;

	info = &ctxt->info;
	dev_info(dev, "Get VSI Parameters:\n");
	dev_info(dev, "\tVSI Number: %d Valid sections: 0x%04x\n",
		 ctxt->vsi_num, le16_to_cpu(info->valid_sections));

	dev_info(dev, "========================\n");
	dev_info(dev, "| Category - Switching |");
	dev_info(dev, "========================\n");
	dev_info(dev, "\tSwitch ID: %u\n", info->sw_id);
	dev_info(dev, "\tAllow Loopback: %s\n", (info->sw_flags &
		 ICE_AQ_VSI_SW_FLAG_ALLOW_LB) ? "enabled" : "disabled");
	dev_info(dev, "\tAllow Local Loopback: %s\n", (info->sw_flags &
		 ICE_AQ_VSI_SW_FLAG_LOCAL_LB) ? "enabled" : "disabled");
	dev_info(dev, "\tApply source VSI pruning: %s\n", (info->sw_flags &
		 ICE_AQ_VSI_SW_FLAG_SRC_PRUNE) ? "enabled" : "disabled");
	dev_info(dev, "\tEgress (Rx VLAN) pruning: %s\n",
		 (info->sw_flags2 & ICE_AQ_VSI_SW_FLAG_RX_PRUNE_EN_M) ?
		 "enabled" : "disabled");
	dev_info(dev, "\tLAN enable: %s\n", (info->sw_flags2 &
		 ICE_AQ_VSI_SW_FLAG_LAN_ENA) ? "enabled" : "disabled");
	dev_info(dev, "\tVEB statistic block ID: %u\n", info->veb_stat_id &
		 ICE_AQ_VSI_SW_VEB_STAT_ID_M);
	dev_info(dev, "\tVEB statistic block ID valid: %d\n",
		 (info->veb_stat_id & ICE_AQ_VSI_SW_VEB_STAT_ID_VALID) ? 1 : 0);

	dev_info(dev, "=======================\n");
	dev_info(dev, "| Category - Security |\n");
	dev_info(dev, "=======================\n");
	dev_info(dev, "\tAllow destination override: %s\n", (info->sec_flags &
		 ICE_AQ_VSI_SEC_FLAG_ALLOW_DEST_OVRD) ? "enabled" : "disabled");
	dev_info(dev, "\tEnable MAC anti-spoof: %s\n", (info->sec_flags &
		 ICE_AQ_VSI_SEC_FLAG_ENA_MAC_ANTI_SPOOF) ? "enabled" : "disabled");
	dev_info(dev, "\tIngress (Tx VLAN) pruning enables: %s\n",
		 (info->sec_flags & ICE_AQ_VSI_SEC_TX_PRUNE_ENA_M) ?
		 "enabled" : "disabled");

	dev_info(dev, "=================================\n");
	dev_info(dev, "| Category: Inner VLAN Handling |\n");
	dev_info(dev, "=================================\n");
	dev_info(dev, "\tPort Based Inner VLAN Insertion: PVLAN ID: %d PRIO: %d\n",
		 le16_to_cpu(info->port_based_inner_vlan) & VLAN_VID_MASK,
		 (le16_to_cpu(info->port_based_inner_vlan) & VLAN_PRIO_MASK) >>
		 VLAN_PRIO_SHIFT);
	dev_info(dev, "\tInner VLAN TX Mode: 0x%02x\n",
		 (info->inner_vlan_flags & ICE_AQ_VSI_INNER_VLAN_TX_MODE_M) >>
		 ICE_AQ_VSI_INNER_VLAN_TX_MODE_S);
	dev_info(dev, "\tInsert PVID: %s\n", (info->inner_vlan_flags &
		 ICE_AQ_VSI_INNER_VLAN_INSERT_PVID) ? "enabled" : "disabled");
	dev_info(dev, "\tInner VLAN and UP expose mode (RX): 0x%02x\n",
		 (info->inner_vlan_flags & ICE_AQ_VSI_INNER_VLAN_EMODE_M) >>
		 ICE_AQ_VSI_INNER_VLAN_EMODE_S);
	dev_info(dev, "\tBlock Inner VLAN from TX Descriptor: %s\n",
		 (info->inner_vlan_flags & ICE_AQ_VSI_INNER_VLAN_BLOCK_TX_DESC) ?
		 "enabled" : "disabled");

	dev_info(dev, "=================================\n");
	dev_info(dev, "| Category: Outer VLAN Handling |\n");
	dev_info(dev, "=================================\n");
	dev_info(dev, "\tPort Based Outer VLAN Insertion: PVID: %d PRIO: %d\n",
		 le16_to_cpu(info->port_based_outer_vlan) & VLAN_VID_MASK,
		 (le16_to_cpu(info->port_based_outer_vlan) & VLAN_PRIO_MASK) >>
		 VLAN_PRIO_SHIFT);
	dev_info(dev, "\tOuter VLAN and UP expose mode (RX): 0x%02x\n",
		 (info->outer_vlan_flags & ICE_AQ_VSI_OUTER_VLAN_EMODE_M) >>
		 ICE_AQ_VSI_OUTER_VLAN_EMODE_S);
	dev_info(dev, "\tOuter Tag type (Tx and Rx): 0x%02x\n",
		 (info->outer_vlan_flags & ICE_AQ_VSI_OUTER_TAG_TYPE_M) >>
		 ICE_AQ_VSI_OUTER_TAG_TYPE_S);
	dev_info(dev, "\tPort Based Outer VLAN Insert Enable: %s\n",
		 (info->outer_vlan_flags &
		 ICE_AQ_VSI_OUTER_VLAN_PORT_BASED_INSERT) ?
		 "enabled" : "disabled");
	dev_info(dev, "\tOuter VLAN TX Mode: 0x%02x\n",
		 (info->outer_vlan_flags & ICE_AQ_VSI_OUTER_VLAN_TX_MODE_M) >>
		 ICE_AQ_VSI_OUTER_VLAN_TX_MODE_S);
	dev_info(dev, "\tBlock Outer VLAN from TX Descriptor: %s\n",
		 (info->outer_vlan_flags & ICE_AQ_VSI_OUTER_VLAN_BLOCK_TX_DESC) ?
		 "enabled" : "disabled");
}

#define ICE_E810T_NEVER_USE_PIN 0xff
#define ZL_VER_MAJOR_SHIFT	24
#define ZL_VER_MAJOR_MASK	ICE_M(0xff, ZL_VER_MAJOR_SHIFT)
#define ZL_VER_MINOR_SHIFT	16
#define ZL_VER_MINOR_MASK	ICE_M(0xff, ZL_VER_MINOR_SHIFT)
#define ZL_VER_REV_SHIFT	8
#define ZL_VER_REV_MASK		ICE_M(0xff, ZL_VER_REV_SHIFT)
#define ZL_VER_BF_SHIFT		0
#define ZL_VER_BF_MASK		ICE_M(0xff, ZL_VER_BF_SHIFT)

/**
 * ice_get_dpll_status - get the detailed state of the clock generator
 * @pf: pointer to PF
 * @buff: buffer for the state to be printed
 * @buff_size: size of the buffer
 *
 * This function reads current status of the ZL CGU and prints it to the buffer
 * buff_size will be updated to reflect the number of bytes written to the
 * buffer
 *
 * Return: 0 on success, error code otherwise
 */
static int
ice_get_dpll_status(struct ice_pf *pf, char *buff, size_t *buff_size)
{
	u8 pin, synce_prio, ptp_prio, ver_major, ver_minor, rev, bugfix;
	struct ice_aqc_get_cgu_abilities abilities = {0};
	struct ice_aqc_get_cgu_input_config cfg = {0};
	struct device *dev = ice_pf_to_dev(pf);
	u32 cgu_id, cgu_cfg_ver, cgu_fw_ver;
	size_t bytes_left = *buff_size;
	struct ice_hw *hw = &pf->hw;
	char pin_name[MAX_PIN_NAME];
	int cnt = 0;
	int status;

	if (!ice_is_cgu_present(hw)) {
		dev_err(dev, "CGU not present\n");
		return -ENODEV;
	}

	memset(&abilities, 0, sizeof(struct ice_aqc_get_cgu_abilities));
	status = ice_aq_get_cgu_abilities(hw, &abilities);
	if (status) {
		dev_err(dev,
			"Failed to read CGU caps, status: %d, Error: 0x%02X\n",
			status, hw->adminq.sq_last_status);
		abilities.num_inputs = 7;
		abilities.pps_dpll_idx = 1;
		abilities.synce_dpll_idx = 0;
	}

	status = ice_aq_get_cgu_info(hw, &cgu_id, &cgu_cfg_ver, &cgu_fw_ver);
	if (status)
		return status;

	if (abilities.cgu_part_num ==
	    ICE_AQC_GET_LINK_TOPO_NODE_NR_ZL30632_80032) {
		cnt = snprintf(buff, bytes_left, "Found ZL80032 CGU\n");

		/* Read DPLL config version from AQ */
		ver_major = (cgu_cfg_ver & ZL_VER_MAJOR_MASK)
			     >> ZL_VER_MAJOR_SHIFT;
		ver_minor = (cgu_cfg_ver & ZL_VER_MINOR_MASK)
			     >> ZL_VER_MINOR_SHIFT;
		rev = (cgu_cfg_ver & ZL_VER_REV_MASK) >> ZL_VER_REV_SHIFT;
		bugfix = (cgu_cfg_ver & ZL_VER_BF_MASK) >> ZL_VER_BF_SHIFT;

		cnt += snprintf(&buff[cnt], bytes_left - cnt,
				"DPLL Config ver: %d.%d.%d.%d\n", ver_major,
				ver_minor, rev, bugfix);
		cnt += snprintf(&buff[cnt], bytes_left - cnt,
				"DPLL FW ver: %i\n", cgu_fw_ver);
	} else if (abilities.cgu_part_num ==
		   ICE_AQC_GET_LINK_TOPO_NODE_NR_SI5383_5384) {
		cnt = snprintf(buff, bytes_left, "Found SI5383/5384 CGU\n");
	}

	cnt += snprintf(&buff[cnt], bytes_left - cnt, "\nCGU Input status:\n");
	cnt += snprintf(&buff[cnt], bytes_left - cnt,
			"                   |            |      priority     |            |\n"
			"      input (idx)  |    state   | EEC (%d) | PPS (%d) | ESync fail |\n",
			abilities.synce_dpll_idx, abilities.pps_dpll_idx);
	cnt += snprintf(&buff[cnt], bytes_left - cnt,
			"  ----------------------------------------------------------------\n");

	for (pin = 0; pin < abilities.num_inputs; pin++) {
		u8 esync_fail = 0;
		u8 esync_en = 0;
		char *pin_state;
		u8 data;

		status = ice_aq_get_input_pin_cfg(hw, &cfg, pin);
		if (status)
			data = ICE_CGU_IN_PIN_FAIL_FLAGS;
		else
			data = (cfg.status & ICE_CGU_IN_PIN_FAIL_FLAGS);

		/* get either e810t pin names or generic ones */
		ice_dpll_pin_idx_to_name(pf, pin, pin_name);

		/* get pin priorities */
		if (ice_aq_get_cgu_ref_prio(hw, abilities.synce_dpll_idx, pin,
					    &synce_prio))
			synce_prio = ICE_E810T_NEVER_USE_PIN;
		if (ice_aq_get_cgu_ref_prio(hw, abilities.pps_dpll_idx, pin,
					    &ptp_prio))
			ptp_prio = ICE_E810T_NEVER_USE_PIN;

		/* if all flags are set, the pin is invalid */
		if (data == ICE_CGU_IN_PIN_FAIL_FLAGS) {
			pin_state = ICE_DPLL_PIN_STATE_INVALID;
		/* if some flags are set, the pin is validating */
		} else if (data) {
			pin_state = ICE_DPLL_PIN_STATE_VALIDATING;
		/* if all flags are cleared, the pin is valid */
		} else {
			pin_state = ICE_DPLL_PIN_STATE_VALID;
			esync_en = (cfg.flags2 &
			    ICE_AQC_GET_CGU_IN_CFG_FLG2_ESYNC_REFSYNC_EN) ==
			   (ICE_AQC_GET_CGU_IN_CFG_ESYNC_EN <<
			    ICE_AQC_GET_CGU_IN_CFG_FLG2_ESYNC_REFSYNC_EN_SHIFT);
			esync_fail = !!(cfg.status &
			    ICE_AQC_GET_CGU_IN_CFG_STATUS_ESYNC_FAIL);
		}

		cnt += snprintf(&buff[cnt], bytes_left - cnt,
				"  %12s (%d) | %10s |     %3d |     %3d |    %4s    |\n",
				pin_name, pin, pin_state, synce_prio, ptp_prio,
				esync_en ? esync_fail ?
				"true" : "false" : "N/A");
	}

	if (!test_bit(ICE_FLAG_DPLL_MONITOR, pf->flags)) {
		cnt += snprintf(&buff[cnt], bytes_left - cnt,
				"\nDPLL Monitoring disabled\n");
	} else {
		/* SYNCE DPLL status */
		ice_dpll_pin_idx_to_name(pf, pf->synce_ref_pin, pin_name);
		cnt += snprintf(&buff[cnt], bytes_left - cnt, "\nEEC DPLL:\n");
		cnt += snprintf(&buff[cnt], bytes_left - cnt,
				"\tCurrent reference:\t%s\n", pin_name);

		cnt += snprintf(&buff[cnt], bytes_left - cnt,
				"\tStatus:\t\t\t%s\n",
				ice_cgu_state_to_name(pf->synce_dpll_state));

		ice_dpll_pin_idx_to_name(pf, pf->ptp_ref_pin, pin_name);
		cnt += snprintf(&buff[cnt], bytes_left - cnt, "\nPPS DPLL:\n");
		cnt += snprintf(&buff[cnt], bytes_left - cnt,
				"\tCurrent reference:\t%s\n", pin_name);
		cnt += snprintf(&buff[cnt], bytes_left - cnt,
				"\tStatus:\t\t\t%s\n",
				ice_cgu_state_to_name(pf->ptp_dpll_state));

		if (pf->ptp_dpll_state != ICE_CGU_STATE_INVALID)
			cnt += snprintf(&buff[cnt], bytes_left - cnt,
					"\tPhase offset [ps]:\t\t\t%lld\n",
					pf->ptp_dpll_phase_offset);
	}

	*buff_size = cnt;
	return 0;
}

/**
 * ice_debugfs_cgu_read - debugfs interface for reading DPLL status
 * @filp: the opened file
 * @user_buf: where to find the user's data
 * @count: the length of the user's data
 * @ppos: file position offset
 *
 * Return: number of bytes read
 */
static ssize_t ice_debugfs_cgu_read(struct file *filp, char __user *user_buf,
				    size_t count, loff_t *ppos)
{
	struct ice_pf *pf = filp->private_data;
	size_t buffer_size = PAGE_SIZE;
	char *kbuff;
	int err;

	if (*ppos != 0)
		return 0;

	kbuff = (char *)get_zeroed_page(GFP_KERNEL);
	if (!kbuff)
		return -ENOMEM;

	err = ice_get_dpll_status(pf, kbuff, &buffer_size);

	if (err) {
		err = -EIO;
		goto err;
	}

	err = simple_read_from_buffer(user_buf, count, ppos, kbuff,
				      buffer_size);

err:
	free_page((unsigned long)kbuff);
	return err;
}

static const struct file_operations ice_debugfs_cgu_fops = {
	.owner = THIS_MODULE,
	.llseek = default_llseek,
	.open  = simple_open,
	.read  = ice_debugfs_cgu_read,
};

static const char *module_id_to_name(u16 module_id)
{
	switch (module_id) {
	case ICE_AQC_FW_LOG_ID_GENERAL:
		return "General";
	case ICE_AQC_FW_LOG_ID_CTRL:
		return "Control (Resets + Autoload)";
	case ICE_AQC_FW_LOG_ID_LINK:
		return "Link Management";
	case ICE_AQC_FW_LOG_ID_LINK_TOPO:
		return "Link Topology Detection";
	case ICE_AQC_FW_LOG_ID_DNL:
		return "DNL";
	case ICE_AQC_FW_LOG_ID_I2C:
		return "I2C";
	case ICE_AQC_FW_LOG_ID_SDP:
		return "SDP";
	case ICE_AQC_FW_LOG_ID_MDIO:
		return "MDIO";
	case ICE_AQC_FW_LOG_ID_ADMINQ:
		return "Admin Queue";
	case ICE_AQC_FW_LOG_ID_HDMA:
		return "HDMA";
	case ICE_AQC_FW_LOG_ID_LLDP:
		return "LLDP";
	case ICE_AQC_FW_LOG_ID_DCBX:
		return "DCBX";
	case ICE_AQC_FW_LOG_ID_DCB:
		return "DCB";
	case ICE_AQC_FW_LOG_ID_XLR:
		return "XLR";
	case ICE_AQC_FW_LOG_ID_NVM:
		return "NVM";
	case ICE_AQC_FW_LOG_ID_AUTH:
		return "Authentication";
	case ICE_AQC_FW_LOG_ID_VPD:
		return "VPD";
	case ICE_AQC_FW_LOG_ID_IOSF:
		return "IOSF";
	case ICE_AQC_FW_LOG_ID_PARSER:
		return "Parser";
	case ICE_AQC_FW_LOG_ID_SW:
		return "Switch";
	case ICE_AQC_FW_LOG_ID_SCHEDULER:
		return "Scheduler";
	case ICE_AQC_FW_LOG_ID_TXQ:
		return "Tx Queue Management";
	case ICE_AQC_FW_LOG_ID_ACL:
		return "ACL";
	case ICE_AQC_FW_LOG_ID_POST:
		return "Post";
	case ICE_AQC_FW_LOG_ID_WATCHDOG:
		return "Watchdog";
	case ICE_AQC_FW_LOG_ID_TASK_DISPATCH:
		return "Task Dispatcher";
	case ICE_AQC_FW_LOG_ID_MNG:
		return "Manageability";
	case ICE_AQC_FW_LOG_ID_SYNCE:
		return "Synce";
	case ICE_AQC_FW_LOG_ID_HEALTH:
		return "Health";
	case ICE_AQC_FW_LOG_ID_TSDRV:
		return "Time Sync";
	case ICE_AQC_FW_LOG_ID_PFREG:
		return "PF Registration";
	case ICE_AQC_FW_LOG_ID_MDLVER:
		return "Module Version";
	default:
		return "Unsupported";
	}
}

static const char *log_level_to_name(u8 log_level)
{
	switch (log_level) {
	case ICE_FWLOG_LEVEL_NONE:
		return "None";
	case ICE_FWLOG_LEVEL_ERROR:
		return "Error";
	case ICE_FWLOG_LEVEL_WARNING:
		return "Warning";
	case ICE_FWLOG_LEVEL_NORMAL:
		return "Normal";
	case ICE_FWLOG_LEVEL_VERBOSE:
		return "Verbose";
	default:
		return "Unsupported";
	}
}

/**
 * ice_fwlog_dump_cfg - Dump current FW logging configuration
 * @hw: pointer to the HW structure
 */
static void ice_fwlog_dump_cfg(struct ice_hw *hw)
{
	struct device *dev = ice_pf_to_dev((struct ice_pf *)(hw->back));
	struct ice_fwlog_cfg *cfg;
	int status;
	u16 i;

	cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);
	if (!cfg)
		return;

	status = ice_fwlog_get(hw, cfg);
	if (status) {
		kfree(cfg);
		return;
	}

	dev_info(dev, "FWLOG Configuration:\n");
	dev_info(dev, "Options: 0x%04x\n", cfg->options);
	dev_info(dev, "\tarq_ena: %s\n",
		 (cfg->options &
		  ICE_FWLOG_OPTION_ARQ_ENA) ? "true" : "false");
	dev_info(dev, "\tuart_ena: %s\n",
		 (cfg->options &
		  ICE_FWLOG_OPTION_UART_ENA) ? "true" : "false");
	dev_info(dev, "\tPF registered: %s\n",
		 (cfg->options &
		  ICE_FWLOG_OPTION_IS_REGISTERED) ? "true" : "false");

	dev_info(dev, "Module Entries:\n");
	for (i = 0; i < ICE_AQC_FW_LOG_ID_MAX; i++) {
		struct ice_fwlog_module_entry *entry =
			&cfg->module_entries[i];

		dev_info(dev, "\tModule ID %d (%s) Log Level %d (%s)\n",
			 entry->module_id, module_id_to_name(entry->module_id),
			 entry->log_level, log_level_to_name(entry->log_level));
	}

	kfree(cfg);
}

/**
 * ice_debugfs_command_write - write into command datum
 * @filp: the opened file
 * @buf: where to find the user's data
 * @count: the length of the user's data
 * @ppos: file position offset
 */
static ssize_t
ice_debugfs_command_write(struct file *filp, const char __user *buf,
			  size_t count, loff_t *ppos)
{
	struct ice_pf *pf = filp->private_data;
	struct device *dev = ice_pf_to_dev(pf);
	struct ice_hw *hw = &pf->hw;
	char *cmd_buf, *cmd_buf_tmp;
	ssize_t ret;
	char **argv;
	int argc;

	/* don't allow partial writes and writes when reset is in progress*/
	if (*ppos != 0 || ice_is_reset_in_progress(pf->state))
		return 0;

	cmd_buf = memdup_user(buf, count + 1);
	if (IS_ERR(cmd_buf))
		return PTR_ERR(cmd_buf);
	cmd_buf[count] = '\0';

	cmd_buf_tmp = strchr(cmd_buf, '\n');
	if (cmd_buf_tmp) {
		*cmd_buf_tmp = '\0';
		count = (size_t)cmd_buf_tmp - (size_t)cmd_buf + 1;
	}

	argv = argv_split(GFP_KERNEL, cmd_buf, &argc);
	if (!argv) {
		ret = -ENOMEM;
		goto err_copy_from_user;
	}

	if (argc > 1 && !strncmp(argv[1], "vsi", 3)) {
		if (argc == 3 && !strncmp(argv[0], "get", 3)) {
			struct ice_vsi_ctx *vsi_ctx;

			vsi_ctx = devm_kzalloc(dev, sizeof(*vsi_ctx),
					       GFP_KERNEL);
			if (!vsi_ctx) {
				ret = -ENOMEM;
				goto command_write_error;
			}
			ret = kstrtou16(argv[2], 0, &vsi_ctx->vsi_num);
			if (ret) {
				devm_kfree(dev, vsi_ctx);
				goto command_help;
			}
			ret = ice_aq_get_vsi_params(hw, vsi_ctx, NULL);
			if (ret) {
				devm_kfree(dev, vsi_ctx);
				goto command_help;
			}

			ice_vsi_dump_ctxt(dev, vsi_ctx);
			devm_kfree(dev, vsi_ctx);
		} else {
			goto command_help;
		}
	} else if (argc == 2 && !strncmp(argv[0], "dump", 4) &&
		   !strncmp(argv[1], "switch", 6)) {
		ret = ice_dump_sw_cfg(hw);
		if (ret) {
			ret = -EINVAL;
			dev_err(dev, "dump switch failed\n");
			goto command_write_error;
		}
	} else if (argc == 2 && !strncmp(argv[0], "dump", 4) &&
		   !strncmp(argv[1], "capabilities", 11)) {
		ice_dump_caps(hw);
	} else if (argc == 2 && !strncmp(argv[0], "dump", 4) &&
		   !strncmp(argv[1], "fwlog_cfg", 9)) {
		ice_fwlog_dump_cfg(&pf->hw);
	} else if (argc == 4 && !strncmp(argv[0], "dump", 4) &&
		   !strncmp(argv[1], "ptp", 3) &&
		   !strncmp(argv[2], "func", 4) &&
		   !strncmp(argv[3], "capabilities", 11)) {
		ice_dump_ptp_func_caps(hw);
	} else if (argc == 4 && !strncmp(argv[0], "dump", 4) &&
		   !strncmp(argv[1], "ptp", 3) &&
		   !strncmp(argv[2], "dev", 3) &&
		   !strncmp(argv[3], "capabilities", 11)) {
		ice_dump_ptp_dev_caps(hw);
	} else if (argc == 2 && !strncmp(argv[0], "dump", 4) &&
		   !strncmp(argv[1], "ports", 5)) {
		dev_info(dev, "port_info:\n");
		ice_dump_port_info(hw->port_info);
#ifdef ICE_ADD_PROBES
	} else if (argc == 2 && !strncmp(argv[0], "dump", 4) &&
		   !strncmp(argv[1], "arfs_stats", 10)) {
		struct ice_vsi *vsi = ice_get_main_vsi(pf);

		if (!vsi) {
			dev_err(dev, "Failed to find PF VSI\n");
		} else if (vsi->netdev->features & NETIF_F_NTUPLE) {
			struct ice_arfs_active_fltr_cntrs *fltr_cntrs;

			fltr_cntrs = vsi->arfs_fltr_cntrs;

			/* active counters can be updated by multiple CPUs */
			smp_mb__before_atomic();
			dev_info(dev, "arfs_active_tcpv4_filters: %d\n",
				 atomic_read(&fltr_cntrs->active_tcpv4_cnt));
			dev_info(dev, "arfs_active_tcpv6_filters: %d\n",
				 atomic_read(&fltr_cntrs->active_tcpv6_cnt));
			dev_info(dev, "arfs_active_udpv4_filters: %d\n",
				 atomic_read(&fltr_cntrs->active_udpv4_cnt));
			dev_info(dev, "arfs_active_udpv6_filters: %d\n",
				 atomic_read(&fltr_cntrs->active_udpv6_cnt));
		}
#endif /* ICE_ADD_PROBES */
	} else if (argc == 2 && !strncmp(argv[0], "dump", 4)) {
		if (!strncmp(argv[1], "mmcast", 6)) {
			ice_dump_sw_rules(hw, ICE_SW_LKUP_MAC);
		} else if (!strncmp(argv[1], "vlan", 4)) {
			ice_dump_sw_rules(hw, ICE_SW_LKUP_VLAN);
		} else if (!strncmp(argv[1], "eth", 3)) {
			ice_dump_sw_rules(hw, ICE_SW_LKUP_ETHERTYPE);
		} else if (!strncmp(argv[1], "pf_vsi", 6)) {
			ice_dump_pf_vsi_list(pf);
		} else if (!strncmp(argv[1], "pf_port_num", 11)) {
			dev_info(dev, "pf_id = %d, port_num = %d\n",
				 hw->pf_id, hw->port_info->lport);
		} else if (!strncmp(argv[1], "pf", 2)) {
			ice_dump_pf(pf);
		} else if (!strncmp(argv[1], "vfs", 3)) {
			ice_dump_all_vfs(pf);
		} else if (!strncmp(argv[1], "fdir_stats", 10)) {
			ice_dump_pf_fdir(pf);
		} else if (!strncmp(argv[1], "reset_stats", 11)) {
			dev_info(dev, "core reset count: %d\n",
				 pf->corer_count);
			dev_info(dev, "global reset count: %d\n",
				 pf->globr_count);
			dev_info(dev, "emp reset count: %d\n", pf->empr_count);
			dev_info(dev, "pf reset count: %d\n", pf->pfr_count);
		} else if ((!strncmp(argv[1], "rclk_status", 11))) {
			if (ice_is_feature_supported(pf, ICE_F_PHY_RCLK))
				ice_dump_rclk_status(pf);
		}

#ifdef CONFIG_DCB
	} else if (argc == 3 && !strncmp(argv[0], "lldp", 4) &&
				!strncmp(argv[1], "get", 3)) {
		u8 mibtype;
		u16 llen, rlen;
		u8 *buff;

		if (!strncmp(argv[2], "local", 5))
			mibtype = ICE_AQ_LLDP_MIB_LOCAL;
		else if (!strncmp(argv[2], "remote", 6))
			mibtype = ICE_AQ_LLDP_MIB_REMOTE;
		else
			goto command_help;

		buff = devm_kzalloc(dev, ICE_LLDPDU_SIZE, GFP_KERNEL);
		if (!buff) {
			ret = -ENOMEM;
			goto command_write_error;
		}

		ret = ice_aq_get_lldp_mib(hw,
					  ICE_AQ_LLDP_BRID_TYPE_NEAREST_BRID,
					  mibtype, (void *)buff,
					  ICE_LLDPDU_SIZE,
					  &llen, &rlen, NULL);

		if (!ret) {
			if (mibtype == ICE_AQ_LLDP_MIB_LOCAL) {
				dev_info(dev, "LLDP MIB (local)\n");
				print_hex_dump(KERN_INFO, "LLDP MIB (local): ",
					       DUMP_PREFIX_OFFSET, 16, 1,
					       buff, llen, true);
			} else if (mibtype == ICE_AQ_LLDP_MIB_REMOTE) {
				dev_info(dev, "LLDP MIB (remote)\n");
				print_hex_dump(KERN_INFO, "LLDP MIB (remote): ",
					       DUMP_PREFIX_OFFSET, 16, 1,
					       buff, rlen, true);
			}
		} else {
			dev_err(dev, "GET LLDP MIB failed. Status: %ld\n", ret);
		}
		devm_kfree(dev, buff);
#endif /* CONFIG_DCB */
	} else if ((argc > 1) && !strncmp(argv[1], "scheduling", 10)) {
		if (argc == 4 && !strncmp(argv[0], "get", 3) &&
		    !strncmp(argv[2], "tree", 4) &&
		    !strncmp(argv[3], "topology", 8)) {
			ice_dump_port_topo(hw->port_info);
		}
	} else {
command_help:
		dev_info(dev, "unknown or invalid command '%s'\n", cmd_buf);
		dev_info(dev, "available commands\n");
		dev_info(dev, "\t get vsi <vsinum>\n");
		dev_info(dev, "\t dump switch\n");
		dev_info(dev, "\t dump ports\n");
		dev_info(dev, "\t dump capabilities\n");
		dev_info(dev, "\t dump fwlog_cfg\n");
		dev_info(dev, "\t dump ptp func capabilities\n");
		dev_info(dev, "\t dump ptp dev capabilities\n");
		dev_info(dev, "\t dump mmcast\n");
		dev_info(dev, "\t dump vlan\n");
		dev_info(dev, "\t dump eth\n");
		dev_info(dev, "\t dump pf_vsi\n");
		dev_info(dev, "\t dump pf\n");
		dev_info(dev, "\t dump pf_port_num\n");
		dev_info(dev, "\t dump vfs\n");
		dev_info(dev, "\t dump reset_stats\n");
		dev_info(dev, "\t dump fdir_stats\n");
		dev_info(dev, "\t get scheduling tree topology\n");
		dev_info(dev, "\t get scheduling tree topology portnum <port>\n");
#ifdef CONFIG_DCB
		dev_info(dev, "\t lldp get local\n");
		dev_info(dev, "\t lldp get remote\n");
#endif /* CONFIG_DCB */
#ifdef ICE_ADD_PROBES
		dev_info(dev, "\t dump arfs_stats\n");
#endif /* ICE_ADD_PROBES */
		if (ice_is_feature_supported(pf, ICE_F_PHY_RCLK))
			dev_info(dev, "\t dump rclk_status\n");
		ret = -EINVAL;
		goto command_write_error;
	}

	/* if we get here, nothing went wrong; return bytes copied */
	ret = (ssize_t)count;

command_write_error:
	argv_free(argv);
err_copy_from_user:
	kfree(cmd_buf);

	/* This function always consumes all of the written input, or produces
	 * an error. Check and enforce this. Otherwise, the write operation
	 * won't complete properly.
	 */
	if (WARN_ON(ret != (ssize_t)count && ret >= 0))
		ret = -EIO;

	return ret;
}

static const struct file_operations ice_debugfs_command_fops = {
	.owner = THIS_MODULE,
	.open  = simple_open,
	.write = ice_debugfs_command_write,
};

/**
 * ice_debugfs_pf_init - setup the debugfs directory
 * @pf: the ice that is starting up
 */
void ice_debugfs_pf_init(struct ice_pf *pf)
{
	const char *name = pci_name(pf->pdev);

	pf->ice_debugfs_pf = debugfs_create_dir(name, ice_debugfs_root);
	if (IS_ERR(pf->ice_debugfs_pf))
		return;

	pf->ice_debugfs_fw = debugfs_create_dir("fw", pf->ice_debugfs_pf);
	if (IS_ERR(pf->ice_debugfs_fw))
		return;

	if (!debugfs_create_file("command", 0600, pf->ice_debugfs_pf,
				 pf, &ice_debugfs_command_fops))
		goto create_failed;

	if (!debugfs_create_file("dump_cluster_id", 0600, pf->ice_debugfs_fw,
				 pf, &dump_cluster_id_fops))
		goto create_failed;

	/* Expose external CGU debugfs interface if CGU available*/
	if (ice_is_feature_supported(pf, ICE_F_CGU)) {
		if (!debugfs_create_file("cgu", 0400, pf->ice_debugfs_pf, pf,
					 &ice_debugfs_cgu_fops))
			goto create_failed;
	}

	return;

create_failed:
	dev_err(ice_pf_to_dev(pf), "debugfs dir/file for %s failed\n", name);
	debugfs_remove_recursive(pf->ice_debugfs_pf);
}

/**
 * ice_debugfs_pf_exit - clear out the ices debugfs entries
 * @pf: the ice that is stopping
 */
void ice_debugfs_pf_exit(struct ice_pf *pf)
{
	debugfs_remove_recursive(pf->ice_debugfs_pf);

	vfree(pf->ice_cluster_blk);
	pf->ice_cluster_blk = NULL;

	pf->ice_debugfs_fw = NULL;
	pf->ice_debugfs_pf = NULL;
}

/**
 * ice_debugfs_init - create root directory for debugfs entries
 */
void ice_debugfs_init(void)
{
	ice_debugfs_root = debugfs_create_dir(KBUILD_MODNAME, NULL);
	if (IS_ERR(ice_debugfs_root))
		pr_info("init of debugfs failed\n");
}

/**
 * ice_debugfs_exit - remove debugfs entries
 */
void ice_debugfs_exit(void)
{
	debugfs_remove_recursive(ice_debugfs_root);
	ice_debugfs_root = NULL;
}
