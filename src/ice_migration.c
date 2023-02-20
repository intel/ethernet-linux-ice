/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2018-2023 Intel Corporation */

#include "ice.h"

struct ice_migration_virtchnl_msg_slot {
	u32 opcode;
	u16 msg_len;
	char msg_buffer[];
};

struct ice_migration_virtchnl_msg_listnode {
	struct list_head node;
	struct ice_migration_virtchnl_msg_slot msg_slot;
};

struct ice_migration_dev_state {
	u16 vsi_id;
} __aligned(8);

/**
 * ice_migration_get_vf - Get ice vf structure pointer by pdev
 * @vf_pdev: pointer to ice vfio pci vf pdev structure
 *
 * Return nonzero for success, NULL for failure.
 */
void *ice_migration_get_vf(struct pci_dev *vf_pdev)
{
	struct pci_dev *pf_pdev = vf_pdev->physfn;
	int vf_id = pci_iov_vf_id(vf_pdev);
	struct ice_pf *pf;

	if (!pf_pdev || vf_id < 0)
		return NULL;

	pf = pci_get_drvdata(pf_pdev);
	return ice_get_vf_by_id(pf, vf_id);
}
EXPORT_SYMBOL(ice_migration_get_vf);

/**
 * ice_migration_init_vf - Init ice vf device state data
 * @opaque: pointer to vf handler in ice vdev
 */
void ice_migration_init_vf(void *opaque)
{
	struct ice_vf *vf = opaque;

	vf->migration_active = true;
	INIT_LIST_HEAD(&vf->virtchnl_msg_list);
	vf->virtchnl_msg_num = 0;
	vf->vm_vsi_num = vf->lan_vsi_num;
}
EXPORT_SYMBOL(ice_migration_init_vf);

/**
 * ice_migration_uninit_vf - uninit VF device state data
 * @opaque: pointer to vf handler in ice vdev
 */
void ice_migration_uninit_vf(void *opaque)
{
	struct ice_migration_virtchnl_msg_listnode *msg_listnode;
	struct ice_migration_virtchnl_msg_listnode *dtmp;
	struct ice_vf *vf = opaque;

	vf->migration_active = false;

	if (list_empty(&vf->virtchnl_msg_list))
		return;
	list_for_each_entry_safe(msg_listnode, dtmp,
				 &vf->virtchnl_msg_list,
				 node) {
		list_del(&msg_listnode->node);
		kfree(msg_listnode);
	}
	vf->virtchnl_msg_num = 0;
}
EXPORT_SYMBOL(ice_migration_uninit_vf);

/**
 * ice_migration_save_vf_msg - Save request message from VF
 * @vf: pointer to the VF structure
 * @event: pointer to the AQ event
 *
 * save VF message for later restore during live migration
 */
void ice_migration_save_vf_msg(struct ice_vf *vf,
			       struct ice_rq_event_info *event)
{
	struct ice_migration_virtchnl_msg_listnode *msg_listnode;
	u32 v_opcode = le32_to_cpu(event->desc.cookie_high);
	u16 msglen = event->msg_len;
	u8 *msg = event->msg_buf;
	struct device *dev;
	struct ice_pf *pf;

	pf = vf->pf;
	dev = ice_pf_to_dev(pf);

	if (!vf->migration_active)
		return;

	switch (v_opcode) {
	case VIRTCHNL_OP_VERSION:
	case VIRTCHNL_OP_GET_VF_RESOURCES:
	case VIRTCHNL_OP_CONFIG_IRQ_MAP:
	case VIRTCHNL_OP_CONFIG_RSS_KEY:
	case VIRTCHNL_OP_CONFIG_RSS_LUT:
	case VIRTCHNL_OP_GET_SUPPORTED_RXDIDS:
		if (vf->virtchnl_msg_num >= VIRTCHNL_MSG_MAX) {
			dev_warn(dev, "VF %d has maximum number virtual channel commands.\n",
				 vf->vf_id);
			return;
		}

		msg_listnode = kzalloc(struct_size(msg_listnode, msg_slot.msg_buffer, msglen),
				       GFP_KERNEL);
		if (!msg_listnode) {
			dev_err(dev, "VF %d failed to allocate memory for msg listnode.",
				vf->vf_id);
			return;
		}
		dev_dbg(dev, "VF %d save virtual channel command, op code: %d, len: %d.",
			vf->vf_id, v_opcode, msglen);
		msg_listnode->msg_slot.opcode = v_opcode;
		msg_listnode->msg_slot.msg_len = msglen;
		memcpy(msg_listnode->msg_slot.msg_buffer, msg, msglen);
		list_add_tail(&msg_listnode->node, &vf->virtchnl_msg_list);
		vf->virtchnl_msg_num++;
		break;
	default:
		break;
	}
}

/**
 * ice_migration_save_devstate- save vf msg to migration buffer
 * @opaque: pointer to vf handler in ice vdev
 * @buf: pointer to vf msg in migration buffer
 * @buf_sz: size of migration buffer
 *
 * The first two bytes in the buffer is vsi id, followed by
 * virtual channel messages.
 *
 * Return 0 for success, negative for error
 */
int ice_migration_save_devstate(void *opaque, u8 *buf, u64 buf_sz)
{
	struct ice_migration_virtchnl_msg_listnode *msg_listnode;
	struct ice_migration_virtchnl_msg_slot *last_op;
	struct ice_vf *vf = opaque;
	struct device *dev = ice_pf_to_dev(vf->pf);
	struct ice_migration_dev_state *devstate;
	u64 total_size = 0;

	/* reserve space to store device state */
	total_size += sizeof(struct ice_migration_dev_state);
	if (total_size > buf_sz) {
		dev_err(dev, "Insufficient buffer to store device state for VF %d.",
			vf->vf_id);
		return -ENOBUFS;
	}

	devstate = (struct ice_migration_dev_state *)buf;
	devstate->vsi_id = vf->vm_vsi_num;
	buf += sizeof(*devstate);

	list_for_each_entry(msg_listnode, &vf->virtchnl_msg_list, node) {
		struct ice_migration_virtchnl_msg_slot *msg_slot;
		u64 slot_size;

		msg_slot = &msg_listnode->msg_slot;
		slot_size = struct_size(msg_slot, msg_buffer,
					msg_slot->msg_len);
		total_size += slot_size;
		if (total_size > buf_sz) {
			dev_err(dev, "Insufficient buffer to store virtchnl message for VF %d op: %d, len: %d.",
				vf->vf_id, msg_slot->opcode, msg_slot->msg_len);
			return -ENOBUFS;
		}
		dev_dbg(dev, "VF %d copy virtchnl message to migration buffer op: %d, len: %d.",
			vf->vf_id, msg_slot->opcode, msg_slot->msg_len);
		memcpy(buf, msg_slot, slot_size);
		buf += slot_size;
	}
	/* reserve space to mark end of vf messages */
	total_size += sizeof(struct ice_migration_virtchnl_msg_slot);
	if (total_size > buf_sz) {
		dev_err(dev, "Insufficient buffer to store virtchnl message for VF %d.",
			vf->vf_id);
		return -ENOBUFS;
	}

	/* use op code unknown to mark end of vc messages */
	last_op = (struct ice_migration_virtchnl_msg_slot *)buf;
	last_op->opcode = VIRTCHNL_OP_UNKNOWN;
	return 0;
}
EXPORT_SYMBOL(ice_migration_save_devstate);

/**
 * ice_migration_restore_devstate - restore device state at dst
 * @opaque: pointer to vf handler in ice vdev
 * @buf: pointer to device state buf in migration buffer
 * @buf_sz: size of migration buffer
 *
 * The first two bytes in the buffer is vsi id, followed by
 * virtual channel messages.
 *
 * Return 0 for success, negative for error
 */
int ice_migration_restore_devstate(void *opaque, const u8 *buf, u64 buf_sz)
{
	struct ice_migration_virtchnl_msg_slot *msg_slot;
	struct ice_vf *vf = opaque;
	struct device *dev = ice_pf_to_dev(vf->pf);
	struct ice_migration_dev_state *devstate;
	struct ice_rq_event_info event;
	u64 total_size = 0;
	u64 op_msglen_sz;
	u64 slot_sz;
	int ret = 0;

	if (!buf)
		return -EINVAL;

	total_size += sizeof(struct ice_migration_dev_state);
	if (total_size > buf_sz) {
		dev_err(dev, "VF %d msg size exceeds buffer size.", vf->vf_id);
		return -ENOBUFS;
	}

	devstate = (struct ice_migration_dev_state *)buf;
	vf->vm_vsi_num = devstate->vsi_id;
	dev_dbg(dev, "VF %d vm vsi num is:%d.", vf->vf_id, vf->vm_vsi_num);
	buf += sizeof(*devstate);
	msg_slot = (struct ice_migration_virtchnl_msg_slot *)buf;
	op_msglen_sz = sizeof(struct ice_migration_virtchnl_msg_slot);
	/* check whether enough space for opcode and msg_len */
	if (total_size + op_msglen_sz > buf_sz) {
		dev_err(dev, "VF %d msg size exceeds buffer size.", vf->vf_id);
		return -ENOBUFS;
	}

	set_bit(ICE_VF_STATE_REPLAY_VC, vf->vf_states);

	while (msg_slot->opcode != VIRTCHNL_OP_UNKNOWN) {
		slot_sz = struct_size(msg_slot, msg_buffer, msg_slot->msg_len);
		total_size += slot_sz;
		/* check whether enough space for whole message */
		if (total_size > buf_sz) {
			dev_err(dev, "VF %d msg size exceeds buffer size.",
				vf->vf_id);
			ret = -ENOBUFS;
			break;
		}
		dev_dbg(dev, "VF %d replay virtchnl message op code: %d, msg len: %d.",
			vf->vf_id, msg_slot->opcode, msg_slot->msg_len);
		event.desc.cookie_high = msg_slot->opcode;
		event.msg_len = msg_slot->msg_len;
		event.desc.retval = vf->vf_id;
		event.msg_buf = (unsigned char *)msg_slot->msg_buffer;
		ret = ice_vc_process_vf_msg(vf->pf, &event);
		if (ret) {
			dev_err(dev, "failed to replay virtchnl message op code: %d.",
				msg_slot->opcode);
			break;
		}
		event.msg_buf = NULL;
		msg_slot = (struct ice_migration_virtchnl_msg_slot *)
					((char *)msg_slot + slot_sz);
		/* check whether enough space for opcode and msg_len */
		if (total_size + op_msglen_sz > buf_sz) {
			dev_err(dev, "VF %d msg size exceeds buffer size.",
				vf->vf_id);
			ret = -ENOBUFS;
			break;
		}
	}
	clear_bit(ICE_VF_STATE_REPLAY_VC, vf->vf_states);
	return ret;
}
EXPORT_SYMBOL(ice_migration_restore_devstate);

/**
 * ice_migration_fix_msg_vsi - change virtual channel msg vsi id
 *
 * @vf: pointer to the VF structure
 * @v_opcode: virtchnl message operation code
 * @msg: pointer to the virtual channel message
 *
 * After migration, the vsi id of virtual channel message is still
 * migration src vsi id. Some virtual channel commands will fail
 * due to unmatch vsi id.
 * Change virtual channel message payload vsi id to real vsi id.
 */
void ice_migration_fix_msg_vsi(struct ice_vf *vf, u32 v_opcode, u8 *msg)
{
	if (!vf->migration_active)
		return;

	switch (v_opcode) {
	case VIRTCHNL_OP_ADD_ETH_ADDR:
	case VIRTCHNL_OP_DEL_ETH_ADDR:
	case VIRTCHNL_OP_ENABLE_QUEUES:
	case VIRTCHNL_OP_DISABLE_QUEUES:
	case VIRTCHNL_OP_CONFIG_RSS_KEY:
	case VIRTCHNL_OP_CONFIG_RSS_LUT:
	case VIRTCHNL_OP_GET_STATS:
	case VIRTCHNL_OP_CONFIG_PROMISCUOUS_MODE:
	case VIRTCHNL_OP_ADD_FDIR_FILTER:
	case VIRTCHNL_OP_DEL_FDIR_FILTER:
	case VIRTCHNL_OP_ADD_VLAN:
	case VIRTCHNL_OP_DEL_VLAN: {
		/* Read the beginning two bytes of message for vsi id */
		u16 *vsi_id = (u16 *)msg;

		if (*vsi_id == vf->vm_vsi_num ||
		    test_bit(ICE_VF_STATE_REPLAY_VC, vf->vf_states))
			*vsi_id = vf->lan_vsi_num;
		break;
	}
	case VIRTCHNL_OP_CONFIG_IRQ_MAP: {
		struct virtchnl_irq_map_info *irqmap_info;
		u16 num_q_vectors_mapped;
		int i;

		irqmap_info = (struct virtchnl_irq_map_info *)msg;
		num_q_vectors_mapped = irqmap_info->num_vectors;
		for (i = 0; i < num_q_vectors_mapped; i++) {
			struct virtchnl_vector_map *map;

			map = &irqmap_info->vecmap[i];
			if (map->vsi_id == vf->vm_vsi_num ||
			    test_bit(ICE_VF_STATE_REPLAY_VC, vf->vf_states))
				map->vsi_id = vf->lan_vsi_num;
		}
		break;
	}
	case VIRTCHNL_OP_CONFIG_VSI_QUEUES: {
		struct virtchnl_vsi_queue_config_info *qci;

		qci = (struct virtchnl_vsi_queue_config_info *)msg;
		if (qci->vsi_id == vf->vm_vsi_num ||
		    test_bit(ICE_VF_STATE_REPLAY_VC, vf->vf_states)) {
			int i;

			qci->vsi_id = vf->lan_vsi_num;
			for (i = 0; i < qci->num_queue_pairs; i++) {
				struct virtchnl_queue_pair_info *qpi;

				qpi = &qci->qpair[i];
				qpi->txq.vsi_id = vf->lan_vsi_num;
				qpi->rxq.vsi_id = vf->lan_vsi_num;
			}
		}
		break;
	}
	default:
		break;
	}
}
