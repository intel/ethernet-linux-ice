/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2018-2024 Intel Corporation */

#include "ice_devlink_health.h"
#include "ice.h"
#include "ice_ethtool_common.h"

#ifdef HAVE_DEVLINK_HEALTH

#define devlink_fmsg_put(fmsg, name, value) (			\
	_Generic((value),					\
		 bool:		devlink_fmsg_bool_pair_put,	\
		 u8:		devlink_fmsg_u8_pair_put,	\
		 u16:		devlink_fmsg_u32_pair_put,	\
		 u32:		devlink_fmsg_u32_pair_put,	\
		 u64:		devlink_fmsg_u64_pair_put,	\
		 char *:	devlink_fmsg_string_pair_put,	\
		 const char *:	devlink_fmsg_string_pair_put)	\
	(fmsg, name, (value)))

#define ICE_DEVLINK_FMSG_PUT_FIELD(fmsg, obj, name) \
	devlink_fmsg_put(fmsg, #name, (obj)->name)

/**
 * ice_devlink_health_report - boilerplate to call given @reporter
 *
 * @reporter: devlink health reporter to call, do nothing on NULL
 * @msg: message to pass up, "event name" is fine
 * @priv_ctx: typically some event struct
 */
static void ice_devlink_health_report(struct devlink_health_reporter *reporter,
				      const char *msg, void *priv_ctx)
{
	int err;

	if (!reporter)
		return;

	err = devlink_health_report(reporter, msg, priv_ctx);
	if (err) {
		struct ice_pf *pf = devlink_health_reporter_priv(reporter);

		dev_err(ice_pf_to_dev(pf),
			"failed to report %s via devlink health, err %d\n",
			msg, err);
	}
}

struct ice_mdd_event {
	enum ice_mdd_src src;
	u16 vf_num;
	u16 queue;
	u8 pf_num;
	u8 event;
};

static const char *ice_mdd_src_to_str(enum ice_mdd_src src)
{
	switch (src) {
	case ICE_MDD_SRC_TX_PQM:
		return "tx_pqm";
	case ICE_MDD_SRC_TX_TCLAN:
		return "tx_tclan";
	case ICE_MDD_SRC_TX_TDPU:
		return "tx_tdpu";
	case ICE_MDD_SRC_RX:
		return "rx";
	default:
		return "invalid";
	}
}

static int
#ifndef HAVE_DEVLINK_HEALTH_OPS_EXTACK
ice_mdd_reporter_dump(struct devlink_health_reporter *reporter,
		      struct devlink_fmsg *fmsg, void *priv_ctx)
#else
ice_mdd_reporter_dump(struct devlink_health_reporter *reporter,
		      struct devlink_fmsg *fmsg, void *priv_ctx,
		      struct netlink_ext_ack __always_unused *extack)
#endif /* HAVE_DEVLINK_HEALTH_OPS_EXTACK */
{
	struct ice_mdd_event *mdd_event = priv_ctx;
	const char *src;

	if (!mdd_event)
		return 0;

	src = ice_mdd_src_to_str(mdd_event->src);

	devlink_fmsg_obj_nest_start(fmsg);
	devlink_fmsg_put(fmsg, "src", src);
	ICE_DEVLINK_FMSG_PUT_FIELD(fmsg, mdd_event, pf_num);
	ICE_DEVLINK_FMSG_PUT_FIELD(fmsg, mdd_event, vf_num);
	ICE_DEVLINK_FMSG_PUT_FIELD(fmsg, mdd_event, event);
	ICE_DEVLINK_FMSG_PUT_FIELD(fmsg, mdd_event, queue);
	devlink_fmsg_obj_nest_end(fmsg);

	return 0;
}

/**
 * ice_devlink_report_mdd_event - Report an MDD event through devlink health
 * @pf: the PF device structure
 * @src: the HW block that was the source of this MDD event
 * @pf_num: the pf_num on which the MDD event occurred
 * @vf_num: the vf_num on which the MDD event occurred
 * @event: the event type of the MDD event
 * @queue: the queue on which the MDD event occurred
 *
 * Report an MDD event that has occurred on this PF.
 */
void ice_devlink_report_mdd_event(struct ice_pf *pf, enum ice_mdd_src src,
				  u8 pf_num, u16 vf_num, u8 event, u16 queue)
{
	struct ice_mdd_event ev = {
		.src = src,
		.pf_num = pf_num,
		.vf_num = vf_num,
		.event = event,
		.queue = queue,
	};

	ice_devlink_health_report(pf->health_reporters.mdd, "MDD event", &ev);
}

/**
 * ice_emit_to_buf - print to @size sized buffer
 *
 * @buf: buffer to print into
 * @at: current pos to write in @buf
 * @size: total space in @buf (incl. prior to @at)
 * @fmt: format of the message to print
 *
 * Return: position in the @buf for next write, @size at most, to ease out
 * error handling.
 */
static __printf(4, 5)
int ice_emit_to_buf(char *buf, int size, int at, const char *fmt, ...)
{
	va_list args;
	int len;

	va_start(args, fmt);
	len = vscnprintf(buf + at, size - at, fmt, args);
	va_end(args);

	return min(at + len, size);
}

/**
 * ice_emit_hex_to_buf - copy of print_hex_dump() from lib/hexdump.c modified to
 * dump into buffer
 *
 * @out: buffer to print to
 * @out_size: total size of @out
 * @out_pos: position in @out to write at
 * @prefix: string to prefix each line with;
 *  caller supplies trailing spaces for alignment if desired
 * @buf: data blob to dump
 * @buf_len: number of bytes in the @buf
 *
 * Return: position in the buf for next write, buf_len at most, to ease out
 * error handling.
 */
static int ice_emit_hex_to_buf(char *out, int out_size, int out_pos,
			       const char *prefix, const void *buf,
			       size_t buf_len)
{
	unsigned char linebuf[32 * 3 + 2 + 32 + 1];
	const int rowsize = 16, groupsize = 1;
	int i, linelen, remaining = buf_len;
	const u8 *ptr = buf;

	for (i = 0; i < buf_len; i += rowsize) {
		linelen = min(remaining, rowsize);
		remaining -= rowsize;

		hex_dump_to_buffer(ptr + i, linelen, rowsize, groupsize,
				   linebuf, sizeof(linebuf), false);
		out_pos = ice_emit_to_buf(out, out_size, out_pos, "%s%.8x: %s\n",
					  prefix, i, linebuf);

		if (out_pos == out_size)
			break;
	}

	return out_pos;
}

/**
 * ice_skb_dump_buf - Dump skb information and contents.
 *
 * copy of skb_dump() from net/core/skbuff.c, modified to dump into buffer
 *
 * @skb: skb to dump
 * @buf: buffer to dump into
 * @buf_size: size of @buf
 * @buf_pos: current position to write in @buf
 *
 * Return: position in the buf for next write.
 */
static int ice_skb_dump_buf(char *buf, int buf_size, int buf_pos,
			    const struct sk_buff *skb)
{
	struct skb_shared_info *sh = skb_shinfo(skb);
	struct net_device *dev = skb->dev;
	const bool toplvl = !buf_pos;
	struct sock *sk = skb->sk;
	struct sk_buff *list_skb;
	bool has_mac, has_trans;
	int headroom, tailroom;
	int i, len, seg_len;

	len = skb->len;

	headroom = skb_headroom(skb);
	tailroom = skb_tailroom(skb);

	has_mac = skb_mac_header_was_set(skb);
	has_trans = skb_transport_header_was_set(skb);

	buf_pos = ice_emit_to_buf(buf, buf_size, buf_pos,
	       "skb len=%u headroom=%u headlen=%u tailroom=%u\n"
	       "mac=(%d,%d) net=(%d,%d) trans=%d\n"
	       "shinfo(txflags=%u nr_frags=%u gso(size=%hu type=%u segs=%hu))\n"
	       "csum(0x%x ip_summed=%u complete_sw=%u valid=%u level=%u)\n"
	       "hash(0x%x sw=%u l4=%u) proto=0x%04x pkttype=%u iif=%d\n",
	       skb->len, headroom, skb_headlen(skb), tailroom,
	       has_mac ? skb->mac_header : -1,
	       has_mac ? skb_mac_header_len(skb) : -1,
	       skb->network_header,
	       has_trans ? skb_network_header_len(skb) : -1,
	       has_trans ? skb->transport_header : -1,
	       sh->tx_flags, sh->nr_frags,
	       sh->gso_size, sh->gso_type, sh->gso_segs,
	       skb->csum, skb->ip_summed, skb->csum_complete_sw,
	       skb->csum_valid, skb->csum_level,
	       skb->hash, skb->sw_hash, skb->l4_hash,
	       ntohs(skb->protocol), skb->pkt_type, skb->skb_iif);

	if (dev)
		buf_pos = ice_emit_to_buf(buf, buf_size, buf_pos, "dev name=%s feat=%pNF\n",
					  dev->name, &dev->features);
	if (sk)
		buf_pos = ice_emit_to_buf(buf, buf_size, buf_pos, "sk family=%hu type=%u proto=%u\n",
					  sk->sk_family, sk->sk_type,
					  sk->sk_protocol);

	if (headroom)
		buf_pos = ice_emit_hex_to_buf(buf, buf_size, buf_pos, "skb headroom: ",
					      skb->head, headroom);

	seg_len = min_t(int, skb_headlen(skb), len);
	if (seg_len)
		buf_pos = ice_emit_hex_to_buf(buf, buf_size, buf_pos, "skb linear:   ",
					      skb->data, seg_len);
	len -= seg_len;

	if (tailroom)
		buf_pos = ice_emit_hex_to_buf(buf, buf_size, buf_pos, "skb tailroom: ",
					      skb_tail_pointer(skb), tailroom);

	for (i = 0; len && i < skb_shinfo(skb)->nr_frags; i++) {
		skb_frag_t *frag = &skb_shinfo(skb)->frags[i];
		u32 p_off, p_len, copied;
		struct page *p;
		u8 *vaddr;

		skb_frag_foreach_page(frag, skb_frag_off(frag),
				      skb_frag_size(frag), p, p_off, p_len,
				      copied) {
			seg_len = min_t(int, p_len, len);
			vaddr = kmap_atomic(p);
			buf_pos = ice_emit_hex_to_buf(buf, buf_size, buf_pos, "skb frag:     ",
						      vaddr + p_off, seg_len);
			kunmap_atomic(vaddr);
			len -= seg_len;

			if (!len || buf_pos == buf_size)
				break;
		}
	}

	if (skb_has_frag_list(skb)) {
		buf_pos = ice_emit_to_buf(buf, buf_size, buf_pos, "skb fraglist:\n");
		skb_walk_frags(skb, list_skb) {
			buf_pos = ice_skb_dump_buf(buf, buf_size, buf_pos,
						   list_skb);

			if (buf_pos == buf_size)
				break;
		}
	}

	if (toplvl)
		buf_pos = ice_emit_to_buf(buf, buf_size, buf_pos, ".");

	return buf_pos;
}

static void ice_dump_ethtool_stats_to_fmsg(struct devlink_fmsg *fmsg,
					   struct net_device *netdev)
{
	const u32 string_set = ETH_SS_STATS;
	u64 *stats;
	u8 *names;
	int scnt;

	scnt = ice_get_sset_count(netdev, string_set);
	devlink_fmsg_put(fmsg, "stats-cnt", (u32)scnt);
	if (scnt <= 0)
		return;

	names = kcalloc(scnt, ETH_GSTRING_LEN, GFP_KERNEL);
	stats = kcalloc(scnt, sizeof(*stats), GFP_KERNEL);
	if (!names || !stats)
		goto out;

	ice_get_strings(netdev, string_set, names);
	ice_get_ethtool_stats(netdev, NULL, stats);

	devlink_fmsg_obj_nest_start(fmsg);
	for (int i = 0; i < scnt; ++i)
		devlink_fmsg_put(fmsg, &names[i * ETH_GSTRING_LEN], stats[i]);
	devlink_fmsg_obj_nest_end(fmsg);
out:
	kfree(names);
	kfree(stats);
}

/**
 * ice_fmsg_put_ptr - put hex value of pointer into fmsg
 *
 * @fmsg: devlink fmsg under construction
 * @name: name to pass
 * @ptr: 64 bit value to print as hex and put into fmsg
 */
static void ice_fmsg_put_ptr(struct devlink_fmsg *fmsg, const char *name,
			     void *ptr)
{
	char buf[sizeof(ptr) * 3];

	sprintf(buf, "%p", ptr);
	devlink_fmsg_put(fmsg, name, buf);
}

struct ice_tx_hang_event {
	u32 head;
	u32 intr;
	u16 vsi_num;
	u16 queue;
	u16 next_to_clean;
	u16 next_to_use;
	struct ice_tx_ring *tx_ring;
};

static int
#ifndef HAVE_DEVLINK_HEALTH_OPS_EXTACK
ice_tx_hang_reporter_dump(struct devlink_health_reporter *reporter,
			  struct devlink_fmsg *fmsg, void *priv_ctx)
#else
ice_tx_hang_reporter_dump(struct devlink_health_reporter *reporter,
			  struct devlink_fmsg *fmsg, void *priv_ctx,
			  struct netlink_ext_ack __always_unused *extack)
#endif /* HAVE_DEVLINK_HEALTH_OPS_EXTACK */
{
	struct ice_tx_hang_event *event = priv_ctx;
	char *skb_txt = NULL;
	struct sk_buff *skb;

	if (!event)
		return 0;

	skb = event->tx_ring->tx_buf->skb;

	devlink_fmsg_obj_nest_start(fmsg);
	ICE_DEVLINK_FMSG_PUT_FIELD(fmsg, event, head);
	ICE_DEVLINK_FMSG_PUT_FIELD(fmsg, event, intr);
	ICE_DEVLINK_FMSG_PUT_FIELD(fmsg, event, vsi_num);
	ICE_DEVLINK_FMSG_PUT_FIELD(fmsg, event, queue);
	ICE_DEVLINK_FMSG_PUT_FIELD(fmsg, event, next_to_clean);
	ICE_DEVLINK_FMSG_PUT_FIELD(fmsg, event, next_to_use);
	devlink_fmsg_put(fmsg, "irq-mapping", event->tx_ring->q_vector->name);
	ice_fmsg_put_ptr(fmsg, "desc-ptr", event->tx_ring->desc);
	ice_fmsg_put_ptr(fmsg, "dma-ptr", (void *)event->tx_ring->dma);
	ice_fmsg_put_ptr(fmsg, "skb-ptr", skb);
	devlink_fmsg_binary_pair_put(fmsg, "desc", event->tx_ring->desc,
				     size_mul(event->tx_ring->count,
					      sizeof(struct ice_tx_desc)));
	if (skb)
		skb_txt = kvmalloc(4 * PAGE_SIZE, GFP_KERNEL);

	if (skb_txt) {
		ice_skb_dump_buf(skb_txt, 4 * PAGE_SIZE, 0, skb);
		devlink_fmsg_put(fmsg, "skb", skb_txt);
		kvfree(skb_txt);
	}
	ice_dump_ethtool_stats_to_fmsg(fmsg, event->tx_ring->vsi->netdev);
	devlink_fmsg_obj_nest_end(fmsg);

	return 0;
}

void ice_report_tx_hang(struct ice_pf *pf, struct ice_tx_ring *tx_ring,
			u16 vsi_num, u32 head, u32 intr)
{
	struct ice_tx_hang_event ev = {
		.head = head,
		.intr = intr,
		.vsi_num = vsi_num,
		.queue = tx_ring->q_index,
		.next_to_clean = tx_ring->next_to_clean,
		.next_to_use = tx_ring->next_to_use,
		.tx_ring = tx_ring,
	};

	ice_devlink_health_report(pf->health_reporters.tx_hang, "Tx hang", &ev);
}

static struct devlink_health_reporter *
ice_init_devlink_rep(struct ice_pf *pf,
		     const struct devlink_health_reporter_ops *ops)
{
	struct devlink *devlink = priv_to_devlink(pf);
	struct devlink_health_reporter *rep;
	const u64 graceful_period = 0;

	rep = devlink_health_reporter_create(devlink, ops, graceful_period, pf);
	if (IS_ERR(rep)) {
		struct device *dev = ice_pf_to_dev(pf);

		ice_dev_err_errno(dev, PTR_ERR(rep), "failed to create devlink %s health reporter",
				  ops->name);
		return NULL;
	}
	return rep;
}

#define ICE_DEFINE_HEALTH_REPORTER_OPS(_name) \
	static const struct devlink_health_reporter_ops ice_ ## _name ## _reporter_ops = { \
	.name = #_name, \
	.dump = ice_ ## _name ## _reporter_dump, \
}

ICE_DEFINE_HEALTH_REPORTER_OPS(mdd);
ICE_DEFINE_HEALTH_REPORTER_OPS(tx_hang);

/**
 * ice_health_init - allocate and init all ice devlink health reporters and
 * accompanied data
 *
 * @pf: PF struct
 */
void ice_health_init(struct ice_pf *pf)
{
	struct ice_health *reps = &pf->health_reporters;

	reps->mdd = ice_init_devlink_rep(pf, &ice_mdd_reporter_ops);
	reps->tx_hang = ice_init_devlink_rep(pf, &ice_tx_hang_reporter_ops);
}

/**
 * ice_deinit_devl_reporter - destroy given devlink health reporter
 * @reporter: reporter to destroy
 */
static void ice_deinit_devl_reporter(struct devlink_health_reporter *reporter)
{
	if (reporter)
#ifdef HAVE_DEVL_HEALTH_REPORTER_DESTROY
		devl_health_reporter_destroy(reporter);
#else
		devlink_health_reporter_destroy(reporter);
#endif
}

/**
 * ice_health_deinit - deallocate all ice devlink health reporters and
 * accompanied data
 *
 * @pf: PF struct
 */
void ice_health_deinit(struct ice_pf *pf)
{
#ifdef HAVE_DEVL_HEALTH_REPORTER_DESTROY
	struct devlink *devlink = priv_to_devlink(pf);

#endif
#ifdef HAVE_DEVL_HEALTH_REPORTER_DESTROY
	devl_lock(devlink);
#endif
	ice_deinit_devl_reporter(pf->health_reporters.mdd);
	ice_deinit_devl_reporter(pf->health_reporters.tx_hang);
#ifdef HAVE_DEVL_HEALTH_REPORTER_DESTROY
	devl_unlock(devlink);
#endif
}

static
void ice_health_assign_healthy_state(struct devlink_health_reporter *reporter)
{
	if (reporter)
		devlink_health_reporter_state_update(reporter,
				DEVLINK_HEALTH_REPORTER_STATE_HEALTHY);
}

/**
 * ice_health_clear - clear devlink health issues after a reset
 * @pf: the PF device structure
 *
 * Mark the PF in healthy state again after a reset has completed.
 */
void ice_health_clear(struct ice_pf *pf)
{
	ice_health_assign_healthy_state(pf->health_reporters.mdd);
	ice_health_assign_healthy_state(pf->health_reporters.tx_hang);
}

#endif /* HAVE_DEVLINK_HEALTH */
