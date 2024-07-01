/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2018-2024 Intel Corporation */

#ifndef _ICE_PTP_H_
#define _ICE_PTP_H_

#include <linux/clocksource.h>
#include <linux/net_tstamp.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/ptp_classify.h>
#include <linux/highuid.h>
#include "kcompat_kthread.h"
#include "ice_ptp_hw.h"

enum ice_ptp_pin_e82x {
	TIME_SYNC_PIN_INDEX = 4,
	PPS_PIN_INDEX
};

/* DPLL REF_SW state */
enum ice_dpll_ref_sw_state {
	ICE_DPLL_REF_SW_DISABLE,
	ICE_DPLL_REF_SW_ENABLE,
	NUM_ICE_DPLL_REF_SW_STATE,
};

enum ice_ptp_pin_e810 {
	GPIO_20 = 0,
	GPIO_21,
	GPIO_22,
	GPIO_23,
	NUM_PTP_PIN_E810,
	NO_GPIO,
};

enum ice_ptp_pin_e810t {
	GNSS = 0,
	SMA1,
	UFL1,
	SMA2,
	UFL2,
	NUM_PTP_PINS_E810T,
	GPIO_NA = ICE_AQC_NVM_SDP_CFG_PIN_SIZE - 1,
	NO_PTP_PIN
};

#define N_CHAN_E810			3

#define E82X_CGU_RCLK_PHY_PINS_NUM	1
#define E82X_CGU_RCLK_PIN_NAME		"NAC_CLK_SYNCE0_PN"

#define ICE_CGU_IN_PIN_FAIL_FLAGS (ICE_AQC_GET_CGU_IN_CFG_STATUS_SCM_FAIL | \
				   ICE_AQC_GET_CGU_IN_CFG_STATUS_CFM_FAIL | \
				   ICE_AQC_GET_CGU_IN_CFG_STATUS_GST_FAIL | \
				   ICE_AQC_GET_CGU_IN_CFG_STATUS_PFM_FAIL)

#define ICE_DPLL_PIN_STATE_INVALID	"invalid"
#define ICE_DPLL_PIN_STATE_VALIDATING	"validating"
#define ICE_DPLL_PIN_STATE_VALID	"valid"

struct ice_perout_channel {
	bool ena;
	u32 gpio_pin;
	u64 period;
	u64 start_time;
};

struct ice_extts_channel {
	bool ena;
	u32 gpio_pin;
	u32 flags;
};

/* The ice hardware captures Tx hardware timestamps in the PHY. The timestamp
 * is stored in a buffer of registers. Depending on the specific hardware,
 * this buffer might be shared across multiple PHY ports.
 *
 * On transmit of a packet to be timestamped, software is responsible for
 * selecting an open index. Hardware makes no attempt to lock or prevent
 * re-use of an index for multiple packets.
 *
 * To handle this, timestamp indexes must be tracked by software to ensure
 * that an index is not re-used for multiple transmitted packets. The
 * structures and functions declared in this file track the available Tx
 * register indexes, as well as provide storage for the SKB pointers.
 *
 * To allow multiple ports to access the shared register block independently,
 * the blocks are split up so that indexes are assigned to each port based on
 * hardware logical port number.
 *
 * The timestamp blocks are handled differently for E810- and E82X-based
 * devices. In E810 devices, each port has its own block of timestamps, while in
 * E82X there is a need to logically break the block of registers into smaller
 * chunks based on the port number to avoid collisions.
 *
 * Example for port 5 in E810:
 *  +--------+--------+--------+--------+--------+--------+--------+--------+
 *  |register|register|register|register|register|register|register|register|
 *  | block  | block  | block  | block  | block  | block  | block  | block  |
 *  |  for   |  for   |  for   |  for   |  for   |  for   |  for   |  for   |
 *  | port 0 | port 1 | port 2 | port 3 | port 4 | port 5 | port 6 | port 7 |
 *  +--------+--------+--------+--------+--------+--------+--------+--------+
 *                                               ^^
 *                                               ||
 *                                               |---  quad offset is always 0
 *                                               ---- quad number
 *
 * Example for port 5 in E82X:
 * +-----------------------------+-----------------------------+
 * |  register block for quad 0  |  register block for quad 1  |
 * |+------+------+------+------+|+------+------+------+------+|
 * ||port 0|port 1|port 2|port 3|||port 0|port 1|port 2|port 3||
 * |+------+------+------+------+|+------+------+------+------+|
 * +-----------------------------+-------^---------------------+
 *                                ^      |
 *                                |      --- quad offset*
 *                                ---- quad number
 *
 *   * PHY port 5 is port 1 in quad 1
 *
 */

/**
 * struct ice_tx_tstamp - Tracking for a single Tx timestamp
 * @skb: pointer to the SKB for this timestamp request
 * @start: jiffies when the timestamp was first requested
 * @cached_tstamp: last read timestamp
 *
 * This structure tracks a single timestamp request. The SKB pointer is
 * provided when initiating a request. The start time is used to ensure that
 * we discard old requests that were not fulfilled within a 2 second time
 * window.
 * Timestamp values in the PHY are read only and do not get cleared except at
 * hardware reset or when a new timestamp value is captured.
 *
 * Some PHY types do not provide a "ready" bitmap indicating which timestamp
 * indexes are valid. In these cases, we use a cached_tstamp to keep track of
 * the last timestamp we read for a given index. If the current timestamp
 * value is the same as the cached value, we assume a new timestamp hasn't
 * been captured. This avoids reporting stale timestamps to the stack. This is
 * only done if the has_ready_bitmap flag is not set in ice_ptp_tx structure.
 */
struct ice_tx_tstamp {
	struct sk_buff *skb;
	unsigned long start;
	u64 cached_tstamp;
};

/**
 * enum ice_tx_tstamp_work - Status of Tx timestamp work function
 * @ICE_TX_TSTAMP_WORK_DONE: Tx timestamp processing is complete
 * @ICE_TX_TSTAMP_WORK_PENDING: More Tx timestamps are pending
 */
enum ice_tx_tstamp_work {
	ICE_TX_TSTAMP_WORK_DONE = 0,
	ICE_TX_TSTAMP_WORK_PENDING,
};

/**
 * struct ice_ptp_pin_desc - hardware pin description data
 * @pin_name_idx: index of the name of pin in ice_pin_names_e810t
 * @index: index of pin in pin_config interface
 * @func: the initial function assigned to the pin at load
 * @chan: the matching channel assigned to this pin
 * @gpio_out: the associated output GPIO pin
 * @gpio_in: the associated input GPIO pin
 *
 * Structure describing a PTP-capable GPIO pin. Used to setup the ptp_pin_desc
 * array for the device. Device families have separate sets of available pins
 * with varying restrictions.
 */
struct ice_ptp_pin_desc {
	u8 pin_name_idx;
	int index;
	unsigned int func;
	unsigned int chan;
	int gpio_out;
	int gpio_in;
};

/**
 * struct ice_ptp_tx - Tracking structure for all Tx timestamp requests on a port
 * @lock: lock to prevent concurrent access to fields of this struct
 * @tstamps: array of len to store outstanding requests
 * @in_use: bitmap of len to indicate which slots are in use
 * @block: which memory block (quad or port) the timestamps are captured in
 * @offset: offset into timestamp block to get the real index
 * @len: length of the tstamps and in_use fields.
 * @init: if true, the tracker is initialized;
 * @calibrating: if true, the PHY is calibrating the Tx offset. During this
 *               window, timestamps are temporarily disabled.
 * @has_ready_bitmap: if true, the hardware has a valid Tx timestamp ready
 *                    bitmap register. If false, fall back to verifying new
 *                    timestamp values against previously cached copy.
 * @last_ll_ts_idx_read: index of the last LL TS read by the FW
 */
struct ice_ptp_tx {
	spinlock_t lock; /* lock protecting in_use bitmap */
	struct ice_tx_tstamp *tstamps;
	unsigned long *in_use;
	u8 block;
	u8 offset;
	u8 len;
	u8 init : 1;
	u8 calibrating : 1;
	u8 has_ready_bitmap : 1;
	s8 last_ll_ts_idx_read;
};

/* Quad and port information for initializing timestamp blocks */
#define INDEX_PER_QUAD			64
#define INDEX_PER_PORT_E82X		16
#define INDEX_PER_PORT_E810		64
#define INDEX_PER_PORT_ETH56G		64

/**
 * struct ice_ptp_port - data used to initialize an external port for PTP
 *
 * This structure contains data indicating whether a single external port is
 * ready for PTP functionality. It is used to track the port initialization
 * and determine when the port's PHY offset is valid.
 *
 * @list_member: list member structure of auxiliary device
 * @tx: Tx timestamp tracking for this port
 * @aux_dev: auxiliary device associated with this port
 * @ov_work: delayed work task for tracking when PHY offset is valid
 * @ps_lock: mutex used to protect the overall PTP PHY start procedure
 * @link_up: indicates whether the link is up
 * @tx_fifo_busy_cnt: number of times the Tx FIFO was busy
 * @port_num: the port number this structure represents
 * @tx_clk: Tx reference clock source
 */
struct ice_ptp_port {
	struct list_head list_member;
	struct ice_ptp_tx tx;
	struct auxiliary_device aux_dev;
	struct kthread_delayed_work ov_work;
	struct mutex ps_lock; /* protects overall PTP PHY start procedure */
	bool link_up;
	u8 tx_fifo_busy_cnt;
	u8 port_num;
	u8 rx_calibrating : 1;
	enum ice_e825c_ref_clk tx_clk;
};

#define GLTSYN_TGT_H_IDX_MAX		4
#define GLTSYN_EVNT_H_IDX_MAX		3

enum ice_ptp_state {
	ICE_PTP_UNINIT = 0,
	ICE_PTP_INITIALIZING,
	ICE_PTP_READY,
	ICE_PTP_RESETTING,
	ICE_PTP_ERROR,
};

enum ice_ptp_tx_interrupt {
	ICE_PTP_TX_INTERRUPT_NONE = 0,
	ICE_PTP_TX_INTERRUPT_SELF,
	ICE_PTP_TX_INTERRUPT_ALL,
};

/**
 * struct ice_ptp_port_owner - data used to handle the PTP clock owner info
 *
 * This structure contains data necessary for the PTP clock owner to correctly
 * handle the timestamping feature for all attached ports.
 *
 * @aux_driver: the structure carring the auxiliary driver information
 * @ports: list of ports handled by this port owner
 * @lock: protect access to ports list
 * @tx_refclks: bitmaps table to store the information about TX reference clocks
 * @kworker: kwork thread for handling periodic work
 * @work: delayed work function for periodic tasks
 */
struct ice_ptp_port_owner {
	struct auxiliary_driver aux_driver;
	struct list_head ports;
	struct mutex lock;
#define ICE_E825_MAX_PHYS 2
	unsigned long tx_refclks[ICE_E825_MAX_PHYS][ICE_REF_CLK_MAX];
	struct kthread_worker *kworker;
	struct kthread_delayed_work work;
};

/**
 * struct ice_ptp - data used for integrating with CONFIG_PTP_1588_CLOCK
 * @state: current state of PTP state machine
 * @tx_interrupt_mode: the TX interrupt mode for the PHC device
 * @port: data for the PHY port initialization procedure
 * @ports_owner: data for the auxiliary driver owner
 * @cached_phc_time: a cached copy of the PHC time for timestamp extension
 * @cached_phc_jiffies: jiffies when cached_phc_time was last updated
 * @ext_ts_chan: the external timestamp channel in use
 * @ext_ts_irq: the external timestamp IRQ in use
 * @perout_channels: periodic output data
 * @extts_channels: channels for external timestamps
 * @info: structure defining PTP hardware capabilities
 * @clock: pointer to registered PTP clock device
 * @tstamp_config: hardware timestamping configuration
 * @sdp_section_exist: true if exists
 * @ice_pin_desc: structure defining pins
 * @ice_pin_desc_num: how many pin_desc structs
 * @phy_kobj: pointer to phy sysfs object
 * @reset_time: kernel time after clock stop on reset
 * @tx_hwtstamp_skipped: number of Tx time stamp requests skipped
 * @tx_hwtstamp_timeouts: number of Tx skbs discarded with no time stamp
 * @tx_hwtstamp_flushed: number of Tx skbs flushed due to interface closed
 * @tx_hwtstamp_discarded: number of Tx skbs discarded due to cached PHC time
 *                         being too old to correctly extend timestamp
 * @late_cached_phc_updates: number of times cached PHC update is late
 */
struct ice_ptp {
	enum ice_ptp_state state;
	enum ice_ptp_tx_interrupt tx_interrupt_mode;
	struct ice_ptp_port port;
	struct ice_ptp_port_owner ports_owner;
	u64 cached_phc_time;
	unsigned long cached_phc_jiffies;
	u8 ext_ts_chan;
	u8 ext_ts_irq;
	struct ice_perout_channel perout_channels[GLTSYN_TGT_H_IDX_MAX];
	struct ice_extts_channel extts_channels[GLTSYN_EVNT_H_IDX_MAX];
	struct ptp_clock_info info;
	struct ptp_clock *clock;
	struct hwtstamp_config tstamp_config;
	bool sdp_section_exist;
	struct ice_ptp_pin_desc *ice_pin_desc;
	u8 ice_pin_desc_num;
	struct kobject *phy_kobj;
	struct kobject *ptp_802_3cx_kobj;
	u64 reset_time;
	u32 tx_hwtstamp_skipped;
	u32 tx_hwtstamp_timeouts;
	u32 tx_hwtstamp_flushed;
	u32 tx_hwtstamp_discarded;
	u32 late_cached_phc_updates;
};

static inline struct ice_ptp *__ptp_port_to_ptp(struct ice_ptp_port *p)
{
	return container_of(p, struct ice_ptp, port);
}

#define ptp_port_to_pf(p) \
	container_of(__ptp_port_to_ptp((p)), struct ice_pf, ptp)

static inline struct ice_ptp *__ptp_info_to_ptp(struct ptp_clock_info *i)
{
	return container_of(i, struct ice_ptp, info);
}

#define ptp_info_to_pf(i) \
	container_of(__ptp_info_to_ptp((i)), struct ice_pf, ptp)

static inline int ice_ptp_get_seq_id(struct sk_buff *skb)
{
	struct ptp_header *hdr;
	int type;

	if (!skb)
		return -1;

	type = ptp_classify_raw(skb);

	if (type == PTP_CLASS_NONE)
		return -1;

	hdr = ptp_parse_header(skb, type);
	if (!hdr)
		return -1;

	return be16_to_cpu(hdr->sequence_id);
}

#define MAC_RX_LINK_COUNTER(_port)	(0x600090 + 0x1000 * (_port))
#define PFTSYN_SEM_BYTES		4
#define PTP_SHARED_CLK_IDX_VALID	BIT(31)
#define PHY_TIMER_SELECT_VALID_BIT	0
#define PHY_TIMER_SELECT_BIT		1
#define PHY_TIMER_SELECT_MASK		0xFFFFFFFC
#define TS_CMD_MASK_EXT			0xFF
#define TS_CMD_MASK			0xF
#define SYNC_EXEC_CMD			0x3
#define ICE_PTP_TS_VALID		BIT(0)
#define FIFO_EMPTY			BIT(2)
#define FIFO_OK				0xFF
#define ICE_PTP_FIFO_NUM_CHECKS		5
#define TX_INTR_QUAD_MASK		0x1Fu
/* Per-channel register definitions */
#define GLTSYN_AUX_OUT(_chan, _idx)	(GLTSYN_AUX_OUT_0(_idx) + ((_chan) * 8))
#define GLTSYN_AUX_IN(_chan, _idx)	(GLTSYN_AUX_IN_0(_idx) + ((_chan) * 8))
#define GLTSYN_CLKO(_chan, _idx)	(GLTSYN_CLKO_0(_idx) + ((_chan) * 8))
#define GLTSYN_TGT_L(_chan, _idx)	(GLTSYN_TGT_L_0(_idx) + ((_chan) * 16))
#define GLTSYN_TGT_H(_chan, _idx)	(GLTSYN_TGT_H_0(_idx) + ((_chan) * 16))
#define GLTSYN_EVNT_L(_chan, _idx)	(GLTSYN_EVNT_L_0(_idx) + ((_chan) * 16))
#define GLTSYN_EVNT_H(_chan, _idx)	(GLTSYN_EVNT_H_0(_idx) + ((_chan) * 16))
#define GLTSYN_EVNT_H_IDX_MAX		3

/* Pin definitions for PTP PPS out */
#define PPS_CLK_GEN_CHAN		3
#define N_EXT_TS_E810			3
#define N_PER_OUT_E810			4
#define N_PER_OUT_E810T			3
#define N_PER_OUT_NO_SMA_E810T		2
#define N_EXT_TS_NO_SMA_E810T		2
/* Macros to derive the low and high addresses for PHY */
#define LOWER_ADDR_SIZE			16
/* Macros to derive offsets for TimeStampLow and TimeStampHigh */
#define PORT_TIMER_ASSOC(_i)		(0x0300102C + ((_i) * 256))
#define ETH_GLTSYN_ENA(_i)		(0x03000348 + ((_i) * 4))

#define MAX_PIN_NAME			15

#define	ICE_PTP_PIN_FREQ_1HZ		1
#define	ICE_PTP_PIN_FREQ_10MHZ		10000000

/* Time allowed for programming periodic clock output */
#define START_OFFS_NS 100000000

#define ICE_PTP_PIN_INVALID		0xFF

/* "dpll <x> pin <y> prio <z>" (always 6 arguments) */
#define ICE_PTP_PIN_PRIO_ARG_CNT	6

/*
 * Examples of possible argument lists and count:
 * "in pin <n> enable <0/1>"
 * "out pin <n> enable <0/1> freq <x>"
 * "in pin <n> freq <x>"
 * "out pin <n> freq <x> esync <z>"
 * "in pin <n> freq <x> phase_delay <y> esync <0/1>"
 * "out pin <n> enable <0/1> freq <x> phase_delay <y> esync <0/1>"
 *
 * count = 3 + x * 2
 * 3 = target pin arguments (<dir> pin <n>)
 * x = int [1-4]  (up to 4: 'param name' + 'value' pairs)
 * 2 = count of args in pair ('param name' + 'value')
 */
#define ICE_PTP_PIN_CFG_1_ARG_CNT	5
#define ICE_PTP_PIN_CFG_2_ARG_CNT	7
#define ICE_PTP_PIN_CFG_3_ARG_CNT	9
#define ICE_PTP_PIN_CFG_4_ARG_CNT	11

#if IS_ENABLED(CONFIG_PTP_1588_CLOCK)
bool ice_is_ptp_supported(struct ice_pf *pf);
struct ice_pf;
int ice_ptp_set_ts_config(struct ice_pf *pf, struct ifreq *ifr);
int ice_ptp_get_ts_config(struct ice_pf *pf, struct ifreq *ifr);
void ice_block_ptp_workthreads_global(struct ice_pf *pf, bool block_enable);
void ice_ptp_restore_timestamp_mode(struct ice_pf *pf);

void ice_ptp_extts_event(struct ice_pf *pf);
s8 ice_ptp_request_ts(struct ice_ptp_tx *tx, struct sk_buff *skb);
enum ice_tx_tstamp_work ice_ptp_process_ts(struct ice_pf *pf);
void ice_ptp_req_tx_single_tstamp(struct ice_ptp_tx *tx, u8 idx);
void ice_ptp_complete_tx_single_tstamp(struct ice_ptp_tx *tx);

u64
ice_ptp_read_src_clk_reg(struct ice_pf *pf, struct ptp_system_timestamp *sts);
void
ice_ptp_rx_hwtstamp(struct ice_rx_ring *rx_ring,
		    union ice_32b_rx_flex_desc *rx_desc, struct sk_buff *skb);
void ice_ptp_rebuild(struct ice_pf *pf, enum ice_reset_req reset_type);
void
ice_ptp_prepare_for_reset(struct ice_pf *pf, enum ice_reset_req reset_type);
void ice_ptp_init(struct ice_pf *pf);
void ice_ptp_release(struct ice_pf *pf);
void ice_ptp_link_change(struct ice_pf *pf, u8 port, bool linkup);
int ice_ptp_check_rx_fifo(struct ice_pf *pf, u8 port);
int ice_ptp_update_incval(struct ice_pf *pf, enum ice_time_ref_freq time_ref_freq,
			  enum ice_src_tmr_mode src_tmr_mode);
void ice_dpll_pin_idx_to_name(struct ice_pf *pf, u8 pin, char *pin_name);
int ice_ptp_phy_restart(struct ice_pf *pf);
int ice_ptp_clock_index(struct ice_pf *pf);
#else /* IS_ENABLED(CONFIG_PTP_1588_CLOCK) */
static inline bool ice_is_ptp_supported(struct ice_pf *pf)
{
	return false;
}
static inline int ice_ptp_set_ts_config(struct ice_pf *pf, struct ifreq *ifr)
{
	return -EOPNOTSUPP;
}

static inline int ice_ptp_get_ts_config(struct ice_pf *pf, struct ifreq *ifr)
{
	return -EOPNOTSUPP;
}

static inline void
ice_block_ptp_workthreads_global(struct ice_pf *pf, bool block_enable) { }

static inline void ice_ptp_restore_timestamp_mode(struct ice_pf *pf) { }
static inline void ice_ptp_extts_event(struct ice_pf *pf) { }
static inline s8
ice_ptp_request_ts(struct ice_ptp_tx *tx, struct sk_buff *skb)
{
	return -1;
}

static inline bool ice_ptp_process_ts(struct ice_pf *pf)
{
	return true;
}

static inline void ice_ptp_req_tx_single_tstamp(struct ice_ptp_tx *tx, u8 idx)
{ }

static inline void ice_ptp_complete_tx_single_tstamp(struct ice_ptp_tx *tx) { }

static inline void
ice_ptp_rx_hwtstamp(struct ice_rx_ring *rx_ring,
		    union ice_32b_rx_flex_desc *rx_desc, struct sk_buff *skb) { }

static inline void
ice_ptp_rebuild(struct ice_pf *pf, enum ice_reset_req reset_type)
{
}

static inline void ice_ptp_prepare_for_reset(struct ice_pf *pf,
					     enum ice_reset_req reset_type)
{
}

static inline void ice_ptp_init(struct ice_pf *pf) { }
static inline void ice_ptp_release(struct ice_pf *pf) { }
static inline void ice_ptp_link_change(struct ice_pf *pf, u8 port, bool linkup)
{
}
static inline int ice_ptp_clock_index(struct ice_pf *pf)
{
	return -1;
}
#endif /* IS_ENABLED(CONFIG_PTP_1588_CLOCK) */
#endif /* _ICE_PTP_H_ */
