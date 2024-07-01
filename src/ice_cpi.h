/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2018-2024 Intel Corporation */

#ifndef _ICE_CPI_H_
#define _ICE_CPI_H_

#define CPI0_PHY1_CMD_DATA	0x7FD028
#define CPI0_LM1_CMD_DATA	0x7FD024
#define CPI_RETRIES_COUNT	10
#define CPI_RETRIES_CADENCE_MS	100

#define CPI_OPCODE_TX_CLK_CFG	0xF1
#define CPI_LM_CMD_REQ		1
#define CPI_LM_CMD_SET		1
union cpi_reg_phy_cmd_data {
	struct {
		u16 data;
		u16 opcode : 8;
		u16 portlane : 3;
		u16 reserved_13_11: 3;
		u16 error : 1;
		u16 ack : 1;
	} field;
	u32 val;
};

struct cpi_tx_clk_cfg {
	u16 refclksrc : 4;
	u16 en_dis_sel : 2;
#define CPI_TX_CLK_PORT_SEL	0
#define CPI_TX_CLK_DISABLE	1
#define CPI_TX_CLK_ENABLE	2
	u16 phy_sel : 4;
	u16 reserved_15_10 : 6;
} __packed;

union cpi_reg_lm_cmd_data {
	struct {
		union {
			struct cpi_tx_clk_cfg tx_clk_cfg;
			u16 raw_data;
		} params;
		u16 opcode : 8;
		u16 portlane : 3;
		u16 reserved_12_11: 2;
		u16 get_set : 1;
		u16 cpi_reset : 1;
		u16 cpi_req : 1;
	} __packed field;
	u32 val;
};

struct ice_cpi_cmd {
	u8 port;
	u8 opcode;
	u16 data;
	bool set;
};

struct ice_cpi_resp {
	u8 port;
	u8 opcode;
	u16 data;
};

int ice_cpi_ena_dis_clk_ref(struct ice_hw *hw, u8 port,
			    enum ice_e825c_ref_clk clk, bool enable);
#endif /* _ICE_CPI_H_ */
