/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2018-2024 Intel Corporation */

#ifndef _ICE_CPI_H_
#define _ICE_CPI_H_

#define CPI0_PHY1_CMD_DATA	0x7FD028
#define CPI0_LM1_CMD_DATA	0x7FD024
#define CPI_RETRIES_COUNT	10
#define CPI_RETRIES_CADENCE_MS	100
#define CPI_MAX_FEC_OPTIONS	8

#define CPI_OPCODE_PORT_STATE			0x1
#define CPI_OPCODE_PORT_STATE_DISABLE		BIT(15)
#define CPI_OPCODE_PORT_STATE_RX_READY		BIT(7)

#define CPI_OPCODE_PORT_MODE			0x3
#define CPI_OPCODE_PORT_MODE_PORT_WIDTH_M	GENMASK(10, 8)
#define CPI_OPCODE_PORT_MODE_SINGLE_LANE	0
#define CPI_OPCODE_PORT_MODE_TWO_LANE		1
#define CPI_OPCODE_PORT_MODE_FOUR_LANE		2
#define CPI_OPCODE_PORT_MODE_EIGHT_LANE		3
#define CPI_OPCODE_PORT_MODE_PORT_MODE_M	GENMASK(7, 0)
#define CPI_OPCODE_PORT_MODE_AN73		BIT(2)
#define CPI_OPCODE_PORT_MODE_10G_SFI		0x0C
#define CPI_OPCODE_PORT_MODE_25G_AUI		0x15
#define CPI_OPCODE_PORT_MODE_50G_LAUI_2		0x17
#define CPI_OPCODE_PORT_MODE_100G_CAUI_4	0x18
#define CPI_OPCODE_PORT_MODE_50G_AUI_2		0x19
#define CPI_OPCODE_PORT_MODE_50G_AUI_1		0x20
#define CPI_OPCODE_PORT_MODE_100G_AUI_4		0x1A
#define CPI_OPCODE_PORT_MODE_50G_AUI_1		0x20
#define CPI_OPCODE_PORT_MODE_100G_AUI_2		0x21

#define CPI_OPCODE_NEG_MODE			0x5
#define CPI_OPCODE_NEG_MODE_FEC_M		GENMASK(15, 12)
#define CPI_OPCODE_NEG_MODE_FEC_NONE		0x0
#define CPI_OPCODE_NEG_MODE_FEC_BASE_R		0x1
#define CPI_OPCODE_NEG_MODE_FEC_RS_528		0x2
#define CPI_OPCODE_NEG_MODE_FEC_RS_544		0x4

#define CPI_OPCODE_PMD_CONTROL			0xC
#define CPI_OPCODE_PMD_CONTROL_SFI		0x0
#define CPI_OPCODE_PMD_CONTROL_TRAINING		0x400

#define CPI_OPCODE_CURATE0			0x18
#define CPI_OPCODE_CURATE0_1000BASE_KX		BIT(3)
#define CPI_OPCODE_CURATE0_2500BASE_KX		BIT(4)
#define CPI_OPCODE_CURATE0_5GBASE_KR		BIT(6)
#define CPI_OPCODE_CURATE0_10GBASE_KR		BIT(8)
#define CPI_OPCODE_CURATE0_40GBASE_KR4		BIT(9)
#define CPI_OPCODE_CURATE0_40GBASE_CR4		BIT(10)

#define CPI_OPCODE_CURATE1			0x19
#define CPI_OPCODE_CURATE1_25GBASE_CR_KR_S	BIT(0)
#define CPI_OPCODE_CURATE1_25GBASE_CR_KR	BIT(1)
#define CPI_OPCODE_CURATE1_25GBASE_KR1		BIT(2)
#define CPI_OPCODE_CURATE1_50GBASE_KR2		BIT(3)
#define CPI_OPCODE_CURATE1_100GBASE_CR4		BIT(4)
#define CPI_OPCODE_CURATE1_100GBASE_KR4		BIT(5)
#define CPI_OPCODE_CURATE1_50GBASE_CP4_KP4	BIT(8)
#define CPI_OPCODE_CURATE1_100GBASE_CK_R2P4	BIT(9)
#define CPI_OPCODE_CURATE1_25GBASE_CR1		BIT(12)
#define CPI_OPCODE_CURATE1_50GBASE_CR2		BIT(13)
#define CPI_OPCODE_CURATE1_100GBASE_KP4		BIT(14)

#define CPI_OPCODE_AN_CONTROL			0x20
#define CPI_OPCODE_AN_CONTROL_CFG		0x0

#define CPI_OPCODE_CUFEC0			0x1E
#define CPI_OPCODE_CUFEC0_10GCL74CAP		BIT(0)
#define CPI_OPCODE_CUFEC0_10GCL74REQ		BIT(1)
#define CPI_OPCODE_CUFEC0_25GCL74CAP		BIT(2)
#define CPI_OPCODE_CUFEC0_25GCL74REQ		BIT(3)
#define CPI_OPCODE_CUFEC0_CL91CL108CAP		BIT(4)
#define CPI_OPCODE_CUFEC0_CL91REQ		BIT(5)
#define CPI_OPCODE_CUFEC0_CL108REQ		BIT(6)
#define CPI_OPCODE_CUFEC0_DISABLE		0

#define CPI_OPCODE_COMMAND			0xF
#define CPI_OPCODE_COMMAND_CMD_M		GENMASK(7, 0)
#define CPI_OPCODE_COMMAND_LANE_M		GENMASK(15, 12)
#define CPI_OPCODE_COMMAND_START_RESTART_CFG	0x1
#define CPI_OPCODE_COMMAND_RESET_PORT		0x9

#define CPI_OPCODE_PHY_CLK			0xF1
#define CPI_OPCODE_PHY_CLK_PHY_SEL_M		GENMASK(9, 6)
#define CPI_OPCODE_PHY_CLK_REF_CTRL_M		GENMASK(5, 4)
#define CPI_OPCODE_PHY_CLK_PORT_SEL		0
#define CPI_OPCODE_PHY_CLK_DISABLE		1
#define CPI_OPCODE_PHY_CLK_ENABLE		2
#define CPI_OPCODE_PHY_CLK_REF_SEL_M		GENMASK(3, 0)

#define CPI_OPCODE_PHY_PCS_RESET		0xF0
#define CPI_OPCODE_PHY_PCS_ONPI_RESET_VAL	0x3F

#define CPI_LM_CMD_REQ		1
#define CPI_LM_CMD_SET		1

#define PHY0	0
#define PHY1	1

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

union cpi_reg_lm_cmd_data {
	struct {
		u16 data;
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

int ice_cpi_exec(struct ice_hw *hw, u8 phy,
		 const struct ice_cpi_cmd *cmd,
		 struct ice_cpi_resp *resp);
int ice_cpi_ena_dis_clk_ref(struct ice_hw *hw, u8 port,
			    enum ice_e825c_ref_clk clk, bool enable);
int ice_cpi_select_clk_ref(struct ice_hw *hw, u8 phy, u8 port,
			   enum ice_e825c_ref_clk clk);
int ice_cpi_set_port_state(struct ice_hw *hw, u8 phy, u8 port, bool disable);
int ice_cpi_set_port_mode(struct ice_hw *hw, u8 phy, u8 port, u8 port_width,
			  u8 port_mode);
int ice_cpi_reset_port(struct ice_hw *hw, u8 phy, u8 port);
#endif /* _ICE_CPI_H_ */
