/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2018-2024 Intel Corporation */

#include "ice_type.h"
#include "ice_ptp_hw.h"
#include "ice_cpi.h"

/**
 * ice_cpi_wait_req0_ack0 - waits for CPI interface to be available
 * @hw: pointer to the HW struct
 * @phy: index of PHY the CPI action is taken on
 *
 * This function checks if CPI interface is ready to use by CPI client.
 * It's done by assuring LM.CMD.REQ and PHY.CMD.ACK bit in CPI
 * interface registers to be 0.
 *
 * Returns 0 on success.
 */
static int ice_cpi_wait_req0_ack0(struct ice_hw *hw, int phy)
{
	union cpi_reg_phy_cmd_data phy_regs;
	union cpi_reg_lm_cmd_data lm_regs;

	for (int i = 0; i < CPI_RETRIES_COUNT; i++) {
		int err;

		/* check if another CPI Client is also accessing CPI */
		err = ice_read_phy_eth56g_raw(hw, phy, CPI0_LM1_CMD_DATA,
					      &lm_regs.val);
		if (err)
			return err;
		if (lm_regs.field.cpi_req)
			return -EBUSY;

		/* check if PHY.ACK is deasserted */
		err = ice_read_phy_eth56g_raw(hw, phy, CPI0_PHY1_CMD_DATA,
					      &phy_regs.val);
		if (err)
			return err;
		if (phy_regs.field.error)
			return -EFAULT;
		if (!phy_regs.field.ack)
			/* req0 and ack0 at this point - ready to go */
			return 0;

		msleep(CPI_RETRIES_CADENCE_MS);
	};

	return -ETIMEDOUT;
}

/**
 * ice_cpi_wait_ack - Waits for the PHY.ACK bit to be asserted/deasserted
 * @hw: pointer to the HW struct
 * @phy: index of PHY the CPI action is taken on
 * @asserted: desired state of PHY.ACK bit
 * @data: pointer to the user data where PHY.data is stored
 *
 * This function checks if PHY.ACK bit is asserted or deasserted, depending
 * on the phase of CPI handshake. If 'asserted' state is required, PHY command
 * data is stored in the 'data' storage.
 *
 * Returns 0 on success.
 */
static int
ice_cpi_wait_ack(struct ice_hw *hw, u8 phy, bool asserted, u32 *data)
{
	union cpi_reg_phy_cmd_data phy_regs;

	for (int i = 0; i < CPI_RETRIES_COUNT; i++) {
		int err;

		err = ice_read_phy_eth56g_raw(hw, phy, CPI0_PHY1_CMD_DATA,
					      &phy_regs.val);
		if (err)
			return err;
		if (phy_regs.field.error)
			return -EFAULT;
		if (asserted && phy_regs.field.ack) {
			if (data)
				*data = phy_regs.val;
			return 0;
		}
		if (!asserted && !phy_regs.field.ack)
			return 0;

		msleep(CPI_RETRIES_CADENCE_MS);
	};

	return -ETIMEDOUT;
}

#define ice_cpi_wait_ack0(hw, phy) \
	ice_cpi_wait_ack(hw, phy, false, NULL)

#define ice_cpi_wait_ack1(hw, phy, data) \
	ice_cpi_wait_ack(hw, phy, true, data)

/**
 * ice_cpi_req0 - deasserts LM.REQ bit
 * @hw: pointer to the HW struct
 * @phy: index of PHY the CPI action is taken on
 * @data: the command data
 *
 * Returns 0 on success.
 */
static int ice_cpi_req0(struct ice_hw *hw, u8 phy, u32 data)
{
	union cpi_reg_lm_cmd_data *lm_regs;
	int err;

	lm_regs = (union cpi_reg_lm_cmd_data *)&data;
	lm_regs->field.cpi_req = 0;

	err = ice_write_phy_eth56g_raw(hw, phy, CPI0_LM1_CMD_DATA,
				       lm_regs->val);
	if (err)
		return err;

	return 0;
}

/**
 * ice_cpi_exec_cmd - writes command data to CPI interface
 * @hw: pointer to the HW struct
 * @phy: index of PHY the CPI action is taken on
 * @data: the command data
 *
 * Returns 0 on success.
 */
static int ice_cpi_exec_cmd(struct ice_hw *hw, int phy, u32 data)
{
	return ice_write_phy_eth56g_raw(hw, phy, CPI0_LM1_CMD_DATA, data);
}

/**
 * ice_cpi_cmd - executes CPI command
 * @hw: pointer to the HW struct
 * @port: port number
 * @cmd: pointer to the command struct to execute
 * @resp: pointer to user allocated CPI response struct
 *
 * This function executes CPI request with respect to CPI handshake
 * mechanism.
 *
 * Returns 0 on success.
 */
static int ice_cpi_cmd(struct ice_hw *hw, u8 port,
		       const struct ice_cpi_cmd *cmd,
		       struct ice_cpi_resp *resp)
{
	union cpi_reg_phy_cmd_data phy_cmd_data;
	union cpi_reg_lm_cmd_data lm_cmd_data;
	int phy, err = 0;

	if (!cmd || !resp)
		return -EINVAL;

	memset(&lm_cmd_data, 0, sizeof(lm_cmd_data));
	phy = port / hw->ptp.ports_per_phy;
	lm_cmd_data.field.cpi_req = CPI_LM_CMD_REQ;
	lm_cmd_data.field.get_set = cmd->set;
	lm_cmd_data.field.opcode = cmd->opcode;
	lm_cmd_data.field.portlane = cmd->port;
	lm_cmd_data.field.params.raw_data = cmd->data;

	/* 1. Try to acquire the bus, PHY ACK should be low before we begin */
	err = ice_cpi_wait_req0_ack0(hw, phy);
	if (err)
		return err;

	/* 2. We start the CPI request */
	err = ice_cpi_exec_cmd(hw, phy, lm_cmd_data.val);
	if (err)
		return err;

	/*
	 * 3. Wait for CPI confirmation, PHY ACK should be asserted and opcode
	 *    echoed in the response
	 */
	err = ice_cpi_wait_ack1(hw, phy, &phy_cmd_data.val);
	if (err)
		return err;
	if (phy_cmd_data.field.ack &&
	    lm_cmd_data.field.opcode != phy_cmd_data.field.opcode)
		return -EFAULT;

	resp->opcode = phy_cmd_data.field.opcode;
	resp->data = phy_cmd_data.field.data;
	resp->port = phy_cmd_data.field.data;

	/* 4. We deassert REQ */
	err = ice_cpi_req0(hw, phy, lm_cmd_data.val);
	if (err)
		return err;

	/* 5. PHY ACK should be deasserted in response */
	return  ice_cpi_wait_ack0(hw, phy);
}

/**
 * ice_cpi_ena_dis_clk_ref - enables/disables Tx reference clock on port
 * @hw: pointer to the HW struct
 * @port: port for which Tx reference clock is enabled/disabled
 * @clk: Tx reference clock to enable or disable
 * @enable: bool value to enable or disable Tx reference clock
 *
 * This function executes CPI request to enable or disable specific
 * Tx reference clock on given PHY.
 *
 * Returns 0 on success.
 */
int ice_cpi_ena_dis_clk_ref(struct ice_hw *hw, u8 port,
			    enum ice_e825c_ref_clk clk, bool enable)
{
	struct cpi_tx_clk_cfg *clk_ref_cfg;
	struct ice_cpi_resp cpi_resp = {0};
	struct ice_cpi_cmd cpi_cmd = {0};
	int err = 0;
	u8 phy;

	phy = port / hw->ptp.ports_per_phy;
	cpi_cmd.opcode = CPI_OPCODE_TX_CLK_CFG;
	cpi_cmd.set = true;
	cpi_cmd.port = 0; /* unused for enable/disable variant */

	clk_ref_cfg = (struct cpi_tx_clk_cfg *)&cpi_cmd.data;
	clk_ref_cfg->refclksrc = clk;
	clk_ref_cfg->en_dis_sel = enable ? CPI_TX_CLK_ENABLE :
	       CPI_TX_CLK_DISABLE;
	clk_ref_cfg->phy_sel = phy;

	err = ice_cpi_cmd(hw, port, &cpi_cmd, &cpi_resp);
	if (err)
		return err;

	return 0;
}
