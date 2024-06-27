/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2018-2024 Intel Corporation */

#ifndef _ICE_PTP_CONSTS_H_
#define _ICE_PTP_CONSTS_H_

/* Constant definitions related to the hardware clock used for PTP 1588
 * features and functionality.
 */
/* Constants defined for the PTP 1588 clock hardware. */

const struct ice_phy_reg_info_eth56g eth56g_phy_res[NUM_ETH56G_PHY_RES] = {
	/* ETH56G_PHY_REG_PTP */
	{
		/* base_addr */
		{
			0x092000,
			0x126000,
			0x1BA000,
			0x24E000,
			0x2E2000,
		},
		/* step */
		0x98,
	},
	/* ETH56G_PHY_MEM_PTP */
	{
		/* base_addr */
		{
			0x093000,
			0x127000,
			0x1BB000,
			0x24F000,
			0x2E3000,
		},
		/* step */
		0x200,
	},
	/* ETH56G_PHY_REG_XPCS */
	{
		/* base_addr */
		{
			0x000000,
			0x009400,
			0x128000,
			0x1BC000,
			0x250000,
		},
		/* step */
		0x21000,
	},
	/* ETH56G_PHY_REG_MAC */
	{
		/* base_addr */
		{
			0x085000,
			0x119000,
			0x1AD000,
			0x241000,
			0x2D5000,
		},
		/* step */
		0x1000,
	},
	/* ETH56G_PHY_REG_GPCS */
	{
		/* base_addr */
		{
			0x084000,
			0x118000,
			0x1AC000,
			0x240000,
			0x2D4000,
		},
		/* step */
		0x400,
	},
};

/*
 * struct ice_time_ref_info_e82x
 *
 * E82X hardware can use different sources as the reference for the PTP
 * hardware clock. Each clock has different characteristics such as a slightly
 * different frequency, etc.
 *
 * This lookup table defines several constants that depend on the current time
 * reference. See the struct ice_time_ref_info_e82x for information about the
 * meaning of each constant.
 */
const struct ice_time_ref_info_e82x e82x_time_ref[NUM_ICE_TIME_REF_FREQ] = {
	/* ICE_TIME_REF_FREQ_25_000 -> 25 MHz */
	{
		/* pll_freq */
		823437500, /* 823.4375 MHz PLL */
		/* nominal_incval */
		0x136e44fabULL,
		/* pps_delay */
		11,
	},

	/* ICE_TIME_REF_FREQ_122_880 -> 122.88 MHz */
	{
		/* pll_freq */
		783360000, /* 783.36 MHz */
		/* nominal_incval */
		0x146cc2177ULL,
		/* pps_delay */
		12,
	},

	/* ICE_TIME_REF_FREQ_125_000 -> 125 MHz */
	{
		/* pll_freq */
		796875000, /* 796.875 MHz */
		/* nominal_incval */
		0x141414141ULL,
		/* pps_delay */
		12,
	},

	/* ICE_TIME_REF_FREQ_153_600 -> 153.6 MHz */
	{
		/* pll_freq */
		816000000, /* 816 MHz */
		/* nominal_incval */
		0x139b9b9baULL,
		/* pps_delay */
		12,
	},

	/* ICE_TIME_REF_FREQ_156_250 -> 156.25 MHz */
	{
		/* pll_freq */
		830078125, /* 830.78125 MHz */
		/* nominal_incval */
		0x134679aceULL,
		/* pps_delay */
		11,
	},

	/* ICE_TIME_REF_FREQ_245_760 -> 245.76 MHz */
	{
		/* pll_freq */
		783360000, /* 783.36 MHz */
		/* nominal_incval */
		0x146cc2177ULL,
		/* pps_delay */
		12,
	},
};

const struct ice_cgu_pll_params_e82x e82x_cgu_params[NUM_ICE_TIME_REF_FREQ] = {
	/* ICE_TIME_REF_FREQ_25_000 -> 25 MHz */
	{
		/* refclk_pre_div */
		1,
		/* feedback_div */
		197,
		/* frac_n_div */
		2621440,
		/* post_pll_div */
		6,
	},

	/* ICE_TIME_REF_FREQ_122_880 -> 122.88 MHz */
	{
		/* refclk_pre_div */
		5,
		/* feedback_div */
		223,
		/* frac_n_div */
		524288,
		/* post_pll_div */
		7,
	},

	/* ICE_TIME_REF_FREQ_125_000 -> 125 MHz */
	{
		/* refclk_pre_div */
		5,
		/* feedback_div */
		223,
		/* frac_n_div */
		524288,
		/* post_pll_div */
		7,
	},

	/* ICE_TIME_REF_FREQ_153_600 -> 153.6 MHz */
	{
		/* refclk_pre_div */
		5,
		/* feedback_div */
		159,
		/* frac_n_div */
		1572864,
		/* post_pll_div */
		6,
	},

	/* ICE_TIME_REF_FREQ_156_250 -> 156.25 MHz */
	{
		/* refclk_pre_div */
		5,
		/* feedback_div */
		159,
		/* frac_n_div */
		1572864,
		/* post_pll_div */
		6,
	},

	/* ICE_TIME_REF_FREQ_245_760 -> 245.76 MHz */
	{
		/* refclk_pre_div */
		10,
		/* feedback_div */
		223,
		/* frac_n_div */
		524288,
		/* post_pll_div */
		7,
	},
};

const
struct ice_cgu_pll_params_e825c e825c_cgu_params[NUM_ICE_TIME_REF_FREQ] = {
	/* ICE_TIME_REF_FREQ_25_000 -> 25 MHz */
	{
		/* tspll_ck_refclkfreq */
		0x19,
		/* tspll_ndivratio */
		1,
		/* tspll_fbdiv_intgr */
		320,
		/* tspll_fbdiv_frac */
		0,
		/* ref1588_ck_div */
		0,
	},

	/* ICE_TIME_REF_FREQ_122_880 -> 122.88 MHz */
	{
		/* tspll_ck_refclkfreq */
		0x29,
		/* tspll_ndivratio */
		3,
		/* tspll_fbdiv_intgr */
		195,
		/* tspll_fbdiv_frac */
		1342177280,
		/* ref1588_ck_div */
		0,
	},

	/* ICE_TIME_REF_FREQ_125_000 -> 125 MHz */
	{
		/* tspll_ck_refclkfreq */
		0x3E,
		/* tspll_ndivratio */
		2,
		/* tspll_fbdiv_intgr */
		128,
		/* tspll_fbdiv_frac */
		0,
		/* ref1588_ck_div */
		0,
	},

	/* ICE_TIME_REF_FREQ_153_600 -> 153.6 MHz */
	{
		/* tspll_ck_refclkfreq */
		0x33,
		/* tspll_ndivratio */
		3,
		/* tspll_fbdiv_intgr */
		156,
		/* tspll_fbdiv_frac */
		1073741824,
		/* ref1588_ck_div */
		0,
	},

	/* ICE_TIME_REF_FREQ_156_250 -> 156.25 MHz */
	{
		/* tspll_ck_refclkfreq */
		0x1F,
		/* tspll_ndivratio */
		5,
		/* tspll_fbdiv_intgr */
		256,
		/* tspll_fbdiv_frac */
		0,
		/* ref1588_ck_div */
		0,
	},

	/* ICE_TIME_REF_FREQ_245_760 -> 245.76 MHz */
	{
		/* tspll_ck_refclkfreq */
		0x52,
		/* tspll_ndivratio */
		3,
		/* tspll_fbdiv_intgr */
		97,
		/* tspll_fbdiv_frac */
		2818572288,
		/* ref1588_ck_div */
		0,
	},
};

/*
 * struct ice_vernier_info_e82x
 *
 * E82X hardware calibrates the delay of the timestamp indication from the
 * actual packet transmission or reception during the initialization of the
 * PHY. To do this, the hardware mechanism uses some conversions between the
 * various clocks within the PHY block. This table defines constants used to
 * calculate the correct conversion ratios in the PHY registers.
 *
 * Many of the values relate to the PAR/PCS clock conversion registers. For
 * these registers, a value of 0 means that the associated register is not
 * used by this link speed, and that the register should be cleared by writing
 * 0. Other values specify the clock frequency in Hz.
 */
const struct ice_vernier_info_e82x e82x_vernier[NUM_ICE_PTP_LNK_SPD] = {
	/* ICE_PTP_LNK_SPD_1G */
	{
		/* tx_par_clk */
		31250000, /* 31.25 MHz */
		/* rx_par_clk */
		31250000, /* 31.25 MHz */
		/* tx_pcs_clk */
		125000000, /* 125 MHz */
		/* rx_pcs_clk */
		125000000, /* 125 MHz */
		/* tx_desk_rsgb_par */
		0, /* unused */
		/* rx_desk_rsgb_par */
		0, /* unused */
		/* tx_desk_rsgb_pcs */
		0, /* unused */
		/* rx_desk_rsgb_pcs */
		0, /* unused */
		/* tx_fixed_delay */
		25140,
		/* pmd_adj_divisor */
		10000000,
		/* rx_fixed_delay */
		17372,
	},
	/* ICE_PTP_LNK_SPD_10G */
	{
		/* tx_par_clk */
		257812500, /* 257.8125 MHz */
		/* rx_par_clk */
		257812500, /* 257.8125 MHz */
		/* tx_pcs_clk */
		156250000, /* 156.25 MHz */
		/* rx_pcs_clk */
		156250000, /* 156.25 MHz */
		/* tx_desk_rsgb_par */
		0, /* unused */
		/* rx_desk_rsgb_par */
		0, /* unused */
		/* tx_desk_rsgb_pcs */
		0, /* unused */
		/* rx_desk_rsgb_pcs */
		0, /* unused */
		/* tx_fixed_delay */
		6938,
		/* pmd_adj_divisor */
		82500000,
		/* rx_fixed_delay */
		6212,
	},
	/* ICE_PTP_LNK_SPD_25G */
	{
		/* tx_par_clk */
		644531250, /* 644.53125 MHZ */
		/* rx_par_clk */
		644531250, /* 644.53125 MHz */
		/* tx_pcs_clk */
		390625000, /* 390.625 MHz */
		/* rx_pcs_clk */
		390625000, /* 390.625 MHz */
		/* tx_desk_rsgb_par */
		0, /* unused */
		/* rx_desk_rsgb_par */
		0, /* unused */
		/* tx_desk_rsgb_pcs */
		0, /* unused */
		/* rx_desk_rsgb_pcs */
		0, /* unused */
		/* tx_fixed_delay */
		2778,
		/* pmd_adj_divisor */
		206250000,
		/* rx_fixed_delay */
		2491,
	},
	/* ICE_PTP_LNK_SPD_25G_RS */
	{
		/* tx_par_clk */
		0, /* unused */
		/* rx_par_clk */
		0, /* unused */
		/* tx_pcs_clk */
		0, /* unused */
		/* rx_pcs_clk */
		0, /* unused */
		/* tx_desk_rsgb_par */
		161132812, /* 162.1328125 MHz Reed Solomon gearbox */
		/* rx_desk_rsgb_par */
		161132812, /* 162.1328125 MHz Reed Solomon gearbox */
		/* tx_desk_rsgb_pcs */
		97656250, /* 97.62625 MHz Reed Solomon gearbox */
		/* rx_desk_rsgb_pcs */
		97656250, /* 97.62625 MHz Reed Solomon gearbox */
		/* tx_fixed_delay */
		3928,
		/* pmd_adj_divisor */
		206250000,
		/* rx_fixed_delay */
		29535,
	},
	/* ICE_PTP_LNK_SPD_40G */
	{
		/* tx_par_clk */
		257812500,
		/* rx_par_clk */
		257812500,
		/* tx_pcs_clk */
		156250000, /* 156.25 MHz */
		/* rx_pcs_clk */
		156250000, /* 156.25 MHz */
		/* tx_desk_rsgb_par */
		0, /* unused */
		/* rx_desk_rsgb_par */
		156250000, /* 156.25 MHz deskew clock */
		/* tx_desk_rsgb_pcs */
		0, /* unused */
		/* rx_desk_rsgb_pcs */
		156250000, /* 156.25 MHz deskew clock */
		/* tx_fixed_delay */
		5666,
		/* pmd_adj_divisor */
		82500000,
		/* rx_fixed_delay */
		4244,
	},
	/* ICE_PTP_LNK_SPD_50G */
	{
		/* tx_par_clk */
		644531250, /* 644.53125 MHZ */
		/* rx_par_clk */
		644531250, /* 644.53125 MHZ */
		/* tx_pcs_clk */
		390625000, /* 390.625 MHz */
		/* rx_pcs_clk */
		390625000, /* 390.625 MHz */
		/* tx_desk_rsgb_par */
		0, /* unused */
		/* rx_desk_rsgb_par */
		195312500, /* 193.3125 MHz deskew clock */
		/* tx_desk_rsgb_pcs */
		0, /* unused */
		/* rx_desk_rsgb_pcs */
		195312500, /* 193.3125 MHz deskew clock */
		/* tx_fixed_delay */
		2778,
		/* pmd_adj_divisor */
		206250000,
		/* rx_fixed_delay */
		2868,
	},
	/* ICE_PTP_LNK_SPD_50G_RS */
	{
		/* tx_par_clk */
		0, /* unused */
		/* rx_par_clk */
		644531250, /* 644.53125 MHz */
		/* tx_pcs_clk */
		0, /* unused */
		/* rx_pcs_clk */
		644531250, /* 644.53125 MHz */
		/* tx_desk_rsgb_par */
		322265625, /* 322.265625 MHz Reed Solomon gearbox */
		/* rx_desk_rsgb_par */
		322265625, /* 322.265625 MHz Reed Solomon gearbox */
		/* tx_desk_rsgb_pcs */
		644531250, /* 644.53125 MHz Reed Solomon gearbox */
		/* rx_desk_rsgb_pcs */
		644531250, /* 644.53125 MHz Reed Solomon gearbox */
		/* tx_fixed_delay */
		2095,
		/* pmd_adj_divisor */
		206250000,
		/* rx_fixed_delay */
		14524,
	},
	/* ICE_PTP_LNK_SPD_100G_RS */
	{
		/* tx_par_clk */
		0, /* unused */
		/* rx_par_clk */
		644531250, /* 644.53125 MHz */
		/* tx_pcs_clk */
		0, /* unused */
		/* rx_pcs_clk */
		644531250, /* 644.53125 MHz */
		/* tx_desk_rsgb_par */
		644531250, /* 644.53125 MHz Reed Solomon gearbox */
		/* rx_desk_rsgb_par */
		644531250, /* 644.53125 MHz Reed Solomon gearbox */
		/* tx_desk_rsgb_pcs */
		644531250, /* 644.53125 MHz Reed Solomon gearbox */
		/* rx_desk_rsgb_pcs */
		644531250, /* 644.53125 MHz Reed Solomon gearbox */
		/* tx_fixed_delay */
		1620,
		/* pmd_adj_divisor */
		206250000,
		/* rx_fixed_delay */
		7775,
	},
};

#endif /* _ICE_PTP_CONSTS_H_ */
