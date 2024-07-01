/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2018-2024 Intel Corporation */

#include "ice.h"
#include "ice_lib.h"
#include "ice_irq.h"
#include "ice_cpi.h"

static const struct ptp_pin_desc ice_pin_desc_e82x[] = {
	/* name        idx func           chan */
	{ "TIME_SYNC", 0,  PTP_PF_EXTTS,  0, { 0, } },
	{ "1PPS",      1,  PTP_PF_PEROUT, 0, { 0, } },
};

static const char ice_pin_names_e810t[10][64] = {
	"GNSS",
	"SMA1",
	"U.FL1",
	"SMA2",
	"U.FL2",
};

static const struct ice_ptp_pin_desc ice_pin_desc_e810t_default[] = {
	{ GNSS, 0, PTP_PF_EXTTS, 0, -1,	 GPIO_21 },
	{ SMA1, 1, PTP_PF_NONE, 1, GPIO_20, GPIO_21 },
	{ UFL1, 2, PTP_PF_NONE, 1, GPIO_20, GPIO_21 },
	{ SMA2, 3, PTP_PF_NONE, 2, GPIO_22, GPIO_23 },
	{ UFL2, 4, PTP_PF_NONE, 2, GPIO_22, GPIO_23 },
};

#define MAX_DPLL_NAME_LEN 4
struct ice_dpll_desc {
	char name[MAX_DPLL_NAME_LEN];
	u8 index;
};

static const struct ice_dpll_desc ice_e810t_dplls[] = {
	/* name  idx */
	{ "EEC", ICE_CGU_DPLL_SYNCE },
	{ "PPS", ICE_CGU_DPLL_PTP },
};

struct dpll_attribute {
	struct device_attribute attr;
	u8 dpll_num;
};

#define DPLL_ATTR(_dpll_num, _name)					\
	struct dpll_attribute dev_attr_##dpll_##_dpll_num##_##_name = {	\
		__ATTR(dpll_##_dpll_num##_##_name, 0444, dpll_##_name##_show, \
		       NULL),						\
		_dpll_num						\
	}

#define DPLL_ATTR_RW(_dpll_num, _name)					\
	struct dpll_attribute dev_attr_##dpll_##_dpll_num##_##_name = { \
		__ATTR(dpll_##_dpll_num##_##_name, 0644, dpll_##_name##_show, \
		       dpll_##_name##_store),				\
		_dpll_num						\
	}

static ssize_t synce_store(struct kobject *kobj,
			   struct kobj_attribute *attr,
			   const char *buf, size_t count);

static ssize_t tx_clk_store(struct kobject *kobj,
			    struct kobj_attribute *attr,
			    const char *buf, size_t count);
static ssize_t clock_1588_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t len);

static ssize_t pin_cfg_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t len);
static ssize_t dpll_ref_sw_show(struct device *dev,
				struct device_attribute *attr,
				char *buf);
static ssize_t dpll_ref_sw_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t len);
static ssize_t ts_pll_cfg_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t len);

static ssize_t pin_cfg_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf);

static ssize_t dpll_offset_show(struct device *dev,
				struct device_attribute *attr,
				char *buf);

static ssize_t dpll_name_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf);

static ssize_t dpll_state_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf);

static ssize_t dpll_ref_pin_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf);
static ssize_t ts_pll_cfg_show(struct device *dev,
			       struct device_attribute *attr, char *buf);

static ssize_t ptp_802_3cx_store(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 const char *buf, size_t count);
static ssize_t ptp_802_3cx_show(struct kobject *kobj,
				struct kobj_attribute *attr,
				char *buf);

static struct kobj_attribute synce_attribute = __ATTR_WO(synce);
static DEVICE_ATTR_RW(pin_cfg);
static struct kobj_attribute tx_clk_attribute = __ATTR_WO(tx_clk);
static DEVICE_ATTR_WO(clock_1588);
static DPLL_ATTR(0, name);
static DPLL_ATTR(0, state);
static DPLL_ATTR(0, ref_pin);
static DPLL_ATTR_RW(0, ref_sw);
static DPLL_ATTR(1, name);
static DPLL_ATTR(1, state);
static DPLL_ATTR(1, ref_pin);
static DPLL_ATTR(1, offset);
static DPLL_ATTR_RW(1, ref_sw);
static DEVICE_ATTR_RW(ts_pll_cfg);
static struct kobj_attribute ptp_802_3cx_attribute = __ATTR_RW(ptp_802_3cx);

#define DPLL_MAX_INPUT_PIN_PRIO	14
#define DPLL_DISABLE_INPUT_PIN_PRIO	0xFF
/**
 * ice_ptp_parse_and_apply_pin_prio - parse and apply pin prio from the buffer
 * @pf: pointer to a pf structure
 * @argc: number of arguments to parse
 * @argv: list of human readable configuration parameters
 *
 * Parse pin prio config from the split user buffer and apply it on given pin.
 * Return 0 on success, negative value otherwise
 */
static int
ice_ptp_parse_and_apply_pin_prio(struct ice_pf *pf, int argc, char **argv)
{
	u8 dpll = 0, pin = 0, prio = 0;
	int i, ret;

	for (i = 0; i < argc; i++) {
		if (!strncmp(argv[i], "prio", sizeof("prio")))
			ret = kstrtou8(argv[++i], 0, &prio);
		else if (!strncmp(argv[i], "dpll", sizeof("dpll")))
			ret = kstrtou8(argv[++i], 0, &dpll);
		else if (!strncmp(argv[i], "pin", sizeof("pin")))
			ret = kstrtou8(argv[++i], 0, &pin);
		else
			ret = -EINVAL;

		if (ret)
			return ret;
	}

	/* priority needs to be in range 0-14 */
	if (prio > DPLL_MAX_INPUT_PIN_PRIO &&
	    prio != DPLL_DISABLE_INPUT_PIN_PRIO)
		return -EINVAL;

	dev_info(ice_pf_to_dev(pf), "%s: dpll: %u, pin:%u, prio:%u\n",
		 __func__, dpll, pin, prio);
	return ice_aq_set_cgu_ref_prio(&pf->hw, dpll, pin, prio);
}

/**
 * ice_ptp_parse_and_apply_output_pin_cfg - parse and apply output pin config
 * @pf: pointer to a pf structure
 * @argc: number of arguments to parse
 * @argv: list of human readable configuration parameters
 *
 * Parse and apply given configuration items in a split user buffer for the
 * output pin.
 * Return 0 on success, negative value otherwise
 */
static int
ice_ptp_parse_and_apply_output_pin_cfg(struct ice_pf *pf, int argc, char **argv)
{
	u8 output_idx, flags = 0, old_flags, old_src_sel, src_sel = 0;
	u32 freq = 0, old_freq, old_src_freq;
	struct ice_hw *hw = &pf->hw;
	bool esync_en_valid = false;
	bool pin_en_valid = false;
	bool esync_en = false;
	bool pin_en = false;
	s32 phase_delay = 0;
	int i, ret;

	output_idx = ICE_PTP_PIN_INVALID;
	for (i = 0; i < argc; i++) {
		if (!strncmp(argv[i], "pin", sizeof("pin"))) {
			ret = kstrtou8(argv[++i], 0, &output_idx);
		} else if (!strncmp(argv[i], "freq", sizeof("freq"))) {
			ret = kstrtou32(argv[++i], 0, &freq);
			flags |= ICE_AQC_SET_CGU_OUT_CFG_UPDATE_FREQ;
		} else if (!strncmp(argv[i], "phase_delay",
				    sizeof("phase_delay"))) {
			ret = kstrtos32(argv[++i], 0, &phase_delay);
			flags |= ICE_AQC_SET_CGU_OUT_CFG_UPDATE_PHASE;
		} else if (!strncmp(argv[i], "esync", sizeof("esync"))) {
			ret = kstrtobool(argv[++i], &esync_en);
			esync_en_valid = true;
		} else if (!strncmp(argv[i], "enable", sizeof("enable"))) {
			ret = kstrtobool(argv[++i], &pin_en);
			pin_en_valid = true;
		} else if (!strncmp(argv[i], "source_sel",
				    sizeof("source_sel"))) {
			ret = kstrtou8(argv[++i], 0, &src_sel);
			flags |= ICE_AQC_SET_CGU_OUT_CFG_UPDATE_SRC_SEL;
		} else {
			ret = -EINVAL;
		}

		if (ret)
			return ret;
	}

	if (!esync_en_valid || !pin_en_valid) {
		ret = ice_aq_get_output_pin_cfg(hw, output_idx,
						&old_flags,
						&old_src_sel,
						&old_freq,
						&old_src_freq);
		if (ret) {
			dev_err(ice_pf_to_dev(pf),
				"Failed to read prev output pin cfg (%u:%s)",
				ret, ice_aq_str(hw->adminq.sq_last_status));
			return ret;
		}
	}

	if (!esync_en_valid)
		if (old_flags & ICE_AQC_GET_CGU_OUT_CFG_ESYNC_EN)
			flags |= ICE_AQC_SET_CGU_OUT_CFG_ESYNC_EN;
		else
			flags &= ~ICE_AQC_SET_CGU_OUT_CFG_ESYNC_EN;
	else
		if (esync_en)
			flags |= ICE_AQC_SET_CGU_OUT_CFG_ESYNC_EN;
		else
			flags &= ~ICE_AQC_SET_CGU_OUT_CFG_ESYNC_EN;

	if (!pin_en_valid)
		if (old_flags & ICE_AQC_SET_CGU_OUT_CFG_OUT_EN)
			flags |= ICE_AQC_SET_CGU_OUT_CFG_OUT_EN;
		else
			flags &= ~ICE_AQC_SET_CGU_OUT_CFG_OUT_EN;
	else
		if (pin_en)
			flags |= ICE_AQC_SET_CGU_OUT_CFG_OUT_EN;
		else
			flags &= ~ICE_AQC_SET_CGU_OUT_CFG_OUT_EN;

	dev_info(ice_pf_to_dev(pf),
		 "output pin:%u, enable: %u, freq:%u, phase_delay:%u, esync:%u, source_sel:%u, flags:%u\n",
		 output_idx, pin_en, freq, phase_delay, esync_en,
		 src_sel, flags);
	return ice_aq_set_output_pin_cfg(hw, output_idx, flags,
					 src_sel, freq, phase_delay);
}

/**
 * ice_ptp_parse_and_apply_input_pin_cfg - parse and apply input pin config
 * @pf: pointer to a pf structure
 * @argc: number of arguments to parse
 * @argv: list of human readable configuration parameters
 *
 * Parse and apply given list of configuration items for the input pin.
 * Return 0 on success, negative value otherwise
 */
static int
ice_ptp_parse_and_apply_input_pin_cfg(struct ice_pf *pf, int argc, char **argv)
{
	struct ice_aqc_get_cgu_input_config old_cfg = {0};
	u8 flags1 = 0, flags2 = 0, input_idx;
	struct ice_hw *hw = &pf->hw;
	bool esync_en_valid = false;
	bool pin_en_valid = false;
	u8 esync_refsync_en = 0;
	bool esync_en = false;
	bool pin_en = false;
	s32 phase_delay = 0;
	u32 freq = 0;
	int i, ret;

	input_idx = ICE_PTP_PIN_INVALID;
	for (i = 0; i < argc; i++) {
		if (!strncmp(argv[i], "pin", sizeof("pin"))) {
			ret = kstrtou8(argv[++i], 0, &input_idx);
		} else if (!strncmp(argv[i], "freq", sizeof("freq"))) {
			ret = kstrtou32(argv[++i], 0, &freq);
			flags1 |= ICE_AQC_SET_CGU_IN_CFG_FLG1_UPDATE_FREQ;
		} else if (!strncmp(argv[i], "phase_delay",
				    sizeof("phase_delay"))) {
			ret = kstrtos32(argv[++i], 0, &phase_delay);
			flags1 |= ICE_AQC_SET_CGU_IN_CFG_FLG1_UPDATE_DELAY;
		} else if (!strncmp(argv[i], "esync", sizeof("esync"))) {
			ret = kstrtobool(argv[++i], &esync_en);
			esync_refsync_en = esync_en ?
					   ICE_AQC_SET_CGU_IN_CFG_ESYNC_EN : 0;
			esync_en_valid = true;
			dev_warn_once(ice_pf_to_dev(pf), "The 'esync' setting has been deprecated, please use 'e_ref_sync.\n");
		} else if (!strncmp(argv[i], "e_ref_sync",
			   sizeof("e_ref_sync"))) {
			ret = kstrtou8(argv[++i], 0, &esync_refsync_en);
			esync_en_valid = true;
		} else if (!strncmp(argv[i], "enable", sizeof("enable"))) {
			ret = kstrtobool(argv[++i], &pin_en);
			pin_en_valid = true;
		} else {
			ret = -EINVAL;
		}

		if (ret)
			return ret;
	}

	/* esync/refsync valid values 0,1,2 */
	if (esync_refsync_en > ICE_AQC_GET_CGU_IN_CFG_REFSYNC_EN)
		return -EINVAL;

	/* refsync is not allowed on any pin */
	if (esync_refsync_en == ICE_AQC_GET_CGU_IN_CFG_REFSYNC_EN &&
	    !refsync_pin_id_valid(hw, input_idx)) {
		dev_warn(ice_pf_to_dev(pf), "Ref-sync is not allowed on pin %d. Use pin 1 or pin 5.\n",
			 input_idx);
		return -EINVAL;
	}

	if (!esync_en_valid || !pin_en_valid) {
		ret = ice_aq_get_input_pin_cfg(hw, input_idx, NULL, NULL,
					       &old_cfg.flags1, &old_cfg.flags2,
					       NULL, NULL);
		if (ret) {
			dev_err(ice_pf_to_dev(pf),
				"Failed to read prev intput pin cfg (%u:%s)",
				ret, ice_aq_str(hw->adminq.sq_last_status));
			return ret;
		}
	}

	if (flags1 == ICE_AQC_SET_CGU_IN_CFG_FLG1_UPDATE_FREQ &&
	    !(old_cfg.flags1 & ICE_AQC_GET_CGU_IN_CFG_FLG1_ANYFREQ)) {
		if (freq != ICE_PTP_PIN_FREQ_1HZ &&
		    freq != ICE_PTP_PIN_FREQ_10MHZ) {
			dev_err(ice_pf_to_dev(pf),
				"Only %i or %i freq supported\n",
				ICE_PTP_PIN_FREQ_1HZ,
				ICE_PTP_PIN_FREQ_10MHZ);
			return -EINVAL;
		}
	}

	if (!esync_en_valid) {
		flags2 &= ~ICE_AQC_SET_CGU_IN_CFG_FLG2_ESYNC_REFSYNC_EN;
		flags2 |= old_cfg.flags2 &
		       ICE_AQC_SET_CGU_IN_CFG_FLG2_ESYNC_REFSYNC_EN;
	} else {
		flags2 &= ~ICE_AQC_SET_CGU_IN_CFG_FLG2_ESYNC_REFSYNC_EN;
		flags2 |= esync_refsync_en <<
			  ICE_AQC_SET_CGU_IN_CFG_FLG2_ESYNC_REFSYNC_EN_SHIFT;
	}

	if (!pin_en_valid)
		if (old_cfg.flags2 & ICE_AQC_GET_CGU_IN_CFG_FLG2_INPUT_EN)
			flags2 |= ICE_AQC_SET_CGU_IN_CFG_FLG2_INPUT_EN;
		else
			flags2 &= ~ICE_AQC_SET_CGU_IN_CFG_FLG2_INPUT_EN;
	else
		if (pin_en)
			flags2 |= ICE_AQC_SET_CGU_IN_CFG_FLG2_INPUT_EN;
		else
			flags2 &= ~ICE_AQC_SET_CGU_IN_CFG_FLG2_INPUT_EN;

	dev_info(ice_pf_to_dev(pf),
		 "input pin:%u, enable: %u, freq:%u, phase_delay:%u, e_ref_sync:%u, flags1:%u, flags2:%u\n",
		  input_idx, pin_en, freq, phase_delay, esync_refsync_en,
		  flags1, flags2);
	return ice_aq_set_input_pin_cfg(&pf->hw, input_idx, flags1, flags2,
					freq, phase_delay);
}

/**
 * synce_store_e825c - setting PHY recovered clock pins in E825-C
 * @pf: pointer to pf structure
 * @ena:  true if enable, false in disable
 * @phy_pin:   pin to be enabled/disabled
 *
 * Return number of bytes written on success or negative value on failure.
 */
static int
synce_store_e825c(struct ice_pf *pf, unsigned int ena, unsigned int phy_pin)
{
	struct ice_ptp_port *ptp_port;
	enum ice_e825c_clk_synce pin;
	struct ice_hw *hw;
	int status;
	u8 divider;

	if (phy_pin >= ICE_E825C_CLK_SYNCE_NUM)
		return -EINVAL;

	ptp_port = &pf->ptp.port;
	hw = &pf->hw;
	pin = (enum ice_e825c_clk_synce)phy_pin;

	/* configure the mux to deliver proper signal to DPLL from the MUX */
	status = ice_cfg_cgu_bypass_mux_e825c(hw, ptp_port->port_num, pin,
					      false, ena);
	if (status)
		return status;

	/* we need to reconfigure the divider if feature is set */
	if (test_bit(ICE_FLAG_ITU_G8262_FILTER_USED, pf->flags)) {
		status = ice_cfg_synce_ethdiv_e825c(hw, &divider, pin);
		if (status)
			return status;

		dev_dbg(ice_hw_to_dev(&pf->hw),
			"SyncE clock divider set to %u\n", divider);
	}

	dev_info(ice_hw_to_dev(&pf->hw), "CLK_SYNCE0 recovered clock: pin %s\n",
		 !!ena ? "Enabled" : "Disabled");

	return 0;
}

/**
 * synce_store_common - setting PHY recovered clock pins in generic devices
 * @pf: pointer to pf structure
 * @ena:  true if enable, false in disable
 * @phy_pin:   pin to be enabled/disabled
 *
 * Return number of bytes written on success or negative value on failure.
 */
static int
synce_store_common(struct ice_pf *pf, unsigned int ena, unsigned int phy_pin)
{
	const char *pin_name;
	u32 freq = 0;
	u8 pin, phy;
	int status;

	if (phy_pin >= ICE_E810_RCLK_PINS_NUM)
		return -EINVAL;

	status = ice_aq_set_phy_rec_clk_out(&pf->hw, phy_pin, !!ena, &freq);
	if (status)
		return -EIO;

	if (ice_is_e810(&pf->hw)) {
		status = ice_get_pf_c827_idx(&pf->hw, &phy);
		if (status)
			return -EIO;

		pin = E810T_CGU_INPUT_C827(phy, phy_pin);
		pin_name = ice_cgu_get_pin_name(&pf->hw, pin, true);
	} else {
		/* e822-based devices for now have only one phy available
		 *  (from Rimmon) and only one DPLL RCLK input pin
		 */
		pin_name = E82X_CGU_RCLK_PIN_NAME;
	}

	dev_info(ice_hw_to_dev(&pf->hw), "%s recovered clock: pin %s\n",
		 !!ena ? "Enabled" : "Disabled", pin_name);

	return 0;
}

/**
 * synce_store - sysfs interface for setting PHY recovered clock pins
 * @kobj:  sysfs node
 * @attr:  sysfs node attributes
 * @buf:   string representing enable and pin number
 * @count: length of the 'buf' string
 *
 * Return number of bytes written on success or negative value on failure.
 */
static ssize_t
synce_store(struct kobject *kobj, struct kobj_attribute *attr,
	    const char *buf, size_t count)
{
	unsigned int ena, phy_pin;
	struct ice_pf *pf;
	int status, cnt;

	pf = ice_kobj_to_pf(kobj);
	if (!pf)
		return -EPERM;

	cnt = sscanf(buf, "%u %u", &ena, &phy_pin);
	if (cnt != 2)
		return -EINVAL;

	if (ice_is_e825c(&pf->hw))
		status = synce_store_e825c(pf, ena, phy_pin);
	else
		status = synce_store_common(pf, ena, phy_pin);
	if (status)
		return status;

	return count;
}

static struct ice_pf *
ice_ptp_aux_dev_to_owner_pf(struct auxiliary_device *aux_dev);
#define ICE_REFCLK_USER_TO_AQ_IDX(x) ((x) + 1)

/**
 * ice_ptp_change_tx_clk - Change Tx reference clock
 * @pf: pointer to pf structure
 * @clk: new Tx clock
 *
 * Return 0 on success, negative value otherwise.
 */
static int ice_ptp_change_tx_clk(struct ice_pf *pf, enum ice_e825c_ref_clk clk)
{
	struct ice_ptp_port_owner *ptp_owner;
	struct ice_port_info *port_info;
	enum ice_e825c_ref_clk old_clk;
	struct ice_pf *owner_pf;
	u8 port_num;
	int err;
	u8 phy;

	old_clk = pf->ptp.port.tx_clk;

	if (old_clk == clk)
		return 0;

	port_num = pf->ptp.port.port_num;
	phy = port_num / pf->hw.ptp.ports_per_phy;
	port_info = pf->hw.port_info;
	owner_pf = ice_ptp_aux_dev_to_owner_pf(&pf->ptp.port.aux_dev);
	ptp_owner = &owner_pf->ptp.ports_owner;

	/* Check if the TX clk is enabled for this PHY, if not - enable it */
	if (!ptp_owner->tx_refclks[phy][clk]) {
		err = ice_cpi_ena_dis_clk_ref(&pf->hw, port_num, clk, true);
		if (err) {
			dev_err(ice_hw_to_dev(&pf->hw), "Failed to enable the %u TX clock for the %u PHY\n",
				clk, phy);
			return err;
		}
	}

	clear_bit(port_num, &ptp_owner->tx_refclks[phy][old_clk]);
	set_bit(port_num, &ptp_owner->tx_refclks[phy][clk]);
	pf->ptp.port.tx_clk = clk;

	/* We are ready to switch to the new TX clk. */
	err = ice_aq_set_link_restart_an(port_info, true, NULL,
					 ICE_REFCLK_USER_TO_AQ_IDX(clk));
	if (err) {
		dev_err(ice_hw_to_dev(&pf->hw), "Failed to switch to %u TX clock for the %u PHY\n",
			clk, phy);
		clear_bit(port_num, &ptp_owner->tx_refclks[phy][clk]);
		set_bit(port_num, &ptp_owner->tx_refclks[phy][old_clk]);
		err = ice_cpi_ena_dis_clk_ref(&pf->hw, port_num, old_clk, true);
		if (err) {
			dev_err(ice_hw_to_dev(&pf->hw), "Failed to restore the %u TX clock for the %u PHY\n",
				old_clk, phy);
			return err;
		}
	}

	/* See if maybe old (or failed) TX can be disabled for powersaving */
	if (!ptp_owner->tx_refclks[phy][old_clk]) {
		err = ice_cpi_ena_dis_clk_ref(&pf->hw, port_num, old_clk,
					      false);
		if (err) {
			dev_warn(ice_hw_to_dev(&pf->hw), "Failed to disable the %u TX clock for the %u PHY\n",
				 old_clk, phy);
			return err;
		}
	}

	return 0;
}

/**
 * tx_clk_store - sysfs interface for changing TX clock for a given port
 * @kobj:  sysfs node
 * @attr:  sysfs node attributes
 * @buf:   string representing enable and pin number
 * @count: length of the 'buf' string
 *
 * Return number of bytes written on success or negative value on failure.
 */
static ssize_t
tx_clk_store(struct kobject *kobj, struct kobj_attribute *attr,
	     const char *buf, size_t count)
{
	enum ice_e825c_ref_clk new_clk;
	struct ice_pf *pf;
	unsigned int clk;
	int err;

	pf = ice_kobj_to_pf(kobj);
	if (!pf)
		return -EPERM;

	if (kstrtouint(buf, 0, &clk))
		return -EINVAL;

	if (clk >= ICE_REF_CLK_MAX)
		return -EINVAL;

	new_clk = (enum ice_e825c_ref_clk)clk;
	if (new_clk == pf->ptp.port.tx_clk) {
		dev_warn(ice_hw_to_dev(&pf->hw), "TX clock already set to %u\n",
			 clk);
		return count;
	}

	err = ice_ptp_change_tx_clk(pf, new_clk);
	if (err) {
		dev_err(ice_hw_to_dev(&pf->hw), "Failed setting TX clock to %u\n",
			clk);
		return -1;
	}

	return count;
}

/**
 * clock_1588_store - sysfs interface for setting 1588 clock as SyncE source
 * @dev:   device that owns the attribute
 * @attr:  sysfs device attribute
 * @buf:   string representing configuration
 * @len:   length of the 'buf' string
 *
 * Return number of bytes written on success or negative value on failure.
 */
static ssize_t clock_1588_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t len)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct ice_pf *pf;
	struct ice_hw *hw;
	int status, cnt;
	u32 ena, pin;

	pf = pci_get_drvdata(pdev);
	hw = &pf->hw;

	if (ice_is_reset_in_progress(pf->state))
		return -EAGAIN;

	cnt = sscanf(buf, "%u %u", &ena, &pin);
	if (cnt != 2)
		return -EINVAL;

	if (pin >= ICE_E825C_CLK_SYNCE_NUM)
		return -EINVAL;

	/* configure the mux to deliver proper signal to DPLL from the MUX */
	status = ice_cfg_cgu_bypass_mux_e825c(hw, 0,
					      (enum ice_e825c_clk_synce)pin,
					      true, ena);
	if (status)
		return status;

	dev_info(ice_hw_to_dev(&pf->hw), "CLK_SYNCE0 recovered clock: 1588 ref %s\n",
		 !!ena ? "Enabled" : "Disabled");

	return len;
}

/**
 * pin_cfg_store - sysfs interface callback for configuration of pins
 * @dev:   device that owns the attribute
 * @attr:  sysfs device attribute
 * @buf:   string representing configuration
 * @len:   length of the 'buf' string
 *
 * Allows set new configuration of a pin, given in a user buffer.
 * Return number of bytes written on success or negative value on failure.
 */
static ssize_t pin_cfg_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t len)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct ice_pf *pf;
	int argc, ret;
	char **argv;

	pf = pci_get_drvdata(pdev);
	if (ice_is_reset_in_progress(pf->state))
		return -EAGAIN;

	argv = argv_split(GFP_KERNEL, buf, &argc);
	if (!argv)
		return -ENOMEM;

	if (argc == ICE_PTP_PIN_PRIO_ARG_CNT) {
		ret = ice_ptp_parse_and_apply_pin_prio(pf, argc, argv);
	} else if (argc == ICE_PTP_PIN_CFG_1_ARG_CNT ||
		   argc == ICE_PTP_PIN_CFG_2_ARG_CNT ||
		   argc == ICE_PTP_PIN_CFG_3_ARG_CNT ||
		   argc == ICE_PTP_PIN_CFG_4_ARG_CNT) {
		if (!strncmp(argv[0], "in", sizeof("in"))) {
			ret = ice_ptp_parse_and_apply_input_pin_cfg(pf,
								    argc - 1,
								    argv + 1);
		} else if (!strncmp(argv[0], "out", sizeof("out"))) {
			ret = ice_ptp_parse_and_apply_output_pin_cfg(pf,
								     argc - 1,
								     argv + 1);
		} else {
			ret = -EINVAL;
			dev_dbg(ice_pf_to_dev(pf),
				"%s: wrong pin direction argument:%s\n",
				__func__, argv[0]);
		}
	} else {
		ret = -EINVAL;
		dev_dbg(ice_pf_to_dev(pf),
			"%s: wrong number of arguments:%d\n",
			__func__, argc);
	}

	if (!ret)
		ret = len;
	argv_free(argv);

	return ret;
}

/**
 * ice_ptp_load_output_pin_cfg - load formated output pin config into buffer
 * @pf: pointer to pf structure
 * @buf: user buffer to fill with returned data
 * @offset: added to buf pointer before first time writing to it
 * @pin_num: number of output pins to be printed
 *
 * Acquires configuration of output pins from FW and load it into
 * provided user buffer.
 * Returns total number of bytes written to the buffer.
 * Negative on failure.
 */
static int
ice_ptp_load_output_pin_cfg(struct ice_pf *pf, char *buf, ssize_t offset,
			    const u8 pin_num)
{
	u8 pin, pin_en, esync_en, dpll, flags;
	struct ice_hw *hw = &pf->hw;
	int count = offset;
	u32 freq, src_freq;

	count += scnprintf(buf + count, PAGE_SIZE, "%s\n", "out");
	count += scnprintf(buf + count, PAGE_SIZE,
			   "|%4s|%8s|%5s|%11s|%6s|\n",
			   "pin", "enabled", "dpll", "freq", "esync");
	for (pin = 0; pin < pin_num; ++pin) {
		int ret = ice_aq_get_output_pin_cfg(hw, pin, &flags,
						    &dpll, &freq, &src_freq);

		if (ret) {
			dev_err(ice_pf_to_dev(pf),
				"err:%d %s failed to read output pin cfg on pin:%u\n",
				ret, ice_aq_str(hw->adminq.sq_last_status),
				pin);
			return ret;
		}
		esync_en = !!(flags & ICE_AQC_GET_CGU_OUT_CFG_ESYNC_EN);
		pin_en = !!(flags & ICE_AQC_GET_CGU_OUT_CFG_OUT_EN);
		dpll &= ICE_AQC_GET_CGU_OUT_CFG_DPLL_SRC_SEL;
		count += scnprintf(buf + count, PAGE_SIZE,
				   "|%4u|%8u|%5u|%11u|%6u|\n",
				   pin, pin_en, dpll, freq, esync_en);
	}

	return count;
}

/**
 * ice_ptp_load_input_pin_cfg - load formated input pin config into buffer
 * @pf: pointer to pf structure
 * @buf: user buffer to fill with returned data
 * @offset: added to buf pointer before first time writing to it
 * @pin_num: number of input pins to be printed
 *
 * Acquires configuration of input pins from FW and load it into
 * provided user buffer.
 * Returns total number of bytes written to the buffer.
 * Negative on failure.
 */
static int
ice_ptp_load_input_pin_cfg(struct ice_pf *pf, char *buf,
			   ssize_t offset, const u8 pin_num)
{
	u8 pin, pin_en, esync_refsync_en, esync_fail, dpll0_prio, dpll1_prio;
	struct ice_hw *hw = &pf->hw;
	const char *pin_state;
	int count = offset;
	u8 status, flags;
	s32 phase_delay;
	u32 freq;

	count += scnprintf(buf + count, PAGE_SIZE, "%s\n", "in");
	count += scnprintf(buf + count, PAGE_SIZE,
			  "|%4s|%8s|%8s|%11s|%12s|%15s|%11s|%11s|\n",
			   "pin", "enabled", "state", "freq", "phase_delay",
			   "eSync/Ref-sync", "DPLL0 prio", "DPLL1 prio");
	for (pin = 0; pin < pin_num; ++pin) {
		int ret;

		ret = ice_aq_get_input_pin_cfg(hw, pin, &status, NULL,
					       NULL, &flags,
					       &freq, &phase_delay);
		if (ret) {
			dev_err(ice_pf_to_dev(pf),
				"err:%d %s failed to read input pin cfg on pin:%u\n",
				ret, ice_aq_str(hw->adminq.sq_last_status),
				pin);
			return ret;
		}

		ret = ice_aq_get_cgu_ref_prio(hw, ICE_CGU_DPLL_SYNCE,
					      pin, &dpll0_prio);
		if (ret) {
			dev_err(ice_pf_to_dev(pf),
				"err:%d %s failed to read DPLL0 pin prio on pin:%u\n",
				ret, ice_aq_str(hw->adminq.sq_last_status),
				pin);
			return ret;
		}

		ret = ice_aq_get_cgu_ref_prio(hw, ICE_CGU_DPLL_PTP,
					      pin, &dpll1_prio);
		if (ret) {
			dev_err(ice_pf_to_dev(pf),
				"err:%d %s failed to read DPLL1 pin prio on pin:%u\n",
				ret, ice_aq_str(hw->adminq.sq_last_status),
				pin);
			return ret;
		}

		esync_refsync_en = FIELD_GET(ICE_AQC_GET_CGU_IN_CFG_FLG2_ESYNC_REFSYNC_EN,
					     flags);
		esync_fail = !!(status &
				ICE_AQC_GET_CGU_IN_CFG_STATUS_ESYNC_FAIL);
		pin_en = !!(flags & ICE_AQC_GET_CGU_IN_CFG_FLG2_INPUT_EN);

		if (status & ICE_CGU_IN_PIN_FAIL_FLAGS)
			pin_state = ICE_DPLL_PIN_STATE_INVALID;
		else if (esync_refsync_en == ICE_AQC_GET_CGU_IN_CFG_ESYNC_EN &&
			 esync_fail)
			pin_state = ICE_DPLL_PIN_STATE_INVALID;
		else
			pin_state = ICE_DPLL_PIN_STATE_VALID;

		count += scnprintf(buf + count, PAGE_SIZE,
				   "|%4u|%8u|%8s|%11u|%12d|%15u|%11u|%11u|\n",
				   pin, pin_en, pin_state, freq,
				   phase_delay, esync_refsync_en, dpll0_prio,
				   dpll1_prio);
	}

	return count;
}

/**
 * ice_ptp_load_pin_cfg - load formated pin config into user buffer
 * @pf: pointer to pf structure
 * @buf: user buffer to fill with returned data
 * @offset: added to buf pointer before first time writing to it
 *
 * Acquires configuration from FW and load it into provided buffer.
 * Returns total number of bytes written to the buffer
 */
static ssize_t
ice_ptp_load_pin_cfg(struct ice_pf *pf, char *buf, ssize_t offset)
{
	struct ice_aqc_get_cgu_abilities abilities;
	struct ice_hw *hw = &pf->hw;
	int ret;

	ret = ice_aq_get_cgu_abilities(hw, &abilities);
	if (ret) {
		dev_err(ice_pf_to_dev(pf),
			"err:%d %s failed to read cgu abilities\n",
			ret, ice_aq_str(hw->adminq.sq_last_status));
		return ret;
	}

	ret = ice_ptp_load_input_pin_cfg(pf, buf, offset,
					 abilities.num_inputs);
	if (ret < 0)
		return ret;
	offset += ret;
	ret = ice_ptp_load_output_pin_cfg(pf, buf, offset,
					  abilities.num_outputs);
	if (ret < 0)
		return ret;
	ret += offset;

	return ret;
}

/**
 * pin_cfg_show - sysfs interface callback for reading pin_cfg file
 * @dev: pointer to dev structure
 * @attr: device attribute pointing sysfs file
 * @buf: user buffer to fill with returned data
 *
 * Collect data and feed the user buffed.
 * Returns total number of bytes written to the buffer
 */
static ssize_t pin_cfg_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct ice_pf *pf;

	pf = pci_get_drvdata(pdev);

	return ice_ptp_load_pin_cfg(pf, buf, 0);
}

/**
 * dpll_name_show - sysfs interface callback for reading dpll_name file
 * @dev: pointer to dev structure
 * @attr: device attribute pointing sysfs file
 * @buf: user buffer to fill with returned data
 *
 * Collect data and feed the user buffed.
 * Returns total number of bytes written to the buffer
 */
static ssize_t dpll_name_show(struct device __always_unused *dev,
			      struct device_attribute *attr, char *buf)
{
	struct dpll_attribute *dpll_attr;
	u8 dpll_num;

	dpll_attr = container_of(attr, struct dpll_attribute, attr);
	dpll_num = dpll_attr->dpll_num;

	if (dpll_num < ICE_CGU_DPLL_MAX)
		return snprintf(buf, PAGE_SIZE, "%s\n",
				ice_e810t_dplls[dpll_num].name);

	return -EINVAL;
}

/**
 * dpll_ref_sw_show - sysfs interface callback for reading dpll_ref_sw file
 * @dev: pointer to dev structure
 * @attr: device attribute pointing sysfs file
 * @buf: user buffer to fill with returned data
 *
 * Collect data and feed the user buffed.
 * Returns total number of bytes written to the buffer
 */

static ssize_t dpll_ref_sw_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u8 los, scm, cfm, gst, pfm, esync;
	struct dpll_attribute *dpll_attr;
	int bytes_left = PAGE_SIZE;
	struct pci_dev *pdev;
	struct ice_pf *pf;
	u8 dpll_num;
	int cnt = 0;
	int status;

	pdev = to_pci_dev(dev);
	pf = pci_get_drvdata(pdev);

	dpll_attr = container_of(attr, struct dpll_attribute, attr);
	dpll_num = dpll_attr->dpll_num;

	if (dpll_num >= ICE_CGU_DPLL_MAX)
		return -EINVAL;

	status = ice_get_dpll_ref_sw_status(&pf->hw, dpll_num,
					    &los, &scm, &cfm,
					    &gst, &pfm, &esync);
	if (status)
		return -EINVAL;
	cnt = snprintf(&buf[cnt], bytes_left - cnt, "los %d\n", los);
	cnt += snprintf(&buf[cnt], bytes_left - cnt, "scm %d\n", scm);
	cnt += snprintf(&buf[cnt], bytes_left - cnt, "cfm %d\n", cfm);
	cnt += snprintf(&buf[cnt], bytes_left - cnt, "gst %d\n", gst);
	cnt += snprintf(&buf[cnt], bytes_left - cnt, "pfm %d\n", pfm);
	cnt += snprintf(&buf[cnt], bytes_left - cnt, "esync %d\n", esync);
	return cnt;
}

/**
 * dpll_ref_sw_store - sysfs interface callback for configuration of dpll_ref_sw
 * @dev:   device that owns the attribute
 * @attr:  sysfs device attribute
 * @buf:   string representing configuration
 * @len:   length of the 'buf' string
 *
 * Return number of bytes written on success or negative value on failure.
 */
static ssize_t dpll_ref_sw_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t len)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct dpll_attribute *dpll_attr;
	u32 ret, enable_state;
	u8 dpll_num, monitor;
	struct ice_pf *pf;
	char **argv;
	int argc;

	dpll_attr = container_of(attr, struct dpll_attribute, attr);
	dpll_num = dpll_attr->dpll_num;

	if (dpll_num >= ICE_CGU_DPLL_MAX)
		return -EINVAL;
	pf = pci_get_drvdata(pdev);
	if (!ice_pf_state_is_nominal(pf))
		return -EAGAIN;
	if (pf->ptp.state != ICE_PTP_READY)
		return -EFAULT;
	argv = argv_split(GFP_KERNEL, buf, &argc);
	if (!argv)
		return -ENOMEM;
	if (argc != 2)
		return -EIO;
	ret = kstrtou32(argv[1], 0, &enable_state);
	if (ret)
		return -EIO;
	if (enable_state != ICE_DPLL_REF_SW_DISABLE &&
	    enable_state != ICE_DPLL_REF_SW_ENABLE)
		return -EIO;

	if (!strncmp(argv[0], "los", sizeof("los")))
		monitor = ICE_AQC_SET_CGU_DPLL_CONFIG_REF_SW_LOS;
	else if (!strncmp(argv[0], "scm", sizeof("scm")))
		monitor = ICE_AQC_SET_CGU_DPLL_CONFIG_REF_SW_SCM;
	else if (!strncmp(argv[0], "cfm", sizeof("cfm")))
		monitor = ICE_AQC_SET_CGU_DPLL_CONFIG_REF_SW_CFM;
	else if (!strncmp(argv[0], "gst", sizeof("gst")))
		monitor = ICE_AQC_SET_CGU_DPLL_CONFIG_REF_SW_GST;
	else if (!strncmp(argv[0], "pfm", sizeof("pfm")))
		monitor = ICE_AQC_SET_CGU_DPLL_CONFIG_REF_SW_PFM;
	else if (!strncmp(argv[0], "esync", sizeof("esync")))
		monitor = ICE_AQC_SET_CGU_DPLL_CONFIG_REF_SW_ESYNC;
	else
		return -EIO;
	ice_set_dpll_ref_sw_status(&pf->hw, dpll_num,
				   monitor, enable_state);
	return len;
}

/**
 * dpll_state_show - sysfs interface callback for reading dpll_state file
 * @dev: pointer to dev structure
 * @attr: device attribute pointing sysfs file
 * @buf: user buffer to fill with returned data
 *
 * Collect data and feed the user buffed.
 * Returns number of bytes written to the buffer or negative value on error
 */
static ssize_t dpll_state_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	enum dpll_lock_status *dpll_state;
	struct dpll_attribute *dpll_attr;
	struct pci_dev *pdev;
	struct ice_pf *pf;
	ssize_t cnt;

	pdev = to_pci_dev(dev);
	pf = pci_get_drvdata(pdev);
	dpll_attr = container_of(attr, struct dpll_attribute, attr);

	switch (dpll_attr->dpll_num) {
	case ICE_CGU_DPLL_SYNCE:
		dpll_state = &pf->synce_dpll_state;
		break;
	case ICE_CGU_DPLL_PTP:
		dpll_state = &pf->ptp_dpll_state;
		break;
	default:
		return -EINVAL;
	}

	cnt = snprintf(buf, PAGE_SIZE, "%d\n", *dpll_state);

	return cnt;
}

/**
 * dpll_ref_pin_show - sysfs callback for reading dpll_ref_pin file
 *
 * @dev: pointer to dev structure
 * @attr: device attribute pointing sysfs file
 * @buf: user buffer to fill with returned data
 *
 * Collect data and feed the user buffed.
 * Returns number of bytes written to the buffer or negative value on error
 */
static ssize_t dpll_ref_pin_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	enum dpll_lock_status *dpll_state;
	struct dpll_attribute *dpll_attr;
	struct pci_dev *pdev;
	struct ice_pf *pf;
	ssize_t cnt;
	u8 pin;

	pdev = to_pci_dev(dev);
	pf = pci_get_drvdata(pdev);
	dpll_attr = container_of(attr, struct dpll_attribute, attr);

	switch (dpll_attr->dpll_num) {
	case ICE_CGU_DPLL_SYNCE:
		dpll_state = &pf->synce_dpll_state;
		pin = pf->synce_ref_pin;
		break;
	case ICE_CGU_DPLL_PTP:
		dpll_state = &pf->ptp_dpll_state;
		pin = pf->ptp_ref_pin;
		break;
	default:
		return -EINVAL;
	}

	switch (*dpll_state) {
	case DPLL_LOCK_STATUS_LOCKED:
	case DPLL_LOCK_STATUS_LOCKED_HO_ACQ:
	case DPLL_LOCK_STATUS_HOLDOVER:
		cnt = snprintf(buf, PAGE_SIZE, "%d\n", pin);
		break;
	default:
		return -EAGAIN;
	}

	return cnt;
}

/**
 * dpll_offset_show - sysfs interface callback for reading dpll_offset file
 * @dev: pointer to dev structure
 * @attr: device attribute pointing sysfs file
 * @buf: user buffer to fill with returned data
 *
 * Returns number of bytes written to the buffer or negative value on error
 */
static ssize_t
dpll_offset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pci_dev *pdev;
	struct ice_pf *pf;

	pdev = to_pci_dev(dev);
	pf = pci_get_drvdata(pdev);

	return snprintf(buf, PAGE_SIZE, "%lld\n", pf->ptp_dpll_phase_offset);
}

/**
 * list_initialized - checks if list is initialized
 * @head: list head potentially containing all the entries
 */
static bool list_initialized(struct list_head head)
{
	return head.next && head.prev;
}

/**
 * ice_ptp_set_e82x_ts_pll_params - set TS PLL params for all PFs
 * @pf: Board private structure
 * @time_ref_freq: TIME_REF frequency to use
 * @clk_src: source of the clock signal
 * @src_tmr_mode: source timer mode (nanoseconds or locked)
 */
static void ice_ptp_set_e82x_ts_pll_params(struct ice_pf *pf,
					   enum ice_time_ref_freq time_ref_freq,
					   enum ice_clk_src clk_src,
					   enum ice_src_tmr_mode src_tmr_mode)
{
	struct ice_ptp_port *port;

	mutex_lock(&pf->ptp.ports_owner.lock);
	if (!list_initialized(pf->ptp.ports_owner.ports)) {
		mutex_unlock(&pf->ptp.ports_owner.lock);
		return;
	}
	list_for_each_entry(port, &pf->ptp.ports_owner.ports, list_member) {
		struct ice_pf *target_pf = ptp_port_to_pf(port);

		target_pf->hw.ptp.time_ref_freq = time_ref_freq;
		target_pf->hw.ptp.src_tmr_mode = src_tmr_mode;
		target_pf->hw.ptp.clk_src = clk_src;
	}
	mutex_unlock(&pf->ptp.ports_owner.lock);
}

/**
 * ts_pll_cfg_store - sysfs interface for setting TS PLL config
 * @dev:   device that owns the attribute
 * @attr:  sysfs device attribute
 * @buf:   string representing configuration
 * @len:   length of the 'buf' string
 *
 * Return number of bytes written on success or negative value on failure.
 */
static ssize_t
ts_pll_cfg_store(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t len)
{
	u32 time_ref_freq, clk_src, src_tmr_mode;
	struct pci_dev *pdev = to_pci_dev(dev);
	enum ice_time_ref_freq time_ref;
	enum ice_src_tmr_mode tmr_mode;
	enum ice_clk_src clk;
	struct ice_pf *pf;
	int argc, ret;
	char **argv;

	pf = pci_get_drvdata(pdev);
	if (!ice_pf_state_is_nominal(pf))
		return -EAGAIN;

	if (pf->ptp.state != ICE_PTP_READY)
		return -EFAULT;

	argv = argv_split(GFP_KERNEL, buf, &argc);
	if (!argv)
		return -ENOMEM;

	if (argc != 3)
		goto command_help;

	ret = kstrtou32(argv[0], 0, &time_ref_freq);
	if (ret)
		goto command_help;
	ret = kstrtou32(argv[1], 0, &clk_src);
	if (ret)
		goto command_help;
	ret = kstrtou32(argv[2], 0, &src_tmr_mode);
	if (ret)
		goto command_help;

	if (src_tmr_mode == ICE_SRC_TMR_MODE_LOCKED &&
	    clk_src != ICE_CLK_SRC_TIME_REF) {
		dev_info(ice_pf_to_dev(pf), "Locked mode available only with TIME_REF as source\n");
		return -EIO;
	}

	time_ref = (enum ice_time_ref_freq)time_ref_freq;
	clk = (enum ice_clk_src)clk_src;
	tmr_mode = (enum ice_src_tmr_mode)src_tmr_mode;

	if (ice_is_e825c(&pf->hw))
		ret = ice_cfg_cgu_pll_e825c(&pf->hw, &time_ref, &clk);
	else
		ret = ice_cfg_cgu_pll_e82x(&pf->hw, &time_ref, &clk);
	if (ret)
		return ret;
	ice_ptp_set_e82x_ts_pll_params(pf, time_ref, clk, tmr_mode);
	ret = ice_ptp_update_incval(pf, time_ref, tmr_mode);
	if (ret)
		return ret;

	return len;

command_help:
	dev_info(ice_pf_to_dev(pf), "Usage: <time_ref_freq> <clk_src> <src_tmr_mode>\ntime_ref_freq (MHz): 0 = 25, 1 = 122, 2 = 125, 3 = 153, 4 = 156.25, 5 = 245.76\nclk_src: 0 = TCXO, 1 = TIME_REF\nsrc_tmr_mode: 0 = NS MODE, 1 = LOCKED MODE\n");
	return -EIO;
}

/**
 * ice_src_tmr_mode_str - Convert src_tmr_mode to string
 * @src_tmr_mode: Source clock mode
 *
 * Convert the specified TIME_REF clock frequency to a string.
 */
static const char *ice_src_tmr_mode_str(enum ice_src_tmr_mode src_tmr_mode)
{
	switch (src_tmr_mode) {
	case ICE_SRC_TMR_MODE_NANOSECONDS:
		return "NS MODE";
	case ICE_SRC_TMR_MODE_LOCKED:
		return "LOCKED MODE";
	default:
		return "Unknown";
	}
}

#define TS_PLL_CFG_BUFF_SIZE	30
/**
 * ts_pll_cfg_show - sysfs callback for reading ts_pll_cfg file
 *
 * @dev: pointer to dev structure
 * @attr: device attribute pointing sysfs file
 * @buf: user buffer to fill with returned data
 *
 * Collect data and feed the user buffed.
 * Returns total number of bytes written to the buffer
 */
static ssize_t
ts_pll_cfg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pci_dev *pdev;
	struct ice_pf *pf;
	size_t cnt;

	pdev = to_pci_dev(dev);
	pf = pci_get_drvdata(pdev);

	if (!ice_pf_state_is_nominal(pf))
		return -EAGAIN;

	if (pf->ptp.state != ICE_PTP_READY)
		return -EFAULT;

	cnt = snprintf(buf, TS_PLL_CFG_BUFF_SIZE, "%s %s %s\n",
		       ice_clk_freq_str(pf->hw.ptp.time_ref_freq),
		       ice_clk_src_str(pf->hw.ptp.clk_src),
		       ice_src_tmr_mode_str(pf->hw.ptp.src_tmr_mode));
	return cnt;
}

/**
 * ice_phy_sysfs_init - initialize sysfs for DPLL
 * @pf: pointer to pf structure
 *
 * Initialize sysfs for handling DPLL in HW.
 */
static void ice_phy_sysfs_init(struct ice_pf *pf)
{
	struct kobject *phy_kobj;

	phy_kobj = kobject_create_and_add("phy", &pf->pdev->dev.kobj);
	if (!phy_kobj) {
		dev_warn(ice_pf_to_dev(pf), "Failed to create PHY kobject\n");
		return;
	}

	if (sysfs_create_file(phy_kobj, &synce_attribute.attr)) {
		dev_warn(ice_pf_to_dev(pf), "Failed to create synce sysfs file\n");
		kobject_put(phy_kobj);
		return;
	}

	if (ice_is_e825c(&pf->hw) &&
	    sysfs_create_file(phy_kobj, &tx_clk_attribute.attr)) {
		dev_warn(ice_pf_to_dev(pf), "Failed to create synce tx_clk file\n");
		kobject_put(phy_kobj);
		return;
	}

	pf->ptp.phy_kobj = phy_kobj;
}

/**
 * ice_pin_cfg_sysfs_init - initialize sysfs for pin_cfg
 * @dev: pointer to pf device structure
 *
 * Initialize sysfs for handling pin configuration in DPLL.
 */
static void ice_pin_cfg_sysfs_init(struct device *dev)
{
	if (device_create_file(dev, &dev_attr_pin_cfg))
		dev_warn(dev, "Failed to create pin_cfg sysfs file\n");
}

/**
 * ice_clock_1588_sysfs_init - initialize sysfs for 1588 SyncE clock source
 * @dev: pointer to pf device structure
 */
static void ice_clock_1588_sysfs_init(struct device *dev)
{
	if (device_create_file(dev, &dev_attr_clock_1588))
		dev_warn(dev, "Failed to create clock_1588 sysfs file\n");
}

/**
 * ice_dpll_attrs_init - initialize sysfs for DPLL attributes
 * @dev: pointer to pf device structure
 *
 * Helper function to allocate and initialize sysfs for DPLL attributes
 */
static void
ice_dpll_attrs_init(struct device *dev)
{
	if (device_create_file(dev, &dev_attr_dpll_0_name.attr))
		dev_warn(dev, "Failed to create dpll_0_name sysfs file\n");
	if (device_create_file(dev, &dev_attr_dpll_0_state.attr))
		dev_warn(dev, "Failed to create dpll_0_state sysfs file\n");
	if (device_create_file(dev, &dev_attr_dpll_0_ref_pin.attr))
		dev_warn(dev, "Failed to create dpll_0_ref_pin sysfs file\n");
	if (device_create_file(dev, &dev_attr_dpll_0_ref_sw.attr))
		dev_warn(dev, "Failed to create dpll_0_ref_sw sysfs file\n");
	if (device_create_file(dev, &dev_attr_dpll_1_name.attr))
		dev_warn(dev, "Failed to create dpll_1_name sysfs file\n");
	if (device_create_file(dev, &dev_attr_dpll_1_state.attr))
		dev_warn(dev, "Failed to create dpll_1_state sysfs file\n");
	if (device_create_file(dev, &dev_attr_dpll_1_ref_pin.attr))
		dev_warn(dev, "Failed to create dpll_1_ref_pin sysfs file\n");
	if (device_create_file(dev, &dev_attr_dpll_1_offset.attr))
		dev_warn(dev, "Failed to create dpll_1_offset sysfs file\n");
	if (device_create_file(dev, &dev_attr_dpll_1_ref_sw.attr))
		dev_warn(dev, "Failed to create dpll_1_ref_sw sysfs file\n");
}

/**
 * ice_ts_pll_sysfs_init - initialize sysfs for internal TS PLL
 * @dev: pointer to pf device structure
 *
 * Initialize sysfs for handling TS PLL in HW.
 */
static void ice_ts_pll_sysfs_init(struct device *dev)
{
	if (device_create_file(dev, &dev_attr_ts_pll_cfg))
		dev_dbg(dev, "Failed to create ts_pll_cfg kobject\n");
}

/**
 * ptp_802_3cx_store - sysfs callback for storing 802.3cx setting
 * @kobj: pointer to kobject structure
 * @attr: pointer to kobj_attribute structure
 * @buf: pointer to the buffer
 * @count: buffer size
 *
 * Parse the string in buf and write to the HW.
 */
static ssize_t ptp_802_3cx_store(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 const char *buf, size_t count)
{
	struct ice_eth56g_params *params;
	unsigned int sfd_ena;
	struct ice_pf *pf;
	int cnt;

	pf = ice_kobj_to_pf(kobj);
	if (!pf)
		return -EPERM;

	cnt = kstrtouint(buf, 10, &sfd_ena);
	if (!cnt)
		return -EINVAL;

	if (ice_ptp_config_sfd(&pf->hw, sfd_ena))
		return -EAGAIN;

	params = &pf->hw.ptp.phy.eth56g;

	params->sfd_ena = sfd_ena;

	return 0;
}

/**
 * ptp_802_3cx_show - sysfs callback for reading 802.3cx setting
 * @kobj: pointer to kobject structure
 * @attr: pointer to kobj_attribute structure
 * @buf: pointer to the buffer
 *
 *  Read the HW settings output to the buf.
 */
static ssize_t ptp_802_3cx_show(struct kobject *kobj,
				struct kobj_attribute *attr,
				char *buf)
{
	const struct ice_pf *pf = ice_kobj_to_pf(kobj);
	const struct ice_eth56g_params *params;

	if (!pf)
		return -EPERM;

	params = &pf->hw.ptp.phy.eth56g;

	return snprintf(buf, PAGE_SIZE, "%u", params->sfd_ena);
}

/**
 * ice_ptp_802_3cx_sysfs_init - initialize sysfs for 802.3cx
 * @pf: pointer to pf structure
 *
 * Initialize sysfs for handling 802.3cx settings
 */
static void ice_ptp_802_3cx_sysfs_init(struct ice_pf *pf)
{
	struct kobject *ptp_802_3cx_kobj;

	ptp_802_3cx_kobj = kobject_create_and_add("ptp_802_3cx_kobj",
						  &pf->pdev->dev.kobj);
	if (!ptp_802_3cx_kobj) {
		dev_warn(ice_pf_to_dev(pf), "Failed to create 802.3cx kobject\n");
		return;
	}

	if (sysfs_create_file(ptp_802_3cx_kobj,
			      &ptp_802_3cx_attribute.attr)) {
		dev_warn(ice_pf_to_dev(pf), "Failed to create sfd sysfs file\n");
		kobject_put(ptp_802_3cx_kobj);
		return;
	}

	pf->ptp.ptp_802_3cx_kobj = ptp_802_3cx_kobj;
}

/**
 * ice_ptp_sysfs_init - initialize sysfs for ptp and synce features
 * @pf: pointer to pf structure
 *
 * Initialize sysfs for handling configuration of ptp and synce features.
 */
static void ice_ptp_sysfs_init(struct ice_pf *pf)
{
	struct device *dev = ice_pf_to_dev(pf);

	if (ice_is_feature_supported(pf, ICE_F_PHY_RCLK))
		ice_phy_sysfs_init(pf);

	if (ice_pf_src_tmr_owned(pf) &&
	    ice_is_feature_supported(pf, ICE_F_CGU)) {
		ice_pin_cfg_sysfs_init(dev);
		ice_dpll_attrs_init(dev);
	}

	if (ice_is_e825c(&pf->hw) && ice_pf_src_tmr_owned(pf))
		ice_clock_1588_sysfs_init(dev);
	if (ice_pf_src_tmr_owned(pf) &&
	    (ice_is_e823(&pf->hw) || ice_is_e825c(&pf->hw)))
		ice_ts_pll_sysfs_init(dev);
	if (ice_is_e825c(&pf->hw))
		ice_ptp_802_3cx_sysfs_init(pf);
}

/**
 * ice_dpll_attrs_release - release sysfs for dpll_attribute
 * @dev: pointer to pf device structure
 */
static void
ice_dpll_attrs_release(struct device *dev)
{
	device_remove_file(dev, &dev_attr_dpll_0_name.attr);
	device_remove_file(dev, &dev_attr_dpll_0_state.attr);
	device_remove_file(dev, &dev_attr_dpll_0_ref_pin.attr);
	device_remove_file(dev, &dev_attr_dpll_0_ref_sw.attr);
	device_remove_file(dev, &dev_attr_dpll_1_name.attr);
	device_remove_file(dev, &dev_attr_dpll_1_state.attr);
	device_remove_file(dev, &dev_attr_dpll_1_ref_pin.attr);
	device_remove_file(dev, &dev_attr_dpll_1_offset.attr);
	device_remove_file(dev, &dev_attr_dpll_1_ref_sw.attr);
}

/**
 * ice_ptp_sysfs_release - release sysfs resources of ptp and synce features
 * @pf: pointer to pf structure
 *
 * Release sysfs interface resources for handling configuration of
 * ptp and synce features.
 */
static void ice_ptp_sysfs_release(struct ice_pf *pf)
{
	struct device *dev = ice_pf_to_dev(pf);

	if (pf->ptp.phy_kobj) {
		sysfs_remove_file(pf->ptp.phy_kobj, &synce_attribute.attr);
		if (ice_is_e825c(&pf->hw))
			sysfs_remove_file(pf->ptp.phy_kobj,
					  &tx_clk_attribute.attr);
		kobject_put(pf->ptp.phy_kobj);
		pf->ptp.phy_kobj = NULL;
	}

	if (ice_pf_src_tmr_owned(pf) &&
	    ice_is_feature_supported(pf, ICE_F_CGU)) {
		device_remove_file(dev, &dev_attr_pin_cfg);
		ice_dpll_attrs_release(dev);
	}

	if (ice_is_e825c(&pf->hw) && ice_pf_src_tmr_owned(pf))
		device_remove_file(dev, &dev_attr_clock_1588);
	if (ice_is_e823(&pf->hw) || ice_is_e825c(&pf->hw))
		device_remove_file(dev, &dev_attr_ts_pll_cfg);
	if (pf->ptp.ptp_802_3cx_kobj) {
		sysfs_remove_file(pf->ptp.ptp_802_3cx_kobj,
				  &ptp_802_3cx_attribute.attr);
		kobject_put(pf->ptp.ptp_802_3cx_kobj);
		pf->ptp.ptp_802_3cx_kobj = NULL;
	}
}

/**
 * ice_set_ptp_pin_func - set functionality of a pin
 * @ptp_pins: pointer to the ptp_pin_desc structure
 * @idx: index of the pin in ptp_pins that should be updated
 * @func: new functionality that should be applied to pin
 *
 * Update the func of specific pin in ptp_pin_desc structure
 * if pin does not exist, then do nothing
 */
static void ice_set_ptp_pin_func(struct ptp_pin_desc *ptp_pins, int idx,
				 enum ptp_pin_function func)
{
	if (idx < 0)
		return;
	ptp_pins[idx].func = func;
}

/**
 * ice_is_ptp_pin_func - checks functionality of a pin
 * @ptp_pins: pointer to the ptp_pin_desc structure
 * @idx: index of the pin in ptp_pins that should be checked
 * @func: functionality that should be checked
 *
 * Check the func of specific pin in ptp_pin_desc structure
 * Returns: true if func of pin equal to input or pin does not exist,
 * otherwise false
 */
static bool ice_is_ptp_pin_func(struct ptp_pin_desc *ptp_pins, int idx,
				enum ptp_pin_function func)
{
	if (idx < 0)
		return true;
	if (ptp_pins[idx].func == func)
		return true;

	return false;
}

/**
 * ice_get_pin_func - calculate new functionality of a pin
 * @dir: new direction should be added to pin functionality
 * @in_func: current pin functionality
 *
 * Calculate new functionality of a pin
 * Returns new functionality according to given direction.
 * If both input and output functionalities are valid, return PTP_PF_NONE
 */
static enum ptp_pin_function ice_get_pin_func(int dir, int in_func)
{
	enum ptp_pin_function curr_func = (enum ptp_pin_function)in_func;

	if (curr_func == PTP_PF_NONE)
		return dir == 0 ? PTP_PF_EXTTS : PTP_PF_PEROUT;
	else if (curr_func == PTP_PF_EXTTS)
		return dir == 0 ? PTP_PF_EXTTS : PTP_PF_NONE;
	else if (curr_func == PTP_PF_PEROUT)
		return dir == 0 ? PTP_PF_NONE : PTP_PF_PEROUT;

	return PTP_PF_NONE;
}

/**
 * ice_get_pin_idx - return the index of a pin in ice_pin_desc structure
 * @ptp_pins: pointer to the ptp_pin_desc structure
 * @idx: pin_name_index of pin
 * @pins_num: number of valid pins
 *
 * Gets the index of pin in ice_pin_desc structure
 * On initialization, allocate first free entry to given pin
 * Returns the index of the pin in ice_pin_desc
 * If pin does not exist, returns -1
 */
static int ice_get_pin_idx(struct ice_ptp_pin_desc *ptp_pins, int idx,
			   int pins_num)
{
	int i;

	for (i = 0; i < pins_num; i++) {
		if (ptp_pins[i].pin_name_idx == idx && idx != GPIO_NA)
			return i;
		if (ptp_pins[i].pin_name_idx == NO_PTP_PIN) {
			ptp_pins[i].pin_name_idx = idx;
			return i;
		}
	}

	return -1;
}

/**
 * ice_get_pin_config_idx - return the index of a pin on pin_config interface
 * @ptp: pointer to the PTP structure
 * @idx: pin_name_index of pin
 *
 * Gets the index of pin in existing pins on the system
 * Returns the index of the pin on pin_config interface.
 * If pin does not exist, returns -1
 */
static int ice_get_pin_config_idx(struct ice_ptp *ptp, int idx)
{
	int i = ice_get_pin_idx(ptp->ice_pin_desc, idx, ptp->ice_pin_desc_num);

	if (i < 0)
		return i;

	return ptp->ice_pin_desc[i].index;
}

/**
 * ice_get_pin_name_idx - return pin_name_idx on pin
 * @ptp: pointer to the PTP structure
 * @idx: enum value of pin
 *
 * Gets pin_name_idx of pin using its index on pin_config interface
 * Returns pin_name_idx of pin
 * If pin does not exist, returns -1
 */
static int ice_get_pin_name_idx(struct ice_ptp *ptp, int idx)
{
	int i, desc_num = ptp->ice_pin_desc_num;

	for (i = 0; i < desc_num; i++)
		if (ptp->ice_pin_desc[i].index == idx)
			return ptp->ice_pin_desc[i].pin_name_idx;

	return -1;
}

/**
 * ice_get_sma_config_e810t
 * @pf: pointer to pf structure
 * @ptp_pins: pointer to the ptp_pin_desc structure
 *
 * Read the configuration of the SMA control logic and put it into the
 * ptp_pin_desc structure
 */
static int
ice_get_sma_config_e810t(struct ice_pf *pf, struct ptp_pin_desc *ptp_pins)
{
	int status, sma1_idx, ufl1_idx, sma2_idx, ufl2_idx;
	struct ptp_clock_info *info = &pf->ptp.info;
	u8 data, i, j, pin_name_idx;

	/* Read initial pin state */
	status = ice_read_sma_ctrl_e810t(&pf->hw, &data);
	if (status)
		return status;

	/* initialize with defaults */
	j = 0;
	for (i = 0; i < info->n_pins; i++) {
		while (pf->ptp.ice_pin_desc[j].pin_name_idx == GPIO_NA) {
			j++;
			if (j >= pf->ptp.ice_pin_desc_num)
				return -1;
		}
		pin_name_idx = pf->ptp.ice_pin_desc[j].pin_name_idx;
		strscpy(ptp_pins[i].name, ice_pin_names_e810t[pin_name_idx],
			sizeof(ptp_pins[i].name));
		ptp_pins[i].index = i;
		pf->ptp.ice_pin_desc[j].index = i;
		ptp_pins[i].func = pf->ptp.ice_pin_desc[j].func;
		ptp_pins[i].chan = pf->ptp.ice_pin_desc[j].chan;
		j++;
	}

	sma1_idx = ice_get_pin_config_idx(&pf->ptp, SMA1);
	ufl1_idx = ice_get_pin_config_idx(&pf->ptp, UFL1);
	sma2_idx = ice_get_pin_config_idx(&pf->ptp, SMA2);
	ufl2_idx = ice_get_pin_config_idx(&pf->ptp, UFL2);

	/* Parse SMA1/UFL1 */
	switch (data & ICE_SMA1_MASK_E810T) {
	case ICE_SMA1_MASK_E810T:
	default:
		ice_set_ptp_pin_func(ptp_pins, sma1_idx, PTP_PF_NONE);
		ice_set_ptp_pin_func(ptp_pins, ufl1_idx, PTP_PF_NONE);
		break;
	case ICE_SMA1_DIR_EN_E810T:
		ice_set_ptp_pin_func(ptp_pins, sma1_idx, PTP_PF_PEROUT);
		ice_set_ptp_pin_func(ptp_pins, ufl1_idx, PTP_PF_NONE);
		break;
	case ICE_SMA1_TX_EN_E810T:
		ice_set_ptp_pin_func(ptp_pins, sma1_idx, PTP_PF_EXTTS);
		ice_set_ptp_pin_func(ptp_pins, ufl1_idx, PTP_PF_NONE);
		break;
	case 0:
		ice_set_ptp_pin_func(ptp_pins, sma1_idx, PTP_PF_EXTTS);
		ice_set_ptp_pin_func(ptp_pins, ufl1_idx, PTP_PF_PEROUT);
		break;
	}

	/* Parse SMA2/UFL2 */
	switch (data & ICE_SMA2_MASK_E810T) {
	case ICE_SMA2_MASK_E810T:
	default:
		ice_set_ptp_pin_func(ptp_pins, sma2_idx, PTP_PF_NONE);
		ice_set_ptp_pin_func(ptp_pins, ufl2_idx, PTP_PF_NONE);
		break;
	case (ICE_SMA2_TX_EN_E810T | ICE_SMA2_UFL2_RX_DIS_E810T):
		ice_set_ptp_pin_func(ptp_pins, sma2_idx, PTP_PF_EXTTS);
		ice_set_ptp_pin_func(ptp_pins, ufl2_idx, PTP_PF_NONE);
		break;
	case (ICE_SMA2_DIR_EN_E810T | ICE_SMA2_UFL2_RX_DIS_E810T):
		ice_set_ptp_pin_func(ptp_pins, sma2_idx, PTP_PF_PEROUT);
		ice_set_ptp_pin_func(ptp_pins, ufl2_idx, PTP_PF_NONE);
		break;
	case (ICE_SMA2_DIR_EN_E810T | ICE_SMA2_TX_EN_E810T):
		ice_set_ptp_pin_func(ptp_pins, sma2_idx, PTP_PF_NONE);
		ice_set_ptp_pin_func(ptp_pins, ufl2_idx, PTP_PF_EXTTS);
		break;
	case ICE_SMA2_DIR_EN_E810T:
		ice_set_ptp_pin_func(ptp_pins, sma2_idx, PTP_PF_PEROUT);
		ice_set_ptp_pin_func(ptp_pins, ufl2_idx, PTP_PF_EXTTS);
		break;
	}
	return 0;
}

/**
 * ice_ptp_set_sma_config_e810t
 * @hw: pointer to the hw struct
 * @ptp_pins: pointer to the ptp_pin_desc structure
 * @info: the driver's PTP info structure
 *
 * Set the configuration of the SMA control logic based on the configuration in
 * num_pins parameter
 */
static int
ice_ptp_set_sma_config_e810t(struct ice_hw *hw,
			     struct ptp_pin_desc *ptp_pins,
			     struct ptp_clock_info *info)
{
	int status, sma1_idx, ufl1_idx, sma2_idx, ufl2_idx;
	struct ice_pf *pf = ptp_info_to_pf(info);
	u8 data;

	sma1_idx = ice_get_pin_config_idx(&pf->ptp, SMA1);
	ufl1_idx = ice_get_pin_config_idx(&pf->ptp, UFL1);
	sma2_idx = ice_get_pin_config_idx(&pf->ptp, SMA2);
	ufl2_idx = ice_get_pin_config_idx(&pf->ptp, UFL2);

	/* SMA1 and UFL1 cannot be set to TX at the same time */
	if ((sma1_idx != -1) && (ufl1_idx != -1) &&
	    ice_is_ptp_pin_func(ptp_pins, sma1_idx, PTP_PF_PEROUT) &&
	    ice_is_ptp_pin_func(ptp_pins, ufl1_idx, PTP_PF_PEROUT))
		return -EINVAL;

	/* SMA2 and UFL2 cannot be set to RX at the same time */
	if ((sma2_idx != -1) && (ufl2_idx != -1) &&
	    ice_is_ptp_pin_func(ptp_pins, sma2_idx, PTP_PF_EXTTS) &&
	    ice_is_ptp_pin_func(ptp_pins, ufl2_idx, PTP_PF_EXTTS))
		return -EINVAL;

	/* Read initial pin state value */
	status = ice_read_sma_ctrl_e810t(hw, &data);
	if (status)
		return status;

	/* Set the right sate based on the desired configuration */
	data &= ~ICE_SMA1_MASK_E810T;
	if (ice_is_ptp_pin_func(ptp_pins, sma1_idx, PTP_PF_NONE) &&
	    ice_is_ptp_pin_func(ptp_pins, ufl1_idx, PTP_PF_NONE)) {
		dev_info(ice_hw_to_dev(hw), "SMA1 + U.FL1 disabled");
		data |= ICE_SMA1_MASK_E810T;
	} else if (ice_is_ptp_pin_func(ptp_pins, sma1_idx, PTP_PF_EXTTS) &&
		   ice_is_ptp_pin_func(ptp_pins, ufl1_idx, PTP_PF_NONE)) {
		dev_info(ice_hw_to_dev(hw), "SMA1 RX");
		data |= ICE_SMA1_TX_EN_E810T;
	} else if (ice_is_ptp_pin_func(ptp_pins, sma1_idx, PTP_PF_NONE) &&
		   ice_is_ptp_pin_func(ptp_pins, ufl1_idx, PTP_PF_PEROUT)) {
		/* U.FL 1 TX will always enable SMA 1 RX */
		dev_info(ice_hw_to_dev(hw), "SMA1 RX + U.FL1 TX");
	} else if (ice_is_ptp_pin_func(ptp_pins, sma1_idx, PTP_PF_EXTTS) &&
		   ice_is_ptp_pin_func(ptp_pins, ufl1_idx, PTP_PF_PEROUT)) {
		dev_info(ice_hw_to_dev(hw), "SMA1 RX + U.FL1 TX");
	} else if (ice_is_ptp_pin_func(ptp_pins, sma1_idx, PTP_PF_PEROUT) &&
		   ice_is_ptp_pin_func(ptp_pins, ufl1_idx, PTP_PF_NONE)) {
		dev_info(ice_hw_to_dev(hw), "SMA1 TX");
		data |= ICE_SMA1_DIR_EN_E810T;
	}

	data &= ~ICE_SMA2_MASK_E810T;
	if (ice_is_ptp_pin_func(ptp_pins, sma2_idx, PTP_PF_NONE) &&
	    ice_is_ptp_pin_func(ptp_pins, ufl2_idx, PTP_PF_NONE)) {
		dev_info(ice_hw_to_dev(hw), "SMA2 + U.FL2 disabled");
		data |= ICE_SMA2_MASK_E810T;
	} else if (ice_is_ptp_pin_func(ptp_pins, sma2_idx, PTP_PF_EXTTS) &&
		   ice_is_ptp_pin_func(ptp_pins, ufl2_idx, PTP_PF_NONE)) {
		dev_info(ice_hw_to_dev(hw), "SMA2 RX");
		data |= (ICE_SMA2_TX_EN_E810T |
			 ICE_SMA2_UFL2_RX_DIS_E810T);
	} else if (ice_is_ptp_pin_func(ptp_pins, sma2_idx, PTP_PF_NONE) &&
		   ice_is_ptp_pin_func(ptp_pins, ufl2_idx, PTP_PF_EXTTS)) {
		dev_info(ice_hw_to_dev(hw), "UFL2 RX");
		data |= (ICE_SMA2_DIR_EN_E810T | ICE_SMA2_TX_EN_E810T);
	} else if (ice_is_ptp_pin_func(ptp_pins, sma2_idx, PTP_PF_PEROUT) &&
		   ice_is_ptp_pin_func(ptp_pins, ufl2_idx, PTP_PF_NONE)) {
		dev_info(ice_hw_to_dev(hw), "SMA2 TX");
		data |= (ICE_SMA2_DIR_EN_E810T |
			 ICE_SMA2_UFL2_RX_DIS_E810T);
	} else if (ice_is_ptp_pin_func(ptp_pins, sma2_idx, PTP_PF_PEROUT) &&
		   ice_is_ptp_pin_func(ptp_pins, ufl2_idx, PTP_PF_EXTTS)) {
		dev_info(ice_hw_to_dev(hw), "SMA2 TX + U.FL2 RX");
		data |= ICE_SMA2_DIR_EN_E810T;
	}

	return ice_write_sma_ctrl_e810t(hw, data);
}

/**
 * ice_ptp_set_sma_e810t
 * @info: the driver's PTP info structure
 * @pin: pin index in kernel structure
 * @func: Pin function to be set (PTP_PF_NONE, PTP_PF_EXTTS or PTP_PF_PEROUT)
 *
 * Set the configuration of a single SMA pin
 */
static int
ice_ptp_set_sma_e810t(struct ptp_clock_info *info, unsigned int pin,
		      enum ptp_pin_function func)
{
	int sma1_idx, ufl1_idx, sma2_idx, ufl2_idx, curr_pin;
	struct ptp_pin_desc ptp_pins[NUM_PTP_PINS_E810T];
	struct ice_pf *pf = ptp_info_to_pf(info);
	struct ice_hw *hw = &pf->hw;
	int err;

	if (func > PTP_PF_PEROUT)
		return -EOPNOTSUPP;

	err = ice_get_sma_config_e810t(pf, ptp_pins);
	if (err)
		return err;

	sma1_idx = ice_get_pin_config_idx(&pf->ptp, SMA1);
	ufl1_idx = ice_get_pin_config_idx(&pf->ptp, UFL1);
	sma2_idx = ice_get_pin_config_idx(&pf->ptp, SMA2);
	ufl2_idx = ice_get_pin_config_idx(&pf->ptp, UFL2);
	curr_pin = ice_get_pin_name_idx(&pf->ptp, pin);

	/* Disable the same function on the other pin sharing the channel */
	if (curr_pin == SMA1 && ice_is_ptp_pin_func(ptp_pins, ufl1_idx, func))
		ice_set_ptp_pin_func(ptp_pins, ufl1_idx, PTP_PF_NONE);
	if (curr_pin == UFL1 && ice_is_ptp_pin_func(ptp_pins, sma1_idx, func))
		ice_set_ptp_pin_func(ptp_pins, sma1_idx, PTP_PF_NONE);

	if (curr_pin == SMA2 && ice_is_ptp_pin_func(ptp_pins, ufl2_idx, func))
		ice_set_ptp_pin_func(ptp_pins, ufl2_idx, PTP_PF_NONE);
	if (curr_pin == UFL2 && ice_is_ptp_pin_func(ptp_pins, sma2_idx, func))
		ice_set_ptp_pin_func(ptp_pins, sma2_idx, PTP_PF_NONE);

	/* Set up new pin function in the temp table */
	ptp_pins[pin].func = func;

	return ice_ptp_set_sma_config_e810t(hw, ptp_pins, info);
}
/**
 * ice_ptp_set_gnss_e810t - Set the configuration of a GNSS pin
 * @info: The driver's PTP info structure
 * @func: Assigned function
 */
static int
ice_ptp_set_gnss_e810t(struct ptp_clock_info *info, enum ptp_pin_function func)
{
	struct ice_pf *pf = ptp_info_to_pf(info);
	int gnss_idx;
	u8 flags2;

	gnss_idx = ice_get_pin_config_idx(&pf->ptp, GNSS);
	if (gnss_idx < 0)
		return -1;
	flags2 = func == PTP_PF_NONE ? 0 : ICE_AQC_SET_CGU_IN_CFG_FLG2_INPUT_EN;

	return ice_aq_set_input_pin_cfg(&pf->hw, gnss_idx, 0, flags2, 0, 0);
}

/**
 * ice_verify_pin_e810t
 * @info: the driver's PTP info structure
 * @pin: Pin index
 * @func: Assigned function
 * @chan: Assigned channel
 *
 * Verify if pin supports requested pin function. If the Check pins consistency.
 * Reconfigure the SMA logic attached to the given pin to enable its
 * desired functionality
 */
static int
ice_verify_pin_e810t(struct ptp_clock_info *info, unsigned int pin,
		     enum ptp_pin_function func, unsigned int chan)
{
	struct ice_pf *pf = ptp_info_to_pf(info);
	int pin_name_idx;

	/* Don't allow channel reassignment */
	if (chan != info->pin_config[pin].chan)
		return -EOPNOTSUPP;

	pin_name_idx = ice_get_pin_name_idx(&pf->ptp, pin);
	/* Check if functions are properly assigned */
	switch (func) {
	case PTP_PF_NONE:
		break;
	case PTP_PF_EXTTS:
		if (pin_name_idx == UFL1)
			return -EOPNOTSUPP;
		break;
	case PTP_PF_PEROUT:
		if (pin_name_idx == UFL2 || pin_name_idx == GNSS)
			return -EOPNOTSUPP;
		break;
	case PTP_PF_PHYSYNC:
		return -EOPNOTSUPP;
	}

	if (pin_name_idx == GNSS)
		return ice_ptp_set_gnss_e810t(info, func);
	else
		return ice_ptp_set_sma_e810t(info, pin, func);
}

/**
 * ice_ptp_is_managed_phy - Check if driver manaages PHY
 * @hw: pointer to HW struct
 */
static bool ice_ptp_is_managed_phy(struct ice_hw *hw)
{
	switch (hw->ptp.phy_model) {
	case ICE_PHY_E810:
		return false;
	default:
		return true;
	}
}

/**
 * ice_ptp_state_str - Convert PTP state to readable string
 * @state: PTP state to convert
 *
 * Returns: the human readable string representation of the provided PTP
 * state, used for printing error messages.
 */
static const char *ice_ptp_state_str(enum ice_ptp_state state)
{
	switch (state) {
	case ICE_PTP_UNINIT:
		return "UNINITIALIZED";
	case ICE_PTP_INITIALIZING:
		return "INITIALIZING";
	case ICE_PTP_READY:
		return "READY";
	case ICE_PTP_RESETTING:
		return "RESETTING";
	case ICE_PTP_ERROR:
		return "ERROR";
	}

	return "UNKNOWN";
}

/**
 * ice_is_ptp_supported - Check if PTP is supported by the device
 * @pf: Board private structure
 *
 * Returns: true if PTP is supported by the device and has not entered an
 * error state. False otherwise.
 */
bool ice_is_ptp_supported(struct ice_pf *pf)
{
	enum ice_ptp_state state = pf->ptp.state;

	/* PTP is not supported on this device */
	if (!test_bit(ICE_FLAG_PTP_SUPPORTED, pf->flags))
		return false;

	/* PTP has been initialized and is not in an error state */
	return state != ICE_PTP_UNINIT && state != ICE_PTP_ERROR;
}

/**
 * ice_ptp_ticks2ns - Converts system ticks to nanoseconds
 * @pf: Board private structure
 * @ticks: Ticks to be converted into ns
 *
 * This function converts PLL ticks into nanoseconds when the PHC works in
 * locked mode.
 */
static u64 ice_ptp_ticks2ns(struct ice_pf *pf, u64 ticks)
{
	if (pf->hw.ptp.src_tmr_mode == ICE_SRC_TMR_MODE_LOCKED) {
		u64 freq = ice_ptp_get_pll_freq(&pf->hw);

		/* Avoid division by zero */
		if (!freq)
			return 0;

		return mul_u64_u64_div_u64(ticks, 1000000000ULL, freq);
	}
	return ticks;
}

/**
 * ice_ptp_ns2ticks - Converts nanoseconds to system ticks
 * @pf: Board private structure
 * @ns: Nanoseconds to be converted into ticks
 *
 * This function converts nanoseconds into PLL ticks when PHC works in
 * locked mode.
 */
static u64 ice_ptp_ns2ticks(struct ice_pf *pf, u64 ns)
{
	if (pf->hw.ptp.src_tmr_mode == ICE_SRC_TMR_MODE_LOCKED) {
		u64 freq = ice_ptp_get_pll_freq(&pf->hw);

		return mul_u64_u64_div_u64(ns, freq, 1000000000ULL);
	}
	return ns;
}

/**
 * ice_ptp_cfg_tx_interrupt - Configure Tx timestamp interrupt for the device
 * @pf: Board private structure
 *
 * Program the device to respond appropriately to the Tx timestamp interrupt
 * cause.
 */
static void ice_ptp_cfg_tx_interrupt(struct ice_pf *pf)
{
	struct ice_hw *hw = &pf->hw;
	bool enable;
	u32 val;

	switch (pf->ptp.tx_interrupt_mode) {
	case ICE_PTP_TX_INTERRUPT_ALL:
		/* React to interrupts across all quads. */
		wr32(hw, PFINT_TSYN_MSK + (0x4 * hw->pf_id), TX_INTR_QUAD_MASK);
		enable = true;
		break;
	case ICE_PTP_TX_INTERRUPT_NONE:
		/* Do not react to interrupts on any quad. */
		wr32(hw, PFINT_TSYN_MSK + (0x4 * hw->pf_id), (u32)0x0);
		enable = false;
		break;
	case ICE_PTP_TX_INTERRUPT_SELF:
	default:
		enable = pf->ptp.tstamp_config.tx_type == HWTSTAMP_TX_ON;
		break;
	}

	/* Configure the Tx timestamp interrupt */
	val = rd32(hw, PFINT_OICR_ENA);
	if (enable)
		val |= PFINT_OICR_TSYN_TX_M;
	else
		val &= ~PFINT_OICR_TSYN_TX_M;
	wr32(hw, PFINT_OICR_ENA, val);
}

/**
 * ice_set_rx_tstamp - Enable or disable Rx timestamping
 * @pf: The PF pointer to search in
 * @on: bool value for whether timestamps are enabled or disabled
 */
static void ice_set_rx_tstamp(struct ice_pf *pf, bool on)
{
	struct ice_vsi *vsi;
	u16 i;

	vsi = ice_get_main_vsi(pf);
	if (!vsi || !vsi->rx_rings)
		return;

	/* Set the timestamp flag for all the Rx rings */
	ice_for_each_rxq(vsi, i) {
		if (!vsi->rx_rings[i])
			continue;
		vsi->rx_rings[i]->ptp_rx = on;
	}
}

/**
 * ice_ptp_disable_timestamp_mode - Disable current timestamp mode
 * @pf: Board private structure
 *
 * Called during preparation for reset to temporarily disable timestamping on
 * the device. Called during remove to disable timestamping while cleaning up
 * driver resources.
 */
static void ice_ptp_disable_timestamp_mode(struct ice_pf *pf)
{
	struct ice_hw *hw = &pf->hw;
	u32 val;

	val = rd32(hw, PFINT_OICR_ENA);
	val &= ~PFINT_OICR_TSYN_TX_M;
	wr32(hw, PFINT_OICR_ENA, val);

	ice_set_rx_tstamp(pf, false);
}

/**
 * ice_ptp_restore_timestamp_mode - Restore timestamp configuration
 * @pf: Board private structure
 *
 * Called at the end of rebuild to restore timestamp configuration after
 * a device reset.
 */
void ice_ptp_restore_timestamp_mode(struct ice_pf *pf)
{
	struct ice_hw *hw = &pf->hw;
	bool enable_rx;

	ice_ptp_cfg_tx_interrupt(pf);

	enable_rx = pf->ptp.tstamp_config.rx_filter == HWTSTAMP_FILTER_ALL;
	ice_set_rx_tstamp(pf, enable_rx);

	/* Trigger an immediate software interrupt to ensure that timestamps
	 * which occurred during reset are handled now.
	 */
	wr32(hw, PFINT_OICR, PFINT_OICR_TSYN_TX_M);
	ice_flush(hw);
}

/**
 * ice_ptp_read_src_clk_reg - Read the source clock register
 * @pf: Board private structure
 * @sts: Optional parameter for holding a pair of system timestamps from
 *       the system clock. Will be ignored if NULL is given.
 */
u64
ice_ptp_read_src_clk_reg(struct ice_pf *pf, struct ptp_system_timestamp *sts)
{
	struct ice_hw *hw = &pf->hw;
	u32 hi, lo, lo2;
	u8 tmr_idx;

	tmr_idx = ice_get_ptp_src_clock_index(hw);
	/* Read the system timestamp pre PHC read */
	ptp_read_system_prets(sts);

	lo = rd32(hw, GLTSYN_TIME_L(tmr_idx));

	/* Read the system timestamp post PHC read */
	ptp_read_system_postts(sts);

	hi = rd32(hw, GLTSYN_TIME_H(tmr_idx));
	lo2 = rd32(hw, GLTSYN_TIME_L(tmr_idx));

	if (lo2 < lo) {
		/* if TIME_L rolled over read TIME_L again and update
		 * system timestamps
		 */
		ptp_read_system_prets(sts);
		lo = rd32(hw, GLTSYN_TIME_L(tmr_idx));
		ptp_read_system_postts(sts);
		hi = rd32(hw, GLTSYN_TIME_H(tmr_idx));
	}

	return ((u64)hi << 32) | lo;
}

/**
 * ice_ptp_extend_32b_ts - Convert a 32b nanoseconds timestamp to 64b
 * @cached_phc_time: recently cached copy of PHC time
 * @in_tstamp: Ingress/egress 32b nanoseconds timestamp value
 *
 * Hardware captures timestamps which contain only 32 bits of nominal
 * nanoseconds, as opposed to the 64bit timestamps that the stack expects.
 * Note that the captured timestamp values may be 40 bits, but the lower
 * 8 bits are sub-nanoseconds and generally discarded.
 *
 * Extend the 32bit nanosecond timestamp using the following algorithm and
 * assumptions:
 *
 * 1) have a recently cached copy of the PHC time
 * 2) assume that the in_tstamp was captured 2^31 nanoseconds (~2.1
 *    seconds) before or after the PHC time was captured.
 * 3) calculate the delta between the cached time and the timestamp
 * 4) if the delta is smaller than 2^31 nanoseconds, then the timestamp was
 *    captured after the PHC time. In this case, the full timestamp is just
 *    the cached PHC time plus the delta.
 * 5) otherwise, if the delta is larger than 2^31 nanoseconds, then the
 *    timestamp was captured *before* the PHC time, i.e. because the PHC
 *    cache was updated after the timestamp was captured by hardware. In this
 *    case, the full timestamp is the cached time minus the inverse delta.
 *
 * This algorithm works even if the PHC time was updated after a Tx timestamp
 * was requested, but before the Tx timestamp event was reported from
 * hardware.
 *
 * This calculation primarily relies on keeping the cached PHC time up to
 * date. If the timestamp was captured more than 2^31 nanoseconds after the
 * PHC time, it is possible that the lower 32bits of PHC time have
 * overflowed more than once, and we might generate an incorrect timestamp.
 *
 * This is prevented by (a) periodically updating the cached PHC time once
 * a second, and (b) discarding any Tx timestamp packet if it has waited for
 * a timestamp for more than one second.
 */
static u64 ice_ptp_extend_32b_ts(u64 cached_phc_time, u32 in_tstamp)
{
	u32 delta, phc_time_lo;
	u64 ns;

	/* Extract the lower 32 bits of the PHC time */
	phc_time_lo = (u32)cached_phc_time;

	/* Calculate the delta between the lower 32bits of the cached PHC
	 * time and the in_tstamp value
	 */
	delta = (in_tstamp - phc_time_lo);

	/* Do not assume that the in_tstamp is always more recent than the
	 * cached PHC time. If the delta is large, it indicates that the
	 * in_tstamp was taken in the past, and should be converted
	 * forward.
	 */
	if (delta > (U32_MAX / 2)) {
		/* reverse the delta calculation here */
		delta = (phc_time_lo - in_tstamp);
		ns = cached_phc_time - delta;
	} else {
		ns = cached_phc_time + delta;
	}

	return ns;
}

/**
 * ice_ptp_extend_40b_ts - Convert a 40b timestamp to 64b nanoseconds
 * @pf: Board private structure
 * @in_tstamp: Ingress/egress 40b timestamp value
 *
 * The Tx and Rx timestamps are 40 bits wide, including 32 bits of nominal
 * nanoseconds, 7 bits of sub-nanoseconds, and a valid bit.
 *
 *  *--------------------------------------------------------------*
 *  | 32 bits of nanoseconds | 7 high bits of sub ns underflow | v |
 *  *--------------------------------------------------------------*
 *
 * The low bit is an indicator of whether the timestamp is valid. The next
 * 7 bits are a capture of the upper 7 bits of the sub-nanosecond underflow,
 * and the remaining 32 bits are the lower 32 bits of the PHC timer.
 *
 * It is assumed that the caller verifies the timestamp is valid prior to
 * calling this function.
 *
 * Extract the 32bit nominal nanoseconds and extend them. Use the cached PHC
 * time stored in the device private PTP structure as the basis for timestamp
 * extension.
 *
 * See ice_ptp_extend_32b_ts for a detailed explanation of the extension
 * algorithm.
 */
static u64 ice_ptp_extend_40b_ts(struct ice_pf *pf, u64 in_tstamp)
{
	const u64 mask = GENMASK_ULL(31, 0);
	unsigned long discard_time;
	u64 ticks;

	/* Discard the hardware timestamp if the cached PHC time is too old */
	discard_time = pf->ptp.cached_phc_jiffies + msecs_to_jiffies(2000);
	if (time_is_before_jiffies(discard_time)) {
		pf->ptp.tx_hwtstamp_discarded++;
		return 0;
	}

	ticks = ice_ptp_extend_32b_ts(pf->ptp.cached_phc_time,
				      (in_tstamp >> 8) & mask);
	return ice_ptp_ticks2ns(pf, ticks);
}

/**
 * ice_ptp_is_tx_tracker_up - Check if Tx tracker is ready for new timestamps
 * @tx: the PTP Tx timestamp tracker to check
 *
 * Check that a given PTP Tx timestamp tracker is up, i.e. that it is ready
 * to accept new timestamp requests.
 *
 * Assumes the tx->lock spinlock is already held.
 */
static bool
ice_ptp_is_tx_tracker_up(struct ice_ptp_tx *tx)
{
	lockdep_assert_held(&tx->lock);

	return tx->init && !tx->calibrating;
}

/**
 * ice_ptp_req_tx_single_tstamp - Request Tx timestamp for a port from FW
 * @tx: the PTP Tx timestamp tracker
 * @idx: index of the timestamp to request
 */
void ice_ptp_req_tx_single_tstamp(struct ice_ptp_tx *tx, u8 idx)
{
	struct ice_ptp_port *ptp_port;
	struct sk_buff *skb;
	struct ice_pf *pf;

	if (!tx->init)
		return;

	ptp_port = container_of(tx, struct ice_ptp_port, tx);
	pf = ptp_port_to_pf(ptp_port);

	/* Drop packets which have waited for more than 2 seconds */
	if (time_is_before_jiffies(tx->tstamps[idx].start + 2 * HZ)) {
		/* Count the number of Tx timestamps that timed out */
		pf->ptp.tx_hwtstamp_timeouts++;

		skb = tx->tstamps[idx].skb;
		tx->tstamps[idx].skb = NULL;
		clear_bit(idx, tx->in_use);

		dev_kfree_skb_any(skb);
		return;
	}

	if (ice_trace_enabled(tx_tstamp_fw_req)) {
		ice_trace(tx_tstamp_fw_req, ice_pf_to_dev(pf), tx,
			  tx->tstamps[idx].skb, idx);
	}

	/* Write TS index to read to the PF register so the FW can read it */
	wr32(&pf->hw, PF_SB_ATQBAL,
	     TS_LL_READ_TS_INTR | FIELD_PREP(TS_LL_READ_TS_IDX, idx) |
	     TS_LL_READ_TS);
	tx->last_ll_ts_idx_read = idx;
}

/**
 * ice_ptp_complete_tx_single_tstamp - Complete Tx timestamp for a port
 * @tx: the PTP Tx timestamp tracker
 */
void ice_ptp_complete_tx_single_tstamp(struct ice_ptp_tx *tx)
{
	struct skb_shared_hwtstamps shhwtstamps = {};
	u8 idx = tx->last_ll_ts_idx_read;
	struct ice_ptp_port *ptp_port;
	u64 raw_tstamp, tstamp;
	struct sk_buff *skb;
	struct device *dev;
	struct ice_pf *pf;
	u32 val;

	if (!tx->init || tx->last_ll_ts_idx_read < 0)
		return;

	ptp_port = container_of(tx, struct ice_ptp_port, tx);
	pf = ptp_port_to_pf(ptp_port);
	dev = ice_pf_to_dev(pf);

	if (ice_trace_enabled(tx_tstamp_fw_done)) {
		ice_trace(tx_tstamp_fw_done, dev, tx, tx->tstamps[idx].skb,
			  idx);
	}

	val = rd32(&pf->hw, PF_SB_ATQBAL);

	/* When the bit is cleared, the TS is ready in the register */
	if (val & TS_LL_READ_TS) {
		dev_dbg(dev, "Failed to get the Tx tstamp - FW not ready. Will retry");
		return;
	}

	/* High 8 bit value of the TS is on the bits 16:23 */
	raw_tstamp = FIELD_GET(TS_LL_READ_TS_HIGH, val);
	raw_tstamp <<= 32;

	/* Read the low 32 bit value */
	raw_tstamp |= (u64)rd32(&pf->hw, PF_SB_ATQBAH);

	/* Devices using this interface always verify the timestamp differs
	 * relative to the last cached timestamp value.
	 */
	if (raw_tstamp == tx->tstamps[idx].cached_tstamp)
		return;

	tx->tstamps[idx].cached_tstamp = raw_tstamp;
	clear_bit(idx, tx->in_use);
	skb = tx->tstamps[idx].skb;
	tx->tstamps[idx].skb = NULL;

	if (!skb)
		return;

	/* Extend the timestamp using cached PHC time */
	tstamp = ice_ptp_extend_40b_ts(pf, raw_tstamp);
	if (tstamp) {
		shhwtstamps.hwtstamp = ns_to_ktime(tstamp);
		if (ice_trace_enabled(tx_tstamp_complete)) {
			ice_trace(tx_tstamp_complete, dev, tx, skb, idx);
		}
	}

	skb_tstamp_tx(skb, &shhwtstamps);
	dev_kfree_skb_any(skb);
}

/**
 * ice_ptp_clear_unexpected_tx_ready - Clear Tx ready bitmap after processing
 * @tx: the PTP Tx timestamp tracker
 * @tstamp_ready: the captured Tx timestamp ready bitmap
 *
 * Process the captured Tx timestamp ready bitmap and compare it with the Tx
 * timestamp tracker status. Detect when the hardware indicates we have
 * a valid Tx timestamp but the Tx tracker had no waiting skb. This is an
 * unexpected case but could potentially leave the device in a state where no
 * Tx timestamp interrupts will be generated.
 */
static void
ice_ptp_clear_unexpected_tx_ready(struct ice_ptp_tx *tx, u64 tstamp_ready)
{
	DECLARE_BITMAP(unexpected_tstamp, 64);
	struct ice_ptp_port *ptp_port;
	unsigned long flags;
	struct device *dev;
	struct ice_pf *pf;
	struct ice_hw *hw;
	u8 phy_idx, idx;

	bitmap_zero(unexpected_tstamp, 64);
	ptp_port = container_of(tx, struct ice_ptp_port, tx);
	pf = ptp_port_to_pf(ptp_port);
	dev = ice_pf_to_dev(pf);
	hw = &pf->hw;

	/* Find any bit which is *set* in the relevant section of tstamp_ready
	 * but *not set* in the Tx timestamp tracker. This likely means
	 * a driver bug resulted in the Tx timestamp index being used without
	 * the in_use bit being set properly in the tracker.
	 */
	spin_lock_irqsave(&tx->lock, flags);
	for_each_clear_bit(idx, tx->in_use, tx->len) {
		phy_idx = idx + tx->offset;

		if (tstamp_ready & BIT_ULL(phy_idx))
			set_bit(phy_idx, unexpected_tstamp);
	}
	spin_unlock_irqrestore(&tx->lock, flags);

	for_each_set_bit(phy_idx, unexpected_tstamp, 64) {
		u64 raw_tstamp;
		int err;

		dev_warn_ratelimited(dev, "clearing unexpected Tx timestamp ready indication on PHY index %d\n",
				     phy_idx);

		err = ice_read_phy_tstamp(hw, tx->block, phy_idx, &raw_tstamp);
		if (err) {
			dev_warn_ratelimited(dev, "Failed to clear Tx timestamp state for PHY index %d, err %pe",
					     phy_idx, ERR_PTR(err));
		}
	}
}

/**
 * ice_ptp_process_tx_tstamp - Process Tx timestamps for a port
 * @tx: the PTP Tx timestamp tracker
 *
 * Process timestamps captured by the PHY associated with this port. To do
 * this, loop over each index with a waiting skb.
 *
 * If a given index has a valid timestamp, perform the following steps:
 *
 * 1) check that the timestamp request is not stale
 * 2) check that a timestamp is ready and available in the PHY memory bank
 * 3) read and copy the timestamp out of the PHY register
 * 4) unlock the index by clearing the associated in_use bit
 * 5) extend the 40 bit timestamp value to get a 64 bit timestamp value
 * 6) send this 64 bit timestamp to the stack
 *
 * Note that we do not hold the tracking lock while reading the Tx timestamp.
 * This is because reading the timestamp requires taking a mutex that might
 * sleep.
 *
 * The only place where we set in_use is when a new timestamp is initiated
 * with a slot index. This is only called in the hard xmit routine where an
 * SKB has a request flag set. The only places where we clear this bit is this
 * function, or during teardown when the Tx timestamp tracker is being
 * removed. A timestamp index will never be re-used until the in_use bit for
 * that index is cleared.
 *
 * If a Tx thread starts a new timestamp, we might not begin processing it
 * right away but we will notice it at the end when we re-queue the task.
 *
 * If a Tx thread starts a new timestamp just after this function exits, the
 * interrupt for that timestamp should re-trigger this function once
 * a timestamp is ready.
 *
 * If a Tx packet has been waiting for more than 2 seconds, it is not possible
 * to correctly extend the timestamp using the cached PHC time. It is
 * extremely unlikely that a packet will ever take this long to timestamp. If
 * we detect a Tx timestamp request that has waited for this long we assume
 * the packet will never be sent by hardware and discard it without reading
 * the timestamp register.
 */
static void ice_ptp_process_tx_tstamp(struct ice_ptp_tx *tx)
{
	struct ice_ptp_port *ptp_port;
	struct ice_pf *pf;
	struct ice_hw *hw;
	u64 tstamp_ready;
	bool link_up;
	int err;
	u8 idx;

	ptp_port = container_of(tx, struct ice_ptp_port, tx);
	pf = ptp_port_to_pf(ptp_port);
	hw = &pf->hw;

	/* Read the Tx ready status first */
	if (tx->has_ready_bitmap) {
		err = ice_get_phy_tx_tstamp_ready(hw, tx->block, &tstamp_ready);
		if (err)
			return;

		/* Check and clear any Tx timestamp ready indication for
		 * a slot that is not currently in use.
		 */
		ice_ptp_clear_unexpected_tx_ready(tx, tstamp_ready);
	}

	/* Drop packets if the link went down */
	link_up = ptp_port->link_up;

	for_each_set_bit(idx, tx->in_use, tx->len) {
		struct skb_shared_hwtstamps shhwtstamps = {};
		u8 phy_idx = idx + tx->offset;
		u64 raw_tstamp = 0, tstamp;
		bool drop_ts = !link_up;
		unsigned long flags;
		struct sk_buff *skb;

		/* Drop packets which have waited for more than 2 seconds */
		if (time_is_before_jiffies(tx->tstamps[idx].start + 2 * HZ)) {
			drop_ts = true;

			if (ice_trace_enabled(tx_tstamp_timeout)) {
				spin_lock_irqsave(&tx->lock, flags);
				ice_trace(tx_tstamp_timeout, ice_pf_to_dev(pf),
					  tx, tx->tstamps[idx].skb, idx);
				spin_unlock_irqrestore(&tx->lock, flags);
			}

			/* Count the number of Tx timestamps that timed out */
			pf->ptp.tx_hwtstamp_timeouts++;
		}

		/* Only read a timestamp from the PHY if its marked as ready
		 * by the tstamp_ready register. This avoids unnecessary
		 * reading of timestamps which are not yet valid. This is
		 * important as we must read all timestamps which are valid
		 * and only timestamps which are valid during each interrupt.
		 * If we do not, the hardware logic for generating a new
		 * interrupt can get stuck on some devices.
		 */
		if (tx->has_ready_bitmap &&
		    !(tstamp_ready & BIT_ULL(phy_idx))) {
			if (drop_ts)
				goto skip_ts_read;

			continue;
		}

		if (ice_trace_enabled(tx_tstamp_fw_req)) {
			spin_lock_irqsave(&tx->lock, flags);
			ice_trace(tx_tstamp_fw_req, ice_pf_to_dev(pf), tx,
				  tx->tstamps[idx].skb, idx);
			spin_unlock_irqrestore(&tx->lock, flags);
		}

		err = ice_read_phy_tstamp(hw, tx->block, phy_idx, &raw_tstamp);
		if (err && !drop_ts)
			continue;

		if (ice_trace_enabled(tx_tstamp_fw_done)) {
			spin_lock_irqsave(&tx->lock, flags);
			ice_trace(tx_tstamp_fw_done, ice_pf_to_dev(pf), tx,
				  tx->tstamps[idx].skb, idx);
			spin_unlock_irqrestore(&tx->lock, flags);
		}

		/* For PHYs which don't implement a proper timestamp ready
		 * bitmap, verify that the timestamp value is different
		 * from the last cached timestamp. If it is not, skip this for
		 * now assuming it hasn't yet been captured by hardware.
		 */
		if (!drop_ts && !tx->has_ready_bitmap &&
		    raw_tstamp == tx->tstamps[idx].cached_tstamp)
			continue;

		/* Discard any timestamp value without the valid bit set */
		if (!(raw_tstamp & ICE_PTP_TS_VALID)) {
			if (ice_trace_enabled(tx_tstamp_invalid)) {
				spin_lock_irqsave(&tx->lock, flags);
				ice_trace(tx_tstamp_invalid, ice_pf_to_dev(pf),
					  tx, tx->tstamps[idx].skb, idx);
				spin_unlock_irqrestore(&tx->lock, flags);
			}
			drop_ts = true;
		}

skip_ts_read:
		spin_lock_irqsave(&tx->lock, flags);
		if (!tx->has_ready_bitmap && raw_tstamp)
			tx->tstamps[idx].cached_tstamp = raw_tstamp;
		clear_bit(idx, tx->in_use);
		skb = tx->tstamps[idx].skb;
		tx->tstamps[idx].skb = NULL;
		spin_unlock_irqrestore(&tx->lock, flags);

		if (!skb)
			continue;

		if (drop_ts) {
			if (ice_trace_enabled(tx_tstamp_dropped)) {
				spin_lock_irqsave(&tx->lock, flags);
				ice_trace(tx_tstamp_dropped, ice_pf_to_dev(pf),
					  tx, skb, idx);
				spin_unlock_irqrestore(&tx->lock, flags);
			}

			dev_kfree_skb_any(skb);
			continue;
		}

		/* Extend the timestamp using cached PHC time */
		tstamp = ice_ptp_extend_40b_ts(pf, raw_tstamp);
		if (tstamp) {
			shhwtstamps.hwtstamp = ns_to_ktime(tstamp);
			if (ice_trace_enabled(tx_tstamp_complete)) {
				spin_lock_irqsave(&tx->lock, flags);
				ice_trace(tx_tstamp_complete,
					  ice_pf_to_dev(pf), tx, skb, idx);
				spin_unlock_irqrestore(&tx->lock, flags);
			}
		}

		skb_tstamp_tx(skb, &shhwtstamps);
		dev_kfree_skb_any(skb);
	}
}

/**
 * ice_ptp_tx_tstamp - Process Tx timestamps for this function.
 * @tx: Tx tracking structure to initialize
 *
 * Returns: ICE_TX_TSTAMP_WORK_PENDING if there are any outstanding incomplete
 * Tx timestamps, or ICE_TX_TSTAMP_WORK_DONE otherwise.
 */
static enum ice_tx_tstamp_work ice_ptp_tx_tstamp(struct ice_ptp_tx *tx)
{
	bool more_timestamps;
	unsigned long flags;

	if (!tx->init)
		return ICE_TX_TSTAMP_WORK_DONE;

	/* Process the Tx timestamp tracker */
	ice_ptp_process_tx_tstamp(tx);

	/* Check if there are outstanding Tx timestamps */
	spin_lock_irqsave(&tx->lock, flags);
	more_timestamps = tx->init && !bitmap_empty(tx->in_use, tx->len);
	spin_unlock_irqrestore(&tx->lock, flags);

	if (more_timestamps)
		return ICE_TX_TSTAMP_WORK_PENDING;

	return ICE_TX_TSTAMP_WORK_DONE;
}

/**
 * ice_ptp_tx_tstamp_owner - Process Tx timestamps for all ports on the device
 * @pf: Board private structure
 *
 * Returns: false if any work remains, true if all work completed.
 */
static enum ice_tx_tstamp_work ice_ptp_tx_tstamp_owner(struct ice_pf *pf)
{
	struct ice_ptp_port *port;
	unsigned int i;

	mutex_lock(&pf->ptp.ports_owner.lock);
	if (!list_initialized(pf->ptp.ports_owner.ports)) {
		mutex_unlock(&pf->ptp.ports_owner.lock);
		return ICE_TX_TSTAMP_WORK_DONE;
	}
	list_for_each_entry(port, &pf->ptp.ports_owner.ports, list_member) {
		struct ice_ptp_tx *tx = &port->tx;

		if (!tx || !tx->init)
			continue;

		ice_ptp_process_tx_tstamp(tx);
	}
	mutex_unlock(&pf->ptp.ports_owner.lock);

	for (i = 0; i < ICE_GET_QUAD_NUM(pf->hw.ptp.num_lports); i++) {
		u64 tstamp_ready;
		int err;

		/* Check each port to determine if there is any new
		 * outstanding work that came in after we processed. If so,
		 * we must report that work is pending to immediately trigger
		 * another interrupt ensuring that we process this data.
		 */
		err = ice_get_phy_tx_tstamp_ready(&pf->hw, i, &tstamp_ready);
		if (err)
			break;
		else if (tstamp_ready)
			return ICE_TX_TSTAMP_WORK_PENDING;
	}

	return ICE_TX_TSTAMP_WORK_DONE;
}

/**
 * ice_ptp_alloc_tx_tracker - Initialize tracking for Tx timestamps
 * @tx: Tx tracking structure to initialize
 *
 * Assumes that the length has already been initialized. Do not call directly,
 * use the ice_ptp_init_tx_* instead.
 */
static int
ice_ptp_alloc_tx_tracker(struct ice_ptp_tx *tx)
{
	struct ice_tx_tstamp *tstamps;
	unsigned long *in_use;

	tstamps = kcalloc(tx->len, sizeof(*tstamps), GFP_KERNEL);
	in_use = bitmap_zalloc(tx->len, GFP_KERNEL);

	if (!tstamps || !in_use) {
		kfree(tstamps);
		bitmap_free(in_use);

		return -ENOMEM;
	}

	tx->tstamps = tstamps;
	tx->in_use = in_use;
	tx->init = 1;
	tx->calibrating = 0;
	tx->last_ll_ts_idx_read = -1;

	spin_lock_init(&tx->lock);

	return 0;
}

/**
 * ice_ptp_flush_tx_tracker - Flush any remaining timestamps from the tracker
 * @pf: Board private structure
 * @tx: the tracker to flush
 *
 * Called during teardown when a Tx tracker is being removed.
 */
static void
ice_ptp_flush_tx_tracker(struct ice_pf *pf, struct ice_ptp_tx *tx)
{
	struct ice_hw *hw = &pf->hw;
	u64 tstamp_ready;
	int err;
	u8 idx;

	err = ice_get_phy_tx_tstamp_ready(hw, tx->block, &tstamp_ready);
	if (err) {
		dev_dbg(ice_pf_to_dev(pf), "Failed to get the Tx tstamp ready bitmap for block %u, err %d\n",
			tx->block, err);

		/* If we fail to read the Tx timestamp ready bitmap just
		 * skip clearing the PHY timestamps.
		 */
		tstamp_ready = 0;
	}

	for_each_set_bit(idx, tx->in_use, tx->len) {
		u8 phy_idx = idx + tx->offset;
		unsigned long flags;
		struct sk_buff *skb;

		/* In case this timestamp is ready, we need to clear it. */
		if (!hw->reset_ongoing && (tstamp_ready & BIT_ULL(phy_idx)))
			ice_clear_phy_tstamp(hw, tx->block, phy_idx);

		spin_lock_irqsave(&tx->lock, flags);
		skb = tx->tstamps[idx].skb;
		tx->tstamps[idx].skb = NULL;
		clear_bit(idx, tx->in_use);
		spin_unlock_irqrestore(&tx->lock, flags);

		/* Count the number of Tx timestamps flushed */
		pf->ptp.tx_hwtstamp_flushed++;

		/* Free the SKB after we've cleared the bit */
		dev_kfree_skb_any(skb);
	}
}

/**
 * ice_ptp_flush_all_tx_tracker - Flush all timestamp trackers on this clock
 * @pf: Board private structure
 *
 * Called by the clock owner to flush all the Tx timestamp trackers associated
 * with the clock.
 */
static void
ice_ptp_flush_all_tx_tracker(struct ice_pf *pf)
{
	struct ice_ptp_port *port;

	mutex_lock(&pf->ptp.ports_owner.lock);
	if (!list_initialized(pf->ptp.ports_owner.ports)) {
		mutex_unlock(&pf->ptp.ports_owner.lock);
		return;
	}
	list_for_each_entry(port, &pf->ptp.ports_owner.ports, list_member)
		ice_ptp_flush_tx_tracker(ptp_port_to_pf(port), &port->tx);
	mutex_unlock(&pf->ptp.ports_owner.lock);
}

/**
 * ice_ptp_release_tx_tracker - Release allocated memory for Tx tracker
 * @pf: Board private structure
 * @tx: Tx tracking structure to release
 *
 * Free memory associated with the Tx timestamp tracker.
 */
static void
ice_ptp_release_tx_tracker(struct ice_pf *pf, struct ice_ptp_tx *tx)
{
	unsigned long flags;

	spin_lock_irqsave(&tx->lock, flags);
	tx->init = 0;
	spin_unlock_irqrestore(&tx->lock, flags);

	/* wait for potentially outstanding interrupt to complete */
	synchronize_irq(ice_get_irq_num(pf, pf->oicr_idx));

	ice_ptp_flush_tx_tracker(pf, tx);

	kfree(tx->tstamps);
	tx->tstamps = NULL;

	bitmap_free(tx->in_use);
	tx->in_use = NULL;

	tx->len = 0;
}

/**
 * ice_ptp_init_tx_eth56g - Initialize tracking for Tx timestamps
 * @pf: Board private structure
 * @tx: the Tx tracking structure to initialize
 * @port: the port this structure tracks
 *
 * Initialize the Tx timestamp tracker for this port. ETH56G PHYs
 * have independent memory blocks for all ports.
 */
static int
ice_ptp_init_tx_eth56g(struct ice_pf *pf, struct ice_ptp_tx *tx, u8 port)
{
	tx->block = port;
	tx->offset = 0;
	tx->len = INDEX_PER_PORT_ETH56G;
	tx->has_ready_bitmap = 1;

	return ice_ptp_alloc_tx_tracker(tx);
}
/**
 * ice_ptp_init_tx_e82x - Initialize tracking for Tx timestamps
 * @pf: Board private structure
 * @tx: the Tx tracking structure to initialize
 * @port: the port this structure tracks
 *
 * Initialize the Tx timestamp tracker for this port. For generic MAC devices,
 * the timestamp block is shared for all ports in the same quad. To avoid
 * ports using the same timestamp index, logically break the block of
 * registers into chunks based on the port number.
 */
static int
ice_ptp_init_tx_e82x(struct ice_pf *pf, struct ice_ptp_tx *tx, u8 port)
{
	tx->block = ICE_GET_QUAD_NUM(port);
	tx->offset = (port % ICE_PORTS_PER_QUAD) * INDEX_PER_PORT_E82X;
	tx->len = INDEX_PER_PORT_E82X;
	tx->has_ready_bitmap = 1;

	return ice_ptp_alloc_tx_tracker(tx);
}

/**
 * ice_ptp_init_tx_e810 - Initialize tracking for Tx timestamps
 * @pf: Board private structure
 * @tx: the Tx tracking structure to initialize
 *
 * Initialize the Tx timestamp tracker for this PF. For E810 devices, each
 * port has its own block of timestamps, independent of the other ports.
 */
static int
ice_ptp_init_tx_e810(struct ice_pf *pf, struct ice_ptp_tx *tx)
{
	tx->block = pf->hw.port_info->lport;
	tx->offset = 0;
	tx->len = INDEX_PER_PORT_E810;
	/* The E810 PHY does not provide a timestamp ready bitmap. Instead,
	 * verify new timestamps against cached copy of the last read
	 * timestamp.
	 */
	tx->has_ready_bitmap = 0;

	return ice_ptp_alloc_tx_tracker(tx);
}

/**
 * ice_ptp_schedule_periodic_work - Helper for scheduling PTP periodic work
 * @ptp: pointer to the PTP structure
 * @delay: delay before starting nex periodic work
 */
static void
ice_ptp_schedule_periodic_work(struct ice_ptp *ptp, unsigned long delay)
{
	struct ice_pf *pf = container_of(ptp, struct ice_pf, ptp);

	if (!ice_pf_src_tmr_owned(pf) ||
	    test_bit(ICE_FLAG_PTP_WT_BLOCKED, pf->flags))
		return;

	kthread_queue_delayed_work(ptp->ports_owner.kworker,
				   &ptp->ports_owner.work, delay);
}

/**
 * ice_ptp_cancel_periodic_work - Helper for cancelling PTP periodic work
 * @ptp: pointer to the PTP structure
 */
static void ice_ptp_cancel_periodic_work(struct ice_ptp *ptp)
{
	struct ice_pf *pf = container_of(ptp, struct ice_pf, ptp);

	if (!ice_pf_src_tmr_owned(pf))
		return;

	kthread_cancel_delayed_work_sync(&ptp->ports_owner.work);
}

/**
 * ice_ptp_update_cached_phctime - Update the cached PHC time values
 * @pf: Board specific private structure
 * @systime: Cached PHC time to write
 *
 * This function updates the system time values which are cached in the PF
 * structure and the Rx rings.
 *
 * This function must be called periodically to ensure that the cached value
 * is never more than 2 seconds old.
 *
 * Note that the cached copy in the PF PTP structure is always updated, even
 * if we can't update the copy in the Rx rings.
 *
 * Returns: 0 on success or -EAGAIN when PF was busy and the update needs to be
 * rescheduled
 */
static int ice_ptp_update_cached_phctime(struct ice_pf *pf, u64 systime)
{
	struct device *dev = ice_pf_to_dev(pf);
	unsigned long update_before;
	int i;

	update_before = pf->ptp.cached_phc_jiffies + msecs_to_jiffies(2000);
	if (pf->ptp.cached_phc_time &&
	    time_is_before_jiffies(update_before)) {
		unsigned long time_taken = jiffies - pf->ptp.cached_phc_jiffies;

		dev_warn_once(dev, "%u msecs passed between update to cached PHC time\n",
			      jiffies_to_msecs(time_taken));
		pf->ptp.late_cached_phc_updates++;
	}

	/* Update the cached PHC time stored in the PF structure */
	WRITE_ONCE(pf->ptp.cached_phc_time, systime);
	WRITE_ONCE(pf->ptp.cached_phc_jiffies, jiffies);

	/* Nothing to do if link is down */
	if (!pf->ptp.port.link_up)
		return 0;

	if (test_and_set_bit(ICE_CFG_BUSY, pf->state))
		return -EAGAIN;

	ice_for_each_vsi(pf, i) {
		struct ice_vsi *vsi = pf->vsi[i];
		int j;

		if (!vsi)
			continue;

		if (vsi->type != ICE_VSI_PF)
			continue;
		if (!vsi->rx_rings)
			continue;

		ice_for_each_rxq(vsi, j) {
			if (!vsi->rx_rings[j])
				continue;
			WRITE_ONCE(vsi->rx_rings[j]->cached_phctime, systime);
		}
	}

	clear_bit(ICE_CFG_BUSY, pf->state);

	return 0;
}

/**
 * ice_ptp_update_cached_phctime_all - Update the cached PHC time for all ports
 * @pf: Board specific private structure
 *
 * Returns: 0 on success or -EAGAIN when PF was busy and the update needs to be
 * rescheduled
 */
static int ice_ptp_update_cached_phctime_all(struct ice_pf *pf)
{
	struct ice_ptp_port *port;
	u64 systime;
	int err = 0;

	mutex_lock(&pf->ptp.ports_owner.lock);
	systime = ice_ptp_read_src_clk_reg(pf, NULL);
	if (!list_initialized(pf->ptp.ports_owner.ports)) {
		mutex_unlock(&pf->ptp.ports_owner.lock);
		return -EAGAIN;
	}
	list_for_each_entry(port, &pf->ptp.ports_owner.ports, list_member) {
		struct ice_pf *peer_pf = ptp_port_to_pf(port);

		err = ice_ptp_update_cached_phctime(peer_pf, systime);
		if (err)
			break;
	}
	mutex_unlock(&pf->ptp.ports_owner.lock);

	return err;
}

/**
 * ice_ptp_reset_cached_phctime - Reset cached PHC time after an update
 * @pf: Board specific private structure
 *
 * This function is called to immediately update the cached PHC time after
 * a .settime or .adjtime call.
 *
 * If updating the PHC time cannot be done immediately, a warning message is
 * logged and the work item is scheduled without delay to minimize the window
 * where a timestamp is extended using the old cached value.
 */
static void ice_ptp_reset_cached_phctime(struct ice_pf *pf)
{
	struct device *dev = ice_pf_to_dev(pf);
	int err;

	/* Update the cached PHC time immediately if possible, otherwise
	 * schedule the work item to execute soon.
	 */
	err = ice_ptp_update_cached_phctime_all(pf);
	if (err) {
		/* If another thread is updating the Rx rings, we won't
		 * properly reset them here. This could lead to reporting of
		 * invalid timestamps, but there isn't much we can do.
		 */
		dev_warn(dev, "%s: ICE_CFG_BUSY, unable to immediately update cached PHC time\n",
			 __func__);

		/* Queue the work item to update the Rx rings when possible */
		ice_ptp_schedule_periodic_work(&pf->ptp, 0);
	}
}

/**
 * ice_ptp_write_init - Set PHC time to provided value
 * @pf: Board private structure
 * @ts: timespec structure that holds the new time value
 *
 * Set the PHC time to the specified time provided in the timespec.
 */
static int ice_ptp_write_init(struct ice_pf *pf, struct timespec64 *ts)
{
	u64 ns = timespec64_to_ns(ts);
	struct ice_hw *hw = &pf->hw;
	u64 val;

	val = ice_ptp_ns2ticks(pf, ns);

	return ice_ptp_init_time(hw, val);
}

/**
 * ice_ptp_write_adj - Adjust PHC clock time atomically
 * @pf: Board private structure
 * @adj: Adjustment in nanoseconds
 *
 * Perform an atomic adjustment of the PHC time by the specified number of
 * nanoseconds.
 */
static int
ice_ptp_write_adj(struct ice_pf *pf, s32 adj)
{
	struct ice_hw *hw = &pf->hw;

	if (adj >= 0)
		adj = (s32)ice_ptp_ns2ticks(pf, adj);
	else
		adj = -((s32)ice_ptp_ns2ticks(pf, -adj));

	return ice_ptp_adj_clock(hw, adj);
}

/**
 * ice_base_incval - Get base timer increment value
 * @pf: Board private structure
 *
 * Look up the base timer increment value for this device. The base increment
 * value is used to define the nominal clock tick rate. This increment value
 * is programmed during device initialization. It is also used as the basis
 * for calculating adjustments using scaled_ppm.
 */
static u64 ice_base_incval(struct ice_pf *pf)
{
	struct ice_hw *hw = &pf->hw;
	u64 incval;

	incval = ice_get_base_incval(hw, hw->ptp.src_tmr_mode);

	dev_dbg(ice_pf_to_dev(pf), "PTP: using base increment value of 0x%016llx\n",
		incval);

	return incval;
}

/**
 * ice_ptp_check_tx_fifo - Check whether Tx FIFO is in an OK state
 * @port: PTP port for which Tx FIFO is checked
 */
static int ice_ptp_check_tx_fifo(struct ice_ptp_port *port)
{
	int quad = ICE_GET_QUAD_NUM(port->port_num);
	int offs = port->port_num % ICE_PORTS_PER_QUAD;
	struct ice_pf *pf;
	struct ice_hw *hw;
	u32 val, phy_sts;
	int err;

	pf = ptp_port_to_pf(port);
	hw = &pf->hw;

	if (port->tx_fifo_busy_cnt == FIFO_OK)
		return 0;

	/* need to read FIFO state */
	if (offs == 0 || offs == 1)
		err = ice_read_quad_reg_e82x(hw, quad, Q_REG_FIFO01_STATUS,
					     &val);
	else
		err = ice_read_quad_reg_e82x(hw, quad, Q_REG_FIFO23_STATUS,
					     &val);

	if (err) {
		dev_err(ice_pf_to_dev(pf), "PTP failed to check port %d Tx FIFO, err %d\n",
			port->port_num, err);
		return err;
	}

	if (offs & 0x1)
		phy_sts = FIELD_GET(Q_REG_FIFO13_M, val);
	else
		phy_sts = FIELD_GET(Q_REG_FIFO02_M, val);

	if (phy_sts & FIFO_EMPTY) {
		port->tx_fifo_busy_cnt = FIFO_OK;
		return 0;
	}

	port->tx_fifo_busy_cnt++;

	dev_dbg(ice_pf_to_dev(pf), "Try %d, port %d FIFO not empty\n",
		port->tx_fifo_busy_cnt, port->port_num);

	if (port->tx_fifo_busy_cnt == ICE_PTP_FIFO_NUM_CHECKS) {
		dev_warn(ice_pf_to_dev(pf),
			 "Port %d Tx FIFO still not empty; resetting quad %d\n",
			 port->port_num, quad);
		ice_ptp_reset_ts_memory_quad_e82x(hw, quad);
		port->tx_fifo_busy_cnt = FIFO_OK;
		return 0;
	}

	return -EAGAIN;
}

/**
 * ice_ptp_wait_for_offsets - Check for valid Tx and Rx offsets
 * @work: Pointer to the kthread_work structure for this task
 *
 * Check whether hardware has completed measuring the Tx and Rx offset values
 * used to configure and enable vernier timestamp calibration.
 *
 * Once the offset in either direction is measured, configure the associated
 * registers with the calibrated offset values and enable timestamping. The Tx
 * and Rx directions are configured independently as soon as their associated
 * offsets are known.
 *
 * This function reschedules itself until both Tx and Rx calibration have
 * completed.
 */
static void ice_ptp_wait_for_offsets(struct kthread_work *work)
{
	struct ice_ptp_port *port;
	struct ice_pf *pf;
	struct ice_hw *hw;
	int tx_err = 0;
	int rx_err;

	port = container_of(work, struct ice_ptp_port, ov_work.work);
	pf = ptp_port_to_pf(port);
	hw = &pf->hw;

	if (ice_is_reset_in_progress(pf->state))
		goto requeue;

	if (port->tx.calibrating) {
		tx_err = ice_ptp_check_tx_fifo(port);
		if (!tx_err)
			tx_err = ice_phy_cfg_tx_offset_e82x(hw, port->port_num);
		if (!tx_err)
			port->tx.calibrating = false;
	}

	if (port->rx_calibrating) {
		rx_err = ice_phy_cfg_rx_offset_e82x(hw, port->port_num);
		if (!rx_err)
			port->rx_calibrating = false;
	}

	if (!tx_err && !port->rx_calibrating)
		return;

requeue:
	/* Tx and/or Rx offset not yet configured, try again later */
	kthread_queue_delayed_work(pf->ptp.ports_owner.kworker,
				   &port->ov_work,
				   msecs_to_jiffies(100));
}

/**
 * ice_ptp_port_phy_stop - Stop timestamping for a PHY port
 * @ptp_port: PTP port to stop
 */
static int ice_ptp_port_phy_stop(struct ice_ptp_port *ptp_port)
{
	struct ice_pf *pf = ptp_port_to_pf(ptp_port);
	u8 port = ptp_port->port_num;
	struct ice_hw *hw = &pf->hw;
	int err;

	if (!ice_ptp_is_managed_phy(hw))
		return 0;

	mutex_lock(&ptp_port->ps_lock);

	switch (hw->ptp.phy_model) {
	case ICE_PHY_ETH56G:
		err = ice_stop_phy_timer_eth56g(hw, port, true);
		break;
	case ICE_PHY_E82X:
		kthread_cancel_delayed_work_sync(&ptp_port->ov_work);

		err = ice_stop_phy_timer_e82x(hw, port, true);
		break;
	default:
		err = -ENODEV;
	}
	if (err && err != -EBUSY)
		dev_err(ice_pf_to_dev(pf), "PTP failed to set PHY port %d down, err %d\n",
			port, err);

	mutex_unlock(&ptp_port->ps_lock);

	return err;
}

/**
 * ice_ptp_port_phy_restart - (Re)start and calibrate PHY timestamping
 * @ptp_port: PTP port for which the PHY start is set
 *
 * Start the PHY timestamping block, and initiate Vernier timestamping
 * calibration. If timestamping cannot be calibrated (such as if link is down)
 * then disable the timestamping block instead.
 */
static int ice_ptp_port_phy_restart(struct ice_ptp_port *ptp_port)
{
	struct ice_pf *pf = ptp_port_to_pf(ptp_port);
	u8 port = ptp_port->port_num;
	struct ice_hw *hw = &pf->hw;
	int err;

	if (!ice_ptp_is_managed_phy(hw))
		return 0;

	if (!ptp_port->link_up)
		return ice_ptp_port_phy_stop(ptp_port);

	mutex_lock(&ptp_port->ps_lock);

	switch (hw->ptp.phy_model) {
	case ICE_PHY_ETH56G:
		err = ice_start_phy_timer_eth56g(hw, port);
		break;
	case ICE_PHY_E82X:
		/* Start the PHY timer in Vernier mode */
		kthread_cancel_delayed_work_sync(&ptp_port->ov_work);

		/* temporarily disable Tx timestamps while calibrating
		 * PHY offset
		 */
		spin_lock(&ptp_port->tx.lock);
		ptp_port->tx.calibrating = true;
		spin_unlock(&ptp_port->tx.lock);
		ptp_port->tx_fifo_busy_cnt = 0;
		ptp_port->rx_calibrating = true;

		/* Start the PHY timer in Vernier mode */
		err = ice_start_phy_timer_e82x(hw, port);
		if (err)
			break;

		kthread_queue_delayed_work(pf->ptp.ports_owner.kworker,
					   &ptp_port->ov_work, 0);
		break;
	default:
		err = -ENODEV;
	}

	if (err)
		dev_err(ice_pf_to_dev(pf), "PTP failed to set PHY port %d up, err %d\n",
			port, err);

	mutex_unlock(&ptp_port->ps_lock);

	return err;
}

/**
 * ice_ptp_phy_restart - Restart PHY
 * @pf: Board private structure
 */
int ice_ptp_phy_restart(struct ice_pf *pf)
{
	struct device *dev = ice_pf_to_dev(pf);
	int err;

	err = ice_ptp_port_phy_restart(&pf->ptp.port);

	if (err) {
		dev_err(dev, "Failed to restart PHY, err %d\n", err);
		return err;
	}

	return 0;
}

/**
 * ice_ptp_link_change - Reconfigure PTP after link status change
 * @pf: Board private structure
 * @port: Port for which the PHY start is set
 * @linkup: Link is up or down
 */
void ice_ptp_link_change(struct ice_pf *pf, u8 port, bool linkup)
{
	struct ice_ptp_port *ptp_port;
	struct ice_hw *hw = &pf->hw;

	if (pf->ptp.state != ICE_PTP_READY)
		return;

	if (WARN_ON_ONCE(port >= hw->ptp.num_lports))
		return;

	ptp_port = &pf->ptp.port;
	if (WARN_ON_ONCE(ptp_port->port_num != port))
		return;

	/* Update cached link status for this port immediately */
	ptp_port->link_up = linkup;

	if (ice_is_e825c(hw)) {
		int pin, err;

		for (pin = 0; pin < ICE_E825C_CLK_SYNCE_NUM; pin++) {
			enum ice_e825c_clk_synce clk_pin;
			bool active, flag;
			u8 port_num;
			u8 divider;

			port_num = ptp_port->port_num;
			clk_pin = (enum ice_e825c_clk_synce)pin;
			err = ice_cgu_bypass_mux_port_active_e825c(hw,
								   port_num,
								   &active,
								   clk_pin);
			if (WARN_ON_ONCE(err))
				return;

			flag = test_bit(ICE_FLAG_ITU_G8262_FILTER_USED,
					pf->flags);
			err = ice_cfg_synce_ethdiv_e825c(hw,
							 &divider,
							 clk_pin);
			if (active && flag && WARN_ON_ONCE(err))
				return;
		}
	}

	switch (hw->ptp.phy_model) {
	case ICE_PHY_E810:
		/* Do not reconfigure E810 PHY */
		return;
	case ICE_PHY_ETH56G:
	case ICE_PHY_E82X:
		ice_ptp_port_phy_restart(ptp_port);
		return;
	default:
		dev_warn(ice_pf_to_dev(pf), "%s: Unknown PHY type\n", __func__);
	}
}

/**
 * ice_ptp_cfg_phy_interrupt - Configure PHY interrupt settings
 * @pf: PF private structure
 * @ena: bool value to enable or disable interrupt
 * @threshold: Minimum number of packets at which intr is triggered
 *
 * Utility function to configure all the PHY interrupt settings, including
 * whether the PHY interrupt is enabled, and what threshold to use. Also
 * configures The E82X timestamp owner to react to interrupts from all PHYs.
 */
static int ice_ptp_cfg_phy_interrupt(struct ice_pf *pf, bool ena, u32 threshold)
{
	struct device *dev = ice_pf_to_dev(pf);
	struct ice_hw *hw = &pf->hw;

	ice_ptp_reset_ts_memory(hw);

	switch (hw->ptp.phy_model) {
	case ICE_PHY_ETH56G: {
		int port;

		for (port = 0; port < hw->ptp.num_lports; port++) {
			int err;

			err = ice_phy_cfg_intr_eth56g(hw, port, ena,
						      threshold);
			if (err) {
				dev_err(dev, "Failed to configure PHY interrupt for port %d, err %d\n",
					port, err);
				return err;
			}
		}

		return 0;
	}
	case ICE_PHY_E82X: {
		int quad;

		for (quad = 0; quad < ICE_GET_QUAD_NUM(hw->ptp.num_lports);
		     quad++) {
			int err;

			err = ice_phy_cfg_intr_e82x(hw, quad, ena,
						    threshold);
			if (err) {
				dev_err(dev, "Failed to configure PHY interrupt for quad %d, err %d\n",
					quad, err);
				return err;
			}
		}

		return 0;
	}
	case ICE_PHY_E810:
		return 0;
	case ICE_PHY_UNSUP:
		return -EOPNOTSUPP;
	}

	dev_warn(dev, "%s: Unexpected PHY model %d\n", __func__,
		 hw->ptp.phy_model);
	return -ENODEV;
}

/**
 * ice_ptp_reset_phy_timestamping - Reset PHY timestamping block
 * @pf: Board private structure
 */
static void ice_ptp_reset_phy_timestamping(struct ice_pf *pf)
{
	ice_ptp_port_phy_restart(&pf->ptp.port);
}

/**
 * ice_ptp_restart_all_phy - Restart all PHYs to recalibrate timestamping
 * @pf: Board private structure
 */
static void ice_ptp_restart_all_phy(struct ice_pf *pf)
{
	struct list_head *entry;

	mutex_lock(&pf->ptp.ports_owner.lock);
	if (!list_initialized(pf->ptp.ports_owner.ports)) {
		mutex_unlock(&pf->ptp.ports_owner.lock);
		return;
	}
	list_for_each(entry, &pf->ptp.ports_owner.ports) {
		struct ice_ptp_port *port = list_entry(entry,
						       struct ice_ptp_port,
						       list_member);

		if (port->link_up)
			ice_ptp_port_phy_restart(port);
	}
	mutex_unlock(&pf->ptp.ports_owner.lock);
}

/**
 * ice_ptp_update_incval - Update clock increment rate
 * @pf: Board private structure
 * @time_ref_freq: TIME_REF frequency to use
 * @src_tmr_mode: Src timer mode (nanoseconds or locked)
 */
int
ice_ptp_update_incval(struct ice_pf *pf, enum ice_time_ref_freq time_ref_freq,
		      enum ice_src_tmr_mode src_tmr_mode)
{
	struct device *dev = ice_pf_to_dev(pf);
	struct ice_hw *hw = &pf->hw;
	struct timespec64 ts;
	int err;

	if (pf->ptp.state != ICE_PTP_READY) {
		dev_err(dev, "PTP is currently in %s state, failed to update incval\n",
			ice_ptp_state_str(pf->ptp.state));
		return -EINVAL;
	}

	if (!ice_ptp_lock(hw))
		return -EBUSY;

	err = ice_ptp_write_incval(hw, ice_get_base_incval(hw, src_tmr_mode));
	if (err) {
		dev_err(dev, "PTP failed to update incval, status %d\n", err);
		goto err_unlock;
	}

	ts = ktime_to_timespec64(ktime_get_real());
	err = ice_ptp_write_init(pf, &ts);
	if (err) {
		ice_dev_err_errno(dev, err,
				  "PTP failed to program time registers");
		goto err_unlock;
	}

	/* unlock PTP semaphore first before resetting PHY timestamping */
	ice_ptp_unlock(hw);
	ice_ptp_reset_ts_memory(hw);
	ice_ptp_restart_all_phy(pf);

	return 0;

err_unlock:
	ice_ptp_unlock(hw);

	return err;
}

/**
 * ice_ptp_adjfine - Adjust clock increment rate
 * @info: the driver's PTP info structure
 * @scaled_ppm: Parts per million with 16-bit fractional field
 *
 * Adjust the frequency of the clock by the indicated scaled ppm from the
 * base frequency.
 */
static int ice_ptp_adjfine(struct ptp_clock_info *info, long scaled_ppm)
{
	struct ice_pf *pf = ptp_info_to_pf(info);
	struct ice_hw *hw = &pf->hw;
	u64 incval;
	int err;

	if (hw->ptp.src_tmr_mode == ICE_SRC_TMR_MODE_LOCKED) {
		dev_err(ice_pf_to_dev(pf),
			"adjfreq not supported in locked mode\n");
		return -EPERM;
	}

	incval = adjust_by_scaled_ppm(ice_base_incval(pf), scaled_ppm);
	err = ice_ptp_write_incval_locked(hw, incval);
	if (err) {
		dev_err(ice_pf_to_dev(pf), "PTP failed to set incval, err %d\n",
			err);
		return -EIO;
	}

	return 0;
}

#ifndef HAVE_PTP_CLOCK_INFO_ADJFINE
/**
 * ice_ptp_adjfreq - Adjust the frequency of the clock
 * @info: the driver's PTP info structure
 * @ppb: Parts per billion adjustment from the base
 *
 * Adjust the frequency of the clock by the indicated parts per billion from the
 * base frequency.
 */
static int ice_ptp_adjfreq(struct ptp_clock_info *info, s32 ppb)
{
	long scaled_ppm;

	/*
	 * We want to calculate
	 *
	 *    scaled_ppm = ppb * 2^16 / 1000
	 *
	 * which simplifies to
	 *
	 *    scaled_ppm = ppb * 2^13 / 125
	 */
	scaled_ppm = ((long)ppb << 13) / 125;
	return ice_ptp_adjfine(info, scaled_ppm);
}
#endif

/**
 * ice_ptp_extts_event - Process PTP external clock event
 * @pf: Board private structure
 */
void ice_ptp_extts_event(struct ice_pf *pf)
{
	struct ptp_clock_event event;
	struct ice_hw *hw = &pf->hw;
	u8 chan, tmr_idx;
	u32 hi, lo;

	/* Don't process timestamp events if PTP is not ready */
	if (pf->ptp.state != ICE_PTP_READY)
		return;

	tmr_idx = hw->func_caps.ts_func_info.tmr_index_owned;
	/* Event time is captured by one of the two matched registers
	 *      GLTSYN_EVNT_L: 32 LSB of sampled time event
	 *      GLTSYN_EVNT_H: 32 MSB of sampled time event
	 * Event is defined in GLTSYN_EVNT_0 register
	 */
	for (chan = 0; chan < GLTSYN_EVNT_H_IDX_MAX; chan++) {
		/* Check if channel is enabled */
		if (pf->ptp.ext_ts_irq & (1 << chan)) {
			lo = rd32(hw, GLTSYN_EVNT_L(chan, tmr_idx));
			hi = rd32(hw, GLTSYN_EVNT_H(chan, tmr_idx));
			event.timestamp = (((u64)hi) << 32) | lo;
			event.type = PTP_CLOCK_EXTTS;
			event.index = chan;

			pf->ptp.ext_ts_irq &= ~(1 << chan);

			/* Fire event if not filtered by CGU state */
			if (ice_is_feature_supported(pf, ICE_F_CGU) &&
			    test_bit(ICE_FLAG_DPLL_MONITOR, pf->flags) &&
			    pf->ptp_dpll_state != DPLL_LOCK_STATUS_LOCKED &&
			    pf->ptp_dpll_state !=
			    DPLL_LOCK_STATUS_LOCKED_HO_ACQ)
				continue;

			ptp_clock_event(pf->ptp.clock, &event);
		}
	}
}

/**
 * ice_ptp_cfg_extts - Configure EXTTS pin and channel
 * @pf: Board private structure
 * @chan: GPIO channel
 * @config: desired EXTTS configuration.
 * @store: If set to true, the values will be stored
 *
 * Configure an external timestamp event on the requested channel.
 */
static void ice_ptp_cfg_extts(struct ice_pf *pf, unsigned int chan,
			      struct ice_extts_channel *config, bool store)
{
	u32 func, aux_reg, gpio_reg, irq_reg;
	struct ice_hw *hw = &pf->hw;
	u8 tmr_idx;

	tmr_idx = hw->func_caps.ts_func_info.tmr_index_owned;

	irq_reg = rd32(hw, PFINT_OICR_ENA);

	if (config->ena) {
		/* Enable the interrupt */
		irq_reg |= PFINT_OICR_TSYN_EVNT_M;
		aux_reg = GLTSYN_AUX_IN_0_INT_ENA_M;

#define GLTSYN_AUX_IN_0_EVNTLVL_RISING_EDGE	BIT(0)
#define GLTSYN_AUX_IN_0_EVNTLVL_FALLING_EDGE	BIT(1)

		/* set event level to requested edge */
		if (config->flags & PTP_FALLING_EDGE)
			aux_reg |= GLTSYN_AUX_IN_0_EVNTLVL_FALLING_EDGE;
		if (config->flags & PTP_RISING_EDGE)
			aux_reg |= GLTSYN_AUX_IN_0_EVNTLVL_RISING_EDGE;

		/* Write GPIO CTL reg.
		 * 0x1 is input sampled by EVENT register(channel)
		 * + num_in_channels * tmr_idx
		 */
		func = 1 + chan + (tmr_idx * 3);
		gpio_reg = FIELD_PREP(GLGEN_GPIO_CTL_PIN_FUNC_M, func);
		pf->ptp.ext_ts_chan |= (1 << chan);
	} else {
		/* clear the values we set to reset defaults */
		aux_reg = 0;
		gpio_reg = 0;
		pf->ptp.ext_ts_chan &= ~(1 << chan);
		if (!pf->ptp.ext_ts_chan)
			irq_reg &= ~PFINT_OICR_TSYN_EVNT_M;
	}

	wr32(hw, PFINT_OICR_ENA, irq_reg);
	wr32(hw, GLTSYN_AUX_IN(chan, tmr_idx), aux_reg);
	wr32(hw, GLGEN_GPIO_CTL(config->gpio_pin), gpio_reg);

	if (store)
		memcpy(&pf->ptp.extts_channels[chan], config, sizeof(*config));
}

/**
 * ice_ptp_disable_all_extts - Disable all EXTTS channels
 * @pf: Board private structure
 */
static void ice_ptp_disable_all_extts(struct ice_pf *pf)
{
	struct ice_extts_channel extts_cfg = {};
	int i;

	for (i = 0; i < pf->ptp.info.n_ext_ts ; i++) {
		if (pf->ptp.extts_channels[i].ena) {
			extts_cfg.gpio_pin = pf->ptp.extts_channels[i].gpio_pin;
			extts_cfg.ena = false;
			ice_ptp_cfg_extts(pf, i, &extts_cfg, false);
		}
	}

	synchronize_irq(ice_get_irq_num(pf, pf->oicr_idx));
}

/**
 * ice_ptp_enable_all_extts - Enable all EXTTS channels
 * @pf: Board private structure
 *
 * Called during reset to restore user configuration.
 */
static void ice_ptp_enable_all_extts(struct ice_pf *pf)
{
	int i;

	for (i = 0; i < pf->ptp.info.n_ext_ts ; i++) {
		if (pf->ptp.extts_channels[i].ena) {
			ice_ptp_cfg_extts(pf, i, &pf->ptp.extts_channels[i],
					  false);
		}
	}
}

/**
 * ice_ptp_cfg_clkout - Configure clock to generate periodic wave
 * @pf: Board private structure
 * @chan: GPIO channel (0-3)
 * @config: desired periodic clk configuration. NULL will disable channel
 * @store: If set to true the values will be stored
 *
 * Configure the internal clock generator modules to generate the clock wave of
 * specified period.
 */
static int ice_ptp_cfg_clkout(struct ice_pf *pf, unsigned int chan,
			      struct ice_perout_channel *config, bool store)
{
	u64 current_time, period, start_time, phase;
	struct ice_hw *hw = &pf->hw;
	u32 func, val, gpio_pin;
	u8 tmr_idx;

	tmr_idx = hw->func_caps.ts_func_info.tmr_index_owned;

	/* 0. Reset mode & out_en in AUX_OUT */
	wr32(hw, GLTSYN_AUX_OUT(chan, tmr_idx), 0);

	/* If we're disabling the output, clear out CLKO and TGT and keep
	 * output level low
	 */
	if (!config || !config->ena) {
		wr32(hw, GLTSYN_CLKO(chan, tmr_idx), 0);
		wr32(hw, GLTSYN_TGT_L(chan, tmr_idx), 0);
		wr32(hw, GLTSYN_TGT_H(chan, tmr_idx), 0);

		val = GLGEN_GPIO_CTL_PIN_DIR_M;
		gpio_pin = pf->ptp.perout_channels[chan].gpio_pin;
		wr32(hw, GLGEN_GPIO_CTL(gpio_pin), val);

		/* Store the value if requested */
		if (store)
			memset(&pf->ptp.perout_channels[chan], 0,
			       sizeof(struct ice_perout_channel));

		return 0;
	}
	period = config->period;
	start_time = config->start_time;
	div64_u64_rem(start_time, period, &phase);
	gpio_pin = config->gpio_pin;

	/* 1. Write clkout with half of required period value */
	if (period & 0x1) {
		dev_err(ice_pf_to_dev(pf), "CLK Period must be an even value\n");
		goto err;
	}

	period >>= 1;
	period = ice_ptp_ns2ticks(pf, period);
	start_time = ice_ptp_ns2ticks(pf, start_time);

	/* For proper operation, the GLTSYN_CLKO must be larger than clock tick
	 */
#define MIN_PULSE 3
	if (period <= MIN_PULSE || period > U32_MAX) {
		dev_err(ice_pf_to_dev(pf), "CLK Period must be > %d && < 2^33",
			MIN_PULSE * 2);
		goto err;
	}

	wr32(hw, GLTSYN_CLKO(chan, tmr_idx), lower_32_bits(period));

	/* Allow time for programming before start_time is hit */
	current_time = ice_ptp_read_src_clk_reg(pf, NULL);

	/* if start time is in the past start the timer at the nearest second
	 * maintaining phase
	 */
	if (start_time < current_time)
		start_time = div64_u64(current_time + NSEC_PER_SEC - 1,
				       NSEC_PER_SEC) * NSEC_PER_SEC + phase;

	start_time -= ice_prop_delay(hw);

	/* 2. Write TARGET time */
	wr32(hw, GLTSYN_TGT_L(chan, tmr_idx), lower_32_bits(start_time));
	wr32(hw, GLTSYN_TGT_H(chan, tmr_idx), upper_32_bits(start_time));

	/* 3. Write AUX_OUT register */
	val = GLTSYN_AUX_OUT_0_OUT_ENA_M | GLTSYN_AUX_OUT_0_OUTMOD_M;
	wr32(hw, GLTSYN_AUX_OUT(chan, tmr_idx), val);

	/* 4. write GPIO CTL reg */
	func = 8 + chan + (tmr_idx * 4);
	val = GLGEN_GPIO_CTL_PIN_DIR_M |
	      FIELD_PREP(GLGEN_GPIO_CTL_PIN_FUNC_M, func);
	wr32(hw, GLGEN_GPIO_CTL(gpio_pin), val);

	/* Store the value if requested */
	if (store) {
		memcpy(&pf->ptp.perout_channels[chan], config,
		       sizeof(struct ice_perout_channel));
		pf->ptp.perout_channels[chan].start_time = phase;
	}

	return 0;
err:
	dev_err(ice_pf_to_dev(pf), "PTP failed to cfg per_clk\n");
	return -EFAULT;
}

/**
 * ice_ptp_disable_all_clkout - Disable all currently configured outputs
 * @pf: pointer to the PF structure
 *
 * Disable all currently configured clock outputs. This is necessary before
 * certain changes to the PTP hardware clock. Use ice_ptp_enable_all_clkout to
 * re-enable the clocks again.
 */
static void ice_ptp_disable_all_clkout(struct ice_pf *pf)
{
	uint i;

	for (i = 0; i < pf->ptp.info.n_per_out; i++)
		if (pf->ptp.perout_channels[i].ena)
			ice_ptp_cfg_clkout(pf, i, NULL, false);
}

/**
 * ice_ptp_enable_all_clkout - Enable all configured periodic clock outputs
 * @pf: pointer to the PF structure
 *
 * Enable all currently configured clock outputs. Use this after
 * ice_ptp_disable_all_clkout to reconfigure the output signals according to
 * their configuration.
 */
static void ice_ptp_enable_all_clkout(struct ice_pf *pf)
{
	uint i;

	for (i = 0; i < pf->ptp.info.n_per_out; i++)
		if (pf->ptp.perout_channels[i].ena)
			ice_ptp_cfg_clkout(pf, i, &pf->ptp.perout_channels[i],
					   false);
}

/**
 * ice_get_gpio - get GPIO pin number
 * @ptp: PF PTP structure
 * @chan: Requested channel
 * @func: Requested function
 *
 * Search in ice_ptp_pin_desc the GPIO number for given channel and
 * functionality
 * Returns GPIO number on success, or -1 if not found
 */
static int ice_get_gpio(struct ice_ptp *ptp, u8 chan, u8 func)
{
	int i;

	for (i = 0; i < ptp->ice_pin_desc_num; i++)
		if (ptp->ice_pin_desc[i].chan == chan) {
			if (func == PTP_CLK_REQ_EXTTS &&
			    ptp->ice_pin_desc[i].gpio_in != -1)
				return ptp->ice_pin_desc[i].gpio_in;
			else if (func == PTP_CLK_REQ_PEROUT &&
				 ptp->ice_pin_desc[i].gpio_out != -1)
				return ptp->ice_pin_desc[i].gpio_out;
		}

	return NO_GPIO;
}

/**
 * ice_ptp_gpio_enable_e810 - Enable/disable ancillary features of PHC
 * @info: the driver's PTP info structure
 * @rq: The requested feature to change
 * @on: Enable/disable flag
 */
static int
ice_ptp_gpio_enable_e810(struct ptp_clock_info *info,
			 struct ptp_clock_request *rq, int on)
{
	struct ice_pf *pf = ptp_info_to_pf(info);
	struct ice_extts_channel extts_cfg = {};
	struct ice_perout_channel clk_cfg = {};
	bool sma_pres = false;
	unsigned int chan;
	u32 gpio_pin;
	int err = 0;

	if (ice_is_feature_supported(pf, ICE_F_SMA_CTRL))
		sma_pres = true;

	switch (rq->type) {
	case PTP_CLK_REQ_PEROUT:
		chan = rq->perout.index;
		if (pf->ptp.sdp_section_exist) {
			clk_cfg.gpio_pin = ice_get_gpio(&pf->ptp, chan,
							PTP_CLK_REQ_PEROUT);
			if (clk_cfg.gpio_pin == NO_GPIO)
				return -1;
		} else if (sma_pres) {
			if (chan == pf->ptp.ice_pin_desc[SMA1].chan)
				clk_cfg.gpio_pin = GPIO_20;
			else if (chan == pf->ptp.ice_pin_desc[SMA2].chan)
				clk_cfg.gpio_pin = GPIO_22;
			else
				return -1;
		} else if (ice_is_e810t(&pf->hw)) {
			if (chan == 0)
				clk_cfg.gpio_pin = GPIO_20;
			else
				clk_cfg.gpio_pin = GPIO_22;
		} else if (chan == PPS_CLK_GEN_CHAN) {
			clk_cfg.gpio_pin = PPS_PIN_INDEX;
		} else {
			clk_cfg.gpio_pin = chan;
		}

		clk_cfg.period = ((rq->perout.period.sec * NSEC_PER_SEC) +
				   rq->perout.period.nsec);
		clk_cfg.start_time = ((rq->perout.start.sec * NSEC_PER_SEC) +
				       rq->perout.start.nsec);
		clk_cfg.ena = !!on;

		err = ice_ptp_cfg_clkout(pf, chan, &clk_cfg, true);
		break;
	case PTP_CLK_REQ_EXTTS:
		if (pf->hw.ptp.src_tmr_mode == ICE_SRC_TMR_MODE_LOCKED) {
			dev_err(ice_pf_to_dev(pf), "Locked mode EXTTS not supported\n");
			return -EOPNOTSUPP;
		}

		chan = rq->extts.index;

		if (pf->ptp.sdp_section_exist) {
			gpio_pin = ice_get_gpio(&pf->ptp, chan,
						PTP_CLK_REQ_EXTTS);
			if (gpio_pin == NO_GPIO)
				return -1;
		} else if (sma_pres) {
			if (chan < pf->ptp.ice_pin_desc[SMA2].chan)
				gpio_pin = GPIO_21;
			else
				gpio_pin = GPIO_23;
		} else if (ice_is_e810t(&pf->hw)) {
			if (chan == 0)
				gpio_pin = GPIO_21;
			else
				gpio_pin = GPIO_23;
		} else {
			gpio_pin = chan;
		}

		extts_cfg.flags = rq->extts.flags;
		extts_cfg.gpio_pin = gpio_pin;
		extts_cfg.ena = !!on;

		ice_ptp_cfg_extts(pf, chan, &extts_cfg, true);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return err;
}

/**
 * ice_verify_pin_e82x - verify if pin supports requested pin function
 * @info: the driver's PTP info structure
 * @pin: Pin index
 * @func: Assigned function
 * @chan: Assigned channel
 */
static int
ice_verify_pin_e82x(struct ptp_clock_info *info, unsigned int pin,
		    enum ptp_pin_function func, unsigned int chan)
{
	/* Don't allow channel and function reassignment */
	if (chan != ice_pin_desc_e82x[pin].chan ||
	    func != ice_pin_desc_e82x[pin].func)
		return -EOPNOTSUPP;

	return 0;
}

/**
 * ice_ptp_gpio_enable_e82x - Enable/disable ancillary features of PHC
 * @info: the driver's PTP info structure
 * @rq: The requested feature to change
 * @on: Enable/disable flag
 */
static int
ice_ptp_gpio_enable_e82x(struct ptp_clock_info *info,
			 struct ptp_clock_request *rq, int on)
{
	struct ice_pf *pf = ptp_info_to_pf(info);
	struct ice_extts_channel extts_cfg = {};
	struct ice_perout_channel clk_cfg = {};
	int err = 0;

	switch (rq->type) {
	case PTP_CLK_REQ_PEROUT:
		clk_cfg.gpio_pin = PPS_PIN_INDEX;
		clk_cfg.period = NSEC_PER_SEC;
		clk_cfg.start_time = ((rq->perout.start.sec * NSEC_PER_SEC) +
				       rq->perout.start.nsec);
		clk_cfg.ena = !!on;

		err = ice_ptp_cfg_clkout(pf, rq->perout.index, &clk_cfg, true);
		break;
	case PTP_CLK_REQ_EXTTS:
		if (pf->hw.ptp.src_tmr_mode == ICE_SRC_TMR_MODE_LOCKED) {
			dev_err(ice_pf_to_dev(pf), "Locked mode EXTTS not supported\n");
			return -EOPNOTSUPP;
		}

		extts_cfg.flags = rq->extts.flags;
		extts_cfg.gpio_pin = TIME_SYNC_PIN_INDEX,
		extts_cfg.ena = !!on;

		ice_ptp_cfg_extts(pf, rq->extts.index, &extts_cfg, true);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return err;
}

/**
 * ice_ptp_gettimex64 - Get the time of the clock
 * @info: the driver's PTP info structure
 * @ts: timespec64 structure to hold the current time value
 * @sts: Optional parameter for holding a pair of system timestamps from
 *       the system clock. Will be ignored if NULL is given.
 *
 * Read the device clock and return the correct value on ns, after converting it
 * into a timespec struct.
 */
static int
ice_ptp_gettimex64(struct ptp_clock_info *info, struct timespec64 *ts,
		   struct ptp_system_timestamp *sts)
{
	struct ice_pf *pf = ptp_info_to_pf(info);
	u64 time_ns;

	time_ns = ice_ptp_ticks2ns(pf, ice_ptp_read_src_clk_reg(pf, sts));

	*ts = ns_to_timespec64(time_ns);

	return 0;
}

#ifndef HAVE_PTP_CLOCK_INFO_GETTIMEX64
/**
 * ice_ptp_gettime64 - Get the time of the clock
 * @info: the driver's PTP info structure
 * @ts: timespec64 structure to hold the current time value
 *
 * Read the device clock and return the correct value on ns, after converting it
 * into a timespec struct.
 */
static int ice_ptp_gettime64(struct ptp_clock_info *info, struct timespec64 *ts)
{
	return ice_ptp_gettimex64(info, ts, NULL);
}

#ifndef HAVE_PTP_CLOCK_INFO_GETTIME64
/**
 * ice_ptp_gettime32 - Get the time of the clock
 * @info: the driver's PTP info structure
 * @ts: timespec structure to hold the current time value
 *
 * Read the device clock and return the correct value on ns, after converting it
 * into a timespec struct.
 */
static int ice_ptp_gettime32(struct ptp_clock_info *info, struct timespec *ts)
{
	struct timespec64 ts64;

	if (ice_ptp_gettime64(info, &ts64))
		return -EFAULT;

	*ts = timespec64_to_timespec(ts64);
	return 0;
}

#endif /* !HAVE_PTP_CLOCK_INFO_GETTIME64 */
#endif /* !HAVE_PTP_CLOCK_INFO_GETTIMEX64 */
/**
 * ice_ptp_settime64 - Set the time of the clock
 * @info: the driver's PTP info structure
 * @ts: timespec64 structure that holds the new time value
 *
 * Set the device clock to the user input value. The conversion from timespec
 * to ns happens in the write function.
 */
static int
ice_ptp_settime64(struct ptp_clock_info *info, const struct timespec64 *ts)
{
	struct ice_pf *pf = ptp_info_to_pf(info);
	struct timespec64 ts64 = *ts;
	struct ice_hw *hw = &pf->hw;
	int status;
	int err;

	/* For Vernier mode, we need to recalibrate after new settime.
	 * Start with marking timestamps as invalid.
	 */
	status = ice_ptp_clear_phy_offset_ready(hw);
	if (status)
		dev_warn(ice_pf_to_dev(pf), "Failed to mark timestamps as invalid before settime\n");

	if (!ice_ptp_lock(hw)) {
		err = -EBUSY;
		goto exit;
	}

	/* Disable periodic outputs */
	ice_ptp_disable_all_clkout(pf);

	err = ice_ptp_write_init(pf, &ts64);

	/* Reenable periodic outputs */
	ice_ptp_enable_all_clkout(pf);
	ice_ptp_unlock(hw);

	if (!err)
		ice_ptp_reset_cached_phctime(pf);

	/* Recalibrate and re-enable timestamp block */
	if (hw->ptp.phy_model == ICE_PHY_E82X)
		ice_ptp_restart_all_phy(pf);
exit:
	if (err) {
		dev_err(ice_pf_to_dev(pf), "PTP failed to set time %d\n", err);
		return err;
	}

	return 0;
}

#ifndef HAVE_PTP_CLOCK_INFO_GETTIME64
/**
 * ice_ptp_settime32 - Set the time of the clock
 * @info: the driver's PTP info structure
 * @ts: timespec structure that holds the new time value
 *
 * Set the device clock to the user input value. The conversion from timespec
 * to ns happens in the write function.
 */
static int
ice_ptp_settime32(struct ptp_clock_info *info, const struct timespec *ts)
{
	struct timespec64 ts64 = timespec_to_timespec64(*ts);

	return ice_ptp_settime64(info, &ts64);
}
#endif /* !HAVE_PTP_CLOCK_INFO_GETTIME64 */

/**
 * ice_ptp_adjtime_nonatomic - Do a non-atomic clock adjustment
 * @info: the driver's PTP info structure
 * @delta: Offset in nanoseconds to adjust the time by
 */
static int ice_ptp_adjtime_nonatomic(struct ptp_clock_info *info, s64 delta)
{
	struct timespec64 now, then;
	int ret;

	then = ns_to_timespec64(delta);
	ret = ice_ptp_gettimex64(info, &now, NULL);
	if (ret)
		return ret;
	now = timespec64_add(now, then);

	return ice_ptp_settime64(info, (const struct timespec64 *)&now);
}

/**
 * ice_ptp_adjtime - Adjust the time of the clock by the indicated delta
 * @info: the driver's PTP info structure
 * @delta: Offset in nanoseconds to adjust the time by
 */
static int ice_ptp_adjtime(struct ptp_clock_info *info, s64 delta)
{
	struct ice_pf *pf = ptp_info_to_pf(info);
	struct ice_hw *hw = &pf->hw;
	s64 delta_ns = delta;
	struct device *dev;
	int err;

	dev = ice_pf_to_dev(pf);

	if (delta >= 0)
		delta = ice_ptp_ns2ticks(pf, delta);
	else
		delta = -ice_ptp_ns2ticks(pf, -delta);

	/* Hardware only supports atomic adjustments using signed 32-bit
	 * integers. For any adjustment outside this range, perform
	 * a non-atomic get->adjust->set flow.
	 */
	if (delta > S32_MAX || delta < S32_MIN) {
		dev_dbg(dev, "delta = %lld, adjtime non-atomic\n", delta);
		return ice_ptp_adjtime_nonatomic(info, delta_ns);
	}

	if (!ice_ptp_lock(hw)) {
		dev_err(dev, "PTP failed to acquire semaphore in adjtime\n");
		return -EBUSY;
	}

	/* Disable periodic outputs */
	ice_ptp_disable_all_clkout(pf);

	err = ice_ptp_write_adj(pf, delta);

	/* Reenable periodic outputs */
	ice_ptp_enable_all_clkout(pf);

	ice_ptp_unlock(hw);

	if (err) {
		ice_dev_err_errno(dev, err, "PTP failed to adjust time");
		return err;
	}

	ice_ptp_reset_cached_phctime(pf);

	return 0;
}

#ifdef HAVE_PTP_CROSSTIMESTAMP
/**
 * ice_ptp_get_syncdevicetime - Get the cross time stamp info
 * @device: Current device time
 * @system: System counter value read synchronously with device time
 * @ctx: Context provided by timekeeping code
 *
 * Read device and system (ART) clock simultaneously and return the corrected
 * clock values in ns.
 */
static int
ice_ptp_get_syncdevicetime(ktime_t *device,
			   struct system_counterval_t *system,
			   void *ctx)
{
	struct ice_pf *pf = ctx;
	struct ice_hw *hw = &pf->hw;
	u32 hh_lock, hh_art_ctl;
	int i;

#define MAX_HH_HW_LOCK_TRIES	5
#define MAX_HH_CTL_LOCK_TRIES	100

	for (i = 0; i < MAX_HH_HW_LOCK_TRIES; i++) {
		/* Get the HW lock */
		hh_lock = rd32(hw, PFHH_SEM + (PFTSYN_SEM_BYTES * hw->pf_id));
		if (hh_lock & PFHH_SEM_BUSY_M) {
			usleep_range(10000, 15000);
			continue;
		}
		break;
	}
	if (hh_lock & PFHH_SEM_BUSY_M) {
		dev_err(ice_pf_to_dev(pf), "PTP failed to get hh lock\n");
		return -EBUSY;
	}

	/* Program cmd to master timer */
	ice_ptp_src_cmd(hw, ICE_PTP_READ_TIME);

	/* Start the ART and device clock sync sequence */
	hh_art_ctl = rd32(hw, GLHH_ART_CTL);
	hh_art_ctl = hh_art_ctl | GLHH_ART_CTL_ACTIVE_M;
	wr32(hw, GLHH_ART_CTL, hh_art_ctl);

	for (i = 0; i < MAX_HH_CTL_LOCK_TRIES; i++) {
		/* Wait for sync to complete */
		hh_art_ctl = rd32(hw, GLHH_ART_CTL);
		if (hh_art_ctl & GLHH_ART_CTL_ACTIVE_M) {
			udelay(1);
			continue;
		} else {
			u32 hh_ts_lo, hh_ts_hi, tmr_idx;
			u64 hh_ts;

			tmr_idx = hw->func_caps.ts_func_info.tmr_index_assoc;
			/* Read ART time */
			hh_ts_lo = rd32(hw, GLHH_ART_TIME_L);
			hh_ts_hi = rd32(hw, GLHH_ART_TIME_H);
			hh_ts = ((u64)hh_ts_hi << 32) | hh_ts_lo;
			*system = convert_art_ns_to_tsc(hh_ts);
			/* Read Device source clock time */
			hh_ts_lo = rd32(hw, GLTSYN_HHTIME_L(tmr_idx));
			hh_ts_hi = rd32(hw, GLTSYN_HHTIME_H(tmr_idx));
			hh_ts = ice_ptp_ticks2ns(pf, (((u64)hh_ts_hi << 32) |
						      hh_ts_lo));
			*device = ns_to_ktime(hh_ts);
			break;
		}
	}

	/* Clear the master timer */
	ice_ptp_src_cmd(hw, ICE_PTP_NOP);

	/* Release HW lock */
	hh_lock = rd32(hw, PFHH_SEM + (PFTSYN_SEM_BYTES * hw->pf_id));
	hh_lock = hh_lock & ~PFHH_SEM_BUSY_M;
	wr32(hw, PFHH_SEM + (PFTSYN_SEM_BYTES * hw->pf_id), hh_lock);

	if (i == MAX_HH_CTL_LOCK_TRIES)
		return -ETIMEDOUT;

	return 0;
}

/**
 * ice_ptp_getcrosststamp_e82x - Capture a device cross timestamp
 * @info: the driver's PTP info structure
 * @cts: The memory to fill the cross timestamp info
 *
 * Capture a cross timestamp between the ART and the device PTP hardware
 * clock. Fill the cross timestamp information and report it back to the
 * caller.
 *
 * This is only valid for E82X and E823 devices which have support for
 * generating the cross timestamp via PCIe PTM.
 *
 * In order to correctly correlate the ART timestamp back to the TSC time, the
 * CPU must have X86_FEATURE_TSC_KNOWN_FREQ.
 */
static int
ice_ptp_getcrosststamp_e82x(struct ptp_clock_info *info,
			    struct system_device_crosststamp *cts)
{
	struct ice_pf *pf = ptp_info_to_pf(info);

	return get_device_system_crosststamp(ice_ptp_get_syncdevicetime,
					     pf, NULL, cts);
}
#endif /* HAVE_PTP_CROSSTIMESTAMP */

/**
 * ice_ptp_get_ts_config - ioctl interface to read the timestamping config
 * @pf: Board private structure
 * @ifr: ioctl data
 *
 * Copy the timestamping config to user buffer
 */
int ice_ptp_get_ts_config(struct ice_pf *pf, struct ifreq *ifr)
{
	struct hwtstamp_config *config;

	if (pf->ptp.state != ICE_PTP_READY)
		return -EIO;

	config = &pf->ptp.tstamp_config;

	return copy_to_user(ifr->ifr_data, config, sizeof(*config)) ?
		-EFAULT : 0;
}

/**
 * ice_ptp_set_timestamp_mode - Setup driver for requested timestamp mode
 * @pf: Board private structure
 * @config: hwtstamp settings requested or saved
 */
static int
ice_ptp_set_timestamp_mode(struct ice_pf *pf, struct hwtstamp_config *config)
{
	switch (config->tx_type) {
	case HWTSTAMP_TX_OFF:
		pf->ptp.tstamp_config.tx_type = HWTSTAMP_TX_OFF;
		break;
	case HWTSTAMP_TX_ON:
		pf->ptp.tstamp_config.tx_type = HWTSTAMP_TX_ON;
		break;
	default:
		return -ERANGE;
	}

	switch (config->rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		pf->ptp.tstamp_config.rx_filter = HWTSTAMP_FILTER_NONE;
		break;
	case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V1_L4_SYNC:
	case HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
#ifdef HAVE_HWTSTAMP_FILTER_NTP_ALL
	case HWTSTAMP_FILTER_NTP_ALL:
#endif /* HAVE_HWTSTAMP_FILTER_NTP_ALL */
	case HWTSTAMP_FILTER_ALL:
		pf->ptp.tstamp_config.rx_filter = HWTSTAMP_FILTER_ALL;
		break;
	default:
		return -ERANGE;
	}

	/* Immediately update the device timestamping mode */
	ice_ptp_restore_timestamp_mode(pf);

	return 0;
}

/**
 * ice_ptp_set_ts_config - ioctl interface to control the timestamping
 * @pf: Board private structure
 * @ifr: ioctl data
 *
 * Get the user config and store it
 */
int ice_ptp_set_ts_config(struct ice_pf *pf, struct ifreq *ifr)
{
	struct hwtstamp_config config;
	int err;

	if (pf->ptp.state != ICE_PTP_READY)
		return -EAGAIN;

	if (copy_from_user(&config, ifr->ifr_data, sizeof(config)))
		return -EFAULT;

	err = ice_ptp_set_timestamp_mode(pf, &config);
	if (err)
		return err;

	/* Return the actual configuration set */
	config = pf->ptp.tstamp_config;

	return copy_to_user(ifr->ifr_data, &config, sizeof(config)) ?
		-EFAULT : 0;
}

/**
 * ice_ptp_rx_hwtstamp - Check for an Rx timestamp
 * @rx_ring: Ring to get the VSI info
 * @rx_desc: Receive descriptor
 * @skb: Particular skb to send timestamp with
 *
 * The driver receives a notification in the receive descriptor with timestamp.
 * The timestamp is in ns, so we must convert the result first.
 */
void
ice_ptp_rx_hwtstamp(struct ice_rx_ring *rx_ring,
		    union ice_32b_rx_flex_desc *rx_desc, struct sk_buff *skb)
{
	struct skb_shared_hwtstamps *hwtstamps;
	u64 ts_ns, cached_time;
	u32 ts_high;

	if (!(rx_desc->wb.time_stamp_low & ICE_PTP_TS_VALID))
		return;

	cached_time = READ_ONCE(rx_ring->cached_phctime);

	/* Do not report a timestamp if we don't have a cached PHC time */
	if (!cached_time)
		return;

	/* Use ice_ptp_extend_32b_ts directly, using the ring-specific cached
	 * PHC value, rather than accessing the PF. This also allows us to
	 * simply pass the upper 32bits of nanoseconds directly. Calling
	 * ice_ptp_extend_40b_ts is unnecessary as it would just discard these
	 * bits itself.
	 */
	ts_high = le32_to_cpu(rx_desc->wb.flex_ts.ts_high);
	ts_ns = ice_ptp_extend_32b_ts(cached_time, ts_high);
	ts_ns = ice_ptp_ticks2ns(rx_ring->vsi->back, ts_ns);

	hwtstamps = skb_hwtstamps(skb);
	memset(hwtstamps, 0, sizeof(*hwtstamps));
	hwtstamps->hwtstamp = ns_to_ktime(ts_ns);
}

/**
 * ice_ptp_disable_sma_pins_e810t - Disable E810-T SMA pins
 * @pf: pointer to the PF structure
 * @info: PTP clock info structure
 *
 * Disable the OS access to the SMA pins. Called to clear out the OS
 * indications of pin support when we fail to setup the E810-T SMA control
 * register.
 */
static void
ice_ptp_disable_sma_pins_e810t(struct ice_pf *pf, struct ptp_clock_info *info)
{
	struct device *dev = ice_pf_to_dev(pf);

	dev_warn(dev, "Failed to configure E810-T SMA pin control\n");

	info->enable = NULL;
	info->verify = NULL;
	info->n_pins = 0;
	info->n_ext_ts = 0;
	info->n_per_out = 0;
}

/**
 * ice_update_ptp_pin_from_nvm - update ptp_pins structure from NVM
 * @pf: pointer to the PF structure
 * @sdp_entries: SDP connection section from NVM
 * @nvm_entries: number of valid entries in sdp_entries
 *
 * Updates ptp_pin structure according to SDP connection section in NVM
 * Returns 0 on success, or -1 if section is corrupted
 */
static int ice_update_ptp_pin_from_nvm(struct ice_pf *pf, u16 *sdp_entries,
				       int nvm_entries)
{
	struct device *dev = ice_pf_to_dev(pf);
	struct ice_ptp *ptp = &pf->ptp;
	struct ice_ptp_pin_desc *ptp_pins;
	int pin_bitmap, idx, i, j;
	u8 chan, direction, sdp_number;

	ptp->ice_pin_desc = devm_kcalloc(dev, ptp->ice_pin_desc_num,
					 sizeof(struct ice_ptp_pin_desc),
					 GFP_KERNEL);
	if (!ptp->ice_pin_desc) {
		dev_err(dev, "Failed to allocate ice_ptp_pin_desc\n");
		return -1;
	}

	ptp_pins = ptp->ice_pin_desc;
	for (i = 0; i < ptp->ice_pin_desc_num; i++) {
		ptp_pins[i].pin_name_idx = NO_PTP_PIN;
		ptp_pins[i].func = PTP_PF_NONE;
		ptp_pins[i].index = -1;
		ptp_pins[i].gpio_in = -1;
		ptp_pins[i].gpio_out = -1;
	}

	for (i = 0; i < nvm_entries; i++) {
		u16 sdp = sdp_entries[i];

		pin_bitmap = FIELD_GET(ICE_AQC_NVM_SDP_CFG_PIN_MASK, sdp);
		chan = FIELD_GET(ICE_AQC_NVM_SDP_CFG_CHAN_MASK, sdp);
		direction = FIELD_GET(ICE_AQC_NVM_SDP_CFG_DIR_MASK, sdp);
		sdp_number = FIELD_GET(ICE_AQC_NVM_SDP_CFG_SDP_NUM_MASK, sdp);
		for (j = 0; j < ICE_AQC_NVM_SDP_CFG_PIN_SIZE; j++) {
			if ((pin_bitmap >> j) & 0x1) {
				idx = ice_get_pin_idx(ptp_pins, j,
						      ptp->ice_pin_desc_num);
				if (idx < 0) {
					dev_err(dev, "E810-T SMA pin not exist\n");
					return -1;
				}
				ptp_pins[idx].func =
					ice_get_pin_func(direction,
							 ptp_pins[idx].func);
				ptp_pins[idx].chan = chan;
				if (direction)
					ptp_pins[idx].gpio_out = sdp_number;
				else
					ptp_pins[idx].gpio_in = sdp_number;
			}
		}
	}

	for (i = 0; i < ptp->ice_pin_desc_num; i++) {
		dev_dbg(dev, "E810 entry[%d] : index %d func %d channel %d gpio_out %d gpio_in %d\n",
			i, ptp_pins[i].pin_name_idx,
			ptp_pins[i].func, ptp->ice_pin_desc[i].chan,
			ptp_pins[i].gpio_out, ptp_pins[i].gpio_in);
	}

	return 0;
}

/**
 * ice_ptp_setup_sma_pins_e810t - Setup the SMA pins
 * @pf: pointer to the PF structure
 * @info: PTP clock info structure
 * @pin_config_num: number of pins on pin_config interface
 *
 * Finish setting up the SMA pins by allocating pin_config, and setting it up
 * according to the current status of the SMA. On failure, disable all of the
 * extended SMA pin support.
 */
static void
ice_ptp_setup_sma_pins_e810t(struct ice_pf *pf, struct ptp_clock_info *info,
			     u8 pin_config_num)
{
	struct device *dev = ice_pf_to_dev(pf);
	struct ice_ptp *ptp = &pf->ptp;
	u8 default_ice_pin_desc_num;
	int err;

	default_ice_pin_desc_num = NUM_PTP_PINS_E810T;
	if (!ptp->sdp_section_exist) {
		ptp->ice_pin_desc_num = default_ice_pin_desc_num;
		ptp->ice_pin_desc =
				devm_kcalloc(dev, default_ice_pin_desc_num,
					     sizeof(struct ice_ptp_pin_desc),
					     GFP_KERNEL);
		if (!ptp->ice_pin_desc) {
			dev_err(dev, "Failed to allocate ice_ptp_pin_desc\n");
			ice_ptp_disable_sma_pins_e810t(pf, info);
			return;
		}
		memcpy(ptp->ice_pin_desc, ice_pin_desc_e810t_default,
		       sizeof(ice_pin_desc_e810t_default));
	}

	info->n_pins = ptp->sdp_section_exist ? pin_config_num :
						default_ice_pin_desc_num;

	/* Allocate memory for kernel pins interface */
	info->pin_config = devm_kcalloc(dev, info->n_pins,
				        sizeof(*info->pin_config), GFP_KERNEL);
	if (!info->pin_config) {
		dev_err(dev, "Failed to allocate pin_config for E810-T SMA pins\n");
		ice_ptp_disable_sma_pins_e810t(pf, info);
		return;
	}

	/* Read current SMA status */
	err = ice_get_sma_config_e810t(pf, info->pin_config);
	if (err)
		ice_ptp_disable_sma_pins_e810t(pf, info);
}

/**
 * ice_ptp_setup_pins_e810 - Setup PTP pins in sysfs
 * @pf: pointer to the PF instance
 * @info: PTP clock capabilities
 */
static void
ice_ptp_setup_pins_e810(struct ice_pf *pf, struct ptp_clock_info *info)
{
	u16 sdp_entries[ICE_AQC_NVM_SDP_CFG_MAX_SECTION_SIZE];
	u8 nvm_entries, pin_config_num;
	struct device *dev = ice_pf_to_dev(pf);
	struct ice_ptp *ptp = &pf->ptp;
	int err;

	err = ice_ptp_read_sdp_section_from_nvm(&pf->hw,
						&ptp->sdp_section_exist,
						&ptp->ice_pin_desc_num,
						&pin_config_num, sdp_entries,
						&nvm_entries);
	if (err) {
		dev_err(dev, "Failed to read SDP connection section\n");
		return;
	}

	if (ptp->sdp_section_exist) {
		err = ice_update_ptp_pin_from_nvm(pf, sdp_entries, nvm_entries);
		if (err) {
			dev_err(dev, "Failed to read SDP connection section\n");
			return;
		}
	}

	if (ice_is_feature_supported(pf, ICE_F_SMA_CTRL)) {
		info->n_ext_ts = N_EXT_TS_E810;
		info->n_per_out = N_PER_OUT_E810T;
		info->verify = ice_verify_pin_e810t;

		/* Complete setup of the SMA pins */
		ice_ptp_setup_sma_pins_e810t(pf, info, pin_config_num);
	} else if (ice_is_e810t(&pf->hw)) {
		info->n_ext_ts = N_EXT_TS_NO_SMA_E810T;
		info->n_per_out = N_PER_OUT_NO_SMA_E810T;
	} else {
		info->n_per_out = N_PER_OUT_E810;
		info->n_ext_ts = N_EXT_TS_E810;
	}
}

/**
 * ice_ptp_setup_pin_desc_e82x - setup ptp_pin_desc structure
 * @pf: Board private structure
 * @info: PTP info to fill
 */
static void
ice_ptp_setup_pin_desc_e82x(struct ice_pf *pf, struct ptp_clock_info *info)
{
	struct ptp_pin_desc *pins;
	int i;

	/* Allocate memory for kernel pins interface */
	info->pin_config = devm_kcalloc(ice_pf_to_dev(pf), info->n_pins,
					sizeof(*info->pin_config), GFP_KERNEL);
	if (!info->pin_config) {
		dev_err(ice_pf_to_dev(pf), "Failed to allocate pin_config for E82X pins\n");
		return;
	}

	pins = info->pin_config;
	for (i = 0; i < info->n_pins; i++) {
		strscpy(pins[i].name, ice_pin_desc_e82x[i].name,
			sizeof(pins[i].name));
		pins[i].index = i;
		pins[i].func = ice_pin_desc_e82x[i].func;
		pins[i].chan = ice_pin_desc_e82x[i].chan;
	}
}

/**
 * ice_ptp_set_funcs_e82x - Set specialized functions for E82x support
 * @pf: Board private structure
 * @info: PTP info to fill
 *
 * Assign functions to the PTP capabiltiies structure for E82x devices.
 * Functions which operate across all device families should be set directly
 * in ice_ptp_set_caps. Only add functions here which are distinct for E82x
 * devices.
 */
static void
ice_ptp_set_funcs_e82x(struct ice_pf *pf, struct ptp_clock_info *info)
{
#ifdef HAVE_PTP_CROSSTIMESTAMP
	if (boot_cpu_has(X86_FEATURE_ART) &&
	    boot_cpu_has(X86_FEATURE_TSC_KNOWN_FREQ))
		info->getcrosststamp = ice_ptp_getcrosststamp_e82x;
#endif /* HAVE_PTP_CROSSTIMESTAMP */
	info->enable = ice_ptp_gpio_enable_e82x;
	info->verify = ice_verify_pin_e82x;
	if (ice_is_e825c(&pf->hw))
		info->n_per_out = pf->hw.func_caps.ts_func_info.gpio_1pps;
	else
		info->n_per_out = 1;
	info->n_ext_ts = 1;
	info->n_pins = 2;

	ice_ptp_setup_pin_desc_e82x(pf, info);
}

/**
 * ice_ptp_set_funcs_e810 - Set specialized functions for E810 support
 * @pf: Board private structure
 * @info: PTP info to fill
 *
 * Assign functions to the PTP capabiltiies structure for E810 devices.
 * Functions which operate across all device families should be set directly
 * in ice_ptp_set_caps. Only add functions here which are distinct for e810
 * devices.
 */
static void
ice_ptp_set_funcs_e810(struct ice_pf *pf, struct ptp_clock_info *info)
{
	info->enable = ice_ptp_gpio_enable_e810;
	ice_ptp_setup_pins_e810(pf, info);
}

/**
 * ice_ptp_set_caps - Set PTP capabilities
 * @pf: Board private structure
 */
static void ice_ptp_set_caps(struct ice_pf *pf)
{
	struct ptp_clock_info *info = &pf->ptp.info;
	struct device *dev = ice_pf_to_dev(pf);

	snprintf(info->name, sizeof(info->name) - 1, "%s-%s-clk",
		 dev_driver_string(dev), dev_name(dev));
	info->owner = THIS_MODULE;
	info->max_adj = 100000000;
	info->adjtime = ice_ptp_adjtime;
#ifdef HAVE_PTP_CLOCK_INFO_ADJFINE
	info->adjfine = ice_ptp_adjfine;
#else
	info->adjfreq = ice_ptp_adjfreq;
#endif
#if defined(HAVE_PTP_CLOCK_INFO_GETTIMEX64)
	info->gettimex64 = ice_ptp_gettimex64;
#elif defined(HAVE_PTP_CLOCK_INFO_GETTIME64)
	info->gettime64 = ice_ptp_gettime64;
#else
	info->gettime = ice_ptp_gettime32;
#endif
#ifdef HAVE_PTP_CLOCK_INFO_GETTIME64
	info->settime64 = ice_ptp_settime64;
#else
	info->settime = ice_ptp_settime32;
#endif /* HAVE_PTP_CLOCK_INFO_GETTIME64 */

	if (ice_is_e810(&pf->hw))
		ice_ptp_set_funcs_e810(pf, info);
	else
		ice_ptp_set_funcs_e82x(pf, info);
}

/**
 * ice_ptp_create_clock - Create PTP clock device for userspace
 * @pf: Board private structure
 *
 * This function creates a new PTP clock device. It only creates one if we
 * don't already have one. Will return error if it can't create one, but success
 * if we already have a device. Should be used by ice_ptp_init to create clock
 * initially, and prevent global resets from creating new clock devices.
 */
static long ice_ptp_create_clock(struct ice_pf *pf)
{
	struct ptp_clock_info *info;
	struct device *dev;

	/* No need to create a clock device if we already have one */
	if (pf->ptp.clock)
		return 0;

	ice_ptp_set_caps(pf);

	info = &pf->ptp.info;
	dev = ice_pf_to_dev(pf);

	/* Attempt to register the clock before enabling the hardware. */
	pf->ptp.clock = ptp_clock_register(info, dev);
	if (IS_ERR(pf->ptp.clock)) {
		ice_dev_err_errno(dev, PTR_ERR(pf->ptp.clock),
				  "Failed to register PTP clock device");
		return PTR_ERR(pf->ptp.clock);
	}

	return 0;
}

/**
 * ice_ptp_request_ts - Request an available Tx timestamp index
 * @tx: the PTP Tx timestamp tracker to request from
 * @skb: the SKB to associate with this timestamp request
 */
s8 ice_ptp_request_ts(struct ice_ptp_tx *tx, struct sk_buff *skb)
{
	struct ice_ptp_port *ptp_port;
	unsigned long flags;
	struct ice_pf *pf;
	s8 phy_idx;
	u8 idx;

	ptp_port = container_of(tx, struct ice_ptp_port, tx);
	pf = ptp_port_to_pf(ptp_port);

	spin_lock_irqsave(&tx->lock, flags);

	/* Check that this tracker is accepting new timestamp requests */
	if (!ice_ptp_is_tx_tracker_up(tx)) {
		ice_trace(tx_tstamp_tracker_down, ice_pf_to_dev(pf), tx,
			  skb, -1);
		spin_unlock_irqrestore(&tx->lock, flags);
		return -1;
	}

	/* Find and set the first available index */
	idx = find_next_zero_bit(tx->in_use, tx->len,
				 tx->last_ll_ts_idx_read + 1);
	if (idx == tx->len)
		idx = find_first_zero_bit(tx->in_use, tx->len);
	if (idx < tx->len) {
		/* We got a valid index that no other thread could have set. Store
		 * a reference to the skb and the start time to allow discarding old
		 * requests.
		 */
		set_bit(idx, tx->in_use);
		tx->tstamps[idx].start = jiffies;
		tx->tstamps[idx].skb = skb_get(skb);
		skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
		phy_idx = idx + tx->offset;

		ice_trace(tx_tstamp_request, ice_pf_to_dev(pf), tx, skb, idx);
	} else {
		/* No indexes were available, so we have to skip this
		 * timestamp.
		 */
		pf->ptp.tx_hwtstamp_skipped++;
		phy_idx = -1;

		ice_trace(tx_tstamp_idx_full, ice_pf_to_dev(pf), tx, skb, -1);
	}

	spin_unlock_irqrestore(&tx->lock, flags);

	return phy_idx;
}

/**
 * ice_ptp_process_ts - Process the PTP Tx timestamps
 * @pf: Board private structure
 *
 * Returns: ICE_TX_TSTAMP_WORK_PENDING if there are any outstanding Tx
 * timestamps that need processing, and ICE_TX_TSTAMP_WORK_DONE otherwise.
 */
enum ice_tx_tstamp_work ice_ptp_process_ts(struct ice_pf *pf)
{
	switch (pf->ptp.tx_interrupt_mode) {
	case ICE_PTP_TX_INTERRUPT_NONE:
		/* This device has the clock owner handle timestamps for it */
		return ICE_TX_TSTAMP_WORK_DONE;
	case ICE_PTP_TX_INTERRUPT_SELF:
		/* This device handles its own timestamps */
		return ice_ptp_tx_tstamp(&pf->ptp.port.tx);
	case ICE_PTP_TX_INTERRUPT_ALL:
		/* This device handles timestamps for all ports */
		return ice_ptp_tx_tstamp_owner(pf);
	default:
		WARN_ONCE(1, "Unexpected Tx timestamp interrupt mode %u\n",
			  pf->ptp.tx_interrupt_mode);
		return ICE_TX_TSTAMP_WORK_DONE;
	}
}

/**
 * ice_dpll_pin_idx_to_name - Return pin name for a corresponding pin
 *
 * @pf: pointer to the PF instance
 * @pin: pin number to get name for
 * @pin_name: pointer to pin name buffer
 *
 * A wrapper for device-specific pin index to name converters that take care
 * of mapping pin indices returned by a netlist to real pin names
 */
void ice_dpll_pin_idx_to_name(struct ice_pf *pf, u8 pin, char *pin_name)
{
	/* if we are on a custom board, print generic descriptions */
	if (!ice_is_feature_supported(pf, ICE_F_SMA_CTRL)) {
		snprintf(pin_name, MAX_PIN_NAME, "Pin %i", pin);
		return;
	}
	snprintf(pin_name, MAX_PIN_NAME, "%s",
		 ice_cgu_get_pin_name(&pf->hw, pin, true));
}

static void ice_handle_cgu_state(struct ice_pf *pf)
{
	const s64 HUNDREDTH_PSEC = 100LL;
	enum dpll_lock_status state;
	char pin_name[MAX_PIN_NAME];
	int ret;

	ret = ice_get_cgu_state(&pf->hw, ICE_CGU_DPLL_SYNCE,
				pf->synce_dpll_state, &pf->synce_ref_pin,
				NULL, NULL, NULL, &state);
	if (ret) {
		dev_err(ice_pf_to_dev(pf), "Failed to get cgu state (%u:%s)",
			ret, ice_aq_str(pf->hw.adminq.sq_last_status));
		return;
	}
	ice_dpll_pin_idx_to_name(pf, pf->synce_ref_pin, pin_name);
	if (pf->synce_dpll_state != state) {
		pf->synce_dpll_state = state;
		dev_warn(ice_pf_to_dev(pf),
			 "DPLL%i state changed to: %s, pin %s",
			 ICE_CGU_DPLL_SYNCE,
			 ice_cgu_state_to_name(pf->synce_dpll_state), pin_name);
	}

	ret = ice_get_cgu_state(&pf->hw, ICE_CGU_DPLL_PTP, pf->ptp_dpll_state,
				&pf->ptp_ref_pin, NULL, NULL,
				&pf->ptp_dpll_phase_offset, &state);
	ice_dpll_pin_idx_to_name(pf, pf->ptp_ref_pin, pin_name);
	/* offset is in 0.01ps, convert to ps */
	pf->ptp_dpll_phase_offset /= HUNDREDTH_PSEC;
	if (pf->ptp_dpll_state != state) {
		pf->ptp_dpll_state = state;
		dev_warn(ice_pf_to_dev(pf),
			 "DPLL%i state changed to: %s, pin %s",
			 ICE_CGU_DPLL_PTP,
			 ice_cgu_state_to_name(pf->ptp_dpll_state), pin_name);
	}
}

/**
 * ice_ptp_maybe_trigger_tx_interrupt - Trigger Tx timstamp interrupt
 * @pf: Board private structure
 *
 * The device PHY issues Tx timestamp interrupts to the driver for processing
 * timestamp data from the PHY. It will not interrupt again until all
 * current timestamp data is read. In rare circumstances, it is possible that
 * the driver fails to read all outstanding data.
 *
 * To avoid getting permanently stuck, periodically check if the PHY has
 * outstanding timestamp data. If so, trigger an interrupt from software to
 * process this data.
 */
static void ice_ptp_maybe_trigger_tx_interrupt(struct ice_pf *pf)
{
	struct device *dev = ice_pf_to_dev(pf);
	struct ice_hw *hw = &pf->hw;
	unsigned int i;

	for (i = 0; i < ICE_GET_QUAD_NUM(hw->ptp.num_lports); i++) {
		u64 tstamp_ready;
		int err;

		err = ice_get_phy_tx_tstamp_ready(&pf->hw, i, &tstamp_ready);
		if (err || tstamp_ready) {
			/* Trigger a software interrupt, to ensure this data
			 * gets processed.
			 */
			dev_dbg(dev, "PTP periodic task detected waiting timestamps. Triggering Tx timestamp interrupt now.\n");

			wr32(hw, PFINT_OICR, PFINT_OICR_TSYN_TX_M);
			ice_flush(hw);

			return;
		}
	}
}

#define TS_PLL_LOS_LOG_CAD 120	/* log every two minutes */

static void ice_ptp_periodic_work(struct kthread_work *work)
{
	struct ice_ptp_port_owner *po = container_of(work,
						     struct ice_ptp_port_owner,
						     work.work);
	struct ice_ptp *ptp = container_of(po, struct ice_ptp, ports_owner);
	struct ice_pf *pf = container_of(ptp, struct ice_pf, ptp);
	struct ice_hw *hw = &pf->hw;
	bool retry = false;
	int err;

	if (test_bit(ICE_FLAG_PTP_WT_BLOCKED, pf->flags))
		return;

	err = ice_ptp_update_cached_phctime_all(pf);
	if (err)
		retry = true;

	if (hw->ptp.phy_model == ICE_PHY_E82X)
		ice_ptp_maybe_trigger_tx_interrupt(pf);

	if (ice_is_feature_supported(pf, ICE_F_CGU))
		if (test_bit(ICE_FLAG_DPLL_MONITOR, pf->flags))
			ice_handle_cgu_state(pf);

	/* only E825C should monitor the TS PLL */
	if (ice_is_e825c(hw)) {
		bool lock_lost;

		err = ice_cgu_ts_pll_lost_lock_e825c(hw, &lock_lost);
		if (err) {
			retry = true;
			dev_err(ice_pf_to_dev(pf),
				"Failed reading TimeSync PLL lock status. Retrying.\n");
		} else if (lock_lost) {
			if (hw->ptp.ts_pll_lock_retries % TS_PLL_LOS_LOG_CAD)
				dev_err(ice_pf_to_dev(pf), "TimeSync PLL lock lost. Retrying to acquire lock with current PLL configuration\n");

			err = ice_cgu_ts_pll_restart_e825c(hw);
			if (err) {
				retry = true;
				dev_err(ice_pf_to_dev(pf), "Failed reading TimeSync PLL lock status. Retrying.\n");
			}

			hw->ptp.ts_pll_lock_retries++;
		} else {
			if (hw->ptp.ts_pll_lock_retries)
				dev_err(ice_pf_to_dev(pf), "TimeSync PLL lock is acquired with %s clock source.\n",
					ice_clk_src_str(hw->ptp.clk_src));

			hw->ptp.ts_pll_lock_retries = 0;
		}
	}

	/* Run twice a second or reschedule soon if we need to retry */
	ice_ptp_schedule_periodic_work(ptp, msecs_to_jiffies(retry ? 10 : 500));
}

/**
 * ice_ptp_prepare_rebuild_sec - Prepare second NAC for PTP reset or rebuild
 * @pf: Board private structure
 * @rebuild: rebuild if true, prepare if false
 * @reset_type: the reset type being performed
 */
static void ice_ptp_prepare_rebuild_sec(struct ice_pf *pf, bool rebuild,
					enum ice_reset_req reset_type)
{
	struct ice_ptp_port *port;

	mutex_lock(&pf->ptp.ports_owner.lock);
	if (!list_initialized(pf->ptp.ports_owner.ports)) {
		mutex_unlock(&pf->ptp.ports_owner.lock);
		return;
	}
	list_for_each_entry(port, &pf->ptp.ports_owner.ports, list_member) {
		struct ice_pf *peer_pf = ptp_port_to_pf(port);

		if (!peer_pf->hw.ptp.primary_nac) {
			if (rebuild)
				ice_ptp_rebuild(peer_pf, reset_type);
			else
				ice_ptp_prepare_for_reset(peer_pf, reset_type);
		}
	}
	mutex_unlock(&pf->ptp.ports_owner.lock);
}

/**
 * ice_ptp_prepare_for_reset - Prepare PTP for reset
 * @pf: Board private structure
 * @reset_type: the reset type being performed
 */
void ice_ptp_prepare_for_reset(struct ice_pf *pf, enum ice_reset_req reset_type)
{
	struct ice_ptp *ptp = &pf->ptp;
	struct ice_hw *hw = &pf->hw;
	u8 src_tmr;

	if (ptp->state != ICE_PTP_READY)
		return;

	ptp->state = ICE_PTP_RESETTING;

	/* Disable timestamping for both Tx and Rx */
	ice_ptp_disable_timestamp_mode(pf);

	ice_ptp_cancel_periodic_work(&pf->ptp);

	if (reset_type == ICE_RESET_PFR)
		return;

	if (ice_pf_src_tmr_owned(pf) && ice_is_e825c(hw))
		ice_ptp_prepare_rebuild_sec(pf, false, reset_type);

	kthread_cancel_delayed_work_sync(&pf->ptp.port.ov_work);

	/* Disable periodic outputs */
	ice_ptp_disable_all_clkout(pf);

	src_tmr = ice_get_ptp_src_clock_index(hw);

	/* Disable source clock */
	wr32(hw, GLTSYN_ENA(src_tmr), (u32)~GLTSYN_ENA_TSYN_ENA_M);

	/* Acquire PHC and system timer to restore after reset */
	ptp->reset_time = ktime_get_real_ns();
}

/**
 * ice_ptp_rebuild_owner - Initialize PTP clock owner after reset
 * @pf: Board private structure
 *
 * Companion function for ice_ptp_rebuild() which handles tasks that only the
 * PTP clock owner instance should perform.
 */
static int ice_ptp_rebuild_owner(struct ice_pf *pf)
{
	struct device *dev = ice_pf_to_dev(pf);
	struct ice_ptp *ptp = &pf->ptp;
	struct ice_hw *hw = &pf->hw;
	struct timespec64 ts;
	u64 time_diff;
	int err;

	err = ice_ptp_init_phc(hw);
	if (err) {
		dev_err(dev, "Failed to initialize PHC, status %d\n",
			err);
		return err;
	}

	/* Acquire the global hardware lock */
	if (!ice_ptp_lock(hw)) {
		err = -EBUSY;
		dev_err(dev, "Failed to acquire PTP hardware semaphore\n");
		return err;
	}

	/* Write the increment time value to PHY and LAN */
	err = ice_ptp_write_incval(hw, ice_base_incval(pf));
	if (err) {
		dev_err(dev, "Failed to write PHC increment value, status %d\n",
			err);
		ice_ptp_unlock(hw);
		return err;
	}

	/* Write the initial Time value to PHY and LAN using the cached PHC
	 * time before the reset and time difference between stopping and
	 * starting the clock.
	 */
	if (ptp->cached_phc_time) {
		time_diff = ktime_get_real_ns() - ptp->reset_time;
		ts = ns_to_timespec64(ptp->cached_phc_time + time_diff);
	} else {
		ts = ktime_to_timespec64(ktime_get_real());
	}

	err = ice_ptp_write_init(pf, &ts);
	if (err) {
		ice_dev_err_errno(dev, err, "Failed to write PHC initial time");
		ice_ptp_unlock(hw);
		return err;
	}

	/* Release the global hardware lock */
	ice_ptp_unlock(hw);

	/* Flush software tracking of any outstanding timestamps since we're
	 * about to flush the PHY timestamp block.
	 */
	ice_ptp_flush_all_tx_tracker(pf);

	/* Configure PHY interrupt settings, and reset timestamp block */
	err = ice_ptp_cfg_phy_interrupt(pf, true, 1);
	if (err)
		return err;

	if (!ice_is_e810(hw))
		ice_ptp_restart_all_phy(pf);

	/* Re-enable all periodic outputs and external timestamp events */
	ice_ptp_enable_all_clkout(pf);
	ice_ptp_enable_all_extts(pf);

	return 0;
}

/**
 * ice_ptp_rebuild - Initialize PTP hardware clock support after reset
 * @pf: Board private structure
 * @reset_type: the reset type being performed
 */
void ice_ptp_rebuild(struct ice_pf *pf, enum ice_reset_req reset_type)
{
	struct ice_ptp *ptp = &pf->ptp;
	int err;

	if (ptp->state != ICE_PTP_RESETTING) {
		if (ptp->state == ICE_PTP_READY) {
			ice_ptp_prepare_for_reset(pf, reset_type);
		} else {
			err = -EINVAL;
			dev_err(ice_pf_to_dev(pf), "PTP was not initialized\n");
			goto err;
		}
	}

	if (ice_pf_src_tmr_owned(pf)) {
		if (reset_type != ICE_RESET_PFR) {
			err = ice_ptp_rebuild_owner(pf);
			if (err)
				goto err;

			if (ice_is_e825c(&pf->hw))
				ice_ptp_prepare_rebuild_sec(pf, true,
							    reset_type);

			ice_block_ptp_workthreads_global(pf, false);
		}

		/* Start periodic work going */
		ice_ptp_schedule_periodic_work(ptp, 0);
	}

	ptp->state = ICE_PTP_READY;

	dev_info(ice_pf_to_dev(pf), "PTP reset successful\n");
	return;
err:
	ptp->state = ICE_PTP_ERROR;
	ice_dev_err_errno(ice_pf_to_dev(pf), err, "PTP reset failed");
}

/**
 * ice_ptp_aux_dev_to_aux_pf - Get PF handle for the auxiliary device
 * @aux_dev: auxiliary device to get the PF for
 */
static struct ice_pf *
ice_ptp_aux_dev_to_aux_pf(struct auxiliary_device *aux_dev)
{
	struct ice_ptp_port *aux_port;
	struct ice_ptp *aux_ptp;

	aux_port = container_of(aux_dev, struct ice_ptp_port, aux_dev);
	aux_ptp = container_of(aux_port, struct ice_ptp, port);
	return container_of(aux_ptp, struct ice_pf, ptp);
}

/**
 * ice_ptp_aux_dev_to_owner_pf - Get owner PF handle for the auxiliary device
 * @aux_dev: auxiliary device to get the owner PF for
 */
static struct ice_pf *
ice_ptp_aux_dev_to_owner_pf(struct auxiliary_device *aux_dev)
{
	struct ice_ptp_port_owner *ports_owner;
	struct auxiliary_driver *aux_drv;
	struct ice_ptp *owner_ptp;

	if (!aux_dev->dev.driver)
		return NULL;

	aux_drv = to_auxiliary_drv(aux_dev->dev.driver);
	ports_owner = container_of(aux_drv, struct ice_ptp_port_owner,
				   aux_driver);
	owner_ptp = container_of(ports_owner, struct ice_ptp, ports_owner);
	return container_of(owner_ptp, struct ice_pf, ptp);
}

/**
 * ice_ptp_auxbus_probe - Probe auxiliary devices
 * @aux_dev: PF's auxiliary device
 * @id: Auxiliary device ID
 */
static int ice_ptp_auxbus_probe(struct auxiliary_device *aux_dev,
				const struct auxiliary_device_id *id)
{
	struct ice_pf *owner_pf = ice_ptp_aux_dev_to_owner_pf(aux_dev);
	struct ice_pf *aux_pf = ice_ptp_aux_dev_to_aux_pf(aux_dev);
	u8 port_num;
	u8 phy;

	if (WARN_ON(!owner_pf) || aux_pf->hw.ptp.phy_model == ICE_PHY_UNSUP)
		return -ENODEV;

	INIT_LIST_HEAD(&aux_pf->ptp.port.list_member);
	mutex_lock(&owner_pf->ptp.ports_owner.lock);
	list_add(&aux_pf->ptp.port.list_member,
		 &owner_pf->ptp.ports_owner.ports);
	mutex_unlock(&owner_pf->ptp.ports_owner.lock);

	port_num = aux_pf->ptp.port.port_num;
	phy = port_num / aux_pf->hw.ptp.ports_per_phy;
	/* clock ENET is enabled by default, mark it */
	set_bit(port_num,
		&owner_pf->ptp.ports_owner.tx_refclks[phy][ICE_REF_CLK_ENET]);
	if (!aux_pf->hw.ptp.primary_nac)
		aux_pf->hw.ptp.primary_hw = &owner_pf->hw;

	return 0;
}

/**
 * ice_ptp_auxbus_remove - Remove auxiliary devices from the bus
 * @aux_dev: PF's auxiliary device
 */
#ifdef HAVE_AUXILIARY_DRIVER_INT_REMOVE
static int ice_ptp_auxbus_remove(struct auxiliary_device *aux_dev)
#else
static void ice_ptp_auxbus_remove(struct auxiliary_device *aux_dev)
#endif
{
	struct ice_pf *owner_pf = ice_ptp_aux_dev_to_owner_pf(aux_dev);
	struct ice_pf *aux_pf = ice_ptp_aux_dev_to_aux_pf(aux_dev);

	if (aux_pf->hw.ptp.phy_model != ICE_PHY_UNSUP) {
	/* revert TX clocks back to default ENET */
		ice_ptp_change_tx_clk(aux_pf, ICE_REF_CLK_ENET);
		mutex_lock(&owner_pf->ptp.ports_owner.lock);
		list_del(&aux_pf->ptp.port.list_member);
		mutex_unlock(&owner_pf->ptp.ports_owner.lock);
	}
#ifdef HAVE_AUXILIARY_DRIVER_INT_REMOVE
	return 0;
#endif
}

/**
 * ice_ptp_auxbus_shutdown
 * @aux_dev: PF's auxiliary device
 */
static void ice_ptp_auxbus_shutdown(struct auxiliary_device *aux_dev)
{
	/* Doing nothing here, but handle to auxbus driver must be satisfied */
}

/**
 * ice_ptp_auxbus_suspend
 * @aux_dev: PF's auxiliary device
 * @state: power management state indicator
 */
static int
ice_ptp_auxbus_suspend(struct auxiliary_device *aux_dev, pm_message_t state)
{
	/* Doing nothing here, but handle to auxbus driver must be satisfied */
	return 0;
}

/**
 * ice_ptp_auxbus_resume
 * @aux_dev: PF's auxiliary device
 */
static int ice_ptp_auxbus_resume(struct auxiliary_device *aux_dev)
{
	/* Doing nothing here, but handle to auxbus driver must be satisfied */
	return 0;
}

/**
 * ice_ptp_auxbus_create_id_table - Create auxiliary device ID table
 * @pf: Board private structure
 * @name: auxiliary bus driver name
 */
static struct auxiliary_device_id *
ice_ptp_auxbus_create_id_table(struct ice_pf *pf, const char *name)
{
	struct auxiliary_device_id *ids;

	/* Second id left empty to terminate the array */
	ids = devm_kcalloc(ice_pf_to_dev(pf), 2,
			   sizeof(struct auxiliary_device_id), GFP_KERNEL);
	if (!ids)
		return NULL;

	snprintf(ids[0].name, sizeof(ids[0].name), "ice.%s", name);

	return ids;
}

/**
 * ice_ptp_register_auxbus_driver - Register PTP auxiliary bus driver
 * @pf: Board private structure
 */
static int ice_ptp_register_auxbus_driver(struct ice_pf *pf)
{
	struct auxiliary_driver *aux_driver;
	struct ice_ptp *ptp;
	char busdev[8] = {};
	struct device *dev;
	char *name;
	int err;

	ptp = &pf->ptp;
	dev = ice_pf_to_dev(pf);
	aux_driver = &ptp->ports_owner.aux_driver;
	INIT_LIST_HEAD(&ptp->ports_owner.ports);
	mutex_init(&ptp->ports_owner.lock);

	if (ice_is_e810(&pf->hw))
		sprintf(busdev, "%u_%u_", pf->pdev->bus->number,
			PCI_SLOT(pf->pdev->devfn));

	name = devm_kasprintf(dev, GFP_KERNEL, "ptp_%sclk%u", busdev,
			      ice_get_ptp_src_clock_index(&pf->hw));

	aux_driver->name = name;
	aux_driver->shutdown = ice_ptp_auxbus_shutdown;
	aux_driver->suspend = ice_ptp_auxbus_suspend;
	aux_driver->remove = ice_ptp_auxbus_remove;
	aux_driver->resume = ice_ptp_auxbus_resume;
	aux_driver->probe = ice_ptp_auxbus_probe;
	aux_driver->id_table = ice_ptp_auxbus_create_id_table(pf, name);
	if (!aux_driver->id_table)
		return -ENOMEM;

	err = auxiliary_driver_register(aux_driver);
	if (err) {
		devm_kfree(dev, aux_driver->id_table);
		dev_err(dev, "Failed registering aux_driver, name <%s>\n",
			name);
	}

	return err;
}

/**
 * ice_ptp_unregister_auxbus_driver - Unregister PTP auxiliary bus driver
 * @pf: Board private structure
 */
static void ice_ptp_unregister_auxbus_driver(struct ice_pf *pf)
{
	struct auxiliary_driver *aux_driver = &pf->ptp.ports_owner.aux_driver;

	auxiliary_driver_unregister(aux_driver);
	devm_kfree(ice_pf_to_dev(pf), (void *)aux_driver->id_table);

	mutex_destroy(&pf->ptp.ports_owner.lock);
}

/**
 * ice_ptp_clock_index - Get the PTP clock index for this device
 * @pf: Board private structure
 *
 * Returns: the PTP clock index associated with this PF, or -1 if no PTP clock
 * is associated.
 */
int ice_ptp_clock_index(struct ice_pf *pf)
{
	struct auxiliary_device *aux_dev;
	struct ice_pf *owner_pf;
	struct ptp_clock *clock;

	aux_dev = &pf->ptp.port.aux_dev;
	owner_pf = ice_ptp_aux_dev_to_owner_pf(aux_dev);
	if (!owner_pf)
		return -1;
	clock = owner_pf->ptp.clock;

	return clock ? ptp_clock_index(clock) : -1;
}

/**
 * ice_ptp_init_owner - Initialize PTP_1588_CLOCK device
 * @pf: Board private structure
 *
 * Setup and initialize a PTP clock device that represents the device hardware
 * clock. Save the clock index for other functions connected to the same
 * hardware resource.
 */
static int ice_ptp_init_owner(struct ice_pf *pf)
{
	struct device *dev = ice_pf_to_dev(pf);
	struct ice_ptp *ptp = &pf->ptp;
	struct ice_hw *hw = &pf->hw;
	struct timespec64 ts;
	int err, itr = 1;

	hw->ptp.src_tmr_mode = ICE_SRC_TMR_MODE_NANOSECONDS;
	hw->ptp.time_ref_freq = hw->func_caps.ts_func_info.time_ref;
	hw->ptp.clk_src = (enum ice_clk_src)hw->func_caps.ts_func_info.clk_src;
	hw->ptp.ts_pll_lock_retries = 0;

	err = ice_ptp_init_phc(hw);
	if (err) {
		dev_err(dev, "Failed to initialize PHC, status %d\n", err);
		goto err_exit;
	}

	/* Acquire the global hardware lock */
	if (!ice_ptp_lock(hw)) {
		err = -EBUSY;
		dev_err(dev, "Failed to acquire PTP hardware semaphore\n");
		goto err_exit;
	}

	/* Write the increment time value to PHY and LAN */
	err = ice_ptp_write_incval(hw, ice_base_incval(pf));
	if (err) {
		dev_err(dev, "Failed to write PHC increment value, status %d\n",
			err);
		ice_ptp_unlock(hw);
		goto err_exit;
	}

	ts = ktime_to_timespec64(ktime_get_real());
	/* Write the initial Time value to PHY and LAN */
	err = ice_ptp_write_init(pf, &ts);
	if (err) {
		ice_dev_err_errno(dev, err, "Failed to write PHC initial time");
		ice_ptp_unlock(hw);
		goto err_exit;
	}

	/* Release the global hardware lock */
	ice_ptp_unlock(hw);

	/* Configure PHY interrupt settings */
	err = ice_ptp_cfg_phy_interrupt(pf, true, itr);
	if (err)
		goto err_exit;

	/* Ensure we have a clock device */
	err = ice_ptp_create_clock(pf);
	if (err) {
		ice_dev_err_errno(dev, err,
				  "Failed to register PTP clock device");
		goto err_clk;
	}

	if (ice_is_feature_supported(pf, ICE_F_CGU)) {
		set_bit(ICE_FLAG_DPLL_MONITOR, pf->flags);
		pf->synce_dpll_state = DPLL_LOCK_STATUS_UNLOCKED;
		pf->ptp_dpll_state = DPLL_LOCK_STATUS_UNLOCKED;
	}
	clear_bit(ICE_FLAG_ITU_G8262_FILTER_USED, pf->flags);

	err = ice_ptp_register_auxbus_driver(pf);
	if (err) {
		ice_dev_err_errno(dev, err, "Failed to register PTP auxbus driver");
		goto err_aux;
	}

	return 0;

err_aux:
	ptp_clock_unregister(pf->ptp.clock);
err_clk:
	ptp->clock = NULL;
err_exit:
	return err;
}

/**
 * ice_ptp_init_work - Initialize PTP work threads
 * @pf: Board private structure
 * @ptp: PF PTP structure
 */
static int ice_ptp_init_work(struct ice_pf *pf, struct ice_ptp *ptp)
{
	struct kthread_worker *kworker;

	/* Initialize work functions */
	if (ice_pf_src_tmr_owned(pf))
		kthread_init_delayed_work(&ptp->ports_owner.work,
					  ice_ptp_periodic_work);

	/* Allocate a kworker for handling work required for the ports
	 * connected to the PTP hardware clock.
	 */
	kworker = kthread_create_worker(0, "ice-ptp-%s",
					dev_name(ice_pf_to_dev(pf)));
	if (IS_ERR(kworker))
		return PTR_ERR(kworker);

	ptp->ports_owner.kworker = kworker;

	/* Start periodic work going */
	ice_ptp_schedule_periodic_work(ptp, 0);

	return 0;
}

/**
 * ice_ptp_init_port - Initialize PTP port structure
 * @pf: Board private structure
 * @ptp_port: PTP port structure
 */
static int ice_ptp_init_port(struct ice_pf *pf, struct ice_ptp_port *ptp_port)
{
	struct ice_hw *hw = &pf->hw;

	mutex_init(&ptp_port->ps_lock);

	switch (hw->ptp.phy_model) {
	case ICE_PHY_ETH56G:
		return ice_ptp_init_tx_eth56g(pf, &ptp_port->tx,
					      ptp_port->port_num);
	case ICE_PHY_E810:
		return ice_ptp_init_tx_e810(pf, &ptp_port->tx);
	case ICE_PHY_E82X:
		ptp_port->rx_calibrating = 0;
		kthread_init_delayed_work(&ptp_port->ov_work,
					  ice_ptp_wait_for_offsets);
		return ice_ptp_init_tx_e82x(pf, &ptp_port->tx,
					    ptp_port->port_num);
	default:
		return -ENODEV;
	}
}

/**
 * ice_block_ptp_workthreads
 * @pf: driver's pf structure
 * @block_enable: enable / disable WT block
 *
 * Enable or disable ICE_FLAG_PTP_WT_BLOCKED flag for this pf.
 * Additionally, since Rx/Tx timestamps depend on cached PHC value
 * which will be out of date when we block workthreads, stop timestamping
 */
static void ice_block_ptp_workthreads(struct ice_pf *pf, bool block_enable)
{
	struct device *dev = ice_pf_to_dev(pf);

	if (block_enable && !test_and_set_bit(ICE_FLAG_PTP_WT_BLOCKED,
					      pf->flags)) {
		dev_dbg(dev, "PTP workthreads blocked");
		ice_ptp_disable_timestamp_mode(pf);
	} else if (!block_enable) {
		clear_bit(ICE_FLAG_PTP_WT_BLOCKED, pf->flags);
		dev_dbg(dev, "PTP workthreads unblocked");
	}
}

/**
 * ice_block_ptp_workthreads_global
 * @pf: driver's pf structure
 * @block_enable: enable / disable global WT block
 *
 * Enable or disable ICE_FLAG_PTP_WT_BLOCKED flag across all PFs.
 * Operation requires PTP Auxbus.
 */
void ice_block_ptp_workthreads_global(struct ice_pf *pf, bool block_enable)
{
	struct auxiliary_device *aux_dev;
	struct ice_ptp_port *port;
	struct ice_pf *owner_pf;

	aux_dev = &pf->ptp.port.aux_dev;
	owner_pf = ice_ptp_aux_dev_to_owner_pf(aux_dev);
	if (!owner_pf)
		return;

	if (!ice_is_ptp_supported(owner_pf)) {
		dev_dbg(ice_pf_to_dev(owner_pf), "PTP not active, skipping unnecessary workthread block");
		return;
	}

	mutex_lock(&owner_pf->ptp.ports_owner.lock);
	if (!list_initialized(owner_pf->ptp.ports_owner.ports)) {
		mutex_unlock(&owner_pf->ptp.ports_owner.lock);
		return;
	}
	list_for_each_entry(port, &owner_pf->ptp.ports_owner.ports, list_member) {
		ice_block_ptp_workthreads(ptp_port_to_pf(port), block_enable);
	}
	mutex_unlock(&owner_pf->ptp.ports_owner.lock);
}

/**
 * ice_ptp_release_auxbus_device
 * @dev: device that utilizes the auxbus
 */
static void ice_ptp_release_auxbus_device(struct device *dev)
{
	/* Doing nothing here, but handle to auxbux device must be satisfied */
}

/**
 * ice_ptp_create_auxbus_device - Create PTP auxiliary bus device
 * @pf: Board private structure
 */
static int ice_ptp_create_auxbus_device(struct ice_pf *pf)
{
	struct auxiliary_device *aux_dev;
	struct ice_ptp *ptp;
	char busdev[8] = {};
	struct device *dev;
	char *name;
	int err;
	u32 id;

	ptp = &pf->ptp;
	id = ptp->port.port_num;
	dev = ice_pf_to_dev(pf);

	aux_dev = &ptp->port.aux_dev;

	if (ice_is_e810(&pf->hw))
		sprintf(busdev, "%u_%u_", pf->pdev->bus->number,
			PCI_SLOT(pf->pdev->devfn));

	name = devm_kasprintf(dev, GFP_KERNEL, "ptp_%sclk%u", busdev,
			      ice_get_ptp_src_clock_index(&pf->hw));

	aux_dev->name = name;
	aux_dev->id = id;
	if (ice_is_e825c(&pf->hw) && !pf->hw.ptp.primary_nac)
		aux_dev->id += pf->hw.ptp.ports_per_phy;
	aux_dev->dev.release = ice_ptp_release_auxbus_device;
	aux_dev->dev.parent = dev;

	err = auxiliary_device_init(aux_dev);
	if (err)
		goto aux_err;

	err = auxiliary_device_add(aux_dev);
	if (err) {
		auxiliary_device_uninit(aux_dev);
		goto aux_err;
	}

	return 0;
aux_err:
	dev_err(dev, "Failed to create PTP auxiliary bus device <%s>\n", name);
	devm_kfree(dev, name);
	return err;
}

/**
 * ice_ptp_remove_auxbus_device - Remove PTP auxiliary bus device
 * @pf: Board private structure
 */
static void ice_ptp_remove_auxbus_device(struct ice_pf *pf)
{
	struct auxiliary_device *aux_dev = &pf->ptp.port.aux_dev;

	auxiliary_device_delete(aux_dev);
	auxiliary_device_uninit(aux_dev);

	memset(aux_dev, 0, sizeof(*aux_dev));
}

/**
 * ice_ptp_init_tx_interrupt_mode - Initialize device Tx interrupt mode
 * @pf: Board private structure
 *
 * Initialize the Tx timestamp interrupt mode for this device. For most device
 * types, each PF processes the interrupt and manages its own timestamps. For
 * E82X-based devices, only the clock owner processes the timestamps. Other
 * PFs disable the interrupt and do not process their own timestamps.
 */
static void ice_ptp_init_tx_interrupt_mode(struct ice_pf *pf)
{
	switch (pf->hw.ptp.phy_model) {
	case ICE_PHY_E82X:
		/* E82X based PHY has the clock owner process the interrupt
		 * for all ports.
		 */
		if (ice_pf_src_tmr_owned(pf))
			pf->ptp.tx_interrupt_mode = ICE_PTP_TX_INTERRUPT_ALL;
		else
			pf->ptp.tx_interrupt_mode = ICE_PTP_TX_INTERRUPT_NONE;
		break;
	default:
		/* other PHY types handle their own Tx interrupt */
		pf->ptp.tx_interrupt_mode = ICE_PTP_TX_INTERRUPT_SELF;
	}
}

/**
 * ice_ptp_init - Initialize PTP hardware clock support
 * @pf: Board private structure
 *
 * Set up the device for interacting with the PTP hardware clock for all
 * functions, both the function that owns the clock hardware, and the
 * functions connected to the clock hardware.
 *
 * The clock owner will allocate and register a ptp_clock with the
 * PTP_1588_CLOCK infrastructure. All functions allocate a kthread and work
 * items used for asynchronous work such as Tx timestamps and periodic work.
 */
void ice_ptp_init(struct ice_pf *pf)
{
	struct device *dev = ice_pf_to_dev(pf);
	struct ice_ptp *ptp = &pf->ptp;
	struct ice_hw *hw = &pf->hw;
	int err;

	ptp->state = ICE_PTP_INITIALIZING;

	if (ice_is_e825c(hw) && !ice_is_primary(pf))
		hw->ptp.primary_nac = false;
	else
		hw->ptp.primary_nac = true;

	ice_ptp_init_hw(hw);

	ice_ptp_init_tx_interrupt_mode(pf);

	/* If this function owns the clock hardware, it must allocate and
	 * configure the PTP clock device to represent it.
	 */
	if (ice_pf_src_tmr_owned(pf)) {
		err = ice_ptp_init_owner(pf);
		if (err)
			goto err;
	}

	ptp->port.tx_clk = ICE_REF_CLK_ENET;
	ptp->port.port_num = hw->pf_id;
	err = ice_ptp_create_auxbus_device(pf);
	if (err)
		goto err_auxdrv;

	err = ice_ptp_init_port(pf, &ptp->port);
	if (err)
		goto err_auxdev;

	/* Configure initial Tx interrupt settings */
	ice_ptp_cfg_tx_interrupt(pf);

	ice_ptp_sysfs_init(pf);

	ptp->state = ICE_PTP_READY;

	err = ice_ptp_init_work(pf, ptp);
	if (err)
		goto err_auxdev;

	/* Start the PHY timestamping block */
	ice_ptp_reset_phy_timestamping(pf);

	dev_info(dev, "PTP init successful\n");
	return;

err_auxdev:
	ice_ptp_remove_auxbus_device(pf);
err_auxdrv:
	if (ice_pf_src_tmr_owned(pf))
		ice_ptp_unregister_auxbus_driver(pf);
err:
	ptp->state = ICE_PTP_ERROR;
	/* If we registered a PTP clock, release it */
	if (pf->ptp.clock) {
		ptp_clock_unregister(ptp->clock);
		pf->ptp.clock = NULL;
	}
	ice_dev_err_errno(dev, err, "PTP init failed");
}

/**
 * ice_ptp_release - Disable the driver/HW support and unregister the clock
 * @pf: Board private structure
 *
 * This function handles the cleanup work required from the initialization by
 * clearing out the important information and unregistering the clock
 */
void ice_ptp_release(struct ice_pf *pf)
{
	if (pf->ptp.state != ICE_PTP_READY)
		return;

	pf->ptp.state = ICE_PTP_UNINIT;

	/* Disable timestamping for both Tx and Rx */
	ice_ptp_disable_timestamp_mode(pf);

	ice_ptp_remove_auxbus_device(pf);
	ice_ptp_release_tx_tracker(pf, &pf->ptp.port.tx);

	ice_ptp_disable_all_extts(pf);

	ice_ptp_cancel_periodic_work(&pf->ptp);

	ice_ptp_port_phy_stop(&pf->ptp.port);
	mutex_destroy(&pf->ptp.port.ps_lock);

	if (pf->ptp.ports_owner.kworker) {
		kthread_destroy_worker(pf->ptp.ports_owner.kworker);
		pf->ptp.ports_owner.kworker = NULL;
	}

	ice_ptp_sysfs_release(pf);

	if (ice_pf_src_tmr_owned(pf))
		ice_ptp_unregister_auxbus_driver(pf);

	if (!pf->ptp.clock)
		return;

	/* Disable periodic outputs */
	ice_ptp_disable_all_clkout(pf);
	ptp_clock_unregister(pf->ptp.clock);
	pf->ptp.clock = NULL;

	dev_info(ice_pf_to_dev(pf), "Removed PTP clock\n");
}
