/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2018-2025 Intel Corporation */

#ifndef _ICE_NVM_H_
#define _ICE_NVM_H_

#define ICE_NVM_CMD_READ		0x0000000B
#define ICE_NVM_CMD_WRITE		0x0000000C

/* NVM Access config bits */
#define ICE_NVM_CFG_MODULE_M		ICE_M(0xFF, 0)
#define ICE_NVM_CFG_MODULE_S		0
#define ICE_NVM_CFG_FLAGS_M		ICE_M(0xF, 8)
#define ICE_NVM_CFG_FLAGS_S		8
#define ICE_NVM_CFG_EXT_FLAGS_M		ICE_M(0xF, 12)
#define ICE_NVM_CFG_EXT_FLAGS_S		12
#define ICE_NVM_CFG_ADAPTER_INFO_M	ICE_M(0xFFFF, 16)
#define ICE_NVM_CFG_ADAPTER_INFO_S	16

/* NVM Read Get Driver Features */
#define ICE_NVM_GET_FEATURES_MODULE	0xE
#define ICE_NVM_GET_FEATURES_FLAGS	0xF

/* NVM Read/Write Mapped Space */
#define ICE_NVM_REG_RW_MODULE	0x0
#define ICE_NVM_REG_RW_FLAGS	0x1

struct ice_orom_civd_info {
	u8 signature[4];	/* Must match ASCII '$CIV' characters */
	u8 checksum;		/* Simple modulo 256 sum of all structure bytes must equal 0 */
	__le32 combo_ver;	/* Combo Image Version number */
	u8 combo_name_len;	/* Length of the unicode combo image version string, max of 32 */
	__le16 combo_name[32];	/* Unicode string representing the Combo Image version */
} __packed;

#define ICE_NVM_ACCESS_MAJOR_VER	0
#define ICE_NVM_ACCESS_MINOR_VER	5

/* NVM Access feature flags. Other bits in the features field are reserved and
 * should be set to zero when reporting the ice_nvm_features structure.
 */
#define ICE_NVM_FEATURES_0_REG_ACCESS	BIT(1)

/* NVM Access Features */
struct ice_nvm_features {
	u8 major;		/* Major version (informational only) */
	u8 minor;		/* Minor version (informational only) */
	u16 size;		/* size of ice_nvm_features structure */
	u8 features[12];	/* Array of feature bits */
};

/* NVM Access command */
struct ice_nvm_access_cmd {
	u32 command;		/* NVM command: READ or WRITE */
	u32 config;		/* NVM command configuration */
	u32 offset;		/* offset to read/write, in bytes */
	u32 data_size;		/* size of data field, in bytes */
};

/* NVM Access data */
union ice_nvm_access_data {
	u32 regval;	/* Storage for register value */
	struct ice_nvm_features drv_features; /* NVM features */
};

u32 ice_nvm_access_get_module(struct ice_nvm_access_cmd *cmd);
u32 ice_nvm_access_get_flags(struct ice_nvm_access_cmd *cmd);
u32 ice_nvm_access_get_adapter(struct ice_nvm_access_cmd *cmd);
int
ice_nvm_access_read(struct ice_hw *hw, struct ice_nvm_access_cmd *cmd,
		    union ice_nvm_access_data *data);
int
ice_nvm_access_write(struct ice_hw *hw, struct ice_nvm_access_cmd *cmd,
		     union ice_nvm_access_data *data);
int
ice_nvm_access_get_features(struct ice_nvm_access_cmd *cmd,
			    union ice_nvm_access_data *data);
int
ice_handle_nvm_access(struct ice_hw *hw, struct ice_nvm_access_cmd *cmd,
		      union ice_nvm_access_data *data);

int
ice_acquire_nvm(struct ice_hw *hw, enum ice_aq_res_access_type access);
void ice_release_nvm(struct ice_hw *hw);
int
ice_aq_read_nvm(struct ice_hw *hw, u16 module_typeid, u32 offset, u16 length,
		void *data, bool last_command, bool read_shadow_ram,
		struct ice_sq_cd *cd);
int
ice_read_flat_nvm(struct ice_hw *hw, u32 offset, u32 *length, u8 *data,
		  bool read_shadow_ram);
int
ice_get_pfa_module_tlv(struct ice_hw *hw, u16 *module_tlv, u16 *module_tlv_len,
		       u16 module_type);
int
ice_get_nvm_minsrevs(struct ice_hw *hw, struct ice_minsrev_info *minsrevs);
int
ice_update_nvm_minsrevs(struct ice_hw *hw, struct ice_minsrev_info *minsrevs);
int
ice_get_inactive_orom_ver(struct ice_hw *hw, struct ice_orom_info *orom);
int
ice_get_inactive_nvm_ver(struct ice_hw *hw, struct ice_nvm_info *nvm);
int
ice_get_inactive_netlist_ver(struct ice_hw *hw, struct ice_netlist_info *netlist);
int
ice_read_pba_string(struct ice_hw *hw, u8 *pba_num, u32 pba_num_size);
int ice_init_nvm(struct ice_hw *hw);
int ice_read_sr_word(struct ice_hw *hw, u16 offset, u16 *data);
int
ice_aq_erase_nvm(struct ice_hw *hw, u16 module_typeid, struct ice_sq_cd *cd);
int
ice_aq_update_nvm(struct ice_hw *hw, u16 module_typeid, u32 offset,
		  u16 length, void *data, bool last_command, u8 command_flags,
		  struct ice_sq_cd *cd);
int ice_nvm_validate_checksum(struct ice_hw *hw);
int ice_nvm_recalculate_checksum(struct ice_hw *hw);
int
ice_nvm_write_activate(struct ice_hw *hw, u16 cmd_flags, u8 *response_flags);
int ice_aq_nvm_update_empr(struct ice_hw *hw);
int
ice_nvm_set_pkg_data(struct ice_hw *hw, bool del_pkg_data_flag, u8 *data,
		     u16 length, struct ice_sq_cd *cd);
int
ice_nvm_pass_component_tbl(struct ice_hw *hw, u8 *data, u16 length,
			   u8 transfer_flag, u8 *comp_response,
			   u8 *comp_response_code, struct ice_sq_cd *cd);
#endif /* _ICE_NVM_H_ */
