// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2018-2019, Intel Corporation. */

#include "ice.h"
#include "ice_lib.h"
#include "ice_devlink.h"
#include "ice_eswitch.h"
#include "ice_fw_update.h"

#ifdef HAVE_DEVLINK_INFO_GET
static void ice_info_get_dsn(struct ice_pf *pf, char *buf, size_t len)
{
	u8 dsn[8];

	/* Copy the DSN into an array in Big Endian format */
	put_unaligned_be64(pci_get_dsn(pf->pdev), dsn);

	snprintf(buf, len, "%8phD", dsn);
}

static int ice_info_pba(struct ice_pf *pf, char *buf, size_t len)
{
	struct ice_hw *hw = &pf->hw;
	enum ice_status status;

	status = ice_read_pba_string(hw, (u8 *)buf, len);
	if (status)
		return -EIO;

	return 0;
}

static int ice_info_fw_mgmt(struct ice_pf *pf, char *buf, size_t len)
{
	struct ice_hw *hw = &pf->hw;

	snprintf(buf, len, "%u.%u.%u", hw->fw_maj_ver, hw->fw_min_ver,
		 hw->fw_patch);

	return 0;
}

static int ice_info_fw_api(struct ice_pf *pf, char *buf, size_t len)
{
	struct ice_hw *hw = &pf->hw;

	snprintf(buf, len, "%u.%u", hw->api_maj_ver, hw->api_min_ver);

	return 0;
}

static int ice_info_fw_build(struct ice_pf *pf, char *buf, size_t len)
{
	struct ice_hw *hw = &pf->hw;

	snprintf(buf, len, "0x%08x", hw->fw_build);

	return 0;
}

static int ice_info_orom_ver(struct ice_pf *pf, char *buf, size_t len)
{
	struct ice_orom_info *orom = &pf->hw.nvm.orom;

	snprintf(buf, len, "%u.%u.%u", orom->major, orom->build, orom->patch);

	return 0;
}

static int ice_info_nvm_ver(struct ice_pf *pf, char *buf, size_t len)
{
	struct ice_nvm_info *nvm = &pf->hw.nvm;

	snprintf(buf, len, "%x.%02x", nvm->major_ver, nvm->minor_ver);

	return 0;
}

static int ice_info_eetrack(struct ice_pf *pf, char *buf, size_t len)
{
	struct ice_nvm_info *nvm = &pf->hw.nvm;

	snprintf(buf, len, "0x%08x", nvm->eetrack);

	return 0;
}

static int ice_info_ddp_pkg_name(struct ice_pf *pf, char *buf, size_t len)
{
	struct ice_hw *hw = &pf->hw;

	snprintf(buf, len, "%s", hw->active_pkg_name);

	return 0;
}

static int ice_info_ddp_pkg_version(struct ice_pf *pf, char *buf, size_t len)
{
	struct ice_pkg_ver *pkg = &pf->hw.active_pkg_ver;

	snprintf(buf, len, "%u.%u.%u.%u", pkg->major, pkg->minor, pkg->update,
		 pkg->draft);

	return 0;
}

static int ice_info_ddp_pkg_bundle_id(struct ice_pf *pf, char *buf, size_t len)
{
	snprintf(buf, len, "0x%08x", pf->hw.active_track_id);

	return 0;
}

static int ice_info_netlist_ver(struct ice_pf *pf, char *buf, size_t len)
{
	struct ice_netlist_ver_info *netlist = &pf->hw.netlist_ver;

	/* The netlist versions are BCD formatted */
	snprintf(buf, len, "%x.%x.%x-%x.%x.%x", netlist->major, netlist->minor,
		 netlist->type >> 16, netlist->type & 0xFFFF, netlist->rev,
		 netlist->cust_ver);

	return 0;
}

static int ice_info_netlist_build(struct ice_pf *pf, char *buf, size_t len)
{
	struct ice_netlist_ver_info *netlist = &pf->hw.netlist_ver;

	snprintf(buf, len, "0x%08x", netlist->hash);

	return 0;
}

#define fixed(key, getter) { ICE_VERSION_FIXED, key, getter }
#define running(key, getter) { ICE_VERSION_RUNNING, key, getter }

enum ice_version_type {
	ICE_VERSION_FIXED,
	ICE_VERSION_RUNNING,
	ICE_VERSION_STORED,
};

static const struct ice_devlink_version {
	enum ice_version_type type;
	const char *key;
	int (*getter)(struct ice_pf *pf, char *buf, size_t len);
} ice_devlink_versions[] = {
	fixed(DEVLINK_INFO_VERSION_GENERIC_BOARD_ID, ice_info_pba),
	running(DEVLINK_INFO_VERSION_GENERIC_FW_MGMT, ice_info_fw_mgmt),
	running("fw.mgmt.api", ice_info_fw_api),
	running("fw.mgmt.build", ice_info_fw_build),
	running(DEVLINK_INFO_VERSION_GENERIC_FW_UNDI, ice_info_orom_ver),
	running("fw.psid.api", ice_info_nvm_ver),
	running(DEVLINK_INFO_VERSION_GENERIC_FW_BUNDLE_ID, ice_info_eetrack),
	running("fw.app.name", ice_info_ddp_pkg_name),
	running(DEVLINK_INFO_VERSION_GENERIC_FW_APP, ice_info_ddp_pkg_version),
	running("fw.app.bundle_id", ice_info_ddp_pkg_bundle_id),
	running("fw.netlist", ice_info_netlist_ver),
	running("fw.netlist.build", ice_info_netlist_build),
};

/**
 * ice_devlink_info_get - .info_get devlink handler
 * @devlink: devlink instance structure
 * @req: the devlink info request
 * @extack: extended netdev ack structure
 *
 * Callback for the devlink .info_get operation. Reports information about the
 * device.
 *
 * Return: zero on success or an error code on failure.
 */
static int ice_devlink_info_get(struct devlink *devlink,
				struct devlink_info_req *req,
				struct netlink_ext_ack *extack)
{
	struct ice_pf *pf = devlink_priv(devlink);
	char buf[100];
	size_t i;
	int err;

	err = devlink_info_driver_name_put(req, KBUILD_MODNAME);
	if (err) {
		NL_SET_ERR_MSG_MOD(extack, "Unable to set driver name");
		return err;
	}

	ice_info_get_dsn(pf, buf, sizeof(buf));

	err = devlink_info_serial_number_put(req, buf);
	if (err) {
		NL_SET_ERR_MSG_MOD(extack, "Unable to set serial number");
		return err;
	}

	for (i = 0; i < ARRAY_SIZE(ice_devlink_versions); i++) {
		enum ice_version_type type = ice_devlink_versions[i].type;
		const char *key = ice_devlink_versions[i].key;

		err = ice_devlink_versions[i].getter(pf, buf, sizeof(buf));
		if (err) {
			NL_SET_ERR_MSG_MOD(extack, "Unable to obtain version info");
			return err;
		}

		switch (type) {
		case ICE_VERSION_FIXED:
			err = devlink_info_version_fixed_put(req, key, buf);
			if (err) {
				NL_SET_ERR_MSG_MOD(extack, "Unable to set fixed version");
				return err;
			}
			break;
		case ICE_VERSION_RUNNING:
			err = devlink_info_version_running_put(req, key, buf);
			if (err) {
				NL_SET_ERR_MSG_MOD(extack, "Unable to set running version");
				return err;
			}
			break;
		case ICE_VERSION_STORED:
			err = devlink_info_version_stored_put(req, key, buf);
			if (err) {
				NL_SET_ERR_MSG_MOD(extack, "Unable to set stored version");
				return err;
			}
			break;
		}
	}

	return 0;
}
#endif /* HAVE_DEVLINK_INFO_GET */

#ifdef HAVE_DEVLINK_PARAMS
enum ice_devlink_param_id {
	ICE_DEVLINK_PARAM_ID_BASE = DEVLINK_PARAM_GENERIC_ID_MAX,
	ICE_DEVLINK_PARAM_ID_FLASH_UPDATE_PRESERVATION_LEVEL,
};

/**
 * ice_devlink_flash_param_get - Get a flash update parameter value
 * @devlink: pointer to the devlink instance
 * @id: the parameter id to get
 * @ctx: context to return the parameter value
 *
 * Reads the value of the given parameter and reports it back via the provided
 * context.
 *
 * Used to get the devlink parameters which control specific driver
 * behaviors during the .flash_update command.
 *
 * Returns: zero on success, or an error code on failure.
 */
static int ice_devlink_flash_param_get(struct devlink *devlink, u32 id,
				       struct devlink_param_gset_ctx *ctx)
{
	struct ice_pf *pf = devlink_priv(devlink);
	struct ice_devlink_flash_params *params;

	params = &pf->flash_params;

	switch (id) {
	case ICE_DEVLINK_PARAM_ID_FLASH_UPDATE_PRESERVATION_LEVEL:
		ctx->val.vu8 = params->preservation_level;
		break;
	default:
		WARN(1, "parameter ID %u is not a flash update parameter", id);
		return -EINVAL;
	}

	return 0;
}

/**
 * ice_devlink_flash_param_set - Set a flash update parameter value
 * @devlink: pointer to the devlink instance
 * @id: the parameter ID to set
 * @ctx: context to return the parameter value
 *
 * Reads the value of the given parameter and reports it back via the provided
 * context.
 *
 * Used to set the devlink parameters which control specific driver
 * behaviors during the .flash_update command.
 *
 * Returns: zero on success, or an error code on failure.
 */
static int ice_devlink_flash_param_set(struct devlink *devlink, u32 id,
				       struct devlink_param_gset_ctx *ctx)
{
	struct ice_pf *pf = devlink_priv(devlink);
	struct ice_devlink_flash_params *params;

	params = &pf->flash_params;

	switch (id) {
	case ICE_DEVLINK_PARAM_ID_FLASH_UPDATE_PRESERVATION_LEVEL:
		params->preservation_level =
			(enum ice_flash_update_preservation)ctx->val.vu8;
		break;
	default:
		WARN(1, "parameter ID %u is not a flash update parameter", id);
		return -EINVAL;
	}

	return 0;
}

/**
 * ice_devlink_flash_preservation_validate - Validate preservation level
 * @devlink: unused pointer to devlink instance
 * @id: the parameter ID to validate
 * @val: value to validate
 * @extack: netlink extended ACK structure
 *
 * Validate that the value for "flash_update_preservation_level" is within the
 * valid range.
 *
 * Returns: zero if the value is valid, -ERANGE if it is out of range, and
 * -EINVAL if this function is called with the wrong id.
 */
static int
ice_devlink_flash_preservation_validate(struct devlink __always_unused *devlink,
					u32 id, union devlink_param_value val,
					struct netlink_ext_ack *extack)
{
	if (WARN_ON(id != ICE_DEVLINK_PARAM_ID_FLASH_UPDATE_PRESERVATION_LEVEL))
		return -EINVAL;

	switch (val.vu8) {
	case ICE_FLASH_UPDATE_PRESERVE_ALL:
	case ICE_FLASH_UPDATE_PRESERVE_LIMITED:
	case ICE_FLASH_UPDATE_PRESERVE_FACTORY_SETTINGS:
	case ICE_FLASH_UPDATE_PRESERVE_NONE:
		return 0;
	}

	return -ERANGE;
}

/* devlink parameters for the ice driver */
static const struct devlink_param ice_devlink_params[] = {
	DEVLINK_PARAM_DRIVER(ICE_DEVLINK_PARAM_ID_FLASH_UPDATE_PRESERVATION_LEVEL,
			     "flash_update_preservation_level",
			     DEVLINK_PARAM_TYPE_U8,
			     BIT(DEVLINK_PARAM_CMODE_RUNTIME),
			     ice_devlink_flash_param_get,
			     ice_devlink_flash_param_set,
			     ice_devlink_flash_preservation_validate),
};
#endif /* HAVE_DEVLINK_PARAMS */

#ifdef HAVE_DEVLINK_FLASH_UPDATE
/**
 * ice_devlink_flash_update - Update firmware stored in flash on the device
 * @devlink: pointer to devlink associated with device to update
 * @path: the path of the firmware file to use via request_firmware
 * @component: name of the component to update, or NULL
 * @extack: netlink extended ACK structure
 *
 * Perform a device flash update. The bulk of the update logic is contained
 * within the ice_flash_pldm_image function.
 *
 * Returns: zero on success, or an error code on failure.
 */
static int
ice_devlink_flash_update(struct devlink *devlink, const char *path,
			 const char *component, struct netlink_ext_ack *extack)
{
	struct ice_pf *pf = devlink_priv(devlink);
	struct device *dev = &pf->pdev->dev;
	struct ice_hw *hw = &pf->hw;
	const struct firmware *fw;
	int err;

	/* individual component update is not yet supported */
	if (component)
		return -EOPNOTSUPP;

	if (!hw->dev_caps.common_cap.nvm_unified_update) {
		NL_SET_ERR_MSG_MOD(extack, "Current firmware does not support unified update");
		return -EOPNOTSUPP;
	}

	err = ice_check_for_pending_update(pf, component, extack);
	if (err)
		return err;

	err = request_firmware(&fw, path, dev);
	if (err) {
		NL_SET_ERR_MSG_MOD(extack, "Unable to read file from disk");
		return err;
	}

	devlink_flash_update_begin_notify(devlink);
	devlink_flash_update_status_notify(devlink, "Preparing to flash",
					   component, 0, 0);
	err = ice_flash_pldm_image(pf, fw, extack);
	devlink_flash_update_end_notify(devlink);

	release_firmware(fw);

	return err;
}
#endif /* HAVE_DEVLINK_FLASH_UPDATE */

static const struct devlink_ops ice_devlink_ops = {
	.eswitch_mode_get = ice_eswitch_mode_get,
	.eswitch_mode_set = ice_eswitch_mode_set,
#ifdef HAVE_DEVLINK_INFO_GET
	.info_get = ice_devlink_info_get,
#endif /* HAVE_DEVLINK_INFO_GET */
#ifdef HAVE_DEVLINK_FLASH_UPDATE
	.flash_update = ice_devlink_flash_update,
#endif /* HAVE_DEVLINK_FLASH_UPDATE */
};

static void ice_devlink_free(void *devlink_ptr)
{
	devlink_free((struct devlink *)devlink_ptr);
}

/**
 * ice_allocate_pf - Allocate devlink and return PF structure pointer
 * @dev: the device to allocate for
 *
 * Allocate a devlink instance for this device and return the private area as
 * the PF structure. The devlink memory is kept track of through devres by
 * adding an action to remove it when unwinding.
 */
struct ice_pf *ice_allocate_pf(struct device *dev)
{
	struct devlink *devlink;

	devlink = devlink_alloc(&ice_devlink_ops, sizeof(struct ice_pf));
	if (!devlink)
		return NULL;

	/* Add an action to teardown the devlink when unwinding the driver */
	if (devm_add_action(dev, ice_devlink_free, devlink)) {
		devlink_free(devlink);
		return NULL;
	}

	return devlink_priv(devlink);
}

/**
 * ice_devlink_register - Register devlink interface for this PF
 * @pf: the PF to register the devlink for.
 *
 * Register the devlink instance associated with this physical function.
 *
 * Return: zero on success or an error code on failure.
 */
int ice_devlink_register(struct ice_pf *pf)
{
	struct devlink *devlink = priv_to_devlink(pf);
	struct device *dev = ice_pf_to_dev(pf);
	int err;

	err = devlink_register(devlink, dev);
	if (err) {
		dev_err(dev, "devlink registration failed: %d\n", err);
		return err;
	}

#ifdef HAVE_DEVLINK_PARAMS
	err = devlink_params_register(devlink, ice_devlink_params,
				      ARRAY_SIZE(ice_devlink_params));
	if (err) {
		dev_err(dev, "devlink params registration failed: %d\n", err);
		return err;
	}

	devlink_params_publish(devlink);
#endif /* HAVE_DEVLINK_PARAMS */

	return 0;
}

/**
 * ice_devlink_unregister - Unregister devlink resources for this PF.
 * @pf: the PF structure to cleanup
 *
 * Releases resources used by devlink and cleans up associated memory.
 */
void ice_devlink_unregister(struct ice_pf *pf)
{
	struct devlink *devlink = priv_to_devlink(pf);

#ifdef HAVE_DEVLINK_PARAMS
	devlink_params_unpublish(devlink);
	devlink_params_unregister(devlink, ice_devlink_params,
				  ARRAY_SIZE(ice_devlink_params));
#endif /* HAVE_DEVLINK_PARAMS */
	devlink_unregister(devlink);
}

/**
 * ice_devlink_create_port - Create a devlink port for this VSI
 * @vsi: the VSI to create a port for
 *
 * Create and register a devlink_port for this VSI.
 *
 * Return: zero on success or an error code on failure.
 */
int ice_devlink_create_port(struct ice_vsi *vsi)
{
	struct devlink_port_attrs attrs = {};
	struct ice_port_info *pi;
	struct devlink *devlink;
	struct device *dev;
	struct ice_pf *pf;
	int err;

	/* Currently we only create devlink_port instances for PF VSIs */
	if (vsi->type != ICE_VSI_PF)
		return -EINVAL;

	pf = vsi->back;
	devlink = priv_to_devlink(pf);
	dev = ice_pf_to_dev(pf);
	pi = pf->hw.port_info;

	attrs.flavour = DEVLINK_PORT_FLAVOUR_PHYSICAL;
	attrs.phys.port_number = pi->lport;
	devlink_port_attrs_set(&vsi->devlink_port, &attrs);
	err = devlink_port_register(devlink, &vsi->devlink_port, vsi->idx);
	if (err) {
		dev_err(dev, "devlink_port_register failed: %d\n", err);
		return err;
	}

	vsi->devlink_port_registered = true;

	return 0;
}

/**
 * ice_devlink_destroy_port - Destroy the devlink_port for this VSI
 * @vsi: the VSI to cleanup
 *
 * Unregisters the devlink_port structure associated with this VSI.
 */
void ice_devlink_destroy_port(struct ice_vsi *vsi)
{
	if (!vsi->devlink_port_registered)
		return;

	devlink_port_type_clear(&vsi->devlink_port);
	devlink_port_unregister(&vsi->devlink_port);

	vsi->devlink_port_registered = false;
}

#ifdef HAVE_DEVLINK_REGIONS
#ifdef HAVE_DEVLINK_REGION_OPS_SNAPSHOT
/**
 * ice_devlink_nvm_snapshot - Capture a snapshot of the Shadow RAM contents
 * @devlink: the devlink instance
 * @extack: extended ACK response structure
 * @data: on exit points to snapshot data buffer
 *
 * This function is called in response to the DEVLINK_CMD_REGION_TRIGGER for
 * the shadow-ram devlink region. It captures a snapshot of the shadow ram
 * contents. This snapshot can later be viewed via the devlink-region
 * interface.
 *
 * @returns zero on success, and updates the data pointer. Returns a non-zero
 * error code on failure.
 */
static int ice_devlink_nvm_snapshot(struct devlink *devlink,
				    struct netlink_ext_ack *extack, u8 **data)
{
	struct ice_pf *pf = devlink_priv(devlink);
	struct device *dev = ice_pf_to_dev(pf);
	struct ice_hw *hw = &pf->hw;
	enum ice_status status;
	u8 *nvm_data;
	u32 nvm_size;

	nvm_size = hw->nvm.flash_size;
	nvm_data = vzalloc(nvm_size);
	if (!nvm_data)
		return -ENOMEM;

	status = ice_acquire_nvm(hw, ICE_RES_READ);
	if (status) {
		dev_dbg(dev, "ice_acquire_nvm failed, err %d aq_err %d\n",
			status, hw->adminq.sq_last_status);
		NL_SET_ERR_MSG_MOD(extack, "Failed to acquire NVM semaphore");
		vfree(nvm_data);
		return -EIO;
	}

	status = ice_read_flat_nvm(hw, 0, &nvm_size, nvm_data, false);
	if (status) {
		dev_dbg(dev, "ice_read_flat_nvm failed after reading %u bytes, err %d aq_err %d\n",
			nvm_size, status, hw->adminq.sq_last_status);
		NL_SET_ERR_MSG_MOD(extack, "Failed to read NVM contents");
		ice_release_nvm(hw);
		vfree(nvm_data);
		return -EIO;
	}

	ice_release_nvm(hw);

	*data = nvm_data;

	return 0;
}

/**
 * ice_devlink_devcaps_snapshot - Capture snapshot of device capabilities
 * @devlink: the devlink instance
 * @extack: extended ACK response structure
 * @data: on exit points to snapshot data buffer
 *
 * This function is called in response to the DEVLINK_CMD_REGION_TRIGGER for
 * the device-caps devlink region. It captures a snapshot of the device
 * capabilities reported by firmware.
 *
 * @returns zero on success, and updates the data pointer. Returns a non-zero
 * error code on failure.
 */
static int
ice_devlink_devcaps_snapshot(struct devlink *devlink,
			     struct netlink_ext_ack *extack, u8 **data)
{
	struct ice_pf *pf = devlink_priv(devlink);
	struct device *dev = ice_pf_to_dev(pf);
	struct ice_hw *hw = &pf->hw;
	enum ice_status status;
	void *devcaps;

	devcaps = vzalloc(ICE_AQ_MAX_BUF_LEN);
	if (!devcaps)
		return -ENOMEM;

	status = ice_aq_list_caps(hw, devcaps, ICE_AQ_MAX_BUF_LEN, NULL,
				  ice_aqc_opc_list_dev_caps, NULL);
	if (status) {
		dev_dbg(dev, "ice_aq_list_caps: failed to read device capabilities, err %d aq_err %d\n",
			status, hw->adminq.sq_last_status);
		NL_SET_ERR_MSG_MOD(extack, "Failed to read device capabilities");
		vfree(devcaps);
		return -EIO;
	}

	*data = (u8 *)devcaps;

	return 0;
}
#endif /* HAVE_DEVLINK_REGION_OPS_SNAPSHOT */

static const struct devlink_region_ops ice_nvm_region_ops = {
	.name = "nvm-flash",
	.destructor = vfree,
#ifdef HAVE_DEVLINK_REGION_OPS_SNAPSHOT
	.snapshot = ice_devlink_nvm_snapshot,
#endif
};

static const struct devlink_region_ops ice_devcaps_region_ops = {
	.name = "device-caps",
	.destructor = vfree,
#ifdef HAVE_DEVLINK_REGION_OPS_SNAPSHOT
	.snapshot = ice_devlink_devcaps_snapshot,
#endif
};

/**
 * ice_devlink_init_regions - Initialize devlink regions
 * @pf: the PF device structure
 *
 * Create devlink regions used to enable access to dump the contents of the
 * flash memory on the device.
 */
void ice_devlink_init_regions(struct ice_pf *pf)
{
	struct devlink *devlink = priv_to_devlink(pf);
	struct device *dev = ice_pf_to_dev(pf);
	u64 nvm_size;

	nvm_size = pf->hw.nvm.flash_size;
	pf->nvm_region = devlink_region_create(devlink, &ice_nvm_region_ops, 1,
					       nvm_size);
	if (IS_ERR(pf->nvm_region)) {
		dev_err(dev, "failed to create NVM devlink region, err %ld\n",
			PTR_ERR(pf->nvm_region));
		pf->nvm_region = NULL;
	}

	pf->devcaps_region = devlink_region_create(devlink,
						   &ice_devcaps_region_ops, 10,
						   ICE_AQ_MAX_BUF_LEN);
	if (IS_ERR(pf->devcaps_region)) {
		dev_err(dev, "failed to create device-caps devlink region, err %ld\n",
			PTR_ERR(pf->devcaps_region));
		pf->devcaps_region = NULL;
	}
}

/**
 * ice_devlink_destroy_regions - Destroy devlink regions
 * @pf: the PF device structure
 *
 * Remove previously created regions for this PF.
 */
void ice_devlink_destroy_regions(struct ice_pf *pf)
{
	if (pf->nvm_region)
		devlink_region_destroy(pf->nvm_region);

	if (pf->devcaps_region)
		devlink_region_destroy(pf->devcaps_region);
}
#endif /* HAVE_DEVLINK_REGIONS */
