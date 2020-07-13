// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2018-2019, Intel Corporation. */

#include <asm/unaligned.h>
#include <linux/uuid.h>
#include <linux/crc32.h>
#include "ice.h"
#include "ice_pldm.h"

/* The firmware files are stored using the layout specified in the
 * "PLDM for Firmware Update Specification", DMTF standard #DSP0267.
 *
 * The following structures and functions are used to parse this header and
 * extract necessary information in order to perform a firmware update.
 *
 * Each of these structures is __packed, and most of the multi-byte fields are
 * LittleEndian. There is no guarantee that the offset of a structure within
 * the file is aligned. All accesses to this data must be done using either
 * get_unaligned_le* functions or memcpy.
 *
 * Due to this restriction, the definitions are not public and any code which
 * accesses the file directly must be kept in this file.
 */

/* UUID for PLDM firmware packages: f018878c-cb7d-4943-9800-a02f059aca02 */
static const uuid_t pldm_firmware_header_id =
	UUID_INIT(0xf018878c, 0xcb7d, 0x4943,
		  0x98, 0x00, 0xa0, 0x2f, 0x05, 0x9a, 0xca, 0x02);

/* Revision number of the PLDM header format this code supports */
#define PACKAGE_HEADER_FORMAT_REVISION 0x01

/* timestamp104 structure defined in PLDM Base specification */
#define PLDM_TIMESTAMP_SIZE 13
struct pldm_timestamp {
	u8 b[PLDM_TIMESTAMP_SIZE];
} __aligned(1);

/* Package Header Information */
struct pldm_header {
	uuid_t id;			    /* PackageHeaderIdentifier */
	u8 revision;			    /* PackageHeaderFormatRevision */
	__le16 size;			    /* PackageHeaderSize */
	struct pldm_timestamp release_date; /* PackageReleaseDateTime */
	__le16 component_bitmap_len;	    /* ComponentBitmapBitLength */
	u8 version_type;		    /* PackageVersionStringType */
	u8 version_len;			    /* PackageVersionStringLength */

	/*
	 * DSP0267 also includes the following variable length fields at the
	 * end of this structure:
	 *
	 * PackageVersionString, length is version_len.
	 *
	 * The total size of this section is
	 *   sizeof(pldm_header) + version_len;
	 */
	char version_string[];		/* PackageVersionString */
} __packed __aligned(1);

/* Firmware Device ID Record */
struct pldm_record_info {
	__le16 record_len;		/* RecordLength */
	u8 descriptor_count;		/* DescriptorCount */
	__le32 device_update_flags;	/* DeviceUpdateOptionFlags */
	u8 version_type;		/* ComponentImageSetVersionType */
	u8 version_len;			/* ComponentImageSetVersionLength */
	__le16 package_data_len;	/* FirmwareDevicePackageDataLength */

	/*
	 * DSP0267 also includes the following variable length fields at the
	 * end of this structure:
	 *
	 * ApplicableComponents, length is component_bitmap_len from header
	 * ComponentImageSetVersionString, length is version_len
	 * RecordDescriptors, a series of TLVs with 16bit type and length
	 * FirmwareDevicePackageData, length is package_data_len
	 *
	 * The total size of each record is
	 *   sizeof(pldm_record_info) +
	 *   component_bitmap_len (converted to bytes!) +
	 *   version_len +
	 *   <length of RecordDescriptors> +
	 *   package_data_len
	 */
	u8 variable_record_data[];
} __packed __aligned(1);

#define PLDM_DESC_ID_PCI_VENDOR_ID	0x0000
#define PLDM_DESC_ID_IANA_ENTERPRISE_ID	0x0001
#define PLDM_DESC_ID_UUID		0x0002
#define PLDM_DESC_ID_PNP_VENDOR_ID	0x0003
#define PLDM_DESC_ID_ACPI_VENDOR_ID	0x0004
#define PLDM_DESC_ID_PCI_DEVICE_ID	0x0100
#define PLDM_DESC_ID_PCI_SUBVENDOR_ID	0x0101
#define PLDM_DESC_ID_PCI_SUBDEV_ID	0x0102
#define PLDM_DESC_ID_PCI_REVISION_ID	0x0103
#define PLDM_DESC_ID_PNP_PRODUCT_ID	0x0104
#define PLDM_DESC_ID_ACPI_PRODUCT_ID	0x0105
#define PLDM_DESC_ID_VENDOR_DEFINED	0xFFFF

/* Firmware Descriptor Definition */
struct pldm_desc_tlv {
	__le16 type;			/* DescriptorType */
	__le16 size;			/* DescriptorSize */
	u8 data[];			/* DescriptorData */
} __aligned(1);

/* Firmware Device Identification Area */
struct pldm_record_area {
	u8 record_count;		/* DeviceIDRecordCount */
	/* This is not a struct type because the size of each record varies */
	u8 records[];
} __aligned(1);

/* Individual Component Image Information */
struct pldm_component_info {
	__le16 classification;		/* ComponentClassfication */
	__le16 identifier;		/* ComponentIdentifier */
	__le32 comparison_stamp;	/* ComponentComparisonStamp */
	__le16 options;			/* componentOptions */
	__le16 activation_method;	/* RequestedComponentActivationMethod */
	__le32 location_offset;		/* ComponentLocationOffset */
	__le32 size;			/* ComponentSize */
	u8 version_type;		/* ComponentVersionStringType */
	u8 version_len;		/* ComponentVersionStringLength */

	/*
	 * DSP0267 also includes the following variable length fields at the
	 * end of this structure:
	 *
	 * ComponentVersionString, length is version_len
	 *
	 * The total size of this section is
	 *   sizeof(pldm_component_info) + version_len;
	 */
	char version_string[];		/* ComponentVersionString */
} __packed __aligned(1);

/* Component Image Information Area */
struct pldm_component_area {
	__le16 component_image_count;
	/* This is not a struct type because the component size varies */
	u8 components[];
} __aligned(1);

/* structure used to store details about the parsed PLDM image file. */
struct ice_pldm_data {
	/* useful pointers */
	struct device *dev;
	struct netlink_ext_ack *extack;
	struct ice_pf *pf;
	const u8 *fw_data;
	size_t fw_size;

	/* current offset of firmware image */
	size_t offset;

	/* PLDM Firmware Package Header */
	u16 total_header_size;
	const struct pldm_header *header;

	/* Start pf the firmware device id records */
	u8 record_count;
	const u8 *record_start;

	/* Start of the component image information */
	u16 component_count;
	const u8 *component_start;

	/* Start of the header CRC */
	const __le32 *header_crc_start;

	/* length of the component bitmap */
	u16 component_bitmap_len;
	u16 bitmap_size;

	/* matching record information */
	const u8 *bitmap_ptr;
	const u8 *package_data_ptr;
	u16 package_data_len;
};

/**
 * ice_first_desc_tlv
 * @start: byte offset of the start of the descriptor TLVs
 *
 * Converts the starting offset of the descriptor TLVs into a pointer to the
 * first descriptor.
 */
#define ice_first_desc_tlv(start)					\
	((const struct pldm_desc_tlv *)(start))

/**
 * ice_next_desc_tlv
 * @desc: pointer to a descriptor TLV
 *
 * Finds the pointer to the next descriptor following a given descriptor
 */
#define ice_next_desc_tlv(desc)						\
	((const struct pldm_desc_tlv *)((desc)->data +			\
					get_unaligned_le16(&(desc)->size)))

/**
 * ice_for_each_desc_tlv
 * @i: variable to store descriptor index
 * @desc: variable to store descriptor pointer
 * @start: byte offset of the start of the descriptors
 * @count: the number of descriptors
 *
 * for loop macro to iterate over all of the descriptors of a given PLDM
 * record.
 */
#define ice_for_each_desc_tlv(i, desc, start, count)			\
	for ((i) = 0, (desc) = ice_first_desc_tlv(start);		\
	     (i) < (count);						\
	     (i)++, (desc) = ice_next_desc_tlv(desc))

/**
 * ice_first_pldm_record
 * @start: byte offset of the start of the PLDM records
 *
 * Converts a starting offset of the PLDM records into a pointer to the first
 * record.
 */
#define ice_first_pldm_record(start)					\
	((const struct pldm_record_info *)(start))

/**
 * ice_next_pldm_record
 * @record: pointer to a PLDM record
 *
 * Finds a pointer to the next record following a given record
 */
#define ice_next_pldm_record(record)					\
	((const struct pldm_record_info *)				\
	 ((const u8 *)(record) + get_unaligned_le16(&(record)->record_len)))

/**
 * ice_for_each_pldm_record
 * @i: variable to store record index
 * @record: variable to store record pointer
 * @start: byte offset of the start of the records
 * @count: the number of records
 *
 * for loop macro to iterate over all of the records of a PLDM file.
 */
#define ice_for_each_pldm_record(i, record, start, count)		\
	for ((i) = 0, (record) = ice_first_pldm_record(start);		\
	     (i) < (count);						\
	     (i)++, (record) = ice_next_pldm_record(record))

/**
 * ice_first_pldm_component
 * @start: byte offset of the start of the PLDM components
 *
 * Convert a starting offset of the PLDM components into a pointer to the
 * first component
 */
#define ice_first_pldm_component(start)					\
	((const struct pldm_component_info *)(start))

/**
 * ice_next_pldm_component
 * @component: pointer to a PLDM component
 *
 * Finds a pointer to the next component following a given component
 */
#define ice_next_pldm_component(component)				\
	((const struct pldm_component_info *)((component)->version_string + \
					      (component)->version_len))

/**
 * ice_for_each_pldm_component
 * @i: variable to store component index
 * @component: variable to store component pointer
 * @start: byte offset to the start of the first component
 * @count: the number of components
 *
 * for loop macro to iterate over all of the components of a PLDM file.
 */
#define ice_for_each_pldm_component(i, component, start, count)		\
	for ((i) = 0, (component) = ice_first_pldm_component(start);	\
	     (i) < (count);						\
	     (i)++, (component) = ice_next_pldm_component(component))

/**
 * ice_check_fw_space - Verify that the firmware image has space left
 * @data: storage for image details
 * @offset: offset to start from
 * @length: length to check for
 *
 * Verify that the firmware data can hold a chunk of bytes with the specified
 * offset and length.
 *
 * Returns: zero on success, or -ENOSPC if the image is too small.
 */
static int
ice_check_fw_space(struct ice_pldm_data *data, size_t offset, size_t length)
{
	struct netlink_ext_ack *extack = data->extack;
	size_t expected_size = offset + length;
	struct device *dev = data->dev;

	if (data->fw_size < expected_size) {
		dev_err(dev, "Firmware file size smaller than expected. Got %zu bytes, needed %zu bytes\n",
			data->fw_size, expected_size);
		NL_SET_ERR_MSG_MOD(extack, "Firmware file size too small");
		return -ENOSPC;
	}

	return 0;
}

/**
 * ice_move_fw_offset - Move the current firmware offset forward
 * @data: storage for image details
 * @bytes_to_move: number of bytes to move the offset forward by
 *
 * Check that there is enough space past the current offset, and then move the
 * offset forward by this ammount.
 *
 * Returns: zero on success, or -ENOSPC if the image is too small.
 */
static int
ice_move_fw_offset(struct ice_pldm_data *data, size_t bytes_to_move)
{
	int err;

	err = ice_check_fw_space(data, data->offset, bytes_to_move);
	if (err)
		return err;

	data->offset += bytes_to_move;

	return 0;
}

/**
 * ice_parse_pldm_header - Validate and extract details about the PLDM header
 * @data: storage for image details
 *
 * Performs initial basic verification of the PLDM image, up to the first
 * firmware record.
 *
 * This includes the following checks and extractions
 *
 *   * Verify that the UUID at the start of the header matches the expected
 *     value as defined in the DSP0267 PLDM specification
 *   * Check that the revision is 0x01
 *   * Extract the total header_size and verify that the image is large enough
 *     to contain at least the length of this header
 *   * Extract the size of the component bitmap length
 *   * Extract a pointer to the start of the record area
 *
 * Returns: zero on success, or a negative error code on failure.
 */
static int ice_parse_pldm_header(struct ice_pldm_data *data)
{
	struct netlink_ext_ack *extack = data->extack;
	const struct pldm_record_area *record_area;
	const struct pldm_header *header;
	struct device *dev = data->dev;
	size_t header_size;
	int err;

	err = ice_move_fw_offset(data, sizeof(*header));
	if (err)
		return err;

	header = (const struct pldm_header *)data->fw_data;
	data->header = header;

	if (!uuid_equal(&header->id, &pldm_firmware_header_id)) {
		dev_err(dev, "Invalid package header identifier. Expected UUID %pUB, but got %pUB\n",
			&pldm_firmware_header_id, &header->id);
		NL_SET_ERR_MSG_MOD(extack, "Invalid package header identifier");
		return -EINVAL;
	}

	if (header->revision != PACKAGE_HEADER_FORMAT_REVISION) {
		dev_err(dev, "Invalid package header revision. Expected revision %u but got %u\n",
			PACKAGE_HEADER_FORMAT_REVISION, header->revision);

		NL_SET_ERR_MSG_MOD(extack, "Invalid package header revision");
		return -EINVAL;
	}

	data->total_header_size = get_unaligned_le16(&header->size);
	header_size = data->total_header_size - sizeof(*header);

	err = ice_check_fw_space(data, data->offset, header_size);
	if (err)
		return err;

	data->component_bitmap_len =
		get_unaligned_le16(&header->component_bitmap_len);

	if (data->component_bitmap_len % 8 != 0) {
		dev_err(dev, "Invalid component bitmap length. The length is %u, which is not a multiple of 8\n",
			data->component_bitmap_len);
		NL_SET_ERR_MSG_MOD(extack, "Invalid component bitmap length");
		return -EINVAL;
	}

	data->bitmap_size = data->component_bitmap_len / 8;

	err = ice_move_fw_offset(data, header->version_len);
	if (err)
		return err;

	/* extract a pointer to the record area, which just follows the main
	 * PLDM header data.
	 */
	record_area = (const struct pldm_record_area *)(data->fw_data +
							 data->offset);

	err = ice_move_fw_offset(data, sizeof(*record_area));
	if (err)
		return err;

	data->record_count = record_area->record_count;
	data->record_start = record_area->records;

	return 0;
}

/**
 * ice_check_desc_tlv_length - Check that the length matches expectation
 * @data: pointer to image details
 * @type: the descriptor type
 * @size: the length from the descriptor header
 *
 * If the descriptor type is one of the documented descriptor types according
 * to the standard, verify that the provided length matches.
 *
 * If the type is not recognized or is VENDOR_DEFINED, return zero.
 *
 * Returns: zero on success, or -EINVAL if the length does not match.
 */
static int
ice_check_desc_tlv_length(struct ice_pldm_data *data, u16 type, u16 size)
{
	struct device *dev = data->dev;
	u16 expected_size;

	switch (type) {
	case PLDM_DESC_ID_PCI_VENDOR_ID:
	case PLDM_DESC_ID_PCI_DEVICE_ID:
	case PLDM_DESC_ID_PCI_SUBVENDOR_ID:
	case PLDM_DESC_ID_PCI_SUBDEV_ID:
		expected_size = 2;
		break;
	case PLDM_DESC_ID_PCI_REVISION_ID:
		expected_size = 1;
		break;
	case PLDM_DESC_ID_PNP_VENDOR_ID:
		expected_size = 3;
		break;
	case PLDM_DESC_ID_IANA_ENTERPRISE_ID:
	case PLDM_DESC_ID_ACPI_VENDOR_ID:
	case PLDM_DESC_ID_PNP_PRODUCT_ID:
	case PLDM_DESC_ID_ACPI_PRODUCT_ID:
		expected_size = 4;
		break;
	case PLDM_DESC_ID_UUID:
		expected_size = 16;
		break;
	case PLDM_DESC_ID_VENDOR_DEFINED:
		return 0;
	default:
		/* Do not report an error on an unexpected TLV */
		dev_dbg(dev, "Found unrecognized TLV type 0x%04x\n", type);
		return 0;
	}

	if (size != expected_size) {
		dev_err(dev, "Found TLV type 0x%04x with unexpected length. Got %u bytes, but expected %u bytes\n",
			type, size, expected_size);
		return -EINVAL;
	}

	return 0;
}

/**
 * ice_parse_desc_tlvs - Check and skip past a number of TLVs
 * @data: storage for image details
 * @desc_count: descriptor count
 *
 * From the current offset, read and skip past the descriptor TLVs, updating
 * the current offset each time.
 *
 * Returns: zero on success, or an error code on failure.
 */
static int ice_parse_desc_tlvs(struct ice_pldm_data *data, u8 desc_count)
{
	const struct pldm_desc_tlv *desc;
	const u8 *desc_start;
	u8 i;

	desc_start = data->fw_data + data->offset;

	ice_for_each_desc_tlv(i, desc, desc_start, desc_count) {
		u16 type, size;
		int err;

		err = ice_move_fw_offset(data, sizeof(*desc));
		if (err)
			return err;

		type = get_unaligned_le16(&desc->type);

		/* According to DSP0267, this only includes the data field */
		size = get_unaligned_le16(&desc->size);

		err = ice_check_desc_tlv_length(data, type, size);
		if (err)
			return err;

		/* check that we have space and move the offset forward */
		err = ice_move_fw_offset(data, size);
		if (err)
			return err;
	}

	return 0;
}

/**
 * ice_parse_one_pldm_record - Verify size of one PLDM record
 * @data: pointer to image details
 * @record: pointer to the record to check
 *
 * This function checks that the record size does not exceed either the size
 * of the firmware file or the total length specified in the header section.
 *
 * It also verifies that the recorded length of the start of the record
 * matches the size calculated by adding the static structure length, the
 * component bitmap length, the version string length, the length of all
 * descriptor TLVs, and the length of the package data.
 *
 * Returns: zero on success, or an error code on failure.
 */
static int ice_parse_one_pldm_record(struct ice_pldm_data *data,
				     const struct pldm_record_info *record)
{
	u16 record_len, package_data_len;
	size_t measured_length;
	int err;

	/* Then check that we have space and move the offset */
	err = ice_move_fw_offset(data, sizeof(*record));
	if (err)
		return err;

	/* read the record length and descriptor count */
	record_len = get_unaligned_le16(&record->record_len);
	package_data_len = get_unaligned_le16(&record->package_data_len);

	/* check that we have space for the component bitmap length */
	err = ice_move_fw_offset(data, data->bitmap_size);
	if (err)
		return err;

	err = ice_move_fw_offset(data, record->version_len);
	if (err)
		return err;

	/* Scan through the descriptor TLVs and find the end */
	err = ice_parse_desc_tlvs(data, record->descriptor_count);
	if (err)
		return err;

	err = ice_move_fw_offset(data, package_data_len);
	if (err)
		return err;

	measured_length = data->offset - ((const u8 *)record - data->fw_data);
	if (measured_length != record_len) {
		dev_err(data->dev, "Unexpected record length. Measured record length is %zu bytes, expected length is %u bytes\n",
			measured_length, record_len);
		NL_SET_ERR_MSG_MOD(data->extack, "Unexpected record length");
		return -EINVAL;
	}

	return 0;
}

/**
 * ice_parse_pldm_records - Locate the start of the component area
 * @data: storage for image details
 *
 * Extract the record count, and loop through each record, searching for the
 * component area.
 *
 * Returns: zero on success, or a negative error code on failure.
 */
static int ice_parse_pldm_records(struct ice_pldm_data *data)
{
	const struct pldm_component_area *component_area;
	const struct pldm_record_info *record;
	int err;
	u8 i;

	ice_for_each_pldm_record(i, record, data->record_start,
				 data->record_count) {
		err = ice_parse_one_pldm_record(data, record);
		if (err)
			return err;
	}

	/* Extract a pointer to the component area, which just follows the
	 * PLDM device record data.
	 */
	component_area = (const struct pldm_component_area *)(data->fw_data +
							      data->offset);

	err = ice_move_fw_offset(data, sizeof(*component_area));
	if (err)
		return err;

	data->component_count =
		get_unaligned_le16(&component_area->component_image_count);
	data->component_start = component_area->components;

	return 0;
}

/**
 * ice_check_component_size - Check that the component data fits in fw image
 * @data: storage for image details
 * @component: the component to process
 *
 * Check that the data pointed to by a given component fits within the
 * firmware image file provided by userspace.
 *
 * Returns: zero on success, or an error code on failure.
 */
static int
ice_check_component_size(struct ice_pldm_data *data,
			 const struct pldm_component_info *component)
{
	u32 offset, size;

	offset = get_unaligned_le32(&component->location_offset);
	size = get_unaligned_le32(&component->size);

	return ice_check_fw_space(data, offset, size);
}

/**
 * ice_parse_pldm_components - Locate the CRC header checksum
 * @data: storage for image details
 *
 * Extract the component count, and find the pointer to the component area.
 * Scan through each component searching for the end, which should point to
 * the package header checksum.
 *
 * Extract the offset to the package header CRC and save it for verification.
 *
 * Returns: zero on success, or a negative error code on failure.
 */
static int ice_parse_pldm_components(struct ice_pldm_data *data)
{
	struct netlink_ext_ack *extack = data->extack;
	const struct pldm_component_info *component;
	struct device *dev = data->dev;
	const __le32 *header_crc_ptr;
	int err;
	u8 i;

	ice_for_each_pldm_component(i, component, data->component_start,
				    data->component_count) {
		err = ice_move_fw_offset(data, sizeof(*component));
		if (err)
			return err;

		err = ice_move_fw_offset(data, component->version_len);
		if (err)
			return err;

		err = ice_check_component_size(data, component);
		if (err)
			return err;
	}

	header_crc_ptr = (const __le32 *)(data->fw_data + data->offset);

	err = ice_move_fw_offset(data, sizeof(*header_crc_ptr));
	if (err)
		return err;

	/* Make sure that we reached the expected offset */
	if (data->offset != data->total_header_size) {
		dev_err(dev, "Invalid firmware header size. Expected %u but got %zu\n",
			data->total_header_size, data->offset);
		NL_SET_ERR_MSG_MOD(extack, "Invalid firmware header size");
		return -EINVAL;
	}

	data->header_crc_start = header_crc_ptr;

	return 0;
}

/**
 * ice_verify_pldm_crc - Verify that the CRC in the header matches
 * @data: storage for image details
 *
 * Calculates the 32-bit CRC using the standard IEEE 802.3 CRC polynomial and
 * compares it to the value stored in the header.
 *
 * Returns: zero if the CRC matches, or an error code on mismatched CRC or
 * header size.
 */
static int ice_verify_pldm_crc(struct ice_pldm_data *data)
{
	struct netlink_ext_ack *extack = data->extack;
	u32 calculated_crc, header_crc;
	struct device *dev = data->dev;
	size_t length;

	header_crc = get_unaligned_le32(data->header_crc_start);

	/* Calculate the 32-bit CRC of the header header contents up to but
	 * not including the checksum. Note that the Linux crc32_le function
	 * does not perform an expected final XOR.
	 */
	length = data->offset - sizeof(header_crc);
	calculated_crc = crc32_le(~0, data->fw_data, length) ^ ~0;

	if (calculated_crc != header_crc) {
		dev_err(dev, "Invalid CRC in firmware header. Got 0x%08x but expected 0x%08x\n",
			calculated_crc, header_crc);
		NL_SET_ERR_MSG_MOD(extack, "Invalid CRC in firmware header");
		return -EBADMSG;
	}

	return 0;
}

/**
 * ice_parse_pldm_image - Validate and extract details from PLDM image
 * @data: storage for image details
 *
 * Verify that the firmware file contains valid data for a PLDM firmware
 * file. Extract useful pointers and data from the firmware file and store
 * them in the data structure.
 *
 * The PLDM firmware file format is defined in DMTF DSP0267 1.0.0. Care
 * should be taken to use get_unaligned_le* when accessing data from the
 * pointers in data.
 *
 * Returns: zero on success, or a negative error code on failure.
 */
static int ice_parse_pldm_image(struct ice_pldm_data *data)
{
	int err;

	if (WARN_ON(!(data->dev && data->fw_data && data->fw_size)))
		return -EINVAL;

	err = ice_parse_pldm_header(data);
	if (err)
		return err;

	err = ice_parse_pldm_records(data);
	if (err)
		return err;

	err = ice_parse_pldm_components(data);
	if (err)
		return err;

	return ice_verify_pldm_crc(data);
}

struct ice_record_details {
	/* descriptor data */
	u16 vendor_id;
	u16 device_id;
	u16 subvendor_id;
	u16 subdevice_id;

	/* offset into the PLDM record data */
	size_t offset;
};

/**
 * ice_read_pci_desc_tlv - Read a PCI device record TLV
 * @details: pointer to details structure
 * @desc: the descriptor to read
 *
 * Check if this descriptor is one of the 4 PCI-related PLDM descriptor TLVs.
 * If it is, copy its relevant data into the associated field in the record
 * details structure.
 *
 * By looping over all of the descriptor TLVs and calling this function for
 * each one, the ice_record_details will then contain a complete set of
 * identifying data for a PCI device associated with the record.
 *
 * The total offset of all TLVs is also tracked. This is required so that the
 * pointer to the package data, which follows the TLVs, can be located by the
 * caller.
 */
static void ice_read_pci_desc_tlv(struct ice_record_details *details,
				  const struct pldm_desc_tlv *desc)
{
	u16 type, size, *ptr;

	type = get_unaligned_le16(&desc->type);
	size = get_unaligned_le16(&desc->size);

	details->offset += sizeof(*desc) + size;

	switch (type) {
	case PLDM_DESC_ID_PCI_VENDOR_ID:
		ptr = &details->vendor_id;
		break;
	case PLDM_DESC_ID_PCI_DEVICE_ID:
		ptr = &details->device_id;
		break;
	case PLDM_DESC_ID_PCI_SUBVENDOR_ID:
		ptr = &details->subvendor_id;
		break;
	case PLDM_DESC_ID_PCI_SUBDEV_ID:
		ptr = &details->subdevice_id;
		break;
	default:
		/* Ignore other TLVs */
		return;
	};

	*ptr = get_unaligned_le16(desc->data);
}

/**
 * ice_find_matching_record - Find the first matching PLDM record
 * @data: storage for image details
 *
 * Search through PLDM records and find the first matching entry. It is
 * expected that only one entry matches.
 *
 * Extract relevant details about the record, including the component bitmap
 * and package data pointer.
 *
 * Returns: zero on success, or a negative error code on failure.
 */
static int ice_find_matching_record(struct ice_pldm_data *data)
{
	struct netlink_ext_ack *extack = data->extack;
	const struct pldm_record_info *record;
	struct device *dev = data->dev;
	u8 i;

	ice_for_each_pldm_record(i, record, data->record_start,
				 data->record_count) {
		const u8 *bitmap_ptr, *package_data_ptr, *desc_start;
		struct ice_record_details details = { 0 };
		u8 desc_count = record->descriptor_count;
		struct ice_hw *hw = &data->pf->hw;
		const struct pldm_desc_tlv *desc;

		details.offset = offsetof(typeof(*record),
					  variable_record_data);

		bitmap_ptr = (const u8 *)record + details.offset;

		details.offset += data->bitmap_size;
		details.offset += record->version_len;

		desc_start = (const u8 *)record + details.offset;

		/* Extract PCI device information, and locate the offset to
		 * the package data.
		 */
		ice_for_each_desc_tlv(i, desc, desc_start, desc_count)
			ice_read_pci_desc_tlv(&details, desc);

		package_data_ptr = (const u8 *)record + details.offset;

		dev_dbg(dev, "Found a record for device %04x:%04x (sub device %04x:%04x)\n",
			details.vendor_id, details.device_id,
			details.subvendor_id, details.subdevice_id);

		/* If this record matches the device, save pointers to the
		 * package data and bitmap, and exit.
		 *
		 * Note that we consider a subdevice ID of 0x0000 in the file
		 * to match any subdevice ID.
		 */
		if (details.vendor_id == hw->vendor_id &&
		    details.device_id == hw->device_id &&
		    details.subvendor_id == hw->subsystem_vendor_id &&
		    (!details.subdevice_id ||
		     details.subdevice_id == hw->subsystem_device_id)) {
			data->bitmap_ptr = bitmap_ptr;
			data->package_data_ptr = package_data_ptr;
			data->package_data_len =
				get_unaligned_le16(&record->package_data_len);

			return 0;
		}
	}

	/* If we get here, none of the records matched the device */
	dev_dbg(dev, "Unable to find a matching record in the PLDM header for this firmware file\n");
	NL_SET_ERR_MSG_MOD(extack, "No matching record in firmware file");
	return -ENOENT;
}

/**
 * ice_send_package_data - Send record package data to firmware
 * @data: storage for image details
 *
 * Send a copy of the package data associated with the matching record to
 * firmware.
 *
 * Note that this function sends an AdminQ command that will fail unless the
 * NVM resource has been acquired.
 *
 * Returns: zero on success, or an error code on failure.
 */
static int ice_send_package_data(struct ice_pldm_data *data)
{
	struct netlink_ext_ack *extack = data->extack;
	struct device *dev = data->dev;
	struct ice_pf *pf = data->pf;
	struct ice_hw *hw = &pf->hw;
	enum ice_status status;
	u8 *package_data;

	package_data = kmemdup(data->package_data_ptr, data->package_data_len,
			       GFP_KERNEL);
	if (!package_data)
		return -ENOMEM;

	status = ice_nvm_set_pkg_data(hw, false, package_data,
				      data->package_data_len, NULL);

	kfree(package_data);

	if (status) {
		dev_err(dev, "Failed to send record package data to firmware, err %s aq_err %s\n",
			ice_stat_str(status),
			ice_aq_str(hw->adminq.sq_last_status));
		NL_SET_ERR_MSG_MOD(extack, "Failed to record package data to firmware");
		return -EIO;
	}

	return 0;
}

/**
 * ice_flash_pldm_image - Write a PLDM-formatted firmware image to the device
 * @pf: private device driver structure
 * @fw: firmware object pointing to the relevant firmware file
 * @extack: netlink extended ACK structure
 *
 * Parse the data for a given firmware file, verifying that it is a valid PLDM
 * formatted image that matches this device.
 *
 * TODO: Extract the device record Package Data and Component Tables and send
 * them to the firmware. Extract and write the flash data for each of the
 * three main flash components, "fw.mgmt", "fw.undi", and "fw.netlist". Notify
 * firmware once the data is written to the inactive banks.
 *
 * TODO: remove the -EOPNOTSUPP error once the complete flash update logic has
 * been implemented.
 *
 * Returns: -EOPNOTSUPP or a negative error code on failure.
 */
int ice_flash_pldm_image(struct ice_pf *pf, const struct firmware *fw,
			 struct netlink_ext_ack *extack)
{
	struct device *dev = ice_pf_to_dev(pf);
	struct ice_hw *hw = &pf->hw;
	struct ice_pldm_data *data;
	enum ice_status status;
	int err;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = dev;
	data->extack = extack;
	data->pf = pf;

	data->fw_data = fw->data;
	data->fw_size = fw->size;

	err = ice_parse_pldm_image(data);
	if (err)
		goto out_release_data;

	err = ice_find_matching_record(data);
	if (err)
		goto out_release_data;

	status = ice_acquire_nvm(hw, ICE_RES_WRITE);
	if (status) {
		dev_err(dev, "Failed to acquire device flash lock, err %s aq_err %s\n",
			ice_stat_str(status),
			ice_aq_str(hw->adminq.sq_last_status));
		NL_SET_ERR_MSG_MOD(extack, "Failed to acquire device flash lock");
		err = -EIO;
		goto out_release_data;
	}

	err = ice_send_package_data(data);
	if (err)
		goto out_release_nvm_resource;

	/* TODO: The complete flash update is not yet fully implemented. */
	err = -EOPNOTSUPP;

out_release_nvm_resource:
	ice_release_nvm(hw);

out_release_data:
	kfree(data);

	return err;
}
