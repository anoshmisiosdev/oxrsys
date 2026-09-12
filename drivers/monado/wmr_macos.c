// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief Enumerate and open a Windows Mixed Reality headset through hidapi.
 */

#include "wmr_macos.h"

#include "os_hid_hidapi.h"

#include "util/u_misc.h"
#include "wmr/wmr_hmd.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>

#include <hidapi.h>

/*
 * USB interface numbers the driver expects, see wmr_prober.c.
 */
#define WMR_HOLO_INTERFACE 2
#define WMR_COMPANION_INTERFACE 0

/*
 * Vendor/product table copied from wmr_prober.c so both stay in step.
 */
static bool
classify_companion(uint16_t vid, uint16_t pid, enum wmr_headset_type *out_type)
{
	switch (vid) {
	case HP_VID:
		switch (pid) {
		case REVERB_G1_PID: *out_type = WMR_HEADSET_REVERB_G1; return true;
		case REVERB_G2_PID: *out_type = WMR_HEADSET_REVERB_G2; return true;
		case REVERB_G2_OMNICEPT_PID: *out_type = WMR_HEADSET_REVERB_G2; return true;
		case VR1000_PID: *out_type = WMR_HEADSET_HP_VR1000; return true;
		default: return false;
		}
	case LENOVO_VID:
		switch (pid) {
		case EXPLORER_PID: *out_type = WMR_HEADSET_LENOVO_EXPLORER; return true;
		case EXPLORER_NO_CONTROLLERS_PID: *out_type = WMR_HEADSET_LENOVO_EXPLORER; return true;
		default: return false;
		}
	case SAMSUNG_VID:
		switch (pid) {
		case ODYSSEY_PLUS_PID: *out_type = WMR_HEADSET_SAMSUNG_800ZAA; return true;
		case ODYSSEY_PID: *out_type = WMR_HEADSET_SAMSUNG_XE700X3AI; return true;
		default: return false;
		}
	case QUANTA_VID:
		switch (pid) {
		case MEDION_ERAZER_X1000_PID: *out_type = WMR_HEADSET_MEDION_ERAZER_X1000; return true;
		default: return false;
		}
	case DELL_VID:
		switch (pid) {
		case VISOR_PID: *out_type = WMR_HEADSET_DELL_VISOR; return true;
		default: return false;
		}
	case ACER_VID:
		switch (pid) {
		case AH100_PID: *out_type = WMR_HEADSET_ACER_AH100; return true;
		case AH101_PID: *out_type = WMR_HEADSET_ACER_AH101; return true;
		default: return false;
		}
	case FUJITSU_VID:
		switch (pid) {
		case FMVHDS1_PID: *out_type = WMR_HEADSET_FUJITSU_FMVHDS1; return true;
		default: return false;
		}
	default: return false;
	}
}

static bool
is_holo_sensors(const struct hid_device_info *info)
{
	return info->vendor_id == MICROSOFT_VID && info->product_id == HOLOLENS_SENSORS_PID;
}

static void
wcs_to_utf8(const wchar_t *src, char *dst, size_t dst_size)
{
	if (dst_size == 0) {
		return;
	}
	dst[0] = '\0';
	if (src == NULL) {
		return;
	}
	size_t written = wcstombs(dst, src, dst_size - 1);
	if (written == (size_t)-1) {
		dst[0] = '\0';
		return;
	}
	dst[written] = '\0';
}

const char *
oxrsys_wmr_open_result_str(enum oxrsys_wmr_open_result result)
{
	switch (result) {
	case OXRSYS_WMR_OPEN_OK: return "ok";
	case OXRSYS_WMR_OPEN_NO_HEADSET: return "no HoloLens Sensors device found";
	case OXRSYS_WMR_OPEN_NO_COMPANION: return "sensors found but no recognised display-control device";
	case OXRSYS_WMR_OPEN_HID_FAILED: return "failed to open a HID interface";
	case OXRSYS_WMR_OPEN_DRIVER_FAILED: return "driver rejected the device";
	default: return "unknown";
	}
}

const char *
oxrsys_wmr_headset_type_str(enum wmr_headset_type type)
{
	switch (type) {
	case WMR_HEADSET_GENERIC: return "Generic WMR";
	case WMR_HEADSET_HP_VR1000: return "HP VR1000";
	case WMR_HEADSET_REVERB_G1: return "HP Reverb G1";
	case WMR_HEADSET_REVERB_G2: return "HP Reverb G2";
	case WMR_HEADSET_SAMSUNG_XE700X3AI: return "Samsung Odyssey";
	case WMR_HEADSET_SAMSUNG_800ZAA: return "Samsung Odyssey+";
	case WMR_HEADSET_LENOVO_EXPLORER: return "Lenovo Explorer";
	case WMR_HEADSET_MEDION_ERAZER_X1000: return "Medion Erazer X1000";
	case WMR_HEADSET_DELL_VISOR: return "Dell Visor";
	case WMR_HEADSET_ACER_AH100: return "Acer AH100";
	case WMR_HEADSET_ACER_AH101: return "Acer AH101";
	case WMR_HEADSET_FUJITSU_FMVHDS1: return "Fujitsu FMVHDS1";
	default: return "unknown";
	}
}

void
oxrsys_wmr_dump_hid_devices(void)
{
	if (hid_init() != 0) {
		fprintf(stderr, "hid_init failed\n");
		return;
	}

	struct hid_device_info *list = hid_enumerate(0, 0);
	if (list == NULL) {
		printf("hidapi reports no HID devices.\n");
		return;
	}

	printf("%-9s %-5s %-6s %-6s %-30s %s\n", "vid:pid", "iface", "usage", "page", "product", "note");
	for (struct hid_device_info *info = list; info != NULL; info = info->next) {
		char product[128];
		wcs_to_utf8(info->product_string, product, sizeof(product));

		const char *note = "";
		enum wmr_headset_type type;
		if (is_holo_sensors(info)) {
			note = info->interface_number == WMR_HOLO_INTERFACE ? "<- HoloLens Sensors (IMU interface)"
			                                                     : "HoloLens Sensors";
		} else if (classify_companion(info->vendor_id, info->product_id, &type)) {
			note = info->interface_number == WMR_COMPANION_INTERFACE ? "<- display control interface"
			                                                          : oxrsys_wmr_headset_type_str(type);
		} else if (info->vendor_id == MICROSOFT_VID &&
		           (info->product_id == WMR_CONTROLLER_PID || info->product_id == ODYSSEY_CONTROLLER_PID ||
		            info->product_id == REVERB_G2_CONTROLLER_PID)) {
			note = "WMR motion controller";
		}

		printf("%04x:%04x %-5d 0x%04x 0x%04x %-30.30s %s\n", info->vendor_id, info->product_id,
		       info->interface_number, info->usage, info->usage_page, product, note);
	}
	hid_free_enumeration(list);
}

/*!
 * Pick the hidapi entry for @p vid:@p pid on @p interface_number. hidapi on
 * macOS reports one entry per top-level collection, so several entries can
 * share an interface; the first is the one Monado's hidraw path would open.
 */
static const struct hid_device_info *
find_interface(const struct hid_device_info *list, uint16_t vid, uint16_t pid, int interface_number)
{
	for (const struct hid_device_info *info = list; info != NULL; info = info->next) {
		if (info->vendor_id == vid && info->product_id == pid && info->interface_number == interface_number) {
			return info;
		}
	}
	return NULL;
}

enum oxrsys_wmr_open_result
oxrsys_wmr_headset_open(enum u_logging_level log_level, struct oxrsys_wmr_headset **out_headset)
{
	*out_headset = NULL;

	if (hid_init() != 0) {
		U_LOG_IFL_E(log_level, "hid_init failed");
		return OXRSYS_WMR_OPEN_HID_FAILED;
	}

	struct hid_device_info *list = hid_enumerate(0, 0);

	// 1. The IMU/tracking interface is the same on every WMR headset.
	const struct hid_device_info *holo = find_interface(list, MICROSOFT_VID, HOLOLENS_SENSORS_PID, WMR_HOLO_INTERFACE);
	if (holo == NULL) {
		bool any_holo = false;
		for (const struct hid_device_info *info = list; info != NULL; info = info->next) {
			any_holo |= is_holo_sensors(info);
		}
		if (any_holo) {
			U_LOG_IFL_E(log_level,
			            "HoloLens Sensors is present but hidapi exposes no interface %d; "
			            "run the probe with --list to see what it reports.",
			            WMR_HOLO_INTERFACE);
		} else {
			U_LOG_IFL_I(log_level, "No HoloLens Sensors device on the bus.");
		}
		hid_free_enumeration(list);
		return OXRSYS_WMR_OPEN_NO_HEADSET;
	}

	// 2. The vendor-specific companion device tells us which headset it is.
	const struct hid_device_info *companion = NULL;
	enum wmr_headset_type type = WMR_HEADSET_GENERIC;
	for (const struct hid_device_info *info = list; info != NULL; info = info->next) {
		enum wmr_headset_type candidate;
		if (!classify_companion(info->vendor_id, info->product_id, &candidate)) {
			continue;
		}
		if (info->interface_number != WMR_COMPANION_INTERFACE) {
			continue;
		}
		if (companion != NULL) {
			U_LOG_IFL_W(log_level, "Found multiple display-control devices, using the last.");
		}
		companion = info;
		type = candidate;
	}
	if (companion == NULL) {
		U_LOG_IFL_E(log_level, "Found HoloLens Sensors but no recognised display-control device.");
		hid_free_enumeration(list);
		return OXRSYS_WMR_OPEN_NO_COMPANION;
	}

	struct oxrsys_wmr_headset *headset = U_TYPED_CALLOC(struct oxrsys_wmr_headset);
	headset->type = type;
	headset->companion_vid = companion->vendor_id;
	headset->companion_pid = companion->product_id;
	wcs_to_utf8(companion->product_string, headset->companion_product, sizeof(headset->companion_product));
	headset->holo_pdev.vendor_id = holo->vendor_id;
	headset->holo_pdev.product_id = holo->product_id;
	headset->holo_pdev.bus = XRT_BUS_TYPE_USB;

	U_LOG_IFL_I(log_level, "Found %s: companion '%s' (%04x:%04x)", oxrsys_wmr_headset_type_str(type),
	            headset->companion_product, companion->vendor_id, companion->product_id);

	// 3. Open both interfaces. Paths are only valid while the list lives.
	struct os_hid_device *hid_holo = NULL;
	struct os_hid_device *hid_companion = NULL;
	int ret = os_hid_open_hidapi_path(holo->path, &hid_holo);
	if (ret != 0) {
		U_LOG_IFL_E(log_level, "Failed to open HoloLens Sensors interface %d (%s): %d", WMR_HOLO_INTERFACE,
		            holo->path, ret);
		hid_free_enumeration(list);
		free(headset);
		return OXRSYS_WMR_OPEN_HID_FAILED;
	}
	ret = os_hid_open_hidapi_path(companion->path, &hid_companion);
	if (ret != 0) {
		U_LOG_IFL_E(log_level, "Failed to open display-control interface %d (%s): %d", WMR_COMPANION_INTERFACE,
		            companion->path, ret);
		hid_holo->destroy(hid_holo);
		hid_free_enumeration(list);
		free(headset);
		return OXRSYS_WMR_OPEN_HID_FAILED;
	}
	hid_free_enumeration(list);

	// 4. Hand over to the Monado driver. It takes ownership of both HID
	// devices, including on failure.
	struct xrt_device *hand_tracker = NULL;
	wmr_hmd_create(type, hid_holo, hid_companion, &headset->holo_pdev, log_level, &headset->hmd, &hand_tracker,
	               &headset->left, &headset->right);
	if (headset->hmd == NULL) {
		U_LOG_IFL_E(log_level, "Monado WMR driver failed to create the HMD device.");
		free(headset);
		return OXRSYS_WMR_OPEN_DRIVER_FAILED;
	}
	// Hand tracking is not built; the driver never returns one here.
	if (hand_tracker != NULL) {
		xrt_device_destroy(&hand_tracker);
	}

	*out_headset = headset;
	return OXRSYS_WMR_OPEN_OK;
}

void
oxrsys_wmr_headset_close(struct oxrsys_wmr_headset **headset_ptr)
{
	struct oxrsys_wmr_headset *headset = *headset_ptr;
	if (headset == NULL) {
		return;
	}
	// Controllers reference the HMD for their transport; drop them first.
	if (headset->left != NULL) {
		xrt_device_destroy(&headset->left);
	}
	if (headset->right != NULL) {
		xrt_device_destroy(&headset->right);
	}
	if (headset->hmd != NULL) {
		xrt_device_destroy(&headset->hmd);
	}
	free(headset);
	*headset_ptr = NULL;
}
