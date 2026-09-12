// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief Enumerate and open a Windows Mixed Reality headset through hidapi.
 */

#include "wmr_macos.h"

#include "os_hid_hidapi.h"
#include "psmv_macos.h"

#include "util/u_misc.h"
#include "wmr/wmr_bt_controller.h"
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
 * Bluetooth motion controllers. hidapi cannot read a USB interface number for
 * Bluetooth HID on macOS and reports -1; newer hidapi also tags the bus.
 */
#define WMR_BLUETOOTH_INTERFACE -1

static bool
is_bt_controller_pid(uint16_t pid)
{
	return pid == WMR_CONTROLLER_PID || pid == ODYSSEY_CONTROLLER_PID || pid == REVERB_G2_CONTROLLER_PID;
}

static bool
is_bt_controller(const struct hid_device_info *info)
{
	if (info->vendor_id != MICROSOFT_VID || !is_bt_controller_pid(info->product_id)) {
		return false;
	}
#if HID_API_VERSION >= HID_API_MAKE_VERSION(0, 13, 0)
	if (info->bus_type == HID_API_BUS_BLUETOOTH) {
		return true;
	}
	if (info->bus_type != HID_API_BUS_UNKNOWN) {
		return false;
	}
#endif
	return info->interface_number == WMR_BLUETOOTH_INTERFACE;
}

/*
 * Left/right is only told apart by the product string, as in Monado's
 * wmr_prober.c. Prefix match so a firmware that appends to the name still
 * classifies.
 */
static enum xrt_device_type
classify_controller_side(const char *product)
{
	if (strncmp(product, WMR_CONTROLLER_LEFT_PRODUCT_STRING, strlen(WMR_CONTROLLER_LEFT_PRODUCT_STRING)) == 0) {
		return XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER;
	}
	if (strncmp(product, WMR_CONTROLLER_RIGHT_PRODUCT_STRING, strlen(WMR_CONTROLLER_RIGHT_PRODUCT_STRING)) ==
	    0) {
		return XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER;
	}
	return XRT_DEVICE_TYPE_UNKNOWN;
}

static const char *
controller_pid_str(uint16_t pid)
{
	switch (pid) {
	case WMR_CONTROLLER_PID: return "WMR motion controller";
	case ODYSSEY_CONTROLLER_PID: return "Samsung Odyssey controller";
	case REVERB_G2_CONTROLLER_PID: return "HP Reverb G2 controller";
	default: return "unknown controller";
	}
}

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
		enum oxrsys_psmv_model psmv_model;
		if (is_holo_sensors(info)) {
			note = info->interface_number == WMR_HOLO_INTERFACE ? "<- HoloLens Sensors (IMU interface)"
			                                                     : "HoloLens Sensors";
		} else if (classify_companion(info->vendor_id, info->product_id, &type)) {
			note = info->interface_number == WMR_COMPANION_INTERFACE ? "<- display control interface"
			                                                          : oxrsys_wmr_headset_type_str(type);
		} else if (is_bt_controller(info)) {
			switch (classify_controller_side(product)) {
			case XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER: note = "<- Bluetooth motion controller (left)"; break;
			case XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER: note = "<- Bluetooth motion controller (right)"; break;
			default: note = "Bluetooth motion controller (side unknown)"; break;
			}
		} else if (info->vendor_id == MICROSOFT_VID && is_bt_controller_pid(info->product_id)) {
			note = "WMR motion controller (not Bluetooth)";
		} else if (oxrsys_psmv_classify(info->vendor_id, info->product_id, &psmv_model)) {
			note = oxrsys_psmv_model_str(psmv_model);
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

/*
 * One Bluetooth controller candidate from the enumeration. hidapi on macOS
 * lists one entry per top-level HID collection, all sharing the device path,
 * so a controller shows up several times; the path identifies the device.
 */
struct bt_candidate
{
	const struct hid_device_info *info;
	char product[128];
};

struct bt_pair
{
	struct bt_candidate left;
	struct bt_candidate right;
};

static void
bt_pair_assign(struct bt_pair *pair, const struct hid_device_info *info, enum u_logging_level log_level)
{
	char product[128];
	wcs_to_utf8(info->product_string, product, sizeof(product));

	struct bt_candidate *slot = NULL;
	switch (classify_controller_side(product)) {
	case XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER: slot = &pair->left; break;
	case XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER: slot = &pair->right; break;
	default:
		U_LOG_IFL_W(log_level, "Ignoring %s %04x:%04x with unrecognised product string '%s'",
		            controller_pid_str(info->product_id), info->vendor_id, info->product_id, product);
		return;
	}

	if (slot->info != NULL) {
		if (strcmp(slot->info->path, info->path) != 0) {
			U_LOG_IFL_W(log_level, "Several %s devices report '%s'; using the first (%s).",
			            controller_pid_str(info->product_id), product, slot->info->path);
		}
		return;
	}
	slot->info = info;
	snprintf(slot->product, sizeof(slot->product), "%s", product);
}

static struct xrt_device *
open_bt_controller(const struct bt_candidate *candidate,
                   enum xrt_device_type controller_type,
                   enum u_logging_level log_level)
{
	const struct hid_device_info *info = candidate->info;
	if (info == NULL) {
		return NULL;
	}

	U_LOG_IFL_I(log_level, "Found %s '%s' (%04x:%04x) over Bluetooth", controller_pid_str(info->product_id),
	            candidate->product, info->vendor_id, info->product_id);

	struct os_hid_device *hid = NULL;
	int ret = os_hid_open_hidapi_path(info->path, &hid);
	if (ret != 0) {
		U_LOG_IFL_E(log_level, "Failed to open Bluetooth controller '%s' (%s): %d", candidate->product,
		            info->path, ret);
		return NULL;
	}

	// The driver owns the HID device from here on, also when it fails. It
	// reads the controller's calibration over the link, which takes a moment.
	struct xrt_device *xdev =
	    wmr_bt_controller_create(hid, controller_type, info->vendor_id, info->product_id, log_level);
	if (xdev == NULL) {
		U_LOG_IFL_E(log_level, "Monado WMR driver failed to create the Bluetooth controller '%s'.",
		            candidate->product);
	}
	return xdev;
}

int
oxrsys_wmr_open_bt_controllers(enum u_logging_level log_level,
                               struct xrt_device **out_left,
                               struct xrt_device **out_right)
{
	*out_left = NULL;
	*out_right = NULL;

	if (hid_init() != 0) {
		U_LOG_IFL_E(log_level, "hid_init failed");
		return 0;
	}

	struct hid_device_info *list = hid_enumerate(MICROSOFT_VID, 0);

	// Group by model so a matched pair is preferred, as wmr_prober.c does.
	struct bt_pair wmr = {0};
	struct bt_pair odyssey = {0};
	struct bt_pair g2 = {0};
	for (const struct hid_device_info *info = list; info != NULL; info = info->next) {
		if (!is_bt_controller(info)) {
			continue;
		}
		switch (info->product_id) {
		case WMR_CONTROLLER_PID: bt_pair_assign(&wmr, info, log_level); break;
		case ODYSSEY_CONTROLLER_PID: bt_pair_assign(&odyssey, info, log_level); break;
		case REVERB_G2_CONTROLLER_PID: bt_pair_assign(&g2, info, log_level); break;
		default: break;
		}
	}

	struct bt_pair chosen = {0};
	if (odyssey.left.info != NULL && odyssey.right.info != NULL) {
		chosen = odyssey;
	} else if (g2.left.info != NULL && g2.right.info != NULL) {
		chosen = g2;
	} else if (wmr.left.info != NULL && wmr.right.info != NULL) {
		chosen = wmr;
	} else {
		chosen.left = g2.left.info != NULL ? g2.left : odyssey.left.info != NULL ? odyssey.left : wmr.left;
		chosen.right = g2.right.info != NULL ? g2.right : odyssey.right.info != NULL ? odyssey.right : wmr.right;
	}

	if (chosen.left.info == NULL && chosen.right.info == NULL) {
		U_LOG_IFL_I(log_level, "No WMR motion controllers connected over Bluetooth.");
		hid_free_enumeration(list);
		return 0;
	}

	// Paths are only valid while the list lives.
	int opened = 0;
	*out_left = open_bt_controller(&chosen.left, XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER, log_level);
	opened += *out_left != NULL;
	*out_right = open_bt_controller(&chosen.right, XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER, log_level);
	opened += *out_right != NULL;
	hid_free_enumeration(list);

	return opened;
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

	// 5. Headsets without their own controller radio: pick up the
	// controllers paired to this machine over Bluetooth instead.
	if (headset->left == NULL && headset->right == NULL) {
		int count = oxrsys_wmr_open_bt_controllers(log_level, &headset->left, &headset->right);
		headset->controllers_bluetooth = count > 0;
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
