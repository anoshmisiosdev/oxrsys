// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief Enumerate and open PlayStation Move controllers through hidapi.
 */

#include "psmv_macos.h"

#include "os_hid_hidapi.h"

#include "util/u_misc.h"
#include "xrt/xrt_prober.h"

// psmv_interface.h declares psmv_found() with a bare cJSON parameter.
#include "cjson/cJSON.h"
#include "psmv/psmv_interface.h"

#include <errno.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>

#include <hidapi.h>

/*
 * hidapi reports the transport since 0.13. Older releases cannot tell USB
 * from Bluetooth, in which case every match is opened.
 */
#if defined(HID_API_VERSION) && defined(HID_API_MAKE_VERSION) && \
    HID_API_VERSION >= HID_API_MAKE_VERSION(0, 13, 0)
#define OXRSYS_PSMV_HAVE_BUS_TYPE 1
#else
#define OXRSYS_PSMV_HAVE_BUS_TYPE 0
#endif

enum oxrsys_psmv_bus
{
	OXRSYS_PSMV_BUS_UNKNOWN,
	OXRSYS_PSMV_BUS_USB,
	OXRSYS_PSMV_BUS_BLUETOOTH,
};

static enum oxrsys_psmv_bus
bus_of(const struct hid_device_info *info)
{
#if OXRSYS_PSMV_HAVE_BUS_TYPE
	switch (info->bus_type) {
	case HID_API_BUS_USB: return OXRSYS_PSMV_BUS_USB;
	case HID_API_BUS_BLUETOOTH: return OXRSYS_PSMV_BUS_BLUETOOTH;
	default: return OXRSYS_PSMV_BUS_UNKNOWN;
	}
#else
	(void)info;
	return OXRSYS_PSMV_BUS_UNKNOWN;
#endif
}

static const char *
bus_str(enum oxrsys_psmv_bus bus)
{
	switch (bus) {
	case OXRSYS_PSMV_BUS_USB: return "usb";
	case OXRSYS_PSMV_BUS_BLUETOOTH: return "bluetooth";
	default: return "unknown";
	}
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

bool
oxrsys_psmv_classify(uint16_t vid, uint16_t pid, enum oxrsys_psmv_model *out_model)
{
	if (vid != PSMV_VID) {
		return false;
	}
	switch (pid) {
	case PSMV_PID_ZCM1: *out_model = OXRSYS_PSMV_MODEL_ZCM1; return true;
	case PSMV_PID_ZCM2: *out_model = OXRSYS_PSMV_MODEL_ZCM2; return true;
	default: return false;
	}
}

const char *
oxrsys_psmv_model_str(enum oxrsys_psmv_model model)
{
	switch (model) {
	case OXRSYS_PSMV_MODEL_ZCM1: return "PS Move ZCM1";
	case OXRSYS_PSMV_MODEL_ZCM2: return "PS Move ZCM2";
	default: return "unknown";
	}
}

const char *
oxrsys_psmv_open_result_str(enum oxrsys_psmv_open_result result)
{
	switch (result) {
	case OXRSYS_PSMV_OPEN_OK: return "ok";
	case OXRSYS_PSMV_OPEN_NONE_FOUND: return "no PS Move controller is connected";
	case OXRSYS_PSMV_OPEN_ALL_FAILED: return "every PS Move controller failed to open";
	case OXRSYS_PSMV_OPEN_HID_FAILED: return "hidapi failed to initialise";
	default: return "unknown";
	}
}

/*
 *
 * Prober shim.
 *
 * psmv_device_create() asks its xrt_prober for two things: open HID
 * interface 0 of the device, and read the serial string. The
 * xrt_prober_device it receives is embedded in oxrsys_psmv_controller, so
 * both callbacks can get back to the hidapi path and serial recorded at
 * enumeration time.
 *
 */

static struct oxrsys_psmv_controller *
controller_from_pdev(struct xrt_prober_device *xpdev)
{
	return (struct oxrsys_psmv_controller *)((char *)xpdev - offsetof(struct oxrsys_psmv_controller, pdev));
}

static int
shim_open_hid_interface(struct xrt_prober *xp,
                        struct xrt_prober_device *xpdev,
                        int iface,
                        struct os_hid_device **out_hid_dev)
{
	(void)xp;
	struct oxrsys_psmv_controller *controller = controller_from_pdev(xpdev);
	if (iface != 0 || controller->hid_path == NULL) {
		return -ENODEV;
	}
	return os_hid_open_hidapi_path(controller->hid_path, out_hid_dev);
}

static int
shim_get_string_descriptor(struct xrt_prober *xp,
                           struct xrt_prober_device *xpdev,
                           enum xrt_prober_string which_string,
                           unsigned char *out_buffer,
                           size_t max_length)
{
	(void)xp;
	struct oxrsys_psmv_controller *controller = controller_from_pdev(xpdev);
	if (which_string != XRT_PROBER_STRING_SERIAL_NUMBER || max_length == 0) {
		return 0;
	}
	// Zero means "none"; the driver then names the controller itself.
	const size_t len = strlen(controller->serial);
	if (len == 0) {
		return 0;
	}
	snprintf((char *)out_buffer, max_length, "%s", controller->serial);
	return (int)strnlen((char *)out_buffer, max_length);
}

static struct xrt_prober g_shim_prober = {
    .tracking = NULL,
    .open_hid_interface = shim_open_hid_interface,
    .get_string_descriptor = shim_get_string_descriptor,
};

/*
 *
 * Enumeration.
 *
 */

int
oxrsys_psmv_dump_hid_devices(void)
{
	if (hid_init() != 0) {
		fprintf(stderr, "hid_init failed\n");
		return 0;
	}

	struct hid_device_info *list = hid_enumerate(PSMV_VID, 0);
	int count = 0;
	printf("%-9s %-13s %-9s %-20s %-30s %s\n", "vid:pid", "model", "bus", "serial", "product", "note");
	for (struct hid_device_info *info = list; info != NULL; info = info->next) {
		enum oxrsys_psmv_model model;
		if (!oxrsys_psmv_classify(info->vendor_id, info->product_id, &model)) {
			continue;
		}
		char product[128];
		char serial[64];
		wcs_to_utf8(info->product_string, product, sizeof(product));
		wcs_to_utf8(info->serial_number, serial, sizeof(serial));

		const enum oxrsys_psmv_bus bus = bus_of(info);
		const char *note = "";
		if (bus == OXRSYS_PSMV_BUS_USB) {
			note = "USB only: no sensor data, pair over Bluetooth";
		} else if (bus == OXRSYS_PSMV_BUS_BLUETOOTH) {
			note = "<- usable";
		}
		printf("%04x:%04x %-13s %-9s %-20.20s %-30.30s %s\n", info->vendor_id, info->product_id,
		       oxrsys_psmv_model_str(model), bus_str(bus), serial, product, note);
		count++;
	}
	hid_free_enumeration(list);

	if (count == 0) {
		printf("No PS Move controller in hidapi's device list.\n");
	}
	return count;
}

enum oxrsys_psmv_open_result
oxrsys_psmv_open_all(enum u_logging_level log_level,
                     struct oxrsys_psmv_controller **out_controllers,
                     size_t capacity,
                     size_t *out_count)
{
	*out_count = 0;
	for (size_t i = 0; i < capacity; i++) {
		out_controllers[i] = NULL;
	}

	if (hid_init() != 0) {
		U_LOG_IFL_E(log_level, "hid_init failed");
		return OXRSYS_PSMV_OPEN_HID_FAILED;
	}

	struct hid_device_info *list = hid_enumerate(PSMV_VID, 0);

	size_t found = 0;
	size_t skipped_usb = 0;
	size_t failed = 0;

	for (const struct hid_device_info *info = list; info != NULL; info = info->next) {
		enum oxrsys_psmv_model model;
		if (!oxrsys_psmv_classify(info->vendor_id, info->product_id, &model)) {
			continue;
		}
		found++;

		// hidapi on macOS lists one entry per top-level collection and
		// the Move has a single one; still, never open a path twice.
		bool duplicate = false;
		for (size_t i = 0; i < *out_count; i++) {
			if (strcmp(out_controllers[i]->hid_path, info->path) == 0) {
				duplicate = true;
				break;
			}
		}
		if (duplicate) {
			continue;
		}

		const enum oxrsys_psmv_bus bus = bus_of(info);
		if (bus == OXRSYS_PSMV_BUS_USB) {
			U_LOG_IFL_W(log_level,
			            "%s is on USB (%s); the controller only streams sensor data over Bluetooth, "
			            "unplug it after pairing.",
			            oxrsys_psmv_model_str(model), info->path);
			skipped_usb++;
			continue;
		}

		if (*out_count >= capacity) {
			U_LOG_IFL_W(log_level, "More than %zu PS Move controllers connected; ignoring the rest.",
			            capacity);
			break;
		}

		struct oxrsys_psmv_controller *controller = U_TYPED_CALLOC(struct oxrsys_psmv_controller);
		controller->model = model;
		controller->vid = info->vendor_id;
		controller->pid = info->product_id;
		controller->bluetooth = bus == OXRSYS_PSMV_BUS_BLUETOOTH;
		wcs_to_utf8(info->serial_number, controller->serial, sizeof(controller->serial));
		controller->hid_path = strdup(info->path);
		controller->pdev.vendor_id = info->vendor_id;
		controller->pdev.product_id = info->product_id;
		controller->pdev.bus = XRT_BUS_TYPE_BLUETOOTH;

		U_LOG_IFL_I(log_level, "Found %s over %s (serial '%s')", oxrsys_psmv_model_str(model), bus_str(bus),
		            controller->serial);

		// The driver takes ownership of the HID device it opens through
		// the shim, including on failure.
		controller->xdev = psmv_device_create(&g_shim_prober, &controller->pdev, NULL);
		if (controller->xdev == NULL) {
			U_LOG_IFL_E(log_level, "Monado PS Move driver failed to open %s (%s).",
			            oxrsys_psmv_model_str(model), info->path);
			free(controller->hid_path);
			free(controller);
			failed++;
			continue;
		}

		out_controllers[(*out_count)++] = controller;
	}
	hid_free_enumeration(list);

	if (*out_count > 0) {
		return OXRSYS_PSMV_OPEN_OK;
	}
	if (found == 0) {
		U_LOG_IFL_I(log_level, "No PS Move controller in hidapi's device list.");
		return OXRSYS_PSMV_OPEN_NONE_FOUND;
	}
	if (failed == 0 && skipped_usb > 0) {
		U_LOG_IFL_I(log_level, "Only USB-attached PS Move controllers found (%zu).", skipped_usb);
		return OXRSYS_PSMV_OPEN_NONE_FOUND;
	}
	return OXRSYS_PSMV_OPEN_ALL_FAILED;
}

void
oxrsys_psmv_close(struct oxrsys_psmv_controller **controller_ptr)
{
	struct oxrsys_psmv_controller *controller = *controller_ptr;
	if (controller == NULL) {
		return;
	}
	if (controller->xdev != NULL) {
		xrt_device_destroy(&controller->xdev);
	}
	free(controller->hid_path);
	free(controller);
	*controller_ptr = NULL;
}
