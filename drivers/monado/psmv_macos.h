// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief Enumerate and open PlayStation Move controllers through hidapi.
 *
 * Monado's PS Move driver (drivers/psmv) only knows how to get at a
 * controller through Monado's prober, which the runtime does not build. This
 * file walks hidapi's device list for Sony's two Move models instead and
 * hands each one to psmv_device_create() through a minimal prober shim that
 * opens the HID interface with os_hid_open_hidapi_path().
 *
 * The result is orientation (IMU fusion), buttons, trigger, LED and rumble.
 * Positional tracking needs the OpenCV ball tracker, which is not built.
 */

#pragma once

#include "util/u_logging.h"
#include "xrt/xrt_device.h"
#include "xrt/xrt_prober.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum oxrsys_psmv_model
{
	//! CECH-ZCM1 (2010, PS3 era): USB pairing only, Bluetooth 2.1.
	OXRSYS_PSMV_MODEL_ZCM1,
	//! CECH-ZCM2 (2016, PS4 era): pairs like any Bluetooth HID gamepad.
	OXRSYS_PSMV_MODEL_ZCM2,
};

struct oxrsys_psmv_controller
{
	//! The Monado device. Always set when open succeeds.
	struct xrt_device *xdev;

	enum oxrsys_psmv_model model;
	uint16_t vid;
	uint16_t pid;
	//! True when hidapi reported the device on the Bluetooth bus (the only
	//! transport that streams sensor data). False on USB or when hidapi is
	//! too old to say.
	bool bluetooth;
	//! hidapi serial string (the Bluetooth address on macOS), may be empty.
	char serial[64];

	/*
	 * Backing storage for the prober shim. Only valid during open.
	 */
	struct xrt_prober_device pdev;
	char *hid_path;
};

/*!
 * Result of oxrsys_psmv_open_all().
 */
enum oxrsys_psmv_open_result
{
	OXRSYS_PSMV_OPEN_OK = 0,
	//! No PS Move HID device is paired/connected.
	OXRSYS_PSMV_OPEN_NONE_FOUND,
	//! Controllers were found but every one failed to open (HID or driver).
	OXRSYS_PSMV_OPEN_ALL_FAILED,
	//! hidapi itself failed to initialise.
	OXRSYS_PSMV_OPEN_HID_FAILED,
};

/*!
 * Vendor/product match for the two Move models. Bluetooth HID devices report
 * interface_number -1 in hidapi on macOS, so this is the whole test.
 */
bool
oxrsys_psmv_classify(uint16_t vid, uint16_t pid, enum oxrsys_psmv_model *out_model);

const char *
oxrsys_psmv_model_str(enum oxrsys_psmv_model model);

const char *
oxrsys_psmv_open_result_str(enum oxrsys_psmv_open_result result);

/*!
 * Open every PS Move controller hidapi can see, up to @p capacity of them,
 * and bring each up through the Monado driver. Controllers on USB are skipped
 * with a log line: the Move only streams sensor reports over Bluetooth.
 *
 * On return @p out_controllers[0..*out_count) own their xrt_devices; release
 * each with oxrsys_psmv_close(). A controller that fails to open is logged
 * and skipped; the result is OK as long as at least one opened.
 */
enum oxrsys_psmv_open_result
oxrsys_psmv_open_all(enum u_logging_level log_level,
                     struct oxrsys_psmv_controller **out_controllers,
                     size_t capacity,
                     size_t *out_count);

void
oxrsys_psmv_close(struct oxrsys_psmv_controller **controller_ptr);

/*!
 * Tracking factory handed to the PS Move driver for every controller opened
 * afterwards (see wmr_psmv_tracking.h). NULL (the default) means orientation
 * only. The factory must outlive the controllers.
 */
void
oxrsys_psmv_set_tracking_factory(struct xrt_tracking_factory *factory);

/*!
 * Print every PS Move HID device hidapi can see, with bus and serial.
 * Diagnostic aid for pairing.
 *
 * @return number of PS Move entries printed.
 */
int
oxrsys_psmv_dump_hid_devices(void);

#ifdef __cplusplus
}
#endif
