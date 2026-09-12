// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief Enumerate and open a Windows Mixed Reality headset through hidapi.
 *
 * Replaces Monado's wmr_prober.c, which relies on the udev/libusb prober
 * that the runtime does not build. The device identification table mirrors
 * wmr_prober.c so the same headsets are recognised.
 */

#pragma once

#include "util/u_logging.h"
#include "wmr/wmr_common.h"
#include "xrt/xrt_device.h"
#include "xrt/xrt_prober.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct oxrsys_wmr_headset
{
	//! Head-mounted display device. Always set when open succeeds.
	struct xrt_device *hmd;
	//! Motion controllers. Either paired through the headset's own radio
	//! (Reverb G2, Odyssey+) or, when the headset provides none, paired to
	//! this machine over Bluetooth (original WMR, Odyssey and G2
	//! controllers). NULL when none are online.
	struct xrt_device *left;
	struct xrt_device *right;
	//! True when @c left / @c right came over Bluetooth rather than the
	//! headset's radio.
	bool controllers_bluetooth;

	enum wmr_headset_type type;

	//! Companion (display control) interface, as reported by hidapi.
	uint16_t companion_vid;
	uint16_t companion_pid;
	char companion_product[128];

	//! Backing storage for the prober device the driver keeps a pointer to.
	struct xrt_prober_device holo_pdev;
};

/*!
 * Result of oxrsys_wmr_headset_open().
 */
enum oxrsys_wmr_open_result
{
	OXRSYS_WMR_OPEN_OK = 0,
	//! No "HoloLens Sensors" HID device on the bus.
	OXRSYS_WMR_OPEN_NO_HEADSET,
	//! Sensors found but no recognised display-control device.
	OXRSYS_WMR_OPEN_NO_COMPANION,
	//! A required HID interface could not be opened.
	OXRSYS_WMR_OPEN_HID_FAILED,
	//! The driver rejected the device (see the WMR log).
	OXRSYS_WMR_OPEN_DRIVER_FAILED,
};

const char *
oxrsys_wmr_open_result_str(enum oxrsys_wmr_open_result result);

const char *
oxrsys_wmr_headset_type_str(enum wmr_headset_type type);

/*!
 * Find the first WMR headset on the bus and bring it up through the Monado
 * driver. On success @p out_headset owns the xrt_devices; release it with
 * oxrsys_wmr_headset_close().
 *
 * When the headset does not bring its own controllers (everything except
 * the Reverb G2 and Odyssey+), the controllers paired to this machine over
 * Bluetooth are opened as well, see oxrsys_wmr_open_bt_controllers().
 */
enum oxrsys_wmr_open_result
oxrsys_wmr_headset_open(enum u_logging_level log_level, struct oxrsys_wmr_headset **out_headset);

void
oxrsys_wmr_headset_close(struct oxrsys_wmr_headset **headset_ptr);

/*!
 * Find the WMR motion controllers paired to this machine over Bluetooth
 * (Microsoft 045e:065b, Odyssey 045e:065d, Reverb G2 045e:066a) and bring
 * them up through the Monado Bluetooth controller driver. They provide IMU
 * orientation (3DoF), buttons, triggers, thumbstick and trackpad; their
 * position is a fixed placeholder because constellation tracking is not
 * built.
 *
 * Works without a headset. A controller that cannot be opened is logged and
 * skipped. The caller owns the returned devices and releases them with
 * xrt_device_destroy(); either pointer is NULL when that side is absent.
 *
 * @return Number of controllers opened, 0 to 2.
 */
int
oxrsys_wmr_open_bt_controllers(enum u_logging_level log_level,
                               struct xrt_device **out_left,
                               struct xrt_device **out_right);

/*!
 * Print every HID device hidapi can see, marking the ones the driver would
 * use. Diagnostic aid for bring-up.
 */
void
oxrsys_wmr_dump_hid_devices(void);

#ifdef __cplusplus
}
#endif
