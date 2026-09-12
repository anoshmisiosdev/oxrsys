// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief hidapi backend for Monado's os_hid_device interface.
 *
 * Monado only ships a Linux hidraw implementation of os/os_hid.h. This
 * backend gives the vendored drivers the same interface on macOS (and any
 * other platform hidapi supports) without touching the Monado sources.
 */

#pragma once

#include "os/os_hid.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * Open a HID interface by its hidapi path (as returned in
 * hid_device_info::path by hid_enumerate()).
 *
 * @return 0 on success, negative errno-style value on failure.
 */
int
os_hid_open_hidapi_path(const char *path, struct os_hid_device **out_hid);

#ifdef __cplusplus
}
#endif
