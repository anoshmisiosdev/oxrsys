// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief os_hid_device for WMR motion controllers relayed by wmr_btstack.
 *
 * See wmr_bt_bridge_protocol.h. The device looks like hidapi to Monado's
 * wmr_bt_controller driver: reads return an input report with the report ID in
 * byte 0, writes take an output report with the report ID in byte 0.
 */

#pragma once

#include "os/os_hid.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * Ask a running wmr_btstack which controllers are connected.
 *
 * @param out_hands Receives up to two of 'L' / 'R', NUL terminated (needs 3 bytes).
 * @return number of connected controllers, or -1 if the bridge isn't running.
 */
int
os_hid_wmr_bridge_list(char *out_hands);

/*!
 * Open the relayed HID device for one hand ('L' or 'R').
 *
 * @return 0 on success, negative errno-style value on failure.
 */
int
os_hid_wmr_bridge_open(char hand, struct os_hid_device **out_hid);

#ifdef __cplusplus
}
#endif
