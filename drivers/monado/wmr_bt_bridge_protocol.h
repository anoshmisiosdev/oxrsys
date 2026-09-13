// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief Wire protocol between wmr_btstack (owns a USB Bluetooth adapter) and the
 *        runtime's WMR controller transport (os_hid_wmr_bridge.c).
 *
 * macOS's Bluetooth stack can't pair 1st-gen WMR motion controllers, so they are paired
 * and connected by wmr_btstack on an external adapter instead. It relays the controllers'
 * raw HID reports over a Unix stream socket so Monado's wmr_bt_controller driver can run
 * unchanged on top of an os_hid_device.
 *
 * Every message, both directions, is a frame:
 *
 *   u8 type | u8 hand ('L', 'R', or 0) | u16 payload length (little endian) | payload
 *
 * Client -> daemon:
 *   WMR_BRIDGE_MSG_LIST       '?'  hand 0, no payload. Daemon replies with a LIST frame.
 *   WMR_BRIDGE_MSG_SUBSCRIBE  'S'  subscribe to one hand's input reports.
 *   WMR_BRIDGE_MSG_OUTPUT     'O'  output report for that hand; payload[0] is the report ID
 *                                   (hidapi hid_write framing).
 *   WMR_BRIDGE_MSG_PAIR       'P'  hand 0, payload u16 seconds: look for controllers in pairing
 *                                   mode for that long (0 stops). Outside a pairing window the
 *                                   adapter only listens for paired controllers reconnecting.
 *   WMR_BRIDGE_MSG_FORGET     'F'  hand 0, no payload: disconnect and forget all paired
 *                                   controllers.
 *
 * Daemon -> client:
 *   WMR_BRIDGE_MSG_LIST       '?'  payload: the hands currently connected, e.g. "LR".
 *   WMR_BRIDGE_MSG_INPUT      'I'  input report; payload[0] is the report ID (hidapi
 *                                   hid_read framing).
 *   WMR_BRIDGE_MSG_CONNECTED  'C'  that hand's controller is connected; payload = its name.
 *   WMR_BRIDGE_MSG_GONE       'D'  that hand's controller disconnected.
 */

#pragma once

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define WMR_BRIDGE_MSG_LIST '?'
#define WMR_BRIDGE_MSG_SUBSCRIBE 'S'
#define WMR_BRIDGE_MSG_OUTPUT 'O'
#define WMR_BRIDGE_MSG_INPUT 'I'
#define WMR_BRIDGE_MSG_CONNECTED 'C'
#define WMR_BRIDGE_MSG_GONE 'D'
#define WMR_BRIDGE_MSG_PAIR 'P'
#define WMR_BRIDGE_MSG_FORGET 'F'

#define WMR_BRIDGE_HEADER_SIZE 4
#define WMR_BRIDGE_MAX_PAYLOAD 256

/*!
 * Status for the Home app, rewritten atomically about once a second while wmr_btstack
 * runs: ~/Library/Application Support/OXRSys/wmr_controllers_status.json. See
 * docs/platforms/wmr.md for the fields.
 */
#define WMR_BRIDGE_STATUS_FILE "wmr_controllers_status.json"

//! Socket lives next to the runtime's other per-user state.
static inline void
wmr_bridge_socket_path(char *out, size_t out_len)
{
	const char *home = getenv("HOME");
	snprintf(out, out_len, "%s/Library/Application Support/OXRSys/wmr_bt.sock", home ? home : "/tmp");
}
