// SPDX-License-Identifier: MPL-2.0
/*
 * oxrsys_psmv_pair: write this Mac's Bluetooth address into a PlayStation
 * Move connected over USB, so the controller tries to connect to this Mac
 * when its PS button is pressed. This is the "psmove pair" step of
 * psmoveapi, without the rest of psmoveapi.
 *
 *   oxrsys_psmv_pair [--host XX:XX:XX:XX:XX:XX] [--wait SECONDS] [--dry-run]
 *
 * The Move only streams sensor data over Bluetooth, and it only connects to
 * the host whose address it was given. Reads the current host address back
 * from the controller (feature report 0x04), writes the new one (feature
 * report 0x05) if it differs, then optionally waits for the controller to
 * show up on the Bluetooth bus after you unplug it and press PS.
 *
 * Caveat, PS3-era Moves (ZCM1, mini-USB): they connect the way the
 * DualShock 3 does, without a pairing dialog, and macOS 12 and later no
 * longer accepts such connections in its Bluetooth HID driver. The address
 * write itself works; whether macOS lets the controller connect is what the
 * --wait check reports. PS4-era Moves (ZCM2, micro-USB) pair like any other
 * gamepad through System Settings and do not need this tool.
 */

#import <IOBluetooth/IOBluetooth.h>

#include <hidapi.h>

#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define PSMV_VID 0x054c
#define PSMV_PID_ZCM1 0x03d5
#define PSMV_PID_ZCM2 0x0c5e

#define REQ_GET_BTADDR 0x04
#define REQ_SET_BTADDR 0x05
#define BTADDR_GET_SIZE 16
#define BTADDR_SET_SIZE 23

/* Address bytes as the controller stores them: least significant first. */
typedef unsigned char btaddr_t[6];

static bool
parse_addr(const char *s, btaddr_t out)
{
	if (strlen(s) != 17) {
		return false;
	}
	for (int i = 0; i < 6; i++) {
		char *end = NULL;
		long v = strtol(s + i * 3, &end, 16);
		if (v < 0 || v > 0xff || end != s + i * 3 + 2) {
			return false;
		}
		if (i < 5 && s[i * 3 + 2] != ':' && s[i * 3 + 2] != '-') {
			return false;
		}
		out[5 - i] = (unsigned char)v;
	}
	return true;
}

static const char *
addr_str(const btaddr_t a)
{
	static char buf[18];
	snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x", a[5], a[4], a[3], a[2], a[1], a[0]);
	return buf;
}

static bool
host_address(btaddr_t out)
{
	@autoreleasepool {
		IOBluetoothHostController *hc = [IOBluetoothHostController defaultController];
		NSString *s = hc != nil ? [hc addressAsString] : nil;
		if (s == nil) {
			return false;
		}
		return parse_addr([s UTF8String], out);
	}
}

static bool
is_usb(const struct hid_device_info *info)
{
#if HID_API_VERSION >= HID_API_MAKE_VERSION(0, 13, 0)
	return info->bus_type == HID_API_BUS_USB;
#else
	(void)info;
	return true;
#endif
}

static bool
is_bluetooth(const struct hid_device_info *info)
{
#if HID_API_VERSION >= HID_API_MAKE_VERSION(0, 13, 0)
	return info->bus_type == HID_API_BUS_BLUETOOTH;
#else
	(void)info;
	return false;
#endif
}

static int
count_bluetooth_moves(void)
{
	int n = 0;
	struct hid_device_info *list = hid_enumerate(PSMV_VID, 0);
	for (struct hid_device_info *i = list; i != NULL; i = i->next) {
		if ((i->product_id == PSMV_PID_ZCM1 || i->product_id == PSMV_PID_ZCM2) && is_bluetooth(i)) {
			n++;
		}
	}
	hid_free_enumeration(list);
	return n;
}

int
main(int argc, char **argv)
{
	const char *host_override = NULL;
	int wait_s = 0;
	bool dry_run = false;
	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "--host") == 0 && i + 1 < argc) {
			host_override = argv[++i];
		} else if (strcmp(argv[i], "--wait") == 0 && i + 1 < argc) {
			wait_s = atoi(argv[++i]);
		} else if (strcmp(argv[i], "--dry-run") == 0) {
			dry_run = true;
		} else {
			fprintf(stderr, "usage: %s [--host XX:XX:XX:XX:XX:XX] [--wait SECONDS] [--dry-run]\n", argv[0]);
			return 2;
		}
	}

	btaddr_t host;
	if (host_override != NULL) {
		if (!parse_addr(host_override, host)) {
			fprintf(stderr, "Bad --host address '%s'\n", host_override);
			return 2;
		}
	} else if (!host_address(host)) {
		fprintf(stderr, "Cannot read this Mac's Bluetooth address (is Bluetooth on?). Use --host.\n");
		return 1;
	}
	printf("Mac Bluetooth address: %s\n", addr_str(host));

	if (hid_init() != 0) {
		fprintf(stderr, "hid_init failed\n");
		return 1;
	}

	int paired = 0;
	int seen = 0;
	btaddr_t handled[8];
	int handled_count = 0;
	struct hid_device_info *list = hid_enumerate(PSMV_VID, 0);
	for (struct hid_device_info *info = list; info != NULL; info = info->next) {
		if (info->product_id != PSMV_PID_ZCM1 && info->product_id != PSMV_PID_ZCM2) {
			continue;
		}
		const char *model = info->product_id == PSMV_PID_ZCM1 ? "ZCM1 (PS3 era)" : "ZCM2 (PS4 era)";
		if (!is_usb(info)) {
			continue; // Only USB-attached controllers can be written.
		}
		// hidapi lists one entry per top-level collection; the first one is enough.
		if (info->interface_number > 0) {
			continue;
		}
		seen++;
		hid_device *dev = hid_open_path(info->path);
		if (dev == NULL) {
			fprintf(stderr, "Move %s on USB: cannot open (%ls)\n", model, hid_error(NULL));
			continue;
		}

		unsigned char btg[BTADDR_GET_SIZE] = {REQ_GET_BTADDR};
		int res = hid_get_feature_report(dev, btg, sizeof(btg));
		if (res != BTADDR_GET_SIZE) {
			fprintf(stderr, "Move %s on USB: cannot read its addresses (%d: %ls)\n", model, res, hid_error(dev));
			hid_close(dev);
			continue;
		}
		btaddr_t controller, current;
		memcpy(controller, btg + 1, 6);
		memcpy(current, btg + 10, 6);
		// hidapi on macOS lists one entry per HID collection of the same
		// controller; handle each controller once.
		bool done_before = false;
		for (int i = 0; i < handled_count; i++) {
			if (memcmp(handled[i], controller, 6) == 0) done_before = true;
		}
		if (done_before) {
			hid_close(dev);
			seen--;
			continue;
		}
		if (handled_count < 8) memcpy(handled[handled_count++], controller, 6);
		printf("Move %s on USB: controller %s, currently paired to host %s\n", model, addr_str(controller),
		       addr_str(current));

		if (memcmp(current, host, 6) == 0) {
			printf("  already carries this Mac's address; nothing to write.\n");
			paired++;
		} else if (dry_run) {
			printf("  would write %s (dry run).\n", addr_str(host));
		} else {
			unsigned char bts[BTADDR_SET_SIZE] = {REQ_SET_BTADDR};
			memcpy(bts + 1, host, 6);
			res = hid_send_feature_report(dev, bts, sizeof(bts));
			if (res != BTADDR_SET_SIZE) {
				fprintf(stderr, "  writing the host address failed (%d: %ls)\n", res, hid_error(dev));
			} else {
				// Read back.
				unsigned char chk[BTADDR_GET_SIZE] = {REQ_GET_BTADDR};
				if (hid_get_feature_report(dev, chk, sizeof(chk)) == BTADDR_GET_SIZE && memcmp(chk + 10, host, 6) == 0) {
					printf("  host address written and verified.\n");
					paired++;
				} else {
					printf("  host address written, but the read-back does not match yet (try again).\n");
				}
			}
		}
		hid_close(dev);
	}
	hid_free_enumeration(list);

	if (seen == 0) {
		fprintf(stderr, "No PlayStation Move on USB. Plug the controller in with its USB cable and retry.\n");
		hid_exit();
		return 1;
	}

	if (paired > 0 && !dry_run) {
		printf("\nNow unplug the USB cable and press the PS button once.\n");
		if (wait_s > 0) {
			printf("Waiting up to %d s for it to appear over Bluetooth...\n", wait_s);
			for (int t = 0; t < wait_s; t++) {
				if (count_bluetooth_moves() > 0) {
					printf("Connected: the Move is on the Bluetooth bus. Check with oxrsys_wmr_probe --psmove.\n");
					hid_exit();
					return 0;
				}
				sleep(1);
			}
			printf("Not connected after %d s. On macOS 12 and later this is expected for a PS3-era (ZCM1)\n"
			       "Move: Apple's Bluetooth HID driver no longer accepts its connection scheme. A PS4-era\n"
			       "(ZCM2, micro-USB) Move pairs through System Settings instead.\n",
			       wait_s);
			hid_exit();
			return 3;
		}
	}
	hid_exit();
	return 0;
}
