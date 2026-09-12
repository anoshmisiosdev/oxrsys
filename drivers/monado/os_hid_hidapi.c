// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief hidapi backend for Monado's os_hid_device interface.
 *
 * Report framing follows hidapi's conventions, which match what Monado's
 * hidraw backend delivers: reads and writes carry the report ID in byte 0 for
 * numbered reports, feature reports take the report ID in byte 0 both ways.
 */

#include "os_hid_hidapi.h"

#include "util/u_misc.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <hidapi.h>
#ifdef __APPLE__
#include <hidapi_darwin.h>
#endif

struct hid_hidapi
{
	struct os_hid_device base;
	hid_device *dev;
};

static int
os_hidapi_read(struct os_hid_device *ohdev, uint8_t *data, size_t length, int milliseconds)
{
	struct hid_hidapi *hdev = (struct hid_hidapi *)ohdev;
	// hidapi: <0 blocks, 0 polls, >0 waits. Returns 0 on timeout, -1 on error.
	return hid_read_timeout(hdev->dev, data, length, milliseconds);
}

static int
os_hidapi_write(struct os_hid_device *ohdev, const uint8_t *data, size_t length)
{
	struct hid_hidapi *hdev = (struct hid_hidapi *)ohdev;
	return hid_write(hdev->dev, data, length);
}

static int
os_hidapi_get_feature(struct os_hid_device *ohdev, uint8_t report_num, uint8_t *data, size_t length)
{
	struct hid_hidapi *hdev = (struct hid_hidapi *)ohdev;
	data[0] = report_num;
	return hid_get_feature_report(hdev->dev, data, length);
}

static int
os_hidapi_get_feature_timeout(struct os_hid_device *ohdev, void *data, size_t length, uint32_t timeout)
{
	struct hid_hidapi *hdev = (struct hid_hidapi *)ohdev;
	int ret = -1;

	// Mirror the hidraw backend: retry roughly once per millisecond until the
	// device answers or the budget is spent.
	for (uint32_t i = 0; i < timeout; i++) {
		ret = hid_get_feature_report(hdev->dev, (unsigned char *)data, length);
		if (ret >= 0) {
			break;
		}
		struct timespec ts = {.tv_sec = 0, .tv_nsec = 1000000};
		nanosleep(&ts, NULL);
	}
	return ret;
}

static int
os_hidapi_set_feature(struct os_hid_device *ohdev, const uint8_t *data, size_t length)
{
	struct hid_hidapi *hdev = (struct hid_hidapi *)ohdev;
	return hid_send_feature_report(hdev->dev, data, length);
}

static int
os_hidapi_get_physical_address(struct os_hid_device *ohdev, uint8_t *data, size_t length)
{
	// hidapi has no portable equivalent of HIDIOCGRAWPHYS; nothing in the
	// vendored drivers needs it.
	(void)ohdev;
	if (length > 0) {
		data[0] = '\0';
	}
	return -ENOTSUP;
}

static void
os_hidapi_destroy(struct os_hid_device *ohdev)
{
	struct hid_hidapi *hdev = (struct hid_hidapi *)ohdev;
	hid_close(hdev->dev);
	free(hdev);
}

int
os_hid_open_hidapi_path(const char *path, struct os_hid_device **out_hid)
{
	if (hid_init() != 0) {
		return -EIO;
	}

#ifdef __APPLE__
	// The default on macOS is to seize the device. The headset's HID
	// interfaces are shared with IOHIDFamily, so open them cooperatively.
	hid_darwin_set_open_exclusive(0);
#endif

	hid_device *dev = hid_open_path(path);
	if (dev == NULL) {
		return -ENODEV;
	}

	struct hid_hidapi *hdev = U_TYPED_CALLOC(struct hid_hidapi);
	hdev->base.read = os_hidapi_read;
	hdev->base.write = os_hidapi_write;
	hdev->base.get_feature = os_hidapi_get_feature;
	hdev->base.get_feature_timeout = os_hidapi_get_feature_timeout;
	hdev->base.set_feature = os_hidapi_set_feature;
	hdev->base.get_physical_address = os_hidapi_get_physical_address;
	hdev->base.destroy = os_hidapi_destroy;
	hdev->dev = dev;

	*out_hid = &hdev->base;
	return 0;
}
