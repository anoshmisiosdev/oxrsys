// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief os_hid_device for WMR motion controllers relayed by wmr_btstack.
 *
 * One Unix-socket connection per controller. Monado's driver reads from its own
 * thread and writes from others, so writes are serialised; reads are only ever
 * made by one thread at a time.
 *
 * If wmr_btstack restarts, reads report timeouts while the device reconnects in
 * the background instead of failing, so the driver's read thread survives.
 */

#include "os_hid_wmr_bridge.h"

#include "wmr_bt_bridge_protocol.h"

#include "util/u_misc.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdbool.h>
#include <pthread.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

struct hid_wmr_bridge
{
	struct os_hid_device base;
	char hand;
	int fd;
	pthread_mutex_t write_lock;
	uint8_t rx[WMR_BRIDGE_HEADER_SIZE + WMR_BRIDGE_MAX_PAYLOAD];
	size_t rx_len;
	int64_t next_reconnect_ms;
};

static int64_t
now_ms(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

static int
bridge_connect(void)
{
	struct sockaddr_un sa = {.sun_family = AF_UNIX};
	char path[512];
	wmr_bridge_socket_path(path, sizeof(path));
	if (strlen(path) >= sizeof(sa.sun_path)) {
		return -ENAMETOOLONG;
	}
	snprintf(sa.sun_path, sizeof(sa.sun_path), "%s", path);

	int fd = socket(AF_UNIX, SOCK_STREAM, 0);
	if (fd < 0) {
		return -errno;
	}
	int one = 1;
	setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof(one));
	if (connect(fd, (struct sockaddr *)&sa, sizeof(sa)) != 0) {
		int err = errno;
		close(fd);
		return -err;
	}
	return fd;
}

static int
send_frame(int fd, uint8_t type, char hand, const uint8_t *payload, size_t len)
{
	if (len > WMR_BRIDGE_MAX_PAYLOAD) {
		return -EMSGSIZE;
	}
	uint8_t frame[WMR_BRIDGE_HEADER_SIZE + WMR_BRIDGE_MAX_PAYLOAD];
	frame[0] = type;
	frame[1] = (uint8_t)hand;
	frame[2] = (uint8_t)(len & 0xff);
	frame[3] = (uint8_t)(len >> 8);
	if (len > 0) {
		memcpy(frame + WMR_BRIDGE_HEADER_SIZE, payload, len);
	}
	size_t total = WMR_BRIDGE_HEADER_SIZE + len;
	size_t sent = 0;
	while (sent < total) {
		ssize_t n = send(fd, frame + sent, total - sent, 0);
		if (n < 0) {
			if (errno == EINTR) continue;
			return -errno;
		}
		sent += (size_t)n;
	}
	return 0;
}

/*!
 * Take one complete frame out of the buffer. Returns its total size, 0 if none is
 * complete yet, -1 if the stream is corrupt.
 */
static int
take_frame(struct hid_wmr_bridge *b, uint8_t *type, const uint8_t **payload, uint16_t *len)
{
	if (b->rx_len < WMR_BRIDGE_HEADER_SIZE) {
		return 0;
	}
	uint16_t l = (uint16_t)(b->rx[2] | (b->rx[3] << 8));
	if (l > WMR_BRIDGE_MAX_PAYLOAD) {
		return -1;
	}
	if (b->rx_len < (size_t)(WMR_BRIDGE_HEADER_SIZE + l)) {
		return 0;
	}
	*type = b->rx[0];
	*payload = b->rx + WMR_BRIDGE_HEADER_SIZE;
	*len = l;
	return WMR_BRIDGE_HEADER_SIZE + l;
}

static void
drop_frame(struct hid_wmr_bridge *b, int size)
{
	memmove(b->rx, b->rx + size, b->rx_len - (size_t)size);
	b->rx_len -= (size_t)size;
}

static void
bridge_disconnect(struct hid_wmr_bridge *b)
{
	pthread_mutex_lock(&b->write_lock);
	if (b->fd >= 0) {
		close(b->fd);
		b->fd = -1;
	}
	b->rx_len = 0;
	pthread_mutex_unlock(&b->write_lock);
	b->next_reconnect_ms = now_ms() + 1000;
}

static bool
bridge_reconnect(struct hid_wmr_bridge *b)
{
	if (now_ms() < b->next_reconnect_ms) {
		return false;
	}
	int fd = bridge_connect();
	if (fd < 0 || send_frame(fd, WMR_BRIDGE_MSG_SUBSCRIBE, b->hand, NULL, 0) != 0) {
		if (fd >= 0) close(fd);
		b->next_reconnect_ms = now_ms() + 1000;
		return false;
	}
	pthread_mutex_lock(&b->write_lock);
	b->fd = fd;
	pthread_mutex_unlock(&b->write_lock);
	return true;
}

static int
os_wmr_bridge_read(struct os_hid_device *ohdev, uint8_t *data, size_t length, int milliseconds)
{
	struct hid_wmr_bridge *b = (struct hid_wmr_bridge *)ohdev;
	int64_t deadline = milliseconds < 0 ? INT64_MAX : now_ms() + milliseconds;

	for (;;) {
		// Deliver a buffered input report first.
		uint8_t type;
		const uint8_t *payload;
		uint16_t len;
		int size;
		while ((size = take_frame(b, &type, &payload, &len)) > 0) {
			if (type == WMR_BRIDGE_MSG_INPUT && len > 0) {
				size_t n = len < length ? len : length;
				memcpy(data, payload, n);
				drop_frame(b, size);
				return (int)n;
			}
			drop_frame(b, size); // CONNECTED / GONE: nothing to hand to the driver
		}
		if (size < 0) {
			bridge_disconnect(b);
		}

		int64_t remaining = deadline - now_ms();
		if (remaining <= 0 && milliseconds >= 0) {
			return 0;
		}
		int wait = milliseconds < 0 ? 100 : (int)(remaining < 100 ? remaining : 100);

		if (b->fd < 0 && !bridge_reconnect(b)) {
			struct timespec ts = {.tv_sec = 0, .tv_nsec = (long)wait * 1000000L};
			nanosleep(&ts, NULL);
			continue;
		}

		struct pollfd pfd = {.fd = b->fd, .events = POLLIN};
		int pr = poll(&pfd, 1, wait);
		if (pr < 0 && errno != EINTR) {
			bridge_disconnect(b);
			continue;
		}
		if (pr <= 0) {
			continue;
		}
		ssize_t n = recv(b->fd, b->rx + b->rx_len, sizeof(b->rx) - b->rx_len, 0);
		if (n <= 0) {
			if (n < 0 && errno == EINTR) continue;
			bridge_disconnect(b); // wmr_btstack went away; keep timing out until it's back
			continue;
		}
		b->rx_len += (size_t)n;
	}
}

static int
os_wmr_bridge_write(struct os_hid_device *ohdev, const uint8_t *data, size_t length)
{
	struct hid_wmr_bridge *b = (struct hid_wmr_bridge *)ohdev;
	pthread_mutex_lock(&b->write_lock);
	int ret = b->fd >= 0 ? send_frame(b->fd, WMR_BRIDGE_MSG_OUTPUT, b->hand, data, length) : -ENOTCONN;
	pthread_mutex_unlock(&b->write_lock);
	return ret == 0 ? (int)length : -1;
}

static int
os_wmr_bridge_get_feature(struct os_hid_device *ohdev, uint8_t report_num, uint8_t *data, size_t length)
{
	(void)ohdev;
	(void)report_num;
	(void)data;
	(void)length;
	return -ENOTSUP;
}

static int
os_wmr_bridge_get_feature_timeout(struct os_hid_device *ohdev, void *data, size_t length, uint32_t timeout)
{
	(void)ohdev;
	(void)data;
	(void)length;
	(void)timeout;
	return -ENOTSUP;
}

static int
os_wmr_bridge_set_feature(struct os_hid_device *ohdev, const uint8_t *data, size_t length)
{
	(void)ohdev;
	(void)data;
	(void)length;
	return -ENOTSUP;
}

static int
os_wmr_bridge_get_physical_address(struct os_hid_device *ohdev, uint8_t *data, size_t length)
{
	(void)ohdev;
	if (length > 0) {
		data[0] = '\0';
	}
	return -ENOTSUP;
}

static void
os_wmr_bridge_destroy(struct os_hid_device *ohdev)
{
	struct hid_wmr_bridge *b = (struct hid_wmr_bridge *)ohdev;
	if (b->fd >= 0) {
		close(b->fd);
	}
	pthread_mutex_destroy(&b->write_lock);
	free(b);
}

int
os_hid_wmr_bridge_list(char *out_hands)
{
	out_hands[0] = '\0';
	int fd = bridge_connect();
	if (fd < 0) {
		return -1;
	}
	int count = -1;
	if (send_frame(fd, WMR_BRIDGE_MSG_LIST, 0, NULL, 0) == 0) {
		uint8_t buf[WMR_BRIDGE_HEADER_SIZE + WMR_BRIDGE_MAX_PAYLOAD];
		size_t have = 0;
		int64_t deadline = now_ms() + 1000;
		while (now_ms() < deadline) {
			struct pollfd pfd = {.fd = fd, .events = POLLIN};
			if (poll(&pfd, 1, 100) <= 0) continue;
			ssize_t n = recv(fd, buf + have, sizeof(buf) - have, 0);
			if (n <= 0) break;
			have += (size_t)n;
			if (have < WMR_BRIDGE_HEADER_SIZE) continue;
			uint16_t len = (uint16_t)(buf[2] | (buf[3] << 8));
			if (have < (size_t)(WMR_BRIDGE_HEADER_SIZE + len)) continue;
			if (buf[0] == WMR_BRIDGE_MSG_LIST) {
				count = 0;
				for (uint16_t i = 0; i < len && count < 2; i++) {
					char h = (char)buf[WMR_BRIDGE_HEADER_SIZE + i];
					if (h == 'L' || h == 'R') out_hands[count++] = h;
				}
				out_hands[count] = '\0';
			}
			break;
		}
	}
	close(fd);
	return count;
}

int
os_hid_wmr_bridge_open(char hand, struct os_hid_device **out_hid)
{
	int fd = bridge_connect();
	if (fd < 0) {
		return fd;
	}
	int ret = send_frame(fd, WMR_BRIDGE_MSG_SUBSCRIBE, hand, NULL, 0);
	if (ret != 0) {
		close(fd);
		return ret;
	}

	struct hid_wmr_bridge *b = U_TYPED_CALLOC(struct hid_wmr_bridge);
	b->base.read = os_wmr_bridge_read;
	b->base.write = os_wmr_bridge_write;
	b->base.get_feature = os_wmr_bridge_get_feature;
	b->base.get_feature_timeout = os_wmr_bridge_get_feature_timeout;
	b->base.set_feature = os_wmr_bridge_set_feature;
	b->base.get_physical_address = os_wmr_bridge_get_physical_address;
	b->base.destroy = os_wmr_bridge_destroy;
	b->hand = hand;
	b->fd = fd;
	pthread_mutex_init(&b->write_lock, NULL);

	*out_hid = &b->base;
	return 0;
}
