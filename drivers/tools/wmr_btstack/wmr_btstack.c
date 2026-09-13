// SPDX-License-Identifier: MPL-2.0
//
// wmr_btstack: pair and connect 1st-gen Windows Mixed Reality motion controllers through an
// external USB Bluetooth adapter (BTstack libusb port), and relay them to the OXRSys runtime.
//
// macOS's own Bluetooth stack runs SDP before pairing and the controllers drop out of
// pairing mode during it (see wmr_bt_pair.swift). Here we own the radio, so the order is
// the one Windows uses: bonding first (the controllers ask for legacy PIN 0000), then HID.
//
// Controllers are picked up automatically:
//   - during a pairing window (-p at start, or a PAIR command from the Home app), a
//     controller in pairing mode (hold the button under the battery cover until the LEDs
//     flash) is found by a duty-cycled inquiry and paired;
//   - a paired controller that is switched on pages the adapter itself. Page scan runs at
//     high duty, and outside a pairing window nothing else uses the radio;
//   - paired controllers that are already awake when the tool starts are paged once.
//
// The runtime's headset helper connects to a Unix socket (wmr_bt_bridge_protocol.h) and runs
// Monado's WMR controller driver on the relayed HID reports. Without a runtime client the
// tool prints the motion data instead. State for the Home app goes to
// ~/Library/Application Support/OXRSys/wmr_controllers_status.json.
//
// Usage: wmr_btstack [-p SECONDS] [-r] [-l hci_log.pklg]
//   -p  pairing window at start (default 120, 0 = only reconnect paired controllers)
//   -r  forget paired controllers

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#include "btstack.h"
#include "btstack_config.h"

#include "wmr_bt_bridge_protocol.h"

#define MAX_CONTROLLERS 2
#define MAX_SEEN 64
#define MAX_CLIENTS 8
#define NAME_PREFIX "Motion controller"

#define WMR_REPORT_STATUS 0x01 // 44-byte button + IMU payload
#define WMR_REPORT_CMD 0x06

#define OUT_QUEUE_LEN 32
#define OUT_REPORT_MAX 80
#define OUT_SPACING_MS 10 // BTstack holds one pending output report per connection
#define MAX_REPLAY 4

#define INQUIRY_UNITS 4    // 5.12 s
#define SCAN_PAUSE_MS 4000 // page-scan-only window between inquiries

typedef enum {
	SLOT_FREE,
	SLOT_BONDING,
	SLOT_CONNECTING,
	SLOT_CONNECTED,
} slot_state_t;

typedef struct {
	uint8_t len;
	uint8_t data[OUT_REPORT_MAX]; // report ID first
} out_report_t;

typedef struct {
	slot_state_t state;
	bd_addr_t addr;
	char name[64];
	uint16_t hid_cid;
	uint8_t init_step;
	btstack_timer_source_t timer;

	// output reports, flushed one at a time
	out_report_t queue[OUT_QUEUE_LEN];
	int q_head, q_count;
	uint8_t inflight[OUT_REPORT_MAX];
	uint32_t last_send_ms;
	btstack_timer_source_t flush_timer;

	// report-enable commands the runtime sent, replayed when the controller reconnects
	out_report_t replay[MAX_REPLAY];
	int replay_count;

	// stats
	uint32_t reports;
	uint32_t reports_at_last_print;
	uint32_t last_print_ms;
	uint32_t reports_at_status;
	float reports_per_second;
} controller_t;

typedef struct {
	int fd;
	char hand; // subscribed hand, 0 = none
	uint8_t rx[WMR_BRIDGE_HEADER_SIZE + WMR_BRIDGE_MAX_PAYLOAD];
	uint16_t rx_len;
	btstack_data_source_t ds;
} client_t;

static controller_t controllers[MAX_CONTROLLERS];
static client_t clients[MAX_CLIENTS];

static struct {
	bd_addr_t addr;
	uint8_t psrm;
	uint16_t clock_offset;
	uint8_t state; // 0 = name pending, 1 = name requested, 2 = done
	char name[64];
} seen[MAX_SEEN];
static int seen_count;
static bool name_request_active;
static bool inquiry_active;
static bool outgoing_active; // bonding or paging: inquiry must wait
static btstack_timer_source_t scan_timer;

static uint8_t hid_descriptor_storage[1024];
static btstack_packet_callback_registration_t hci_event_callback_registration;

static char socket_path[512];
static btstack_data_source_t listen_ds;

int wmr_pairing_seconds_at_start = 120; // set by main.c -p
static uint32_t pairing_until_ms;       // 0 = no pairing window
static char adapter_address[18];
static char status_path[512];
static btstack_timer_source_t status_timer;
static uint32_t status_last_ms;

static bd_addr_t bonded[NVM_NUM_LINK_KEYS];
static int bonded_count;
static int bonded_next;

static void schedule_scan(uint32_t delay_ms);
static void connect_next_bonded(void);

static char
hand_char(const controller_t *c)
{
	if (strstr(c->name, "Left")) return 'L';
	if (strstr(c->name, "Right")) return 'R';
	return '?';
}

static const char *
hand_of(const controller_t *c)
{
	static char s[2][2];
	static int i;
	i = (i + 1) % 2;
	s[i][0] = hand_char(c);
	s[i][1] = 0;
	return s[i];
}

static controller_t *
controller_for_addr(const bd_addr_t addr)
{
	for (int i = 0; i < MAX_CONTROLLERS; i++) {
		if (controllers[i].state != SLOT_FREE && bd_addr_cmp(controllers[i].addr, addr) == 0) {
			return &controllers[i];
		}
	}
	return NULL;
}

static controller_t *
controller_for_cid(uint16_t cid)
{
	for (int i = 0; i < MAX_CONTROLLERS; i++) {
		if (controllers[i].state != SLOT_FREE && controllers[i].hid_cid == cid) {
			return &controllers[i];
		}
	}
	return NULL;
}

static controller_t *
controller_for_hand(char hand)
{
	for (int i = 0; i < MAX_CONTROLLERS; i++) {
		if (controllers[i].state == SLOT_CONNECTED && hand_char(&controllers[i]) == hand) {
			return &controllers[i];
		}
	}
	return NULL;
}

static controller_t *
controller_alloc(const bd_addr_t addr, const char *name)
{
	for (int i = 0; i < MAX_CONTROLLERS; i++) {
		if (controllers[i].state == SLOT_FREE) {
			controller_t *c = &controllers[i];
			memset(c, 0, sizeof(*c));
			bd_addr_copy(c->addr, addr);
			snprintf(c->name, sizeof(c->name), "%s", name ? name : "controller");
			return c;
		}
	}
	return NULL;
}

static bool
all_slots_busy(void)
{
	for (int i = 0; i < MAX_CONTROLLERS; i++) {
		if (controllers[i].state == SLOT_FREE) return false;
	}
	return true;
}

static bool
pairing_active(void)
{
	return pairing_until_ms != 0 && btstack_run_loop_get_time_ms() < pairing_until_ms;
}

// --- remembered names (link keys live in BTstack's TLV; names are ours) ---------

static char names_path[512];

static bool
lookup_saved_name(const bd_addr_t addr, char *out, size_t out_len)
{
	FILE *f = fopen(names_path, "r");
	if (!f) return false;
	char line[128];
	bool found = false;
	const char *want = bd_addr_to_str(addr);
	while (fgets(line, sizeof(line), f)) {
		line[strcspn(line, "\n")] = 0;
		if (strncmp(line, want, 17) == 0 && line[17] == ' ') {
			snprintf(out, out_len, "%s", line + 18);
			found = true;
		}
	}
	fclose(f);
	return found;
}

static void
save_name(const bd_addr_t addr, const char *name)
{
	char existing[64];
	if (lookup_saved_name(addr, existing, sizeof(existing)) && strcmp(existing, name) == 0) return;
	FILE *f = fopen(names_path, "a");
	if (!f) return;
	fprintf(f, "%s %s\n", bd_addr_to_str(addr), name);
	fclose(f);
}

// --- runtime bridge: clients -------------------------------------------------

static int
subscribers_for(char hand)
{
	int n = 0;
	for (int i = 0; i < MAX_CLIENTS; i++) n += clients[i].fd > 0 && clients[i].hand == hand;
	return n;
}

static void
client_close(client_t *cl)
{
	if (cl->fd <= 0) return;
	btstack_run_loop_remove_data_source(&cl->ds);
	close(cl->fd);
	if (cl->hand) printf("[%c] runtime detached\n", cl->hand);
	memset(cl, 0, sizeof(*cl));
}

static void
client_send(client_t *cl, uint8_t type, char hand, const uint8_t *payload, uint16_t len)
{
	uint8_t frame[WMR_BRIDGE_HEADER_SIZE + WMR_BRIDGE_MAX_PAYLOAD];
	if (len > WMR_BRIDGE_MAX_PAYLOAD) len = WMR_BRIDGE_MAX_PAYLOAD;
	frame[0] = type;
	frame[1] = (uint8_t)hand;
	little_endian_store_16(frame, 2, len);
	if (len) memcpy(frame + WMR_BRIDGE_HEADER_SIZE, payload, len);
	ssize_t n = send(cl->fd, frame, WMR_BRIDGE_HEADER_SIZE + len, 0);
	if (n == (ssize_t)(WMR_BRIDGE_HEADER_SIZE + len)) return;
	if (n < 0 && errno == EAGAIN) return; // runtime is behind: drop this report, framing intact
	client_close(cl);                     // partial write or dead peer
}

static void
broadcast(uint8_t type, char hand, const uint8_t *payload, uint16_t len)
{
	for (int i = 0; i < MAX_CLIENTS; i++) {
		if (clients[i].fd > 0 && clients[i].hand == hand) client_send(&clients[i], type, hand, payload, len);
	}
}

static void queue_output(controller_t *c, const uint8_t *report, uint16_t len);

static void
remember_for_replay(controller_t *c, const uint8_t *report, uint16_t len)
{
	// 06 03 <what> ...: status / IMU report enables. The controller forgets them when the
	// link drops, the runtime's driver only sends them once at start.
	if (len < 3 || report[0] != WMR_REPORT_CMD || report[1] != 0x03) return;
	for (int i = 0; i < c->replay_count; i++) {
		if (c->replay[i].data[2] == report[2]) {
			c->replay[i].len = (uint8_t)len;
			memcpy(c->replay[i].data, report, len);
			return;
		}
	}
	if (c->replay_count < MAX_REPLAY) {
		c->replay[c->replay_count].len = (uint8_t)len;
		memcpy(c->replay[c->replay_count].data, report, len);
		c->replay_count++;
	}
}

static void
start_pairing(uint16_t seconds)
{
	if (seconds == 0) {
		pairing_until_ms = 0;
		if (inquiry_active) gap_inquiry_stop();
		printf("Pairing stopped\n");
		return;
	}
	pairing_until_ms = btstack_run_loop_get_time_ms() + (uint32_t)seconds * 1000u;
	if (pairing_until_ms == 0) pairing_until_ms = 1;
	printf("Pairing for %us: hold the button under a controller's battery cover until the LEDs flash\n",
	       seconds);
	schedule_scan(0);
}

static void
forget_all(void)
{
	printf("Forgetting all paired controllers\n");
	for (int i = 0; i < MAX_CONTROLLERS; i++) {
		if (controllers[i].state == SLOT_CONNECTED || controllers[i].state == SLOT_CONNECTING) {
			hid_host_disconnect(controllers[i].hid_cid);
		}
	}
	gap_delete_all_link_keys();
	unlink(names_path);
	bonded_count = bonded_next = 0;
	seen_count = 0;
}

static void
client_handle_frame(client_t *cl, uint8_t type, char hand, const uint8_t *payload, uint16_t len)
{
	switch (type) {
	case WMR_BRIDGE_MSG_LIST: {
		char hands[3] = {0};
		int n = 0;
		if (controller_for_hand('L')) hands[n++] = 'L';
		if (controller_for_hand('R')) hands[n++] = 'R';
		client_send(cl, WMR_BRIDGE_MSG_LIST, 0, (const uint8_t *)hands, (uint16_t)n);
		break;
	}
	case WMR_BRIDGE_MSG_SUBSCRIBE: {
		if (hand != 'L' && hand != 'R') {
			client_close(cl);
			return;
		}
		cl->hand = hand;
		controller_t *c = controller_for_hand(hand);
		printf("[%c] runtime attached%s\n", hand, c ? "" : " (controller not connected yet)");
		if (c) {
			client_send(cl, WMR_BRIDGE_MSG_CONNECTED, hand, (const uint8_t *)c->name, (uint16_t)strlen(c->name));
		} else {
			client_send(cl, WMR_BRIDGE_MSG_GONE, hand, NULL, 0);
		}
		break;
	}
	case WMR_BRIDGE_MSG_PAIR:
		start_pairing(len >= 2 ? little_endian_read_16(payload, 0) : 60);
		break;
	case WMR_BRIDGE_MSG_FORGET: forget_all(); break;
	case WMR_BRIDGE_MSG_OUTPUT: {
		controller_t *c = controller_for_hand(cl->hand);
		if (!c || len < 1 || len > OUT_REPORT_MAX) break;
		remember_for_replay(c, payload, len);
		queue_output(c, payload, len);
		break;
	}
	default: break;
	}
}

static void
client_ds_handler(btstack_data_source_t *ds, btstack_data_source_callback_type_t type)
{
	UNUSED(type);
	client_t *cl = NULL;
	for (int i = 0; i < MAX_CLIENTS; i++) {
		if (&clients[i].ds == ds) cl = &clients[i];
	}
	if (!cl) return;

	ssize_t n = recv(cl->fd, cl->rx + cl->rx_len, sizeof(cl->rx) - cl->rx_len, 0);
	if (n <= 0) {
		if (n < 0 && errno == EAGAIN) return;
		client_close(cl);
		return;
	}
	cl->rx_len += (uint16_t)n;

	while (cl->rx_len >= WMR_BRIDGE_HEADER_SIZE) {
		uint16_t len = little_endian_read_16(cl->rx, 2);
		if (len > WMR_BRIDGE_MAX_PAYLOAD) {
			client_close(cl);
			return;
		}
		uint16_t total = WMR_BRIDGE_HEADER_SIZE + len;
		if (cl->rx_len < total) break;
		client_handle_frame(cl, cl->rx[0], (char)cl->rx[1], cl->rx + WMR_BRIDGE_HEADER_SIZE, len);
		if (cl->fd <= 0) return; // closed while handling
		memmove(cl->rx, cl->rx + total, cl->rx_len - total);
		cl->rx_len -= total;
	}
}

static void
listen_ds_handler(btstack_data_source_t *ds, btstack_data_source_callback_type_t type)
{
	UNUSED(type);
	int fd = accept(btstack_run_loop_get_data_source_fd(ds), NULL, NULL);
	if (fd < 0) return;
	client_t *cl = NULL;
	for (int i = 0; i < MAX_CLIENTS; i++) {
		if (clients[i].fd <= 0) {
			cl = &clients[i];
			break;
		}
	}
	if (!cl) {
		close(fd);
		return;
	}
	fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);
	int sndbuf = 256 * 1024;
	setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
	int nosigpipe = 1;
	setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &nosigpipe, sizeof(nosigpipe));

	memset(cl, 0, sizeof(*cl));
	cl->fd = fd;
	btstack_run_loop_set_data_source_fd(&cl->ds, fd);
	btstack_run_loop_set_data_source_handler(&cl->ds, client_ds_handler);
	btstack_run_loop_enable_data_source_callbacks(&cl->ds, DATA_SOURCE_CALLBACK_READ);
	btstack_run_loop_add_data_source(&cl->ds);
}

static void
remove_socket(void)
{
	if (socket_path[0]) unlink(socket_path);
}

static void
bridge_listen(void)
{
	wmr_bridge_socket_path(socket_path, sizeof(socket_path));
	int fd = socket(AF_UNIX, SOCK_STREAM, 0);
	struct sockaddr_un sa = {.sun_family = AF_UNIX};
	if (fd < 0 || strlen(socket_path) >= sizeof(sa.sun_path)) {
		printf("Runtime bridge disabled: cannot create socket\n");
		return;
	}
	snprintf(sa.sun_path, sizeof(sa.sun_path), "%s", socket_path);
	unlink(socket_path);
	if (bind(fd, (struct sockaddr *)&sa, sizeof(sa)) != 0 || listen(fd, 4) != 0) {
		printf("Runtime bridge disabled: bind %s failed: %s\n", socket_path, strerror(errno));
		close(fd);
		return;
	}
	chmod(socket_path, 0600);
	atexit(remove_socket);
	fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);
	btstack_run_loop_set_data_source_fd(&listen_ds, fd);
	btstack_run_loop_set_data_source_handler(&listen_ds, listen_ds_handler);
	btstack_run_loop_enable_data_source_callbacks(&listen_ds, DATA_SOURCE_CALLBACK_READ);
	btstack_run_loop_add_data_source(&listen_ds);
	printf("Runtime bridge: %s\n", socket_path);
}

// --- output reports ----------------------------------------------------------

static void
flush_timer_handler(btstack_timer_source_t *ts)
{
	controller_t *c = btstack_run_loop_get_timer_context(ts);
	if (c->state != SLOT_CONNECTED) {
		c->q_count = 0;
		return;
	}
	uint32_t now = btstack_run_loop_get_time_ms();
	uint32_t since = now - c->last_send_ms;
	if (c->q_count > 0 && since >= OUT_SPACING_MS) {
		out_report_t *r = &c->queue[c->q_head];
		// BTstack keeps a pointer to the report until the channel can send; keep it stable.
		memcpy(c->inflight, r->data, r->len);
		uint8_t status = hid_host_send_report(c->hid_cid, c->inflight[0], c->inflight + 1, r->len - 1);
		if (status != ERROR_CODE_SUCCESS) {
			printf("[%s] output report %02x dropped: 0x%02x\n", hand_of(c), c->inflight[0], status);
		}
		c->q_head = (c->q_head + 1) % OUT_QUEUE_LEN;
		c->q_count--;
		c->last_send_ms = now;
		since = 0;
	}
	if (c->q_count > 0) {
		btstack_run_loop_set_timer(ts, OUT_SPACING_MS - since);
		btstack_run_loop_add_timer(ts);
	}
}

static void
queue_output(controller_t *c, const uint8_t *report, uint16_t len)
{
	if (c->q_count == OUT_QUEUE_LEN) {
		printf("[%s] output queue full, dropping report %02x\n", hand_of(c), report[0]);
		return;
	}
	out_report_t *slot = &c->queue[(c->q_head + c->q_count) % OUT_QUEUE_LEN];
	slot->len = (uint8_t)len;
	memcpy(slot->data, report, len);
	c->q_count++;
	btstack_run_loop_remove_timer(&c->flush_timer);
	btstack_run_loop_set_timer_handler(&c->flush_timer, flush_timer_handler);
	btstack_run_loop_set_timer_context(&c->flush_timer, c);
	btstack_run_loop_set_timer(&c->flush_timer, 0);
	btstack_run_loop_add_timer(&c->flush_timer);
}

static void
send_cmd(controller_t *c, uint8_t cmd_id, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4)
{
	uint8_t buf[64] = {WMR_REPORT_CMD, cmd_id, b1, b2, b3, b4};
	queue_output(c, buf, sizeof(buf));
}

// --- scanning / connecting ---------------------------------------------------

static void
scan_timer_handler(btstack_timer_source_t *ts)
{
	UNUSED(ts);
	if (!pairing_active()) {
		if (pairing_until_ms != 0) {
			pairing_until_ms = 0;
			printf("Pairing window closed; listening for paired controllers only\n");
		}
		return; // restarted by start_pairing()
	}
	if (inquiry_active || name_request_active || outgoing_active || all_slots_busy()) {
		schedule_scan(SCAN_PAUSE_MS);
		return;
	}
	if (gap_inquiry_start(INQUIRY_UNITS) == 0) inquiry_active = true;
}

static void
schedule_scan(uint32_t delay_ms)
{
	btstack_run_loop_remove_timer(&scan_timer);
	btstack_run_loop_set_timer_handler(&scan_timer, scan_timer_handler);
	btstack_run_loop_set_timer(&scan_timer, delay_ms);
	btstack_run_loop_add_timer(&scan_timer);
}

static void
connect_hid(controller_t *c)
{
	c->state = SLOT_CONNECTING;
	outgoing_active = true;
	uint8_t status = hid_host_connect(c->addr, HID_PROTOCOL_MODE_REPORT, &c->hid_cid);
	if (status != ERROR_CODE_SUCCESS) {
		printf("[%s] HID connect failed to start: 0x%02x\n", hand_of(c), status);
		c->state = SLOT_FREE;
		outgoing_active = false;
		connect_next_bonded();
	}
}

static void
connect_timer_handler(btstack_timer_source_t *ts)
{
	connect_hid(btstack_run_loop_get_timer_context(ts));
}

// Page bonded controllers once each at start-up, in case they are awake and waiting (e.g.
// the tool was restarted). A sleeping one costs a page timeout, kept short in btstack_main.
static void
connect_next_bonded(void)
{
	while (bonded_next < bonded_count) {
		const uint8_t *addr = bonded[bonded_next++];
		if (controller_for_addr(addr)) continue;
		char name[64];
		if (!lookup_saved_name(addr, name, sizeof(name))) continue; // not one of ours
		controller_t *c = controller_alloc(addr, name);
		if (!c) break;
		printf("Checking paired '%s' %s\n", name, bd_addr_to_str(addr));
		connect_hid(c);
		return;
	}
	schedule_scan(0);
}

static void
found_named_device(const bd_addr_t addr, const char *name)
{
	if (strncmp(name, NAME_PREFIX, strlen(NAME_PREFIX)) != 0) return;
	if (controller_for_addr(addr)) return;

	controller_t *c = controller_alloc(addr, name);
	if (!c) return;

	// Only controllers in pairing mode answer inquiry, so this is always a (re)pair.
	link_key_t key;
	link_key_type_t type;
	if (gap_get_link_key_for_bd_addr((uint8_t *)addr, key, &type)) {
		gap_drop_link_key_for_bd_addr((uint8_t *)addr);
	}
	printf("Found '%s' %s in pairing mode; pairing\n", name, bd_addr_to_str(addr));
	if (inquiry_active) gap_inquiry_stop();
	c->state = SLOT_BONDING;
	outgoing_active = true;
	if (gap_dedicated_bonding(c->addr, 0) != 0) {
		printf("[%s] pairing failed to start\n", hand_of(c));
		c->state = SLOT_FREE;
		outgoing_active = false;
		schedule_scan(0);
	}
}

static void
next_name_request(void)
{
	if (name_request_active || inquiry_active || outgoing_active) return;
	for (int i = 0; i < seen_count; i++) {
		if (seen[i].state == 0) {
			seen[i].state = 1;
			name_request_active = true;
			gap_remote_name_request(seen[i].addr, seen[i].psrm, seen[i].clock_offset | 0x8000);
			return;
		}
	}
	schedule_scan(SCAN_PAUSE_MS);
}

// --- WMR controller protocol --------------------------------------------------

static int32_t
rd24(const uint8_t *p)
{
	return ((int32_t)((uint32_t)p[0] << 8 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 24)) >> 8;
}

// Standalone mode only: the same sequence Monado's wmr_controller_base_init sends, minus the
// calibration read. With the runtime attached its driver does this itself.
static void
init_timer_handler(btstack_timer_source_t *ts)
{
	controller_t *c = btstack_run_loop_get_timer_context(ts);
	if (c->state != SLOT_CONNECTED) return;
	send_cmd(c, 0x00, 0x00, 0, 0, 0);    // zero / reinit
	send_cmd(c, 0x04, 0xc1, 0x02, 0, 0); // quiesce/restart tasks
	send_cmd(c, 0x03, 0x01, 0x00, 0x02, 0); // status reports on
	send_cmd(c, 0x03, 0x02, 0xe1, 0x02, 0); // IMU on
}

static void
replay_timer_handler(btstack_timer_source_t *ts)
{
	controller_t *c = btstack_run_loop_get_timer_context(ts);
	if (c->state != SLOT_CONNECTED) return;
	for (int i = 0; i < c->replay_count; i++) queue_output(c, c->replay[i].data, c->replay[i].len);
}

static void
print_motion(controller_t *c, const uint8_t *p)
{
	uint32_t t = btstack_run_loop_get_time_ms();
	if (t - c->last_print_ms < 50) return; // ~20 lines/s per controller
	double hz = c->last_print_ms ? (c->reports - c->reports_at_last_print) * 1000.0 / (t - c->last_print_ms) : 0;
	c->last_print_ms = t;
	c->reports_at_last_print = c->reports;

	uint8_t buttons = p[0];
	int stick_x = p[1] | ((p[2] & 0x0f) << 8);
	int stick_y = (p[2] >> 4) | (p[3] << 4);
	double acc[3], gyro[3];
	for (int i = 0; i < 3; i++) acc[i] = rd24(p + 8 + 3 * i) / 49000.0;   // m/s^2
	for (int i = 0; i < 3; i++) gyro[i] = rd24(p + 19 + 3 * i) * 0.00001; // rad/s
	double g = sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);

	printf("[%s] %4.0fHz acc % 6.2f % 6.2f % 6.2f |%5.2f| gyro % 6.2f % 6.2f % 6.2f  "
	       "stick %+.2f %+.2f trig %3u pad %3u,%3u btn %c%c%c%c%c%c\n",
	       hand_of(c), hz, acc[0], acc[1], acc[2], g, gyro[0], gyro[1], gyro[2], (stick_x - 0x7ff) / (double)0x7ff,
	       (stick_y - 0x7ff) / (double)0x7ff, p[4], p[5], p[6], buttons & 0x01 ? 'S' : '-',
	       buttons & 0x02 ? 'H' : '-', buttons & 0x04 ? 'M' : '-', buttons & 0x08 ? 'G' : '-',
	       buttons & 0x10 ? 'P' : '-', buttons & 0x40 ? 'T' : '-');
}

static void
handle_report(controller_t *c, const uint8_t *report, uint16_t len)
{
	// BTstack delivers the HID transaction header (0xA1 = DATA | Input) first.
	if (len < 2 || report[0] != 0xa1) return;
	const uint8_t *payload = report + 1; // report ID + data, hidapi framing
	uint16_t n = len - 1;
	char hand = hand_char(c);

	if (subscribers_for(hand) > 0) {
		broadcast(WMR_BRIDGE_MSG_INPUT, hand, payload, n);
		if (payload[0] == WMR_REPORT_STATUS) {
			c->reports++;
			uint32_t t = btstack_run_loop_get_time_ms();
			if (t - c->last_print_ms >= 10000) {
				if (c->last_print_ms) {
					printf("[%c] relaying to runtime, %.0f reports/s\n", hand,
					       (c->reports - c->reports_at_last_print) * 1000.0 / (t - c->last_print_ms));
				}
				c->last_print_ms = t;
				c->reports_at_last_print = c->reports;
			}
		}
		return;
	}

	if (payload[0] == WMR_REPORT_STATUS && n >= 45) {
		c->reports++;
		print_motion(c, payload + 1);
	}
}

static void
controller_connected(controller_t *c)
{
	c->state = SLOT_CONNECTED;
	outgoing_active = false;
	char hand = hand_char(c);
	printf("[%c] connected '%s'\n", hand, c->name);
	broadcast(WMR_BRIDGE_MSG_CONNECTED, hand, (const uint8_t *)c->name, (uint16_t)strlen(c->name));

	btstack_run_loop_set_timer_context(&c->timer, c);
	if (subscribers_for(hand) > 0) {
		btstack_run_loop_set_timer_handler(&c->timer, replay_timer_handler);
	} else {
		btstack_run_loop_set_timer_handler(&c->timer, init_timer_handler);
	}
	btstack_run_loop_set_timer(&c->timer, 300);
	btstack_run_loop_add_timer(&c->timer);
	connect_next_bonded(); // keep checking / scanning for the other hand
}

// --- status for the Home app -------------------------------------------------

static const char *
slot_state_name(slot_state_t state)
{
	switch (state) {
	case SLOT_BONDING: return "pairing";
	case SLOT_CONNECTING: return "connecting";
	case SLOT_CONNECTED: return "connected";
	default: return "free";
	}
}

static void
json_string(FILE *f, const char *s)
{
	fputc('"', f);
	for (; *s; s++) {
		if (*s == '"' || *s == '\\') fputc('\\', f);
		if ((unsigned char)*s >= 0x20) fputc(*s, f);
	}
	fputc('"', f);
}

static void
write_status(const char *adapter_state)
{
	if (!status_path[0]) return;
	char tmp[600];
	snprintf(tmp, sizeof(tmp), "%s.tmp", status_path);
	FILE *f = fopen(tmp, "w");
	if (!f) return;

	uint32_t now = btstack_run_loop_get_time_ms();
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	uint32_t pairing_left = pairing_active() ? (pairing_until_ms - now + 999) / 1000 : 0;

	fprintf(f, "{\n  \"process_id\": %d,\n  \"updated_unix_ms\": %lld,\n", (int)getpid(),
	        (long long)ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
	fprintf(f, "  \"adapter_state\": \"%s\",\n  \"adapter_address\": \"%s\",\n", adapter_state, adapter_address);
	fprintf(f, "  \"pairing_seconds_left\": %u,\n", pairing_left);
	fprintf(f, "  \"runtime_attached\": [");
	bool first = true;
	for (const char *h = "LR"; *h; h++) {
		if (subscribers_for(*h) > 0) {
			fprintf(f, "%s\"%c\"", first ? "" : ", ", *h);
			first = false;
		}
	}
	fprintf(f, "],\n  \"controllers\": [");
	first = true;
	for (int i = 0; i < MAX_CONTROLLERS; i++) {
		controller_t *c = &controllers[i];
		if (c->state == SLOT_FREE) continue;
		fprintf(f, "%s\n    {\"hand\": \"%c\", \"name\": ", first ? "" : ",", hand_char(c));
		json_string(f, c->name);
		fprintf(f, ", \"address\": \"%s\", \"state\": \"%s\", \"reports_per_second\": %.0f}", bd_addr_to_str(c->addr),
		        slot_state_name(c->state), c->state == SLOT_CONNECTED ? c->reports_per_second : 0.0f);
		first = false;
	}
	fprintf(f, "%s],\n  \"paired\": [", first ? "" : "\n  ");
	first = true;
	FILE *names = fopen(names_path, "r");
	if (names) {
		char line[128];
		while (fgets(line, sizeof(line), names)) {
			line[strcspn(line, "\n")] = 0;
			if (strlen(line) < 19 || line[17] != ' ') continue;
			bd_addr_t addr;
			link_key_t key;
			link_key_type_t type;
			line[17] = 0;
			if (!sscanf_bd_addr(line, addr) || !gap_get_link_key_for_bd_addr(addr, key, &type)) continue;
			const char *name = line + 18;
			char hand = strstr(name, "Left") ? 'L' : strstr(name, "Right") ? 'R' : '?';
			fprintf(f, "%s\n    {\"hand\": \"%c\", \"name\": ", first ? "" : ",", hand);
			json_string(f, name);
			fprintf(f, ", \"address\": \"%s\"}", line);
			first = false;
		}
		fclose(names);
	}
	fprintf(f, "%s]\n}\n", first ? "" : "\n  ");
	fclose(f);
	rename(tmp, status_path);
}

static void
status_timer_handler(btstack_timer_source_t *ts)
{
	uint32_t now = btstack_run_loop_get_time_ms();
	uint32_t dt = now - status_last_ms;
	status_last_ms = now;
	for (int i = 0; i < MAX_CONTROLLERS; i++) {
		controller_t *c = &controllers[i];
		if (dt > 0) c->reports_per_second = (c->reports - c->reports_at_status) * 1000.0f / dt;
		c->reports_at_status = c->reports;
	}
	write_status("ready");
	btstack_run_loop_set_timer(ts, 1000);
	btstack_run_loop_add_timer(ts);
}

static void
remove_status(void)
{
	if (status_path[0]) unlink(status_path);
}

//! Called by main.c when the USB adapter can't be opened or stops.
void wmr_btstack_adapter_failed(void);
void
wmr_btstack_adapter_failed(void)
{
	write_status("no_adapter");
	status_path[0] = 0; // keep the file for the Home app to read
	printf("No usable USB Bluetooth adapter (is it plugged in, and not used by another program?)\n");
	exit(2);
}

// --- events -------------------------------------------------------------------

static void
packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
	UNUSED(channel);
	UNUSED(size);
	if (packet_type != HCI_EVENT_PACKET) return;

	bd_addr_t addr;
	switch (hci_event_packet_get_type(packet)) {
	case BTSTACK_EVENT_STATE:
		if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
			gap_local_bd_addr(addr);
			printf("Adapter %s ready.\n"
			       "  Paired controller: press its Windows button.\n", bd_addr_to_str(addr));
			if (wmr_pairing_seconds_at_start > 0) {
				printf("  New controller: within %ds, hold the button under the battery cover until the LEDs flash.\n",
				       wmr_pairing_seconds_at_start);
			}
			snprintf(adapter_address, sizeof(adapter_address), "%s", bd_addr_to_str(addr));
			bridge_listen();
			status_last_ms = btstack_run_loop_get_time_ms();
			btstack_run_loop_set_timer_handler(&status_timer, status_timer_handler);
			btstack_run_loop_set_timer(&status_timer, 0);
			btstack_run_loop_add_timer(&status_timer);
			if (wmr_pairing_seconds_at_start > 0) {
				pairing_until_ms = btstack_run_loop_get_time_ms() + (uint32_t)wmr_pairing_seconds_at_start * 1000u;
			}
			btstack_link_key_iterator_t it;
			bonded_count = bonded_next = 0;
			if (gap_link_key_iterator_init(&it)) {
				link_key_t key;
				link_key_type_t type;
				while (bonded_count < NVM_NUM_LINK_KEYS &&
				       gap_link_key_iterator_get_next(&it, bonded[bonded_count], key, &type)) {
					bonded_count++;
				}
				gap_link_key_iterator_done(&it);
			}
			connect_next_bonded();
		}
		break;

	case GAP_EVENT_INQUIRY_RESULT: {
		gap_event_inquiry_result_get_bd_addr(packet, addr);
		int idx = -1;
		for (int i = 0; i < seen_count; i++) {
			if (bd_addr_cmp(seen[i].addr, addr) == 0) idx = i;
		}
		if (idx < 0 && seen_count < MAX_SEEN) {
			idx = seen_count++;
			bd_addr_copy(seen[idx].addr, addr);
			seen[idx].state = 0;
		}
		if (idx < 0) break;
		seen[idx].psrm = gap_event_inquiry_result_get_page_scan_repetition_mode(packet);
		seen[idx].clock_offset = gap_event_inquiry_result_get_clock_offset(packet);
		if (gap_event_inquiry_result_get_name_available(packet)) {
			int len = gap_event_inquiry_result_get_name_len(packet);
			if (len > 63) len = 63;
			memcpy(seen[idx].name, gap_event_inquiry_result_get_name(packet), len);
			seen[idx].name[len] = 0;
			seen[idx].state = 2;
			found_named_device(addr, seen[idx].name);
		}
		break;
	}

	case GAP_EVENT_INQUIRY_COMPLETE:
		inquiry_active = false;
		// Anything still answering inquiry is in pairing mode now; look at names again so a
		// controller put back into pairing mode is picked up.
		for (int i = 0; i < seen_count; i++) {
			if (seen[i].state == 2 && strncmp(seen[i].name, NAME_PREFIX, strlen(NAME_PREFIX)) == 0 &&
			    !controller_for_addr(seen[i].addr)) {
				seen[i].state = 0;
			}
		}
		next_name_request();
		break;

	case HCI_EVENT_REMOTE_NAME_REQUEST_COMPLETE: {
		name_request_active = false;
		hci_event_remote_name_request_complete_get_bd_addr(packet, addr);
		for (int i = 0; i < seen_count; i++) {
			if (bd_addr_cmp(seen[i].addr, addr) != 0) continue;
			seen[i].state = 2;
			if (hci_event_remote_name_request_complete_get_status(packet) == 0) {
				snprintf(seen[i].name, sizeof(seen[i].name), "%s",
				         hci_event_remote_name_request_complete_get_remote_name(packet));
				found_named_device(addr, seen[i].name);
			}
		}
		next_name_request();
		break;
	}

	case HCI_EVENT_USER_CONFIRMATION_REQUEST:
		hci_event_user_confirmation_request_get_bd_addr(packet, addr);
		gap_ssp_confirmation_response(addr);
		break;

	case HCI_EVENT_PIN_CODE_REQUEST:
		hci_event_pin_code_request_get_bd_addr(packet, addr);
		gap_pin_code_response(addr, "0000");
		break;

	case GAP_EVENT_DEDICATED_BONDING_COMPLETED: {
		gap_event_dedicated_bonding_completed_get_address(packet, addr);
		uint8_t status = gap_event_dedicated_bonding_completed_get_status(packet);
		controller_t *c = controller_for_addr(addr);
		if (!c) break;
		if (status != ERROR_CODE_SUCCESS) {
			printf("[%s] pairing failed, status 0x%02x; still scanning\n", hand_of(c), status);
			c->state = SLOT_FREE;
			outgoing_active = false;
			schedule_scan(0);
			break;
		}
		printf("[%s] paired '%s' %s\n", hand_of(c), c->name, bd_addr_to_str(addr));
		save_name(addr, c->name);
		// Dedicated bonding tears its ACL link down right after pairing; paging again while
		// that disconnect is in flight fails ("Command packet sent failed").
		c->state = SLOT_CONNECTING;
		btstack_run_loop_set_timer_handler(&c->timer, connect_timer_handler);
		btstack_run_loop_set_timer_context(&c->timer, c);
		btstack_run_loop_set_timer(&c->timer, 1500);
		btstack_run_loop_add_timer(&c->timer);
		break;
	}

	case HCI_EVENT_HID_META:
		switch (hci_event_hid_meta_get_subevent_code(packet)) {
		case HID_SUBEVENT_INCOMING_CONNECTION: {
			// A paired controller switched on and paged us.
			uint16_t cid = hid_subevent_incoming_connection_get_hid_cid(packet);
			hid_subevent_incoming_connection_get_address(packet, addr);
			controller_t *c = controller_for_addr(addr);
			if (c && c->state == SLOT_CONNECTED) {
				hid_host_decline_connection(cid);
				break;
			}
			if (!c) {
				char name[64];
				c = controller_alloc(addr, lookup_saved_name(addr, name, sizeof(name)) ? name : NULL);
			}
			if (!c) {
				hid_host_decline_connection(cid);
				break;
			}
			if (inquiry_active) gap_inquiry_stop();
			c->hid_cid = cid;
			c->state = SLOT_CONNECTING;
			printf("[%s] reconnecting '%s'\n", hand_of(c), c->name);
			hid_host_accept_connection(cid, HID_PROTOCOL_MODE_REPORT);
			break;
		}

		case HID_SUBEVENT_CONNECTION_OPENED: {
			uint16_t cid = hid_subevent_connection_opened_get_hid_cid(packet);
			controller_t *c = controller_for_cid(cid);
			if (!c || c->state == SLOT_CONNECTED) break; // incoming: already up via its first report
			uint8_t status = hid_subevent_connection_opened_get_status(packet);
			if (status != ERROR_CODE_SUCCESS) {
				// 0x04 = page timeout: not switched on.
				if (status != ERROR_CODE_PAGE_TIMEOUT) {
					printf("[%s] connection failed, status 0x%02x\n", hand_of(c), status);
				}
				c->state = SLOT_FREE;
				outgoing_active = false;
				connect_next_bonded();
				break;
			}
			controller_connected(c);
			break;
		}

		case HID_SUBEVENT_REPORT: {
			controller_t *c = controller_for_cid(hid_subevent_report_get_hid_cid(packet));
			if (!c) break;
			if (c->state == SLOT_CONNECTING) {
				// Incoming connections deliver reports before CONNECTION_OPENED (SDP for the
				// descriptor still runs); they are the same controller, so treat it as up.
				controller_connected(c);
			}
			handle_report(c, hid_subevent_report_get_report(packet), hid_subevent_report_get_report_len(packet));
			break;
		}

		case HID_SUBEVENT_CONNECTION_CLOSED: {
			controller_t *c = controller_for_cid(hid_subevent_connection_closed_get_hid_cid(packet));
			if (!c) break;
			char hand = hand_char(c);
			printf("[%c] disconnected (switched off or out of range)\n", hand);
			btstack_run_loop_remove_timer(&c->timer);
			btstack_run_loop_remove_timer(&c->flush_timer);
			c->state = SLOT_FREE;
			outgoing_active = false;
			broadcast(WMR_BRIDGE_MSG_GONE, hand, NULL, 0);
			schedule_scan(SCAN_PAUSE_MS);
			break;
		}
		default: break;
		}
		break;

	default: break;
	}
}

int btstack_main(int argc, const char *argv[]);
int
btstack_main(int argc, const char *argv[])
{
	UNUSED(argc);
	UNUSED(argv);

	const char *home = getenv("HOME");
	snprintf(status_path, sizeof(status_path), "%s/Library/Application Support/OXRSys/%s", home ? home : "/tmp",
	         WMR_BRIDGE_STATUS_FILE);
	atexit(remove_status);
	snprintf(names_path, sizeof(names_path), "%s/Library/Application Support/OXRSys/wmr_btstack_names.txt",
	         home ? home : "/tmp");

	l2cap_init();
	sdp_init();
	hid_host_init(hid_descriptor_storage, sizeof(hid_descriptor_storage));
	hid_host_register_packet_handler(packet_handler);

	gap_set_local_name("OXRSys WMR 00:00:00:00:00:00");
	gap_set_class_of_device(0x2c010c); // computer / laptop
	gap_ssp_set_io_capability(SSP_IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
	gap_ssp_set_authentication_requirement(SSP_IO_AUTHREQ_MITM_PROTECTION_NOT_REQUIRED_GENERAL_BONDING);
	gap_set_default_link_policy_settings(LM_LINK_POLICY_ENABLE_ROLE_SWITCH);
	hci_set_master_slave_policy(HCI_ROLE_MASTER);

	// Reconnects: paired controllers page us when switched on. Listen hard (40 ms window
	// every 80 ms, interlaced) and don't sit in long outgoing pages.
	gap_set_page_scan_activity(0x0080, 0x0040);
	gap_set_page_scan_type(PAGE_SCAN_MODE_INTERLACED);
	gap_set_page_timeout(0x2000); // 5.12 s
	gap_connectable_control(1);
	gap_discoverable_control(0);

	hci_event_callback_registration.callback = &packet_handler;
	hci_add_event_handler(&hci_event_callback_registration);

	setvbuf(stdout, NULL, _IOLBF, 0);
	if (hci_power_control(HCI_POWER_ON) != 0) {
		wmr_btstack_adapter_failed();
	}
	return 0;
}
