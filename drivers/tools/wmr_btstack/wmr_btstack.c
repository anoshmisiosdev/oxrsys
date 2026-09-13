// SPDX-License-Identifier: MPL-2.0
//
// wmr_btstack: pair 1st-gen Windows Mixed Reality motion controllers through an external
// USB Bluetooth adapter (BTstack libusb port) and print their motion data.
//
// macOS's own Bluetooth stack runs SDP before pairing and the controllers drop out of
// pairing mode during it (see wmr_bt_pair.swift). Here we own the radio, so the order
// is the one Windows uses: dedicated bonding (SSP "just works") first, then HID.
//
// Usage: wmr_btstack [-u USBPATH] [-r]   (BTstack port options; -r forgets bondings)
//   Put a controller in pairing mode (battery cover off, hold the pairing button until
//   the LEDs flash). Already-bonded controllers reconnect when you press their Windows
//   button. Ctrl-C to quit.

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "btstack.h"
#include "btstack_config.h"

#define MAX_CONTROLLERS 2
#define MAX_SEEN 64
#define NAME_PREFIX "Motion controller"

#define WMR_REPORT_STATUS 0x01 // 44-byte button + IMU payload
#define WMR_REPORT_CMD 0x06

typedef enum {
	SLOT_FREE,
	SLOT_BONDING,
	SLOT_CONNECTING,
	SLOT_CONNECTED,
} slot_state_t;

typedef struct {
	slot_state_t state;
	bd_addr_t addr;
	char name[64];
	uint16_t hid_cid;
	uint8_t init_step;
	btstack_timer_source_t init_timer;
	// stats
	uint32_t reports;
	uint32_t reports_at_last_print;
	uint64_t last_print_ms;
	uint64_t last_ticks;
} controller_t;

static controller_t controllers[MAX_CONTROLLERS];

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

static uint8_t hid_descriptor_storage[1024];
static btstack_packet_callback_registration_t hci_event_callback_registration;

static uint64_t
now_ms(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
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

static const char *
hand_of(const controller_t *c)
{
	if (strstr(c->name, "Left")) return "L";
	if (strstr(c->name, "Right")) return "R";
	return "?";
}

static void
start_inquiry(void)
{
	if (inquiry_active || name_request_active) return;
	int free_slots = 0;
	for (int i = 0; i < MAX_CONTROLLERS; i++) free_slots += controllers[i].state == SLOT_FREE;
	if (free_slots == 0) return;
	if (gap_inquiry_start(8) == 0) inquiry_active = true;
}

static void
connect_hid(controller_t *c)
{
	c->state = SLOT_CONNECTING;
	uint8_t status = hid_host_connect(c->addr, HID_PROTOCOL_MODE_REPORT, &c->hid_cid);
	printf("[%s] HID connect -> 0x%02x\n", hand_of(c), status);
	if (status != ERROR_CODE_SUCCESS) {
		c->state = SLOT_FREE;
		start_inquiry();
	}
}

// --- remembered names (link keys live in BTstack's TLV; names are ours) ---------

static char names_path[512];

static void
names_path_init(void)
{
	const char *home = getenv("HOME");
	snprintf(names_path, sizeof(names_path), "%s/Library/Application Support/oxrsys/wmr_btstack_names.txt",
	         home ? home : "/tmp");
}

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

// --- reconnecting bonded controllers ------------------------------------------

static bd_addr_t bonded[NVM_NUM_LINK_KEYS];
static int bonded_count;
static int bonded_next;

static void
connect_timer_handler(btstack_timer_source_t *ts)
{
	controller_t *c = btstack_run_loop_get_timer_context(ts);
	connect_hid(c);
}

static void
connect_later(controller_t *c, uint32_t delay_ms)
{
	c->state = SLOT_CONNECTING;
	btstack_run_loop_set_timer_handler(&c->init_timer, connect_timer_handler);
	btstack_run_loop_set_timer_context(&c->init_timer, c);
	btstack_run_loop_set_timer(&c->init_timer, delay_ms);
	btstack_run_loop_add_timer(&c->init_timer);
}

// Try bonded controllers one at a time (paging a switched-off one takes ~5 s to time out),
// then fall back to scanning for controllers in pairing mode.
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
		printf("Trying paired '%s' %s (press its Windows button if it's asleep)\n", name, bd_addr_to_str(addr));
		connect_hid(c);
		return;
	}
	start_inquiry();
}

static void
found_named_device(const bd_addr_t addr, const char *name)
{
	if (strncmp(name, NAME_PREFIX, strlen(NAME_PREFIX)) != 0) return;
	if (controller_for_addr(addr)) return;

	controller_t *c = controller_alloc(addr, name);
	if (!c) return;

	link_key_t key;
	link_key_type_t type;
	if (gap_get_link_key_for_bd_addr((uint8_t *)addr, key, &type)) {
		printf("Found bonded '%s' %s in pairing mode; re-bonding\n", name, bd_addr_to_str(addr));
		gap_drop_link_key_for_bd_addr((uint8_t *)addr);
	} else {
		printf("Found '%s' %s in pairing mode; bonding first (no SDP yet)\n", name, bd_addr_to_str(addr));
	}
	if (inquiry_active) {
		gap_inquiry_stop();
	}
	c->state = SLOT_BONDING;
	int r = gap_dedicated_bonding(c->addr, 0);
	if (r != 0) {
		printf("[%s] gap_dedicated_bonding failed to start: %d\n", hand_of(c), r);
		c->state = SLOT_FREE;
	}
}

static void
next_name_request(void)
{
	if (name_request_active || inquiry_active) return;
	for (int i = 0; i < seen_count; i++) {
		if (seen[i].state == 0) {
			seen[i].state = 1;
			name_request_active = true;
			gap_remote_name_request(seen[i].addr, seen[i].psrm, seen[i].clock_offset | 0x8000);
			return;
		}
	}
	start_inquiry();
}

// --- WMR controller protocol --------------------------------------------------

static int32_t
rd24(const uint8_t *p)
{
	return ((int32_t)((uint32_t)p[0] << 8 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 24)) >> 8;
}

static void
send_cmd(controller_t *c, uint8_t cmd_id, uint8_t block_id, uint32_t addr_or_arg, bool raw_fw)
{
	uint8_t buf[63];
	memset(buf, 0, sizeof(buf));
	if (raw_fw) {
		// wmr_controller_fw_cmd: prefix(0x06) cmd_id block_id le32 addr, padded to 64
		buf[0] = cmd_id;
		buf[1] = block_id;
		little_endian_store_32(buf, 2, addr_or_arg);
	} else {
		buf[0] = cmd_id;
		buf[1] = block_id;
		buf[2] = addr_or_arg & 0xff;
		buf[3] = (addr_or_arg >> 8) & 0xff;
	}
	uint8_t status = hid_host_send_report(c->hid_cid, WMR_REPORT_CMD, buf, sizeof(buf));
	if (status != ERROR_CODE_SUCCESS) {
		printf("[%s] send report 06 %02x %02x -> 0x%02x\n", hand_of(c), cmd_id, block_id, status);
	}
}

// Same sequence Monado's wmr_controller_base_init sends (minus the config-block read).
static void
init_timer_handler(btstack_timer_source_t *ts)
{
	controller_t *c = btstack_run_loop_get_timer_context(ts);
	if (c->state != SLOT_CONNECTED) return;
	switch (c->init_step++) {
	case 0: send_cmd(c, 0x00, 0x00, 0, true); break;             // zero / reinit
	case 1: send_cmd(c, 0x04, 0xc1, 0x02, true); break;          // quiesce/restart tasks
	case 2: send_cmd(c, 0x03, 0x01, 0x0200, false); break;       // 06 03 01 00 02: status reports on
	case 3:
		send_cmd(c, 0x03, 0x02, 0x02e1, false);                  // 06 03 02 e1 02: IMU on
		printf("[%s] sent status + IMU enable; waiting for motion reports\n", hand_of(c));
		return;
	default: return;
	}
	btstack_run_loop_set_timer(ts, 150);
	btstack_run_loop_add_timer(ts);
}

static void
handle_report(controller_t *c, const uint8_t *report, uint16_t len)
{
	// BTstack delivers the HID transaction header (0xA1 = DATA | Input) first.
	if (len < 2 || report[0] != 0xa1) return;
	uint8_t id = report[1];
	const uint8_t *p = report + 2;
	uint16_t n = len - 2;

	if (id != WMR_REPORT_STATUS) {
		printf("[%s] report 0x%02x (%u bytes):", hand_of(c), id, n);
		for (uint16_t i = 0; i < n && i < 24; i++) printf(" %02x", p[i]);
		printf("\n");
		return;
	}
	if (n < 44) {
		printf("[%s] short status report (%u bytes)\n", hand_of(c), n);
		return;
	}

	c->reports++;
	uint64_t t = now_ms();
	if (t - c->last_print_ms < 50) return; // ~20 lines/s per controller
	double hz = c->last_print_ms ? (c->reports - c->reports_at_last_print) * 1000.0 / (t - c->last_print_ms) : 0;
	c->last_print_ms = t;
	c->reports_at_last_print = c->reports;

	uint8_t buttons = p[0];
	int stick_x = p[1] | ((p[2] & 0x0f) << 8);
	int stick_y = (p[2] >> 4) | (p[3] << 4);
	uint8_t trigger = p[4];
	uint8_t pad_x = p[5], pad_y = p[6];
	uint8_t battery = p[7];

	double acc[3], gyro[3];
	for (int i = 0; i < 3; i++) acc[i] = rd24(p + 8 + 3 * i) / 49000.0;     // m/s^2
	int16_t temp = (int16_t)little_endian_read_16(p, 17);
	for (int i = 0; i < 3; i++) gyro[i] = rd24(p + 19 + 3 * i) * 0.00001;   // rad/s
	uint32_t ticks = little_endian_read_32(p, 28);                          // 100 ns/tick
	double g = sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);

	printf("[%s] %5.0fHz acc % 7.2f % 7.2f % 7.2f |%5.2f| gyro % 6.2f % 6.2f % 6.2f  "
	       "stick %+.2f %+.2f trig %3u pad %3u,%3u btn %c%c%c%c%c%c bat %3u%% t %u temp %d\n",
	       hand_of(c), hz, acc[0], acc[1], acc[2], g, gyro[0], gyro[1], gyro[2],
	       (stick_x - 0x7ff) / (double)0x7ff, (stick_y - 0x7ff) / (double)0x7ff, trigger,
	       pad_x, pad_y,
	       buttons & 0x01 ? 'S' : '-', buttons & 0x02 ? 'H' : '-', buttons & 0x04 ? 'M' : '-',
	       buttons & 0x08 ? 'G' : '-', buttons & 0x10 ? 'P' : '-', buttons & 0x40 ? 'T' : '-',
	       battery, ticks, temp);
	c->last_ticks = ticks;
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
			printf("Adapter %s ready. Put a WMR controller in pairing mode "
			       "(hold the button under the battery cover until the LEDs flash),\n"
			       "or press the Windows button on an already-paired one.\n",
			       bd_addr_to_str(addr));
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
			seen[idx].psrm = gap_event_inquiry_result_get_page_scan_repetition_mode(packet);
			seen[idx].clock_offset = gap_event_inquiry_result_get_clock_offset(packet);
			seen[idx].state = 0;
		}
		if (idx >= 0 && gap_event_inquiry_result_get_name_available(packet)) {
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
		// Forget unnamed devices from earlier rounds so a controller switched into pairing
		// mode later is looked up again.
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
		// Bonding in progress takes the radio; otherwise keep resolving / scanning.
		bool bonding = false;
		for (int i = 0; i < MAX_CONTROLLERS; i++) bonding |= controllers[i].state == SLOT_BONDING;
		if (!bonding) next_name_request();
		break;
	}

	case HCI_EVENT_USER_CONFIRMATION_REQUEST:
		hci_event_user_confirmation_request_get_bd_addr(packet, addr);
		printf("SSP confirmation (%06" PRIu32 ") from %s; accepting\n", little_endian_read_32(packet, 8),
		       bd_addr_to_str(addr));
		gap_ssp_confirmation_response(addr);
		break;

	case HCI_EVENT_PIN_CODE_REQUEST:
		hci_event_pin_code_request_get_bd_addr(packet, addr);
		printf("Legacy PIN request from %s; replying 0000\n", bd_addr_to_str(addr));
		gap_pin_code_response(addr, "0000");
		break;

	case GAP_EVENT_DEDICATED_BONDING_COMPLETED: {
		gap_event_dedicated_bonding_completed_get_address(packet, addr);
		uint8_t status = gap_event_dedicated_bonding_completed_get_status(packet);
		controller_t *c = controller_for_addr(addr);
		if (!c) break;
		if (status != ERROR_CODE_SUCCESS) {
			printf("[%s] bonding failed, status 0x%02x; still scanning\n", hand_of(c), status);
			c->state = SLOT_FREE;
			next_name_request();
			break;
		}
		printf("[%s] PAIRED '%s' %s\n", hand_of(c), c->name, bd_addr_to_str(addr));
		save_name(addr, c->name);
		// Dedicated bonding tears its ACL link down right after pairing; paging again while
		// that disconnect is still in flight fails ("Command packet sent failed").
		connect_later(c, 1500);
		break;
	}

	case HCI_EVENT_HID_META:
		switch (hci_event_hid_meta_get_subevent_code(packet)) {
		case HID_SUBEVENT_INCOMING_CONNECTION: {
			uint16_t cid = hid_subevent_incoming_connection_get_hid_cid(packet);
			hid_subevent_incoming_connection_get_address(packet, addr);
			controller_t *c = controller_for_addr(addr);
			if (!c) c = controller_alloc(addr, NULL);
			if (!c) {
				hid_host_decline_connection(cid);
				break;
			}
			// Pull a friendly name from the inquiry cache if we have one.
			for (int i = 0; i < seen_count; i++) {
				if (bd_addr_cmp(seen[i].addr, addr) == 0 && seen[i].name[0]) {
					snprintf(c->name, sizeof(c->name), "%s", seen[i].name);
				}
			}
			c->hid_cid = cid;
			c->state = SLOT_CONNECTING;
			printf("Incoming HID connection from %s\n", bd_addr_to_str(addr));
			hid_host_accept_connection(cid, HID_PROTOCOL_MODE_REPORT);
			break;
		}

		case HID_SUBEVENT_CONNECTION_OPENED: {
			uint16_t cid = hid_subevent_connection_opened_get_hid_cid(packet);
			controller_t *c = controller_for_cid(cid);
			if (!c) break;
			uint8_t status = hid_subevent_connection_opened_get_status(packet);
			if (status != ERROR_CODE_SUCCESS) {
				printf("[%s] HID connection failed, status 0x%02x\n", hand_of(c), status);
				c->state = SLOT_FREE;
				connect_next_bonded();
				break;
			}
			c->state = SLOT_CONNECTED;
			c->init_step = 0;
			printf("[%s] HID connected; initialising controller\n", hand_of(c));
			btstack_run_loop_set_timer_handler(&c->init_timer, init_timer_handler);
			btstack_run_loop_set_timer_context(&c->init_timer, c);
			btstack_run_loop_set_timer(&c->init_timer, 300);
			btstack_run_loop_add_timer(&c->init_timer);
			start_inquiry(); // look for the other hand
			break;
		}

		case HID_SUBEVENT_DESCRIPTOR_AVAILABLE:
			break;

		case HID_SUBEVENT_REPORT: {
			controller_t *c = controller_for_cid(hid_subevent_report_get_hid_cid(packet));
			if (c) handle_report(c, hid_subevent_report_get_report(packet), hid_subevent_report_get_report_len(packet));
			break;
		}

		case HID_SUBEVENT_CONNECTION_CLOSED: {
			controller_t *c = controller_for_cid(hid_subevent_connection_closed_get_hid_cid(packet));
			if (!c) break;
			printf("[%s] disconnected (%u motion reports received)\n", hand_of(c), c->reports);
			btstack_run_loop_remove_timer(&c->init_timer);
			c->state = SLOT_FREE;
			start_inquiry();
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

	names_path_init();
	l2cap_init();
	sdp_init();
	hid_host_init(hid_descriptor_storage, sizeof(hid_descriptor_storage));
	hid_host_register_packet_handler(packet_handler);

	gap_set_local_name("OXRSys WMR 00:00:00:00:00:00");
	gap_set_class_of_device(0x2c010c); // computer / laptop
	gap_ssp_set_io_capability(SSP_IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
	gap_ssp_set_authentication_requirement(SSP_IO_AUTHREQ_MITM_PROTECTION_NOT_REQUIRED_GENERAL_BONDING);
	gap_set_default_link_policy_settings(LM_LINK_POLICY_ENABLE_SNIFF_MODE | LM_LINK_POLICY_ENABLE_ROLE_SWITCH);
	hci_set_master_slave_policy(HCI_ROLE_MASTER);
	gap_connectable_control(1);   // bonded controllers page us when switched on
	gap_discoverable_control(0);

	hci_event_callback_registration.callback = &packet_handler;
	hci_add_event_handler(&hci_event_callback_registration);

	setvbuf(stdout, NULL, _IOLBF, 0);
	hci_power_control(HCI_POWER_ON);
	return 0;
}
