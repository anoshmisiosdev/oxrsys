// SPDX-License-Identifier: MPL-2.0
//
// Minimal BTstack libusb bootstrap for wmr_btstack (replaces port/libusb/main.c, which pulls in
// LE, audio and chipset code we don't use).

#include <getopt.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include "btstack_config.h"

#include "btstack_debug.h"
#include "btstack_event.h"
#include "btstack_memory.h"
#include "btstack_run_loop.h"
#include "btstack_run_loop_posix.h"
#include "btstack_signal.h"
#include "btstack_tlv_posix.h"
#include "classic/btstack_link_key_db_tlv.h"
#include "hci.h"
#include "hci_dump.h"
#include "hci_dump_posix_fs.h"
#include "hci_transport.h"
#include "hci_transport_usb.h"

#include "wmr_bt_bridge_protocol.h"

int btstack_main(int argc, const char *argv[]);
extern int wmr_pairing_seconds_at_start;

static char tlv_db_path[512];
static const btstack_tlv_t *tlv_impl;
static btstack_tlv_posix_t tlv_context;
static bool tlv_reset;
static bool shutdown_triggered;
static btstack_packet_callback_registration_t hci_event_callback_registration;

static void
packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
	UNUSED(channel);
	UNUSED(size);
	if (packet_type != HCI_EVENT_PACKET || hci_event_packet_get_type(packet) != BTSTACK_EVENT_STATE) return;

	switch (btstack_event_state_get_state(packet)) {
	case HCI_STATE_WORKING: {
		bd_addr_t local_addr;
		gap_local_bd_addr(local_addr);
		const char *home = getenv("HOME");
		snprintf(tlv_db_path, sizeof(tlv_db_path), "%s/Library/Application Support/OXRSys/wmr_btstack_%s.tlv",
		         home ? home : "/tmp", bd_addr_to_str_with_delimiter(local_addr, '-'));
		if (tlv_reset) unlink(tlv_db_path);
		tlv_impl = btstack_tlv_posix_init_instance(&tlv_context, tlv_db_path);
		btstack_tlv_set_instance(tlv_impl, &tlv_context);
		hci_set_link_key_db(btstack_link_key_db_tlv_get_instance(tlv_impl, &tlv_context));
		printf("Bondings: %s\n", tlv_db_path);
		break;
	}
	case HCI_STATE_OFF:
		btstack_tlv_posix_deinit(&tlv_context);
		if (shutdown_triggered) exit(0);
		break;
	default: break;
	}
}

static void
force_exit(int sig)
{
	(void)sig;
	_exit(0); // atexit handlers (socket unlink) were already run by the first exit attempt or don't matter
}

static void
trigger_shutdown(void)
{
	printf("\nShutting down adapter...\n");
	shutdown_triggered = true;
	// Powering the adapter off can stall on some dongles; don't hang Ctrl-C on it.
	signal(SIGALRM, force_exit);
	alarm(2);
	hci_power_control(HCI_POWER_OFF);
}

int
main(int argc, const char *argv[])
{
	const char *log_path = NULL;
	int opt;
	while ((opt = getopt(argc, (char *const *)argv, "p:rl:h")) != -1) {
		switch (opt) {
		case 'p': wmr_pairing_seconds_at_start = atoi(optarg); break;
		case 'r': tlv_reset = true; break;
		case 'l': log_path = optarg; break;
		default:
			printf("usage: %s [-p SECONDS] [-r] [-l hci_log.pklg]\n"
			       "  -p  look for controllers in pairing mode for SECONDS at start (default 120, 0 = off)\n"
			       "  -r  forget paired controllers\n",
			       argv[0]);
			return opt == 'h' ? 0 : 2;
		}
	}

	// Home and the headset helper both start the tool on demand; a second copy would only fail to
	// open the adapter (and briefly report it missing), so leave the running one alone.
	{
		struct sockaddr_un sa = {.sun_family = AF_UNIX};
		wmr_bridge_socket_path(sa.sun_path, sizeof(sa.sun_path));
		int fd = socket(AF_UNIX, SOCK_STREAM, 0);
		if (fd >= 0 && connect(fd, (struct sockaddr *)&sa, sizeof(sa)) == 0) {
			printf("wmr_btstack is already running (%s)\n", sa.sun_path);
			close(fd);
			return 0;
		}
		if (fd >= 0) close(fd);
	}

	const char *home = getenv("HOME");
	char dir[512];
	snprintf(dir, sizeof(dir), "%s/Library/Application Support/OXRSys", home ? home : "/tmp");
	mkdir(dir, 0755);

	char default_log[600];
	if (!log_path) {
		snprintf(default_log, sizeof(default_log), "%s/wmr_btstack_hci.pklg", dir);
		log_path = default_log;
	}

	btstack_memory_init();
	btstack_run_loop_init(btstack_run_loop_posix_get_instance());

	// PacketLogger trace outside /tmp so it survives a reboot.
	hci_dump_posix_fs_open(log_path, HCI_DUMP_PACKETLOGGER);
	hci_dump_init(hci_dump_posix_fs_get_instance());
	printf("HCI trace: %s\n", log_path);

	hci_init(hci_transport_usb_instance(), NULL);

	hci_event_callback_registration.callback = &packet_handler;
	hci_add_event_handler(&hci_event_callback_registration);
	btstack_signal_register_callback(SIGINT, &trigger_shutdown);
	btstack_signal_register_callback(SIGTERM, &trigger_shutdown);
	signal(SIGPIPE, SIG_IGN);

	btstack_main(argc, argv);
	btstack_run_loop_execute();
	return 0;
}
