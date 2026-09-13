#!/bin/sh
# Build wmr_btstack against a BTstack checkout (default ~/src/btstack) and Homebrew libusb.
#
# The BTstack libusb transport is compiled from a patched copy: on macOS it skips
# libusb_set_configuration / libusb_reset_device right after opening the adapter. A reset
# re-enumerates the device underneath the open handle, and doing that (plus SCO iso
# transfers, disabled in btstack_config.h) is the prime suspect for the IOUSBHostFamily
# kernel data abort seen on macOS 27 beta with a CSR8510 clone.
set -eu

HERE="$(cd "$(dirname "$0")" && pwd)"
BTSTACK="${BTSTACK:-$HOME/src/btstack}"
OUT="${OUT:-$HERE/build}"
mkdir -p "$OUT"

sed \
	-e 's/#if !defined(__FreeBSD__)$/#if !defined(__FreeBSD__) \&\& !defined(__APPLE__)/' \
	-e 's/^    r = libusb_set_configuration(aHandle, configuration);/    r = 0; \/\/ skipped: already configured by macOS/' \
	"$BTSTACK/platform/libusb/hci_transport_h2_libusb.c" > "$OUT/hci_transport_h2_libusb.c"

grep -q '!defined(__APPLE__)' "$OUT/hci_transport_h2_libusb.c" || { echo "reset patch did not apply" >&2; exit 1; }
grep -q 'skipped: already configured' "$OUT/hci_transport_h2_libusb.c" || { echo "config patch did not apply" >&2; exit 1; }

SRC="
 $BTSTACK/src/btstack_hid.c
 $BTSTACK/src/btstack_hid_parser.c
 $BTSTACK/src/btstack_linked_list.c
 $BTSTACK/src/btstack_memory.c
 $BTSTACK/src/btstack_memory_pool.c
 $BTSTACK/src/btstack_run_loop.c
 $BTSTACK/src/btstack_run_loop_base.c
 $BTSTACK/src/btstack_tlv.c
 $BTSTACK/src/btstack_util.c
 $BTSTACK/src/hci.c
 $BTSTACK/src/hci_cmd.c
 $BTSTACK/src/hci_dump.c
 $BTSTACK/src/l2cap.c
 $BTSTACK/src/l2cap_signaling.c
 $BTSTACK/src/ad_parser.c
 $BTSTACK/src/classic/btstack_link_key_db_tlv.c
 $BTSTACK/src/classic/hid_host.c
 $BTSTACK/src/classic/sdp_client.c
 $BTSTACK/src/classic/sdp_server.c
 $BTSTACK/src/classic/sdp_util.c
 $BTSTACK/platform/posix/btstack_run_loop_posix.c
 $BTSTACK/platform/posix/btstack_signal.c
 $BTSTACK/platform/posix/btstack_tlv_posix.c
 $BTSTACK/platform/posix/hci_dump_posix_fs.c
 $OUT/hci_transport_h2_libusb.c
 $HERE/main.c
 $HERE/wmr_btstack.c
"

# shellcheck disable=SC2086
cc -O2 -std=gnu11 -Wall -Wno-unused-parameter \
	-I"$HERE" -I"$HERE/../../monado" -I"$BTSTACK/src" -I"$BTSTACK/platform/posix" -I"$BTSTACK/platform/libusb" \
	-I"$BTSTACK/3rd-party/rijndael" -I"$BTSTACK/3rd-party/micro-ecc" \
	-I"$BTSTACK/3rd-party/bluedroid/encoder/include" -I"$BTSTACK/3rd-party/bluedroid/decoder/include" -I"$BTSTACK/3rd-party/yxml" -I"$BTSTACK/3rd-party/md5" -I"$BTSTACK/3rd-party/lc3-google/include" \
	$(pkg-config --cflags libusb-1.0) \
	$SRC \
	$(pkg-config --libs libusb-1.0) \
	-o "$OUT/wmr_btstack"

echo "built $OUT/wmr_btstack"
