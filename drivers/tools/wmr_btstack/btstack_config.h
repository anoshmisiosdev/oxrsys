// btstack_config.h for wmr_btstack: Classic-only HID host over libusb.
//
// Deliberately smaller than BTstack's port/libusb config: no SCO (it claims USB interface 1
// and runs isochronous transfers, which we don't need and which exercise the macOS USB
// user client paths that panicked the kernel on macOS 27 beta), no LE, no mesh/audio.

#ifndef BTSTACK_CONFIG_H
#define BTSTACK_CONFIG_H

#define HAVE_ASSERT
#define HAVE_MALLOC
#define HAVE_POSIX_FILE_IO
#define HAVE_POSIX_TIME

#define ENABLE_CLASSIC
#define ENABLE_LOG_ERROR
#define ENABLE_LOG_INFO
#define ENABLE_PRINTF_HEXDUMP

#define HCI_ACL_PAYLOAD_SIZE (1691 + 4)
#define HCI_INCOMING_PRE_BUFFER_SIZE 14

#define NVM_NUM_DEVICE_DB_ENTRIES 16
#define NVM_NUM_LINK_KEYS 16

#endif
