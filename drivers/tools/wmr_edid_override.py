#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
"""Generate a macOS display override that lets a Windows Mixed Reality
headset's panel appear as a normal desktop display.

WMR headsets carry Microsoft's vendor-specific EDID block (OUI CA-12-5C) that
marks the panel as a non-desktop head-mounted display. macOS reads the EDID,
builds a display pipe for the panel, and then keeps it off the desktop, so it
never shows up in CGGetOnlineDisplayList and nothing can be drawn on it.

Apple's display overrides support `edid-patches`: byte patches applied to the
EDID before it is interpreted. This tool finds every connected display whose
EDID has the Microsoft block, rewrites that block's tag to a reserved value so
parsers skip it, fixes the extension checksum, and writes the override plist
under /Library/Displays/Contents/Resources/Overrides/.

Usage:
  wmr_edid_override.py                 # show what would be written
  sudo wmr_edid_override.py --install  # write the override(s)
  wmr_edid_override.py --edid HEX      # work from an EDID hex dump instead of IOKit

After installing, unplug and replug the headset's HDMI/DisplayPort cable (or
sleep and wake the Mac) so macOS re-reads the EDID.
"""

import argparse
import os
import plistlib
import re
import subprocess
import sys

MICROSOFT_OUI = bytes([0x5C, 0x12, 0xCA])  # little-endian in the block
OVERRIDES_ROOT = "/Library/Displays/Contents/Resources/Overrides"


def edids_from_iokit():
    """Return the unique EDIDs IOKit currently holds for connected sinks."""
    out = subprocess.run(["ioreg", "-l", "-w0"], capture_output=True, text=True, errors="replace").stdout
    seen = []
    for m in re.finditer(r'"EDID" = <([0-9a-fA-F]+)>', out):
        edid = bytes.fromhex(m.group(1))
        if len(edid) >= 128 and edid not in seen:
            seen.append(edid)
    return seen


def vendor_product(edid):
    vendor = edid[8] << 8 | edid[9]
    product = edid[10] | edid[11] << 8
    return vendor, product


def product_name(edid):
    for off in (54, 72, 90, 108):
        if edid[off:off + 3] == b"\x00\x00\x00" and edid[off + 3] == 0xFC:
            return edid[off + 5:off + 18].decode("ascii", "replace").strip()
    return "unknown"


def find_microsoft_block(edid):
    """Return (block_offset, extension_offset) of the Microsoft VSDB, or None."""
    ext_count = edid[126]
    for n in range(ext_count):
        ext = 128 * (n + 1)
        if ext + 128 > len(edid) or edid[ext] != 0x02:  # CTA-861 only
            continue
        dtd_start = edid[ext + 2]
        i = ext + 4
        while i < ext + dtd_start and i < ext + 127:
            tag = edid[i] >> 5
            length = edid[i] & 0x1F
            if tag == 3 and edid[i + 1:i + 4] == MICROSOFT_OUI:
                return i, ext
            i += 1 + length
    return None


def describe_microsoft_block(edid, off):
    usage = edid[off + 5]
    container = edid[off + 6:off + 22].hex()
    return (f"version {edid[off + 4]}, desktop-use={'yes' if usage & 0x80 else 'no'}, "
            f"primary-use={usage & 0x1F}, container {container}")


def build_patches(edid, block_off, ext_off):
    patched = bytearray(edid)
    patched[block_off] = edid[block_off] & 0x1F  # tag 0 (reserved), same length
    checksum_off = ext_off + 127
    patched[checksum_off] = (256 - sum(patched[ext_off:checksum_off]) % 256) % 256
    assert sum(patched[ext_off:ext_off + 128]) % 256 == 0
    return [
        {"offset": block_off, "data": bytes([patched[block_off]])},
        {"offset": checksum_off, "data": bytes([patched[checksum_off]])},
    ]


def override_path(vendor, product):
    return os.path.join(OVERRIDES_ROOT, f"DisplayVendorID-{vendor:x}", f"DisplayProductID-{product:x}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--install", action="store_true", help="write the override plist(s); needs root")
    ap.add_argument("--edid", help="EDID as a hex string instead of reading IOKit")
    args = ap.parse_args()

    edids = [bytes.fromhex(args.edid)] if args.edid else edids_from_iokit()
    if not edids:
        print("IOKit holds no EDID for any connected display.", file=sys.stderr)
        return 1

    found = 0
    for edid in edids:
        vendor, product = vendor_product(edid)
        hit = find_microsoft_block(edid)
        name = product_name(edid)
        if hit is None:
            print(f"{name} ({vendor:04x}:{product:04x}): no Microsoft HMD block, nothing to do.")
            continue
        found += 1
        block_off, ext_off = hit
        print(f"{name} ({vendor:04x}:{product:04x}): Microsoft HMD block at byte {block_off} "
              f"({describe_microsoft_block(edid, block_off)})")
        patches = build_patches(edid, block_off, ext_off)
        plist = {
            "DisplayProductName": f"{name} (WMR panel, HMD flag hidden)",
            "edid-patches": patches,
        }
        path = override_path(vendor, product)
        for p in patches:
            print(f"  patch byte {p['offset']}: 0x{edid[p['offset']]:02x} -> 0x{p['data'][0]:02x}")
        print(f"  override: {path}")
        if not args.install:
            print(plistlib.dumps(plist, fmt=plistlib.FMT_XML).decode())
            continue
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "wb") as f:
                plistlib.dump(plist, f, fmt=plistlib.FMT_XML)
        except PermissionError:
            print("  permission denied: re-run with sudo", file=sys.stderr)
            return 1
        print("  written. Unplug and replug the headset's video cable so macOS re-reads the EDID.")

    if found == 0:
        print("No connected display carries the Microsoft HMD block. Is the headset's video cable connected?",
              file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
