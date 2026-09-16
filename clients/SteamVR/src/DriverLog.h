// SPDX-License-Identifier: MPL-2.0

#pragma once

// Logging for the SteamVR driver.
//
// Everything goes to vrserver's own log through vr::IVRDriverLog, so it shows
// up in `Steam/logs/vrserver.txt` next to the rest of SteamVR's startup trace.
// The same lines are mirrored to a plain file so that failures that happen
// before (or instead of) driver-context initialisation are still visible.

namespace oxrsys
{

void DriverLogOpen(const char* pchMirrorPath);
void DriverLogClose();
void DriverLogPrintf(const char* pchFormat, ...) __attribute__((format(printf, 1, 2)));

} // namespace oxrsys

#define OXRSYS_LOG(...) ::oxrsys::DriverLogPrintf(__VA_ARGS__)
