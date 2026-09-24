// SPDX-License-Identifier: MPL-2.0
//
// The last per-eye FOV and IPD a streaming headset reported, persisted across
// runs so a session can report the real view from its first frame.
//
// Many applications (OpenVR games through OpenComposite in particular) read the
// projection once, before any headset client has connected. Reporting the
// placeholder symmetric FOV and 63mm IPD then leaves them rendering a view the
// headset never displays. With a saved view the first xrLocateViews already
// matches what the client will show.

#pragma once

#include <string>

namespace oxrsys::runtime
{

struct SavedHeadsetView
{
    bool valid = false;
    float ipd = 0.0f;       // metres
    float fov[4] = {};      // left eye: left, right, up, down (radians, XrFovf signs)
    std::string clientName; // informational only
};

// Sanity limits for a headset view: FOV signs as XrFovf (left/down negative),
// half-angles below 90 degrees, IPD between 40 and 90 mm.
bool IsPlausibleHeadsetView(float ipd, const float fov[4]);

// True when the two views differ by more than float noise.
bool HeadsetViewsDiffer(const SavedHeadsetView& a, float ipd, const float fov[4]);

// "key=value" lines; unknown keys are ignored so the format can grow.
std::string FormatHeadsetView(const SavedHeadsetView& view);
bool ParseHeadsetView(const std::string& text, SavedHeadsetView& view);

bool LoadHeadsetView(const std::string& path, SavedHeadsetView& view);
// Writes through a temporary file and a rename, so a reader never sees half a file.
bool SaveHeadsetView(const std::string& path, const SavedHeadsetView& view);

} // namespace oxrsys::runtime
