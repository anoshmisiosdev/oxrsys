// SPDX-License-Identifier: MPL-2.0

#include "HeadsetViewStore.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace oxrsys::runtime
{

bool IsPlausibleHeadsetView(float ipd, const float fov[4])
{
    constexpr float kMaxHalfAngle = 1.55f; // just under 90 degrees
    for (int i = 0; i < 4; ++i)
    {
        if (!std::isfinite(fov[i]) || std::fabs(fov[i]) > kMaxHalfAngle)
        {
            return false;
        }
    }
    return std::isfinite(ipd) && ipd >= 0.040f && ipd <= 0.090f && fov[0] < 0.0f && fov[1] > 0.0f &&
           fov[2] > 0.0f && fov[3] < 0.0f;
}

bool HeadsetViewsDiffer(const SavedHeadsetView& a, float ipd, const float fov[4])
{
    if (!a.valid)
    {
        return true;
    }
    constexpr float kEpsilon = 1e-4f;
    if (std::fabs(a.ipd - ipd) > kEpsilon)
    {
        return true;
    }
    for (int i = 0; i < 4; ++i)
    {
        if (std::fabs(a.fov[i] - fov[i]) > kEpsilon)
        {
            return true;
        }
    }
    return false;
}

std::string FormatHeadsetView(const SavedHeadsetView& view)
{
    char buffer[512];
    std::snprintf(buffer, sizeof(buffer),
                  "# Last view reported by a streaming headset (written by the OXRSys runtime)\n"
                  "ipd=%.6f\nfov_left=%.6f\nfov_right=%.6f\nfov_up=%.6f\nfov_down=%.6f\nclient=%s\n",
                  view.ipd, view.fov[0], view.fov[1], view.fov[2], view.fov[3],
                  view.clientName.c_str());
    return buffer;
}

bool ParseHeadsetView(const std::string& text, SavedHeadsetView& view)
{
    SavedHeadsetView parsed;
    bool have[5] = {};
    std::istringstream stream(text);
    std::string line;
    while (std::getline(stream, line))
    {
        if (line.empty() || line[0] == '#')
        {
            continue;
        }
        const size_t eq = line.find('=');
        if (eq == std::string::npos)
        {
            continue;
        }
        const std::string key = line.substr(0, eq);
        const std::string value = line.substr(eq + 1);
        if (key == "client")
        {
            parsed.clientName = value;
            continue;
        }
        char* end = nullptr;
        const float number = std::strtof(value.c_str(), &end);
        if (end == value.c_str())
        {
            continue;
        }
        if (key == "ipd") { parsed.ipd = number; have[0] = true; }
        else if (key == "fov_left") { parsed.fov[0] = number; have[1] = true; }
        else if (key == "fov_right") { parsed.fov[1] = number; have[2] = true; }
        else if (key == "fov_up") { parsed.fov[2] = number; have[3] = true; }
        else if (key == "fov_down") { parsed.fov[3] = number; have[4] = true; }
    }
    for (bool h : have)
    {
        if (!h)
        {
            return false;
        }
    }
    if (!IsPlausibleHeadsetView(parsed.ipd, parsed.fov))
    {
        return false;
    }
    parsed.valid = true;
    view = parsed;
    return true;
}

bool LoadHeadsetView(const std::string& path, SavedHeadsetView& view)
{
    if (path.empty())
    {
        return false;
    }
    std::ifstream file(path);
    if (!file)
    {
        return false;
    }
    std::stringstream contents;
    contents << file.rdbuf();
    return ParseHeadsetView(contents.str(), view);
}

bool SaveHeadsetView(const std::string& path, const SavedHeadsetView& view)
{
    if (path.empty() || !view.valid)
    {
        return false;
    }
    const std::string temporary = path + ".tmp";
    {
        std::ofstream file(temporary, std::ios::trunc);
        if (!file)
        {
            return false;
        }
        file << FormatHeadsetView(view);
        if (!file)
        {
            return false;
        }
    }
    return std::rename(temporary.c_str(), path.c_str()) == 0;
}

} // namespace oxrsys::runtime
