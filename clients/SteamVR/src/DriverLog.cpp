// SPDX-License-Identifier: MPL-2.0

#include "DriverLog.h"

#include <openvr_driver.h>

#include <cstdarg>
#include <cstdio>
#include <mutex>

namespace oxrsys
{

namespace
{
std::mutex g_logMutex;
std::FILE* g_mirrorFile = nullptr;
} // namespace

void DriverLogOpen(const char* pchMirrorPath)
{
    std::lock_guard<std::mutex> lock(g_logMutex);
    if (g_mirrorFile != nullptr || pchMirrorPath == nullptr)
    {
        return;
    }

    g_mirrorFile = std::fopen(pchMirrorPath, "a");
}

void DriverLogClose()
{
    std::lock_guard<std::mutex> lock(g_logMutex);
    if (g_mirrorFile != nullptr)
    {
        std::fclose(g_mirrorFile);
        g_mirrorFile = nullptr;
    }
}

void DriverLogPrintf(const char* pchFormat, ...)
{
    char buffer[1024];

    va_list args;
    va_start(args, pchFormat);
    std::vsnprintf(buffer, sizeof(buffer), pchFormat, args);
    va_end(args);

    std::lock_guard<std::mutex> lock(g_logMutex);

    if (vr::VRDriverLog() != nullptr)
    {
        vr::VRDriverLog()->Log(buffer);
    }

    if (g_mirrorFile != nullptr)
    {
        std::fputs(buffer, g_mirrorFile);
        std::fputc('\n', g_mirrorFile);
        std::fflush(g_mirrorFile);
    }
}

} // namespace oxrsys
