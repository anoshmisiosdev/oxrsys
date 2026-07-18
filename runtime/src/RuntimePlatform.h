// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <cstdint>
#include <string>

namespace oxrsys::runtime_platform
{

enum class PlatformKind
{
    MacOS,
    Linux,
    Windows,
    Unknown,
};

struct EnvironmentPaths
{
    std::string home;
    std::string xdgConfigHome;
    std::string xdgStateHome;
    std::string appData;
};

PlatformKind CurrentPlatform();
EnvironmentPaths CurrentEnvironmentPaths();
std::string ConfigRootForPlatform(PlatformKind platform, const EnvironmentPaths& environment);
std::string StateRootForPlatform(PlatformKind platform, const EnvironmentPaths& environment);
std::string ConfigRoot();
std::string StateRoot();
std::string ModuleDirectory(const void* symbolAddress);
uint64_t ProcessId();

// Raise the calling thread's scheduling priority so time-sensitive per-frame
// work is not delayed behind default-priority threads. Best effort on every
// platform: failures just keep the default policy.
void SetCurrentThreadTimeSensitive();

} // namespace oxrsys::runtime_platform
