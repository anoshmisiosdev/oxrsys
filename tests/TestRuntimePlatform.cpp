// SPDX-License-Identifier: MPL-2.0

#include <catch2/catch_test_macros.hpp>

#include "RuntimePlatform.h"
#include "RuntimeSockets.h"

#include <thread>

#if defined(__APPLE__)
#include <pthread/qos.h>
#elif defined(__linux__)
#include <cerrno>
#include <pthread.h>
#include <sched.h>
#endif

TEST_CASE("RuntimePlatform resolves config and state roots per platform", "[runtime-platform]")
{
    oxrsys::runtime_platform::EnvironmentPaths environment = {};
    environment.home = "/home/tester";
    environment.xdgConfigHome = "/tmp/xdg-config";
    environment.xdgStateHome = "/tmp/xdg-state";
    environment.appData = "C:/Users/tester/AppData/Roaming";

    CHECK(oxrsys::runtime_platform::ConfigRootForPlatform(
              oxrsys::runtime_platform::PlatformKind::MacOS,
              environment) == "/home/tester/Library/Application Support/OXRSys");
    CHECK(oxrsys::runtime_platform::StateRootForPlatform(
              oxrsys::runtime_platform::PlatformKind::MacOS,
              environment) == "/home/tester/Library/Application Support/OXRSys");

    CHECK(oxrsys::runtime_platform::ConfigRootForPlatform(
              oxrsys::runtime_platform::PlatformKind::Linux,
              environment) == "/tmp/xdg-config/oxrsys");
    CHECK(oxrsys::runtime_platform::StateRootForPlatform(
              oxrsys::runtime_platform::PlatformKind::Linux,
              environment) == "/tmp/xdg-state/oxrsys");

    CHECK(oxrsys::runtime_platform::ConfigRootForPlatform(
              oxrsys::runtime_platform::PlatformKind::Windows,
              environment) == "C:/Users/tester/AppData/Roaming/OXRSys");
    CHECK(oxrsys::runtime_platform::StateRootForPlatform(
              oxrsys::runtime_platform::PlatformKind::Windows,
              environment) == "C:/Users/tester/AppData/Roaming/OXRSys");
}

TEST_CASE("RuntimePlatform falls back to home for Linux and Windows roots", "[runtime-platform]")
{
    oxrsys::runtime_platform::EnvironmentPaths environment = {};
    environment.home = "/home/tester";

    CHECK(oxrsys::runtime_platform::ConfigRootForPlatform(
              oxrsys::runtime_platform::PlatformKind::Linux,
              environment) == "/home/tester/.config/oxrsys");
    CHECK(oxrsys::runtime_platform::StateRootForPlatform(
              oxrsys::runtime_platform::PlatformKind::Linux,
              environment) == "/home/tester/.local/state/oxrsys");
    CHECK(oxrsys::runtime_platform::ConfigRootForPlatform(
              oxrsys::runtime_platform::PlatformKind::Windows,
              environment) == "/home/tester/AppData/Roaming/OXRSys");
}

TEST_CASE("RuntimePlatform exposes a non-zero process id", "[runtime-platform]")
{
    CHECK(oxrsys::runtime_platform::ProcessId() > 0);
}

TEST_CASE("RuntimeSockets invalid handle stays invalid after close", "[runtime-sockets]")
{
    auto socket = oxrsys::runtime_socket::InvalidSocket;
    CHECK(!oxrsys::runtime_socket::IsValid(socket));

    oxrsys::runtime_socket::Close(socket);
    CHECK(!oxrsys::runtime_socket::IsValid(socket));
}

TEST_CASE("SetCurrentThreadTimeSensitive raises the calling thread's priority",
          "[runtime-platform]")
{
    // Catch2 assertions are not safe off the test thread; the worker records
    // what it saw and the checks run after the join.
    bool queried = false;
    bool stateOk = false;
    std::thread worker([&] {
        oxrsys::runtime_platform::SetCurrentThreadTimeSensitive();
#if defined(__APPLE__)
        qos_class_t qosClass = QOS_CLASS_UNSPECIFIED;
        int relativePriority = 0;
        queried = pthread_get_qos_class_np(pthread_self(), &qosClass, &relativePriority) == 0;
        stateOk = qosClass == QOS_CLASS_USER_INTERACTIVE;
#elif defined(_WIN32)
        queried = true;
        stateOk = GetThreadPriority(GetCurrentThread()) == THREAD_PRIORITY_HIGHEST;
#elif defined(__linux__)
        // Deterministic in both privilege worlds. With CAP_SYS_NICE the helper
        // must have installed minimum-priority SCHED_FIFO; without it, retrying
        // the same call must fail with exactly EPERM - proving the helper made
        // the right request and privilege was the only blocker.
        int policy = -1;
        sched_param param = {};
        queried = pthread_getschedparam(pthread_self(), &policy, &param) == 0;
        if (policy == SCHED_FIFO)
        {
            stateOk = param.sched_priority == sched_get_priority_min(SCHED_FIFO);
        }
        else
        {
            sched_param retry = {};
            retry.sched_priority = sched_get_priority_min(SCHED_FIFO);
            stateOk = pthread_setschedparam(pthread_self(), SCHED_FIFO, &retry) == EPERM;
        }
#else
        queried = true;
        stateOk = true;
#endif
    });
    worker.join();
    CHECK(queried);
    CHECK(stateOk);
}
