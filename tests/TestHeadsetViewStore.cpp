// SPDX-License-Identifier: MPL-2.0

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "HeadsetViewStore.h"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <unistd.h>

using Catch::Matchers::WithinAbs;
using namespace oxrsys::runtime;

namespace
{

SavedHeadsetView Quest2View()
{
    SavedHeadsetView view;
    view.valid = true;
    view.ipd = 0.0583f;
    view.fov[0] = -0.9076f;
    view.fov[1] = 0.7330f;
    view.fov[2] = 0.8378f;
    view.fov[3] = -0.8727f;
    view.clientName = "Oculus Quest2";
    return view;
}

} // namespace

TEST_CASE("Headset view round-trips through its text form", "[headset-view]")
{
    const SavedHeadsetView original = Quest2View();
    SavedHeadsetView parsed;
    REQUIRE(ParseHeadsetView(FormatHeadsetView(original), parsed));
    CHECK(parsed.valid);
    CHECK_THAT(parsed.ipd, WithinAbs(original.ipd, 1e-6));
    for (int i = 0; i < 4; ++i)
    {
        CHECK_THAT(parsed.fov[i], WithinAbs(original.fov[i], 1e-6));
    }
    CHECK(parsed.clientName == "Oculus Quest2");
    CHECK_FALSE(HeadsetViewsDiffer(parsed, original.ipd, original.fov));
}

TEST_CASE("Headset view parsing rejects incomplete or implausible data", "[headset-view]")
{
    SavedHeadsetView view;
    CHECK_FALSE(ParseHeadsetView("", view));
    CHECK_FALSE(ParseHeadsetView("ipd=0.06\nfov_left=-0.9\nfov_right=0.7\nfov_up=0.8\n", view));
    // Swapped signs.
    CHECK_FALSE(ParseHeadsetView("ipd=0.06\nfov_left=0.9\nfov_right=-0.7\nfov_up=0.8\nfov_down=-0.8\n", view));
    // IPD in millimetres, not metres.
    CHECK_FALSE(ParseHeadsetView("ipd=63\nfov_left=-0.9\nfov_right=0.7\nfov_up=0.8\nfov_down=-0.8\n", view));
    CHECK_FALSE(view.valid);
    CHECK(ParseHeadsetView("# comment\nextra=1\nipd=0.06\nfov_left=-0.9\nfov_right=0.7\nfov_up=0.8\nfov_down=-0.8\n", view));
}

TEST_CASE("Headset view saves atomically and loads back", "[headset-view]")
{
    const std::filesystem::path dir =
        std::filesystem::temp_directory_path() / ("oxrsys-headset-view-" + std::to_string(getpid()));
    std::filesystem::create_directories(dir);
    const std::string path = (dir / "headset_view.txt").string();

    SavedHeadsetView missing;
    CHECK_FALSE(LoadHeadsetView(path, missing));

    REQUIRE(SaveHeadsetView(path, Quest2View()));
    CHECK_FALSE(std::filesystem::exists(path + ".tmp"));
    SavedHeadsetView loaded;
    REQUIRE(LoadHeadsetView(path, loaded));
    CHECK_THAT(loaded.fov[0], WithinAbs(-0.9076, 1e-6));

    SavedHeadsetView invalid;
    CHECK_FALSE(SaveHeadsetView(path, invalid));
    std::filesystem::remove_all(dir);
}
