// SPDX-License-Identifier: MPL-2.0

#include <catch2/catch_test_macros.hpp>

#include "DiscoveryTargets.h"

#include <arpa/inet.h>
#include <net/if.h>

using oxrsys::ComputeDiscoveryBroadcastTargets;
using oxrsys::DescribeDiscoveryTargets;
using oxrsys::Ipv4InterfaceAddress;

namespace
{
uint32_t Ip(const char* text)
{
    in_addr addr = {};
    REQUIRE(inet_pton(AF_INET, text, &addr) == 1);
    return addr.s_addr;
}

constexpr unsigned int kUpBroadcast = IFF_UP | IFF_RUNNING | IFF_BROADCAST;

Ipv4InterfaceAddress Iface(const char* name, const char* address, const char* netmask,
                           const char* broadcast, unsigned int flags = kUpBroadcast)
{
    Ipv4InterfaceAddress iface;
    iface.name = name;
    iface.address = Ip(address);
    iface.netmask = Ip(netmask);
    iface.broadcast = broadcast != nullptr ? Ip(broadcast) : 0;
    iface.flags = flags;
    return iface;
}
} // namespace

TEST_CASE("Discovery targets: wired uplink plus Internet Sharing hotspot", "[discovery]")
{
    // The reported multi-homed Mac: en12 is the default route, bridge100 is
    // the Internet Sharing hotspot the Quest joins.
    const auto targets = ComputeDiscoveryBroadcastTargets({
        Iface("lo0", "127.0.0.1", "255.0.0.0", nullptr, IFF_UP | IFF_RUNNING | IFF_LOOPBACK),
        Iface("en12", "10.56.18.0", "255.255.252.0", "10.56.19.255"),
        Iface("bridge100", "192.168.2.1", "255.255.255.0", "192.168.2.255"),
    });
    REQUIRE(targets.size() == 2);
    CHECK(targets[0].interfaceName == "en12");
    CHECK(targets[0].localAddress == Ip("10.56.18.0"));
    CHECK(targets[0].broadcastAddress == Ip("10.56.19.255"));
    CHECK(targets[1].interfaceName == "bridge100");
    CHECK(targets[1].localAddress == Ip("192.168.2.1"));
    CHECK(targets[1].broadcastAddress == Ip("192.168.2.255"));
    CHECK(DescribeDiscoveryTargets(targets) ==
          "en12 10.56.18.0 -> 10.56.19.255, bridge100 192.168.2.1 -> 192.168.2.255");
}

TEST_CASE("Discovery targets: broadcast derived from netmask when not reported", "[discovery]")
{
    const auto targets = ComputeDiscoveryBroadcastTargets({
        Iface("en0", "192.168.1.42", "255.255.255.0", nullptr),
    });
    REQUIRE(targets.size() == 1);
    CHECK(targets[0].broadcastAddress == Ip("192.168.1.255"));
}

TEST_CASE("Discovery targets: skips down, non-broadcast, tunnel and link-less entries", "[discovery]")
{
    const auto targets = ComputeDiscoveryBroadcastTargets({
        Iface("en1", "192.168.5.2", "255.255.255.0", "192.168.5.255", IFF_UP | IFF_BROADCAST),
        Iface("en2", "192.168.6.2", "255.255.255.0", "192.168.6.255", IFF_RUNNING | IFF_BROADCAST),
        Iface("utun3", "10.8.0.2", "255.255.255.0", nullptr,
              IFF_UP | IFF_RUNNING | IFF_POINTOPOINT),
        Iface("en3", "0.0.0.0", "255.255.255.0", "192.168.7.255"),
        Iface("en4", "192.168.8.2", "255.255.255.255", nullptr), // /32: no broadcast
        Iface("en5", "192.168.9.2", "0.0.0.0", nullptr),         // no mask, no broadcast
    });
    CHECK(targets.empty());
    CHECK(DescribeDiscoveryTargets(targets) == "(none)");
}

TEST_CASE("Discovery targets: duplicates collapse, distinct aliases stay", "[discovery]")
{
    const auto targets = ComputeDiscoveryBroadcastTargets({
        Iface("en0", "192.168.1.42", "255.255.255.0", "192.168.1.255"),
        Iface("en0", "192.168.1.42", "255.255.255.0", "192.168.1.255"),
        Iface("en0", "172.16.0.9", "255.255.0.0", "172.16.255.255"),
    });
    REQUIRE(targets.size() == 2);
    CHECK(targets[1].broadcastAddress == Ip("172.16.255.255"));
}

TEST_CASE("Discovery targets: live enumeration never yields loopback", "[discovery]")
{
    for (const auto& target : ComputeDiscoveryBroadcastTargets(oxrsys::EnumerateIpv4Interfaces()))
    {
        CHECK((ntohl(target.localAddress) >> 24) != 127);
        CHECK(target.broadcastAddress != target.localAddress);
    }
}
