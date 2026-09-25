// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace oxrsys
{

// One IPv4 address of a local interface, as reported by getifaddrs(). All
// addresses are in network byte order, exactly as they appear in sockaddr_in.
struct Ipv4InterfaceAddress
{
    std::string name;
    uint32_t address = 0;
    uint32_t netmask = 0;
    uint32_t broadcast = 0; // 0 when the interface reported none
    unsigned int flags = 0; // IFF_* flags
};

// Where the Wi-Fi discovery beacon goes for one interface: the interface's
// directed broadcast address, sent from a socket bound to the interface's own
// address so the client (which connects to the announce's source address)
// sees an address on its own subnet.
struct DiscoveryBroadcastTarget
{
    std::string interfaceName;
    uint32_t localAddress = 0;     // network byte order
    uint32_t broadcastAddress = 0; // network byte order

    bool operator==(const DiscoveryBroadcastTarget& other) const
    {
        return interfaceName == other.interfaceName &&
               localAddress == other.localAddress &&
               broadcastAddress == other.broadcastAddress;
    }
    bool operator!=(const DiscoveryBroadcastTarget& other) const { return !(*this == other); }
};

// Pure: selects interfaces that are up, running, broadcast-capable, not
// loopback/point-to-point, with a usable address, and computes each one's
// directed broadcast address (the reported one, else address | ~netmask).
// Duplicates are dropped; order follows the input.
std::vector<DiscoveryBroadcastTarget> ComputeDiscoveryBroadcastTargets(
    const std::vector<Ipv4InterfaceAddress>& interfaces);

// Enumerates the host's IPv4 interface addresses via getifaddrs().
std::vector<Ipv4InterfaceAddress> EnumerateIpv4Interfaces();

// "en0 192.168.2.1 -> 192.168.2.255, en12 ..." for logging.
std::string DescribeDiscoveryTargets(const std::vector<DiscoveryBroadcastTarget>& targets);

} // namespace oxrsys
