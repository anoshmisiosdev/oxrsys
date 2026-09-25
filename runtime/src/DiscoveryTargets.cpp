// SPDX-License-Identifier: MPL-2.0

#include "DiscoveryTargets.h"

#include <algorithm>

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>

namespace oxrsys
{

namespace
{

bool IsUsableUnicast(uint32_t addressNetworkOrder)
{
    const uint32_t host = ntohl(addressNetworkOrder);
    if (host == 0 || host == INADDR_BROADCAST)
    {
        return false;
    }
    if ((host >> 24) == 127) // loopback net
    {
        return false;
    }
    if ((host >> 28) == 0xE) // multicast
    {
        return false;
    }
    return true;
}

std::string FormatIpv4(uint32_t addressNetworkOrder)
{
    in_addr addr = {};
    addr.s_addr = addressNetworkOrder;
    char text[INET_ADDRSTRLEN] = {};
    inet_ntop(AF_INET, &addr, text, sizeof(text));
    return text;
}

} // namespace

std::vector<DiscoveryBroadcastTarget> ComputeDiscoveryBroadcastTargets(
    const std::vector<Ipv4InterfaceAddress>& interfaces)
{
    std::vector<DiscoveryBroadcastTarget> targets;
    for (const Ipv4InterfaceAddress& iface : interfaces)
    {
        const unsigned int required = IFF_UP | IFF_RUNNING | IFF_BROADCAST;
        if ((iface.flags & required) != required)
        {
            continue;
        }
        if ((iface.flags & (IFF_LOOPBACK | IFF_POINTOPOINT)) != 0)
        {
            continue;
        }
        if (!IsUsableUnicast(iface.address))
        {
            continue;
        }

        uint32_t broadcast = iface.broadcast;
        if (broadcast == 0 || broadcast == iface.address)
        {
            if (iface.netmask == 0)
            {
                continue;
            }
            broadcast = iface.address | ~iface.netmask;
        }
        // A /31 or /32 has no directed broadcast worth sending to.
        if (broadcast == iface.address || ntohl(broadcast) == INADDR_BROADCAST)
        {
            continue;
        }

        DiscoveryBroadcastTarget target;
        target.interfaceName = iface.name;
        target.localAddress = iface.address;
        target.broadcastAddress = broadcast;
        const bool duplicate = std::any_of(targets.begin(), targets.end(),
            [&](const DiscoveryBroadcastTarget& existing) {
                return existing.localAddress == target.localAddress &&
                       existing.broadcastAddress == target.broadcastAddress;
            });
        if (!duplicate)
        {
            targets.push_back(std::move(target));
        }
    }
    return targets;
}

std::vector<Ipv4InterfaceAddress> EnumerateIpv4Interfaces()
{
    std::vector<Ipv4InterfaceAddress> result;
    ifaddrs* list = nullptr;
    if (getifaddrs(&list) != 0)
    {
        return result;
    }
    for (ifaddrs* ifa = list; ifa != nullptr; ifa = ifa->ifa_next)
    {
        if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_INET)
        {
            continue;
        }
        Ipv4InterfaceAddress entry;
        entry.name = ifa->ifa_name != nullptr ? ifa->ifa_name : "";
        entry.flags = ifa->ifa_flags;
        entry.address = reinterpret_cast<const sockaddr_in*>(ifa->ifa_addr)->sin_addr.s_addr;
        if (ifa->ifa_netmask != nullptr && ifa->ifa_netmask->sa_family == AF_INET)
        {
            entry.netmask =
                reinterpret_cast<const sockaddr_in*>(ifa->ifa_netmask)->sin_addr.s_addr;
        }
        if ((ifa->ifa_flags & IFF_BROADCAST) != 0 && ifa->ifa_broadaddr != nullptr &&
            ifa->ifa_broadaddr->sa_family == AF_INET)
        {
            entry.broadcast =
                reinterpret_cast<const sockaddr_in*>(ifa->ifa_broadaddr)->sin_addr.s_addr;
        }
        result.push_back(std::move(entry));
    }
    freeifaddrs(list);
    return result;
}

std::string DescribeDiscoveryTargets(const std::vector<DiscoveryBroadcastTarget>& targets)
{
    if (targets.empty())
    {
        return "(none)";
    }
    std::string text;
    for (const DiscoveryBroadcastTarget& target : targets)
    {
        if (!text.empty())
        {
            text += ", ";
        }
        text += target.interfaceName + " " + FormatIpv4(target.localAddress) + " -> " +
                FormatIpv4(target.broadcastAddress);
    }
    return text;
}

} // namespace oxrsys
