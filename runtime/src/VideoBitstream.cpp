// SPDX-License-Identifier: MPL-2.0

#include "VideoBitstream.h"

#include <algorithm>
#include <cstring>

namespace oxrsys::video_bitstream
{
namespace
{

constexpr uint8_t kAnnexBStartCode[] = {0x00, 0x00, 0x00, 0x01};

size_t StartCodeSizeAt(const uint8_t* data, size_t size, size_t offset)
{
    if (data == nullptr || offset + 3 > size)
    {
        return 0;
    }
    if (data[offset] == 0x00 && data[offset + 1] == 0x00)
    {
        if (data[offset + 2] == 0x01)
        {
            return 3;
        }
        if (offset + 4 <= size && data[offset + 2] == 0x00 && data[offset + 3] == 0x01)
        {
            return 4;
        }
    }
    return 0;
}

size_t FindStartCode(const uint8_t* data, size_t size, size_t offset)
{
    if (data == nullptr || offset >= size)
    {
        return size;
    }
    for (size_t i = offset; i + 3 <= size; ++i)
    {
        if (StartCodeSizeAt(data, size, i) != 0)
        {
            return i;
        }
    }
    return size;
}

const uint8_t* PayloadAfterStartCode(const uint8_t* data, size_t size, size_t& payloadSize)
{
    const size_t startCodeSize = StartCodeSizeAt(data, size, 0);
    if (startCodeSize == 0 || startCodeSize >= size)
    {
        payloadSize = 0;
        return nullptr;
    }
    payloadSize = size - startCodeSize;
    return data + startCodeSize;
}

uint32_t ReadBigEndianLength(const uint8_t* data, size_t size, size_t offset, size_t prefixBytes)
{
    if (data == nullptr || prefixBytes == 0 || prefixBytes > 4 || offset + prefixBytes > size)
    {
        return 0;
    }
    uint32_t value = 0;
    for (size_t i = 0; i < prefixBytes; ++i)
    {
        value = (value << 8u) | data[offset + i];
    }
    return value;
}

bool ParseLengthPrefixed(oxr::protocol::VideoCodec codec,
                         const uint8_t* data,
                         size_t size,
                         size_t prefixBytes,
                         bool fallbackKeyframe,
                         std::vector<AnnexBNalUnit>& units)
{
    size_t offset = 0;
    std::vector<AnnexBNalUnit> parsed;
    while (offset + prefixBytes <= size)
    {
        const uint32_t nalSize = ReadBigEndianLength(data, size, offset, prefixBytes);
        offset += prefixBytes;
        if (nalSize == 0 || offset + nalSize > size)
        {
            return false;
        }

        AnnexBNalUnit unit;
        unit.bytes.resize(sizeof(kAnnexBStartCode) + nalSize);
        std::memcpy(unit.bytes.data(), kAnnexBStartCode, sizeof(kAnnexBStartCode));
        std::memcpy(unit.bytes.data() + sizeof(kAnnexBStartCode), data + offset, nalSize);
        unit.keyframe = IsKeyframeNal(codec, unit.bytes.data(), unit.bytes.size());
        unit.parameterSet = IsParameterSetNal(codec, unit.bytes.data(), unit.bytes.size());
        parsed.push_back(std::move(unit));
        offset += nalSize;
    }

    if (offset != size || parsed.empty())
    {
        return false;
    }

    if (fallbackKeyframe &&
        std::none_of(parsed.begin(), parsed.end(), [](const AnnexBNalUnit& unit) { return unit.keyframe; }))
    {
        parsed.front().keyframe = true;
    }
    units = std::move(parsed);
    return true;
}

} // namespace

bool IsAnnexB(const uint8_t* data, size_t size)
{
    return StartCodeSizeAt(data, size, 0) != 0 ||
           FindStartCode(data, size, 0) != size;
}

bool IsKeyframeNal(oxr::protocol::VideoCodec codec, const uint8_t* annexBData, size_t size)
{
    size_t payloadSize = 0;
    const uint8_t* payload = PayloadAfterStartCode(annexBData, size, payloadSize);
    if (payload == nullptr || payloadSize == 0)
    {
        return false;
    }

    if (codec == oxr::protocol::VideoCodec::H264)
    {
        const uint8_t nalType = payload[0] & 0x1fu;
        return nalType == 5;
    }
    if (codec == oxr::protocol::VideoCodec::H265)
    {
        const uint8_t nalType = (payload[0] >> 1u) & 0x3fu;
        return nalType >= 16 && nalType <= 23;
    }
    return false;
}

bool IsParameterSetNal(oxr::protocol::VideoCodec codec, const uint8_t* annexBData, size_t size)
{
    size_t payloadSize = 0;
    const uint8_t* payload = PayloadAfterStartCode(annexBData, size, payloadSize);
    if (payload == nullptr || payloadSize == 0)
    {
        return false;
    }

    if (codec == oxr::protocol::VideoCodec::H264)
    {
        const uint8_t nalType = payload[0] & 0x1fu;
        return nalType == 7 || nalType == 8;
    }
    if (codec == oxr::protocol::VideoCodec::H265)
    {
        const uint8_t nalType = (payload[0] >> 1u) & 0x3fu;
        return nalType == 32 || nalType == 33 || nalType == 34;
    }
    return false;
}

std::vector<AnnexBNalUnit> NormalizeToAnnexB(oxr::protocol::VideoCodec codec,
                                             const uint8_t* data,
                                             size_t size,
                                             bool fallbackKeyframe)
{
    std::vector<AnnexBNalUnit> units;
    if (data == nullptr || size == 0)
    {
        return units;
    }

    if (IsAnnexB(data, size))
    {
        size_t start = FindStartCode(data, size, 0);
        while (start < size)
        {
            const size_t startCodeSize = StartCodeSizeAt(data, size, start);
            const size_t payloadStart = start + startCodeSize;
            const size_t next = FindStartCode(data, size, payloadStart);
            if (payloadStart < next)
            {
                AnnexBNalUnit unit;
                unit.bytes.resize(sizeof(kAnnexBStartCode) + (next - payloadStart));
                std::memcpy(unit.bytes.data(), kAnnexBStartCode, sizeof(kAnnexBStartCode));
                std::memcpy(unit.bytes.data() + sizeof(kAnnexBStartCode),
                            data + payloadStart,
                            next - payloadStart);
                unit.keyframe = IsKeyframeNal(codec, unit.bytes.data(), unit.bytes.size());
                unit.parameterSet = IsParameterSetNal(codec, unit.bytes.data(), unit.bytes.size());
                units.push_back(std::move(unit));
            }
            start = next;
        }
    }
    else
    {
        for (size_t prefixBytes : {4u, 2u, 1u})
        {
            if (ParseLengthPrefixed(codec, data, size, prefixBytes, fallbackKeyframe, units))
            {
                break;
            }
        }
    }

    if (units.empty())
    {
        AnnexBNalUnit unit;
        unit.bytes.resize(sizeof(kAnnexBStartCode) + size);
        std::memcpy(unit.bytes.data(), kAnnexBStartCode, sizeof(kAnnexBStartCode));
        std::memcpy(unit.bytes.data() + sizeof(kAnnexBStartCode), data, size);
        unit.keyframe = fallbackKeyframe ||
            IsKeyframeNal(codec, unit.bytes.data(), unit.bytes.size());
        unit.parameterSet = IsParameterSetNal(codec, unit.bytes.data(), unit.bytes.size());
        units.push_back(std::move(unit));
    }
    else if (fallbackKeyframe &&
             std::none_of(units.begin(), units.end(), [](const AnnexBNalUnit& unit) { return unit.keyframe; }))
    {
        units.front().keyframe = true;
    }

    return units;
}

bool VisitAnnexBUnits(oxr::protocol::VideoCodec codec,
                      const uint8_t* data,
                      size_t size,
                      bool fallbackKeyframe,
                      const NalUnitVisitor& visitor)
{
    if (!visitor)
    {
        return false;
    }

    const std::vector<AnnexBNalUnit> units =
        NormalizeToAnnexB(codec, data, size, fallbackKeyframe);
    for (const AnnexBNalUnit& unit : units)
    {
        if (!unit.bytes.empty())
        {
            visitor(unit.bytes.data(), unit.bytes.size(), unit.keyframe);
        }
    }
    return !units.empty();
}

} // namespace oxrsys::video_bitstream
