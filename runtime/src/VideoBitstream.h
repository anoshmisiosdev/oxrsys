// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include <oxrsys/protocol/Protocol.h>

namespace oxrsys::video_bitstream
{

struct AnnexBNalUnit
{
    std::vector<uint8_t> bytes;
    bool keyframe = false;
    bool parameterSet = false;
};

bool IsAnnexB(const uint8_t* data, size_t size);
bool IsKeyframeNal(oxr::protocol::VideoCodec codec, const uint8_t* annexBData, size_t size);
bool IsParameterSetNal(oxr::protocol::VideoCodec codec, const uint8_t* annexBData, size_t size);

std::vector<AnnexBNalUnit> NormalizeToAnnexB(oxr::protocol::VideoCodec codec,
                                             const uint8_t* data,
                                             size_t size,
                                             bool fallbackKeyframe);

using NalUnitVisitor = std::function<void(const uint8_t* data,
                                          size_t size,
                                          bool keyframe)>;

bool VisitAnnexBUnits(oxr::protocol::VideoCodec codec,
                      const uint8_t* data,
                      size_t size,
                      bool fallbackKeyframe,
                      const NalUnitVisitor& visitor);

} // namespace oxrsys::video_bitstream
