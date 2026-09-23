// SPDX-License-Identifier: MPL-2.0

#include <catch2/catch_test_macros.hpp>

#include "EncoderHelperIpc.h"

#include <cstring>
#include <vector>

using namespace oxrsys::enc_ipc;

namespace
{

// Mirrors HevcEncoderHelperClient::SubmitFrame / the helper's Encode handler.
std::vector<uint8_t> BuildEncodePayload(uint64_t cookie, uint32_t slot, int64_t ptsNs,
                                        bool forceKeyframe)
{
    std::vector<uint8_t> payload;
    PutU64(payload, cookie);
    PutU32(payload, slot);
    PutI64(payload, ptsNs);
    payload.push_back(forceKeyframe ? 1 : 0);
    return payload;
}

} // namespace

TEST_CASE("Encoder helper frames carry a little-endian header the peer can parse", "[encoder_helper]")
{
    const std::vector<uint8_t> payload = BuildEncodePayload(0x0123456789ABCDEFull, 2, -1234567890LL, true);
    const std::vector<uint8_t> framed = Frame(MsgType::Encode, payload);

    REQUIRE(framed.size() == kHeaderBytes + payload.size());

    // Header bytes are explicitly little-endian, never a memcpy'd struct: the
    // parent is x86_64 (Rosetta) and the child is native arm64.
    Reader header(framed.data(), framed.size());
    CHECK(header.U32() == kMagic);
    CHECK(header.U16() == static_cast<uint16_t>(MsgType::Encode));
    CHECK(header.U16() == kProtocolVersion);
    CHECK(header.U32() == static_cast<uint32_t>(payload.size()));
    CHECK(header.ok());

    Reader body(framed.data() + kHeaderBytes, framed.size() - kHeaderBytes);
    CHECK(body.U64() == 0x0123456789ABCDEFull);
    CHECK(body.U32() == 2u);
    CHECK(body.I64() == -1234567890LL);
    const uint8_t* forceKeyframe = body.Bytes(1);
    REQUIRE(forceKeyframe != nullptr);
    CHECK(*forceKeyframe == 1);
    CHECK(body.ok());
    CHECK(body.remaining() == 0);
}

TEST_CASE("Encoder helper FrameDone round-trips its metrics", "[encoder_helper]")
{
    std::vector<uint8_t> payload;
    PutU64(payload, 0xDEADBEEFCAFEF00Dull);
    payload.push_back(0); // not dropped
    PutF64(payload, 8.125);
    payload.push_back(1); // keyframe

    Reader reader(payload.data(), payload.size());
    CHECK(reader.U64() == 0xDEADBEEFCAFEF00Dull);
    const uint8_t* dropped = reader.Bytes(1);
    REQUIRE(dropped != nullptr);
    CHECK(*dropped == 0);
    CHECK(reader.F64() == 8.125);
    const uint8_t* keyframe = reader.Bytes(1);
    REQUIRE(keyframe != nullptr);
    CHECK(*keyframe == 1);
    CHECK(reader.ok());
}

TEST_CASE("Encoder helper reader reports underflow instead of reading past the payload",
          "[encoder_helper]")
{
    std::vector<uint8_t> payload;
    PutU32(payload, 7);

    Reader reader(payload.data(), payload.size());
    CHECK(reader.U32() == 7u);
    CHECK(reader.ok());

    // A truncated message must fail the whole parse, not return garbage.
    CHECK(reader.U64() == 0u);
    CHECK_FALSE(reader.ok());
    CHECK(reader.Bytes(1) == nullptr);
}

TEST_CASE("Encoder helper protocol bounds are sane for the runtime's slot count", "[encoder_helper]")
{
    // The parent sends the authoritative slot count in Init; kMaxSlots only
    // bounds validation on the child.
    CHECK(kMaxSlots >= 3u);
    CHECK(kMaxPayloadBytes >= 1u * 1024u * 1024u);
    // 'BGRA' is the compose format the runtime hands the helper.
    CHECK(kPixelFormatBGRA == 0x42475241u);
}
