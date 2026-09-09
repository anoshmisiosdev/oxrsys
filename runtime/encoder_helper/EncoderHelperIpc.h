// SPDX-License-Identifier: MPL-2.0

#pragma once

// -----------------------------------------------------------------------------
// Wire protocol between the OXRSys runtime (parent, x86_64 under Rosetta) and
// the native-arm64 HEVC encoder helper (child), spoken over an inherited Unix
// stream socket. This header is FRAMEWORK-FREE by construction so it compiles
// unchanged in the x86_64 runtime dylib and the arm64 helper executable.
//
// IOSurface transfer is NOT carried on this socket. Surfaces travel once, at
// session start, as Mach send rights (IOSurfaceCreateMachPort ->
// IOSurfaceLookupFromMachPort) over a bootstrap rendezvous. The mach message
// layout for that transfer lives at the bottom of this header (a kernel-copied
// same-host message, so a fixed struct is safe there). Everything else — frame
// submission and encoded results — is this socket protocol.
//
// Rules enforced here:
//  * No Foundation / Metal / VideoToolbox / IOSurface includes.
//  * Explicit little-endian field serialization (both macOS arches are LE, but
//    we never memcpy a struct across the boundary — padding is not contract).
//  * Presentation timestamps cross the boundary as int64 nanoseconds that the
//    parent supplies and the child echoes back verbatim. The child never
//    generates a timestamp the parent compares against: mach_absolute_time is
//    NOT comparable across the Rosetta boundary (x86_64 timebase 1/1, native
//    arm64 125/3), so the helper only ever measures deltas within its own
//    process and reports them as already-converted milliseconds.
// -----------------------------------------------------------------------------

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace oxrsys::enc_ipc
{

// Frame magic on the wire, ASCII "OXH1".
inline constexpr uint32_t kMagic = 0x4F584831u;
inline constexpr uint16_t kProtocolVersion = 1;

// Bound the read loop before any allocation. One HEVC access unit for a
// 2272x1264 stereo frame is far below this.
inline constexpr uint32_t kMaxPayloadBytes = 8u * 1024u * 1024u;

// Matches VideoEncoder::SlotCount. The parent sends the authoritative count in
// Init; this is only an upper bound for validation on the child.
inline constexpr uint32_t kMaxSlots = 16;

// fourcc of the compose target the runtime hands us, 'BGRA'.
inline constexpr uint32_t kPixelFormatBGRA = 0x42475241u;

enum class MsgType : uint16_t
{
    Init = 1,       // parent -> child: session config (surfaces arrive via mach next)
    InitAck = 2,    // child -> parent: session create outcome + hardware flag
    Encode = 3,     // parent -> child: encode slot N as opaque cookie F
    Nal = 4,        // child -> parent: one Annex-B NAL unit for a cookie
    FrameDone = 5,  // child -> parent: terminal result for a cookie (+ metrics)
    SetBitrate = 6, // parent -> child: live bitrate change
    Shutdown = 7,   // parent -> child: orderly exit
};

// Preset selector, mirrors ConfigValues::encoderPreset.
enum class PresetCode : uint32_t
{
    Balanced = 0,
    Speed = 1,
    Quality = 2,
};

// InitAck status codes — reported so the parent log can say exactly where the
// helper failed rather than a generic "unavailable".
enum class InitStatus : uint32_t
{
    Ok = 0,
    SurfaceTransferFailed = 1,
    SessionCreateFailed = 2,
    HardwareUnavailable = 3, // session created but RequireHardware not honored
};

// -----------------------------------------------------------------------------
// Fixed 12-byte frame header: [magic u32][type u16][version u16][payloadLen u32].
// -----------------------------------------------------------------------------
inline constexpr size_t kHeaderBytes = 12;

// --- little-endian scalar helpers ---
inline void PutU16(std::vector<uint8_t>& b, uint16_t v)
{
    b.push_back((uint8_t)(v & 0xFF));
    b.push_back((uint8_t)((v >> 8) & 0xFF));
}
inline void PutU32(std::vector<uint8_t>& b, uint32_t v)
{
    for (int i = 0; i < 4; ++i) b.push_back((uint8_t)((v >> (8 * i)) & 0xFF));
}
inline void PutU64(std::vector<uint8_t>& b, uint64_t v)
{
    for (int i = 0; i < 8; ++i) b.push_back((uint8_t)((v >> (8 * i)) & 0xFF));
}
inline void PutI64(std::vector<uint8_t>& b, int64_t v) { PutU64(b, (uint64_t)v); }
inline void PutF64(std::vector<uint8_t>& b, double v)
{
    uint64_t u;
    std::memcpy(&u, &v, sizeof(u));
    PutU64(b, u);
}

// Cursor-based reader over a payload buffer. All getters bounds-check and set
// ok=false on underflow; callers check ok() once at the end.
class Reader
{
public:
    Reader(const uint8_t* data, size_t size) : data_(data), size_(size) {}

    uint16_t U16()
    {
        if (pos_ + 2 > size_) { ok_ = false; return 0; }
        uint16_t v = (uint16_t)data_[pos_] | ((uint16_t)data_[pos_ + 1] << 8);
        pos_ += 2;
        return v;
    }
    uint32_t U32()
    {
        if (pos_ + 4 > size_) { ok_ = false; return 0; }
        uint32_t v = 0;
        for (int i = 0; i < 4; ++i) v |= (uint32_t)data_[pos_ + i] << (8 * i);
        pos_ += 4;
        return v;
    }
    uint64_t U64()
    {
        if (pos_ + 8 > size_) { ok_ = false; return 0; }
        uint64_t v = 0;
        for (int i = 0; i < 8; ++i) v |= (uint64_t)data_[pos_ + i] << (8 * i);
        pos_ += 8;
        return v;
    }
    int64_t I64() { return (int64_t)U64(); }
    double F64()
    {
        uint64_t u = U64();
        double v;
        std::memcpy(&v, &u, sizeof(v));
        return v;
    }
    // Returns a pointer into the buffer for `len` bytes (zero-copy) or nullptr.
    const uint8_t* Bytes(size_t len)
    {
        if (pos_ + len > size_) { ok_ = false; return nullptr; }
        const uint8_t* p = data_ + pos_;
        pos_ += len;
        return p;
    }

    bool ok() const { return ok_; }
    size_t remaining() const { return size_ - pos_; }

private:
    const uint8_t* data_;
    size_t size_;
    size_t pos_ = 0;
    bool ok_ = true;
};

// Build a complete framed message: header + payload.
inline std::vector<uint8_t> Frame(MsgType type, const std::vector<uint8_t>& payload)
{
    std::vector<uint8_t> out;
    out.reserve(kHeaderBytes + payload.size());
    PutU32(out, kMagic);
    PutU16(out, (uint16_t)type);
    PutU16(out, kProtocolVersion);
    PutU32(out, (uint32_t)payload.size());
    out.insert(out.end(), payload.begin(), payload.end());
    return out;
}

// -----------------------------------------------------------------------------
// Mach rendezvous message (parent <-> child), same-host kernel copy. Layout is
// identical for both arches; a fixed struct is intentional and safe here. This
// is transport for IOSurface send rights only; nothing perf-sensitive rides it.
// -----------------------------------------------------------------------------
//
// The mach.h types are pulled in by whichever .mm includes this in a mach
// context; kept out of the framework-free section by guarding on the include.
#if defined(__MACH__) && defined(OXRSYS_ENC_IPC_WANT_MACH)
} // namespace oxrsys::enc_ipc

#include <mach/mach.h>

namespace oxrsys::enc_ipc
{

// Child -> parent: hand the parent a send right to the child's receive port so
// the parent can push surface ports back. msgh_id = kMsgIdChildPort.
// Parent -> child: one IOSurface send right (MOVE_SEND) tagged with the slot it
// backs and its geometry. msgh_id = kMsgIdSurface.
inline constexpr mach_msg_id_t kMsgIdChildPort = 0x0A78'0001;
inline constexpr mach_msg_id_t kMsgIdSurface = 0x0A78'0002;

struct PortMsg
{
    mach_msg_header_t header;
    mach_msg_body_t body;
    mach_msg_port_descriptor_t port;
    uint32_t slot;
    uint32_t width;
    uint32_t height;
    uint32_t pixelFormat;
};

struct PortMsgRecv
{
    PortMsg msg;
    mach_msg_trailer_t trailer;
};

#endif // mach section

} // namespace oxrsys::enc_ipc
