// SPDX-License-Identifier: MPL-2.0

#pragma once

// -----------------------------------------------------------------------------
// Framework-free wire primitives shared by the OXRSys runtime (which may be
// x86_64 under Rosetta inside a Wine process) and a native-arm64 helper process,
// spoken over a Unix stream socket.
//
// This header is FRAMEWORK-FREE by construction so it compiles unchanged in the
// runtime dylib and in the helper executable, for either architecture.
//
// Rules enforced here:
//  * No Foundation / Metal / IOSurface includes outside the guarded Mach section.
//  * Explicit little-endian field serialization. Both macOS architectures are
//    little-endian, but a struct is never memcpy'd across the boundary: padding
//    is not contract.
//  * Timestamps cross as int64 nanoseconds supplied by one side and echoed back
//    verbatim. mach_absolute_time is NOT comparable across the Rosetta boundary
//    (x86_64 timebase 1/1, native arm64 125/3), so each process only ever
//    measures deltas within itself.
//
// IOSurface transfer does not ride this socket. Surfaces travel once, as Mach
// send rights (IOSurfaceCreateMachPort -> IOSurfaceLookupFromMachPort) over a
// bootstrap rendezvous; that message layout is at the bottom of this header,
// behind OXRSYS_HELPER_IPC_WANT_MACH. It is a same-host kernel copy, so a fixed
// struct is safe there.
// -----------------------------------------------------------------------------

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace oxrsys::helper_ipc
{

// Frame magic on the wire, ASCII "OXH1".
inline constexpr uint32_t kMagic = 0x4F584831u;

// magic(4) + type(2) + version(2) + payloadLength(4)
inline constexpr size_t kHeaderBytes = 12;

// Bound the read loop before any allocation.
inline constexpr uint32_t kMaxPayloadBytes = 8u * 1024u * 1024u;

// fourcc of the shared compose target, 'BGRA'.
inline constexpr uint32_t kPixelFormatBGRA = 0x42475241u;

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

// -----------------------------------------------------------------------------
// Mach rendezvous message (parent <-> child), same-host kernel copy. The layout
// is identical for both architectures; a fixed struct is intentional and safe
// here. This carries IOSurface send rights only; nothing perf-sensitive.
//
// The mach.h types are pulled in by whichever .mm includes this in a mach
// context, so they stay out of the framework-free section.
// -----------------------------------------------------------------------------
#if defined(__MACH__) && defined(OXRSYS_HELPER_IPC_WANT_MACH)
} // namespace oxrsys::helper_ipc

#include <mach/mach.h>

namespace oxrsys::helper_ipc
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

} // namespace oxrsys::helper_ipc
