// SPDX-License-Identifier: MPL-2.0
//
// Wire protocol between the runtime (any architecture, inside the game
// process) and oxrsys-headset-helper (native arm64, owns the wired headset).
//
// Transport: a Unix stream socket, framed exactly like the encoder helper
// (12-byte little-endian header, see EncoderHelperIpc.h), plus a one-off Mach
// rendezvous for IOSurface send rights, again the encoder helper's scheme in
// the same direction: the runtime checks in a per-connection bootstrap name,
// the helper looks it up and replies with a send right to its own port, and
// the runtime pushes one surface per slot to that port.
//
//   runtime                                   helper
//   Hello {version, pid, rendezvous}  ──────►
//                                     ◄──────  HeadsetInfo {panel, eyes, fov, refresh, controllers}
//                                     ◄──────  Tracking {TrackingPacket}   (continuous, ~250 Hz)
//   Surfaces {count}                  ──────►  (then: helper looks up rendezvous, mach transfer)
//                                     ◄──────  SurfacesAck {status}
//   SubmitFrame {slot, ts, pose}      ──────►  (latest frame shown at the next refresh)
//                                     ◄──────  FrameReleased {slot}        (GPU finished reading it)
//   Bye                               ──────►  (helper drops the surfaces and shows the lobby)
//
// The helper runs the lobby (a head-tracked room) whenever no client is
// connected or the connected client has not submitted a frame recently, so
// the panel is never blank and a game's own window is never needed on it.

#pragma once

#include "../encoder_helper/EncoderHelperIpc.h"

#include <string>
#include <unistd.h>

namespace oxrsys::headset_ipc
{

inline constexpr uint16_t kProtocolVersion = 1;
inline constexpr uint32_t kMaxSlots = 4;

// Default socket path: per-user, reachable from a Wine process's Unix side.
inline std::string DefaultSocketPath()
{
    return "/tmp/oxrsys-headset-" + std::to_string((unsigned)getuid()) + ".sock";
}

enum class MsgType : uint16_t
{
    Hello = 1,          // runtime -> helper
    HeadsetInfo = 2,    // helper -> runtime
    Tracking = 3,       // helper -> runtime: raw oxr::protocol::TrackingPacket
    Surfaces = 4,       // runtime -> helper: slot count (mach transfer follows)
    SurfacesAck = 5,    // helper -> runtime: status
    SubmitFrame = 6,    // runtime -> helper
    FrameReleased = 7,  // helper -> runtime
    Bye = 8,            // runtime -> helper
};

enum class SurfacesStatus : uint32_t
{
    Ok = 0,
    RendezvousFailed = 1,
    TransferFailed = 2,
};

// HeadsetInfo payload, all little-endian:
//   u32 panelW, u32 panelH, u32 eyeW, u32 eyeH,
//   f32 fovLeft, f32 fovRight, f32 fovUp, f32 fovDown   (left eye, radians)
//   u32 refreshHz, u32 flags (bit0: display ready),
//   u32 nameLen, bytes name, u32 controllersLen, bytes controllers
struct HeadsetInfo
{
    uint32_t panelW = 0, panelH = 0, eyeW = 0, eyeH = 0;
    float fov[4] = {};
    uint32_t refreshHz = 90;
    bool displayReady = false;
    std::string name;
    std::string controllers;
};

inline void PutF32(std::vector<uint8_t>& b, float v)
{
    uint32_t u;
    std::memcpy(&u, &v, sizeof(u));
    enc_ipc::PutU32(b, u);
}

inline float GetF32(enc_ipc::Reader& r)
{
    uint32_t u = r.U32();
    float v;
    std::memcpy(&v, &u, sizeof(v));
    return v;
}

inline void PutString(std::vector<uint8_t>& b, const std::string& s)
{
    enc_ipc::PutU32(b, (uint32_t)s.size());
    b.insert(b.end(), s.begin(), s.end());
}

inline std::string GetString(enc_ipc::Reader& r)
{
    uint32_t len = r.U32();
    if (len > 4096) { return {}; }
    const uint8_t* p = r.Bytes(len);
    return p ? std::string((const char*)p, len) : std::string();
}

inline std::vector<uint8_t> EncodeHeadsetInfo(const HeadsetInfo& info)
{
    std::vector<uint8_t> p;
    enc_ipc::PutU32(p, info.panelW);
    enc_ipc::PutU32(p, info.panelH);
    enc_ipc::PutU32(p, info.eyeW);
    enc_ipc::PutU32(p, info.eyeH);
    for (float f : info.fov) PutF32(p, f);
    enc_ipc::PutU32(p, info.refreshHz);
    enc_ipc::PutU32(p, info.displayReady ? 1u : 0u);
    PutString(p, info.name);
    PutString(p, info.controllers);
    return p;
}

inline bool DecodeHeadsetInfo(const std::vector<uint8_t>& payload, HeadsetInfo& info)
{
    enc_ipc::Reader r(payload.data(), payload.size());
    info.panelW = r.U32();
    info.panelH = r.U32();
    info.eyeW = r.U32();
    info.eyeH = r.U32();
    for (float& f : info.fov) f = GetF32(r);
    info.refreshHz = r.U32();
    info.displayReady = (r.U32() & 1u) != 0;
    info.name = GetString(r);
    info.controllers = GetString(r);
    return r.ok();
}

// SubmitFrame payload: u32 slot, i64 timestampNs, u32 hasPose,
//   f32 qx,qy,qz,qw, f32 px,py,pz  (the head pose the frame was rendered for)
struct SubmitFrame
{
    uint32_t slot = 0;
    int64_t timestampNs = 0;
    bool hasPose = false;
    float orientation[4] = {0, 0, 0, 1};
    float position[3] = {0, 0, 0};
};

inline std::vector<uint8_t> EncodeSubmitFrame(const SubmitFrame& f)
{
    std::vector<uint8_t> p;
    enc_ipc::PutU32(p, f.slot);
    enc_ipc::PutI64(p, f.timestampNs);
    enc_ipc::PutU32(p, f.hasPose ? 1u : 0u);
    for (float v : f.orientation) PutF32(p, v);
    for (float v : f.position) PutF32(p, v);
    return p;
}

inline bool DecodeSubmitFrame(const std::vector<uint8_t>& payload, SubmitFrame& f)
{
    enc_ipc::Reader r(payload.data(), payload.size());
    f.slot = r.U32();
    f.timestampNs = r.I64();
    f.hasPose = (r.U32() & 1u) != 0;
    for (float& v : f.orientation) v = GetF32(r);
    for (float& v : f.position) v = GetF32(r);
    return r.ok();
}

inline std::vector<uint8_t> Frame(MsgType type, const std::vector<uint8_t>& payload)
{
    std::vector<uint8_t> out;
    out.reserve(enc_ipc::kHeaderBytes + payload.size());
    enc_ipc::PutU32(out, enc_ipc::kMagic);
    enc_ipc::PutU16(out, (uint16_t)type);
    enc_ipc::PutU16(out, kProtocolVersion);
    enc_ipc::PutU32(out, (uint32_t)payload.size());
    out.insert(out.end(), payload.begin(), payload.end());
    return out;
}

} // namespace oxrsys::headset_ipc
