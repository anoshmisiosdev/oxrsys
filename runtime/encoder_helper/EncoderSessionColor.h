// SPDX-License-Identifier: MPL-2.0

#pragma once

// -----------------------------------------------------------------------------
// The one colour contract every OXRSys VideoToolbox compression session applies,
// shared by the in-process encoder (runtime/src/VideoEncoder.mm, x86_64 or
// arm64) and the out-of-process helper (main.mm, arm64). Both paths encode the
// same 8-bit BGRA compose surface, and the client decodes whatever arrives with
// the colour description written into the SPS VUI. If the two sessions were
// configured separately they could drift (they did: the helper once set none of
// these, so its stream decoded with different colours from the in-process one).
// Keep every colour-affecting session property here, and only here.
//
// SDR BT.709 for every codec and profile, HEVC Main10 included: Main10 is a
// 10-bit bitstream from the same 8-bit SDR source, not an HDR/BT.2020 stream.
// Range is not set here: VideoToolbox has no compression property for it. Every
// hardware encoder signals video (limited) range for a BGRA source; the
// software HEVC encoder (all a Rosetta process gets for HEVC) signals full
// range. The helper only ever runs a hardware session.
//
// Header-only, system frameworks only, so the helper stays a single translation
// unit with no runtime dependencies.
// -----------------------------------------------------------------------------

#include <CoreVideo/CoreVideo.h>
#include <VideoToolbox/VideoToolbox.h>

namespace oxrsys::encoder_color
{

inline const CFStringRef kPrimaries = kCVImageBufferColorPrimaries_ITU_R_709_2;
inline const CFStringRef kTransferFunction = kCVImageBufferTransferFunction_ITU_R_709_2;
inline const CFStringRef kYCbCrMatrix = kCVImageBufferYCbCrMatrix_ITU_R_709_2;

struct ApplyResult
{
    OSStatus primaries = noErr;
    OSStatus transferFunction = noErr;
    OSStatus yCbCrMatrix = noErr;

    bool ok() const
    {
        return primaries == noErr && transferFunction == noErr && yCbCrMatrix == noErr;
    }
};

// Applies the contract to a freshly created session, before
// VTCompressionSessionPrepareToEncodeFrames. VideoToolbox writes these values
// into the H.264/H.265 VUI and uses the matching matrix for its RGB-to-YCbCr
// conversion. The caller logs a partial failure in its own log.
inline ApplyResult ApplySessionColorProperties(VTCompressionSessionRef session)
{
    ApplyResult result;
    result.primaries =
        VTSessionSetProperty(session, kVTCompressionPropertyKey_ColorPrimaries, kPrimaries);
    result.transferFunction =
        VTSessionSetProperty(session, kVTCompressionPropertyKey_TransferFunction, kTransferFunction);
    result.yCbCrMatrix =
        VTSessionSetProperty(session, kVTCompressionPropertyKey_YCbCrMatrix, kYCbCrMatrix);
    return result;
}

} // namespace oxrsys::encoder_color
