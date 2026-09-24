// SPDX-License-Identifier: MPL-2.0
// Decodes an Annex-B HEVC dump with VideoToolbox and prints its VUI colour description and the raw
// Y/Cb/Cr samples at tools/color/color_ramp_xr.mm's patch centres (left eye of a side-by-side frame).
// Build: clang++ -std=c++17 -fobjc-arc -O1 hevc_color_probe.mm -o hevc_color_probe \
//   -framework Foundation -framework VideoToolbox -framework CoreMedia -framework CoreVideo
// Run: ./hevc_color_probe stream.h265 [frames-to-skip]
#import <Foundation/Foundation.h>
#import <VideoToolbox/VideoToolbox.h>
#include <vector>
#include <cstdio>

static std::vector<std::vector<uint8_t>> SplitNals(const std::vector<uint8_t>& d) {
    std::vector<std::vector<uint8_t>> out; size_t i = 0, start = std::string::npos;
    auto isStart = [&](size_t p) { return p + 4 <= d.size() && d[p]==0 && d[p+1]==0 && d[p+2]==0 && d[p+3]==1; };
    for (i = 0; i + 4 <= d.size(); ++i) if (isStart(i)) { if (start != std::string::npos) out.emplace_back(d.begin()+start, d.begin()+i); start = i + 4; i += 3; }
    if (start != std::string::npos) out.emplace_back(d.begin()+start, d.end());
    return out;
}
int main(int argc, char** argv) {
    FILE* f = fopen(argv[1], "rb"); std::vector<uint8_t> data; int c; while ((c = fgetc(f)) != EOF) data.push_back((uint8_t)c); fclose(f);
    int skipFrames = argc > 2 ? atoi(argv[2]) : 60;
    auto nals = SplitNals(data);
    std::vector<uint8_t> vps, sps, pps; CMVideoFormatDescriptionRef fmt = nullptr; VTDecompressionSessionRef session = nullptr;
    __block CVPixelBufferRef last = nullptr; __block int decoded = 0;
    for (auto& n : nals) {
        int type = (n[0] >> 1) & 0x3f;
        if (type == 32) vps = n; else if (type == 33) sps = n; else if (type == 34) pps = n;
        else if (type <= 21) {
            if (!fmt) {
                if (vps.empty() || sps.empty() || pps.empty()) continue;
                const uint8_t* ps[3] = {vps.data(), sps.data(), pps.data()}; size_t sz[3] = {vps.size(), sps.size(), pps.size()};
                if (CMVideoFormatDescriptionCreateFromHEVCParameterSets(nullptr, 3, ps, sz, 4, nullptr, &fmt) != noErr) { printf("fmt fail\n"); return 1; }
                CFDictionaryRef ext = CMFormatDescriptionGetExtensions(fmt);
                NSDictionary* e = (__bridge NSDictionary*)ext;
                printf("VUI: FullRangeVideo=%s Matrix=%s Transfer=%s Primaries=%s\n",
                       [[e[(id)kCMFormatDescriptionExtension_FullRangeVideo] description] UTF8String] ?: "(absent)",
                       [[e[(id)kCMFormatDescriptionExtension_YCbCrMatrix] description] UTF8String] ?: "(absent)",
                       [[e[(id)kCMFormatDescriptionExtension_TransferFunction] description] UTF8String] ?: "(absent)",
                       [[e[(id)kCMFormatDescriptionExtension_ColorPrimaries] description] UTF8String] ?: "(absent)");
                bool full = [e[(id)kCMFormatDescriptionExtension_FullRangeVideo] boolValue];
                NSDictionary* attrs = @{(id)kCVPixelBufferPixelFormatTypeKey : @(full ? kCVPixelFormatType_420YpCbCr8BiPlanarFullRange : kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange)};
                VTDecompressionOutputCallbackRecord cb = {nullptr, nullptr};
                if (VTDecompressionSessionCreate(nullptr, fmt, nullptr, (__bridge CFDictionaryRef)attrs, nullptr, &session) != noErr) { printf("session fail\n"); return 1; }
            }
            std::vector<uint8_t> avcc(4 + n.size()); uint32_t len = CFSwapInt32HostToBig((uint32_t)n.size()); memcpy(avcc.data(), &len, 4); memcpy(avcc.data()+4, n.data(), n.size());
            CMBlockBufferRef bb = nullptr; CMBlockBufferCreateWithMemoryBlock(nullptr, nullptr, avcc.size(), nullptr, nullptr, 0, avcc.size(), 0, &bb);
            CMBlockBufferReplaceDataBytes(avcc.data(), bb, 0, avcc.size());
            CMSampleBufferRef sb = nullptr; size_t ssz = avcc.size();
            CMSampleBufferCreateReady(nullptr, bb, fmt, 1, 0, nullptr, 1, &ssz, &sb);
            VTDecompressionSessionDecodeFrameWithOutputHandler(session, sb, 0, nullptr, ^(OSStatus st, VTDecodeInfoFlags, CVImageBufferRef img, CMTime, CMTime) {
                if (st == noErr && img) { if (last) CVPixelBufferRelease(last); last = CVPixelBufferRetain(img); decoded++; }
            });
            VTDecompressionSessionWaitForAsynchronousFrames(session);
            CFRelease(sb); CFRelease(bb);
            if (decoded >= skipFrames) break;
        }
    }
    if (!last) { printf("no frame\n"); return 1; }
    CVPixelBufferLockBaseAddress(last, kCVPixelBufferLock_ReadOnly);
    size_t w = CVPixelBufferGetWidthOfPlane(last, 0), h = CVPixelBufferGetHeightOfPlane(last, 0);
    uint8_t* Y = (uint8_t*)CVPixelBufferGetBaseAddressOfPlane(last, 0); size_t ys = CVPixelBufferGetBytesPerRowOfPlane(last, 0);
    uint8_t* UV = (uint8_t*)CVPixelBufferGetBaseAddressOfPlane(last, 1); size_t uvs = CVPixelBufferGetBytesPerRowOfPlane(last, 1);
    printf("decoded %d frames, %zux%zu, pixel format %s\n", decoded, w, h, CVPixelBufferGetPixelFormatType(last) == kCVPixelFormatType_420YpCbCr8BiPlanarFullRange ? "420f (full)" : "420v (video)");
    size_t eyeW = w / 2;
    printf("left-eye patches Y:"); for (int i = 0; i < 8; i++) { size_t x = (size_t)(eyeW * (i + 0.5) / 8), y = (size_t)(h * 0.75); printf(" %d", Y[y*ys+x]); } printf("\n");
    printf("left-eye patches CbCr:"); for (int i = 0; i < 8; i++) { size_t x = (size_t)(eyeW * (i + 0.5) / 8)/2*2, y = (size_t)(h * 0.75)/2; printf(" %d/%d", UV[y*uvs+x], UV[y*uvs+x+1]); } printf("\n");
    printf("left-eye ramp Y:"); for (int i = 0; i < 16; i++) { size_t x = (size_t)(eyeW * (i + 0.5) / 16), y = (size_t)(h * 0.25); printf(" %d", Y[y*ys+x]); } printf("\n");
    return 0;
}
