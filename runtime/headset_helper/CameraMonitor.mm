// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief A desktop window showing what the headset's tracking cameras see.
 */

#include "CameraMonitor.h"

#include "vit_monitor.h"
#include "wmr/wmr_hmd.h"
#include "xrt/xrt_device.h"

#import <AppKit/AppKit.h>

#include <dlfcn.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <unordered_set>
#include <utility>
#include <vector>

namespace oxrsys {

namespace {

constexpr uint32_t kCameras = OXRSYS_VIT_MONITOR_MAX_CAMERAS;
constexpr CGFloat kGap = 8.0;
constexpr CGFloat kStatusHeight = 66.0;
//! PS Move sphere radius, for the circle size.
constexpr float kSphereRadiusM = 0.0225f;

int64_t
NowNs()
{
	return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
	    .count();
}

struct CamBuffer
{
	uint32_t width = 0, height = 0;
	std::vector<uint8_t> pixels;
	int64_t timestampNs = 0;
	//! Value of the push counter when this buffer was written; 0 = never.
	uint64_t seq = 0;
};

struct Intrinsics
{
	bool valid = false;
	uint32_t width = 640, height = 480;
	float fx = 0.0f, fy = 0.0f, cx = 0.0f, cy = 0.0f;
};

} // namespace

} // namespace oxrsys

/*
 *
 * The view and window delegate.
 *
 */

@interface OXRSysCameraMonitorView : NSView
@property(nonatomic, assign) oxrsys::CameraMonitor::Impl *impl;
@end

@interface OXRSysCameraMonitorWindowDelegate : NSObject <NSWindowDelegate>
@end

struct oxrsys::CameraMonitor::Impl
{
	// Written by the camera thread, swapped in by the main thread.
	std::mutex mutex;
	CamBuffer back[kCameras];
	uint64_t pushed[kCameras] = {0, 0};

	// Main thread only from here on.
	CamBuffer front[kCameras];
	Intrinsics intrinsics[kCameras];

	NSWindow *window = nil;
	OXRSysCameraMonitorView *view = nil;
	OXRSysCameraMonitorWindowDelegate *delegate = nil;
	NSTimer *timer = nil;

	void *shimHandle = nullptr;
	oxrsys_vit_monitor_snapshot_fn snapshotFn = nullptr;
	std::unique_ptr<struct oxrsys_vit_monitor_snapshot> snapshot;
	bool haveSnapshot = false;
	uint64_t lastSequence = 0;
	//! Feature ids of the previous and current snapshot: a feature seen in
	//! both is "tracked", one only in the current is "new".
	std::unordered_set<int64_t> previousIds[kCameras];
	std::unordered_set<int64_t> currentIds[kCameras];

	std::function<void(CameraMonitorInfo &)> provider;
	CameraMonitorInfo info;

	// Rates, refreshed once a second.
	int64_t rateNs = 0;
	uint64_t rateFrames[kCameras] = {0, 0};
	uint64_t ratePoses = 0;
	double cameraFps[kCameras] = {0.0, 0.0};
	double posesPerSecond = 0.0;

	void UpdateFromSources();
	void Draw(CGContextRef ctx, NSRect bounds);
	void DrawCamera(CGContextRef ctx, uint32_t cam, NSRect rect);
	void DrawStatus(NSRect area);
};

@implementation OXRSysCameraMonitorView

- (BOOL)isOpaque
{
	return YES;
}

- (void)drawRect:(NSRect)dirty
{
	(void)dirty;
	CGContextRef ctx = NSGraphicsContext.currentContext.CGContext;
	if (self.impl != nullptr && ctx != nullptr) {
		self.impl->Draw(ctx, self.bounds);
	}
}

@end

@implementation OXRSysCameraMonitorWindowDelegate

// Closing only hides the window; the helper keeps running.
- (BOOL)windowShouldClose:(NSWindow *)sender
{
	[sender orderOut:nil];
	return NO;
}

@end

namespace oxrsys {

/*
 *
 * Data flow.
 *
 */

void
CameraMonitor::Impl::UpdateFromSources()
{
	{
		std::lock_guard<std::mutex> lock(mutex);
		for (uint32_t cam = 0; cam < kCameras; cam++) {
			if (back[cam].seq > front[cam].seq) {
				std::swap(back[cam], front[cam]);
			}
		}
	}

	if (snapshotFn != nullptr && snapshot && snapshotFn(snapshot.get())) {
		haveSnapshot = true;
		if (snapshot->sequence != lastSequence) {
			lastSequence = snapshot->sequence;
			for (uint32_t cam = 0; cam < kCameras && cam < snapshot->camera_count; cam++) {
				previousIds[cam].swap(currentIds[cam]);
				currentIds[cam].clear();
				const oxrsys_vit_monitor_camera &c = snapshot->cams[cam];
				for (uint32_t i = 0; i < c.feature_count; i++) {
					currentIds[cam].insert(c.features[i].id);
				}
			}
		}
	}

	const int64_t now = NowNs();
	if (rateNs == 0) {
		rateNs = now;
		std::lock_guard<std::mutex> lock(mutex);
		for (uint32_t cam = 0; cam < kCameras; cam++) rateFrames[cam] = pushed[cam];
		ratePoses = haveSnapshot ? snapshot->poses_popped : 0;
	} else if (now - rateNs >= 1000000000LL) {
		const double seconds = (double)(now - rateNs) / 1e9;
		for (uint32_t cam = 0; cam < kCameras; cam++) {
			uint64_t frames;
			{
				std::lock_guard<std::mutex> lock(mutex);
				frames = pushed[cam];
			}
			cameraFps[cam] = (double)(frames - std::min(frames, rateFrames[cam])) / seconds;
			rateFrames[cam] = frames;
		}
		if (haveSnapshot) {
			const uint64_t poses = snapshot->poses_popped;
			posesPerSecond = (double)(poses - std::min(poses, ratePoses)) / seconds;
			ratePoses = poses;
		}
		rateNs = now;
	}

	info = CameraMonitorInfo();
	if (provider) {
		provider(info);
	}
}

/*
 *
 * Drawing (main thread, inside drawRect:).
 *
 */

static void
SetColor(CGContextRef ctx, float r, float g, float b, float a)
{
	CGContextSetRGBFillColor(ctx, r, g, b, a);
	CGContextSetRGBStrokeColor(ctx, r, g, b, a);
}

//! Warm for near, cool for far; gray when the depth is unknown.
static void
DepthColor(float depth, float *r, float *g, float *b)
{
	if (!(depth > 0.0f) || !std::isfinite(depth)) {
		*r = *g = *b = 0.7f;
		return;
	}
	const float t = std::clamp((depth - 0.3f) / (4.0f - 0.3f), 0.0f, 1.0f);
	*r = 1.0f - 0.8f * t;
	*g = 0.55f + 0.05f * t;
	*b = 0.1f + 0.9f * t;
}

static void
DrawText(NSString *text, NSPoint at, NSColor *color, CGFloat size)
{
	NSDictionary *attrs = @{
		NSFontAttributeName : [NSFont monospacedSystemFontOfSize:size weight:NSFontWeightRegular],
		NSForegroundColorAttributeName : color,
	};
	[text drawAtPoint:at withAttributes:attrs];
}

void
CameraMonitor::Impl::DrawCamera(CGContextRef ctx, uint32_t cam, NSRect rect)
{
	const CamBuffer &buf = front[cam];
	SetColor(ctx, 0.08f, 0.08f, 0.09f, 1.0f);
	CGContextFillRect(ctx, rect);

	if (buf.width == 0 || buf.height == 0 || buf.pixels.empty()) {
		DrawText([NSString stringWithFormat:@"cam %u: no frames yet", cam],
		         NSMakePoint(rect.origin.x + 8, rect.origin.y + rect.size.height - 20), NSColor.lightGrayColor, 11);
		return;
	}

	// The image, scaled to the rect (the caller already kept the aspect).
	CFDataRef data = CFDataCreate(kCFAllocatorDefault, buf.pixels.data(), (CFIndex)buf.pixels.size());
	CGDataProviderRef provider = CGDataProviderCreateWithCFData(data);
	CGColorSpaceRef gray = CGColorSpaceCreateDeviceGray();
	CGImageRef image = CGImageCreate(buf.width, buf.height, 8, 8, buf.width, gray, kCGImageAlphaNone, provider, nullptr,
	                                 false, kCGRenderingIntentDefault);
	if (image != nullptr) {
		CGContextSaveGState(ctx);
		CGContextSetInterpolationQuality(ctx, kCGInterpolationLow);
		CGContextDrawImage(ctx, rect, image);
		CGContextRestoreGState(ctx);
		CGImageRelease(image);
	}
	CGColorSpaceRelease(gray);
	CGDataProviderRelease(provider);
	CFRelease(data);

	const CGFloat sx = rect.size.width / (CGFloat)buf.width;
	const CGFloat sy = rect.size.height / (CGFloat)buf.height;
	// Image row 0 is at the top; the view's y axis points up.
	auto toView = [&](float u, float v) {
		return NSMakePoint(rect.origin.x + (CGFloat)u * sx, rect.origin.y + rect.size.height - (CGFloat)v * sy);
	};

	// Basalt's features: a square per feature, coloured by depth, filled
	// when the feature was already in the previous pose, hollow when new.
	uint32_t featureCount = 0;
	if (haveSnapshot && cam < snapshot->camera_count && info.showFeatures) {
		const oxrsys_vit_monitor_camera &c = snapshot->cams[cam];
		featureCount = c.feature_count;
		const CGFloat half = 3.0;
		CGContextSetLineWidth(ctx, 1.0);
		for (uint32_t i = 0; i < c.feature_count; i++) {
			const oxrsys_vit_monitor_feature &f = c.features[i];
			float r, g, b;
			DepthColor(f.depth, &r, &g, &b);
			SetColor(ctx, r, g, b, 0.9f);
			const NSPoint p = toView(f.u, f.v);
			const CGRect box = CGRectMake(p.x - half, p.y - half, 2 * half, 2 * half);
			if (previousIds[cam].count(f.id) != 0) {
				CGContextFillRect(ctx, box);
			} else {
				CGContextStrokeRect(ctx, box);
			}
		}
	}

	// The sphere-tracked controller, projected into camera 0 as a pinhole
	// (the lens distortion is ignored; near the centre it is small).
	if (cam == 0 && info.controllerPositionValid && intrinsics[0].valid) {
		const Intrinsics &k = intrinsics[0];
		// Monado camera space (+Y up, -Z forward) to the image (+Y down, +Z forward).
		const float x = info.controllerPosition[0];
		const float y = -info.controllerPosition[1];
		const float z = -info.controllerPosition[2];
		if (z > 0.05f) {
			const float u = k.fx * x / z + k.cx;
			const float v = k.fy * y / z + k.cy;
			const CGFloat radius = std::max<CGFloat>(4.0, (CGFloat)(k.fx * kSphereRadiusM / z) * sx);
			// Intrinsics are for the calibration size; rescale if the frame differs.
			const CGFloat ku = (CGFloat)buf.width / (CGFloat)std::max<uint32_t>(k.width, 1);
			const CGFloat kv = (CGFloat)buf.height / (CGFloat)std::max<uint32_t>(k.height, 1);
			const NSPoint p = toView(u * (float)ku, v * (float)kv);
			SetColor(ctx, 1.0f, 0.2f, 0.9f, 1.0f);
			CGContextSetLineWidth(ctx, 2.0);
			CGContextStrokeEllipseInRect(ctx, CGRectMake(p.x - radius, p.y - radius, 2 * radius, 2 * radius));
			CGContextMoveToPoint(ctx, p.x - radius - 4, p.y);
			CGContextAddLineToPoint(ctx, p.x + radius + 4, p.y);
			CGContextMoveToPoint(ctx, p.x, p.y - radius - 4);
			CGContextAddLineToPoint(ctx, p.x, p.y + radius + 4);
			CGContextStrokePath(ctx);
		}
	}

	// WMR controllers: LED blobs from the controller frames, and a box per
	// controller (or per cluster of blobs not assigned to one yet).
	uint32_t ledBlobs = 0;
	if (cam < 2 && info.controllerTracking) {
		const ControllerCameraOverlay &ov = info.controllerCams[cam];
		ledBlobs = ov.blobCount;
		// Blob coordinates are in the controller frame, which has the SLAM frame's size.
		auto handColor = [&](int hand, float a) {
			if (hand == 0) {
				SetColor(ctx, 0.2f, 0.85f, 1.0f, a); // left: cyan
			} else if (hand == 1) {
				SetColor(ctx, 1.0f, 0.55f, 0.1f, a); // right: orange
			} else {
				SetColor(ctx, 1.0f, 1.0f, 0.3f, a); // unassigned: yellow
			}
		};
		for (const ControllerBlob &b : ov.blobs) {
			handColor(b.hand, 0.9f);
			const NSPoint p = toView(b.x, b.y);
			CGContextFillEllipseInRect(ctx, CGRectMake(p.x - 2, p.y - 2, 4, 4));
		}
		for (const ControllerBox &box : ov.boxes) {
			const NSPoint a = toView(box.x0, box.y0);
			const NSPoint b = toView(box.x1, box.y1);
			const CGFloat pad = 6.0;
			CGRect r = CGRectMake(std::min(a.x, b.x) - pad, std::min(a.y, b.y) - pad, std::fabs(b.x - a.x) + 2 * pad,
			                      std::fabs(b.y - a.y) + 2 * pad);
			handColor(box.hand, 1.0f);
			CGContextSetLineWidth(ctx, box.hand >= 0 ? 2.5 : 1.5);
			if (box.hand < 0) {
				const CGFloat dash[] = {4.0, 3.0};
				CGContextSetLineDash(ctx, 0, dash, 2);
			}
			CGContextStrokeRect(ctx, r);
			CGContextSetLineDash(ctx, 0, nullptr, 0);
			NSColor *color = box.hand == 0   ? [NSColor colorWithCalibratedRed:0.2 green:0.85 blue:1.0 alpha:1.0]
			                 : box.hand == 1 ? [NSColor colorWithCalibratedRed:1.0 green:0.55 blue:0.1 alpha:1.0]
			                                 : [NSColor colorWithCalibratedRed:1.0 green:1.0 blue:0.3 alpha:1.0];
			NSString *tag = box.hand == 0 ? @"L" : box.hand == 1 ? @"R" : [NSString stringWithFormat:@"? %u", box.blobs];
			DrawText(tag, NSMakePoint(r.origin.x, r.origin.y + r.size.height + 1), color, box.hand >= 0 ? 15 : 11);
		}
	}

	NSString *label = [NSString stringWithFormat:@"cam %u  %ux%u  %.1f fps  %u features", cam, buf.width, buf.height,
	                                             cameraFps[cam], featureCount];
	if (info.controllerTracking) {
		label = [label stringByAppendingFormat:@"  %u LED blobs", ledBlobs];
	}
	DrawText(label, NSMakePoint(rect.origin.x + 6, rect.origin.y + rect.size.height - 18), NSColor.whiteColor, 11);
}

void
CameraMonitor::Impl::DrawStatus(NSRect area)
{
	NSMutableString *line1 = [NSMutableString stringWithFormat:@"head: %s", info.trackingKind.c_str()];
	if (!info.slamNote.empty()) {
		[line1 appendFormat:@"  (%s)", info.slamNote.c_str()];
	}
	if (info.haveHeadPosition) {
		[line1 appendFormat:@"   position (%.2f, %.2f, %.2f) m", info.headPosition[0], info.headPosition[1],
		                    info.headPosition[2]];
	}
	if (haveSnapshot) {
		[line1 appendFormat:@"   poses %.1f/s", posesPerSecond];
	}

	NSMutableString *line2 = [NSMutableString string];
	if (haveSnapshot) {
		if (!snapshot->real_loaded) {
			[line2 appendFormat:@"VIT: %s", snapshot->real_library];
		} else if (!snapshot->tracker_created) {
			[line2 appendString:@"VIT: plugin loaded, no tracker"];
		} else {
			[line2 appendFormat:@"features L %u  R %u%s   IMU %llu samples   images %llu / %llu",
			                    snapshot->cams[0].feature_count, snapshot->cams[1].feature_count,
			                    snapshot->features_enabled ? "" : " (extension unavailable)",
			                    (unsigned long long)snapshot->imu_samples_pushed,
			                    (unsigned long long)snapshot->cams[0].images_pushed,
			                    (unsigned long long)snapshot->cams[1].images_pushed];
		}
	} else if (snapshotFn == nullptr) {
		[line2 appendString:@"features: no VIT monitor shim (liboxrsys-vit-monitor.dylib)"];
	}
	if (info.controllerPositionValid) {
		[line2 appendFormat:@"   controller (%.2f, %.2f, %.2f) m in cam 0", info.controllerPosition[0],
		                    info.controllerPosition[1], info.controllerPosition[2]];
	}

	NSMutableString *line3 = [NSMutableString string];
	if (info.controllerTracking) {
		[line3 appendFormat:@"controllers (%.0f LED frames/s): %s", info.controllerFps, info.controllerStatus.c_str()];
		if (info.opticalWeight[0] > 0.0f || info.opticalWeight[1] > 0.0f) {
			[line3 appendFormat:@"   optical position: L %.0f%% R %.0f%%", info.opticalWeight[0] * 100.0f,
			                    info.opticalWeight[1] * 100.0f];
		}
	}

	DrawText(line1, NSMakePoint(area.origin.x + 8, area.origin.y + 44), NSColor.whiteColor, 11);
	DrawText(line2, NSMakePoint(area.origin.x + 8, area.origin.y + 26), NSColor.lightGrayColor, 11);
	DrawText(line3, NSMakePoint(area.origin.x + 8, area.origin.y + 8), NSColor.whiteColor, 11);
}

void
CameraMonitor::Impl::Draw(CGContextRef ctx, NSRect bounds)
{
	UpdateFromSources();

	SetColor(ctx, 0.12f, 0.12f, 0.13f, 1.0f);
	CGContextFillRect(ctx, bounds);

	const NSRect status = NSMakeRect(bounds.origin.x, bounds.origin.y, bounds.size.width, kStatusHeight);
	const CGFloat availableW = std::max<CGFloat>(bounds.size.width - 3 * kGap, 2);
	const CGFloat availableH = std::max<CGFloat>(bounds.size.height - kStatusHeight - 2 * kGap, 1);
	const CGFloat columnW = availableW / 2;
	for (uint32_t cam = 0; cam < kCameras; cam++) {
		// Keep the frame's aspect (4:3 for WMR cameras) inside the column.
		const CamBuffer &buf = front[cam];
		const CGFloat aspect = (buf.width > 0 && buf.height > 0) ? (CGFloat)buf.width / (CGFloat)buf.height : 4.0 / 3.0;
		CGFloat w = columnW, h = w / aspect;
		if (h > availableH) {
			h = availableH;
			w = h * aspect;
		}
		const CGFloat x = kGap + cam * (columnW + kGap) + (columnW - w) / 2;
		const CGFloat y = kStatusHeight + kGap + (availableH - h) / 2;
		DrawCamera(ctx, cam, NSMakeRect(std::floor(x), std::floor(y), std::floor(w), std::floor(h)));
	}
	DrawStatus(status);
}

/*
 *
 * Public interface.
 *
 */

CameraMonitor::CameraMonitor() : impl_(new Impl) {}

CameraMonitor::~CameraMonitor()
{
	Close();
	delete impl_;
}

void
CameraMonitor::SetIntrinsics(uint32_t cam, uint32_t width, uint32_t height, float fx, float fy, float cx, float cy)
{
	if (cam >= kCameras) return;
	Intrinsics &k = impl_->intrinsics[cam];
	k.valid = width > 0 && height > 0 && fx > 0.0f && fy > 0.0f;
	k.width = width;
	k.height = height;
	k.fx = fx;
	k.fy = fy;
	k.cx = cx;
	k.cy = cy;
}

bool
CameraMonitor::SetIntrinsicsFromHeadset(struct xrt_device *hmd)
{
	if (hmd == nullptr || hmd->name != XRT_DEVICE_GENERIC_HMD) return false;
	struct wmr_hmd *wh = wmr_hmd(hmd);
	bool any = false;
	for (int i = 0; i < wh->config.tcam_count && i < (int)kCameras; i++) {
		const struct wmr_camera_config *cam = wh->config.tcams[i];
		if (cam == nullptr) continue;
		// Same numbers wmr_psmv_tracking.c derives: the calibration blob
		// normalises the intrinsics by the image size.
		const float w = (float)cam->roi.extent.w;
		const float h = (float)cam->roi.extent.h;
		const auto &p = cam->distortion6KT.params;
		SetIntrinsics((uint32_t)i, cam->roi.extent.w, cam->roi.extent.h, p.fx * w, p.fy * h, p.cx * w, p.cy * h);
		any = true;
	}
	return any;
}

void
CameraMonitor::PushFrame(uint32_t cam, const uint8_t *data, uint32_t width, uint32_t height, size_t stride,
                         int64_t timestampNs)
{
	if (cam >= kCameras || data == nullptr || width == 0 || height == 0) return;
	std::lock_guard<std::mutex> lock(impl_->mutex);
	CamBuffer &b = impl_->back[cam];
	b.width = width;
	b.height = height;
	b.pixels.resize((size_t)width * height);
	if (stride == width) {
		memcpy(b.pixels.data(), data, b.pixels.size());
	} else {
		for (uint32_t y = 0; y < height; y++) {
			memcpy(b.pixels.data() + (size_t)y * width, data + (size_t)y * stride, width);
		}
	}
	b.timestampNs = timestampNs;
	b.seq = ++impl_->pushed[cam];
}

bool
CameraMonitor::Open(CGDirectDisplayID avoidDisplay,
                    const std::string &shimPath,
                    std::function<void(CameraMonitorInfo &)> provider)
{
	Impl *impl = impl_;
	if (impl->window != nil) return true;
	impl->provider = std::move(provider);

	if (!shimPath.empty()) {
		// The same file Monado dlopened for the tracker: dlopen returns the
		// same image, so the snapshot comes from the live shim.
		impl->shimHandle = dlopen(shimPath.c_str(), RTLD_NOW | RTLD_LOCAL);
		if (impl->shimHandle != nullptr) {
			impl->snapshotFn = (oxrsys_vit_monitor_snapshot_fn)dlsym(impl->shimHandle, "oxrsys_vit_monitor_snapshot");
		}
		if (impl->snapshotFn == nullptr) {
			fprintf(stderr, "CameraMonitor: no feature overlay, %s: %s\n", shimPath.c_str(), dlerror());
		} else {
			impl->snapshot = std::make_unique<struct oxrsys_vit_monitor_snapshot>();
		}
	}

	// A screen that is not the headset panel; the main screen failing that.
	NSScreen *screen = nil;
	for (NSScreen *candidate in [NSScreen screens]) {
		NSNumber *number = candidate.deviceDescription[@"NSScreenNumber"];
		if (number != nil && (CGDirectDisplayID)number.unsignedIntValue == avoidDisplay) continue;
		screen = candidate;
		if (candidate == [NSScreen mainScreen]) break;
	}
	if (screen == nil) screen = [NSScreen mainScreen];

	const CGFloat camW = 480, camH = 360;
	const NSRect content = NSMakeRect(0, 0, 2 * camW + 3 * kGap, camH + kStatusHeight + 2 * kGap);
	NSRect frame = content;
	if (screen != nil) {
		const NSRect visible = screen.visibleFrame;
		frame.origin.x = visible.origin.x + (visible.size.width - content.size.width) / 2;
		frame.origin.y = visible.origin.y + (visible.size.height - content.size.height) / 2;
	}
	const NSWindowStyleMask style = NSWindowStyleMaskTitled | NSWindowStyleMaskClosable | NSWindowStyleMaskResizable |
	                                NSWindowStyleMaskMiniaturizable;
	NSWindow *window = [[NSWindow alloc] initWithContentRect:frame styleMask:style backing:NSBackingStoreBuffered
	                                                   defer:NO
	                                                  screen:screen];
	window.title = @"OXRSys headset cameras";
	window.releasedWhenClosed = NO;
	window.contentMinSize = NSMakeSize(400, 200);
	impl->delegate = [OXRSysCameraMonitorWindowDelegate new];
	window.delegate = impl->delegate;

	OXRSysCameraMonitorView *view = [[OXRSysCameraMonitorView alloc] initWithFrame:content];
	view.impl = impl;
	view.autoresizingMask = NSViewWidthSizable | NSViewHeightSizable;
	window.contentView = view;
	impl->window = window;
	impl->view = view;

	impl->timer = [NSTimer timerWithTimeInterval:1.0 / 30.0
	                                     repeats:YES
	                                       block:^(NSTimer *t) {
	                                         (void)t;
	                                         if (window.isVisible) {
		                                         view.needsDisplay = YES;
	                                         }
	                                       }];
	[[NSRunLoop mainRunLoop] addTimer:impl->timer forMode:NSRunLoopCommonModes];

	[window setFrameOrigin:frame.origin];
	[window orderFrontRegardless];
	return true;
}

void
CameraMonitor::Close()
{
	Impl *impl = impl_;
	if (impl->timer != nil) {
		[impl->timer invalidate];
		impl->timer = nil;
	}
	if (impl->view != nil) {
		impl->view.impl = nullptr;
		impl->view = nil;
	}
	if (impl->window != nil) {
		impl->window.delegate = nil;
		[impl->window orderOut:nil];
		impl->window = nil;
	}
	impl->delegate = nil;
	impl->provider = nullptr;
	impl->snapshotFn = nullptr;
	if (impl->shimHandle != nullptr) {
		dlclose(impl->shimHandle);
		impl->shimHandle = nullptr;
	}
}

} // namespace oxrsys
