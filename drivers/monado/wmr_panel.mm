// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief Present distortion-corrected stereo images on a WMR headset's panel.
 */

#include "wmr_panel.h"

#import <AppKit/AppKit.h>
#import <QuartzCore/QuartzCore.h>

#include "xrt/xrt_device.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <mutex>
#include <thread>

namespace oxrsys {

/*
 *
 * Shaders.
 *
 */

static const char *kPanelShaderSource = R"MSL(
#include <metal_stdlib>
using namespace metal;

struct MeshIn {
    float4 pos_ruv  [[attribute(0)]];
    float4 guv_buv  [[attribute(1)]];
};

struct MeshOut {
    float4 position [[position]];
    float2 r_uv;
    float2 g_uv;
    float2 b_uv;
};

vertex MeshOut mesh_vertex(MeshIn in [[stage_in]]) {
    MeshOut out;
    // Monado meshes are Vulkan-oriented (y = -1 is the top); Metal has +y up.
    out.position = float4(in.pos_ruv.x, -in.pos_ruv.y, 0.0, 1.0);
    out.r_uv = in.pos_ruv.zw;
    out.g_uv = in.guv_buv.xy;
    out.b_uv = in.guv_buv.zw;
    return out;
}

fragment float4 mesh_fragment(MeshOut in [[stage_in]], texture2d<float> eye [[texture(0)]]) {
    constexpr sampler s(address::clamp_to_border, border_color::opaque_black, filter::linear);
    float r = eye.sample(s, in.r_uv).r;
    float g = eye.sample(s, in.g_uv).g;
    float b = eye.sample(s, in.b_uv).b;
    return float4(r, g, b, 1.0);
}
)MSL";

/*
 *
 * Display helpers.
 *
 */

static std::vector<CGDirectDisplayID>
online_displays()
{
	CGDirectDisplayID ids[16];
	uint32_t count = 0;
	CGGetOnlineDisplayList(16, ids, &count);
	return std::vector<CGDirectDisplayID>(ids, ids + count);
}

static CFArrayRef
copy_all_modes(CGDirectDisplayID display)
{
	const void *keys[] = {kCGDisplayShowDuplicateLowResolutionModes};
	const void *values[] = {kCFBooleanTrue};
	CFDictionaryRef opts = CFDictionaryCreate(NULL, keys, values, 1, &kCFTypeDictionaryKeyCallBacks,
	                                          &kCFTypeDictionaryValueCallBacks);
	CFArrayRef modes = CGDisplayCopyAllDisplayModes(display, opts);
	CFRelease(opts);
	return modes;
}

//! Exact pixel size if available (else largest), highest refresh among those.
static CGDisplayModeRef
find_native_mode(CGDirectDisplayID display, uint32_t w, uint32_t h, bool *out_exact)
{
	CFArrayRef modes = copy_all_modes(display);
	if (modes == NULL) {
		return NULL;
	}
	CGDisplayModeRef best = NULL;
	bool best_exact = false;
	size_t best_area = 0;
	double best_rate = -1.0;
	for (CFIndex i = 0; i < CFArrayGetCount(modes); i++) {
		CGDisplayModeRef mode = (CGDisplayModeRef)CFArrayGetValueAtIndex(modes, i);
		const size_t mw = CGDisplayModeGetPixelWidth(mode), mh = CGDisplayModeGetPixelHeight(mode);
		const bool exact = mw == w && mh == h;
		const size_t area = mw * mh;
		const double rate = CGDisplayModeGetRefreshRate(mode);
		bool better;
		if (exact != best_exact) {
			better = exact;
		} else if (!exact && area != best_area) {
			better = area > best_area;
		} else {
			better = rate > best_rate;
		}
		if (better) {
			best = mode;
			best_exact = exact;
			best_area = area;
			best_rate = rate;
		}
	}
	if (best != NULL) {
		CGDisplayModeRetain(best);
	}
	CFRelease(modes);
	if (out_exact != NULL) {
		*out_exact = best_exact;
	}
	return best;
}

static CGDirectDisplayID
find_panel_display(uint32_t w, uint32_t h, const std::vector<CGDirectDisplayID> &before)
{
	const std::vector<CGDirectDisplayID> now = online_displays();
	for (CGDirectDisplayID id : now) {
		if (CGDisplayIsMain(id) || CGDisplayIsBuiltin(id)) {
			continue;
		}
		bool exact = false;
		CGDisplayModeRef mode = find_native_mode(id, w, h, &exact);
		if (mode != NULL) {
			CGDisplayModeRelease(mode);
		}
		if (exact) {
			return id;
		}
	}
	for (CGDirectDisplayID id : now) {
		bool was_online = false;
		for (CGDirectDisplayID old : before) {
			was_online |= old == id;
		}
		if (!was_online && !CGDisplayIsMain(id) && !CGDisplayIsBuiltin(id)) {
			return id;
		}
	}
	return 0;
}

static NSScreen *
screen_for_display(CGDirectDisplayID display)
{
	for (NSScreen *screen in [NSScreen screens]) {
		NSNumber *number = screen.deviceDescription[@"NSScreenNumber"];
		if (number != nil && (CGDirectDisplayID)number.unsignedIntValue == display) {
			return screen;
		}
	}
	return nil;
}

static NSRect
cocoa_rect_from_cg_bounds(CGRect bounds)
{
	const CGFloat main_h = CGDisplayBounds(CGMainDisplayID()).size.height;
	return NSMakeRect(bounds.origin.x, main_h - bounds.origin.y - bounds.size.height, bounds.size.width,
	                  bounds.size.height);
}

/*
 *
 * Implementation.
 *
 */

struct WmrPanel::Impl
{
	// CPU copy of the driver's mesh, uploaded by EnsureRenderer().
	std::vector<float> meshVertices;
	uint32_t meshStride = 0;
	std::vector<uint32_t> meshIndices;
	uint32_t meshIndexOffsets[2] = {};
	uint32_t meshIndexCounts[2] = {};
	bool meshOk = false;

	std::mutex rendererMutex;
	id<MTLDevice> device = nil;
	id<MTLRenderPipelineState> meshPipeline = nil;
	id<MTLBuffer> meshVertexBuffer = nil;
	id<MTLBuffer> meshIndexBuffer = nil;
	bool pipelineOk = false;

	bool captured = false;

	// Touched on the main thread (window creation) and the presenter.
	std::mutex windowMutex;
	NSWindow *window = nil;
	CAMetalLayer *layer = nil;
	std::atomic<bool> ready{false};
	std::atomic<bool> closing{false};
};

WmrPanel::WmrPanel(const struct xrt_hmd_parts *parts) : impl_(new Impl)
{
	geometry_.panel_w = (uint32_t)parts->screens[0].w_pixels;
	geometry_.panel_h = (uint32_t)parts->screens[0].h_pixels;
	geometry_.view_count = (uint32_t)parts->view_count > 2 ? 2 : (uint32_t)parts->view_count;
	for (uint32_t i = 0; i < geometry_.view_count; i++) {
		WmrPanelView &v = geometry_.views[i];
		v.x_pixels = parts->views[i].viewport.x_pixels;
		v.y_pixels = parts->views[i].viewport.y_pixels;
		v.w_pixels = parts->views[i].viewport.w_pixels;
		v.h_pixels = parts->views[i].viewport.h_pixels;
		v.render_w = parts->views[i].display.w_pixels;
		v.render_h = parts->views[i].display.h_pixels;
		v.fov[0] = parts->distortion.fov[i].angle_left;
		v.fov[1] = parts->distortion.fov[i].angle_right;
		v.fov[2] = parts->distortion.fov[i].angle_up;
		v.fov[3] = parts->distortion.fov[i].angle_down;
	}

	const auto &mesh = parts->distortion.mesh;
	if (mesh.vertices == NULL || mesh.uv_channels_count != 3 || mesh.index_count_total == 0) {
		fprintf(stderr, "WmrPanel: driver has no 3-channel distortion mesh\n");
		return;
	}
	impl_->meshStride = mesh.stride;
	impl_->meshVertices.assign(mesh.vertices, mesh.vertices + (size_t)mesh.vertex_count * (mesh.stride / sizeof(float)));
	impl_->meshIndices.resize(mesh.index_count_total);
	for (uint32_t i = 0; i < mesh.index_count_total; i++) {
		impl_->meshIndices[i] = (uint32_t)mesh.indices[i];
	}
	for (uint32_t i = 0; i < geometry_.view_count; i++) {
		impl_->meshIndexOffsets[i] = mesh.index_offsets[i];
		impl_->meshIndexCounts[i] = mesh.index_counts[i];
	}
	impl_->meshOk = true;
}

WmrPanel::~WmrPanel()
{
	Close();
	delete impl_;
}

std::vector<CGDirectDisplayID>
WmrPanel::OnlineDisplays()
{
	return online_displays();
}

bool
WmrPanel::EnsureRenderer(id<MTLDevice> device)
{
	std::lock_guard<std::mutex> lock(impl_->rendererMutex);
	if (impl_->pipelineOk && impl_->device == device) {
		return true;
	}
	if (!impl_->meshOk || device == nil) {
		return false;
	}

	NSError *error = nil;
	id<MTLLibrary> library = [device newLibraryWithSource:@(kPanelShaderSource) options:nil error:&error];
	if (library == nil) {
		fprintf(stderr, "WmrPanel: shader compile failed: %s\n", error.localizedDescription.UTF8String);
		return false;
	}

	MTLVertexDescriptor *vdesc = [MTLVertexDescriptor new];
	vdesc.attributes[0].format = MTLVertexFormatFloat4;
	vdesc.attributes[0].offset = 0;
	vdesc.attributes[0].bufferIndex = 0;
	vdesc.attributes[1].format = MTLVertexFormatFloat4;
	vdesc.attributes[1].offset = 16;
	vdesc.attributes[1].bufferIndex = 0;
	vdesc.layouts[0].stride = impl_->meshStride;
	vdesc.layouts[0].stepFunction = MTLVertexStepFunctionPerVertex;

	MTLRenderPipelineDescriptor *desc = [MTLRenderPipelineDescriptor new];
	desc.vertexFunction = [library newFunctionWithName:@"mesh_vertex"];
	desc.fragmentFunction = [library newFunctionWithName:@"mesh_fragment"];
	desc.vertexDescriptor = vdesc;
	desc.colorAttachments[0].pixelFormat = MTLPixelFormatBGRA8Unorm;
	id<MTLRenderPipelineState> pipeline = [device newRenderPipelineStateWithDescriptor:desc error:&error];
	if (pipeline == nil) {
		fprintf(stderr, "WmrPanel: mesh pipeline failed: %s\n", error.localizedDescription.UTF8String);
		return false;
	}

	impl_->meshPipeline = pipeline;
	impl_->meshVertexBuffer = [device newBufferWithBytes:impl_->meshVertices.data()
	                                              length:impl_->meshVertices.size() * sizeof(float)
	                                             options:MTLResourceStorageModeShared];
	impl_->meshIndexBuffer = [device newBufferWithBytes:impl_->meshIndices.data()
	                                             length:impl_->meshIndices.size() * sizeof(uint32_t)
	                                            options:MTLResourceStorageModeShared];
	impl_->device = device;
	impl_->pipelineOk = true;

	// The layer may already exist (Open() before AttachGraphics()).
	CAMetalLayer *layer = nil;
	{
		std::lock_guard<std::mutex> wlock(impl_->windowMutex);
		layer = impl_->layer;
	}
	if (layer != nil) {
		layer.device = device;
	}
	return true;
}

bool
WmrPanel::Open(const WmrPanelOptions &options)
{
	if (!impl_->meshOk) {
		return false;
	}

	CGDirectDisplayID display = options.display_id;
	if (display == 0) {
		const auto deadline = std::chrono::steady_clock::now() +
		                      std::chrono::milliseconds((int64_t)(options.display_timeout_s * 1000.0));
		while (display == 0 && !impl_->closing.load()) {
			display = find_panel_display(geometry_.panel_w, geometry_.panel_h, options.displays_before);
			if (display != 0 || std::chrono::steady_clock::now() >= deadline) {
				break;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(250));
		}
	}
	if (display == 0) {
		fprintf(stderr, "WmrPanel: no display for a %ux%u panel appeared within %.0f s\n", geometry_.panel_w,
		        geometry_.panel_h, options.display_timeout_s);
		return false;
	}

	if (CGDisplayIsInMirrorSet(display)) {
		CGDisplayConfigRef config = NULL;
		if (CGBeginDisplayConfiguration(&config) == kCGErrorSuccess) {
			CGConfigureDisplayMirrorOfDisplay(config, display, kCGNullDirectDisplay);
			CGCompleteDisplayConfiguration(config, kCGConfigureForSession);
		}
	}

	bool exact = false;
	CGDisplayModeRef mode = find_native_mode(display, geometry_.panel_w, geometry_.panel_h, &exact);
	if (mode == NULL) {
		fprintf(stderr, "WmrPanel: display %u reports no modes\n", display);
		return false;
	}
	if (options.capture && CGDisplayCapture(display) == kCGErrorSuccess) {
		impl_->captured = true;
	}
	CGDisplayModeRef current = CGDisplayCopyDisplayMode(display);
	const bool needsSwitch = current == NULL || !CFEqual(current, mode);
	if (current != NULL) {
		CGDisplayModeRelease(current);
	}
	if (needsSwitch) {
		CGDisplaySetDisplayMode(display, mode, NULL);
		std::this_thread::sleep_for(std::chrono::milliseconds(700));
	}
	CGDisplayModeRelease(mode);
	mode = CGDisplayCopyDisplayMode(display);
	refreshHz_ = mode ? CGDisplayModeGetRefreshRate(mode) : 0.0;
	const size_t gotW = mode ? CGDisplayModeGetPixelWidth(mode) : 0;
	const size_t gotH = mode ? CGDisplayModeGetPixelHeight(mode) : 0;
	if (mode) {
		CGDisplayModeRelease(mode);
	}
	displayId_ = display;
	fprintf(stderr, "WmrPanel: display %u at %zux%zu @ %.1f Hz%s\n", display, gotW, gotH, refreshHz_,
	        exact ? "" : " (not the panel's native size, image will be scaled)");

	// AppKit windows must be created on the main thread. Do it directly when we
	// are already there; otherwise queue it and let frames drop until ready.
	const bool captured = impl_->captured;
	Impl *impl = impl_;
	const uint32_t viewCount = geometry_.view_count;
	(void)viewCount;
	void (^create)(void) = ^{
	  if (impl->closing.load()) {
		  return;
	  }
	  NSScreen *screen = screen_for_display(display);
	  NSRect frame = cocoa_rect_from_cg_bounds(CGDisplayBounds(display));
	  if (screen != nil) {
		  frame = screen.frame;
	  } else {
		  screen = [NSScreen mainScreen];
	  }
	  const CGFloat scale = screen != nil ? screen.backingScaleFactor : 1.0;

	  NSWindow *window = [[NSWindow alloc] initWithContentRect:frame
	                                                 styleMask:NSWindowStyleMaskBorderless
	                                                   backing:NSBackingStoreBuffered
	                                                     defer:NO
	                                                    screen:screen];
	  window.title = @"OXRSys headset panel";
	  window.releasedWhenClosed = NO;
	  window.backgroundColor = [NSColor blackColor];
	  window.level = captured ? CGShieldingWindowLevel() : NSScreenSaverWindowLevel;
	  window.hidesOnDeactivate = NO;
	  window.collectionBehavior = NSWindowCollectionBehaviorCanJoinAllSpaces | NSWindowCollectionBehaviorStationary |
	                              NSWindowCollectionBehaviorFullScreenAuxiliary |
	                              NSWindowCollectionBehaviorIgnoresCycle;

	  NSView *view = [[NSView alloc] initWithFrame:NSMakeRect(0, 0, frame.size.width, frame.size.height)];
	  CAMetalLayer *layer = [CAMetalLayer layer];
	  layer.device = impl->device;
	  layer.pixelFormat = MTLPixelFormatBGRA8Unorm;
	  layer.framebufferOnly = YES;
	  layer.contentsScale = scale;
	  layer.drawableSize = CGSizeMake(frame.size.width * scale, frame.size.height * scale);
	  layer.displaySyncEnabled = YES;
	  layer.maximumDrawableCount = 3;
	  layer.frame = view.bounds;
	  view.layer = layer;
	  view.wantsLayer = YES;
	  view.autoresizingMask = NSViewWidthSizable | NSViewHeightSizable;
	  window.contentView = view;
	  [window setFrame:frame display:YES];
	  [window orderFrontRegardless];

	  {
		  std::lock_guard<std::mutex> lock(impl->windowMutex);
		  impl->window = window;
		  impl->layer = layer;
	  }
	  impl->ready.store(true);
	  fprintf(stderr, "WmrPanel: window up, drawable %.0fx%.0f\n", layer.drawableSize.width, layer.drawableSize.height);
	};
	if ([NSThread isMainThread]) {
		create();
	} else {
		dispatch_async(dispatch_get_main_queue(), create);
	}
	return true;
}

bool
WmrPanel::IsReady() const
{
	return impl_->ready.load();
}

void
WmrPanel::Close()
{
	impl_->closing.store(true);
	impl_->ready.store(false);
	NSWindow *window = nil;
	{
		std::lock_guard<std::mutex> lock(impl_->windowMutex);
		window = impl_->window;
		impl_->window = nil;
		impl_->layer = nil;
	}
	if (window != nil) {
		void (^close)(void) = ^{
		  [window orderOut:nil];
		};
		if ([NSThread isMainThread]) {
			close();
		} else {
			dispatch_async(dispatch_get_main_queue(), close);
		}
	}
	if (impl_->captured) {
		CGDisplayRelease(displayId_);
		impl_->captured = false;
	}
}

bool
WmrPanel::Present(id<MTLCommandQueue> queue,
                  const WmrPanelEyeInput &left,
                  const WmrPanelEyeInput &right,
                  void (^completed)(void))
{
	CAMetalLayer *layer = nil;
	{
		std::lock_guard<std::mutex> lock(impl_->windowMutex);
		layer = impl_->layer;
	}
	if (layer == nil || !impl_->ready.load()) {
		return false;
	}

	@autoreleasepool {
		id<CAMetalDrawable> drawable = [layer nextDrawable];
		if (drawable == nil) {
			return false;
		}

		id<MTLCommandBuffer> cmd = [queue commandBuffer];
		const WmrPanelEyeInput *eyes[2] = {&left, &right};
		for (uint32_t i = 0; i < geometry_.view_count; i++) {
			if (eyes[i]->waitEvent != nil && eyes[i]->waitValue != 0) {
				[cmd encodeWaitForEvent:eyes[i]->waitEvent value:eyes[i]->waitValue];
			}
		}

		MTLRenderPassDescriptor *rp = [MTLRenderPassDescriptor renderPassDescriptor];
		rp.colorAttachments[0].texture = drawable.texture;
		rp.colorAttachments[0].loadAction = MTLLoadActionClear;
		rp.colorAttachments[0].storeAction = MTLStoreActionStore;
		rp.colorAttachments[0].clearColor = MTLClearColorMake(0, 0, 0, 1);
		id<MTLRenderCommandEncoder> enc = [cmd renderCommandEncoderWithDescriptor:rp];
		[enc setRenderPipelineState:impl_->meshPipeline];
		[enc setVertexBuffer:impl_->meshVertexBuffer offset:0 atIndex:0];

		// The drawable may not be exactly the panel size (scaled mode); map
		// the panel viewports proportionally.
		const double sx = (double)drawable.texture.width / (double)geometry_.panel_w;
		const double sy = (double)drawable.texture.height / (double)geometry_.panel_h;
		for (uint32_t i = 0; i < geometry_.view_count; i++) {
			if (eyes[i]->texture == nil) {
				continue;
			}
			const WmrPanelView &v = geometry_.views[i];
			MTLViewport vp = {v.x_pixels * sx, v.y_pixels * sy, v.w_pixels * sx, v.h_pixels * sy, 0.0, 1.0};
			[enc setViewport:vp];
			MTLScissorRect sc = {(NSUInteger)(v.x_pixels * sx), (NSUInteger)(v.y_pixels * sy),
			                     (NSUInteger)(v.w_pixels * sx), (NSUInteger)(v.h_pixels * sy)};
			[enc setScissorRect:sc];
			[enc setFragmentTexture:eyes[i]->texture atIndex:0];
			[enc drawIndexedPrimitives:MTLPrimitiveTypeTriangleStrip
			                indexCount:impl_->meshIndexCounts[i]
			                 indexType:MTLIndexTypeUInt32
			               indexBuffer:impl_->meshIndexBuffer
			         indexBufferOffset:impl_->meshIndexOffsets[i] * sizeof(uint32_t)];
		}
		[enc endEncoding];

		if (completed != nil) {
			[cmd addCompletedHandler:^(id<MTLCommandBuffer> buffer) {
			  (void)buffer;
			  completed();
			}];
		}
		[cmd presentDrawable:drawable];
		[cmd commit];
		presentedFrames_++;
	}
	return true;
}

bool
WmrPanel::PresentClear(id<MTLCommandQueue> queue, double r, double g, double b)
{
	CAMetalLayer *layer = nil;
	{
		std::lock_guard<std::mutex> lock(impl_->windowMutex);
		layer = impl_->layer;
	}
	if (layer == nil || !impl_->ready.load()) {
		return false;
	}
	@autoreleasepool {
		id<CAMetalDrawable> drawable = [layer nextDrawable];
		if (drawable == nil) {
			return false;
		}
		id<MTLCommandBuffer> cmd = [queue commandBuffer];
		MTLRenderPassDescriptor *rp = [MTLRenderPassDescriptor renderPassDescriptor];
		rp.colorAttachments[0].texture = drawable.texture;
		rp.colorAttachments[0].loadAction = MTLLoadActionClear;
		rp.colorAttachments[0].storeAction = MTLStoreActionStore;
		rp.colorAttachments[0].clearColor = MTLClearColorMake(r, g, b, 1);
		id<MTLRenderCommandEncoder> enc = [cmd renderCommandEncoderWithDescriptor:rp];
		[enc endEncoding];
		[cmd presentDrawable:drawable];
		[cmd commit];
	}
	return true;
}

} // namespace oxrsys
