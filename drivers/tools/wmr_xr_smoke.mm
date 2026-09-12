// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief End-to-end check of the wired headset through OpenXR.
 *
 * A minimal native OpenXR client: loader -> OXRSys runtime -> WiredHeadset.
 * Renders a tangent-space grid per eye, tinted by the view's orientation, and
 * submits it as a projection layer, so a working setup shows a grid on the
 * panel that stays put while the head turns and changes colour with yaw.
 *
 * Run with the build's runtime manifest and a config that enables the wired
 * headset, for example:
 *   XR_RUNTIME_JSON=build/runtime/oxrsys-runtime.json ./build/drivers/oxrsys_wmr_xr_smoke --seconds 20
 *
 * Usage: oxrsys_wmr_xr_smoke [--seconds N]
 */

#define XR_USE_GRAPHICS_API_METAL
#import <Metal/Metal.h>
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

static const char *kShader = R"MSL(
#include <metal_stdlib>
using namespace metal;
struct Uniforms { float4 tan_fov; float3 tint; float time; };
struct Out { float4 position [[position]]; float2 uv; };
vertex Out vs(uint vid [[vertex_id]]) {
    float2 pos = float2((vid == 1) ? 3.0 : -1.0, (vid == 2) ? -3.0 : 1.0);
    Out o; o.position = float4(pos, 0, 1); o.uv = float2((pos.x + 1.0) * 0.5, (1.0 - pos.y) * 0.5); return o;
}
fragment float4 fs(Out in [[stage_in]], constant Uniforms &u [[buffer(0)]]) {
    float2 t = float2(mix(u.tan_fov.x, u.tan_fov.y, in.uv.x), mix(u.tan_fov.z, u.tan_fov.w, in.uv.y));
    float2 g = abs(fract(t * 4.0) - 0.5);
    float grid = (min(g.x, g.y) < 0.03) ? 1.0 : 0.0;
    float r = length(t);
    float ring = (abs(r - 0.5) < 0.01 || abs(r - 1.0) < 0.01) ? 1.0 : 0.0;
    float cross = (abs(t.x) < 0.006 || abs(t.y) < 0.006) ? 1.0 : 0.0;
    float3 c = u.tint * 0.35 + float3(grid) * 0.5 + float3(ring) * float3(1.0, 0.8, 0.2) + float3(cross) * float3(1.0, 0.2, 0.2);
    if (in.uv.x < 0.01 || in.uv.x > 0.99 || in.uv.y < 0.01 || in.uv.y > 0.99) c = float3(0.2, 1.0, 0.2);
    return float4(c, 1.0);
}
)MSL";

struct Uniforms
{
	float tan_fov[4];
	float tint[3];
	float time;
};

#define XR_CHECK(expr)                                                                                             \
	do {                                                                                                           \
		XrResult _r = (expr);                                                                                      \
		if (XR_FAILED(_r)) {                                                                                       \
			fprintf(stderr, "%s failed: %d\n", #expr, (int)_r);                                                    \
			return 1;                                                                                              \
		}                                                                                                          \
	} while (0)

int
main(int argc, char **argv)
{
	double seconds = 20.0;
	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "--seconds") == 0 && i + 1 < argc) {
			seconds = atof(argv[++i]);
		}
	}

	@autoreleasepool {
		id<MTLDevice> device = MTLCreateSystemDefaultDevice();
		id<MTLCommandQueue> queue = [device newCommandQueue];

		NSError *error = nil;
		id<MTLLibrary> lib = [device newLibraryWithSource:@(kShader) options:nil error:&error];
		if (lib == nil) {
			fprintf(stderr, "shader: %s\n", error.localizedDescription.UTF8String);
			return 1;
		}

		XrInstance instance = XR_NULL_HANDLE;
		const char *extensions[] = {XR_KHR_METAL_ENABLE_EXTENSION_NAME};
		XrInstanceCreateInfo ici = {XR_TYPE_INSTANCE_CREATE_INFO};
		strncpy(ici.applicationInfo.applicationName, "oxrsys_wmr_xr_smoke", XR_MAX_APPLICATION_NAME_SIZE - 1);
		ici.applicationInfo.applicationVersion = 1;
		strncpy(ici.applicationInfo.engineName, "none", XR_MAX_ENGINE_NAME_SIZE - 1);
		ici.applicationInfo.apiVersion = XR_API_VERSION_1_0;
		ici.enabledExtensionCount = 1;
		ici.enabledExtensionNames = extensions;
		XR_CHECK(xrCreateInstance(&ici, &instance));

		XrInstanceProperties props = {XR_TYPE_INSTANCE_PROPERTIES};
		XR_CHECK(xrGetInstanceProperties(instance, &props));
		printf("Runtime: %s\n", props.runtimeName);

		XrSystemGetInfo sgi = {XR_TYPE_SYSTEM_GET_INFO};
		sgi.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
		XrSystemId systemId = XR_NULL_SYSTEM_ID;
		XR_CHECK(xrGetSystem(instance, &sgi, &systemId));

		PFN_xrGetMetalGraphicsRequirementsKHR getReqs = nullptr;
		XR_CHECK(xrGetInstanceProcAddr(instance, "xrGetMetalGraphicsRequirementsKHR",
		                               (PFN_xrVoidFunction *)&getReqs));
		XrGraphicsRequirementsMetalKHR reqs = {XR_TYPE_GRAPHICS_REQUIREMENTS_METAL_KHR};
		XR_CHECK(getReqs(instance, systemId, &reqs));

		uint32_t viewCount = 0;
		XR_CHECK(xrEnumerateViewConfigurationViews(instance, systemId, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 0,
		                                           &viewCount, nullptr));
		std::vector<XrViewConfigurationView> configViews(viewCount, {XR_TYPE_VIEW_CONFIGURATION_VIEW});
		XR_CHECK(xrEnumerateViewConfigurationViews(instance, systemId, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,
		                                           viewCount, &viewCount, configViews.data()));
		printf("Views: %u, recommended %ux%u\n", viewCount, configViews[0].recommendedImageRectWidth,
		       configViews[0].recommendedImageRectHeight);

		XrGraphicsBindingMetalKHR binding = {XR_TYPE_GRAPHICS_BINDING_METAL_KHR};
		binding.commandQueue = (__bridge void *)queue;
		XrSessionCreateInfo sci = {XR_TYPE_SESSION_CREATE_INFO};
		sci.next = &binding;
		sci.systemId = systemId;
		XrSession session = XR_NULL_HANDLE;
		XR_CHECK(xrCreateSession(instance, &sci, &session));

		uint32_t formatCount = 0;
		XR_CHECK(xrEnumerateSwapchainFormats(session, 0, &formatCount, nullptr));
		std::vector<int64_t> formats(formatCount);
		XR_CHECK(xrEnumerateSwapchainFormats(session, formatCount, &formatCount, formats.data()));
		const int64_t format = formats.front();

		MTLRenderPipelineDescriptor *pd = [MTLRenderPipelineDescriptor new];
		pd.vertexFunction = [lib newFunctionWithName:@"vs"];
		pd.fragmentFunction = [lib newFunctionWithName:@"fs"];
		pd.colorAttachments[0].pixelFormat = (MTLPixelFormat)format;
		id<MTLRenderPipelineState> pipeline = [device newRenderPipelineStateWithDescriptor:pd error:&error];
		if (pipeline == nil) {
			fprintf(stderr, "pipeline (format %lld): %s\n", (long long)format, error.localizedDescription.UTF8String);
			return 1;
		}

		std::vector<XrSwapchain> swapchains(viewCount);
		std::vector<std::vector<XrSwapchainImageMetalKHR>> images(viewCount);
		for (uint32_t i = 0; i < viewCount; i++) {
			XrSwapchainCreateInfo sc = {XR_TYPE_SWAPCHAIN_CREATE_INFO};
			sc.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_SAMPLED_BIT;
			sc.format = format;
			sc.sampleCount = 1;
			sc.width = configViews[i].recommendedImageRectWidth;
			sc.height = configViews[i].recommendedImageRectHeight;
			sc.faceCount = 1;
			sc.arraySize = 1;
			sc.mipCount = 1;
			XR_CHECK(xrCreateSwapchain(session, &sc, &swapchains[i]));
			uint32_t imageCount = 0;
			XR_CHECK(xrEnumerateSwapchainImages(swapchains[i], 0, &imageCount, nullptr));
			images[i].assign(imageCount, {XR_TYPE_SWAPCHAIN_IMAGE_METAL_KHR});
			XR_CHECK(xrEnumerateSwapchainImages(swapchains[i], imageCount, &imageCount,
			                                    (XrSwapchainImageBaseHeader *)images[i].data()));
		}

		XrReferenceSpaceCreateInfo rsci = {XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
		rsci.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
		rsci.poseInReferenceSpace.orientation.w = 1.0f;
		XrSpace space = XR_NULL_HANDLE;
		XR_CHECK(xrCreateReferenceSpace(session, &rsci, &space));

		// Pump events until the session is ready, then begin it.
		bool running = false;
		bool exitRequested = false;
		XrSessionState state = XR_SESSION_STATE_UNKNOWN;
		const auto start = std::chrono::steady_clock::now();
		uint32_t frames = 0;
		while (!exitRequested) {
			XrEventDataBuffer event = {XR_TYPE_EVENT_DATA_BUFFER};
			while (xrPollEvent(instance, &event) == XR_SUCCESS) {
				if (event.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED) {
					state = ((XrEventDataSessionStateChanged *)&event)->state;
					if (state == XR_SESSION_STATE_READY) {
						XrSessionBeginInfo bi = {XR_TYPE_SESSION_BEGIN_INFO};
						bi.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
						XR_CHECK(xrBeginSession(session, &bi));
						running = true;
					} else if (state == XR_SESSION_STATE_STOPPING) {
						XR_CHECK(xrEndSession(session));
						running = false;
					} else if (state == XR_SESSION_STATE_EXITING || state == XR_SESSION_STATE_LOSS_PENDING) {
						exitRequested = true;
					}
				}
				event = {XR_TYPE_EVENT_DATA_BUFFER};
			}
			const double elapsed =
			    std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
			if (elapsed >= seconds) {
				break;
			}
			if (!running) {
				[NSThread sleepForTimeInterval:0.01];
				continue;
			}

			XrFrameState frameState = {XR_TYPE_FRAME_STATE};
			XR_CHECK(xrWaitFrame(session, nullptr, &frameState));
			XR_CHECK(xrBeginFrame(session, nullptr));

			XrViewLocateInfo vli = {XR_TYPE_VIEW_LOCATE_INFO};
			vli.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
			vli.displayTime = frameState.predictedDisplayTime;
			vli.space = space;
			XrViewState viewState = {XR_TYPE_VIEW_STATE};
			std::vector<XrView> views(viewCount, {XR_TYPE_VIEW});
			uint32_t located = 0;
			XR_CHECK(xrLocateViews(session, &vli, &viewState, viewCount, &located, views.data()));

			std::vector<XrCompositionLayerProjectionView> projViews(viewCount,
			                                                        {XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW});
			id<MTLCommandBuffer> cmd = [queue commandBuffer];
			for (uint32_t i = 0; i < viewCount; i++) {
				uint32_t index = 0;
				XR_CHECK(xrAcquireSwapchainImage(swapchains[i], nullptr, &index));
				XrSwapchainImageWaitInfo wi = {XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
				wi.timeout = XR_INFINITE_DURATION;
				XR_CHECK(xrWaitSwapchainImage(swapchains[i], &wi));

				id<MTLTexture> target = (__bridge id<MTLTexture>)images[i][index].texture;
				MTLRenderPassDescriptor *rp = [MTLRenderPassDescriptor renderPassDescriptor];
				rp.colorAttachments[0].texture = target;
				rp.colorAttachments[0].loadAction = MTLLoadActionClear;
				rp.colorAttachments[0].storeAction = MTLStoreActionStore;
				id<MTLRenderCommandEncoder> enc = [cmd renderCommandEncoderWithDescriptor:rp];
				Uniforms u;
				u.tan_fov[0] = tanf(views[i].fov.angleLeft);
				u.tan_fov[1] = tanf(views[i].fov.angleRight);
				u.tan_fov[2] = tanf(views[i].fov.angleUp);
				u.tan_fov[3] = tanf(views[i].fov.angleDown);
				// Tint from the view's forward vector: turning the head changes the colour.
				const XrQuaternionf q = views[i].pose.orientation;
				const float fx = 2.0f * (q.x * q.z + q.w * q.y);
				const float fy = 2.0f * (q.y * q.z - q.w * q.x);
				const float fz = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
				u.tint[0] = 0.5f + 0.5f * fx;
				u.tint[1] = 0.5f + 0.5f * fy;
				u.tint[2] = 0.5f + 0.5f * fz;
				u.time = (float)elapsed;
				[enc setRenderPipelineState:pipeline];
				[enc setFragmentBytes:&u length:sizeof(u) atIndex:0];
				[enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
				[enc endEncoding];

				projViews[i].pose = views[i].pose;
				projViews[i].fov = views[i].fov;
				projViews[i].subImage.swapchain = swapchains[i];
				projViews[i].subImage.imageRect.offset = {0, 0};
				projViews[i].subImage.imageRect.extent = {(int32_t)configViews[i].recommendedImageRectWidth,
				                                          (int32_t)configViews[i].recommendedImageRectHeight};
				projViews[i].subImage.imageArrayIndex = 0;
			}
			[cmd commit];
			for (uint32_t i = 0; i < viewCount; i++) {
				XR_CHECK(xrReleaseSwapchainImage(swapchains[i], nullptr));
			}

			XrCompositionLayerProjection layer = {XR_TYPE_COMPOSITION_LAYER_PROJECTION};
			layer.space = space;
			layer.viewCount = viewCount;
			layer.views = projViews.data();
			const XrCompositionLayerBaseHeader *layers[] = {(const XrCompositionLayerBaseHeader *)&layer};
			XrFrameEndInfo fei = {XR_TYPE_FRAME_END_INFO};
			fei.displayTime = frameState.predictedDisplayTime;
			fei.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
			fei.layerCount = 1;
			fei.layers = layers;
			XR_CHECK(xrEndFrame(session, &fei));
			frames++;

			if (frames % 90 == 0) {
				const XrQuaternionf q = views[0].pose.orientation;
				printf("[%5.1f s] %u frames, period %.2f ms, head q=(%.3f %.3f %.3f %.3f) flags=0x%llx\n", elapsed,
				       frames, frameState.predictedDisplayPeriod / 1e6, q.x, q.y, q.z, q.w,
				       (unsigned long long)viewState.viewStateFlags);
				fflush(stdout);
			}
		}

		printf("%u frames submitted.\n", frames);
		if (running) {
			XR_CHECK(xrRequestExitSession(session));
		}
		for (XrSwapchain sc : swapchains) {
			xrDestroySwapchain(sc);
		}
		xrDestroySpace(space);
		xrDestroySession(session);
		xrDestroyInstance(instance);
	}
	return 0;
}
