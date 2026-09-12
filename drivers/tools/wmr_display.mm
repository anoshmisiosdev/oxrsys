// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief Bring-up tool: put a distortion-corrected stereo test scene on a
 * Windows Mixed Reality headset's panel.
 *
 * Flow:
 *   1. Open the headset through the Monado driver (this switches the panel on).
 *   2. Wait for macOS to enumerate the panel as a display, or use --display-id.
 *   3. Switch it to its native mode, capture it, and cover it with a Metal layer.
 *   4. Every refresh: read the IMU pose, render a procedural room per eye,
 *      then warp both eyes through the driver's per-channel distortion mesh.
 *
 * Usage:
 *   oxrsys_wmr_display [--simulate] [--display-id ID] [--display-timeout S]
 *                      [--seconds N] [--pattern] [--no-distortion]
 *                      [--screenshot PATH] [--log-level LEVEL]
 *
 *   --simulate        Synthetic headset in a desktop window; no hardware needed.
 *   --display-id ID   Use this CGDirectDisplayID instead of auto-detecting.
 *   --display-timeout Seconds to wait for the panel to appear (default 20).
 *   --seconds N       Exit after N seconds.
 *   --pattern         Draw a lens-calibration pattern instead of the room.
 *   --no-distortion   Blit the eye images straight to the panel.
 *   --screenshot PATH Write the panel image as PNG after ~30 frames, then exit.
 */

#import <AppKit/AppKit.h>
#import <CoreGraphics/CoreGraphics.h>
#import <ImageIO/ImageIO.h>
#import <Metal/Metal.h>
#import <QuartzCore/QuartzCore.h>
#import <simd/simd.h>

#include "wmr_macos.h"

#include "math/m_api.h"
#include "os/os_time.h"
#include "util/u_logging.h"
#include "xrt/xrt_device.h"

#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

/*
 *
 * Options.
 *
 */

struct options
{
	bool simulate = false;
	bool pattern = false;
	bool distortion = true;
	CGDirectDisplayID display_id = kCGNullDirectDisplay;
	double display_timeout_s = 20.0;
	double seconds = -1.0;
	const char *screenshot = NULL;
	int screenshot_frame = 30;
	enum u_logging_level log_level = U_LOGGING_INFO;
};

static void
usage(const char *argv0)
{
	fprintf(stderr,
	        "usage: %s [--simulate] [--display-id ID] [--display-timeout S] [--seconds N]\n"
	        "          [--pattern] [--no-distortion] [--screenshot PATH] [--log-level LEVEL]\n",
	        argv0);
}

static bool
parse_log_level(const char *text, enum u_logging_level *out_level)
{
	static const struct
	{
		const char *name;
		enum u_logging_level level;
	} table[] = {
	    {"trace", U_LOGGING_TRACE}, {"debug", U_LOGGING_DEBUG}, {"info", U_LOGGING_INFO},
	    {"warn", U_LOGGING_WARN},   {"error", U_LOGGING_ERROR},
	};
	for (const auto &entry : table) {
		if (strcmp(text, entry.name) == 0) {
			*out_level = entry.level;
			return true;
		}
	}
	return false;
}

static bool
parse_options(int argc, char **argv, options *opts)
{
	for (int i = 1; i < argc; i++) {
		const char *arg = argv[i];
		const bool has_value = i + 1 < argc;
		if (strcmp(arg, "--simulate") == 0) {
			opts->simulate = true;
		} else if (strcmp(arg, "--pattern") == 0) {
			opts->pattern = true;
		} else if (strcmp(arg, "--no-distortion") == 0) {
			opts->distortion = false;
		} else if (strcmp(arg, "--display-id") == 0 && has_value) {
			opts->display_id = (CGDirectDisplayID)strtoul(argv[++i], NULL, 0);
		} else if (strcmp(arg, "--display-timeout") == 0 && has_value) {
			opts->display_timeout_s = atof(argv[++i]);
		} else if (strcmp(arg, "--seconds") == 0 && has_value) {
			opts->seconds = atof(argv[++i]);
		} else if (strcmp(arg, "--screenshot") == 0 && has_value) {
			opts->screenshot = argv[++i];
		} else if (strcmp(arg, "--log-level") == 0 && has_value) {
			if (!parse_log_level(argv[++i], &opts->log_level)) {
				return false;
			}
		} else {
			return false;
		}
	}
	return true;
}

/*
 *
 * Small quaternion helpers (OpenXR convention: x, y, z, w).
 *
 */

static struct xrt_quat
quat_mul(const struct xrt_quat &a, const struct xrt_quat &b)
{
	struct xrt_quat q;
	q.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
	q.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
	q.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
	q.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
	return q;
}

static struct xrt_vec3
quat_rotate(const struct xrt_quat &q, const struct xrt_vec3 &v)
{
	// v' = v + 2w(q x v) + 2 q x (q x v)
	const float cx = q.y * v.z - q.z * v.y;
	const float cy = q.z * v.x - q.x * v.z;
	const float cz = q.x * v.y - q.y * v.x;
	const float ccx = q.y * cz - q.z * cy;
	const float ccy = q.z * cx - q.x * cz;
	const float ccz = q.x * cy - q.y * cx;
	return {v.x + 2.0f * (q.w * cx + ccx), v.y + 2.0f * (q.w * cy + ccy), v.z + 2.0f * (q.w * cz + ccz)};
}

static simd_float3x3
quat_to_simd_matrix(const struct xrt_quat &q)
{
	const struct xrt_vec3 cx = quat_rotate(q, {1.0f, 0.0f, 0.0f});
	const struct xrt_vec3 cy = quat_rotate(q, {0.0f, 1.0f, 0.0f});
	const struct xrt_vec3 cz = quat_rotate(q, {0.0f, 0.0f, 1.0f});
	return simd_matrix(simd_make_float3(cx.x, cx.y, cx.z), simd_make_float3(cy.x, cy.y, cy.z),
	                   simd_make_float3(cz.x, cz.y, cz.z));
}

/*
 *
 * Headset description shared by the real and simulated paths.
 *
 */

struct view_geometry
{
	uint32_t x_pixels, y_pixels, w_pixels, h_pixels; // Viewport on the panel.
	uint32_t render_w, render_h;                     // Eye image size.
	struct xrt_fov fov;
};

struct display_source
{
	uint32_t panel_w = 0;
	uint32_t panel_h = 0;
	uint32_t view_count = 0;
	view_geometry views[XRT_MAX_VIEWS] = {};

	// Distortion mesh in Monado's layout: [x, y, ru, rv, gu, gv, bu, bv] per
	// vertex with x,y in [-1,1] (Vulkan orientation: -1 is the top), triangle
	// strips with degenerate joins, one strip per view.
	std::vector<float> mesh_vertices;
	uint32_t mesh_stride_floats = 8;
	std::vector<uint32_t> mesh_indices;
	uint32_t mesh_index_offsets[XRT_MAX_VIEWS] = {};
	uint32_t mesh_index_counts[XRT_MAX_VIEWS] = {};

	// Filled per frame.
	virtual bool get_eye_poses(int64_t at_timestamp_ns, struct xrt_pose *out_poses, uint32_t count) = 0;
	virtual ~display_source() = default;
};

struct wmr_display_source : display_source
{
	struct oxrsys_wmr_headset *headset = NULL;

	~wmr_display_source() override
	{
		oxrsys_wmr_headset_close(&headset);
	}

	bool get_eye_poses(int64_t at_timestamp_ns, struct xrt_pose *out_poses, uint32_t count) override
	{
		const struct xrt_vec3 default_eye_relation = {0.063f, 0.0f, 0.0f};
		struct xrt_space_relation head = {};
		struct xrt_fov fovs[XRT_MAX_VIEWS] = {};
		xrt_result_t xret = headset->hmd->get_view_poses(headset->hmd, &default_eye_relation, at_timestamp_ns,
		                                                 XRT_VIEW_TYPE_STEREO, count, &head, fovs, out_poses);
		return xret == XRT_SUCCESS;
	}
};

static bool
wmr_display_source_init(wmr_display_source *src, enum u_logging_level log_level)
{
	enum oxrsys_wmr_open_result result = oxrsys_wmr_headset_open(log_level, &src->headset);
	if (result != OXRSYS_WMR_OPEN_OK) {
		fprintf(stderr, "No usable headset: %s\n", oxrsys_wmr_open_result_str(result));
		return false;
	}

	const struct xrt_hmd_parts *parts = src->headset->hmd->hmd;
	if (parts == NULL || parts->view_count == 0) {
		fprintf(stderr, "Headset reports no HMD parts.\n");
		return false;
	}
	if (parts->distortion.mesh.vertices == NULL || parts->distortion.mesh.uv_channels_count != 3) {
		fprintf(stderr, "Headset has no 3-channel distortion mesh.\n");
		return false;
	}

	src->panel_w = (uint32_t)parts->screens[0].w_pixels;
	src->panel_h = (uint32_t)parts->screens[0].h_pixels;
	src->view_count = (uint32_t)parts->view_count;
	for (uint32_t i = 0; i < src->view_count; i++) {
		view_geometry &vg = src->views[i];
		vg.x_pixels = parts->views[i].viewport.x_pixels;
		vg.y_pixels = parts->views[i].viewport.y_pixels;
		vg.w_pixels = parts->views[i].viewport.w_pixels;
		vg.h_pixels = parts->views[i].viewport.h_pixels;
		vg.render_w = parts->views[i].display.w_pixels;
		vg.render_h = parts->views[i].display.h_pixels;
		vg.fov = parts->distortion.fov[i];
	}

	const auto &mesh = parts->distortion.mesh;
	src->mesh_stride_floats = mesh.stride / sizeof(float);
	src->mesh_vertices.assign(mesh.vertices, mesh.vertices + (size_t)mesh.vertex_count * src->mesh_stride_floats);
	src->mesh_indices.resize(mesh.index_count_total);
	for (uint32_t i = 0; i < mesh.index_count_total; i++) {
		src->mesh_indices[i] = (uint32_t)mesh.indices[i];
	}
	for (uint32_t i = 0; i < src->view_count; i++) {
		src->mesh_index_offsets[i] = mesh.index_offsets[i];
		src->mesh_index_counts[i] = mesh.index_counts[i];
	}

	printf("Headset: %s, panel %ux%u, %u views\n", oxrsys_wmr_headset_type_str(src->headset->type), src->panel_w,
	       src->panel_h, src->view_count);
	return true;
}

/*
 * Synthetic headset: two square eyes side by side, a mild radial distortion
 * with per-channel scale, and a head that slowly looks around.
 */
struct simulated_display_source : display_source
{
	int64_t start_ns = 0;

	bool get_eye_poses(int64_t at_timestamp_ns, struct xrt_pose *out_poses, uint32_t count) override
	{
		if (start_ns == 0) {
			start_ns = at_timestamp_ns;
		}
		const double t = (double)(at_timestamp_ns - start_ns) / 1e9;
		const float yaw = (float)(0.35 * sin(t * 0.5));
		const float pitch = (float)(0.15 * sin(t * 0.9));

		const struct xrt_quat q_yaw = {0.0f, sinf(yaw * 0.5f), 0.0f, cosf(yaw * 0.5f)};
		const struct xrt_quat q_pitch = {sinf(pitch * 0.5f), 0.0f, 0.0f, cosf(pitch * 0.5f)};
		const struct xrt_quat q = quat_mul(q_yaw, q_pitch);

		for (uint32_t i = 0; i < count; i++) {
			const struct xrt_vec3 offset = {i == 0 ? -0.0315f : 0.0315f, 0.0f, 0.0f};
			out_poses[i].orientation = q;
			out_poses[i].position = quat_rotate(q, offset);
		}
		return true;
	}
};

static void
simulated_distortion(uint32_t view, float u, float v, float *out_uv6)
{
	(void)view;
	const float k1[3] = {0.22f, 0.20f, 0.18f}; // Per-channel radial gain.
	const float px = u * 2.0f - 1.0f;
	const float py = v * 2.0f - 1.0f;
	const float r2 = px * px + py * py;
	for (int c = 0; c < 3; c++) {
		const float scale = 1.0f + k1[c] * r2;
		out_uv6[c * 2 + 0] = (px * scale) * 0.5f + 0.5f;
		out_uv6[c * 2 + 1] = (py * scale) * 0.5f + 0.5f;
	}
}

static void
simulated_display_source_init(simulated_display_source *src)
{
	src->panel_w = 2880;
	src->panel_h = 1600;
	src->view_count = 2;
	for (uint32_t i = 0; i < 2; i++) {
		view_geometry &vg = src->views[i];
		vg.x_pixels = i * 1440;
		vg.y_pixels = 0;
		vg.w_pixels = 1440;
		vg.h_pixels = 1600;
		vg.render_w = 1440;
		vg.render_h = 1600;
		vg.fov.angle_left = (float)(-48.0 * M_PI / 180.0);
		vg.fov.angle_right = (float)(48.0 * M_PI / 180.0);
		vg.fov.angle_up = (float)(50.0 * M_PI / 180.0);
		vg.fov.angle_down = (float)(-50.0 * M_PI / 180.0);
	}

	// Same layout Monado's u_distortion_mesh builds.
	const uint32_t cells = 32;
	const uint32_t vert_cols = cells + 1;
	const uint32_t vert_rows = cells + 1;
	const uint32_t stride = 8;
	src->mesh_stride_floats = stride;
	src->mesh_vertices.resize((size_t)vert_cols * vert_rows * src->view_count * stride);
	size_t i = 0;
	uint32_t vertex_offsets[XRT_MAX_VIEWS] = {};
	for (uint32_t view = 0; view < src->view_count; view++) {
		vertex_offsets[view] = (uint32_t)(i / stride);
		for (uint32_t r = 0; r < vert_rows; r++) {
			const float v = (float)r / (float)cells;
			for (uint32_t c = 0; c < vert_cols; c++) {
				const float u = (float)c / (float)cells;
				src->mesh_vertices[i + 0] = u * 2.0f - 1.0f;
				src->mesh_vertices[i + 1] = v * 2.0f - 1.0f;
				simulated_distortion(view, u, v, &src->mesh_vertices[i + 2]);
				i += stride;
			}
		}
	}
	const uint32_t index_count_per_view = cells * (vert_cols * 2 + 2);
	src->mesh_indices.resize((size_t)index_count_per_view * src->view_count);
	i = 0;
	for (uint32_t view = 0; view < src->view_count; view++) {
		src->mesh_index_offsets[view] = (uint32_t)i;
		src->mesh_index_counts[view] = index_count_per_view;
		const uint32_t off = vertex_offsets[view];
		auto index_for = [&](uint32_t r, uint32_t c) { return off + r * vert_cols + c; };
		for (uint32_t r = 0; r < cells; r++) {
			src->mesh_indices[i++] = index_for(r, 0);
			for (uint32_t c = 0; c < vert_cols; c++) {
				src->mesh_indices[i++] = index_for(r, c);
				src->mesh_indices[i++] = index_for(r + 1, c);
			}
			src->mesh_indices[i++] = index_for(r + 1, vert_cols - 1);
		}
	}
}

/*
 *
 * Display selection and capture.
 *
 */

static void
list_online_displays(void)
{
	CGDirectDisplayID ids[16];
	uint32_t count = 0;
	CGGetOnlineDisplayList(16, ids, &count);
	fprintf(stderr, "Online displays:\n");
	for (uint32_t i = 0; i < count; i++) {
		CGDisplayModeRef mode = CGDisplayCopyDisplayMode(ids[i]);
		fprintf(stderr, "  id %u: %zux%zu px @ %.1f Hz, vendor 0x%x model 0x%x%s%s\n", ids[i],
		        mode ? CGDisplayModeGetPixelWidth(mode) : 0, mode ? CGDisplayModeGetPixelHeight(mode) : 0,
		        mode ? CGDisplayModeGetRefreshRate(mode) : 0.0, CGDisplayVendorNumber(ids[i]),
		        CGDisplayModelNumber(ids[i]), CGDisplayIsMain(ids[i]) ? " (main)" : "",
		        CGDisplayIsInMirrorSet(ids[i]) ? " (mirrored)" : "");
		if (mode) {
			CGDisplayModeRelease(mode);
		}
	}
}

/*!
 * Best mode for the panel: exact pixel size, highest refresh rate.
 */
static CGDisplayModeRef
find_native_mode(CGDirectDisplayID display, uint32_t w, uint32_t h)
{
	const void *keys[] = {kCGDisplayShowDuplicateLowResolutionModes};
	const void *values[] = {kCFBooleanTrue};
	CFDictionaryRef opts = CFDictionaryCreate(NULL, keys, values, 1, &kCFTypeDictionaryKeyCallBacks,
	                                          &kCFTypeDictionaryValueCallBacks);
	CFArrayRef modes = CGDisplayCopyAllDisplayModes(display, opts);
	CFRelease(opts);
	if (modes == NULL) {
		return NULL;
	}

	CGDisplayModeRef best = NULL;
	double best_rate = -1.0;
	for (CFIndex i = 0; i < CFArrayGetCount(modes); i++) {
		CGDisplayModeRef mode = (CGDisplayModeRef)CFArrayGetValueAtIndex(modes, i);
		if (CGDisplayModeGetPixelWidth(mode) != w || CGDisplayModeGetPixelHeight(mode) != h) {
			continue;
		}
		const double rate = CGDisplayModeGetRefreshRate(mode);
		if (rate > best_rate) {
			best_rate = rate;
			best = mode;
		}
	}
	if (best != NULL) {
		CGDisplayModeRetain(best);
	}
	CFRelease(modes);
	return best;
}

/*!
 * Look for a display whose native pixel size matches the panel. Returns
 * kCGNullDirectDisplay if none is online yet.
 */
static CGDirectDisplayID
find_panel_display(uint32_t w, uint32_t h)
{
	CGDirectDisplayID ids[16];
	uint32_t count = 0;
	CGGetOnlineDisplayList(16, ids, &count);
	for (uint32_t i = 0; i < count; i++) {
		if (CGDisplayIsMain(ids[i]) || CGDisplayIsBuiltin(ids[i])) {
			continue;
		}
		CGDisplayModeRef mode = find_native_mode(ids[i], w, h);
		if (mode != NULL) {
			CGDisplayModeRelease(mode);
			return ids[i];
		}
	}
	return kCGNullDirectDisplay;
}

static bool
unmirror_display(CGDirectDisplayID display)
{
	if (!CGDisplayIsInMirrorSet(display)) {
		return true;
	}
	CGDisplayConfigRef config = NULL;
	if (CGBeginDisplayConfiguration(&config) != kCGErrorSuccess) {
		return false;
	}
	CGConfigureDisplayMirrorOfDisplay(config, display, kCGNullDirectDisplay);
	return CGCompleteDisplayConfiguration(config, kCGConfigureForSession) == kCGErrorSuccess;
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

/*
 *
 * Metal renderer.
 *
 */

static const char *kShaderSource = R"MSL(
#include <metal_stdlib>
using namespace metal;

struct EyeUniforms {
    float3x3 rotation;   // eye-local to world
    float3   position;   // eye position in world
    float4   tan_fov;    // tan(left), tan(right), tan(up), tan(down)
    float    time;
    uint     pattern;
};

struct FullscreenOut {
    float4 position [[position]];
    float2 uv;
};

static FullscreenOut fullscreen_triangle(uint vid) {
    // Oversized triangle; uv has v = 0 at the top row of the texture.
    float2 pos = float2((vid == 1) ? 3.0 : -1.0, (vid == 2) ? -3.0 : 1.0);
    FullscreenOut out;
    out.position = float4(pos, 0.0, 1.0);
    out.uv = float2((pos.x + 1.0) * 0.5, (1.0 - pos.y) * 0.5);
    return out;
}

vertex FullscreenOut fullscreen_vertex(uint vid [[vertex_id]]) {
    return fullscreen_triangle(vid);
}

static float grid_line(float2 p, float spacing, float width) {
    float2 g = abs(fract(p / spacing - 0.5) - 0.5) / fwidth(p / spacing);
    float line = min(g.x, g.y);
    return 1.0 - smoothstep(0.0, width, line);
}

// A room 6 m wide, floor at -1.6 m, ceiling at +1.4 m, with coloured walls
// so left/right/up/down are unambiguous when checking orientation.
static float3 shade_room(float3 origin, float3 dir) {
    const float half_w = 3.0;
    const float floor_y = -1.6;
    const float ceil_y = 1.4;
    float best_t = 1e9;
    float3 color = float3(0.02, 0.02, 0.03);

    // Planes: normal, offset, base colour.
    float3 normals[6] = { float3(0,1,0), float3(0,-1,0), float3(1,0,0), float3(-1,0,0), float3(0,0,1), float3(0,0,-1) };
    float offsets[6]  = { floor_y, ceil_y, -half_w, half_w, -half_w, half_w };
    float3 colors[6]  = { float3(0.35,0.35,0.38), float3(0.25,0.25,0.30),
                          float3(0.55,0.15,0.15), float3(0.15,0.55,0.15),
                          float3(0.15,0.15,0.60), float3(0.55,0.50,0.15) };
    for (int i = 0; i < 6; i++) {
        float denom = dot(normals[i], dir);
        if (abs(denom) < 1e-5) continue;
        float t = (offsets[i] - dot(normals[i], origin)) / denom;
        if (t <= 0.0 || t >= best_t) continue;
        float3 hit = origin + dir * t;
        // Grid coordinates on the plane.
        float2 p = (abs(normals[i].y) > 0.5) ? hit.xz : ((abs(normals[i].x) > 0.5) ? hit.zy : hit.xy);
        float line = grid_line(p, 0.5, 1.2);
        float fade = exp(-t * 0.12);
        float3 c = mix(colors[i], float3(0.95), line * 0.8) * fade;
        best_t = t;
        color = c;
    }
    // A bright marker straight ahead (-Z) at eye height, and a small sun.
    float ahead = dot(dir, float3(0, 0, -1));
    color += float3(1.0, 0.9, 0.5) * smoothstep(0.9985, 0.9995, ahead);
    return color;
}

static float3 shade_pattern(float2 uv, float4 tan_fov) {
    // Concentric rings and a crosshair in tangent space: rings should look
    // circular through the lens if the distortion mesh is right.
    float2 t = float2(mix(tan_fov.x, tan_fov.y, uv.x), mix(tan_fov.z, tan_fov.w, uv.y));
    float r = length(t);
    float rings = 1.0 - smoothstep(0.0, 0.02, abs(fract(r * 4.0) - 0.5) - 0.45);
    float cross = (abs(t.x) < 0.004 || abs(t.y) < 0.004) ? 1.0 : 0.0;
    float2 g = abs(fract(uv * 10.0) - 0.5);
    float grid = (min(g.x, g.y) < 0.02) ? 0.35 : 0.0;
    float3 c = float3(0.05) + float3(rings) * 0.6 + float3(grid) + float3(1.0, 0.2, 0.2) * cross;
    // Edge frame so the visible boundary of the eye image is obvious.
    if (uv.x < 0.01 || uv.x > 0.99 || uv.y < 0.01 || uv.y > 0.99) c = float3(0.2, 1.0, 0.2);
    return c;
}

fragment float4 eye_fragment(FullscreenOut in [[stage_in]], constant EyeUniforms &u [[buffer(0)]]) {
    if (u.pattern != 0) {
        return float4(shade_pattern(in.uv, u.tan_fov), 1.0);
    }
    float tx = mix(u.tan_fov.x, u.tan_fov.y, in.uv.x);
    float ty = mix(u.tan_fov.z, u.tan_fov.w, in.uv.y);
    float3 dir_eye = normalize(float3(tx, ty, -1.0));
    float3 dir = u.rotation * dir_eye;
    return float4(shade_room(u.position, dir), 1.0);
}

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

vertex FullscreenOut blit_vertex(uint vid [[vertex_id]]) {
    return fullscreen_triangle(vid);
}

fragment float4 blit_fragment(FullscreenOut in [[stage_in]], texture2d<float> tex [[texture(0)]]) {
    constexpr sampler s(address::clamp_to_edge, filter::linear);
    return tex.sample(s, in.uv);
}
)MSL";

struct EyeUniforms
{
	simd_float3x3 rotation;
	simd_float3 position;
	simd_float4 tan_fov;
	float time;
	uint32_t pattern;
};

@interface WmrRenderer : NSObject
@property(nonatomic, strong) id<MTLDevice> device;
@property(nonatomic, strong) id<MTLCommandQueue> queue;
@property(nonatomic, strong) id<MTLRenderPipelineState> eyePipeline;
@property(nonatomic, strong) id<MTLRenderPipelineState> meshPipeline;
@property(nonatomic, strong) id<MTLRenderPipelineState> blitPipeline;
@property(nonatomic, strong) id<MTLBuffer> meshVertexBuffer;
@property(nonatomic, strong) id<MTLBuffer> meshIndexBuffer;
@property(nonatomic, strong) NSArray<id<MTLTexture>> *eyeTextures;
@property(nonatomic, strong) id<MTLTexture> panelTexture;
@property(nonatomic, assign) display_source *source;
@property(nonatomic, assign) bool pattern;
@property(nonatomic, assign) bool distortion;
@property(nonatomic, assign) int64_t startNs;
@end

@implementation WmrRenderer

- (BOOL)setupWithSource:(display_source *)source pattern:(bool)pattern distortion:(bool)distortion
{
	self.source = source;
	self.pattern = pattern;
	self.distortion = distortion;
	self.startNs = os_monotonic_get_ns();

	self.device = MTLCreateSystemDefaultDevice();
	if (self.device == nil) {
		fprintf(stderr, "No Metal device.\n");
		return NO;
	}
	self.queue = [self.device newCommandQueue];

	NSError *error = nil;
	MTLCompileOptions *copts = [MTLCompileOptions new];
	id<MTLLibrary> library = [self.device newLibraryWithSource:@(kShaderSource) options:copts error:&error];
	if (library == nil) {
		fprintf(stderr, "Shader compile failed: %s\n", error.localizedDescription.UTF8String);
		return NO;
	}

	MTLRenderPipelineDescriptor *desc = [MTLRenderPipelineDescriptor new];
	desc.vertexFunction = [library newFunctionWithName:@"fullscreen_vertex"];
	desc.fragmentFunction = [library newFunctionWithName:@"eye_fragment"];
	desc.colorAttachments[0].pixelFormat = MTLPixelFormatBGRA8Unorm;
	self.eyePipeline = [self.device newRenderPipelineStateWithDescriptor:desc error:&error];
	if (self.eyePipeline == nil) {
		fprintf(stderr, "Eye pipeline failed: %s\n", error.localizedDescription.UTF8String);
		return NO;
	}

	MTLVertexDescriptor *vdesc = [MTLVertexDescriptor new];
	vdesc.attributes[0].format = MTLVertexFormatFloat4;
	vdesc.attributes[0].offset = 0;
	vdesc.attributes[0].bufferIndex = 0;
	vdesc.attributes[1].format = MTLVertexFormatFloat4;
	vdesc.attributes[1].offset = 16;
	vdesc.attributes[1].bufferIndex = 0;
	vdesc.layouts[0].stride = source->mesh_stride_floats * sizeof(float);
	vdesc.layouts[0].stepFunction = MTLVertexStepFunctionPerVertex;

	desc = [MTLRenderPipelineDescriptor new];
	desc.vertexFunction = [library newFunctionWithName:@"mesh_vertex"];
	desc.fragmentFunction = [library newFunctionWithName:@"mesh_fragment"];
	desc.vertexDescriptor = vdesc;
	desc.colorAttachments[0].pixelFormat = MTLPixelFormatBGRA8Unorm;
	self.meshPipeline = [self.device newRenderPipelineStateWithDescriptor:desc error:&error];
	if (self.meshPipeline == nil) {
		fprintf(stderr, "Mesh pipeline failed: %s\n", error.localizedDescription.UTF8String);
		return NO;
	}

	desc = [MTLRenderPipelineDescriptor new];
	desc.vertexFunction = [library newFunctionWithName:@"blit_vertex"];
	desc.fragmentFunction = [library newFunctionWithName:@"blit_fragment"];
	desc.colorAttachments[0].pixelFormat = MTLPixelFormatBGRA8Unorm;
	self.blitPipeline = [self.device newRenderPipelineStateWithDescriptor:desc error:&error];
	if (self.blitPipeline == nil) {
		fprintf(stderr, "Blit pipeline failed: %s\n", error.localizedDescription.UTF8String);
		return NO;
	}

	self.meshVertexBuffer = [self.device newBufferWithBytes:source->mesh_vertices.data()
	                                                 length:source->mesh_vertices.size() * sizeof(float)
	                                                options:MTLResourceStorageModeShared];
	self.meshIndexBuffer = [self.device newBufferWithBytes:source->mesh_indices.data()
	                                                length:source->mesh_indices.size() * sizeof(uint32_t)
	                                               options:MTLResourceStorageModeShared];

	NSMutableArray *eyes = [NSMutableArray array];
	for (uint32_t i = 0; i < source->view_count; i++) {
		MTLTextureDescriptor *tdesc =
		    [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatBGRA8Unorm
		                                                       width:source->views[i].render_w
		                                                      height:source->views[i].render_h
		                                                   mipmapped:NO];
		tdesc.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
		tdesc.storageMode = MTLStorageModePrivate;
		[eyes addObject:[self.device newTextureWithDescriptor:tdesc]];
	}
	self.eyeTextures = eyes;

	MTLTextureDescriptor *pdesc =
	    [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatBGRA8Unorm
	                                                       width:source->panel_w
	                                                      height:source->panel_h
	                                                   mipmapped:NO];
	pdesc.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
	pdesc.storageMode = MTLStorageModeShared; // CPU-readable for --screenshot.
	self.panelTexture = [self.device newTextureWithDescriptor:pdesc];
	return YES;
}

- (void)fillUniforms:(EyeUniforms *)u forView:(uint32_t)view pose:(const struct xrt_pose *)pose
{
	u->rotation = quat_to_simd_matrix(pose->orientation);
	u->position = simd_make_float3(pose->position.x, pose->position.y, pose->position.z);
	const struct xrt_fov &fov = self.source->views[view].fov;
	u->tan_fov = simd_make_float4(tanf(fov.angle_left), tanf(fov.angle_right), tanf(fov.angle_up),
	                              tanf(fov.angle_down));
	u->time = (float)((double)(os_monotonic_get_ns() - self.startNs) / 1e9);
	u->pattern = self.pattern ? 1 : 0;
}

/*!
 * Render one frame into the panel texture, then blit it to the drawable.
 */
- (void)renderToDrawable:(id<CAMetalDrawable>)drawable
{
	display_source *src = self.source;
	const int64_t now_ns = os_monotonic_get_ns();
	// Predict a little ahead: one frame of render plus half a scanout.
	const int64_t predict_ns = now_ns + 16000000;

	struct xrt_pose poses[XRT_MAX_VIEWS];
	for (uint32_t i = 0; i < XRT_MAX_VIEWS; i++) {
		math_pose_identity(&poses[i]);
	}
	src->get_eye_poses(predict_ns, poses, src->view_count);

	id<MTLCommandBuffer> cmd = [self.queue commandBuffer];

	// Pass 1: each eye's image.
	for (uint32_t i = 0; i < src->view_count; i++) {
		MTLRenderPassDescriptor *rp = [MTLRenderPassDescriptor renderPassDescriptor];
		rp.colorAttachments[0].texture = self.eyeTextures[i];
		rp.colorAttachments[0].loadAction = MTLLoadActionClear;
		rp.colorAttachments[0].storeAction = MTLStoreActionStore;
		rp.colorAttachments[0].clearColor = MTLClearColorMake(0, 0, 0, 1);
		id<MTLRenderCommandEncoder> enc = [cmd renderCommandEncoderWithDescriptor:rp];
		EyeUniforms u;
		[self fillUniforms:&u forView:i pose:&poses[i]];
		[enc setRenderPipelineState:self.eyePipeline];
		[enc setFragmentBytes:&u length:sizeof(u) atIndex:0];
		[enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
		[enc endEncoding];
	}

	// Pass 2: warp both eyes onto the panel texture.
	{
		MTLRenderPassDescriptor *rp = [MTLRenderPassDescriptor renderPassDescriptor];
		rp.colorAttachments[0].texture = self.panelTexture;
		rp.colorAttachments[0].loadAction = MTLLoadActionClear;
		rp.colorAttachments[0].storeAction = MTLStoreActionStore;
		rp.colorAttachments[0].clearColor = MTLClearColorMake(0, 0, 0, 1);
		id<MTLRenderCommandEncoder> enc = [cmd renderCommandEncoderWithDescriptor:rp];
		for (uint32_t i = 0; i < src->view_count; i++) {
			const view_geometry &vg = src->views[i];
			MTLViewport vp = {(double)vg.x_pixels, (double)vg.y_pixels, (double)vg.w_pixels,
			                  (double)vg.h_pixels, 0.0, 1.0};
			[enc setViewport:vp];
			MTLScissorRect sc = {vg.x_pixels, vg.y_pixels, vg.w_pixels, vg.h_pixels};
			[enc setScissorRect:sc];
			if (self.distortion) {
				[enc setRenderPipelineState:self.meshPipeline];
				[enc setVertexBuffer:self.meshVertexBuffer offset:0 atIndex:0];
				[enc setFragmentTexture:self.eyeTextures[i] atIndex:0];
				[enc drawIndexedPrimitives:MTLPrimitiveTypeTriangleStrip
				                indexCount:src->mesh_index_counts[i]
				                 indexType:MTLIndexTypeUInt32
				               indexBuffer:self.meshIndexBuffer
				         indexBufferOffset:src->mesh_index_offsets[i] * sizeof(uint32_t)];
			} else {
				[enc setRenderPipelineState:self.blitPipeline];
				[enc setFragmentTexture:self.eyeTextures[i] atIndex:0];
				[enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
			}
		}
		[enc endEncoding];
	}

	// Pass 3: panel texture to the drawable (scaled when windowed).
	{
		MTLRenderPassDescriptor *rp = [MTLRenderPassDescriptor renderPassDescriptor];
		rp.colorAttachments[0].texture = drawable.texture;
		rp.colorAttachments[0].loadAction = MTLLoadActionDontCare;
		rp.colorAttachments[0].storeAction = MTLStoreActionStore;
		id<MTLRenderCommandEncoder> enc = [cmd renderCommandEncoderWithDescriptor:rp];
		[enc setRenderPipelineState:self.blitPipeline];
		[enc setFragmentTexture:self.panelTexture atIndex:0];
		[enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
		[enc endEncoding];
	}

	[cmd presentDrawable:drawable];
	[cmd commit];
}

- (BOOL)writePanelPNG:(const char *)path
{
	// Drain the queue so the last frame's writes to the shared texture are done.
	id<MTLCommandBuffer> cmd = [self.queue commandBuffer];
	[cmd commit];
	[cmd waitUntilCompleted];

	const NSUInteger w = self.panelTexture.width;
	const NSUInteger h = self.panelTexture.height;
	std::vector<uint8_t> pixels(w * h * 4);
	[self.panelTexture getBytes:pixels.data() bytesPerRow:w * 4 fromRegion:MTLRegionMake2D(0, 0, w, h) mipmapLevel:0];

	CGColorSpaceRef cs = CGColorSpaceCreateDeviceRGB();
	const CGBitmapInfo bitmap_info = (CGBitmapInfo)kCGBitmapByteOrder32Little | (CGBitmapInfo)kCGImageAlphaNoneSkipFirst;
	CGContextRef ctx = CGBitmapContextCreate(pixels.data(), w, h, 8, w * 4, cs, bitmap_info);
	CGImageRef image = CGBitmapContextCreateImage(ctx);
	NSURL *url = [NSURL fileURLWithPath:@(path)];
	CGImageDestinationRef dest = CGImageDestinationCreateWithURL((__bridge CFURLRef)url, CFSTR("public.png"), 1, NULL);
	BOOL ok = NO;
	if (dest != NULL) {
		CGImageDestinationAddImage(dest, image, NULL);
		ok = CGImageDestinationFinalize(dest);
		CFRelease(dest);
	}
	CGImageRelease(image);
	CGContextRelease(ctx);
	CGColorSpaceRelease(cs);
	return ok;
}

@end

/*
 *
 * Application glue.
 *
 */

@interface WmrDisplayApp : NSObject <NSApplicationDelegate>
@property(nonatomic, strong) NSWindow *window;
@property(nonatomic, strong) CAMetalLayer *layer;
@property(nonatomic, strong) WmrRenderer *renderer;
@property(nonatomic, strong) CADisplayLink *displayLink;
@property(nonatomic, strong) NSTimer *watchdogTimer;
@property(nonatomic, strong) NSTimer *fallbackTimer;
@property(nonatomic, assign) bool fallbackTimerActive;
@property(nonatomic, assign) int displayLinkFrames;
@property(nonatomic, assign) options opts;
@property(nonatomic, assign) CGDirectDisplayID capturedDisplay;
@property(nonatomic, assign) int frames;
@property(nonatomic, assign) int64_t startNs;
@property(nonatomic, assign) int exitCode;
@end

static WmrDisplayApp *g_app = nil;

static void
on_signal(int sig)
{
	(void)sig;
	dispatch_async(dispatch_get_main_queue(), ^{
	  [NSApp terminate:nil];
	});
}

@implementation WmrDisplayApp

- (void)shutdown
{
	if (self.displayLink != nil) {
		[self.displayLink invalidate];
		self.displayLink = nil;
	}
	[self.watchdogTimer invalidate];
	self.watchdogTimer = nil;
	[self.fallbackTimer invalidate];
	self.fallbackTimer = nil;
	if (self.capturedDisplay != kCGNullDirectDisplay) {
		CGDisplayRelease(self.capturedDisplay);
		self.capturedDisplay = kCGNullDirectDisplay;
	}
	if (self.window != nil) {
		[self.window orderOut:nil];
		self.window = nil;
	}
	if (self.renderer != nil && self.renderer.source != NULL) {
		delete self.renderer.source;
		self.renderer.source = NULL;
	}
}

- (void)applicationWillTerminate:(NSNotification *)notification
{
	(void)notification;
	[self shutdown];
	printf("%d frames rendered.\n", self.frames);
	// -[NSApplication terminate:] always exits 0; report our own status.
	exit(self.exitCode);
}

- (void)renderFrame
{
	@autoreleasepool {
		id<CAMetalDrawable> drawable = [self.layer nextDrawable];
		if (drawable == nil) {
			return;
		}
		[self.renderer renderToDrawable:drawable];
		self.frames++;

		const options &opts = self.opts;
		if (opts.screenshot != NULL && self.frames == opts.screenshot_frame) {
			if ([self.renderer writePanelPNG:opts.screenshot]) {
				printf("Wrote %s\n", opts.screenshot);
			} else {
				fprintf(stderr, "Failed to write %s\n", opts.screenshot);
				self.exitCode = 1;
			}
			[NSApp terminate:nil];
		}
	}
}

- (void)onDisplayLink:(CADisplayLink *)link
{
	(void)link;
	self.displayLinkFrames++;
	[self renderFrame];
}

/*!
 * Runs a few times a second independently of the display link: enforces
 * --seconds, and if the display link never delivers callbacks (seen when the
 * process has no interactive window-server session) drives frames itself so
 * --screenshot still completes.
 */
- (void)onWatchdog:(NSTimer *)timer
{
	(void)timer;
	const options &opts = self.opts;
	const double elapsed = (double)(os_monotonic_get_ns() - self.startNs) / 1e9;

	if (self.displayLinkFrames == 0 && elapsed > 1.0) {
		if (!self.fallbackTimerActive) {
			fprintf(stderr, "Display link delivered no frames after 1 s; falling back to a timer.\n");
			self.fallbackTimerActive = true;
			self.fallbackTimer = [NSTimer scheduledTimerWithTimeInterval:1.0 / 60.0
			                                                     target:self
			                                                   selector:@selector(onFallbackTimer:)
			                                                   userInfo:nil
			                                                    repeats:YES];
		}
	}

	if (opts.seconds >= 0.0 && elapsed >= opts.seconds) {
		[NSApp terminate:nil];
	}
	// A screenshot run that produced nothing after a generous window is a failure.
	if (opts.screenshot != NULL && elapsed > 30.0) {
		fprintf(stderr, "Timed out waiting for %d frames (rendered %d).\n", opts.screenshot_frame, self.frames);
		self.exitCode = 1;
		[NSApp terminate:nil];
	}
}

- (void)onFallbackTimer:(NSTimer *)timer
{
	(void)timer;
	[self renderFrame];
}

- (BOOL)createWindowOnScreen:(NSScreen *)screen frame:(NSRect)frame fullscreen:(BOOL)fullscreen pixelSize:(CGSize)pixels
{
	NSWindowStyleMask style = fullscreen ? NSWindowStyleMaskBorderless : NSWindowStyleMaskTitled;
	self.window = [[NSWindow alloc] initWithContentRect:frame styleMask:style backing:NSBackingStoreBuffered defer:NO screen:screen];
	self.window.title = @"OXRSys WMR display";
	self.window.releasedWhenClosed = NO;
	if (fullscreen) {
		self.window.level = CGShieldingWindowLevel();
		self.window.hidesOnDeactivate = NO;
		self.window.collectionBehavior = NSWindowCollectionBehaviorCanJoinAllSpaces | NSWindowCollectionBehaviorStationary;
		[NSCursor hide];
	}

	NSView *view = [[NSView alloc] initWithFrame:NSMakeRect(0, 0, frame.size.width, frame.size.height)];
	view.wantsLayer = YES;
	self.layer = [CAMetalLayer layer];
	self.layer.device = self.renderer.device;
	self.layer.pixelFormat = MTLPixelFormatBGRA8Unorm;
	self.layer.framebufferOnly = YES;
	self.layer.contentsScale = screen.backingScaleFactor;
	self.layer.drawableSize = pixels;
	self.layer.displaySyncEnabled = YES;
	view.layer = self.layer;
	self.window.contentView = view;
	[self.window makeKeyAndOrderFront:nil];

	self.displayLink = [view displayLinkWithTarget:self selector:@selector(onDisplayLink:)];
	[self.displayLink addToRunLoop:[NSRunLoop mainRunLoop] forMode:NSRunLoopCommonModes];
	return YES;
}

- (BOOL)startSimulated
{
	auto *src = new simulated_display_source();
	simulated_display_source_init(src);
	self.renderer = [WmrRenderer new];
	if (![self.renderer setupWithSource:src pattern:self.opts.pattern distortion:self.opts.distortion]) {
		delete src;
		return NO;
	}
	NSScreen *screen = [NSScreen mainScreen];
	const CGFloat w = 1440, h = 800;
	NSRect frame = NSMakeRect(NSMidX(screen.visibleFrame) - w / 2, NSMidY(screen.visibleFrame) - h / 2, w, h);
	printf("Simulated headset: panel %ux%u shown in a %.0fx%.0f window\n", src->panel_w, src->panel_h, w, h);
	return [self createWindowOnScreen:screen
	                            frame:frame
	                       fullscreen:NO
	                        pixelSize:CGSizeMake(w * screen.backingScaleFactor, h * screen.backingScaleFactor)];
}

- (BOOL)startHeadset
{
	auto *src = new wmr_display_source();
	if (!wmr_display_source_init(src, self.opts.log_level)) {
		delete src;
		return NO;
	}
	self.renderer = [WmrRenderer new];
	if (![self.renderer setupWithSource:src pattern:self.opts.pattern distortion:self.opts.distortion]) {
		delete src;
		return NO;
	}

	// The panel was switched on by the driver; give macOS time to see it.
	CGDirectDisplayID display = self.opts.display_id;
	if (display == kCGNullDirectDisplay) {
		const int64_t deadline = os_monotonic_get_ns() + (int64_t)(self.opts.display_timeout_s * 1e9);
		printf("Waiting up to %.0f s for a %ux%u display to appear...\n", self.opts.display_timeout_s, src->panel_w,
		       src->panel_h);
		while (display == kCGNullDirectDisplay && os_monotonic_get_ns() < deadline) {
			display = find_panel_display(src->panel_w, src->panel_h);
			if (display == kCGNullDirectDisplay) {
				[[NSRunLoop mainRunLoop] runUntilDate:[NSDate dateWithTimeIntervalSinceNow:0.5]];
			}
		}
		if (display == kCGNullDirectDisplay) {
			fprintf(stderr, "No display with a %ux%u mode appeared. Pass --display-id to force one.\n",
			        src->panel_w, src->panel_h);
			list_online_displays();
			return NO;
		}
	}

	if (!unmirror_display(display)) {
		fprintf(stderr, "Could not take display %u out of its mirror set.\n", display);
	}

	CGDisplayModeRef mode = find_native_mode(display, src->panel_w, src->panel_h);
	if (mode == NULL) {
		fprintf(stderr, "Display %u has no %ux%u mode.\n", display, src->panel_w, src->panel_h);
		list_online_displays();
		return NO;
	}

	if (CGDisplayCapture(display) != kCGErrorSuccess) {
		fprintf(stderr, "CGDisplayCapture failed for display %u.\n", display);
		CGDisplayModeRelease(mode);
		return NO;
	}
	self.capturedDisplay = display;

	CGDisplayModeRef current = CGDisplayCopyDisplayMode(display);
	const bool needs_switch = current == NULL || !CFEqual(current, mode);
	if (current != NULL) {
		CGDisplayModeRelease(current);
	}
	if (needs_switch && CGDisplaySetDisplayMode(display, mode, NULL) != kCGErrorSuccess) {
		fprintf(stderr, "Could not switch display %u to %ux%u.\n", display, src->panel_w, src->panel_h);
	}
	printf("Display %u: %zux%zu @ %.1f Hz, captured\n", display, CGDisplayModeGetPixelWidth(mode),
	       CGDisplayModeGetPixelHeight(mode), CGDisplayModeGetRefreshRate(mode));
	CGDisplayModeRelease(mode);

	// Let the window server settle after a mode switch before we look up the screen.
	[[NSRunLoop mainRunLoop] runUntilDate:[NSDate dateWithTimeIntervalSinceNow:0.5]];

	NSScreen *screen = screen_for_display(display);
	NSRect frame;
	if (screen != nil) {
		frame = screen.frame;
	} else {
		// Fall back to CG bounds; Cocoa's y axis is flipped relative to CG.
		CGRect bounds = CGDisplayBounds(display);
		frame = NSMakeRect(bounds.origin.x, bounds.origin.y, bounds.size.width, bounds.size.height);
		screen = [NSScreen mainScreen];
	}
	return [self createWindowOnScreen:screen
	                            frame:frame
	                       fullscreen:YES
	                        pixelSize:CGSizeMake(src->panel_w, src->panel_h)];
}

- (void)applicationDidFinishLaunching:(NSNotification *)notification
{
	(void)notification;
	self.startNs = os_monotonic_get_ns();
	self.capturedDisplay = kCGNullDirectDisplay;
	BOOL ok = self.opts.simulate ? [self startSimulated] : [self startHeadset];
	if (!ok) {
		self.exitCode = 1;
		[NSApp terminate:nil];
		return;
	}
	self.watchdogTimer = [NSTimer scheduledTimerWithTimeInterval:0.25
	                                                     target:self
	                                                   selector:@selector(onWatchdog:)
	                                                   userInfo:nil
	                                                    repeats:YES];
	[NSApp activateIgnoringOtherApps:YES];
}

@end

int
main(int argc, char **argv)
{
	options opts;
	if (!parse_options(argc, argv, &opts)) {
		usage(argv[0]);
		return 2;
	}

	static const char *level_names[] = {"trace", "debug", "info", "warn", "error"};
	if (opts.log_level <= U_LOGGING_ERROR) {
		setenv("WMR_LOG", level_names[opts.log_level], 0);
	}

	@autoreleasepool {
		[NSApplication sharedApplication];
		[NSApp setActivationPolicy:NSApplicationActivationPolicyRegular];
		g_app = [WmrDisplayApp new];
		g_app.opts = opts;
		NSApp.delegate = g_app;
		signal(SIGINT, on_signal);
		signal(SIGTERM, on_signal);
		[NSApp run];
	}
	return g_app.exitCode;
}
