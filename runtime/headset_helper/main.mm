// SPDX-License-Identifier: MPL-2.0
//
// oxrsys-headset-helper: the process that owns a wired headset.
//
// It opens the headset through the Monado driver, takes over the panel's
// display (captured, shielding-level window, so no other window can appear on
// it), runs tracking, and shows a head-tracked lobby whenever no OpenXR
// session is submitting frames. The runtime, living inside whatever process
// the game runs in (including an x86_64 Wine process under Rosetta), connects
// over a Unix socket, receives tracking, shares its frame surfaces once over a
// Mach rendezvous, and then submits frames by slot number. See
// HeadsetHelperIpc.h for the protocol.
//
//   oxrsys-headset-helper [--socket PATH] [--display-id ID] [--no-capture]
//                         [--log-level trace|debug|info|warn|error]
//
// Native arm64 only: it links the Monado driver and Metal directly.

#define OXRSYS_ENC_IPC_WANT_MACH 1
#include "HeadsetHelperIpc.h"

#include "psmv_macos.h"
#include "wmr_macos.h"
#include "wmr_panel.h"
#include "wmr_psmv_tracking.h"

#include "os/os_time.h"
#include "xrt/xrt_device.h"

#include <oxrsys/protocol/Protocol.h>

#import <AppKit/AppKit.h>
#import <IOSurface/IOSurface.h>
#import <Metal/Metal.h>
#import <simd/simd.h>

#include <mach/mach.h>
#include <servers/bootstrap.h>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace oxrsys::headset_ipc;
using oxrsys::enc_ipc::PortMsg;
using oxrsys::enc_ipc::PortMsgRecv;
using oxrsys::enc_ipc::kMsgIdChildPort;
using oxrsys::enc_ipc::kMsgIdSurface;
using oxrsys::enc_ipc::Reader;

/*
 *
 * Logging: stderr plus a file next to the runtime's own log.
 *
 */

static FILE* g_logFile = nullptr;

static void
LogLine(const char* level, const char* fmt, ...)
{
	char buf[1024];
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	const auto now = std::chrono::system_clock::now();
	const time_t t = std::chrono::system_clock::to_time_t(now);
	struct tm tmv;
	localtime_r(&t, &tmv);
	char ts[32];
	strftime(ts, sizeof(ts), "%H:%M:%S", &tmv);
	fprintf(stderr, "[%s] [%s] HeadsetHelper: %s\n", ts, level, buf);
	if (g_logFile != nullptr) {
		fprintf(g_logFile, "[%s] [%s] %s\n", ts, level, buf);
		fflush(g_logFile);
	}
}
#define LOGI(...) LogLine("info", __VA_ARGS__)
#define LOGW(...) LogLine("warn", __VA_ARGS__)
#define LOGE(...) LogLine("error", __VA_ARGS__)

/*
 *
 * Small helpers.
 *
 */

static int64_t
SteadyNowNs()
{
	return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
	    .count();
}

static struct xrt_vec3
quat_rotate(const struct xrt_quat& q, const struct xrt_vec3& v)
{
	const float cx = q.y * v.z - q.z * v.y;
	const float cy = q.z * v.x - q.x * v.z;
	const float cz = q.x * v.y - q.y * v.x;
	const float ccx = q.y * cz - q.z * cy;
	const float ccy = q.z * cx - q.x * cz;
	const float ccz = q.x * cy - q.y * cx;
	return {v.x + 2.0f * (q.w * cx + ccx), v.y + 2.0f * (q.w * cy + ccy), v.z + 2.0f * (q.w * cz + ccz)};
}

static struct xrt_vec3
YawRotate(const struct xrt_quat& q, float x, float y, float z)
{
	const float yaw = atan2f(2.0f * (q.w * q.y + q.x * q.z), 1.0f - 2.0f * (q.x * q.x + q.y * q.y));
	const float c = cosf(yaw), s = sinf(yaw);
	return {c * x + s * z, y, -s * x + c * z};
}

static simd_float3x3
quat_to_simd_matrix(const struct xrt_quat& q)
{
	const struct xrt_vec3 cx = quat_rotate(q, {1.0f, 0.0f, 0.0f});
	const struct xrt_vec3 cy = quat_rotate(q, {0.0f, 1.0f, 0.0f});
	const struct xrt_vec3 cz = quat_rotate(q, {0.0f, 0.0f, 1.0f});
	return simd_matrix(simd_make_float3(cx.x, cx.y, cx.z), simd_make_float3(cy.x, cy.y, cy.z),
	                   simd_make_float3(cz.x, cz.y, cz.z));
}

static enum xrt_input_name
GripPoseInput(const struct xrt_device* xdev)
{
	for (size_t i = 0; i < xdev->input_count; i++) {
		if (XRT_GET_INPUT_TYPE(xdev->inputs[i].name) == XRT_INPUT_TYPE_POSE) {
			return xdev->inputs[i].name;
		}
	}
	return XRT_INPUT_GENERIC_HEAD_POSE;
}

/*
 * Map a controller's inputs onto the packet's Touch-style fields (same
 * mapping the in-process backend used).
 */
static void
MapControllerInputs(const struct xrt_device* xdev, bool left, oxr::protocol::TrackingPacket& packet)
{
	using namespace oxr::protocol;
	float& trigger = left ? packet.leftTrigger : packet.rightTrigger;
	float& grip = left ? packet.leftGrip : packet.rightGrip;
	float* stick = left ? packet.leftThumbstick : packet.rightThumbstick;
	const uint32_t primaryClick = left ? BUTTON_X : BUTTON_A;
	const uint32_t secondaryClick = left ? BUTTON_Y : BUTTON_B;
	const uint32_t stickClick = left ? BUTTON_LEFT_THUMBSTICK : BUTTON_RIGHT_THUMBSTICK;
	const uint32_t triggerClick = left ? BUTTON_LEFT_TRIGGER : BUTTON_RIGHT_TRIGGER;
	const uint32_t gripClick = left ? BUTTON_LEFT_GRIP : BUTTON_RIGHT_GRIP;

	for (size_t i = 0; i < xdev->input_count; i++) {
		const struct xrt_input& in = xdev->inputs[i];
		const bool b = in.value.boolean;
		switch (in.name) {
		case XRT_INPUT_WMR_TRIGGER_VALUE:
		case XRT_INPUT_ODYSSEY_CONTROLLER_TRIGGER_VALUE:
		case XRT_INPUT_G2_CONTROLLER_TRIGGER_VALUE:
		case XRT_INPUT_PSMV_TRIGGER_VALUE:
			trigger = in.value.vec1.x;
			if (trigger > 0.5f) packet.buttonState |= triggerClick;
			break;
		case XRT_INPUT_WMR_SQUEEZE_CLICK:
		case XRT_INPUT_ODYSSEY_CONTROLLER_SQUEEZE_CLICK:
		case XRT_INPUT_PSMV_MOVE_CLICK:
			if (b) { grip = 1.0f; packet.buttonState |= gripClick; }
			break;
		case XRT_INPUT_G2_CONTROLLER_SQUEEZE_VALUE:
			grip = in.value.vec1.x;
			if (grip > 0.5f) packet.buttonState |= gripClick;
			break;
		case XRT_INPUT_WMR_MENU_CLICK:
		case XRT_INPUT_ODYSSEY_CONTROLLER_MENU_CLICK:
		case XRT_INPUT_G2_CONTROLLER_MENU_CLICK:
		case XRT_INPUT_PSMV_START_CLICK:
			if (b) packet.buttonState |= BUTTON_MENU;
			break;
		case XRT_INPUT_WMR_THUMBSTICK:
		case XRT_INPUT_ODYSSEY_CONTROLLER_THUMBSTICK:
		case XRT_INPUT_G2_CONTROLLER_THUMBSTICK:
			stick[0] = in.value.vec2.x;
			stick[1] = in.value.vec2.y;
			break;
		case XRT_INPUT_WMR_THUMBSTICK_CLICK:
		case XRT_INPUT_ODYSSEY_CONTROLLER_THUMBSTICK_CLICK:
		case XRT_INPUT_G2_CONTROLLER_THUMBSTICK_CLICK:
			if (b) packet.buttonState |= stickClick;
			break;
		case XRT_INPUT_WMR_TRACKPAD_CLICK:
		case XRT_INPUT_ODYSSEY_CONTROLLER_TRACKPAD_CLICK:
		case XRT_INPUT_G2_CONTROLLER_A_CLICK:
		case XRT_INPUT_G2_CONTROLLER_X_CLICK:
		case XRT_INPUT_PSMV_CROSS_CLICK:
			if (b) packet.buttonState |= primaryClick;
			break;
		case XRT_INPUT_G2_CONTROLLER_B_CLICK:
		case XRT_INPUT_G2_CONTROLLER_Y_CLICK:
		case XRT_INPUT_PSMV_CIRCLE_CLICK:
			if (b) packet.buttonState |= secondaryClick;
			break;
		default: break;
		}
	}
}

/*
 *
 * Lobby shader: the room from the display tool, one fullscreen pass per eye.
 *
 */

static const char* kLobbyShader = R"MSL(
#include <metal_stdlib>
using namespace metal;

struct EyeUniforms {
    float3x3 rotation;
    float3   position;
    float4   tan_fov;
    float    time;
    uint     connected;
};

struct Out { float4 position [[position]]; float2 uv; };

vertex Out lobby_vertex(uint vid [[vertex_id]]) {
    float2 pos = float2((vid == 1) ? 3.0 : -1.0, (vid == 2) ? -3.0 : 1.0);
    Out o; o.position = float4(pos, 0.0, 1.0); o.uv = float2((pos.x + 1.0) * 0.5, (1.0 - pos.y) * 0.5); return o;
}

static float grid_line(float2 p, float spacing, float width) {
    float2 g = abs(fract(p / spacing - 0.5) - 0.5) / fwidth(p / spacing);
    return 1.0 - smoothstep(0.0, width, min(g.x, g.y));
}

static float3 shade_room(float3 origin, float3 dir, float time, bool connected) {
    const float half_w = 3.0, floor_y = -1.6, ceil_y = 1.4;
    float best_t = 1e9;
    float3 color = float3(0.02, 0.02, 0.03);
    float3 normals[6] = { float3(0,1,0), float3(0,-1,0), float3(1,0,0), float3(-1,0,0), float3(0,0,1), float3(0,0,-1) };
    float offsets[6]  = { floor_y, ceil_y, -half_w, half_w, -half_w, half_w };
    float3 base = connected ? float3(0.10, 0.22, 0.14) : float3(0.16, 0.16, 0.22);
    float3 colors[6]  = { float3(0.30,0.30,0.34), float3(0.20,0.20,0.26), base, base, base * 1.3, base };
    for (int i = 0; i < 6; i++) {
        float denom = dot(normals[i], dir);
        if (abs(denom) < 1e-5) continue;
        float t = (offsets[i] - dot(normals[i], origin)) / denom;
        if (t <= 0.0 || t >= best_t) continue;
        float3 hit = origin + dir * t;
        float2 p = (abs(normals[i].y) > 0.5) ? hit.xz : ((abs(normals[i].x) > 0.5) ? hit.zy : hit.xy);
        float line = grid_line(p, 0.5, 1.2);
        float fade = exp(-t * 0.12);
        color = mix(colors[i], float3(0.9), line * 0.7) * fade;
        best_t = t;
    }
    // A slowly pulsing marker straight ahead so a frozen frame is obvious.
    float ahead = dot(dir, float3(0, 0, -1));
    float pulse = 0.6 + 0.4 * sin(time * 2.0);
    color += float3(1.0, 0.85, 0.4) * pulse * smoothstep(0.998, 0.9995, ahead);
    return color;
}

fragment float4 lobby_fragment(Out in [[stage_in]], constant EyeUniforms &u [[buffer(0)]]) {
    float tx = mix(u.tan_fov.x, u.tan_fov.y, in.uv.x);
    float ty = mix(u.tan_fov.z, u.tan_fov.w, in.uv.y);
    float3 dir = u.rotation * normalize(float3(tx, ty, -1.0));
    return float4(shade_room(u.position, dir, u.time, u.connected != 0), 1.0);
}
)MSL";

struct EyeUniforms
{
	simd_float3x3 rotation;
	simd_float3 position;
	simd_float4 tan_fov;
	float time;
	uint32_t connected;
};

/*
 *
 * Socket I/O (blocking, framed).
 *
 */

static bool
WriteAll(int fd, const uint8_t* data, size_t len)
{
	while (len > 0) {
		ssize_t n = ::send(fd, data, len, MSG_NOSIGNAL);
		if (n <= 0) {
			if (n < 0 && errno == EINTR) continue;
			return false;
		}
		data += n;
		len -= (size_t)n;
	}
	return true;
}

static bool
ReadAll(int fd, uint8_t* data, size_t len)
{
	while (len > 0) {
		ssize_t n = ::recv(fd, data, len, 0);
		if (n <= 0) {
			if (n < 0 && errno == EINTR) continue;
			return false;
		}
		data += n;
		len -= (size_t)n;
	}
	return true;
}

static bool
RecvFramed(int fd, MsgType& type, std::vector<uint8_t>& payload)
{
	uint8_t hdr[oxrsys::enc_ipc::kHeaderBytes];
	if (!ReadAll(fd, hdr, sizeof(hdr))) return false;
	Reader r(hdr, sizeof(hdr));
	const uint32_t magic = r.U32();
	const uint16_t t = r.U16();
	const uint16_t version = r.U16();
	const uint32_t len = r.U32();
	if (magic != oxrsys::enc_ipc::kMagic || version != kProtocolVersion || len > oxrsys::enc_ipc::kMaxPayloadBytes) {
		LOGE("bad frame header (magic 0x%x version %u len %u)", magic, version, len);
		return false;
	}
	payload.resize(len);
	if (len > 0 && !ReadAll(fd, payload.data(), len)) return false;
	type = (MsgType)t;
	return true;
}

/*
 *
 * The helper.
 *
 */

struct Options
{
	std::string socketPath = DefaultSocketPath();
	CGDirectDisplayID displayId = 0;
	bool capture = true;
	enum u_logging_level logLevel = U_LOGGING_INFO;
};

struct ClientState
{
	int fd = -1;
	pid_t pid = 0;
	std::string rendezvous;
	std::mutex writeMutex;

	// Frame surfaces, indexed by slot.
	std::vector<id<MTLTexture>> slotTextures;
	std::vector<IOSurfaceRef> surfaces;

	// Latest submitted frame.
	std::mutex frameMutex;
	int latestSlot = -1;
	int64_t latestSubmitNs = 0;
	std::vector<uint32_t> superseded; // slots to release after the next present
};

struct Helper
{
	Options opts;

	// Headset.
	struct oxrsys_wmr_headset* headset = nullptr;
	oxrsys::WmrPanel* panel = nullptr;
	oxrsys::WmrPanelGeometry geometry;
	uint32_t refreshHz = 90;
	struct xrt_device* controllers[2] = {nullptr, nullptr};
	struct oxrsys_psmv_controller* psmv[4] = {};
	size_t psmvCount = 0;
	struct oxrsys_wmr_psmv_tracking* sphereTracking = nullptr;
	std::string controllerKind = "none";
	float eyeHeightM = 1.6f;

	// Metal.
	id<MTLDevice> device = nil;
	id<MTLCommandQueue> queue = nil;
	id<MTLRenderPipelineState> lobbyPipeline = nil;
	id<MTLTexture> lobbyEyes[2] = {nil, nil};
	int64_t startNs = 0;

	// Tracking state shared with the render thread.
	std::mutex poseMutex;
	oxr::protocol::TrackingPacket latestPacket = {};
	bool haveHead = false;

	// Client.
	std::mutex clientMutex;
	std::shared_ptr<ClientState> client;

	std::atomic<bool> running{true};
	std::thread trackingThread, serverThread, renderThread;
	int listenFd = -1;
	uint64_t presented = 0, lobbyFrames = 0;
};

static Helper g;

/*
 *
 * Client messaging.
 *
 */

static bool
SendToClient(const std::shared_ptr<ClientState>& c, MsgType type, const std::vector<uint8_t>& payload)
{
	if (!c || c->fd < 0) return false;
	const std::vector<uint8_t> bytes = Frame(type, payload);
	std::lock_guard<std::mutex> lock(c->writeMutex);
	return WriteAll(c->fd, bytes.data(), bytes.size());
}

static std::shared_ptr<ClientState>
CurrentClient()
{
	std::lock_guard<std::mutex> lock(g.clientMutex);
	return g.client;
}

static void
DropClient(const std::shared_ptr<ClientState>& c, const char* why)
{
	{
		std::lock_guard<std::mutex> lock(g.clientMutex);
		if (g.client != c) return;
		g.client.reset();
	}
	LOGI("client pid %d gone (%s); back to the lobby", (int)c->pid, why);
	if (c->fd >= 0) {
		::shutdown(c->fd, SHUT_RDWR);
		::close(c->fd);
		c->fd = -1;
	}
	// Textures may still be in flight on the GPU; keep them alive until the
	// queue drains, then let ARC free them.
	id<MTLCommandBuffer> cmd = [g.queue commandBuffer];
	std::shared_ptr<ClientState> keep = c;
	[cmd addCompletedHandler:^(id<MTLCommandBuffer> b) {
	  (void)b;
	  std::lock_guard<std::mutex> lock(keep->frameMutex);
	  keep->slotTextures.clear();
	  for (IOSurfaceRef s : keep->surfaces) {
		  if (s) CFRelease(s);
	  }
	  keep->surfaces.clear();
	}];
	[cmd commit];
}

static HeadsetInfo
MakeHeadsetInfo()
{
	HeadsetInfo info;
	info.panelW = g.geometry.panel_w;
	info.panelH = g.geometry.panel_h;
	info.eyeW = g.geometry.views[0].render_w;
	info.eyeH = g.geometry.views[0].render_h;
	for (int i = 0; i < 4; i++) info.fov[i] = g.geometry.views[0].fov[i];
	info.refreshHz = g.refreshHz;
	info.displayReady = g.panel != nullptr && g.panel->IsReady();
	info.name = std::string("Windows Mixed Reality ") + oxrsys_wmr_headset_type_str(g.headset->type);
	info.controllers = g.controllerKind;
	return info;
}

/*
 * Mach rendezvous: look up the runtime's per-connection name, hand it a send
 * right to a fresh receive port, and collect one IOSurface per slot.
 */
static SurfacesStatus
ReceiveSurfaces(const std::shared_ptr<ClientState>& c, uint32_t count)
{
	mach_port_t bootstrapPort = MACH_PORT_NULL;
	task_get_bootstrap_port(mach_task_self(), &bootstrapPort);
	mach_port_t runtimePort = MACH_PORT_NULL;
	kern_return_t kr = bootstrap_look_up(bootstrapPort, c->rendezvous.c_str(), &runtimePort);
	if (kr != KERN_SUCCESS || runtimePort == MACH_PORT_NULL) {
		LOGE("bootstrap_look_up('%s') failed: %d (%s)", c->rendezvous.c_str(), kr, bootstrap_strerror(kr));
		return SurfacesStatus::RendezvousFailed;
	}
	mach_port_t rx = MACH_PORT_NULL;
	kr = mach_port_allocate(mach_task_self(), MACH_PORT_RIGHT_RECEIVE, &rx);
	if (kr != KERN_SUCCESS) {
		mach_port_deallocate(mach_task_self(), runtimePort);
		return SurfacesStatus::RendezvousFailed;
	}
	PortMsg msg = {};
	msg.header.msgh_bits = MACH_MSGH_BITS(MACH_MSG_TYPE_COPY_SEND, 0) | MACH_MSGH_BITS_COMPLEX;
	msg.header.msgh_size = sizeof(msg);
	msg.header.msgh_remote_port = runtimePort;
	msg.header.msgh_id = kMsgIdChildPort;
	msg.body.msgh_descriptor_count = 1;
	msg.port.name = rx;
	msg.port.disposition = MACH_MSG_TYPE_MAKE_SEND;
	msg.port.type = MACH_MSG_PORT_DESCRIPTOR;
	kr = mach_msg(&msg.header, MACH_SEND_MSG, sizeof(msg), 0, MACH_PORT_NULL, MACH_MSG_TIMEOUT_NONE, MACH_PORT_NULL);
	mach_port_deallocate(mach_task_self(), runtimePort);
	if (kr != KERN_SUCCESS) {
		LOGE("mach_msg(send helper port) failed: %d", kr);
		mach_port_mod_refs(mach_task_self(), rx, MACH_PORT_RIGHT_RECEIVE, -1);
		return SurfacesStatus::RendezvousFailed;
	}

	std::vector<IOSurfaceRef> surfaces(count, nullptr);
	std::vector<id<MTLTexture>> textures(count, nil);
	uint32_t received = 0;
	for (uint32_t i = 0; i < count; i++) {
		PortMsgRecv rmsg = {};
		kr = mach_msg(&rmsg.msg.header, MACH_RCV_MSG | MACH_RCV_TIMEOUT, 0, sizeof(rmsg), rx, 5000, MACH_PORT_NULL);
		if (kr != KERN_SUCCESS) {
			LOGE("mach_msg(recv surface %u/%u) failed: %d", i, count, kr);
			break;
		}
		if (rmsg.msg.header.msgh_id != kMsgIdSurface || rmsg.msg.body.msgh_descriptor_count != 1) continue;
		const uint32_t slot = rmsg.msg.slot;
		mach_port_t surfPort = rmsg.msg.port.name;
		if (slot >= count) {
			mach_port_deallocate(mach_task_self(), surfPort);
			continue;
		}
		IOSurfaceRef surf = IOSurfaceLookupFromMachPort(surfPort);
		mach_port_deallocate(mach_task_self(), surfPort);
		if (surf == nullptr) {
			LOGE("IOSurfaceLookupFromMachPort(slot %u) returned null", slot);
			continue;
		}
		MTLTextureDescriptor* desc =
		    [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatBGRA8Unorm
		                                                       width:IOSurfaceGetWidth(surf)
		                                                      height:IOSurfaceGetHeight(surf)
		                                                   mipmapped:NO];
		desc.usage = MTLTextureUsageShaderRead;
		id<MTLTexture> tex = [g.device newTextureWithDescriptor:desc iosurface:surf plane:0];
		if (tex == nil) {
			LOGE("newTextureWithDescriptor:iosurface failed for slot %u", slot);
			CFRelease(surf);
			continue;
		}
		surfaces[slot] = surf;
		textures[slot] = tex;
		received++;
	}
	mach_port_mod_refs(mach_task_self(), rx, MACH_PORT_RIGHT_RECEIVE, -1);

	if (received != count) {
		for (IOSurfaceRef s : surfaces) {
			if (s) CFRelease(s);
		}
		return SurfacesStatus::TransferFailed;
	}
	{
		std::lock_guard<std::mutex> lock(c->frameMutex);
		c->surfaces = std::move(surfaces);
		c->slotTextures = std::move(textures);
		c->latestSlot = -1;
	}
	LOGI("received %u frame surfaces (%zux%zu) from pid %d", count, IOSurfaceGetWidth(c->surfaces[0]),
	     IOSurfaceGetHeight(c->surfaces[0]), (int)c->pid);
	return SurfacesStatus::Ok;
}

static void
ClientLoop(std::shared_ptr<ClientState> c)
{
	pthread_setname_np("oxrsys-helper-client");
	MsgType type;
	std::vector<uint8_t> payload;
	while (g.running.load() && RecvFramed(c->fd, type, payload)) {
		switch (type) {
		case MsgType::Hello: {
			Reader r(payload.data(), payload.size());
			const uint16_t version = r.U16();
			c->pid = (pid_t)r.U32();
			c->rendezvous = GetString(r);
			if (version != kProtocolVersion) {
				LOGE("client protocol %u, ours %u", version, kProtocolVersion);
				DropClient(c, "protocol mismatch");
				return;
			}
			LOGI("client pid %d connected (rendezvous '%s')", (int)c->pid, c->rendezvous.c_str());
			SendToClient(c, MsgType::HeadsetInfo, EncodeHeadsetInfo(MakeHeadsetInfo()));
			break;
		}
		case MsgType::Surfaces: {
			Reader r(payload.data(), payload.size());
			const uint32_t count = r.U32();
			SurfacesStatus status = SurfacesStatus::TransferFailed;
			if (count >= 1 && count <= kMaxSlots) {
				status = ReceiveSurfaces(c, count);
			}
			std::vector<uint8_t> p;
			oxrsys::enc_ipc::PutU32(p, (uint32_t)status);
			SendToClient(c, MsgType::SurfacesAck, p);
			break;
		}
		case MsgType::SubmitFrame: {
			SubmitFrame f;
			if (!DecodeSubmitFrame(payload, f)) break;
			std::lock_guard<std::mutex> lock(c->frameMutex);
			if (f.slot < c->slotTextures.size()) {
				if (c->latestSlot >= 0 && c->latestSlot != (int)f.slot) {
					c->superseded.push_back((uint32_t)c->latestSlot);
				}
				c->latestSlot = (int)f.slot;
				c->latestSubmitNs = SteadyNowNs();
			}
			break;
		}
		case MsgType::Bye: DropClient(c, "bye"); return;
		default: break;
		}
	}
	DropClient(c, "socket closed");
}

static void
ServerLoop()
{
	pthread_setname_np("oxrsys-helper-server");
	while (g.running.load()) {
		int fd = ::accept(g.listenFd, nullptr, nullptr);
		if (fd < 0) {
			if (errno == EINTR) continue;
			if (g.running.load()) LOGE("accept failed: %s", strerror(errno));
			break;
		}
		const int bufSize = 1 << 20;
		setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &bufSize, sizeof(bufSize));
		setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &bufSize, sizeof(bufSize));
		auto c = std::make_shared<ClientState>();
		c->fd = fd;
		std::shared_ptr<ClientState> old;
		{
			std::lock_guard<std::mutex> lock(g.clientMutex);
			old = g.client;
			g.client = c;
		}
		if (old) {
			LOGW("a new client replaced the previous one");
			if (old->fd >= 0) {
				::shutdown(old->fd, SHUT_RDWR);
			}
		}
		std::thread(ClientLoop, c).detach();
	}
}

/*
 *
 * Tracking.
 *
 */

static void
TrackingLoop()
{
	pthread_setname_np("oxrsys-helper-tracking");
	const int64_t periodNs = 1000000000LL / 250;
	const int64_t horizonNs = 2 * (1000000000LL / std::max<uint32_t>(g.refreshHz, 1));
	struct xrt_device* hmd = g.headset->hmd;

	while (g.running.load()) {
		const int64_t loopStart = SteadyNowNs();
		const int64_t monadoNow = os_monotonic_get_ns();

		struct xrt_space_relation rel = {};
		const xrt_result_t xret = xrt_device_get_tracked_pose(hmd, XRT_INPUT_GENERIC_HEAD_POSE, monadoNow + horizonNs, &rel);
		const bool valid = xret == XRT_SUCCESS && (rel.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0;

		oxr::protocol::TrackingPacket packet = {};
		packet.timestampNs = loopStart;
		packet.headPosition[1] = g.eyeHeightM;
		packet.headOrientation[3] = 1.0f;
		if (valid) {
			packet.headOrientation[0] = rel.pose.orientation.x;
			packet.headOrientation[1] = rel.pose.orientation.y;
			packet.headOrientation[2] = rel.pose.orientation.z;
			packet.headOrientation[3] = rel.pose.orientation.w;
		}
		if ((rel.relation_flags & XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT) != 0) {
			packet.headAngularVelocity[0] = rel.angular_velocity.x;
			packet.headAngularVelocity[1] = rel.angular_velocity.y;
			packet.headAngularVelocity[2] = rel.angular_velocity.z;
		}
		packet.ipd = 0.063f;
		for (int i = 0; i < 4; i++) packet.eyeFov[i] = g.geometry.views[0].fov[i];

		for (int h = 0; h < 2; h++) {
			struct xrt_device* ctrl = g.controllers[h];
			if (ctrl == nullptr) continue;
			const bool left = h == 0;
			xrt_device_update_inputs(ctrl);
			struct xrt_space_relation cr = {};
			const xrt_result_t r2 = xrt_device_get_tracked_pose(ctrl, GripPoseInput(ctrl), monadoNow + horizonNs, &cr);
			if (r2 != XRT_SUCCESS || (cr.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) == 0) continue;
			float* pos = left ? packet.leftControllerPos : packet.rightControllerPos;
			float* rot = left ? packet.leftControllerRot : packet.rightControllerRot;
			struct xrt_vec3 offset;
			if ((cr.relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) != 0) {
				offset = quat_rotate(rel.pose.orientation, cr.pose.position);
			} else {
				offset = YawRotate(rel.pose.orientation, left ? -0.18f : 0.18f, -0.35f, -0.40f);
			}
			pos[0] = packet.headPosition[0] + offset.x;
			pos[1] = packet.headPosition[1] + offset.y;
			pos[2] = packet.headPosition[2] + offset.z;
			rot[0] = cr.pose.orientation.x;
			rot[1] = cr.pose.orientation.y;
			rot[2] = cr.pose.orientation.z;
			rot[3] = cr.pose.orientation.w;
			packet.trackingFlags |= left ? oxr::protocol::TRACKING_FLAG_LEFT_CONTROLLER_ACTIVE
			                             : oxr::protocol::TRACKING_FLAG_RIGHT_CONTROLLER_ACTIVE;
			MapControllerInputs(ctrl, left, packet);
		}

		{
			std::lock_guard<std::mutex> lock(g.poseMutex);
			g.latestPacket = packet;
			g.haveHead = valid;
		}
		if (auto c = CurrentClient()) {
			std::vector<uint8_t> p((const uint8_t*)&packet, (const uint8_t*)&packet + sizeof(packet));
			if (!SendToClient(c, MsgType::Tracking, p)) {
				DropClient(c, "tracking send failed");
			}
		}

		const int64_t elapsed = SteadyNowNs() - loopStart;
		if (elapsed < periodNs) {
			std::this_thread::sleep_for(std::chrono::nanoseconds(periodNs - elapsed));
		}
	}
}

/*
 *
 * Rendering: a client's latest frame, or the lobby.
 *
 */

static void
RenderLoop()
{
	pthread_setname_np("oxrsys-helper-render");
	int64_t lastStatusNs = SteadyNowNs();
	bool wasClientFrame = false;

	while (g.running.load()) {
		if (!g.panel->IsReady()) {
			// Nothing to show on; hand every submitted slot straight back so
			// the client keeps running (tracking still works without a panel).
			if (auto c = CurrentClient()) {
				std::vector<uint32_t> release;
				{
					std::lock_guard<std::mutex> lock(c->frameMutex);
					release.swap(c->superseded);
					if (c->latestSlot >= 0) {
						release.push_back((uint32_t)c->latestSlot);
						c->latestSlot = -1;
					}
				}
				for (uint32_t s : release) {
					std::vector<uint8_t> p;
					oxrsys::enc_ipc::PutU32(p, s);
					SendToClient(c, MsgType::FrameReleased, p);
				}
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
			continue;
		}

		// A client frame is "live" for half a second after submission.
		std::shared_ptr<ClientState> c = CurrentClient();
		int slot = -1;
		std::vector<uint32_t> release;
		id<MTLTexture> slotTexture = nil;
		if (c) {
			std::lock_guard<std::mutex> lock(c->frameMutex);
			if (c->latestSlot >= 0 && SteadyNowNs() - c->latestSubmitNs < 500000000LL &&
			    (size_t)c->latestSlot < c->slotTextures.size()) {
				slot = c->latestSlot;
				slotTexture = c->slotTextures[(size_t)slot];
				release.swap(c->superseded);
			}
		}

		oxrsys::WmrPanelEyeInput eyes[2];
		if (slotTexture != nil) {
			// Side-by-side: left eye in the left half of the shared surface.
			for (int i = 0; i < 2; i++) {
				eyes[i].texture = slotTexture;
				eyes[i].uvOffset[0] = i == 0 ? 0.0f : 0.5f;
				eyes[i].uvOffset[1] = 0.0f;
				eyes[i].uvScale[0] = 0.5f;
				eyes[i].uvScale[1] = 1.0f;
			}
			if (!wasClientFrame) LOGI("showing frames from pid %d", (int)c->pid);
			wasClientFrame = true;
		} else {
			// Lobby: render the room per eye from the latest head pose.
			oxr::protocol::TrackingPacket packet;
			{
				std::lock_guard<std::mutex> lock(g.poseMutex);
				packet = g.latestPacket;
			}
			struct xrt_quat q = {packet.headOrientation[0], packet.headOrientation[1], packet.headOrientation[2],
			                     packet.headOrientation[3]};
			id<MTLCommandBuffer> cmd = [g.queue commandBuffer];
			for (int i = 0; i < 2; i++) {
				MTLRenderPassDescriptor* rp = [MTLRenderPassDescriptor renderPassDescriptor];
				rp.colorAttachments[0].texture = g.lobbyEyes[i];
				rp.colorAttachments[0].loadAction = MTLLoadActionClear;
				rp.colorAttachments[0].storeAction = MTLStoreActionStore;
				id<MTLRenderCommandEncoder> enc = [cmd renderCommandEncoderWithDescriptor:rp];
				EyeUniforms u;
				u.rotation = quat_to_simd_matrix(q);
				const struct xrt_vec3 eyeOffset = quat_rotate(q, {i == 0 ? -0.0315f : 0.0315f, 0.0f, 0.0f});
				u.position = simd_make_float3(eyeOffset.x, g.eyeHeightM + eyeOffset.y, eyeOffset.z);
				const float* fov = g.geometry.views[i].fov;
				u.tan_fov = simd_make_float4(tanf(fov[0]), tanf(fov[1]), tanf(fov[2]), tanf(fov[3]));
				u.time = (float)((double)(SteadyNowNs() - g.startNs) / 1e9);
				u.connected = c ? 1u : 0u;
				[enc setRenderPipelineState:g.lobbyPipeline];
				[enc setFragmentBytes:&u length:sizeof(u) atIndex:0];
				[enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
				[enc endEncoding];
				eyes[i].texture = g.lobbyEyes[i];
			}
			[cmd commit];
			if (wasClientFrame) LOGI("no frames for 0.5 s; showing the lobby");
			wasClientFrame = false;
			g.lobbyFrames++;
		}

		std::shared_ptr<ClientState> keep = c;
		const bool presented = g.panel->Present(g.queue, eyes[0], eyes[1], ^{
		  if (keep && !release.empty()) {
			  for (uint32_t s : release) {
				  std::vector<uint8_t> p;
				  oxrsys::enc_ipc::PutU32(p, s);
				  SendToClient(keep, MsgType::FrameReleased, p);
			  }
		  }
		});
		if (presented) {
			g.presented++;
		} else {
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
		}

		const int64_t now = SteadyNowNs();
		if (now - lastStatusNs >= 10000000000LL) {
			LOGI("%llu frames presented (%llu lobby), client %s", (unsigned long long)g.presented,
			     (unsigned long long)g.lobbyFrames, c ? "connected" : "none");
			lastStatusNs = now;
		}
	}
}

/*
 *
 * Setup.
 *
 */

static bool
OpenHeadset()
{
	std::vector<CGDirectDisplayID> before = oxrsys::WmrPanel::OnlineDisplays();
	const enum oxrsys_wmr_open_result result = oxrsys_wmr_headset_open(g.opts.logLevel, &g.headset);
	if (result != OXRSYS_WMR_OPEN_OK) {
		LOGE("no usable headset: %s", oxrsys_wmr_open_result_str(result));
		return false;
	}

	g.panel = new oxrsys::WmrPanel(g.headset->hmd->hmd);
	g.geometry = g.panel->Geometry();
	oxrsys::WmrPanelOptions po;
	po.display_id = g.opts.displayId;
	po.capture = g.opts.capture;
	po.displays_before = before;
	if (!g.panel->Open(po)) {
		LOGE("the panel did not appear as a display; see docs/platforms/wmr.md (EDID override)");
		// Keep running: tracking still serves the runtime.
	} else {
		const double hz = g.panel->RefreshRateHz();
		g.refreshHz = hz > 1.0 ? (uint32_t)lround(hz) : 90u;
	}

	// Controllers: the headset's own, else PS Moves (sphere-tracked with OpenCV).
	if (g.headset->left != nullptr || g.headset->right != nullptr) {
		g.controllers[0] = g.headset->left;
		g.controllers[1] = g.headset->right;
		g.controllerKind = g.headset->controllers_bluetooth ? "WMR controllers (Bluetooth)" : "WMR controllers (headset radio)";
	} else {
		struct xrt_tracking_factory* factory = oxrsys_wmr_psmv_tracking_create(g.headset->hmd, g.opts.logLevel, &g.sphereTracking);
		oxrsys_psmv_set_tracking_factory(factory);
		if (oxrsys_psmv_open_all(g.opts.logLevel, g.psmv, 4, &g.psmvCount) == OXRSYS_PSMV_OPEN_OK) {
			g.controllers[1] = g.psmv[0]->xdev;
			if (g.psmvCount > 1) g.controllers[0] = g.psmv[1]->xdev;
			g.controllerKind = "PlayStation Move";
		} else {
			oxrsys_psmv_set_tracking_factory(nullptr);
			oxrsys_wmr_psmv_tracking_destroy(&g.sphereTracking);
		}
	}
	LOGI("%s open: panel %ux%u @ %u Hz, eyes %ux%u, controllers %s", oxrsys_wmr_headset_type_str(g.headset->type),
	     g.geometry.panel_w, g.geometry.panel_h, g.refreshHz, g.geometry.views[0].render_w,
	     g.geometry.views[0].render_h, g.controllerKind.c_str());
	return true;
}

static bool
SetupMetal()
{
	g.device = MTLCreateSystemDefaultDevice();
	if (g.device == nil) return false;
	g.queue = [g.device newCommandQueue];
	g.queue.label = @"oxrsys-headset-helper";
	if (!g.panel->EnsureRenderer(g.device)) {
		LOGE("panel renderer failed");
		return false;
	}
	NSError* error = nil;
	id<MTLLibrary> lib = [g.device newLibraryWithSource:@(kLobbyShader) options:nil error:&error];
	if (lib == nil) {
		LOGE("lobby shader: %s", error.localizedDescription.UTF8String);
		return false;
	}
	MTLRenderPipelineDescriptor* desc = [MTLRenderPipelineDescriptor new];
	desc.vertexFunction = [lib newFunctionWithName:@"lobby_vertex"];
	desc.fragmentFunction = [lib newFunctionWithName:@"lobby_fragment"];
	desc.colorAttachments[0].pixelFormat = MTLPixelFormatBGRA8Unorm;
	g.lobbyPipeline = [g.device newRenderPipelineStateWithDescriptor:desc error:&error];
	if (g.lobbyPipeline == nil) {
		LOGE("lobby pipeline: %s", error.localizedDescription.UTF8String);
		return false;
	}
	for (int i = 0; i < 2; i++) {
		MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatBGRA8Unorm
		                                                                             width:g.geometry.views[i].render_w
		                                                                            height:g.geometry.views[i].render_h
		                                                                         mipmapped:NO];
		td.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
		td.storageMode = MTLStorageModePrivate;
		g.lobbyEyes[i] = [g.device newTextureWithDescriptor:td];
	}
	g.startNs = SteadyNowNs();
	return true;
}

static bool
SetupSocket()
{
	::unlink(g.opts.socketPath.c_str());
	g.listenFd = ::socket(AF_UNIX, SOCK_STREAM, 0);
	if (g.listenFd < 0) {
		LOGE("socket: %s", strerror(errno));
		return false;
	}
	struct sockaddr_un addr = {};
	addr.sun_family = AF_UNIX;
	snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", g.opts.socketPath.c_str());
	if (::bind(g.listenFd, (struct sockaddr*)&addr, sizeof(addr)) != 0 || ::listen(g.listenFd, 2) != 0) {
		LOGE("bind/listen %s: %s", g.opts.socketPath.c_str(), strerror(errno));
		return false;
	}
	LOGI("listening on %s", g.opts.socketPath.c_str());
	return true;
}

static void
Shutdown()
{
	if (!g.running.exchange(false)) return;
	LOGI("shutting down");
	if (g.listenFd >= 0) {
		::shutdown(g.listenFd, SHUT_RDWR);
		::close(g.listenFd);
		g.listenFd = -1;
	}
	::unlink(g.opts.socketPath.c_str());
	if (auto c = CurrentClient()) DropClient(c, "shutdown");
	if (g.trackingThread.joinable()) g.trackingThread.join();
	if (g.renderThread.joinable()) g.renderThread.join();
	if (g.serverThread.joinable()) g.serverThread.join();
	for (size_t i = 0; i < g.psmvCount; i++) oxrsys_psmv_close(&g.psmv[i]);
	oxrsys_psmv_set_tracking_factory(nullptr);
	oxrsys_wmr_psmv_tracking_destroy(&g.sphereTracking);
	if (g.panel) {
		g.panel->Close();
		delete g.panel;
		g.panel = nullptr;
	}
	if (g.headset) oxrsys_wmr_headset_close(&g.headset);
}

@interface HelperApp : NSObject <NSApplicationDelegate>
@end
@implementation HelperApp
- (void)applicationWillTerminate:(NSNotification*)n
{
	(void)n;
	Shutdown();
}
@end

static void
OnSignal(int sig)
{
	(void)sig;
	dispatch_async(dispatch_get_main_queue(), ^{
	  [NSApp terminate:nil];
	});
}

int
main(int argc, char** argv)
{
	signal(SIGPIPE, SIG_IGN);
	for (int i = 1; i < argc; i++) {
		const bool hasValue = i + 1 < argc;
		if (strcmp(argv[i], "--socket") == 0 && hasValue) {
			g.opts.socketPath = argv[++i];
		} else if (strcmp(argv[i], "--display-id") == 0 && hasValue) {
			g.opts.displayId = (CGDirectDisplayID)strtoul(argv[++i], nullptr, 0);
		} else if (strcmp(argv[i], "--no-capture") == 0) {
			g.opts.capture = false;
		} else if (strcmp(argv[i], "--log-level") == 0 && hasValue) {
			const char* l = argv[++i];
			g.opts.logLevel = strcmp(l, "trace") == 0 ? U_LOGGING_TRACE
			                  : strcmp(l, "debug") == 0 ? U_LOGGING_DEBUG
			                  : strcmp(l, "warn") == 0  ? U_LOGGING_WARN
			                  : strcmp(l, "error") == 0 ? U_LOGGING_ERROR
			                                            : U_LOGGING_INFO;
		} else {
			fprintf(stderr, "usage: %s [--socket PATH] [--display-id ID] [--no-capture] [--log-level LEVEL]\n", argv[0]);
			return 2;
		}
	}
	static const char* levelNames[] = {"trace", "debug", "info", "warn", "error"};
	if (g.opts.logLevel <= U_LOGGING_ERROR) {
		setenv("WMR_LOG", levelNames[g.opts.logLevel], 0);
		setenv("PSMV_LOG", levelNames[g.opts.logLevel], 0);
	}
	if (const char* home = getenv("HOME")) {
		std::string path = std::string(home) + "/Library/Application Support/OXRSys/oxrsys-headset-helper.log";
		g_logFile = fopen(path.c_str(), "a");
	}

	@autoreleasepool {
		[NSApplication sharedApplication];
		[NSApp setActivationPolicy:NSApplicationActivationPolicyAccessory];
		HelperApp* delegate = [HelperApp new];
		NSApp.delegate = delegate;

		if (!OpenHeadset() || !SetupMetal() || !SetupSocket()) {
			Shutdown();
			return 1;
		}
		g.trackingThread = std::thread(TrackingLoop);
		g.renderThread = std::thread(RenderLoop);
		g.serverThread = std::thread(ServerLoop);
		signal(SIGINT, OnSignal);
		signal(SIGTERM, OnSignal);
		[NSApp run];
	}
	return 0;
}
