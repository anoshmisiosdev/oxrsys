// Copyright 2023, Collabora, Ltd.
// Copyright 2026, OXRSys contributors.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  macOS build of Monado's VIT (visual-inertial tracking) plugin
 *         loader. Upstream's t_vit_loader.c only knows Linux and Android and
 *         stops the build with "Unknown platform" elsewhere; dlopen works the
 *         same on macOS, so this is that file with the guards dropped.
 */

#include "tracking/t_vit_loader.h"
#include "util/u_logging.h"
#include "vit/vit_interface.h"

#include <dlfcn.h>
#include <stdlib.h>

static bool
vit_get_proc(void *handle, const char *name, void *proc_ptr)
{
	(void)dlerror();
	void *proc = dlsym(handle, name);
	const char *err = dlerror();
	if (err != NULL || proc == NULL) {
		U_LOG_E("Failed to load symbol %s: %s", name, err != NULL ? err : "not found");
		return false;
	}
	*(void **)proc_ptr = proc;
	return true;
}

bool
t_vit_bundle_load(struct t_vit_bundle *vit, const char *path)
{
	vit->handle = dlopen(path, RTLD_LAZY | RTLD_LOCAL);
	if (vit->handle == NULL) {
		U_LOG_E("Failed to open VIT library: %s", dlerror());
		return false;
	}

#define GET_PROC(SYM)                                                                                                  \
	do {                                                                                                           \
		if (!vit_get_proc(vit->handle, "vit_" #SYM, &vit->SYM)) {                                              \
			dlclose(vit->handle);                                                                          \
			vit->handle = NULL;                                                                            \
			return false;                                                                                  \
		}                                                                                                      \
	} while (0)

	GET_PROC(api_get_version);
	vit->api_get_version(&vit->version.major, &vit->version.minor, &vit->version.patch);
	if (vit->version.major != VIT_HEADER_VERSION_MAJOR) {
		U_LOG_E("Incompatible VIT versions: expecting %u.%u.%u but got %u.%u.%u",             //
		        VIT_HEADER_VERSION_MAJOR, VIT_HEADER_VERSION_MINOR, VIT_HEADER_VERSION_PATCH, //
		        vit->version.major, vit->version.minor, vit->version.patch);
		dlclose(vit->handle);
		vit->handle = NULL;
		return false;
	}

	GET_PROC(tracker_create);
	GET_PROC(tracker_destroy);
	GET_PROC(tracker_has_image_format);
	GET_PROC(tracker_get_supported_extensions);
	GET_PROC(tracker_get_enabled_extensions);
	GET_PROC(tracker_enable_extension);
	GET_PROC(tracker_start);
	GET_PROC(tracker_stop);
	GET_PROC(tracker_reset);
	GET_PROC(tracker_is_running);
	GET_PROC(tracker_push_imu_sample);
	GET_PROC(tracker_push_img_sample);
	GET_PROC(tracker_add_imu_calibration);
	GET_PROC(tracker_add_camera_calibration);
	GET_PROC(tracker_pop_pose);
	GET_PROC(tracker_get_timing_titles);
	GET_PROC(pose_destroy);
	GET_PROC(pose_get_data);
	GET_PROC(pose_get_timing);
	GET_PROC(pose_get_features);
#undef GET_PROC

	return true;
}

void
t_vit_bundle_unload(struct t_vit_bundle *vit)
{
	if (vit->handle != NULL) {
		dlclose(vit->handle);
		vit->handle = NULL;
	}
}
