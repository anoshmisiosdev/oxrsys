// SPDX-License-Identifier: MPL-2.0
/*!
 * @file
 * @brief The two util/u_file.h entry points Monado leaves out on macOS.
 *
 * Monado's u_file.cpp provides the config-directory basics for non-Linux
 * platforms, but the sub-path variant and the hand-tracking model lookup are
 * Linux-only. The WMR driver uses the sub-path variant to cache controller
 * calibration blobs, which are slow to read from the controller's flash.
 */

#include "util/u_file.h"

#include "xrt/xrt_config_os.h"

#ifdef XRT_OS_OSX

#include <stdio.h>
#include <sys/syslimits.h>

FILE *
u_file_open_file_in_config_dir_subpath(const char *subpath, const char *filename, const char *mode)
{
	char relative[PATH_MAX];
	int i = snprintf(relative, sizeof(relative), "%s/%s", subpath, filename);
	if (i < 0 || i >= (int)sizeof(relative)) {
		return NULL;
	}
	// Monado's macOS implementation creates missing parent directories.
	return u_file_open_file_in_config_dir(relative, mode);
}

int
u_file_get_hand_tracking_models_dir(char *out_path, size_t out_path_size)
{
	// Hand tracking is not built on this platform.
	if (out_path_size > 0) {
		out_path[0] = '\0';
	}
	return -1;
}

#endif /* XRT_OS_OSX */
