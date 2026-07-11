/* SPDX-License-Identifier: Apache-2.0 */

#include "lockstep.h"

#include <errno.h>

#include <zephyr/sys/util.h>

#if defined(CONFIG_CUBS2_LOCKSTEP)
#include <cerebri_lockstep/sequence.h>
#endif

#if defined(CONFIG_BOARD_NATIVE_SIM)
#include <nsi_host_trampolines.h>
#include <nsi_main.h>

void *cubs2_lockstep_host_map(const char *path, unsigned long size);
void cubs2_lockstep_host_unmap(void *mapping, unsigned long size);

static struct cubs2_lockstep_shared *g_shared;
#elif defined(CONFIG_CUBS2_FASTDYN)
/* FastDyn resolves this global from the ELF and maps it through QEMU's
 * file-backed main RAM. No firmware address is duplicated on the host. */
struct cubs2_lockstep_shared cubs2_fastdyn_lockstep_shared;
static struct cubs2_lockstep_shared *g_shared = &cubs2_fastdyn_lockstep_shared;
#else
static struct cubs2_lockstep_shared *g_shared;
#endif

#if defined(CONFIG_CUBS2_LOCKSTEP)
static struct cerebri_lockstep_sequence g_lockstep;
#endif

int cubs2_lockstep_start(void)
{
#if defined(CONFIG_CUBS2_LOCKSTEP)
#if defined(CONFIG_BOARD_NATIVE_SIM)
	char *path = nsi_host_getenv("CUBS2_NATIVE_SIL_SHM");

	if (path == NULL || path[0] == '\0') {
		return 0;
	}
	g_shared = cubs2_lockstep_host_map(path, sizeof(*g_shared));
	if (g_shared == NULL || g_shared->magic != CUBS2_LOCKSTEP_MAGIC) {
		if (g_shared != NULL) {
			cubs2_lockstep_host_unmap(g_shared, sizeof(*g_shared));
			g_shared = NULL;
		}
		return -EIO;
	}
#elif defined(CONFIG_CUBS2_FASTDYN)
	*g_shared = (struct cubs2_lockstep_shared){
		.magic = CUBS2_LOCKSTEP_MAGIC,
	};
#endif
	return cerebri_lockstep_sequence_init(&g_lockstep, &g_shared->odometry_sequence,
					      &g_shared->response_sequence, &g_shared->terminate,
					      false);
#else
	return 0;
#endif
}
bool cubs2_lockstep_enabled(void)
{
	return g_shared != NULL;
}

int cubs2_lockstep_receive(synapse_topic_OdometryData_t *odometry)
{
#if defined(CONFIG_CUBS2_LOCKSTEP)
	int rc = cerebri_lockstep_sequence_wait(&g_lockstep);

	if (rc == -ECANCELED) {
#if defined(CONFIG_BOARD_NATIVE_SIM)
		nsi_exit(0);
#endif
		return rc;
	}
	if (rc != 0) {
		return rc;
	}
	*odometry = g_shared->odometry;
	return 0;
#else
	ARG_UNUSED(odometry);
	return -ENOTSUP;
#endif
}

int cubs2_lockstep_send(const synapse_topic_PwmSignalOutputsData_t *pwm,
			const synapse_topic_AttitudeCommandData_t *attitude)
{
#if defined(CONFIG_CUBS2_LOCKSTEP)
	g_shared->pwm = *pwm;
	g_shared->attitude = *attitude;
	cerebri_lockstep_sequence_respond(&g_lockstep);
	return 0;
#else
	ARG_UNUSED(pwm);
	ARG_UNUSED(attitude);
	return -ENOTSUP;
#endif
}
