/* SPDX-License-Identifier: Apache-2.0 */

#include "native_sil_transport.h"

#include <stdbool.h>

#if defined(CONFIG_BOARD_NATIVE_SIM)

#include <nsi_host_trampolines.h>
#include <nsi_main.h>

#include <errno.h>
#include <zephyr/kernel.h>

void *cubs2_native_sil_host_map(const char *path, unsigned long size);
void cubs2_native_sil_host_unmap(void *mapping, unsigned long size);

static struct cubs2_native_sil_shared *g_native_sil_shared;
static uint32_t g_native_sil_sequence;
static bool g_native_sil_cooperative;

int cubs2_native_sil_transport_start(void)
{
	char *path = nsi_host_getenv("CUBS2_NATIVE_SIL_SHM");

	if (path == NULL || path[0] == '\0') {
		return 0;
	}
	g_native_sil_shared = cubs2_native_sil_host_map(path, sizeof(*g_native_sil_shared));
	if (g_native_sil_shared == NULL) {
		return -EIO;
	}
	if (g_native_sil_shared->magic != CUBS2_NATIVE_SIL_MAGIC) {
		cubs2_native_sil_host_unmap(g_native_sil_shared, sizeof(*g_native_sil_shared));
		g_native_sil_shared = NULL;
		return -EIO;
	}
	g_native_sil_cooperative = nsi_host_getenv("CUBS2_NATIVE_SIL_COOPERATIVE") != NULL;
	return 0;
}

bool cubs2_native_sil_transport_enabled(void)
{
	return g_native_sil_shared != NULL;
}

int cubs2_native_sil_transport_receive(synapse_topic_ExternalOdometryData_t *odometry)
{
	uint32_t sequence;

	do {
		sequence =
			__atomic_load_n(&g_native_sil_shared->odometry_sequence, __ATOMIC_ACQUIRE);
		if (sequence == g_native_sil_sequence && g_native_sil_cooperative) {
			k_sleep(K_USEC(100));
		}
	} while (sequence == g_native_sil_sequence);
	if (__atomic_load_n(&g_native_sil_shared->terminate, __ATOMIC_ACQUIRE) != 0U) {
		nsi_exit(0);
	}
	*odometry = g_native_sil_shared->odometry;
	g_native_sil_sequence = sequence;
	return 0;
}

int cubs2_native_sil_transport_send(const synapse_topic_PwmSignalOutputsData_t *pwm,
				    const synapse_topic_AttitudeCommandData_t *attitude)
{
	g_native_sil_shared->pwm = *pwm;
	g_native_sil_shared->attitude = *attitude;
	__atomic_store_n(&g_native_sil_shared->response_sequence, g_native_sil_sequence,
			 __ATOMIC_RELEASE);
	return 0;
}

#else

int cubs2_native_sil_transport_start(void)
{
	return 0;
}

bool cubs2_native_sil_transport_enabled(void)
{
	return false;
}

int cubs2_native_sil_transport_receive(synapse_topic_ExternalOdometryData_t *odometry)
{
	(void)odometry;
	return -1;
}

int cubs2_native_sil_transport_send(const synapse_topic_PwmSignalOutputsData_t *pwm,
				    const synapse_topic_AttitudeCommandData_t *attitude)
{
	(void)pwm;
	(void)attitude;
	return -1;
}

#endif
