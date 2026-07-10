/* SPDX-License-Identifier: Apache-2.0 */

#define _POSIX_C_SOURCE 200809L

#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "fmi3FunctionTypes.h"
#include "native_sil_transport.h"

#define MAX_FMI_VALUES 128
#define MAX_NAME_LEN   128
#define INPUT_COUNT    9
#define ODOMETRY_COUNT 18

struct fmi_api {
	void *library;
	fmi3InstantiateCoSimulationTYPE *instantiate;
	fmi3EnterInitializationModeTYPE *enter_initialization;
	fmi3ExitInitializationModeTYPE *exit_initialization;
	fmi3TerminateTYPE *terminate;
	fmi3SetFloat64TYPE *set_float64;
	fmi3GetFloat64TYPE *get_float64;
	fmi3DoStepTYPE *do_step;
	fmi3FreeInstanceTYPE *free_instance;
};

struct runner_config {
	char token[256];
	fmi3ValueReference input_vrs[MAX_FMI_VALUES];
	size_t input_count;
	fmi3ValueReference trace_vrs[MAX_FMI_VALUES];
	char trace_names[MAX_FMI_VALUES][MAX_NAME_LEN];
	size_t trace_count;
	fmi3ValueReference odometry_vrs[MAX_FMI_VALUES];
	size_t odometry_count;
};

struct output_files {
	FILE *plant;
	FILE *odometry;
	FILE *pwm;
	FILE *attitude;
	FILE *metrics;
};

struct recorded_step {
	double time;
	double trace[MAX_FMI_VALUES];
	synapse_topic_ExternalOdometryData_t odometry;
	synapse_topic_PwmSignalOutputsData_t pwm;
	synapse_topic_AttitudeCommandData_t attitude;
	double odometry_wall;
	double response_wall;
	uint64_t odometry_sequence;
	uint64_t pwm_sequence;
	uint64_t attitude_sequence;
	bool forwarded;
};

struct direct_transport {
	struct cubs2_native_sil_shared *shared;
	uint32_t sequence;
	double response_timeout_s;
};

static double monotonic_seconds(void)
{
	struct timespec value;

	if (clock_gettime(CLOCK_MONOTONIC, &value) != 0) {
		return 0.0;
	}
	return (double)value.tv_sec + (double)value.tv_nsec * 1.0e-9;
}

static int pace_until(double wall_start, double simulation_time, double simulation_speed)
{
	const double deadline = wall_start + simulation_time / simulation_speed;
	struct timespec target = {
		.tv_sec = (time_t)deadline,
		.tv_nsec = (long)((deadline - floor(deadline)) * 1.0e9),
	};
	int rc;

	if (monotonic_seconds() >= deadline) {
		return 0;
	}
	do {
		rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &target, NULL);
	} while (rc == EINTR);
	if (rc != 0) {
		fprintf(stderr, "native SIL pacing failed: %s\n", strerror(rc));
		return -1;
	}
	return 0;
}

static char *trim(char *text)
{
	char *end;

	while (*text == ' ' || *text == '\t' || *text == '\r' || *text == '\n') {
		text++;
	}
	end = text + strlen(text);
	while (end > text &&
	       (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r' || end[-1] == '\n')) {
		*--end = '\0';
	}
	return text;
}

static int parse_vrs(char *text, fmi3ValueReference *values, size_t *count)
{
	char *save = NULL;
	char *item;
	size_t used = 0;

	for (item = strtok_r(text, ",", &save); item != NULL; item = strtok_r(NULL, ",", &save)) {
		char *end = NULL;
		unsigned long value;

		if (used >= MAX_FMI_VALUES) {
			return -1;
		}
		errno = 0;
		value = strtoul(trim(item), &end, 10);
		if (errno != 0 || end == item || *trim(end) != '\0' || value > UINT32_MAX) {
			return -1;
		}
		values[used++] = (fmi3ValueReference)value;
	}
	*count = used;
	return 0;
}

static int parse_names(char *text, char names[][MAX_NAME_LEN], size_t *count)
{
	char *save = NULL;
	char *item;
	size_t used = 0;

	for (item = strtok_r(text, ",", &save); item != NULL; item = strtok_r(NULL, ",", &save)) {
		item = trim(item);
		if (used >= MAX_FMI_VALUES || *item == '\0' || strlen(item) >= MAX_NAME_LEN) {
			return -1;
		}
		strcpy(names[used++], item);
	}
	*count = used;
	return 0;
}

static int load_config(const char *path, struct runner_config *config)
{
	FILE *file = fopen(path, "r");
	char line[16384];
	size_t name_count = 0;

	if (file == NULL) {
		fprintf(stderr, "cannot open runner config %s: %s\n", path, strerror(errno));
		return -1;
	}
	memset(config, 0, sizeof(*config));
	while (fgets(line, sizeof(line), file) != NULL) {
		char *separator = strchr(line, '=');
		char *key;
		char *value;

		if (separator == NULL) {
			continue;
		}
		*separator = '\0';
		key = trim(line);
		value = trim(separator + 1);
		if (strcmp(key, "token") == 0) {
			if (strlen(value) >= sizeof(config->token)) {
				fclose(file);
				return -1;
			}
			strcpy(config->token, value);
		} else if (strcmp(key, "input_vrs") == 0) {
			if (parse_vrs(value, config->input_vrs, &config->input_count) != 0) {
				fclose(file);
				return -1;
			}
		} else if (strcmp(key, "trace_vrs") == 0) {
			if (parse_vrs(value, config->trace_vrs, &config->trace_count) != 0) {
				fclose(file);
				return -1;
			}
		} else if (strcmp(key, "trace_names") == 0) {
			if (parse_names(value, config->trace_names, &name_count) != 0) {
				fclose(file);
				return -1;
			}
		} else if (strcmp(key, "odometry_vrs") == 0) {
			if (parse_vrs(value, config->odometry_vrs, &config->odometry_count) != 0) {
				fclose(file);
				return -1;
			}
		}
	}
	fclose(file);
	if (config->token[0] == '\0' || config->input_count != INPUT_COUNT ||
	    config->trace_count == 0 || name_count != config->trace_count ||
	    config->odometry_count != ODOMETRY_COUNT) {
		fprintf(stderr, "runner config has inconsistent FMI metadata\n");
		return -1;
	}
	return 0;
}

static int load_symbol(void *library, const char *name, void *target, size_t target_size)
{
	void *symbol = dlsym(library, name);

	if (symbol == NULL) {
		fprintf(stderr, "missing FMI symbol %s: %s\n", name, dlerror());
		return -1;
	}
	if (target_size != sizeof(symbol)) {
		fprintf(stderr, "FMI function-pointer size is unsupported on this host\n");
		return -1;
	}
	memcpy(target, &symbol, sizeof(symbol));
	return 0;
}

static int load_fmi_api(const char *path, struct fmi_api *api)
{
	memset(api, 0, sizeof(*api));
	api->library = dlopen(path, RTLD_NOW | RTLD_LOCAL);
	if (api->library == NULL) {
		fprintf(stderr, "cannot load FMI library %s: %s\n", path, dlerror());
		return -1;
	}
#define LOAD_FMI(field, symbol)                                                                    \
	do {                                                                                       \
		if (load_symbol(api->library, symbol, &api->field, sizeof(api->field)) != 0) {      \
			goto fail;                                                                   \
		}                                                                                  \
	} while (0)
	LOAD_FMI(instantiate, "fmi3InstantiateCoSimulation");
	LOAD_FMI(enter_initialization, "fmi3EnterInitializationMode");
	LOAD_FMI(exit_initialization, "fmi3ExitInitializationMode");
	LOAD_FMI(terminate, "fmi3Terminate");
	LOAD_FMI(set_float64, "fmi3SetFloat64");
	LOAD_FMI(get_float64, "fmi3GetFloat64");
	LOAD_FMI(do_step, "fmi3DoStep");
	LOAD_FMI(free_instance, "fmi3FreeInstance");
#undef LOAD_FMI
	return 0;

fail:
	dlclose(api->library);
	memset(api, 0, sizeof(*api));
	return -1;
}

static int close_outputs(struct output_files *outputs)
{
	FILE **files[] = {&outputs->plant, &outputs->odometry, &outputs->pwm, &outputs->attitude,
			  &outputs->metrics};
	int result = 0;

	for (size_t i = 0; i < sizeof(files) / sizeof(files[0]); i++) {
		if (*files[i] != NULL && fclose(*files[i]) != 0) {
			result = -1;
		}
		*files[i] = NULL;
	}
	return result;
}

static int open_outputs(char **argv, struct output_files *outputs)
{
	FILE **files[] = {&outputs->plant, &outputs->odometry, &outputs->pwm, &outputs->attitude,
			  &outputs->metrics};

	memset(outputs, 0, sizeof(*outputs));
	for (size_t i = 0; i < sizeof(files) / sizeof(files[0]); i++) {
		*files[i] = fopen(argv[7 + (int)i], "w");
		if (*files[i] == NULL) {
			fprintf(stderr, "cannot open output %s: %s\n", argv[7 + (int)i],
				strerror(errno));
			close_outputs(outputs);
			return -1;
		}
	}
	return 0;
}

static int parse_positive_double(const char *text, double *value)
{
	char *end = NULL;

	errno = 0;
	*value = strtod(text, &end);
	if (errno != 0 || end == text || *end != '\0' || !isfinite(*value) || *value <= 0.0) {
		return -1;
	}
	return 0;
}

static void write_headers(const struct runner_config *config, const struct output_files *outputs)
{
	fputs("time", outputs->plant);
	for (size_t i = 0; i < config->trace_count; i++) {
		fprintf(outputs->plant, ",%s", config->trace_names[i]);
	}
	fputc('\n', outputs->plant);
	fputs("sim_time_s,timestamp_us,x_m,y_m,z_m,qw,qx,qy,qz,vx_m_s,vy_m_s,vz_m_s,"
	      "roll_rate_rad_s,pitch_rate_rad_s,yaw_rate_rad_s,flags,status,source_"
	      "id,id,"
	      "tracking_valid,bridge_wall_s,bridge_seq\n",
	      outputs->odometry);
	fputs("sim_time_s,timestamp_us,active_mask,port", outputs->pwm);
	for (int i = 0; i < 16; i++) {
		fprintf(outputs->pwm, ",output%d_us", i);
	}
	fputs(",bridge_wall_s,bridge_seq,lockstep_timestamp_us,forwarded_to_plant\n", outputs->pwm);
	fputs("sim_time_s,timestamp_us,roll_cmd_rad,pitch_cmd_rad,yaw_cmd_rad,"
	      "rate_roll_cmd_rad_s,rate_pitch_cmd_rad_s,rate_yaw_cmd_rad_s,thrust_"
	      "cmd,type_mask,"
	      "bridge_wall_s,bridge_seq,lockstep_timestamp_us\n",
	      outputs->attitude);
}

static int fmi_get(const struct fmi_api *api, fmi3Instance instance,
		   const fmi3ValueReference *vrs, size_t count, double *values)
{
	if (api->get_float64(instance, vrs, count, values, count) != fmi3OK) {
		fprintf(stderr, "fmi3GetFloat64 failed\n");
		return -1;
	}
	for (size_t i = 0; i < count; i++) {
		if (!isfinite(values[i])) {
			fprintf(stderr, "FMI output %zu is not finite\n", i);
			return -1;
		}
	}
	return 0;
}

static int capture_plant_row(const struct fmi_api *api, fmi3Instance instance,
			     const struct runner_config *config, double *values)
{
	return fmi_get(api, instance, config->trace_vrs, config->trace_count, values);
}

static int make_odometry(const struct fmi_api *api, fmi3Instance instance,
			 const struct runner_config *config,
			 synapse_topic_ExternalOdometryData_t *odometry)
{
	double value[ODOMETRY_COUNT];

	if (fmi_get(api, instance, config->odometry_vrs, ODOMETRY_COUNT, value) != 0) {
		return -1;
	}
	memset(odometry, 0, sizeof(*odometry));
	odometry->timestamp_us = (uint64_t)llround(value[0]);
	odometry->position_enu_m =
		(synapse_types_Vec3f_t){(float)value[1], (float)value[2], (float)value[3]};
	odometry->attitude = (synapse_types_Quaternionf_t){(float)value[4], (float)value[5],
							   (float)value[6], (float)value[7]};
	odometry->linear_velocity_enu_m_s =
		(synapse_types_Vec3f_t){(float)value[8], (float)value[9], (float)value[10]};
	odometry->angular_velocity_flu_rad_s =
		(synapse_types_RateTriplet_t){(float)value[11], (float)value[12], (float)value[13]};
	odometry->flags = (uint8_t)llround(value[14]);
	odometry->status = (uint8_t)llround(value[15]);
	odometry->source_id = (uint8_t)llround(value[16]);
	odometry->id = (uint8_t)llround(value[17]);
	return 0;
}

static void write_odometry_row(FILE *file, const synapse_topic_ExternalOdometryData_t *value,
			       double wall, uint64_t sequence)
{
	const bool valid = (value->flags & UINT8_C(15)) == UINT8_C(15) &&
			   (value->flags & UINT8_C(64)) == 0 && value->status != UINT8_C(3);

	fprintf(file,
		"%.9f,%llu,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
		"%.9g,%.9g,%.9g,%u,%u,%u,%u,%d,%.9f,%llu\n",
		(double)value->timestamp_us / 1.0e6, (unsigned long long)value->timestamp_us,
		value->position_enu_m.x, value->position_enu_m.y, value->position_enu_m.z,
		value->attitude.w, value->attitude.x, value->attitude.y, value->attitude.z,
		value->linear_velocity_enu_m_s.x, value->linear_velocity_enu_m_s.y,
		value->linear_velocity_enu_m_s.z, value->angular_velocity_flu_rad_s.roll,
		value->angular_velocity_flu_rad_s.pitch, value->angular_velocity_flu_rad_s.yaw,
		value->flags, value->status, value->source_id, value->id, valid ? 1 : 0, wall,
		(unsigned long long)sequence);
}

static void quaternion_to_euler(const synapse_types_Quaternionf_t *q, double *roll, double *pitch,
				double *yaw)
{
	const double sinr = 2.0 * ((double)q->w * q->x + (double)q->y * q->z);
	const double cosr = 1.0 - 2.0 * ((double)q->x * q->x + (double)q->y * q->y);
	const double sinp = 2.0 * ((double)q->w * q->y - (double)q->z * q->x);
	const double siny = 2.0 * ((double)q->w * q->z + (double)q->x * q->y);
	const double cosy = 1.0 - 2.0 * ((double)q->y * q->y + (double)q->z * q->z);

	*roll = atan2(sinr, cosr);
	*pitch = asin(fmax(-1.0, fmin(1.0, sinp)));
	*yaw = atan2(siny, cosy);
}

static void write_attitude_row(FILE *file, const synapse_topic_AttitudeCommandData_t *value,
			       double sim_time, double wall, uint64_t sequence,
			       uint64_t lockstep_timestamp)
{
	double roll;
	double pitch;
	double yaw;

	quaternion_to_euler(&value->attitude, &roll, &pitch, &yaw);
	fprintf(file, "%.9f,%llu,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%u,%.9f,%llu,%llu\n", sim_time,
		(unsigned long long)value->timestamp_us, roll, pitch, yaw,
		value->body_rate_flu_rad_s.roll, value->body_rate_flu_rad_s.pitch,
		value->body_rate_flu_rad_s.yaw, value->thrust, value->type_mask, wall,
		(unsigned long long)sequence, (unsigned long long)lockstep_timestamp);
}

static int open_direct_transport(const char *path, struct direct_transport *transport)
{
	int fd = open(path, O_RDWR);
	struct stat status;
	void *mapping;

	if (fd < 0) {
		fprintf(stderr, "cannot open native SIL shared memory %s: %s\n", path,
			strerror(errno));
		return -1;
	}
	if (fstat(fd, &status) != 0 || status.st_size < (off_t)sizeof(*transport->shared)) {
		fprintf(stderr, "native SIL shared memory is smaller than the transport layout\n");
		close(fd);
		return -1;
	}
	mapping = mmap(NULL, sizeof(*transport->shared), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	close(fd);
	if (mapping == MAP_FAILED) {
		fprintf(stderr, "cannot map native SIL shared memory: %s\n", strerror(errno));
		return -1;
	}
	transport->shared = mapping;
	transport->sequence = 0;
	if (transport->shared->magic != CUBS2_NATIVE_SIL_MAGIC) {
		fprintf(stderr, "native SIL shared memory has invalid magic\n");
		munmap(mapping, sizeof(*transport->shared));
		transport->shared = NULL;
		return -1;
	}
	return 0;
}

static void write_pwm_row(FILE *file, const synapse_topic_PwmSignalOutputsData_t *value,
			  double sim_time, double wall, uint64_t sequence,
			  uint64_t lockstep_timestamp, bool forwarded);

static uint16_t pwm_output_at(const synapse_topic_PwmSignalOutputsData_t *value, size_t index)
{
	switch (index) {
	case 0:
		return value->output0_us;
	case 1:
		return value->output1_us;
	case 2:
		return value->output2_us;
	case 3:
		return value->output3_us;
	case 4:
		return value->output4_us;
	case 5:
		return value->output5_us;
	case 6:
		return value->output6_us;
	case 7:
		return value->output7_us;
	case 8:
		return value->output8_us;
	case 9:
		return value->output9_us;
	case 10:
		return value->output10_us;
	case 11:
		return value->output11_us;
	case 12:
		return value->output12_us;
	case 13:
		return value->output13_us;
	case 14:
		return value->output14_us;
	case 15:
		return value->output15_us;
	default:
		return 0;
	}
}

static int exchange_direct(struct direct_transport *transport,
			   const synapse_topic_ExternalOdometryData_t *odometry,
			   synapse_topic_PwmSignalOutputsData_t *pwm,
			   synapse_topic_AttitudeCommandData_t *attitude)
{
	const double deadline = monotonic_seconds() + transport->response_timeout_s;
	uint32_t spins = 0;

	transport->shared->odometry = *odometry;
	transport->sequence++;
	__atomic_store_n(&transport->shared->odometry_sequence, transport->sequence,
			 __ATOMIC_RELEASE);
	while (__atomic_load_n(&transport->shared->response_sequence, __ATOMIC_ACQUIRE) !=
	       transport->sequence) {
		spins++;
		if ((spins & 0x3ffU) == 0U && monotonic_seconds() >= deadline) {
			fprintf(stderr,
				"native SIL shared memory response timed out at sequence %u\n",
				transport->sequence);
			return -1;
		}
	}
	*pwm = transport->shared->pwm;
	*attitude = transport->shared->attitude;
	if (pwm->timestamp_us < odometry->timestamp_us) {
		fprintf(stderr, "native SIL shared memory returned stale PWM at %llu\n",
			(unsigned long long)odometry->timestamp_us);
		return -1;
	}
	return 0;
}

static void write_pwm_row(FILE *file, const synapse_topic_PwmSignalOutputsData_t *value,
			  double sim_time, double wall, uint64_t sequence,
			  uint64_t lockstep_timestamp, bool forwarded)
{
	fprintf(file, "%.9f,%llu,%u,%u", sim_time, (unsigned long long)value->timestamp_us,
		value->active_mask, value->port);
	for (int i = 0; i < 16; i++) {
		fprintf(file, ",%u", pwm_output_at(value, (size_t)i));
	}
	fprintf(file, ",%.9f,%llu,%llu,%d\n", wall, (unsigned long long)sequence,
		(unsigned long long)lockstep_timestamp, forwarded ? 1 : 0);
}

static void write_records(const struct runner_config *config, const struct output_files *outputs,
			  const struct recorded_step *records, size_t count)
{
	for (size_t row = 0; row < count; row++) {
		const struct recorded_step *record = &records[row];

		fprintf(outputs->plant, "%.17g", record->time);
		for (size_t i = 0; i < config->trace_count; i++) {
			fprintf(outputs->plant, ",%.17g", record->trace[i]);
		}
		fputc('\n', outputs->plant);
		write_odometry_row(outputs->odometry, &record->odometry, record->odometry_wall,
				   record->odometry_sequence);
		write_pwm_row(outputs->pwm, &record->pwm, record->time, record->response_wall,
			      record->pwm_sequence, record->odometry.timestamp_us,
			      record->forwarded);
		write_attitude_row(outputs->attitude, &record->attitude, record->time,
				   record->response_wall, record->attitude_sequence,
				   record->odometry.timestamp_us);
	}
}

static int run_loop(const struct fmi_api *api, fmi3Instance instance,
		    const struct runner_config *config, const struct output_files *outputs,
		    double t_end, double simulation_speed, struct direct_transport *direct)
{
	const double required_records = ceil(t_end / 0.02) + 1.0;
	const double wall_start = monotonic_seconds();
	struct recorded_step *records;
	size_t capacity;
	size_t count = 0;
	double time = 0.0;
	double step_wall = 0.0;
	double simulated = 0.0;
	uint64_t sequence = 0;

	if (required_records > (double)(SIZE_MAX / sizeof(*records))) {
		fprintf(stderr, "native SIL trace is too large\n");
		return -1;
	}
	capacity = (size_t)required_records;
	records = calloc(capacity, sizeof(*records));
	if (records == NULL) {
		fprintf(stderr, "cannot allocate native SIL trace\n");
		return -1;
	}
	while (true) {
		struct recorded_step *record;
		double inputs[INPUT_COUNT];
		const bool will_step = time < t_end - 1.0e-12;

		if (count >= capacity) {
			fprintf(stderr, "native SIL trace capacity exceeded\n");
			goto fail;
		}
		record = &records[count++];
		record->time = time;
		if (capture_plant_row(api, instance, config, record->trace) != 0 ||
		    make_odometry(api, instance, config, &record->odometry) != 0) {
			goto fail;
		}
		if (pace_until(wall_start, time, simulation_speed) != 0) {
			goto fail;
		}
		sequence++;
		record->odometry_sequence = sequence;
		record->odometry_wall = monotonic_seconds() - wall_start;
		if (exchange_direct(direct, &record->odometry, &record->pwm, &record->attitude) !=
		    0) {
			goto fail;
		}
		record->response_wall = monotonic_seconds() - wall_start;
		record->pwm_sequence = ++sequence;
		record->attitude_sequence = ++sequence;
		record->forwarded =
			will_step && record->pwm.timestamp_us >= record->odometry.timestamp_us;
		if (!will_step) {
			break;
		}
			for (size_t i = 0; i < INPUT_COUNT; i++) {
				inputs[i] = (double)pwm_output_at(&record->pwm, i);
		}
		if (api->set_float64(instance, config->input_vrs, INPUT_COUNT, inputs,
				     INPUT_COUNT) != fmi3OK) {
			fprintf(stderr, "fmi3SetFloat64 failed\n");
			goto fail;
		}
		{
			const double dt = fmin(0.02, t_end - time);
			const double start = monotonic_seconds();
			fmi3Boolean event_needed = fmi3False;
			fmi3Boolean terminate = fmi3False;
			fmi3Boolean early_return = fmi3False;
			double last_time = time;
			const fmi3Status status =
				api->do_step(instance, time, dt, fmi3True, &event_needed, &terminate,
					     &early_return, &last_time);

			step_wall += monotonic_seconds() - start;
			const double expected_time = time + dt;
			const double time_tolerance = 1.0e-12 * fmax(1.0, fabs(expected_time));

			if (status != fmi3OK || event_needed || terminate || early_return ||
			    !isfinite(last_time) || fabs(last_time - expected_time) > time_tolerance) {
				fprintf(stderr, "unsupported FMI step result at %.6f\n", time);
				goto fail;
			}
			time = last_time;
			simulated += dt;
		}
	}
	write_records(config, outputs, records, count);
	fprintf(outputs->metrics, "plant_step_wall_s=%.17g\nplant_simulated_s=%.17g\n", step_wall,
		simulated);
	free(records);
	return 0;

fail:
	free(records);
	return -1;
}
int main(int argc, char **argv)
{
	struct runner_config config;
	struct fmi_api api;
	struct output_files outputs;
	fmi3Instance instance = NULL;
	struct direct_transport direct = {0};
	bool initialized = false;
	double t_end;
	double timeout;
	double simulation_speed;
	int result = EXIT_FAILURE;

	if (argc != 12) {
		fprintf(stderr,
			"usage: %s FMU_SO CONFIG SHM T_END TIMEOUT SPEED PLANT_CSV "
			"ODOM_CSV PWM_CSV "
			"ATTITUDE_CSV METRICS\n",
			argv[0]);
		return EXIT_FAILURE;
	}
	memset(&api, 0, sizeof(api));
	memset(&outputs, 0, sizeof(outputs));
	if (parse_positive_double(argv[4], &t_end) != 0 ||
	    parse_positive_double(argv[5], &timeout) != 0 ||
	    parse_positive_double(argv[6], &simulation_speed) != 0 ||
	    load_config(argv[2], &config) != 0) {
		return EXIT_FAILURE;
	}
	if (load_fmi_api(argv[1], &api) != 0) {
		return EXIT_FAILURE;
	}
	if (open_outputs(argv, &outputs) != 0) {
		dlclose(api.library);
		return EXIT_FAILURE;
	}
	write_headers(&config, &outputs);
	instance = api.instantiate("rumoca-native-sil", config.token, NULL, fmi3False, fmi3False,
				   fmi3False, fmi3False, NULL, 0,
				   NULL, NULL, NULL);
	if (instance == NULL ||
	    api.enter_initialization(instance, fmi3True, 1.0e-6, 0.0, fmi3False, 0.0) !=
		    fmi3OK ||
	    api.exit_initialization(instance) != fmi3OK) {
		fprintf(stderr, "FMI initialization failed\n");
		goto cleanup_fmi;
	}
	initialized = true;
	if (strncmp(argv[3], "shm:", 4) != 0) {
		fprintf(stderr, "native SIL runner requires a shm: transport\n");
		goto cleanup_fmi;
	}
	if (open_direct_transport(argv[3] + 4, &direct) != 0) {
		goto cleanup_fmi;
	}
	direct.response_timeout_s = timeout;
	result = run_loop(&api, instance, &config, &outputs, t_end, simulation_speed, &direct) == 0
			 ? EXIT_SUCCESS
			 : EXIT_FAILURE;

cleanup_fmi:
	if (direct.shared != NULL) {
		__atomic_store_n(&direct.shared->terminate, 1U, __ATOMIC_RELEASE);
		__atomic_add_fetch(&direct.shared->odometry_sequence, 1U, __ATOMIC_RELEASE);
		munmap(direct.shared, sizeof(*direct.shared));
	}
	if (instance != NULL) {
		if (initialized) {
			(void)api.terminate(instance);
		}
		api.free_instance(instance);
	}
	dlclose(api.library);
	if (close_outputs(&outputs) != 0) {
		fprintf(stderr, "failed to flush native SIL output files\n");
		result = EXIT_FAILURE;
	}
	return result;
}
