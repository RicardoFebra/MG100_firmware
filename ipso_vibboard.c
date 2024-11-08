/*
 * Copyright (c) 2017 Linaro Limited
 * Copyright (c) 2018-2019 Foundries.io
 * Copyright (c) 2020 Laird Connectivity
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Generic sensor is based off temperature sensor.
 * It is a template used to create other sensors.
 */

#define LOG_MODULE_NAME net_ipso_vibboard
#define LOG_LEVEL CONFIG_LWM2M_LOG_LEVEL

#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <stdint.h>
#include <init.h>

#include "lwm2m_object.h"
#include "lwm2m_engine.h"
#include "lwm2m_resource_ids.h"

#define MAX_INSTANCE_COUNT CONFIG_LWM2M_IPSO_VIBBOARD_INSTANCE_COUNT

#define IPSO_OBJECT_ID 42790
#define DEVICE_ID_ID 1000
#define DEVICE_NAME_ID 1001
#define VIBBOARD_MODE_ID 1002
#define DEVICE_STATE_ID 1003
#define DEVICE_BATTERY_VOLTAGE_ID 1004
#define MACHINE_STATE_ID 3000
#define TD_ACC_SCALES_ID 4000
#define TD_ACC_X_AXIS_ID 4001
#define TD_ACC_Y_AXIS_ID 4002
#define TD_ACC_Z_AXIS_ID 4003
#define FD_ACC_SCALES_ID 5000
#define FD_ACC_DATA_PEAKS_FREQ_X_AXIS_ID 5001
#define FD_ACC_DATA_PEAKS_AMPS_X_AXIS_ID 5002
#define FD_ACC_DATA_PEAKS_FREQ_Y_AXIS_ID 5003
#define FD_ACC_DATA_PEAKS_AMPS_Y_AXIS_ID 5004
#define FD_ACC_DATA_PEAKS_FREQ_Z_AXIS_ID 5005
#define FD_ACC_DATA_PEAKS_AMPS_Z_AXIS_ID 5006

#define SENSOR_NAME "Vibboard"

#define UNIT_STR_MAX_SIZE 15

#define TD_ACC_SCALE_MAX_INSTANCE_COUNT 5
#define TD_ACC_X_AXIS_MAX_INSTANCE_COUNT TD_ACC_SCALE_MAX_INSTANCE_COUNT
#define TD_ACC_Y_AXIS_MAX_INSTANCE_COUNT TD_ACC_SCALE_MAX_INSTANCE_COUNT
#define TD_ACC_Z_AXIS_MAX_INSTANCE_COUNT TD_ACC_SCALE_MAX_INSTANCE_COUNT
#define FD_ACC_SCALE_MAX_INSTANCE_COUNT 2
#define FD_ACC_DATA_PEAKS_X_AXIS_MAX_INSTANCE_COUNT 4
#define FD_ACC_DATA_PEAKS_Y_AXIS_MAX_INSTANCE_COUNT 4
#define FD_ACC_DATA_PEAKS_Z_AXIS_MAX_INSTANCE_COUNT 4

/*
 * Calculate resource instances as follows:
 * start with NUMBER_OF_OBJ_FIELDS
 * subtract EXEC resources (1)
 */

/* resource state variables */
static int8_t device_id[MAX_INSTANCE_COUNT];
static char device_name[MAX_INSTANCE_COUNT][UNIT_STR_MAX_SIZE];
static int8_t vibboard_mode[MAX_INSTANCE_COUNT];
static int8_t device_state[MAX_INSTANCE_COUNT];
static uint16_t device_battery_voltage[MAX_INSTANCE_COUNT];
static int8_t machine_state[MAX_INSTANCE_COUNT];
static u8_t td_acc_scales[MAX_INSTANCE_COUNT][TD_ACC_SCALE_MAX_INSTANCE_COUNT];
static float td_acc_x_axis[MAX_INSTANCE_COUNT][TD_ACC_X_AXIS_MAX_INSTANCE_COUNT];
static float td_acc_y_axis[MAX_INSTANCE_COUNT][TD_ACC_Y_AXIS_MAX_INSTANCE_COUNT];
static float td_acc_z_axis[MAX_INSTANCE_COUNT][TD_ACC_Z_AXIS_MAX_INSTANCE_COUNT];
static u8_t fd_acc_scales[MAX_INSTANCE_COUNT][FD_ACC_SCALE_MAX_INSTANCE_COUNT];
static int32_t fd_acc_data_peaks_freq_x_axis[MAX_INSTANCE_COUNT][FD_ACC_DATA_PEAKS_X_AXIS_MAX_INSTANCE_COUNT];
static int32_t fd_acc_data_peaks_amps_x_axis[MAX_INSTANCE_COUNT][FD_ACC_DATA_PEAKS_X_AXIS_MAX_INSTANCE_COUNT];
static int32_t fd_acc_data_peaks_freq_y_axis[MAX_INSTANCE_COUNT][FD_ACC_DATA_PEAKS_Y_AXIS_MAX_INSTANCE_COUNT];
static int32_t fd_acc_data_peaks_amps_y_axis[MAX_INSTANCE_COUNT][FD_ACC_DATA_PEAKS_Y_AXIS_MAX_INSTANCE_COUNT];
static int32_t fd_acc_data_peaks_freq_z_axis[MAX_INSTANCE_COUNT][FD_ACC_DATA_PEAKS_Z_AXIS_MAX_INSTANCE_COUNT];
static int32_t fd_acc_data_peaks_amps_z_axis[MAX_INSTANCE_COUNT][FD_ACC_DATA_PEAKS_Z_AXIS_MAX_INSTANCE_COUNT];

static struct lwm2m_engine_obj sensor;
static struct lwm2m_engine_obj_field fields[] = {
    OBJ_FIELD_DATA(DEVICE_ID_ID, R, S8),
    OBJ_FIELD_DATA(DEVICE_NAME_ID, R, STRING),
	OBJ_FIELD_DATA(VIBBOARD_MODE_ID, R, S8),
    OBJ_FIELD_DATA(DEVICE_STATE_ID, R, S8),
	OBJ_FIELD_DATA(DEVICE_BATTERY_VOLTAGE_ID, R, U16),
    OBJ_FIELD_DATA(MACHINE_STATE_ID, R_OPT, S8),
	OBJ_FIELD_DATA(TD_ACC_SCALES_ID, R_OPT, U8),
	OBJ_FIELD_DATA(TD_ACC_X_AXIS_ID, R_OPT, FLOAT32),
	OBJ_FIELD_DATA(TD_ACC_Y_AXIS_ID, R_OPT, FLOAT32),
	OBJ_FIELD_DATA(TD_ACC_Z_AXIS_ID, R_OPT, FLOAT32),
	OBJ_FIELD_DATA(FD_ACC_SCALES_ID, R_OPT, U8),
	OBJ_FIELD_DATA(FD_ACC_DATA_PEAKS_FREQ_X_AXIS_ID, R_OPT, S32),
	OBJ_FIELD_DATA(FD_ACC_DATA_PEAKS_AMPS_X_AXIS_ID, R_OPT, S32),
	OBJ_FIELD_DATA(FD_ACC_DATA_PEAKS_FREQ_Y_AXIS_ID, R_OPT, S32),
	OBJ_FIELD_DATA(FD_ACC_DATA_PEAKS_AMPS_Y_AXIS_ID, R_OPT, S32),
	OBJ_FIELD_DATA(FD_ACC_DATA_PEAKS_AMPS_Z_AXIS_ID, R_OPT, S32),
	OBJ_FIELD_DATA(FD_ACC_DATA_PEAKS_FREQ_Z_AXIS_ID, R_OPT, S32),
#if ADD_TIMESTAMPS
	OBJ_FIELD_DATA(TIMESTAMP_RID, RW_OPT, TIME),
#endif
};

#define NUMBER_OF_OBJ_FIELDS ARRAY_SIZE(fields)
#ifdef CONFIG_LWM2M_IPSO_VIBBOARD_TIMESTAMP
#define ADD_TIMESTAMPS 1
#else
#define ADD_TIMESTAMPS 0
#endif

#define RESOURCE_INSTANCE_COUNT (NUMBER_OF_OBJ_FIELDS - ADD_TIMESTAMPS) + \
	+ TD_ACC_SCALE_MAX_INSTANCE_COUNT + \
	+ TD_ACC_X_AXIS_MAX_INSTANCE_COUNT + TD_ACC_Y_AXIS_MAX_INSTANCE_COUNT + TD_ACC_Z_AXIS_MAX_INSTANCE_COUNT + \
	+ FD_ACC_SCALE_MAX_INSTANCE_COUNT + \
	+ FD_ACC_DATA_PEAKS_X_AXIS_MAX_INSTANCE_COUNT + FD_ACC_DATA_PEAKS_X_AXIS_MAX_INSTANCE_COUNT + \
	+ FD_ACC_DATA_PEAKS_Y_AXIS_MAX_INSTANCE_COUNT + FD_ACC_DATA_PEAKS_Y_AXIS_MAX_INSTANCE_COUNT + \
	+ FD_ACC_DATA_PEAKS_Z_AXIS_MAX_INSTANCE_COUNT + FD_ACC_DATA_PEAKS_Z_AXIS_MAX_INSTANCE_COUNT 

static struct lwm2m_engine_obj_inst inst[MAX_INSTANCE_COUNT];
static struct lwm2m_engine_res res[MAX_INSTANCE_COUNT][NUMBER_OF_OBJ_FIELDS];
static struct lwm2m_engine_res_inst res_inst[MAX_INSTANCE_COUNT][RESOURCE_INSTANCE_COUNT];

static struct lwm2m_engine_obj_inst *vibboard_create(uint16_t obj_inst_id)
{
	int index, i = 0, j = 0;

	/* Check that there is no other instance with this ID */
	for (index = 0; index < MAX_INSTANCE_COUNT; index++) {
		if (inst[index].obj && inst[index].obj_inst_id == obj_inst_id) {
			LOG_ERR("Can not create instance - "
				"already existing: %u",
				obj_inst_id);
			return NULL;
		}
	}

	for (index = 0; index < MAX_INSTANCE_COUNT; index++) {
		if (!inst[index].obj) {
			break;
		}
	}

	if (index >= MAX_INSTANCE_COUNT) {
		LOG_ERR("Can not create instance - no more room: %u",
			obj_inst_id);
		return NULL;
	}

	/* Set default values */
    device_id[obj_inst_id] = -1;
	strncpy(device_name[obj_inst_id], "NotAssigned",UNIT_STR_MAX_SIZE);
	vibboard_mode[obj_inst_id] = -99;
    device_state[obj_inst_id] = -1;
	device_battery_voltage[obj_inst_id] = 0;
    machine_state[obj_inst_id] = -1;
    (void)memset(td_acc_scales[obj_inst_id], 0, sizeof(td_acc_scales[obj_inst_id][0])*ARRAY_SIZE(td_acc_scales[0]));
	(void)memset(td_acc_x_axis[obj_inst_id], 0, sizeof(td_acc_x_axis[obj_inst_id][0])*ARRAY_SIZE(td_acc_x_axis[0]));
	(void)memset(td_acc_y_axis[obj_inst_id], 0, sizeof(td_acc_y_axis[obj_inst_id][0])*ARRAY_SIZE(td_acc_y_axis[0]));
	(void)memset(td_acc_z_axis[obj_inst_id], 0, sizeof(td_acc_z_axis[obj_inst_id][0])*ARRAY_SIZE(td_acc_z_axis[0]));
	(void)memset(fd_acc_scales[obj_inst_id], 0, sizeof(fd_acc_scales[obj_inst_id][0])*ARRAY_SIZE(fd_acc_scales[0]));
	(void)memset(fd_acc_data_peaks_freq_x_axis[obj_inst_id], 0, sizeof(fd_acc_data_peaks_freq_x_axis[obj_inst_id][0])*ARRAY_SIZE(fd_acc_data_peaks_freq_x_axis[0]));
	(void)memset(fd_acc_data_peaks_amps_x_axis[obj_inst_id], 0, sizeof(fd_acc_data_peaks_amps_x_axis[obj_inst_id][0])*ARRAY_SIZE(fd_acc_data_peaks_amps_x_axis[0]));
	(void)memset(fd_acc_data_peaks_freq_y_axis[obj_inst_id], 0, sizeof(fd_acc_data_peaks_freq_y_axis[obj_inst_id][0])*ARRAY_SIZE(fd_acc_data_peaks_freq_y_axis[0]));
	(void)memset(fd_acc_data_peaks_amps_y_axis[obj_inst_id], 0, sizeof(fd_acc_data_peaks_amps_y_axis[obj_inst_id][0])*ARRAY_SIZE(fd_acc_data_peaks_amps_y_axis[0]));
	(void)memset(fd_acc_data_peaks_freq_z_axis[obj_inst_id], 0, sizeof(fd_acc_data_peaks_freq_z_axis[obj_inst_id][0])*ARRAY_SIZE(fd_acc_data_peaks_freq_z_axis[0]));
	(void)memset(fd_acc_data_peaks_amps_z_axis[obj_inst_id], 0, sizeof(fd_acc_data_peaks_amps_z_axis[obj_inst_id][0])*ARRAY_SIZE(fd_acc_data_peaks_amps_z_axis[0]));

	(void)memset(res[obj_inst_id], 0, sizeof(res[obj_inst_id][0]) * ARRAY_SIZE(res[obj_inst_id]));
	init_res_instance(res_inst[obj_inst_id], ARRAY_SIZE(res_inst[obj_inst_id]));

	/* initialize instance resource data */
	INIT_OBJ_RES_DATA(DEVICE_ID_ID, res[obj_inst_id], i, res_inst[obj_inst_id], j, &device_id[obj_inst_id], sizeof(device_id[0]));
	INIT_OBJ_RES_DATA(DEVICE_NAME_ID, res[obj_inst_id], i, res_inst[obj_inst_id], j, device_name[obj_inst_id], UNIT_STR_MAX_SIZE);
	INIT_OBJ_RES_DATA(VIBBOARD_MODE_ID, res[obj_inst_id], i, res_inst[obj_inst_id], j, &vibboard_mode[obj_inst_id], sizeof(vibboard_mode[0]));
	INIT_OBJ_RES_DATA(DEVICE_STATE_ID, res[obj_inst_id], i, res_inst[obj_inst_id], j, &device_state[obj_inst_id], sizeof(device_state[0]));
	INIT_OBJ_RES_DATA(DEVICE_BATTERY_VOLTAGE_ID, res[obj_inst_id], i, res_inst[obj_inst_id], j, &device_battery_voltage[obj_inst_id], sizeof(device_battery_voltage[0]));
	INIT_OBJ_RES_DATA(MACHINE_STATE_ID, res[obj_inst_id], i, res_inst[obj_inst_id], j, &machine_state[obj_inst_id], sizeof(machine_state[0]));
	INIT_OBJ_RES_MULTI_DATA(TD_ACC_SCALES_ID, res[obj_inst_id], i, res_inst[obj_inst_id], j, 
		TD_ACC_SCALE_MAX_INSTANCE_COUNT, false,
		td_acc_scales[obj_inst_id], sizeof(td_acc_scales[0]));
	INIT_OBJ_RES_MULTI_DATA(TD_ACC_X_AXIS_ID, res[obj_inst_id], i, res_inst[obj_inst_id], j, 
		TD_ACC_X_AXIS_MAX_INSTANCE_COUNT, false,
		td_acc_x_axis[obj_inst_id], sizeof(td_acc_x_axis[0])); 
	INIT_OBJ_RES_MULTI_DATA(TD_ACC_Y_AXIS_ID, res[obj_inst_id], i, res_inst[obj_inst_id], j, 
		TD_ACC_Y_AXIS_MAX_INSTANCE_COUNT, false,
		td_acc_y_axis[obj_inst_id], sizeof(td_acc_y_axis[0]));
	INIT_OBJ_RES_MULTI_DATA(TD_ACC_Z_AXIS_ID, res[obj_inst_id], i, res_inst[obj_inst_id], j, 
		TD_ACC_Z_AXIS_MAX_INSTANCE_COUNT, false,
		td_acc_z_axis[obj_inst_id], sizeof(td_acc_z_axis[0]));
	INIT_OBJ_RES_MULTI_DATA(FD_ACC_SCALES_ID, res[obj_inst_id], i, res_inst[obj_inst_id], j, 
		FD_ACC_SCALE_MAX_INSTANCE_COUNT, false,
		fd_acc_scales[obj_inst_id], sizeof(fd_acc_scales[0]));
	INIT_OBJ_RES_MULTI_DATA(FD_ACC_DATA_PEAKS_FREQ_X_AXIS_ID, res[obj_inst_id], i, res_inst[obj_inst_id], j, 
		FD_ACC_DATA_PEAKS_X_AXIS_MAX_INSTANCE_COUNT, false,
		fd_acc_data_peaks_freq_x_axis[obj_inst_id], sizeof(fd_acc_data_peaks_freq_x_axis[0]));
	INIT_OBJ_RES_MULTI_DATA(FD_ACC_DATA_PEAKS_AMPS_X_AXIS_ID, res[obj_inst_id], i, res_inst[obj_inst_id], j, 
		FD_ACC_DATA_PEAKS_X_AXIS_MAX_INSTANCE_COUNT, false,
		fd_acc_data_peaks_amps_x_axis[obj_inst_id], sizeof(fd_acc_data_peaks_amps_x_axis[0]));
	INIT_OBJ_RES_MULTI_DATA(FD_ACC_DATA_PEAKS_FREQ_Y_AXIS_ID, res[obj_inst_id], i, res_inst[obj_inst_id], j, 
		FD_ACC_DATA_PEAKS_Y_AXIS_MAX_INSTANCE_COUNT, false,
		fd_acc_data_peaks_freq_y_axis[obj_inst_id], sizeof(fd_acc_data_peaks_freq_y_axis[0]));
	INIT_OBJ_RES_MULTI_DATA(FD_ACC_DATA_PEAKS_AMPS_Y_AXIS_ID, res[obj_inst_id], i, res_inst[obj_inst_id], j, 
		FD_ACC_DATA_PEAKS_Y_AXIS_MAX_INSTANCE_COUNT, false,
		fd_acc_data_peaks_amps_y_axis[obj_inst_id], sizeof(fd_acc_data_peaks_amps_y_axis[0]));
	INIT_OBJ_RES_MULTI_DATA(FD_ACC_DATA_PEAKS_AMPS_Z_AXIS_ID, res[obj_inst_id], i, res_inst[obj_inst_id], j, 
		FD_ACC_DATA_PEAKS_Z_AXIS_MAX_INSTANCE_COUNT, false,
		fd_acc_data_peaks_amps_z_axis[obj_inst_id], sizeof(fd_acc_data_peaks_amps_z_axis[0]));
	INIT_OBJ_RES_MULTI_DATA(FD_ACC_DATA_PEAKS_FREQ_Z_AXIS_ID, res[obj_inst_id], i, res_inst[obj_inst_id], j,
		FD_ACC_DATA_PEAKS_Z_AXIS_MAX_INSTANCE_COUNT, false,
		fd_acc_data_peaks_freq_z_axis[obj_inst_id], sizeof(fd_acc_data_peaks_freq_z_axis[0]));
#if ADD_TIMESTAMPS
	INIT_OBJ_RES_OPTDATA(TIMESTAMP_RID, res[obj_inst_id], i, res_inst[obj_inst_id], j);
#endif

	inst[obj_inst_id].resources = res[obj_inst_id];
	inst[obj_inst_id].resource_count = i;
	LOG_DBG("Created IPSO %s Sensor instance: %d", SENSOR_NAME, obj_inst_id);

	return &inst[obj_inst_id];
}

static int ipso_vibboard_init(struct device *dev)
{
	sensor.obj_id = IPSO_OBJECT_ID;
	sensor.fields = fields;
	sensor.field_count = ARRAY_SIZE(fields);
	sensor.max_instance_count = MAX_INSTANCE_COUNT;
	sensor.create_cb = vibboard_create;
	lwm2m_register_obj(&sensor);

	return 0;
}

SYS_INIT(ipso_vibboard_init, APPLICATION,
	 CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
