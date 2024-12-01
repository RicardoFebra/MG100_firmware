/**
 * @file lcz_lwm2m_client.c
 * @brief
 *
 * Copyright (c) 2017 Linaro Limited
 * Copyright (c) 2017-2019 Foundries.io
 * Copyright (c) 2020 Laird Connectivity
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log.h>
#define LOG_LEVEL LOG_LEVEL_DBG
LOG_MODULE_REGISTER(lwm2m_client);

/******************************************************************************/
/* Includes                                                                   */
/******************************************************************************/
#include <zephyr.h>
#include <zephyr/types.h>
#include <drivers/gpio.h>
#include <net/lwm2m.h>
#include <string.h>
#include <stddef.h>

#include "lcz_dns.h"
#include "led_configuration.h"
#include "dis.h"
#include "qrtc.h"
#include "laird_power.h"
#include "ble_lwm2m_service.h"
#include "lte.h"
#include "lcz_lwm2m_client.h"

#include "stdio.h"

#define ENPOINT_UNIQUE_CODE "02"

/******************************************************************************/
/* Local Constant, Macro and Type Definitions                                 */
/******************************************************************************/
#if !defined(CONFIG_NET_IPV6) && !defined(CONFIG_NET_IPV4)
#error LwM2M requires either IPV6 or IPV4 support
#endif

#if defined(CONFIG_LWM2M_DTLS_SUPPORT)
#define TLS_TAG CONFIG_LWM2M_PSK_TAG
#endif /* CONFIG_LWM2M_DTLS_SUPPORT */

/******************************************************************************/
/* Local Data Definitions                                                     */
/******************************************************************************/
static uint8_t led_state;
static uint32_t lwm2m_time;
static struct lwm2m_ctx client;
static bool lwm2m_initialized;
static char server_addr[CONFIG_DNS_RESOLVER_ADDR_MAX_SIZE];

static u8_t bat_idx = LWM2M_DEVICE_PWR_SRC_TYPE_BAT_INT;
static int bat_mv = 3800;
static int bat_ma = 125;
static u8_t usb_idx = LWM2M_DEVICE_PWR_SRC_TYPE_USB;
static int usb_mv = 5000;
static int usb_ma = 900;
static u8_t bat_level = 95;
static u8_t bat_status = LWM2M_DEVICE_BATTERY_STATUS_CHARGING;

/******************************************************************************/
/* Local Function Prototypes                                                  */
/******************************************************************************/
static void lwm2m_client_init_internal(void);

static int device_reboot_cb(uint16_t obj_inst_id);
static int device_factory_default_cb(uint16_t obj_inst_id);
static int lwm2m_setup(const char *serial_number, const char *imei);
static void rd_client_event(struct lwm2m_ctx *client,
			    enum lwm2m_rd_client_event client_event);
static int led_on_off_cb(uint16_t obj_inst_id, uint16_t res_id,
			 uint16_t res_inst_id, uint8_t *data, uint16_t data_len,
			 bool last_block, size_t total_size);
static int resolve_server_address(void);
static void create_bl654_sensor_objects(void);
static void create_vibboard_objects(void);
static struct float32_value make_float32_value(float v);
static size_t lwm2m_str_size(const char *s);

/******************************************************************************/
/* Global Function Definitions                                                */
/******************************************************************************/
void lwm2m_client_init(void)
{
	lwm2m_client_init_internal();
}

int lwm2m_set_vibboard_data(struct vibboard_device *vibboard_devices)
{
	int result = 0;
	int aux_result = 0;
	LOG_DBG("RC before new messages: %d\n", result);
	LOG_DBG("LWM2M initialized: %d\n", lwm2m_initialized);

	printk("lwm2m_set_vibboard_data\n");
	//print_vibboard_table(vibboard_devices);

	if (lwm2m_initialized) {

		char lwm2m_obj_path[20];
		char lwm2m_path_object[6] = "42790";

		struct float32_value float32_aux_value;
		int8_t int_8_val;
		uint8_t uint_8_val;
		uint16_t uint_16_val;
		uint32_t uint_32_val;
		int32_t int32_t_val;

		/* Vibration data*/
		for (int i = 0; i < VIBBOARD_NR; i++) {
			// 42790/Vibboard_id/object
			printk("Vibboard %d\n", vibboard_devices[i].device_id);
			if (vibboard_devices[i].device_id != -1) {

				snprintk(lwm2m_obj_path, sizeof(lwm2m_obj_path), "%s/%d/1000", lwm2m_path_object, vibboard_devices[i].device_id);
				int_8_val = vibboard_devices[i].device_id;
				result += lwm2m_engine_set_s8(lwm2m_obj_path, int_8_val);

				snprintk(lwm2m_obj_path,sizeof(lwm2m_obj_path), "%s/%d/1001", lwm2m_path_object, vibboard_devices[i].device_id);
				char string_value[14];
				memcpy(string_value,vibboard_devices[i].device_name,sizeof(string_value));
				result += lwm2m_engine_set_string(lwm2m_obj_path, string_value);

				snprintk(lwm2m_obj_path, sizeof(lwm2m_obj_path), "%s/%d/1002", lwm2m_path_object, vibboard_devices[i].device_id);
				int_8_val = vibboard_devices[i].vibboard_mode;
				result += lwm2m_engine_set_s8(lwm2m_obj_path, int_8_val);

				snprintk(lwm2m_obj_path, sizeof(lwm2m_obj_path), "%s/%d/1003", lwm2m_path_object, vibboard_devices[i].device_id);
				int_8_val = vibboard_devices[i].device_state;
				result += lwm2m_engine_set_s8(lwm2m_obj_path, int_8_val);

				snprintk(lwm2m_obj_path, sizeof(lwm2m_obj_path), "%s/%d/1004", lwm2m_path_object, vibboard_devices[i].device_id);
				uint_16_val = vibboard_devices[i].device_battery_voltage;
				result += lwm2m_engine_set_u16(lwm2m_obj_path, uint_16_val);

				snprintk(lwm2m_obj_path, sizeof(lwm2m_obj_path), "%s/%d/3000", lwm2m_path_object, vibboard_devices[i].device_id);
				int_8_val = vibboard_devices[i].TD_machine_state;
				result += lwm2m_engine_set_s8(lwm2m_obj_path, int_8_val);

				/*-----------TD----------*/
				for (int j = 0; j < TD_NR_FEATURES; j++) {
					// snprintk(lwm2m_obj_path, sizeof(lwm2m_obj_path), "%s/%d/4000/%d", lwm2m_path_object, vibboard_devices[i].device_id, j);
					// uint_8_val = vibboard_devices[i].TD_acc_sp_data_scalars[j];
					// result += lwm2m_engine_set_u8(lwm2m_obj_path, uint_8_val);

					snprintk(lwm2m_obj_path, sizeof(lwm2m_obj_path), "%s/%d/4001/%d", lwm2m_path_object, vibboard_devices[i].device_id, j);
					float32_aux_value = make_float32_value(vibboard_devices[i].TD_acc_sp_data_x_axis[j]);
					result += lwm2m_engine_set_float32(lwm2m_obj_path, &float32_aux_value);

					snprintk(lwm2m_obj_path, sizeof(lwm2m_obj_path), "%s/%d/4002/%d", lwm2m_path_object, vibboard_devices[i].device_id, j);
					float32_aux_value = make_float32_value(vibboard_devices[i].TD_acc_sp_data_y_axis[j]);
					result += lwm2m_engine_set_float32(lwm2m_obj_path, &float32_aux_value);

					snprintk(lwm2m_obj_path, sizeof(lwm2m_obj_path), "%s/%d/4003/%d", lwm2m_path_object, vibboard_devices[i].device_id, j);
					float32_aux_value = make_float32_value(vibboard_devices[i].TD_acc_sp_data_z_axis[j]);
					result += lwm2m_engine_set_float32(lwm2m_obj_path, &float32_aux_value);
				}

				/*-----------FD----------*/
				// snprintk(lwm2m_obj_path, sizeof(lwm2m_obj_path), "%s/%d/5000/0", lwm2m_path_object, vibboard_devices[i].device_id);
				// uint_8_val = vibboard_devices[i].FD_acc_data_peaks_freq_scalar;
				// result += lwm2m_engine_set_u8(lwm2m_obj_path, uint_8_val);
				
				// snprintk(lwm2m_obj_path, sizeof(lwm2m_obj_path), "%s/%d/5000/1", lwm2m_path_object, vibboard_devices[i].device_id);
				// uint_8_val = vibboard_devices[i].FD_acc_data_peaks_amp_scalar;
				// result += lwm2m_engine_set_u8(lwm2m_obj_path, uint_8_val);

				for (int j = 0; j < FD_NR_PEAKS; j++) {
					snprintk(lwm2m_obj_path, sizeof(lwm2m_obj_path), "%s/%d/5001/%d", lwm2m_path_object, vibboard_devices[i].device_id, j);
					int32_t_val = vibboard_devices[i].FD_acc_data_peaks_freq_x_axis[j];
					result += lwm2m_engine_set_s32(lwm2m_obj_path, int32_t_val);

					snprintk(lwm2m_obj_path, sizeof(lwm2m_obj_path), "%s/%d/5002/%d", lwm2m_path_object, vibboard_devices[i].device_id, j);
					int32_t_val = vibboard_devices[i].FD_acc_data_peaks_amps_x_axis[j];
					result += lwm2m_engine_set_s32(lwm2m_obj_path, int32_t_val);

					snprintk(lwm2m_obj_path, sizeof(lwm2m_obj_path), "%s/%d/5003/%d", lwm2m_path_object, vibboard_devices[i].device_id, j);
					int32_t_val = vibboard_devices[i].FD_acc_data_peaks_freq_y_axis[j];
					result += lwm2m_engine_set_s32(lwm2m_obj_path, int32_t_val);

					snprintk(lwm2m_obj_path, sizeof(lwm2m_obj_path), "%s/%d/5004/%d", lwm2m_path_object, vibboard_devices[i].device_id, j);
					int32_t_val = vibboard_devices[i].FD_acc_data_peaks_amps_y_axis[j];
					result += lwm2m_engine_set_s32(lwm2m_obj_path, int32_t_val);

					snprintk(lwm2m_obj_path, sizeof(lwm2m_obj_path), "%s/%d/5005/%d", lwm2m_path_object, vibboard_devices[i].device_id, j);
					int32_t_val = vibboard_devices[i].FD_acc_data_peaks_freq_z_axis[j];
					result += lwm2m_engine_set_s32(lwm2m_obj_path, int32_t_val);

					snprintk(lwm2m_obj_path, sizeof(lwm2m_obj_path), "%s/%d/5006/%d", lwm2m_path_object, vibboard_devices[i].device_id, j);
					int32_t_val = vibboard_devices[i].FD_acc_data_peaks_amps_z_axis[j];
					aux_result += lwm2m_engine_set_s32(lwm2m_obj_path, int32_t_val);
				}

				snprintk(lwm2m_obj_path, sizeof(lwm2m_obj_path), "%s/%d/8000", lwm2m_path_object, vibboard_devices[i].device_id);
				int_8_val = vibboard_devices[i].closer_beacon_id;
				result += lwm2m_engine_set_s8(lwm2m_obj_path, int_8_val);

				printk("Closer beacon id: %d \n", vibboard_devices[i].closer_beacon_id);

				snprintk(lwm2m_obj_path, sizeof(lwm2m_obj_path), "%s/%d/8001", lwm2m_path_object, vibboard_devices[i].device_id);
				int_8_val = vibboard_devices[i].closer_beacon_rssi;
				result += lwm2m_engine_set_s8(lwm2m_obj_path, int_8_val);
				
				printk("Closer beacon rssi: %d \n", vibboard_devices[i].closer_beacon_rssi);
			}
			printk("Aux result: %d \n", aux_result);aux_result=0;
		}

	LOG_DBG("RC after new messages: %d\n", result);
	
	}

	return result;

}

int lwm2m_set_bl654_sensor_data(float temperature, float humidity,
				float pressure)
{
	int result = 0;

	if (lwm2m_initialized) {
		struct float32_value float_value;

#ifdef CONFIG_LWM2M_IPSO_TEMP_SENSOR
		float_value = make_float32_value(temperature);
		result += lwm2m_engine_set_float32("3303/0/5700", &float_value);
#endif

		/* Temperature is used to test generic sensor */
#ifdef CONFIG_LWM2M_IPSO_GENERIC_SENSOR
		float_value = make_float32_value(temperature);
		result += lwm2m_engine_set_float32("3303/0/5700", &float_value);
#endif

#ifdef CONFIG_LWM2M_IPSO_HUMIDITY_SENSOR
		float_value = make_float32_value(humidity);
		result += lwm2m_engine_set_float32("3304/0/5700", &float_value);
#endif

#ifdef CONFIG_LWM2M_IPSO_PRESSURE_SENSOR
		float_value = make_float32_value(pressure);
		result += lwm2m_engine_set_float32("3323/0/5700", &float_value);
#endif
	}
	return result;
}

/******************************************************************************/
/* Local Function Definitions                                                 */
/******************************************************************************/
static int device_reboot_cb(uint16_t obj_inst_id)
{
#ifdef CONFIG_REBOOT
	LOG_INF("DEVICE: REBOOT");
	power_reboot_module(REBOOT_TYPE_NORMAL);
	return 0;
#else
	return -1;
#endif
}

static int device_factory_default_cb(uint16_t obj_inst_id)
{
	LOG_INF("DEVICE: FACTORY DEFAULT");
	return -1;
}

static void *current_time_read_cb(uint16_t obj_inst_id, uint16_t res_id,
				  uint16_t res_inst_id, size_t *data_len)
{
	/* The device object doesn't allow this to be set because
	 * reads are intercepted */
	ARG_UNUSED(obj_inst_id);
	lwm2m_time = Qrtc_GetEpoch();
	*data_len = sizeof(lwm2m_time);
	return &lwm2m_time;
}

static int lwm2m_setup(const char *serial_number, const char *imei)
{
	int ret;
	char *server_url;
	uint16_t server_url_len;
	uint8_t server_url_flags;

	/* setup SECURITY object */

	/* Server URL */
	ret = lwm2m_engine_get_res_data("0/0/0", (void **)&server_url,
					&server_url_len, &server_url_flags);
	if (ret < 0) {
		return ret;
	}

	snprintk(server_url, server_url_len, "coap%s//%s",
		 IS_ENABLED(CONFIG_LWM2M_DTLS_SUPPORT) ? "s:" : ":",
		 server_addr);
	LOG_WRN("Server URL: %s", log_strdup(server_url));

	/* Security Mode */
	LOG_DBG("Security Mode: %d",
		IS_ENABLED(CONFIG_LWM2M_DTLS_SUPPORT) ? 0 : 3);
	lwm2m_engine_set_u8("0/0/2",
			    IS_ENABLED(CONFIG_LWM2M_DTLS_SUPPORT) ? 0 : 3);
#if defined(CONFIG_LWM2M_DTLS_SUPPORT)
	lwm2m_engine_set_string("0/0/3", (char *)ble_lwm2m_get_client_id());
	lwm2m_engine_set_opaque("0/0/5", (void *)ble_lwm2m_get_client_psk(),
				CONFIG_LWM2M_PSK_SIZE);
#else
	lwm2m_engine_set_string("0/0/3", "0");
	lwm2m_engine_set_opaque("0/0/5", NULL, 0);
#endif /* CONFIG_LWM2M_DTLS_SUPPORT */
	

	/* setup SERVER object */

	/* setup DEVICE object */
	char *s;
	s = (char *)dis_get_manufacturer_name();
	lwm2m_engine_set_res_data("3/0/0", s, lwm2m_str_size(s),
				  LWM2M_RES_DATA_FLAG_RO);
	s = (char *)dis_get_model_number();
	lwm2m_engine_set_res_data("3/0/1", s, lwm2m_str_size(s),
				  LWM2M_RES_DATA_FLAG_RO);
	s = (char *)dis_get_software_revision();
	lwm2m_engine_set_res_data("3/0/2", (char *)serial_number,
				  lwm2m_str_size(serial_number),
				  LWM2M_RES_DATA_FLAG_RO);
	lwm2m_engine_set_res_data("3/0/3", s, lwm2m_str_size(s),
				  LWM2M_RES_DATA_FLAG_RO);
	lwm2m_engine_register_exec_callback("3/0/4", device_reboot_cb);
	lwm2m_engine_register_exec_callback("3/0/5", device_factory_default_cb);
	lwm2m_engine_register_read_callback("3/0/13", current_time_read_cb);

	lwm2m_engine_create_res_inst("3/0/6/0");
	lwm2m_engine_set_res_data("3/0/6/0", &bat_idx, sizeof(bat_idx), 0);
	lwm2m_engine_create_res_inst("3/0/7/0");
	lwm2m_engine_set_res_data("3/0/7/0", &bat_mv, sizeof(bat_mv), 0);
	lwm2m_engine_create_res_inst("3/0/8/0");
	lwm2m_engine_set_res_data("3/0/8/0", &bat_ma, sizeof(bat_ma), 0);
	lwm2m_engine_create_res_inst("3/0/6/1");
	lwm2m_engine_set_res_data("3/0/6/1", &usb_idx, sizeof(usb_idx), 0);
	lwm2m_engine_create_res_inst("3/0/7/1");
	lwm2m_engine_set_res_data("3/0/7/1", &usb_mv, sizeof(usb_mv), 0);
	lwm2m_engine_create_res_inst("3/0/8/1");
	lwm2m_engine_set_res_data("3/0/8/1", &usb_ma, sizeof(usb_ma), 0);

	/* IPSO: Light Control object */
	// lwm2m_engine_create_obj_inst("3311/0");
	// lwm2m_engine_register_post_write_callback("3311/0/5850", led_on_off_cb);

	/* setup objects for remote sensors */
	create_bl654_sensor_objects();

	create_vibboard_objects();

	return 0;
}

static void rd_client_event(struct lwm2m_ctx *client,
			    enum lwm2m_rd_client_event client_event)
{
	switch (client_event) {
	case LWM2M_RD_CLIENT_EVENT_NONE:
		/* do nothing */
		break;

	case LWM2M_RD_CLIENT_EVENT_BOOTSTRAP_REG_FAILURE:
		LOG_DBG("Bootstrap registration failure!");
		break;

	case LWM2M_RD_CLIENT_EVENT_BOOTSTRAP_REG_COMPLETE:
		LOG_DBG("Bootstrap registration complete");
		break;

	case LWM2M_RD_CLIENT_EVENT_BOOTSTRAP_TRANSFER_COMPLETE:
		LOG_DBG("Bootstrap transfer complete");
		break;

	case LWM2M_RD_CLIENT_EVENT_REGISTRATION_FAILURE:
		LOG_DBG("Registration failure!");
		break;

	case LWM2M_RD_CLIENT_EVENT_REGISTRATION_COMPLETE:
		LOG_DBG("Registration complete");
		break;

	case LWM2M_RD_CLIENT_EVENT_REG_UPDATE_FAILURE:
		LOG_DBG("Registration update failure!");
		break;

	case LWM2M_RD_CLIENT_EVENT_REG_UPDATE_COMPLETE:
		LOG_DBG("Registration update complete");
		break;

	case LWM2M_RD_CLIENT_EVENT_DEREGISTER_FAILURE:
		LOG_DBG("Deregister failure!");
		break;

	case LWM2M_RD_CLIENT_EVENT_DISCONNECT:
		LOG_DBG("Disconnected");
		break;
	case LWM2M_RD_CLIENT_EVENT_QUEUE_MODE_RX_OFF:
		/* do nothing */
		break;
	}
}

static int resolve_server_address(void)
{
	struct addrinfo hints = {
		.ai_family = AF_UNSPEC,
		.ai_socktype = SOCK_DGRAM,
	};

	static struct addrinfo *result;
	int ret = dns_resolve_server_addr(ble_lwm2m_get_peer_url(), NULL,
					  &hints, &result);
	if (ret == 0) {
		ret = dns_build_addr_string(server_addr, result);
	}
	freeaddrinfo(result);
	return ret;
}

static void lwm2m_client_init_internal(void)
{
	lwm2m_initialized = false;
	struct lte_status *lte_status = lteGetStatus();
	int ret;

	ret = resolve_server_address();
	if (ret < 0) {
		return;
	}
	LOG_DBG("LWM2M Setup");
	ret = lwm2m_setup(lte_status->serialNumber, lte_status->IMEI);
	if (ret < 0) {
		LOG_ERR("Cannot setup LWM2M fields (%d)", ret);
		return;
	}

	(void)memset(&client, 0x0, sizeof(client));
#if defined(CONFIG_LWM2M_DTLS_SUPPORT)
	client.tls_tag = TLS_TAG;
#endif

	/* client.sec_obj_inst is 0 as a starting point */
	char endpoint_name[CONFIG_LWM2M_CLIENT_ENDPOINT_MAX_SIZE];
	memset(endpoint_name, 0, sizeof(endpoint_name));
	snprintk(endpoint_name, CONFIG_LWM2M_CLIENT_ENDPOINT_MAX_SIZE, "%s_%s",
		dis_get_model_number(), ENPOINT_UNIQUE_CODE);
	LOG_DBG("Endpoint name: %s", log_strdup(endpoint_name));
	lwm2m_rd_client_start(&client, endpoint_name, rd_client_event);
	lwm2m_initialized = true;
}

static int led_on_off_cb(uint16_t obj_inst_id, uint16_t res_id,
			 uint16_t res_inst_id, uint8_t *data, uint16_t data_len,
			 bool last_block, size_t total_size)
{
	uint8_t led_val = *(uint8_t *)data;
	if (led_val != led_state) {
		if (led_val) {
			led_turn_on(GREEN_LED);
		} else {
			led_turn_off(GREEN_LED);
		}
		led_state = led_val;
		/* reset time on counter */
		lwm2m_engine_set_s32("3311/0/5852", 0);
	}

	return 0;
}

static void create_bl654_sensor_objects(void)
{
	/* The BL654 Sensor contains a BME 280. */
	/* 5603 and 5604 are the range of values supported by sensor. */
	struct float32_value float_value;
#ifdef CONFIG_LWM2M_IPSO_TEMP_SENSOR
	lwm2m_engine_create_obj_inst("3303/0");
	lwm2m_engine_set_string("3303/0/5701", "C");
	float_value.val1 = -40;
	lwm2m_engine_set_float32("3303/0/5603", &float_value);
	float_value.val1 = 85;
	lwm2m_engine_set_float32("3303/0/5604", &float_value);
#endif

#ifdef CONFIG_LWM2M_IPSO_GENERIC_SENSOR
	/* temperature used for test */
	lwm2m_engine_create_obj_inst("3303/0");
	lwm2m_engine_set_string("3303/0/5701", "C");
	float_value.val1 = -40;
	lwm2m_engine_set_float32("3303/0/5603", &float_value);
	float_value.val1 = 85;
	lwm2m_engine_set_float32("3303/0/5604", &float_value);
#endif

#ifdef CONFIG_LWM2M_IPSO_HUMIDITY_SENSOR
	lwm2m_engine_create_obj_inst("3304/0");
	lwm2m_engine_set_string("3304/0/5701", "%");
	float_value.val1 = 0;
	lwm2m_engine_set_float32("3304/0/5603", &float_value);
	float_value.val1 = 100;
	lwm2m_engine_set_float32("3304/0/5604", &float_value);
#endif

#ifdef CONFIG_LWM2M_IPSO_PRESSURE_SENSOR
	lwm2m_engine_create_obj_inst("3323/0");
	lwm2m_engine_set_string("3323/0/5701", "Pa");
	float_value.val1 = 300;
	lwm2m_engine_set_float32("3323/0/5603", &float_value);
	float_value.val1 = 1100000;
	lwm2m_engine_set_float32("3323/0/5604", &float_value);
#endif
}

static void create_vibboard_objects(void){

	char lwm2m_obj_path[20];
	char lwm2m_path_object[6] = "42790";
	char lwm2m_resource_path[20];
	for (int i = 0; i < VIBBOARD_NR; i++) {

		snprintk(lwm2m_obj_path, sizeof(lwm2m_obj_path), "%s/%d", lwm2m_path_object, i);
		lwm2m_engine_create_obj_inst(lwm2m_obj_path);

		for (int j = 0;j < TD_NR_FEATURES; j++){
			// snprintk(lwm2m_resource_path, sizeof(lwm2m_resource_path), "%s/4000/%d", lwm2m_obj_path, j);
			// lwm2m_engine_create_res_inst(lwm2m_resource_path);
			snprintk(lwm2m_resource_path, sizeof(lwm2m_resource_path), "%s/4001/%d", lwm2m_obj_path, j);
			lwm2m_engine_create_res_inst(lwm2m_resource_path);
			snprintk(lwm2m_resource_path, sizeof(lwm2m_resource_path), "%s/4002/%d", lwm2m_obj_path, j);
			lwm2m_engine_create_res_inst(lwm2m_resource_path);
			snprintk(lwm2m_resource_path, sizeof(lwm2m_resource_path), "%s/4003/%d", lwm2m_obj_path, j);
			lwm2m_engine_create_res_inst(lwm2m_resource_path);	
		}
		// snprintk(lwm2m_resource_path, sizeof(lwm2m_resource_path), "%s/5000/%d", lwm2m_obj_path, 0);
		// lwm2m_engine_create_res_inst(lwm2m_resource_path);
		// snprintk(lwm2m_resource_path, sizeof(lwm2m_resource_path), "%s/5000/%d", lwm2m_obj_path, 1);
		// lwm2m_engine_create_res_inst(lwm2m_resource_path);

		for (int j = 0;j < FD_NR_PEAKS; j++){
			snprintk(lwm2m_resource_path, sizeof(lwm2m_resource_path), "%s/5001/%d", lwm2m_obj_path, j);
			lwm2m_engine_create_res_inst(lwm2m_resource_path);
			snprintk(lwm2m_resource_path, sizeof(lwm2m_resource_path), "%s/5002/%d", lwm2m_obj_path, j);
			lwm2m_engine_create_res_inst(lwm2m_resource_path);
			snprintk(lwm2m_resource_path, sizeof(lwm2m_resource_path), "%s/5003/%d", lwm2m_obj_path, j);
			lwm2m_engine_create_res_inst(lwm2m_resource_path);
			snprintk(lwm2m_resource_path, sizeof(lwm2m_resource_path), "%s/5004/%d", lwm2m_obj_path, j);
			lwm2m_engine_create_res_inst(lwm2m_resource_path);
			snprintk(lwm2m_resource_path, sizeof(lwm2m_resource_path), "%s/5005/%d", lwm2m_obj_path, j);
			lwm2m_engine_create_res_inst(lwm2m_resource_path);
			snprintk(lwm2m_resource_path, sizeof(lwm2m_resource_path), "%s/5006/%d", lwm2m_obj_path, j);
			lwm2m_engine_create_res_inst(lwm2m_resource_path);
		}
	}
}

static struct float32_value make_float32_value(float v)
{
	struct float32_value f;

	f.val1 = (s32_t)v;
	//f.val2 = (s32_t)(LWM2M_FLOAT32_DEC_MAX * (v - f.val1));
	f.val2 = (s32_t)((v - f.val1) * LWM2M_FLOAT32_DEC_MAX + 0.5); // multiply decimal portion by 1000000 and round
	f.val2 = f.val2 / 10000 * 10000; // truncate to 2 decimal places with LWM2M_FLOAT32_DEC_MAX=1000000

	return f;
}

static size_t lwm2m_str_size(const char *s)
{
	return strlen(s) + 1;
}
