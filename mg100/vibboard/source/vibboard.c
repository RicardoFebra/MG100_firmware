/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "vibboard.h"

#include <zephyr/types.h>
#include <stddef.h>
#include <stdlib.h>
#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <sys/byteorder.h>

#include "ad_find.h" //AdHandler

#include "FrameworkIncludes.h"


/******************************************************************************/
/* Local Data Definitions                                                     */
/******************************************************************************/

struct vibboard_device *vibboard_devices_aux;
int vibboard_count = 0;

#define NAME_LEN 30

static bool FindVibboardAdvertisement(AdHandle_t *pHandle);

static struct bt_conn *default_conn;

static bool data_cb_vibboard(struct bt_data *data, void *user_data){

	struct vibboard_device *vibboard_aux_aux = user_data;

	printk("Callback data type: %d\n", data->type);
	printk("Callback data values: ");
	for (int i = 0; i < data->data_len; i++) {
		printk("%x ", data->data[i]);
	}
	printk("\n");

	switch(data->type){
		case BT_DATA_NAME_COMPLETE:
			memcpy(vibboard_aux_aux->device_name, data->data, MIN(data->data_len, NAME_LEN - 1));
			// vibboard device id is the number in the first two ascii characters of the device name
			char device_id_str[2];
			memcpy(device_id_str, vibboard_aux_aux->device_name, 2);
			vibboard_aux_aux->device_id = atoi(device_id_str);

			return false;

		case BT_DATA_MANUFACTURER_DATA:
			if ((data->data_len) <= 31) {
				vibboard_aux_aux->TD_machine_state = data->data[0];
				vibboard_aux_aux->device_state = 0;
			}else{
				vibboard_aux_aux->TD_machine_state = -2;
				vibboard_aux_aux->device_state = 1;
				fill_vibboard_SP_data(&vibboard_aux_aux, data);
			}
			return false;
		default:
			return true;
	}
	return true;
}

static void device_found(const bt_addr_le_t *addr, s8_t rssi, u8_t type,
			 struct net_buf_simple *ad)
{

	int err;

	if (default_conn) {
		return;
	}

	if (MatchVibboardManufacturerID(ad)) {
		printk("Vibboard device found\n");
	}else{
		return;
	}

	char addr_str[BT_ADDR_LE_STR_LEN];
	struct vibboard_device vibboard_aux_aux;

	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
	printk("[DEVICE]: %s, AD evt type %u, AD data len %u, RSSI %i\n",
	       addr_str, type, ad->len, rssi);
	// print data in hex
	printk("Data: ");
	for (int i = 0; i < ad->len; i++) {
		printk("%x ", ad->data[i]);
	}
	printk("\n");
	MatchVibboardBT_ID(ad, &vibboard_aux_aux);
	GetVibboardBT_Data(ad, &vibboard_aux_aux);
	//setting up the bt_data_parse function and prints it.
	//bt_data_parse(ad, data_cb_vibboard, &vibboard_aux_aux);

	/* connect only to devices in close proximity */
	if (rssi < -70) {
		return;
	}

	// if (bt_le_scan_stop()) {
	// 	return;
	// }

	vibboard_update_table(vibboard_aux_aux);

	// err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
	// 			BT_LE_CONN_PARAM_DEFAULT, &default_conn);
	// if (err) {
	// 	printk("Create conn to %s failed (%u)\n", addr_str, err);
	// 	start_scan();
	// }
}

void vibboard_init_table(struct vibboard_device *vibboard_devices){

	for (int i = 0; i < VIBBOARD_NR; i++){
		vibboard_devices[i].device_id = -1;
		vibboard_devices[i].vibboard_mode = -99;
		vibboard_devices[i].device_state = -1;
		vibboard_devices[i].TD_machine_state = -1;
		memcpy(vibboard_devices[i].device_name, "NotAssigned", 15);
		for (int j = 0; j < TD_NR_FEATURES; j++){
			vibboard_devices[i].TD_acc_sp_data_scalars[j] = -1;
			vibboard_devices[i].TD_acc_sp_data_x_axis[j] = -1;
			vibboard_devices[i].TD_acc_sp_data_y_axis[j] = -1;
			vibboard_devices[i].TD_acc_sp_data_z_axis[j] = -1;
		}
		vibboard_devices[i].FD_acc_data_peaks_freq_scalar = 0;
		vibboard_devices[i].FD_acc_data_peaks_amp_scalar = 0;
		for (int j = 0; j < FD_NR_PEAKS; j++){
			vibboard_devices[i].FD_acc_data_peaks_freq_x_axis[j] = 0;
			vibboard_devices[i].FD_acc_data_peaks_amps_x_axis[j] = 0;
			vibboard_devices[i].FD_acc_data_peaks_freq_y_axis[j] = 0;
			vibboard_devices[i].FD_acc_data_peaks_amps_y_axis[j] = 0;
			vibboard_devices[i].FD_acc_data_peaks_freq_z_axis[j] = 0;
			vibboard_devices[i].FD_acc_data_peaks_amps_z_axis[j] = 0;
		}
	}
}

void vibboard_remove_id_from_table(struct vibboard_device *vibboard_devices, int device_id){
	vibboard_devices[device_id].device_id = -1;
	vibboard_devices[device_id].vibboard_mode = -99;
	vibboard_devices[device_id].device_state = -1;
	vibboard_devices[device_id].TD_machine_state = -1;
	memcpy(vibboard_devices[device_id].device_name, "NotAssigned", 15);
}

void vibboard_update_table(struct vibboard_device vibboard_aux_aux){
	// check if the device is already in the table

	if (vibboard_aux_aux.device_id == -1){
		return;
	}else{
		// update the device in the table
		vibboard_devices_aux[vibboard_aux_aux.device_id].device_id = vibboard_aux_aux.device_id;
		memcpy(vibboard_devices_aux[vibboard_aux_aux.device_id].device_name, vibboard_aux_aux.device_name, sizeof(vibboard_aux_aux.device_name));
		vibboard_devices_aux[vibboard_aux_aux.device_id].vibboard_mode = vibboard_aux_aux.vibboard_mode;
		vibboard_devices_aux[vibboard_aux_aux.device_id].device_state = vibboard_aux_aux.device_state;
		vibboard_devices_aux[vibboard_aux_aux.device_id].TD_machine_state = vibboard_aux_aux.TD_machine_state;
		vibboard_devices_aux[vibboard_aux_aux.device_id].device_battery_voltage = vibboard_aux_aux.device_battery_voltage;
		vibboard_devices_aux[vibboard_aux_aux.device_id].last_relative_update_time = k_uptime_get();
	}

	if(vibboard_aux_aux.device_state == 1){

		for (int i = 0; i < TD_NR_FEATURES; i++){
			vibboard_devices_aux[vibboard_aux_aux.device_id].TD_acc_sp_data_scalars[i] = vibboard_aux_aux.TD_acc_sp_data_scalars[i];
			vibboard_devices_aux[vibboard_aux_aux.device_id].TD_acc_sp_data_x_axis[i] = vibboard_aux_aux.TD_acc_sp_data_x_axis[i];
			vibboard_devices_aux[vibboard_aux_aux.device_id].TD_acc_sp_data_y_axis[i] = vibboard_aux_aux.TD_acc_sp_data_y_axis[i];
			vibboard_devices_aux[vibboard_aux_aux.device_id].TD_acc_sp_data_z_axis[i] = vibboard_aux_aux.TD_acc_sp_data_z_axis[i];
		}

		vibboard_devices_aux[vibboard_aux_aux.device_id].FD_acc_data_peaks_freq_scalar = vibboard_aux_aux.FD_acc_data_peaks_freq_scalar;
		vibboard_devices_aux[vibboard_aux_aux.device_id].FD_acc_data_peaks_amp_scalar = vibboard_aux_aux.FD_acc_data_peaks_amp_scalar;

		for (int i = 0; i < FD_NR_PEAKS; i++){
			vibboard_devices_aux[vibboard_aux_aux.device_id].FD_acc_data_peaks_freq_x_axis[i] = vibboard_aux_aux.FD_acc_data_peaks_freq_x_axis[i];
			vibboard_devices_aux[vibboard_aux_aux.device_id].FD_acc_data_peaks_amps_x_axis[i] = vibboard_aux_aux.FD_acc_data_peaks_amps_x_axis[i];
			vibboard_devices_aux[vibboard_aux_aux.device_id].FD_acc_data_peaks_freq_y_axis[i] = vibboard_aux_aux.FD_acc_data_peaks_freq_y_axis[i];
			vibboard_devices_aux[vibboard_aux_aux.device_id].FD_acc_data_peaks_amps_y_axis[i] = vibboard_aux_aux.FD_acc_data_peaks_amps_y_axis[i];
			vibboard_devices_aux[vibboard_aux_aux.device_id].FD_acc_data_peaks_freq_z_axis[i] = vibboard_aux_aux.FD_acc_data_peaks_freq_z_axis[i];
			vibboard_devices_aux[vibboard_aux_aux.device_id].FD_acc_data_peaks_amps_z_axis[i] = vibboard_aux_aux.FD_acc_data_peaks_amps_z_axis[i];
		}

	}

	VibboardMsg_t *pMsg = (VibboardMsg_t *)BufferPool_TryToTake(sizeof(VibboardMsg_t));	
	if (pMsg == NULL) {
		return;
	}
	pMsg->header.msgCode = FMC_VIBBOARD_EVENT;
	pMsg->header.rxId = FWK_ID_CLOUD;
	FRAMEWORK_MSG_SEND(pMsg);
}

void print_vibboard_table(struct vibboard_device *vibboard_devices){
	for (int i = 0; i < VIBBOARD_NR; i++){
		printk("Vibboard %d ", vibboard_devices[i].device_id);
		printk("Device name: %s ", vibboard_devices[i].device_name);
		printk("Vibboard mode: %d ", vibboard_devices[i].vibboard_mode);
		printk("Vibboard state: %d ", vibboard_devices[i].device_state);
		printk("Device voltage: %d ", vibboard_devices[i].device_battery_voltage);
		printk("TD machine state: %d ", vibboard_devices[i].TD_machine_state);
		printk("TD acc sp data x axis 1st: %d ", (int)vibboard_devices[i].TD_acc_sp_data_x_axis[0]);
		printk("TD acc sp data x axis 2nd: %d ", (int)vibboard_devices[i].TD_acc_sp_data_x_axis[1]);
		printk("TD acc sp data x axis 3nd: %d ", (int)vibboard_devices[i].TD_acc_sp_data_x_axis[2]);
		printk("TD acc sp data x axis 4nd: %d ", (int)vibboard_devices[i].TD_acc_sp_data_x_axis[3]);
		printk("TD acc sp data x axis 5nd: %d ", (int)vibboard_devices[i].TD_acc_sp_data_x_axis[4]);
		printk("FD acc data peaks freq x axis 1st: %d ", vibboard_devices[i].FD_acc_data_peaks_freq_x_axis[0]);
		printk("FD acc data peaks amps x axis 1st: %d ", vibboard_devices[i].FD_acc_data_peaks_amps_x_axis[0]);
		printk("FD acc data peaks amps z axis 1st: %d ", vibboard_devices[i].FD_acc_data_peaks_amps_z_axis[0]);
		printk("FD acc data peaks amps z axis 2st: %d ", vibboard_devices[i].FD_acc_data_peaks_amps_z_axis[1]);
		printk("FD acc data peaks amps z axis 3st: %d ", vibboard_devices[i].FD_acc_data_peaks_amps_z_axis[2]);
		printk("FD acc data peaks amps z axis 4st: %d ", vibboard_devices[i].FD_acc_data_peaks_amps_z_axis[3]);
		printk("\n");
	}
}

bool GetVibboardBT_Data(struct net_buf_simple *ad, struct vibboard_device *user_data){

	struct vibboard_device *vibboard_aux_aux = user_data;

	AdHandle_t manHandle = AdFind_Type(
		ad->data, ad->len, BT_DATA_MANUFACTURER_DATA, BT_DATA_INVALID);
	if (manHandle.pPayload == NULL) {
		return false;
	}
	struct bt_data bt_data_aux;
	bt_data_aux.data_len = manHandle.size;
	bt_data_aux.type = BT_DATA_MANUFACTURER_DATA;
	u8_t aux_data[bt_data_aux.data_len];
	bt_data_aux.data = aux_data;
	// copy payload to bt_data_aux data but ignore first 2 bytes
	memcpy(bt_data_aux.data, manHandle.pPayload + 2, bt_data_aux.data_len);

	printk("GetVibboardBT_Data: Size = %d, Type = %d\n", bt_data_aux.data_len, bt_data_aux.type);

	if ((manHandle.size) <= 31) {
		vibboard_aux_aux->device_state = 0;
		vibboard_aux_aux->TD_machine_state = manHandle.pPayload[0];
	}else{
		vibboard_aux_aux->TD_machine_state = -1;
		vibboard_aux_aux->device_state = 1;
		fill_vibboard_SP_data(&vibboard_aux_aux, &bt_data_aux);
	}

	return true;
}

bool MatchVibboardBT_ID(struct net_buf_simple *ad, struct vibboard_device *vibboard_aux_aux){
	AdHandle_t nameHandle = AdFind_Type(
		ad->data, ad->len, BT_DATA_NAME_COMPLETE, BT_DATA_INVALID);
	if (nameHandle.pPayload == NULL) {
		return false;
	}

	memcpy(vibboard_aux_aux->device_name, nameHandle.pPayload, MIN(nameHandle.size, NAME_LEN - 1));
	// vibboard device id is the number in the first two ascii characters of the device name
	char device_id_str[2];
	memcpy(device_id_str, vibboard_aux_aux->device_name, 2);
	vibboard_aux_aux->device_id = atoi(device_id_str);

	printk("MatchVibboardBT_ID: Device ID = %d\n", vibboard_aux_aux->device_id);

	return true;
}

bool MatchVibboardManufacturerID(struct net_buf_simple *ad)
{
	AdHandle_t manHandle = AdFind_Type(
		ad->data, ad->len, BT_DATA_MANUFACTURER_DATA, BT_DATA_INVALID);
	if (manHandle.pPayload == NULL) {
		return false;
	}

	return FindVibboardAdvertisement(&manHandle);
}

static bool FindVibboardAdvertisement(AdHandle_t *pHandle)
{
	if (pHandle->pPayload != NULL) {
		if (memcmp(pHandle->pPayload, VIBBOARD_AD_HEADER,
				sizeof(VIBBOARD_AD_HEADER)) == 0) {
			return true;
		}
	}
	return false;
}

void start_scan_vibboard(struct vibboard_device* vibboard_devices){
	int err;

	vibboard_devices_aux = vibboard_devices;

	struct bt_le_scan_param scan_param = {
		.type       = BT_LE_SCAN_TYPE_ACTIVE,
		.options    = BT_LE_SCAN_OPT_NONE,
		.interval   = BT_GAP_SCAN_FAST_INTERVAL,
		.window     = BT_GAP_SCAN_FAST_WINDOW,
	};

	/* This demo doesn't require active scan */
	err = bt_le_scan_start(&scan_param, device_found);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
	}

	// get device data

	printk("Scanning successfully started\n");

}


static void connected(struct bt_conn *conn, u8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		printk("Failed to connect to %s (%u)\n", addr, err);

		bt_conn_unref(default_conn);
		default_conn = NULL;

		start_scan();
		return;
	}

	if (conn != default_conn) {
		return;
	}

	printk("Connected: %s\n", addr);

	bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

static void disconnected(struct bt_conn *conn, u8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (conn != default_conn) {
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected: %s (reason 0x%02x)\n", addr, reason);

	bt_conn_unref(default_conn);
	default_conn = NULL;

	start_scan();
}

static struct bt_conn_cb conn_callbacks = {
		.connected = connected,
		.disconnected = disconnected,
};


void fill_vibboard_SP_data(struct vibboard_device **vibboard_device, struct bt_data *data){

	int32_t int32_val_aux;
	int8_t int8_val_aux;
	uint16_t uint16_val_aux;

	printk("[Important] Filling vibboard SP data\n");
	printk("Data fill_vibboard_SP_data: ");
	for (int i = 0; i < data->data_len; i++) {
		printk("%x ", data->data[i]);
	}
	printk("\n");
	// fill the vibboard device struct with the data from the advertisement
	int Vibboard_Vars_value_index=0;
	int8_val_aux = data->data[Vibboard_Vars_value_index++];
	(*vibboard_device)->vibboard_mode = int8_val_aux;
	printf("vibboard_mode: %d\n", (*vibboard_device)->vibboard_mode);
	int8_val_aux = data->data[Vibboard_Vars_value_index++];
	(*vibboard_device)->device_state = int8_val_aux;
	printf("device_state: %d\n", (*vibboard_device)->device_state);
	int8_val_aux = data->data[Vibboard_Vars_value_index++];
	(*vibboard_device)->TD_machine_state = int8_val_aux;
	printf("TD_machine_state: %d\n", (*vibboard_device)->TD_machine_state);
	uint16_val_aux = data->data[Vibboard_Vars_value_index++] << 8 | data->data[Vibboard_Vars_value_index++];
	(*vibboard_device)->device_battery_voltage = uint16_val_aux;
	printf("TD_battery_voltage: %d\n", (*vibboard_device)->device_battery_voltage);
	
	/*---------------TD-------------*/
	int TD_Features_Carac_value_index=Vibboard_Vars_value_index;
	// get the scalar of the TD features
	for (int i = 0; i < TD_NR_FEATURES; i++){
		(*vibboard_device)->TD_acc_sp_data_scalars[i] = 100;
	}

	// for each n bytes convert to integer and divide by the corresponding scalar. Order is mean, std_dev, kurtosis, skewness, rms
	for (int i = 0; i < TD_NR_FEATURES; i++){
		// get 4 bytes
		int32_val_aux = ((data->data[TD_Features_Carac_value_index] << 24) | (data->data[TD_Features_Carac_value_index + 1] << 16) | (data->data[TD_Features_Carac_value_index + 2] << 8) | (data->data[TD_Features_Carac_value_index + 3]));
		// divide by the scalar
		(*vibboard_device)->TD_acc_sp_data_x_axis[i] = (float)int32_val_aux / (*vibboard_device)->TD_acc_sp_data_scalars[i];
		printf("TD Value x axis: %3.2f ,transmitted: %d,  Hexadecimal : %x %x %x %x\t",(*vibboard_device)->TD_acc_sp_data_x_axis[i],  int32_val_aux, data->data[TD_Features_Carac_value_index], data->data[TD_Features_Carac_value_index + 1], data->data[TD_Features_Carac_value_index + 2], data->data[TD_Features_Carac_value_index + 3]);

		TD_Features_Carac_value_index += 4;
	}
	printf("\n");
	// repeat for y and z axis
	for (int i = 0; i < TD_NR_FEATURES; i++){
		int32_val_aux = ((data->data[TD_Features_Carac_value_index] << 24) | (data->data[TD_Features_Carac_value_index + 1] << 16) | (data->data[TD_Features_Carac_value_index + 2] << 8) | (data->data[TD_Features_Carac_value_index + 3]));
		(*vibboard_device)->TD_acc_sp_data_y_axis[i] = (float)int32_val_aux / (*vibboard_device)->TD_acc_sp_data_scalars[i];
		printf("TD Value y axis: %3.2f ,transmitted: %d,  Hexadecimal : %x %x %x %x\t",(*vibboard_device)->TD_acc_sp_data_y_axis[i],  int32_val_aux, data->data[TD_Features_Carac_value_index], data->data[TD_Features_Carac_value_index + 1], data->data[TD_Features_Carac_value_index + 2], data->data[TD_Features_Carac_value_index + 3]);

		TD_Features_Carac_value_index += 4;
	}
	printf("\n");
	for (int i = 0; i < TD_NR_FEATURES; i++){
		int32_val_aux = ((data->data[TD_Features_Carac_value_index] << 24) | (data->data[TD_Features_Carac_value_index + 1] << 16) | (data->data[TD_Features_Carac_value_index + 2] << 8) | (data->data[TD_Features_Carac_value_index + 3]));
		(*vibboard_device)->TD_acc_sp_data_z_axis[i] = (float)int32_val_aux / (*vibboard_device)->TD_acc_sp_data_scalars[i];
		printf("TD Value z axis: %3.2f ,transmitted: %d,  Hexadecimal : %x %x %x %x\t",(*vibboard_device)->TD_acc_sp_data_z_axis[i],  int32_val_aux, data->data[TD_Features_Carac_value_index], data->data[TD_Features_Carac_value_index + 1], data->data[TD_Features_Carac_value_index + 2], data->data[TD_Features_Carac_value_index + 3]);

		TD_Features_Carac_value_index += 4;
	}
	printf("\n");
	/*---------------FD-------------*/
	int FD_Features_Carac_value_index=TD_Features_Carac_value_index;
	// get the peaks
	// x axis
	for (int i = 0; i < FD_NR_PEAKS; i++){
		(*vibboard_device)->FD_acc_data_peaks_freq_x_axis[i] = ((data->data[FD_Features_Carac_value_index] << 24) | (data->data[FD_Features_Carac_value_index + 1] << 16) | (data->data[FD_Features_Carac_value_index + 2] << 8) | (data->data[FD_Features_Carac_value_index + 3]));
		printk("FD Frequency x axis: %d, Hexadecimal : %x %x %x %x\t", (*vibboard_device)->FD_acc_data_peaks_freq_x_axis[i], data->data[FD_Features_Carac_value_index], data->data[FD_Features_Carac_value_index + 1], data->data[FD_Features_Carac_value_index + 2], data->data[FD_Features_Carac_value_index + 3]);
		
		FD_Features_Carac_value_index += 4;
		(*vibboard_device)->FD_acc_data_peaks_amps_x_axis[i] = ((data->data[FD_Features_Carac_value_index] << 24) | (data->data[FD_Features_Carac_value_index + 1] << 16) | (data->data[FD_Features_Carac_value_index + 2] << 8) | (data->data[FD_Features_Carac_value_index + 3]));
		printk("FD Amplitude x axis: %d, Hexadecimal : %x %x %x %x\n", (*vibboard_device)->FD_acc_data_peaks_amps_x_axis[i], data->data[FD_Features_Carac_value_index], data->data[FD_Features_Carac_value_index + 1], data->data[FD_Features_Carac_value_index + 2], data->data[FD_Features_Carac_value_index + 3]);
		
		FD_Features_Carac_value_index += 4;
	}
	// y axis
	for (int i = 0; i < FD_NR_PEAKS; i++){
		(*vibboard_device)->FD_acc_data_peaks_freq_y_axis[i] = ((data->data[FD_Features_Carac_value_index] << 24) | (data->data[FD_Features_Carac_value_index + 1] << 16) | (data->data[FD_Features_Carac_value_index + 2] << 8) | (data->data[FD_Features_Carac_value_index + 3]));
		printk("FD Frequency y axis: %d, Hexadecimal : %x %x %x %x\t", (*vibboard_device)->FD_acc_data_peaks_freq_y_axis[i], data->data[FD_Features_Carac_value_index], data->data[FD_Features_Carac_value_index + 1], data->data[FD_Features_Carac_value_index + 2], data->data[FD_Features_Carac_value_index + 3]);
		
		FD_Features_Carac_value_index += 4;
		(*vibboard_device)->FD_acc_data_peaks_amps_y_axis[i] = ((data->data[FD_Features_Carac_value_index] << 24) | (data->data[FD_Features_Carac_value_index + 1] << 16) | (data->data[FD_Features_Carac_value_index + 2] << 8) | (data->data[FD_Features_Carac_value_index + 3]));
		printk("FD Amplitude y axis: %d, Hexadecimal : %x %x %x %x\n", (*vibboard_device)->FD_acc_data_peaks_freq_y_axis[i], data->data[FD_Features_Carac_value_index], data->data[FD_Features_Carac_value_index + 1], data->data[FD_Features_Carac_value_index + 2], data->data[FD_Features_Carac_value_index + 3]);
		
		FD_Features_Carac_value_index += 4;	
	}
	// z axis
	for (int i = 0; i < FD_NR_PEAKS; i++){
		(*vibboard_device)->FD_acc_data_peaks_freq_z_axis[i] = ((data->data[FD_Features_Carac_value_index] << 24) | (data->data[FD_Features_Carac_value_index + 1] << 16) | (data->data[FD_Features_Carac_value_index + 2] << 8) | (data->data[FD_Features_Carac_value_index + 3]));
		printk("FD Frequency z axis: %d, Hexadecimal : %x %x %x %x\t", (*vibboard_device)->FD_acc_data_peaks_freq_z_axis[i], data->data[FD_Features_Carac_value_index], data->data[FD_Features_Carac_value_index + 1], data->data[FD_Features_Carac_value_index + 2], data->data[FD_Features_Carac_value_index + 3]);
		
		FD_Features_Carac_value_index += 4;
		(*vibboard_device)->FD_acc_data_peaks_amps_z_axis[i] = ((data->data[FD_Features_Carac_value_index] << 24) | (data->data[FD_Features_Carac_value_index + 1] << 16) | (data->data[FD_Features_Carac_value_index + 2] << 8) | (data->data[FD_Features_Carac_value_index + 3]));
		printk("FD Amplitude z axis: %d, Hexadecimal : %x %x %x %x\n", (*vibboard_device)->FD_acc_data_peaks_freq_z_axis[i], data->data[FD_Features_Carac_value_index], data->data[FD_Features_Carac_value_index + 1], data->data[FD_Features_Carac_value_index + 2], data->data[FD_Features_Carac_value_index + 3]);
		
		FD_Features_Carac_value_index += 4;
	}
}