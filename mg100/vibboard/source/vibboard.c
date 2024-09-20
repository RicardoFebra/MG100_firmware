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
				vibboard_aux_aux->TD_device_state = 0;
			}else{
				vibboard_aux_aux->TD_machine_state = -1;
				vibboard_aux_aux->TD_device_state = 1;
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

	if (MatchVibboard(ad)) {
		printk("Correct device found");
	}else{
		return;
	}

	char addr_str[BT_ADDR_LE_STR_LEN];
	struct vibboard_device vibboard_aux_aux;
	vibboard_aux_aux.TD_acc_mean_scalar = 100;
	vibboard_aux_aux.TD_acc_std_dev_scalar = 100;
	vibboard_aux_aux.TD_acc_kurtosis_scalar = 100;
	vibboard_aux_aux.TD_acc_skewness_scalar = 100;
	vibboard_aux_aux.TD_acc_rms_scalar = 100;

	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
	printk("Device found: %s (RSSI %d)\n", addr_str, rssi);
	printk("Device type: %u\n", type);
	// print data in hex
	printk("Data: ");
	for (int i = 0; i < ad->len; i++) {
		printk("%x ", ad->data[i]);
	}
	printk("\n");
	printk("Device data length: %u\n", ad->len);

	//setting up the bt_data_parse function and prints it.
	bt_data_parse(ad, data_cb_vibboard, &vibboard_aux_aux);
	printk("Device name: %s \n", vibboard_aux_aux.device_name);

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
		vibboard_devices[i].TD_device_state = -1;
		vibboard_devices[i].TD_machine_state = -1;
		memcpy(vibboard_devices[i].device_name, "No device", 9);
	}
}

void vibboard_update_table(struct vibboard_device vibboard_aux_aux){
	// check if the device is already in the table

	if (vibboard_aux_aux.device_id == -1){
		return;
	}else{
		// update the device in the table
		vibboard_devices_aux[vibboard_aux_aux.device_id].device_id = vibboard_aux_aux.device_id;
		memcpy(vibboard_devices_aux[vibboard_aux_aux.device_id].device_name, vibboard_aux_aux.device_name, sizeof(vibboard_aux_aux.device_name));
		vibboard_devices_aux[vibboard_aux_aux.device_id].TD_device_state = vibboard_aux_aux.TD_device_state;
		vibboard_devices_aux[vibboard_aux_aux.device_id].TD_machine_state = vibboard_aux_aux.TD_machine_state;
	}

	if(vibboard_aux_aux.TD_device_state == 1){

		vibboard_devices_aux[vibboard_aux_aux.device_id].TD_acc_mean_scalar = vibboard_aux_aux.TD_acc_mean_scalar;
		vibboard_devices_aux[vibboard_aux_aux.device_id].TD_acc_std_dev_scalar = vibboard_aux_aux.TD_acc_std_dev_scalar;
		vibboard_devices_aux[vibboard_aux_aux.device_id].TD_acc_kurtosis_scalar = vibboard_aux_aux.TD_acc_kurtosis_scalar;
		vibboard_devices_aux[vibboard_aux_aux.device_id].TD_acc_skewness_scalar = vibboard_aux_aux.TD_acc_skewness_scalar;
		vibboard_devices_aux[vibboard_aux_aux.device_id].TD_acc_rms_scalar = vibboard_aux_aux.TD_acc_rms_scalar;

		vibboard_devices_aux[vibboard_aux_aux.device_id].TD_acc_mean_x_axis = vibboard_aux_aux.TD_acc_mean_x_axis;
		vibboard_devices_aux[vibboard_aux_aux.device_id].TD_acc_std_dev_x_axis = vibboard_aux_aux.TD_acc_std_dev_x_axis;
		vibboard_devices_aux[vibboard_aux_aux.device_id].TD_acc_kurtosis_x_axis = vibboard_aux_aux.TD_acc_kurtosis_x_axis;
		vibboard_devices_aux[vibboard_aux_aux.device_id].TD_acc_skewness_x_axis = vibboard_aux_aux.TD_acc_skewness_x_axis;
		vibboard_devices_aux[vibboard_aux_aux.device_id].TD_acc_rms_x_axis = vibboard_aux_aux.TD_acc_rms_x_axis;

		vibboard_devices_aux[vibboard_aux_aux.device_id].TD_acc_mean_y_axis = vibboard_aux_aux.TD_acc_mean_y_axis;
		vibboard_devices_aux[vibboard_aux_aux.device_id].TD_acc_std_dev_y_axis = vibboard_aux_aux.TD_acc_std_dev_y_axis;
		vibboard_devices_aux[vibboard_aux_aux.device_id].TD_acc_kurtosis_y_axis = vibboard_aux_aux.TD_acc_kurtosis_y_axis;
		vibboard_devices_aux[vibboard_aux_aux.device_id].TD_acc_skewness_y_axis = vibboard_aux_aux.TD_acc_skewness_y_axis;
		vibboard_devices_aux[vibboard_aux_aux.device_id].TD_acc_rms_y_axis = vibboard_aux_aux.TD_acc_rms_y_axis;

		vibboard_devices_aux[vibboard_aux_aux.device_id].TD_acc_mean_z_axis = vibboard_aux_aux.TD_acc_mean_z_axis;
		vibboard_devices_aux[vibboard_aux_aux.device_id].TD_acc_std_dev_z_axis = vibboard_aux_aux.TD_acc_std_dev_z_axis;
		vibboard_devices_aux[vibboard_aux_aux.device_id].TD_acc_kurtosis_z_axis = vibboard_aux_aux.TD_acc_kurtosis_z_axis;
		vibboard_devices_aux[vibboard_aux_aux.device_id].TD_acc_skewness_z_axis = vibboard_aux_aux.TD_acc_skewness_z_axis;
		vibboard_devices_aux[vibboard_aux_aux.device_id].TD_acc_rms_z_axis = vibboard_aux_aux.TD_acc_rms_z_axis;

		vibboard_devices_aux[vibboard_aux_aux.device_id].FD_acc_data_peaks_freq_scalar = vibboard_aux_aux.FD_acc_data_peaks_freq_scalar;
		vibboard_devices_aux[vibboard_aux_aux.device_id].FD_acc_data_peaks_amp_scalar = vibboard_aux_aux.FD_acc_data_peaks_amp_scalar;

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
		printk("TD device state: %d ", vibboard_devices[i].TD_device_state);
		printk("TD machine state: %d ", vibboard_devices[i].TD_machine_state);
		printk("\n");
	}
}

bool MatchVibboard(struct net_buf_simple *ad)
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

	/* This demo doesn't require active scan */
	err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
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
	// fill the vibboard device struct with the data from the advertisement
	/*---------------TD-------------*/
	uint8_t TD_Features_Carac_value_index=0;
	// get the scalar of the TD features
	(*vibboard_device)->TD_acc_mean_scalar = data->data[TD_Features_Carac_value_index];
	(*vibboard_device)->TD_acc_std_dev_scalar = data->data[TD_Features_Carac_value_index++];
	(*vibboard_device)->TD_acc_kurtosis_scalar = data->data[TD_Features_Carac_value_index++];
	(*vibboard_device)->TD_acc_skewness_scalar = data->data[TD_Features_Carac_value_index++];
	(*vibboard_device)->TD_acc_rms_scalar = data->data[TD_Features_Carac_value_index++];
	// for each 2 bytes convert to integer and divide by the corresponding scalar. Order is mean, std_dev, kurtosis, skewness, rms
	for (int i = 0; i < 5; i++){
		// get the 2 bytes
		uint16_t value = data->data[TD_Features_Carac_value_index] | (data->data[TD_Features_Carac_value_index + 1] << 8);
		// divide by the scalar
		switch(i){
			case 0:
				(*vibboard_device)->TD_acc_mean_x_axis = (float)value / (*vibboard_device)->TD_acc_mean_scalar;
				break;
			case 1:
				(*vibboard_device)->TD_acc_std_dev_x_axis = (float)value / (*vibboard_device)->TD_acc_std_dev_scalar;
				break;
			case 2:
				(*vibboard_device)->TD_acc_kurtosis_x_axis = (float)value / (*vibboard_device)->TD_acc_kurtosis_scalar;
				break;
			case 3:
				(*vibboard_device)->TD_acc_skewness_x_axis = (float)value / (*vibboard_device)->TD_acc_skewness_scalar;
				break;
			case 4:
				(*vibboard_device)->TD_acc_rms_x_axis = (float)value / (*vibboard_device)->TD_acc_rms_scalar;
				break;
		}
		TD_Features_Carac_value_index += 2;
	}
	// repeat for y and z axis
	for (int i = 0; i < 5; i++){
		uint16_t value = data->data[TD_Features_Carac_value_index] | (data->data[TD_Features_Carac_value_index + 1] << 8);
		switch(i){
			case 0:
				(*vibboard_device)->TD_acc_mean_y_axis = (float)value / (*vibboard_device)->TD_acc_mean_scalar;
				break;
			case 1:
				(*vibboard_device)->TD_acc_std_dev_y_axis = (float)value / (*vibboard_device)->TD_acc_std_dev_scalar;
				break;
			case 2:
				(*vibboard_device)->TD_acc_kurtosis_y_axis = (float)value / (*vibboard_device)->TD_acc_kurtosis_scalar;
				break;
			case 3:
				(*vibboard_device)->TD_acc_skewness_y_axis = (float)value / (*vibboard_device)->TD_acc_skewness_scalar;
				break;
			case 4:
				(*vibboard_device)->TD_acc_rms_y_axis = (float)value / (*vibboard_device)->TD_acc_rms_scalar;
				break;
		}
		TD_Features_Carac_value_index += 2;
	}
	for (int i = 0; i < 5; i++){
		uint16_t value = data->data[TD_Features_Carac_value_index] | (data->data[TD_Features_Carac_value_index + 1] << 8);
		switch(i){
			case 0:
				(*vibboard_device)->TD_acc_mean_z_axis = (float)value / (*vibboard_device)->TD_acc_mean_scalar;
				break;
			case 1:
				(*vibboard_device)->TD_acc_std_dev_z_axis = (float)value / (*vibboard_device)->TD_acc_std_dev_scalar;
				break;
			case 2:
				(*vibboard_device)->TD_acc_kurtosis_z_axis = (float)value / (*vibboard_device)->TD_acc_kurtosis_scalar;
				break;
			case 3:
				(*vibboard_device)->TD_acc_skewness_z_axis = (float)value / (*vibboard_device)->TD_acc_skewness_scalar;
				break;
			case 4:
				(*vibboard_device)->TD_acc_rms_z_axis = (float)value / (*vibboard_device)->TD_acc_rms_scalar;
				break;
		}
		TD_Features_Carac_value_index += 2;
	}
	/*---------------FD-------------*/
	uint8_t FD_Features_Carac_value_index=TD_Features_Carac_value_index;
	// get the scalars for the peaks
	(*vibboard_device)->FD_acc_data_peaks_freq_scalar = data->data[TD_Features_Carac_value_index + 2];
	(*vibboard_device)->FD_acc_data_peaks_amp_scalar = data->data[TD_Features_Carac_value_index + 3];
	// get the peaks
	// x axis
	for (int i = 0; i < FD_NR_PEAKS; i++){
		(*vibboard_device)->FD_acc_data_peaks_freq_x_axis[i] = data->data[FD_Features_Carac_value_index] | (data->data[FD_Features_Carac_value_index + 1] << 8);
		FD_Features_Carac_value_index += 2;
	}
	for (int i = 0; i < FD_NR_PEAKS; i++){
		(*vibboard_device)->FD_acc_data_peaks_amps_x_axis[i] = data->data[FD_Features_Carac_value_index] | (data->data[FD_Features_Carac_value_index + 1] << 8);
		FD_Features_Carac_value_index += 2;
	}
	// y axis
	for (int i = 0; i < FD_NR_PEAKS; i++){
		(*vibboard_device)->FD_acc_data_peaks_freq_y_axis[i] = data->data[FD_Features_Carac_value_index] | (data->data[FD_Features_Carac_value_index + 1] << 8);
		FD_Features_Carac_value_index += 2;
	}
	for (int i = 0; i < FD_NR_PEAKS; i++){
		(*vibboard_device)->FD_acc_data_peaks_amps_y_axis[i] = data->data[FD_Features_Carac_value_index] | (data->data[FD_Features_Carac_value_index + 1] << 8);
		FD_Features_Carac_value_index += 2;
	}
	// z axis
	for (int i = 0; i < FD_NR_PEAKS; i++){
		(*vibboard_device)->FD_acc_data_peaks_freq_z_axis[i] = data->data[FD_Features_Carac_value_index] | (data->data[FD_Features_Carac_value_index + 1] << 8);
		FD_Features_Carac_value_index += 2;
	}
	for (int i = 0; i < FD_NR_PEAKS; i++){
		(*vibboard_device)->FD_acc_data_peaks_amps_z_axis[i] = data->data[FD_Features_Carac_value_index] | (data->data[FD_Features_Carac_value_index + 1] << 8);
		FD_Features_Carac_value_index += 2;
	}
}