/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "vibboard.h"

#include <zephyr/types.h>
#include <stddef.h>
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

#define NAME_LEN 30

static bool FindVibboardAdvertisement(AdHandle_t *pHandle);

static struct bt_conn *default_conn;

static bool data_cb(struct bt_data *data, void *user_data){

	char *name = user_data;

	switch(data->type){
		case BT_DATA_NAME_SHORTENED:
		case BT_DATA_NAME_COMPLETE:
			memcpy(name, data->data, MIN(data->data_len, NAME_LEN - 1));
			return false;
		default:
			return false;
	}
}

static void device_found(const bt_addr_le_t *addr, s8_t rssi, u8_t type,
			 struct net_buf_simple *ad)
{
	char addr_str[BT_ADDR_LE_STR_LEN];
	char name[30];
	int err;

	if (default_conn) {
		return;
	}

	if (MatchVibboard(ad)) {
		printk("Correct device found");
	}else{
		return;
	}

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
	bt_data_parse(ad, data_cb, name);
	printk("Device name: %s \n", name);

	/* connect only to devices in close proximity */
	if (rssi < -70) {
		return;
	}

	// if (bt_le_scan_stop()) {
	// 	return;
	// }

	// err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
	// 			BT_LE_CONN_PARAM_DEFAULT, &default_conn);
	// if (err) {
	// 	printk("Create conn to %s failed (%u)\n", addr_str, err);
	// 	start_scan();
	// }
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

void start_scan(void)
{
	int err;

	/* This demo doesn't require active scan */
	err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
		return;
	}

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
