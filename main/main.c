/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <time.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>

#include "lwip/err.h"
#include "lwip/sys.h"
/* BLE */
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "ble_spp_server.h"
#include "driver/uart.h"

static const char *tag = "NimBLE_SPP_BLE_PRPH";
static int ble_spp_server_gap_event(struct ble_gap_event *event, void *arg);
static uint8_t own_addr_type;
int gatt_svr_register(void);
QueueHandle_t spp_common_uart_queue = NULL;
static bool is_connect = false;
uint16_t connection_handle;
static uint16_t ble_svc_gatt_read_val_handle, ble_spp_svc_gatt_read_val_handle;
char ssid[32], pswd[32];

#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static int s_retry_num = 0;

static void
event_handler(void *arg,
              esp_event_base_t event_base,
              int32_t event_id,
              void *event_data)
{
	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
		esp_wifi_connect();
	} else if (event_base == WIFI_EVENT &&
	           event_id == WIFI_EVENT_STA_DISCONNECTED) {
		if (s_retry_num < 5) {
			esp_wifi_connect();
			s_retry_num++;
			ESP_LOGI(tag, "retry to connect to the AP");
		} else {
			xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
		}
		ESP_LOGI(tag, "connect to the AP fail");
	} else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
		ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
		ESP_LOGI(tag, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
		s_retry_num = 0;
		xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
	}
}

void
wifi_init_sta(char *ssid, char *pswd)
{

	esp_event_handler_instance_t instance_any_id;
	esp_event_handler_instance_t instance_got_ip;
	ESP_ERROR_CHECK(esp_event_handler_instance_register(
	    WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
	ESP_ERROR_CHECK(esp_event_handler_instance_register(
	    IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

	wifi_config_t wifi_config = {
	    .sta =
	        {
	            /* Setting a password implies station will connect to all security modes including WEP/WPA.
				 * However these modes are deprecated and not advisable to be used. Incase your Access point
				 * doesn't support WPA2, these mode can be enabled by commenting below line */
	            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
	        },
	};

	strncpy((char *)wifi_config.sta.ssid, ssid, 32);
	strncpy((char *)wifi_config.sta.password, pswd, 32);
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());

	ESP_LOGI(tag, "wifi_init_sta finished.");

	/* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
	EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
	                                       WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
	                                       pdFALSE,
	                                       pdFALSE,
	                                       portMAX_DELAY);

	/* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
	if (bits & WIFI_CONNECTED_BIT) {
		ESP_LOGI(tag, "connected to ap SSID:%s password:%s", ssid, pswd);
	} else if (bits & WIFI_FAIL_BIT) {
		ESP_LOGI(tag,
		         "Failed to connect to SSID:%s, password:%s",
		         (char *)wifi_config.sta.ssid,
		         (char *)wifi_config.sta.password);
		s_retry_num = 0;
		strncpy((char *)wifi_config.sta.ssid, "", 32);
		strncpy((char *)wifi_config.sta.password, "", 32);
	} else {
		ESP_LOGE(tag, "UNEXPECTED EVENT");
	}
}
/*
   mapping function from Arduino

   Syntax: map(value, fromLow, fromHigh, toLow, toHigh)

   value: the number to map.
   fromLow: the lower bound of the value’s current range.
   fromHigh: the upper bound of the value’s current range.
   toLow: the lower bound of the value’s target range.
   toHigh: the upper bound of the value’s target range.
 */
long
map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* 16 Bit Alert Notification Service UUID */
#define BLE_SVC_ANS_UUID16 0x1811

/* 16 Bit Alert Notification Service Characteristic UUIDs */
#define BLE_SVC_ANS_CHR_UUID16_SUP_NEW_ALERT_CAT 0x2a47

/* 16 Bit SPP Service UUID */
#define BLE_SVC_SPP_UUID16 0xABF0

/* 16 Bit SPP Service Characteristic UUID */
#define BLE_SVC_SPP_CHR_UUID16 0xABF1

void ble_store_config_init(void);

// Queue to receive incomming messages
xQueueHandle parser_queue;

/**
 * Logs information about a connection to the console.
 */
static void
ble_spp_server_print_conn_desc(struct ble_gap_conn_desc *desc)
{
	MODLOG_DFLT(INFO,
	            "handle=%d our_ota_addr_type=%d our_ota_addr=",
	            desc->conn_handle,
	            desc->our_ota_addr.type);
	print_addr(desc->our_ota_addr.val);
	MODLOG_DFLT(
	    INFO, " our_id_addr_type=%d our_id_addr=", desc->our_id_addr.type);
	print_addr(desc->our_id_addr.val);
	MODLOG_DFLT(INFO,
	            " peer_ota_addr_type=%d peer_ota_addr=",
	            desc->peer_ota_addr.type);
	print_addr(desc->peer_ota_addr.val);
	MODLOG_DFLT(
	    INFO, " peer_id_addr_type=%d peer_id_addr=", desc->peer_id_addr.type);
	print_addr(desc->peer_id_addr.val);
	MODLOG_DFLT(INFO,
	            " conn_itvl=%d conn_latency=%d supervision_timeout=%d "
	            "encrypted=%d authenticated=%d bonded=%d\n",
	            desc->conn_itvl,
	            desc->conn_latency,
	            desc->supervision_timeout,
	            desc->sec_state.encrypted,
	            desc->sec_state.authenticated,
	            desc->sec_state.bonded);
}

/**
 * Enables advertising with the following parameters:
 *     o General discoverable mode.
 *     o Undirected connectable mode.
 */
static void
ble_spp_server_advertise(void)
{
	struct ble_gap_adv_params adv_params;
	struct ble_hs_adv_fields fields;
	const char *name;
	int rc;

	/**
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info).
     *     o Advertising tx power.
     *     o Device name.
     *     o 16-bit service UUIDs (alert notifications).
     */

	memset(&fields, 0, sizeof fields);

	/* Advertise two flags:
     *     o Discoverability in forthcoming advertisement (general)
     *     o BLE-only (BR/EDR unsupported).
     */
	fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

	/* Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assigning the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
	fields.tx_pwr_lvl_is_present = 1;
	fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

	name = ble_svc_gap_device_name();
	fields.name = (uint8_t *)name;
	fields.name_len = strlen(name);
	fields.name_is_complete = 1;

	fields.uuids16 = (ble_uuid16_t[]){BLE_UUID16_INIT(GATT_SVR_SVC_ALERT_UUID)};
	fields.num_uuids16 = 1;
	fields.uuids16_is_complete = 1;

	rc = ble_gap_adv_set_fields(&fields);
	if (rc != 0) {
		MODLOG_DFLT(ERROR, "error setting advertisement data; rc=%d\n", rc);
		return;
	}

	/* Begin advertising. */
	memset(&adv_params, 0, sizeof adv_params);
	adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
	adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
	rc = ble_gap_adv_start(own_addr_type,
	                       NULL,
	                       BLE_HS_FOREVER,
	                       &adv_params,
	                       ble_spp_server_gap_event,
	                       NULL);
	if (rc != 0) {
		MODLOG_DFLT(ERROR, "error enabling advertisement; rc=%d\n", rc);
		return;
	}
}

/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that forms.
 * ble_spp_server uses the same callback for all connections.
 *
 * @param event                 The type of event being signalled.
 * @param ctxt                  Various information pertaining to the event.
 * @param arg                   Application-specified argument; unused by
 *                                  ble_spp_server.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular GAP event being signalled.
 */
static int
ble_spp_server_gap_event(struct ble_gap_event *event, void *arg)
{
	struct ble_gap_conn_desc desc;
	int rc;

	switch (event->type) {
	case BLE_GAP_EVENT_CONNECT:
		/* A new connection was established or a connection attempt failed. */
		MODLOG_DFLT(INFO,
		            "connection %s; status=%d ",
		            event->connect.status == 0 ? "established" : "failed",
		            event->connect.status);
		if (event->connect.status == 0) {
			rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
			assert(rc == 0);
			ble_spp_server_print_conn_desc(&desc);
			is_connect = true;
			connection_handle = event->connect.conn_handle;
		}
		MODLOG_DFLT(INFO, "\n");
		if (event->connect.status != 0) {
			/* Connection failed; resume advertising. */
			ble_spp_server_advertise();
		}
		return 0;

	case BLE_GAP_EVENT_DISCONNECT:
		MODLOG_DFLT(INFO, "disconnect; reason=%d ", event->disconnect.reason);
		ble_spp_server_print_conn_desc(&event->disconnect.conn);
		MODLOG_DFLT(INFO, "\n");

		/* Connection terminated; resume advertising. */
		ble_spp_server_advertise();
		return 0;

	case BLE_GAP_EVENT_CONN_UPDATE:
		/* The central has updated the connection parameters. */
		MODLOG_DFLT(
		    INFO, "connection updated; status=%d ", event->conn_update.status);
		rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
		assert(rc == 0);
		ble_spp_server_print_conn_desc(&desc);
		MODLOG_DFLT(INFO, "\n");
		return 0;

	case BLE_GAP_EVENT_ADV_COMPLETE:
		MODLOG_DFLT(
		    INFO, "advertise complete; reason=%d", event->adv_complete.reason);
		ble_spp_server_advertise();
		return 0;

	case BLE_GAP_EVENT_MTU:
		MODLOG_DFLT(INFO,
		            "mtu update event; conn_handle=%d cid=%d mtu=%d\n",
		            event->mtu.conn_handle,
		            event->mtu.channel_id,
		            event->mtu.value);
		return 0;

	default:
		return 0;
	}
}

static void
ble_spp_server_on_reset(int reason)
{
	MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

static void
ble_spp_server_on_sync(void)
{
	int rc;

	rc = ble_hs_util_ensure_addr(0);
	assert(rc == 0);

	/* Figure out address to use while advertising (no privacy for now) */
	rc = ble_hs_id_infer_auto(0, &own_addr_type);
	if (rc != 0) {
		MODLOG_DFLT(ERROR, "error determining address type; rc=%d\n", rc);
		return;
	}

	/* Printing ADDR */
	uint8_t addr_val[6] = {0};
	rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);

	MODLOG_DFLT(INFO, "Device Address: ");
	print_addr(addr_val);
	MODLOG_DFLT(INFO, "\n");
	/* Begin advertising. */
	ble_spp_server_advertise();
}

void
ble_spp_server_host_task(void *param)
{
	ESP_LOGI(tag, "BLE Host Task Started");
	/* This function will return only when nimble_port_stop() is executed */
	nimble_port_run();

	nimble_port_freertos_deinit();
}

/* Callback function for custom service */
static int
ble_svc_gatt_handler(uint16_t conn_handle,
                     uint16_t attr_handle,
                     struct ble_gatt_access_ctxt *ctxt,
                     void *arg)
{
	switch (ctxt->op) {
	case BLE_GATT_ACCESS_OP_READ_CHR:
		ESP_LOGI(tag, "Callback for read");
		break;

	case BLE_GATT_ACCESS_OP_WRITE_CHR: {
		uint8_t msg[20];
		int rc = ble_hs_mbuf_to_flat(ctxt->om, msg, 20, NULL);
		if (rc == 0)
			ESP_LOGI(tag, "Read successfully");
		else
			ESP_LOGI(tag, "Could not read");
		msg[ctxt->om->om_len - 2] = '\0';
		xQueueSendToBack(parser_queue, msg, 1000 / portTICK_PERIOD_MS);
	} break;

	default:
		ESP_LOGI(tag, "\nDefault Callback");
		break;
	}
	return 0;
}

/* Define new custom service */
static const struct ble_gatt_svc_def new_ble_svc_gatt_defs[] = {
    {
        /*** Service: GATT */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_ANS_UUID16),
        .characteristics =
            (struct ble_gatt_chr_def[]){
                {
                    /* Support new alert category */
                    .uuid = BLE_UUID16_DECLARE(
                        BLE_SVC_ANS_CHR_UUID16_SUP_NEW_ALERT_CAT),
                    .access_cb = ble_svc_gatt_handler,
                    .val_handle = &ble_svc_gatt_read_val_handle,
                    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                             BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
                },
                {
                    0, /* No more characteristics */
                }},
    },
    {
        /*** Service: SPP */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_SPP_UUID16),
        .characteristics =
            (struct ble_gatt_chr_def[]){
                {
                    /* Support SPP service */
                    .uuid = BLE_UUID16_DECLARE(BLE_SVC_SPP_CHR_UUID16),
                    .access_cb = ble_svc_gatt_handler,
                    .val_handle = &ble_spp_svc_gatt_read_val_handle,
                    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                             BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
                },
                {
                    0, /* No more characteristics */
                }},
    },
    {
        0, /* No more services. */
    },
};

int
gatt_svr_register(void)
{
	int rc = 0;

	rc = ble_gatts_count_cfg(new_ble_svc_gatt_defs);

	if (rc != 0) {
		return rc;
	}

	rc = ble_gatts_add_svcs(new_ble_svc_gatt_defs);
	if (rc != 0) {
		return rc;
	}

	return 0;
}

void
parser_task(void *args)
{
	double x = 0.0;
	double y = 1.0;
	srand(time(NULL));

	// Guarenateed keep x1 between x and y.
	uint8_t rx_msg[20];
	for (;;) {
		if (xQueueReceive(parser_queue, rx_msg, 6000 / portTICK_PERIOD_MS) !=
		    pdTRUE)
			continue;
		if (strstr((const char *)rx_msg, "freq") != NULL) {
			ESP_LOGI(tag, "Got frequency: %s", (const char *)rx_msg);
		}
		if (strstr((const char *)rx_msg, "mag") != NULL) {
			ESP_LOGI(tag, "Got maginute: %s", (const char *)rx_msg);
		}
		if (strcmp((const char *)rx_msg, "rprov") == 0) {
			ESP_LOGI(tag, "Resetting provision");
		}
		if (strcmp((const char *)rx_msg, "sos") == 0) {
			ESP_LOGI(tag, "Starting sos");
		}
		if (strcmp((const char *)rx_msg, "rstfs") == 0) {
			ESP_LOGI(tag, "Resetting factory app for server");
		}
		if (strcmp((const char *)rx_msg, "rstfc") == 0) {
			ESP_LOGI(tag, "Resetting factory app for client");
		}
		if (strcmp((const char *)rx_msg, "upl") == 0) {
			ESP_LOGI(tag, "Begining upload");
		}
		if (strcmp((const char *)rx_msg, "ota") == 0) {
			ESP_LOGI(tag, "Begining ota");
		}
		if (strstr((const char *)rx_msg, "ssid") != NULL) {
			char temp_message[5];
			sscanf((const char *)rx_msg, "%s %s", temp_message, ssid);
			ESP_LOGI(tag, "Got SSID: %s", ssid);
		}
		if (strstr((const char *)rx_msg, "pswd") != NULL) {
			char temp_message[5];
			sscanf((const char *)rx_msg, "%s %s", temp_message, pswd);
			ESP_LOGI(tag, "Got Password: %s", pswd);
		}
		if (strcmp((const char *)rx_msg, "wifi") == 0) {
			ESP_LOGI(tag, "ESP_WIFI_MODE_STA");
			wifi_init_sta(ssid, pswd);
		}
		if (strcmp((const char *)rx_msg, "info") == 0) {
			char *message = malloc(sizeof(char) * 1000);
			if (message == NULL)
				ESP_LOGE("BLE SPP", "Malloc failed, system out of memory");
			sprintf(message, "%s %d\n%s %d\n", "sos s", 0, "mag s", 2);
			printf("%s\n", message);

			/* Server messages for Info command*/

			struct os_mbuf *om_info = NULL;
			om_info = ble_hs_mbuf_from_flat(message, strlen(message));
			int rc = ble_gattc_notify_custom(
			    connection_handle, ble_spp_svc_gatt_read_val_handle, om_info);
			if (rc == 0)
				ESP_LOGI(tag, "Notification sent successfully");
			else
				ESP_LOGE(tag, "Notification not sent successfully");

			vTaskDelay(2000 / portTICK_PERIOD_MS);

			sprintf(message, "%s %0.2f\n%s %d\n", "freq s", 0.7, "pair s", 0);
			printf("%s\n", message);

			om_info = NULL;
			om_info = ble_hs_mbuf_from_flat(message, strlen(message));
			rc = ble_gattc_notify_custom(
			    connection_handle, ble_spp_svc_gatt_read_val_handle, om_info);
			if (rc == 0)
				ESP_LOGI(tag, "Notification sent successfully");
			else
				ESP_LOGE(tag, "Notification not sent successfully");

			vTaskDelay(2000 / portTICK_PERIOD_MS);

			sprintf(message, "%s %d\n", "charger con s", 0);
			printf("%s\n", message);

			om_info = NULL;
			om_info = ble_hs_mbuf_from_flat(message, strlen(message));
			rc = ble_gattc_notify_custom(
			    connection_handle, ble_spp_svc_gatt_read_val_handle, om_info);
			if (rc == 0)
				ESP_LOGI(tag, "Notification sent successfully");
			else
				ESP_LOGE(tag, "Notification not sent successfully");

			vTaskDelay(2000 / portTICK_PERIOD_MS);

			double x1 = x + rand() * (y - x) / RAND_MAX;
			sprintf(message, "%s %0.1f\n", "batt s", x1);
			printf("%s\n", message);

			om_info = NULL;
			om_info = ble_hs_mbuf_from_flat(message, strlen(message));
			rc = ble_gattc_notify_custom(
			    connection_handle, ble_spp_svc_gatt_read_val_handle, om_info);
			if (rc == 0)
				ESP_LOGI(tag, "Notification sent successfully");
			else
				ESP_LOGE(tag, "Notification not sent successfully");

			x1 = x + rand() * (8.0 - 0.5) / RAND_MAX;
			sprintf(message, "%s %0.2f\n", "remaining space s", x1);
			printf("%s\n", message);

			om_info = NULL;
			om_info = ble_hs_mbuf_from_flat(message, strlen(message));
			rc = ble_gattc_notify_custom(
			    connection_handle, ble_spp_svc_gatt_read_val_handle, om_info);
			if (rc == 0)
				ESP_LOGI(tag, "Notification sent successfully");
			else
				ESP_LOGE(tag, "Notification not sent successfully");

			vTaskDelay(2000 / portTICK_PERIOD_MS);

			/* Client messages for Info command*/

			sprintf(message, "%s %d\n%s %d\n", "sos c", 0, "mag c", 2);
			printf("%s\n", message);

			vTaskDelay(2000 / portTICK_PERIOD_MS);

			sprintf(message, "%s %0.2f\n%s %d\n", "freq c", 0.7, "pair c", 0);
			printf("%s\n", message);

			om_info = NULL;
			om_info = ble_hs_mbuf_from_flat(message, strlen(message));
			rc = ble_gattc_notify_custom(
			    connection_handle, ble_spp_svc_gatt_read_val_handle, om_info);
			if (rc == 0)
				ESP_LOGI(tag, "Notification sent successfully");
			else
				ESP_LOGE(tag, "Notification not sent successfully");

			vTaskDelay(2000 / portTICK_PERIOD_MS);

			sprintf(message, "%s %d\n", "charger con c", 0);
			printf("%s\n", message);

			om_info = NULL;
			om_info = ble_hs_mbuf_from_flat(message, strlen(message));
			rc = ble_gattc_notify_custom(
			    connection_handle, ble_spp_svc_gatt_read_val_handle, om_info);
			if (rc == 0)
				ESP_LOGI(tag, "Notification sent successfully");
			else
				ESP_LOGE(tag, "Notification not sent successfully");

			vTaskDelay(2000 / portTICK_PERIOD_MS);

			x1 = x + rand() * (y - x) / RAND_MAX;
			sprintf(message, "%s %0.1f\n", "batt c", x1);
			printf("%s\n", message);

			vTaskDelay(2000 / portTICK_PERIOD_MS);

			om_info = NULL;
			om_info = ble_hs_mbuf_from_flat(message, strlen(message));
			rc = ble_gattc_notify_custom(
			    connection_handle, ble_spp_svc_gatt_read_val_handle, om_info);
			if (rc == 0)
				ESP_LOGI(tag, "Notification sent successfully");
			else
				ESP_LOGE(tag, "Notification not sent successfully");

			x1 = x + rand() * (8.0 - 0.5) / RAND_MAX;
			sprintf(message, "%s %0.2f\n", "remaining space c", x1);
			printf("%s\n", message);

			om_info = NULL;
			om_info = ble_hs_mbuf_from_flat(message, strlen(message));
			rc = ble_gattc_notify_custom(
			    connection_handle, ble_spp_svc_gatt_read_val_handle, om_info);
			if (rc == 0)
				ESP_LOGI(tag, "Notification sent successfully");
			else
				ESP_LOGE(tag, "Notification not sent successfully");
		}
	}

	if (uxQueueMessagesWaiting(parser_queue) == 0)
		vTaskDelay(100 / portTICK_PERIOD_MS);
}

void
app_main(void)
{
	int rc;

	/* Initialize NVS — it is used to store PHY calibration data */
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
	    ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	s_wifi_event_group = xEventGroupCreate();

	ESP_ERROR_CHECK(esp_netif_init());

	ESP_ERROR_CHECK(esp_event_loop_create_default());
	esp_netif_create_default_wifi_sta();

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));

	ESP_ERROR_CHECK(esp_nimble_hci_and_controller_init());

	nimble_port_init();

	/* Initialize the NimBLE host configuration. */
	ble_hs_cfg.reset_cb = ble_spp_server_on_reset;
	ble_hs_cfg.sync_cb = ble_spp_server_on_sync;
	ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb_spp;
	ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

	ble_hs_cfg.sm_io_cap = CONFIG_EXAMPLE_IO_TYPE;
#ifdef CONFIG_EXAMPLE_BONDING
	ble_hs_cfg.sm_bonding = 1;
#endif
#ifdef CONFIG_EXAMPLE_MITM
	ble_hs_cfg.sm_mitm = 1;
#endif
#ifdef CONFIG_EXAMPLE_USE_SC
	ble_hs_cfg.sm_sc = 1;
#else
	ble_hs_cfg.sm_sc = 0;
#endif
#ifdef CONFIG_EXAMPLE_BONDING
	ble_hs_cfg.sm_our_key_dist = 1;
	ble_hs_cfg.sm_their_key_dist = 1;
#endif

	rc = new_gatt_svr_init();
	assert(rc == 0);

	/* Register custom service */
	rc = gatt_svr_register();
	assert(rc == 0);

	/* Set the default device name. */
	rc = ble_svc_gap_device_name_set("nimble-ble-spp-svr");
	assert(rc == 0);

	/* XXX Need to have template for store */
	ble_store_config_init();

	nimble_port_freertos_init(ble_spp_server_host_task);

	parser_queue = xQueueCreate(10, sizeof(uint8_t) * 20);
	xTaskCreate(parser_task, "parser task", 1024 * 4, NULL, 4, NULL);
}
