#include <stdio.h>
#include "nvs_flash.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "services/gap/ble_svc_gap.h"

static const char *TAG = "BLE Server";

static int ble_server_gap_event(struct ble_gap_event *event, void *arg);
static int motor_state = 0;

static uint8_t own_addr_type;
bool leg_client_connected = false, phone_connected = false;
bool client_control = false;
int state = -1, phone_subscription = 0;
uint16_t esp_handle,                        // esp's connection handle
    phone_handle = 0,                       // phone's connection handle
    ble_svc_gatt_read_val_handle,           // gatt profiles svc read value handle
    ble_spp_svc_gatt_read_val_handle,       // spp profiles svc read value handle
    ble_cl2sr_svc_gatt_read_val_handle,     // second spp profiles svc read value hndl
    ble_calc_freq_svc_gatt_read_val_handle, // calc frequency profiles svc read val hndl
    ble_recv_freq_svc_gatt_read_val_handle, // recv frequency profiles svc read val hndl
    ble_time_svc_gatt_read_val_handle,      // time profiles svc read value handle
    ble_walk_chr_svc_gatt_read_val_handle,  // walk charac. profiles svc read val hndl
    ble_rec_chr_svc_gatt_read_val_handle;   // recv charc. profiles svc read val hndl

xQueueHandle parser_queue; // Queue to receive incoming messages

/*
 * 16 Bit WALK UUIDs for SPP and ESP-ESP services
 * - 16 Bit SPP Service UUID
 * - 16 Bit SPP Service Characteristic UUID
 * - 16 Bit SPP Service UUID
 * - 16 Bit SPP Service Characteristic UUID
 */
#define BLE_SVC_SPP_UUID16 0xABF0
#define BLE_SVC_SPP_CHR_UUID16 0xABF1
#define BLE_SVC_CL2SR_UUID16 0xACF0
#define BLE_SVC_CL2SR_CHR_UUID16 0xACF1

void ble_store_config_init(void);

/**
 * Stops ble and deinitialized NimBLE Stack
 */
void
stop_ble_prph()
{
	/*
	 *ble_gap_adv_stop();
	 *vTaskDelay(1000 / portTICK_PERIOD_MS);
	 */
	esp_err_t ret = nimble_port_stop();
	if (ret == ESP_OK) {
		nimble_port_deinit();

		ret = esp_nimble_hci_and_controller_deinit();
		if (ret != ESP_OK) {
			ESP_LOGE(TAG, "esp_nimble_hci_and_controller_deinit() failed with error: %d", ret);
		}
	}
}

esp_err_t
ble_notify_client(uint16_t connection_handle, uint16_t chr_val_handle, const char *msg)
{
	struct os_mbuf *txom = NULL;
	txom = ble_hs_mbuf_from_flat(msg, strlen(msg));
	return ble_gattc_notify_custom(connection_handle, chr_val_handle, txom);
}

/**
 * Enables advertising with the following parameters:
 *     o General discoverable mode.
 *     o Undirected connectable mode.
 */
static void
ble_server_advertise()
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

	fields.uuids16 = (ble_uuid16_t[]){BLE_UUID16_INIT(BLE_SVC_CL2SR_UUID16)};
	fields.num_uuids16 = 1;
	fields.uuids16_is_complete = 1;

	rc = ble_gap_adv_set_fields(&fields);
	if (rc != 0) {
		ESP_LOGE(TAG, "error setting advertisement data; rc=%d\n", rc);
		return;
	}

	/* Begin advertising. */
	memset(&adv_params, 0, sizeof adv_params);
	adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
	adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
	rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_server_gap_event, NULL);
	if (rc != 0) {
		ESP_LOGE(TAG, "error enabling advertisement; rc=%d\n", rc);
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
ble_server_gap_event(struct ble_gap_event *event, void *arg)
{
	struct ble_gap_conn_desc desc;
	int rc;

	switch (event->type) {
	case BLE_GAP_EVENT_CONNECT:
		/* A new connection was established or a connection attempt failed. */
		if (event->connect.status == 0) {
			rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
			assert(rc == 0);

			ble_server_advertise();
		}

		if (event->connect.status != 0) {
			/* Connection failed; resume advertising. */
			ble_server_advertise();
		}
		return 0;

	case BLE_GAP_EVENT_SUBSCRIBE:
		ESP_LOGI(TAG, "Subscribe event");

		if (event->subscribe.attr_handle == ble_spp_svc_gatt_read_val_handle) {
			ESP_LOGI(TAG, "Phone Subscription");
			phone_connected = !phone_connected;
			printf("Phone connection handle %d\n", event->subscribe.conn_handle);
			phone_handle = event->subscribe.conn_handle;
		} else if (event->subscribe.attr_handle == ble_cl2sr_svc_gatt_read_val_handle) {
		}

		return 0;

	case BLE_GAP_EVENT_DISCONNECT:
		if (!phone_connected) {
			ESP_LOGI(TAG, "Phone disconnected");
		}
		return 0;

	case BLE_GAP_EVENT_CONN_UPDATE:
		/* The central has updated the connection parameters. */
		rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
		assert(rc == 0);
		return 0;

	case BLE_GAP_EVENT_ADV_COMPLETE:
		ble_server_advertise();
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
ble_server_on_reset(int reason)
{
	MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

static void
ble_server_on_sync(void)
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

	/* Begin advertising. */
	ble_server_advertise();
}

void
ble_server_host_task(void *param)
{
	ESP_LOGI(TAG, "BLE Host Task Started");
	/* This function will return only when nimble_port_stop() is executed */
	nimble_port_run();

	nimble_port_freertos_deinit();
}

/* Callback function for server service */
static int
ble_server_svc_gatt_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
	switch (ctxt->op) {
	case BLE_GATT_ACCESS_OP_READ_CHR:
		ESP_LOGI(TAG, "Callback for read");
		break;

	case BLE_GATT_ACCESS_OP_WRITE_CHR:
		ESP_LOGI(TAG, "Data received in write event,conn_handle = %x,attr_handle = %x", conn_handle, attr_handle);
		break;

	default:
		ESP_LOGI(TAG, "Default Callback");
		break;
	}
	return 0;
}

/* Callback function for spp service */
static int
ble_spp_svc_gatt_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
	switch (ctxt->op) {
	case BLE_GATT_ACCESS_OP_READ_CHR:
		ESP_LOGI(TAG, "Callback for read");
		break;

	case BLE_GATT_ACCESS_OP_WRITE_CHR: {
		uint8_t msg[50];
		int rc = ble_hs_mbuf_to_flat(ctxt->om, msg, 50, NULL);
		if (rc == 0)
			ESP_LOGI(TAG, "Read successfully");
		else
			ESP_LOGI(TAG, "Could not read");
		msg[ctxt->om->om_len - 2] = '\0';
		ESP_LOGI(TAG, "Raw message: %s", msg);
		xQueueSendToBack(parser_queue, msg, 1000 / portTICK_PERIOD_MS);
	} break;

	default:
		ESP_LOGI(TAG, "\nDefault Callback");
		break;
	}
	return 0;
}

static const struct ble_gatt_svc_def walk_ble_svc_gatt_defs[] = {
    {
        /*** Service: SPP */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_SPP_UUID16),
        .characteristics = (struct ble_gatt_chr_def[]){{
                                                           .uuid = BLE_UUID16_DECLARE(BLE_SVC_SPP_CHR_UUID16),
                                                           .access_cb = ble_spp_svc_gatt_handler,
                                                           .val_handle = &ble_spp_svc_gatt_read_val_handle,
                                                           .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                                                                    BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
                                                       },
                                                       {
                                                           0, /* No more characteristics */
                                                       }},
    },
    {
        0,
    }};

float
float_rand(float min, float max)
{
	float scale = rand() / (float)RAND_MAX; /* [0, 1.0] */
	return min + scale * (max - min);       /* [min, max] */
}

void
parser_task(void *args)
{
	uint8_t msg[50];   // incoming messages
	char x[30], y[30]; // incoming messages first and second part
	esp_err_t rc;      // Return code
	char *ssid, *pswd; // WiFi SSID and Password
	float manual_freq;
	int vib_strength;

	for (;;) {
		if (xQueueReceive(parser_queue, msg, 6000 / portTICK_PERIOD_MS) != pdTRUE)
			continue;

		// Show command received from phone
		ESP_LOGI(TAG, "\nCommand from phone: %s\n", (const char *)msg);

		if ((strcmp((const char *)msg, "info") == 0)) {

			char *message = malloc(sizeof(char *) * 100);
			if (message == NULL)
				ESP_LOGE("BLE SPP", "Malloc failed, system out of memory");

			if (message != NULL)
				sprintf(message, "%s %s\n", "tag s", "abcxyz");

			ESP_LOGI(TAG, "%s\n", message);

			struct os_mbuf *om_info = NULL;
			om_info = ble_hs_mbuf_from_flat(message, strlen(message));
			int rc = ble_gattc_notify_custom(phone_handle, ble_spp_svc_gatt_read_val_handle, om_info);
			if (rc == 0)
				ESP_LOGI(TAG, "Notification sent successfully");
			else
				ESP_LOGE(TAG, "Notification not sent successfully");

			vTaskDelay(2000 / portTICK_PERIOD_MS);

			if (message != NULL)
				sprintf(message, "%s %d\n%s %d\n", "sos s", rand() % (1 - 0 + 1), "mag s", rand() % (4 - 0 + 1));

			ESP_LOGI(TAG, "%s\n", message);

			om_info = ble_hs_mbuf_from_flat(message, strlen(message));
			rc = ble_gattc_notify_custom(phone_handle, ble_spp_svc_gatt_read_val_handle, om_info);
			if (rc == 0)
				ESP_LOGI(TAG, "Notification sent successfully");
			else
				ESP_LOGE(TAG, "Notification not sent successfully");

			vTaskDelay(2000 / portTICK_PERIOD_MS);

			if (message != NULL)
				sprintf(message, "%s %0.2f\n%s %d\n", "freq s", float_rand(0.0, 1.2), "pair s", rand() % (1 - 0 + 1));

			ESP_LOGI(TAG, "%s\n", message);

			om_info = NULL;
			om_info = ble_hs_mbuf_from_flat(message, strlen(message));
			rc = ble_gattc_notify_custom(phone_handle, ble_spp_svc_gatt_read_val_handle, om_info);
			if (rc == 0)
				ESP_LOGI(TAG, "Notification sent successfully");
			else
				ESP_LOGE(TAG, "Notification not sent successfully");

			vTaskDelay(2000 / portTICK_PERIOD_MS);

			if (message != NULL)
				sprintf(message, "%s %d\n", "charger con s", rand() % (1 - 0 + 1));

			ESP_LOGI(TAG, "%s\n", message);

			om_info = NULL;
			om_info = ble_hs_mbuf_from_flat(message, strlen(message));
			rc = ble_gattc_notify_custom(phone_handle, ble_spp_svc_gatt_read_val_handle, om_info);
			if (rc == 0)
				ESP_LOGI(TAG, "Notification sent successfully");
			else
				ESP_LOGE(TAG, "Notification not sent successfully");

			vTaskDelay(2000 / portTICK_PERIOD_MS);

			if (message != NULL)
				sprintf(message, "%s %.2f\n", "batt s", float_rand(0.00, 1.00));

			ESP_LOGI(TAG, "%s\n", message);

			om_info = NULL;
			om_info = ble_hs_mbuf_from_flat(message, strlen(message));
			rc = ble_gattc_notify_custom(phone_handle, ble_spp_svc_gatt_read_val_handle, om_info);
			if (rc == 0)
				ESP_LOGI(TAG, "Notification sent successfully");
			else
				ESP_LOGE(TAG, "Notification not sent successfully");

			if (message != NULL)
				sprintf(message, "%s %s\n", "tag c", "abcxyz");

			ESP_LOGI(TAG, "%s\n", message);

			om_info = ble_hs_mbuf_from_flat(message, strlen(message));
			rc = ble_gattc_notify_custom(phone_handle, ble_spp_svc_gatt_read_val_handle, om_info);
			if (rc == 0)
				ESP_LOGI(TAG, "Notification sent successfully");
			else
				ESP_LOGE(TAG, "Notification not sent successfully");

			vTaskDelay(2000 / portTICK_PERIOD_MS);

			if (message != NULL)
				sprintf(message, "%s %d\n%s %d\n", "sos c", rand() % (1 - 0 + 1), "mag c", rand() % (4 - 0 + 1));

			ESP_LOGI(TAG, "%s\n", message);

			om_info = NULL;
			om_info = ble_hs_mbuf_from_flat(message, strlen(message));
			rc = ble_gattc_notify_custom(phone_handle, ble_spp_svc_gatt_read_val_handle, om_info);
			if (rc == 0)
				ESP_LOGI(TAG, "Notification sent successfully");
			else
				ESP_LOGE(TAG, "Notification not sent successfully");

			vTaskDelay(2000 / portTICK_PERIOD_MS);

			if (message != NULL)
				sprintf(message, "%s %0.2f\n%s %d\n", "freq c", float_rand(0.0, 1.2), "pair c", rand() % (1 - 0 + 1));

			ESP_LOGI(TAG, "%s\n", message);

			om_info = NULL;
			om_info = ble_hs_mbuf_from_flat(message, strlen(message));
			rc = ble_gattc_notify_custom(phone_handle, ble_spp_svc_gatt_read_val_handle, om_info);
			if (rc == 0)
				ESP_LOGI(TAG, "Notification sent successfully");
			else
				ESP_LOGE(TAG, "Notification not sent successfully");

			vTaskDelay(2000 / portTICK_PERIOD_MS);

			if (message != NULL)
				sprintf(message, "%s %d\n", "charger con c", rand() % (1 - 0 + 1));

			ESP_LOGI(TAG, "%s\n", message);

			om_info = NULL;
			om_info = ble_hs_mbuf_from_flat(message, strlen(message));
			rc = ble_gattc_notify_custom(phone_handle, ble_spp_svc_gatt_read_val_handle, om_info);
			if (rc == 0)
				ESP_LOGI(TAG, "Notification sent successfully");
			else
				ESP_LOGE(TAG, "Notification not sent successfully");

			vTaskDelay(2000 / portTICK_PERIOD_MS);

			if (message != NULL)
				sprintf(message, "%s %.2f\n", "batt c", float_rand(0.00, 1.00));

			ESP_LOGI(TAG, "%s\n", message);

			om_info = NULL;
			om_info = ble_hs_mbuf_from_flat(message, strlen(message));
			rc = ble_gattc_notify_custom(phone_handle, ble_spp_svc_gatt_read_val_handle, om_info);
			if (rc == 0)
				ESP_LOGI(TAG, "Notification sent successfully");
			else
				ESP_LOGE(TAG, "Notification not sent successfully");
		} else if (strstr((const char *)msg, "freq s") != NULL) {
			sscanf((const char *)msg, "%s %s %f", x, y, &manual_freq);
			ESP_LOGI(TAG, "Setting server's frequency to %.2f", manual_freq);
		} else if (strstr((const char *)msg, "mag s") != NULL) {
			sscanf((const char *)msg, "%s %s %d", x, y, &vib_strength);
			ESP_LOGI(TAG, "Setting servers's mag to %d", vib_strength);
		} else if (strstr((const char *)msg, "freq c") != NULL) {
			sscanf((const char *)msg, "%s %s %f", x, y, &manual_freq);
			ESP_LOGI(TAG, "Setting server's frequency to %.2f", manual_freq);
		} else if (strstr((const char *)msg, "mag c") != NULL) {
			sscanf((const char *)msg, "%s %s %d", x, y, &vib_strength);
			ESP_LOGI(TAG, "Setting servers's mag to %d", vib_strength);
		} else if (strstr((const char *)msg, "ssid") != NULL) {
			sscanf((const char *)msg, "%s %s", x, y);
			ESP_LOGI(TAG, "Setting ssid to %s", y);
		} else if (strstr((const char *)msg, "pswd") != NULL) {
			sscanf((const char *)msg, "%s %s", x, y);
			ESP_LOGI(TAG, "Setting password to %s", y);
		} else if (strcmp((const char *)msg, "ota") == 0) {
			ESP_LOGI(TAG, "starting OTA");
		} else if (strcmp((const char *)msg, "upl") == 0) {
			ESP_LOGI(TAG, "starting uploading procedure");
		}

		if (uxQueueMessagesWaiting(parser_queue) == 0)
			vTaskDelay(100 / portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
}

void
start_parser_task()
{
	xTaskCreatePinnedToCore(parser_task, "parser task", 1024 * 4, NULL, 4, NULL, 0);
}

void
gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
	char buf[BLE_UUID_STR_LEN];

	switch (ctxt->op) {
	case BLE_GATT_REGISTER_OP_SVC:
		ESP_LOGI(TAG,
		         "registered service %s with handle=%d\n",
		         ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
		         ctxt->svc.handle);
		break;

	case BLE_GATT_REGISTER_OP_CHR:
		ESP_LOGI(TAG,
		         "registering characteristic %s with "
		         "def_handle=%d val_handle=%d\n",
		         ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
		         ctxt->chr.def_handle,
		         ctxt->chr.val_handle);
		break;

	case BLE_GATT_REGISTER_OP_DSC:
		ESP_LOGI(TAG,
		         "registering descriptor %s with handle=%d\n",
		         ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
		         ctxt->dsc.handle);
		break;

	default:
		assert(0);
		break;
	}
}

/*
 * Initialize the BLE server with characteristics.
 */
int
gatt_svr_init(void)
{
	int rc;

	ble_svc_gap_init();
	ble_svc_gatt_init();

	rc = ble_gatts_count_cfg(walk_ble_svc_gatt_defs);
	if (rc != 0) {
		return rc;
	}

	rc = ble_gatts_add_svcs(walk_ble_svc_gatt_defs);
	if (rc != 0) {
		return rc;
	}

	return 0;
}

/*
 * Start BLE server for WALK
 */
void
ble_server_start()
{
	esp_log_level_set("NimBLE", ESP_LOG_DEBUG);
	esp_log_level_set(TAG, ESP_LOG_DEBUG);

	int rc;
	/* Initialize NVS — it is used to store PHY calibration data */
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);
	ESP_ERROR_CHECK(esp_nimble_hci_and_controller_init());

	nimble_port_init();

	/* Initialize the NimBLE host configuration. */
	ble_hs_cfg.reset_cb = ble_server_on_reset;
	ble_hs_cfg.sync_cb = ble_server_on_sync;
	ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
	ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
	ble_hs_cfg.sm_io_cap = 3;
	ble_hs_cfg.sm_bonding = 1;
	ble_hs_cfg.sm_mitm = 1;
	ble_hs_cfg.sm_sc = 1;
	ble_hs_cfg.sm_sc = 0;
	ble_hs_cfg.sm_our_key_dist = 1;
	ble_hs_cfg.sm_their_key_dist = 1;

	rc = gatt_svr_init();
	assert(rc == 0);

	/* Set the default device name. */
	rc = ble_svc_gap_device_name_set("abcxyz");
	assert(rc == 0);

	/* XXX Need to have template for store */
	ble_store_config_init();

	nimble_port_freertos_init(ble_server_host_task);

	parser_queue = xQueueCreate(10, sizeof(uint8_t) * 50);

	start_parser_task();
}

void
app_main()
{
	ble_server_start();
}
