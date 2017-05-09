

#include "freertos/FreeRTOS.h"
#include "debug.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"


// ----------bt
#include "esp_log.h"
#include "nvs_flash.h"
#include "bt.h"

#include "esp_blufi_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
// ----------bt

// MQTT Library include
#include "mqtt.h"

extern void bt_task();

//[ BLE

static const char tag[] = "BLE";

static bool bd_already_enable = false;
static bool bd_already_init = false;
static mqtt_client *client;

/*static uint8_t ibeacon_prefix[] = {
    0x02,0x01,0x00,0x1A,0xFF,0x4C,0x00,0x02,0x15
};*/


void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
   esp_ble_gap_cb_param_t *p = (esp_ble_gap_cb_param_t *)param;

   if (p->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
      // Check for iBeacon adv prefix. Ignore 3rd byte, this varies from beacon type to beacon type
      //p->scan_rst.ble_adv[2]  = 0x00;
      //for (int i=0; i < sizeof(ibeacon_prefix); i++) {
      //   if (p->scan_rst.ble_adv[i] != ibeacon_prefix[i]) {
      //      return;
      //   }
      //}
      ESP_LOGI(tag, "BDA: %02x:%02x:%02x:%02x:%02x:%02x, RSSI %d",
         p->scan_rst.bda[0],
         p->scan_rst.bda[1],
         p->scan_rst.bda[2],
         p->scan_rst.bda[3],
         p->scan_rst.bda[4],
         p->scan_rst.bda[5],
         p->scan_rst.rssi);

      // Allocates storage (12 characters for id + length of ("/test/pingback/")
      char *topic = (char*)malloc(27 * sizeof(char));
      // Print
      sprintf(topic, "/test/pingback/%02x%02x%02x%02x%02x%02x",
                        p->scan_rst.bda[0],
                        p->scan_rst.bda[1],
                        p->scan_rst.bda[2],
                        p->scan_rst.bda[3],
                        p->scan_rst.bda[4],
                        p->scan_rst.bda[5]);
      char *rssi = (char*)malloc(255 * sizeof(char));
      sprintf(rssi, "%d", p->scan_rst.rssi);

      mqtt_publish(client, topic , rssi, sizeof(rssi), 0, 0);
   }
}


void bt_task() {
   esp_err_t ret;

   esp_bt_controller_init();

    if(esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED){
        if (esp_bt_controller_enable(ESP_BT_MODE_BTDM)) {
            INFO("BT Enable failed");
            goto end;
        }
   }

   if (!bd_already_init) {

       ret = esp_bluedroid_init();
       if (ret != ESP_OK) {
          INFO("init bluedroid failed\n");
          goto end;
       }
       bd_already_init = true;
   }

   if (!bd_already_enable) {
       ret = esp_bluedroid_enable();
       if (ret) {
          INFO("enable bluedroid failed\n");
          goto end;
       }
       bd_already_enable = true;
   }
   ret = esp_ble_gap_register_callback(gap_event_handler);
   if (ret != ESP_OK) {
      INFO("esp_ble_gap_register_callback: rc=%d",ret);
      goto end;
   }
   static esp_ble_scan_params_t ble_scan_params = {
      .scan_type              = BLE_SCAN_TYPE_PASSIVE,
      .own_addr_type          = ESP_PUBLIC_ADDR,
      .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
      .scan_interval          = 0x50,
      .scan_window            = 0x30
   };
   ret = esp_ble_gap_set_scan_params(&ble_scan_params);
   if (ret != ESP_OK) {
      INFO("esp_ble_gap_set_scan_params: rc=%d", ret);
      goto end;
   }

   ret = esp_ble_gap_start_scanning(60);
   if (ret != ESP_OK) {
      INFO("esp_ble_gap_start_scanning: rc=%d", ret);
      goto end;
   }
   ESP_LOGI(tag, "Wait for scans...");
end:

    return;
}

//] BLE


//[ MQTT

void mqtt_connected_cb(void *self, void *params)
{
    client = (mqtt_client *)self;
    // test needs to be replaced by "indoorposition" later... or whatever topic to use further on when used "in production"
    mqtt_subscribe(client, "/test/execute_scan", 0);
    mqtt_publish(client, "/test", "howdy!", 6, 0, 0);
}

void mqtt_disconnected_cb(void *self, void *params)
{

}

void mqtt_reconnect_cb(void *self, void *params)
{

}

void mqtt_subscribe_cb(void *self, void *params)
{
    //INFO("[APP] Subscribe ok, test publish msg\n");
    //mqtt_client *client = (mqtt_client *)self;
    //mqtt_publish(client, "/test", "abcde", 5, 0, 0);
}

void mqtt_publish_cb(void *self, void *params)
{

}

// when ever there's a message, whatever message, coming in through the one subscribed topic, this will trigger a BLE scan, which
// will be completed and results be sent...
void mqtt_data_cb(void *self, void *params)
{
    mqtt_event_data_t *event_data = (mqtt_event_data_t *)params;

    if (event_data->data_offset == 0) {

        char *topic = malloc(event_data->topic_length + 1);
        memcpy(topic, event_data->topic, event_data->topic_length);
        topic[event_data->topic_length] = 0;
        INFO("[APP] got command on execute_scan topic: %s\n", topic);
        free(topic);
    }

    char *data = malloc(event_data->data_length + 1);
    memcpy(data, event_data->data, event_data->data_length);
    data[event_data->data_length] = 0;
    INFO("[APP] command was: [%d/%d bytes] - %s\n",
         event_data->data_length + event_data->data_offset,
         event_data->data_total_length,
         data);

    //client = (mqtt_client *)self;

    INFO("[APP] acknowledge command");
    // now go ahead, acknowledge the command...
    // todo: put actual useable client content into the message:
    //          - own ip
    //          - software revision
    //          - capabilities...
    mqtt_publish(client, "/test/acknowledge_execute_scan", data, event_data->data_length, 0, 0);

    // now do a scan...
    INFO("[APP] do a scan");

    bt_task();
    free(data);
}

mqtt_settings settings = {
    .host = CONFIG_MQTT_BROKER_HOST,
#if defined(CONFIG_MQTT_SECURITY_ON)
    .port = CONFIG_MQTT_BROKER_PORT_ENCRYPTED, // encrypted
#else
    .port = CONFIG_MQTT_BROKER_PORT_UNENCRYPTED, // unencrypted
#endif
    .client_id = CONFIG_MQTT_BROKER_CLIENT_ID,
    .username = CONFIG_MQTT_BROKER_USERNAME,
    .password = CONFIG_MQTT_BROKER_PASSWORD,
    .clean_session = 0,
    .keepalive = 120,
    .lwt_topic = CONFIG_INDOOR_LOCATION_PUBLISH_TOPIC,
    .lwt_msg = "offline",
    .lwt_qos = 0,
    .lwt_retain = 0,
    .connected_cb = mqtt_connected_cb,
    .disconnected_cb = mqtt_disconnected_cb,
    .reconnect_cb = mqtt_reconnect_cb,
    .subscribe_cb = mqtt_subscribe_cb,
    .publish_cb = mqtt_publish_cb,
    .data_cb = mqtt_data_cb
};

//] MQTT

//[ WIFI

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{

    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        ESP_ERROR_CHECK(esp_wifi_connect());
        break;

    case SYSTEM_EVENT_STA_GOT_IP:
        // start mqtt...
        mqtt_start(&settings);
        INFO("MQTT started...\n");
        // Notice that, all callback will called in mqtt_task
        // All function publish, subscribe
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        INFO("WIFI disconnected...\n");
        mqtt_stop();
        ESP_ERROR_CHECK(esp_wifi_connect());
        break;
    default:
        break;
    }
    return ESP_OK;

}

void wifi_conn_init(void)
{

}

//] WIFI

void app_main(void)
{
    INFO("[APP] Starting up..");
    ESP_ERROR_CHECK( nvs_flash_init() );
    //[ Wifi Connect
    INFO("[APP] Start, connect to Wifi network: %s ..\n", CONFIG_WIFI_SSID);

    tcpip_adapter_init();

    ESP_ERROR_CHECK( esp_event_loop_init(wifi_event_handler, NULL) );

    wifi_init_config_t icfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&icfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
            .bssid_set = false
        },
    };

    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK( esp_wifi_start());
    //]

    while (true) {
        vTaskDelay(300 / portTICK_PERIOD_MS);
        //INFO("main loop");
        //if (client)
        //    bt_task();
        //mqtt_client *client = (mqtt_client *)self;
        //mqtt_publish(client, "/test/alive", "esp32-is-alive", 6, 0, 0);

    }
}
