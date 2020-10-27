#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <sys/time.h>
#include <sys/param.h>
#include "esp_netif.h"
#include "esp_eth.h"
#include <esp_http_server.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_tls.h"
#include "esp_http_client.h"
#include "esp_camera.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_timer.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_bt_defs.h"
#include "esp_ibeacon_api.h"

#define EXAMPLE_ESP_WIFI_SSID      "ATTbGDNjUI"
#define EXAMPLE_ESP_WIFI_PASS      "b5t64nxrr#xn"
#define EXAMPLE_ESP_MAXIMUM_RETRY  5
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define MAX_HTTP_RECV_BUFFER 512
#define MAX_HTTP_OUTPUT_BUFFER 512
#define NODE_SERVER_BASE_URL "http://192.168.1.135:3000/"

#define PATH_LOSS 2.0

typedef struct {
        httpd_req_t *req;
        size_t len;
} jpg_chunking_t;

void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

void ble_ibeacon_appRegister(void);

void ble_ibeacon_init(void);

void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

void wifi_init_sta(void);

esp_err_t http_event_handler(esp_http_client_event_t *evt);

void http_rest_camera_create();

size_t jpg_encode_stream(void * arg, size_t index, const void* data, size_t len);

esp_err_t downloadJPG_httpd_handler(httpd_req_t *req);

esp_err_t captureJPG_httpd_handler(httpd_req_t *req);

esp_err_t iBeacon_httpd_handler(httpd_req_t *req);

httpd_handle_t start_webserver(void);

void stop_webserver(httpd_handle_t server);

void disconnect_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

void connect_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
