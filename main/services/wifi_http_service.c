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

static const char *TAG = "WIFI_HTTP_SERVICE";

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

static esp_ble_adv_params_t ble_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_NONCONN_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

static uint8_t derived_mac_addr[6] = {0};
static char ipv4_str[16] = {0};
static char mac_str[18] = {0};
static char targetURL[150] = {0};
static char postData[250] = {0};
static char local_response_buffer[MAX_HTTP_OUTPUT_BUFFER] = {0};

static camera_fb_t * fb = NULL;
static size_t fb_len = 0;

extern esp_ble_ibeacon_vendor_t vendor_config;
struct timeval tv_now;
int64_t time_prev = 0;
int64_t time_curr = 0;
double_t smoothedDistanceMeters = -255;

void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    if(event ==ESP_GAP_BLE_SCAN_RESULT_EVT){
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        if(scan_result->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
            /* Search for BLE iBeacon Packet */
            if(smoothedDistanceMeters < 0) {
                smoothedDistanceMeters = pow(10.0, (ibeacon_data->ibeacon_vendor.measured_power - scan_result->scan_rst.rssi) / (10.0 * PATH_LOSS));
            }
            else {
                smoothedDistanceMeters = (0.7)*smoothedDistanceMeters + (0.3)*pow(10.0, (ibeacon_data->ibeacon_vendor.measured_power - scan_result->scan_rst.rssi) / (10.0 * PATH_LOSS));
            }
        }
    }
}

void ble_ibeacon_appRegister(void)
{
    esp_err_t status;
    ESP_LOGI(TAG, "register callback");
    //register the scan callback function to the gap module
    if ((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK) {
        ESP_LOGE(TAG, "gap register error: %s", esp_err_to_name(status));
        return;
    }
}

void ble_ibeacon_init() {  // uint8_t iBeaconMode) {
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);

    esp_bluedroid_init();
    esp_bluedroid_enable();
    ble_ibeacon_appRegister();

    esp_ble_gap_set_scan_params(&ble_scan_params);
    esp_ble_ibeacon_t ibeacon_adv_data;
    esp_err_t status = esp_ble_config_ibeacon_data (&vendor_config, &ibeacon_adv_data);
    if (status == ESP_OK){
        esp_ble_gap_config_adv_data_raw((uint8_t*)&ibeacon_adv_data, sizeof(ibeacon_adv_data));
    }
    else {
        ESP_LOGE(TAG, "Config iBeacon data failed: %s\n", esp_err_to_name(status));
    }
}

void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        ESP_ERROR_CHECK(esp_read_mac(derived_mac_addr, ESP_MAC_WIFI_STA));
        sprintf(mac_str, "%02x:%02x:%02x:%02x:%02x:%02x", derived_mac_addr[0], derived_mac_addr[1], derived_mac_addr[2], derived_mac_addr[3], derived_mac_addr[4], derived_mac_addr[5]);
        sprintf(ipv4_str, IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
	        .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s", EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s", EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

esp_err_t http_event_handler(esp_http_client_event_t *evt) {
    static char *output_buffer;  // Buffer to store response of http request from event handler
    static int output_len;       // Stores number of bytes read
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            /*
             *  Check for chunked encoding is added as the URL for chunked encoding used in this example returns binary data.
             *  However, event handler can also be used in case chunked encoding is used.
             */
            if (!esp_http_client_is_chunked_response(evt->client)) {
                // If user_data buffer is configured, copy the response into the buffer
                if (evt->user_data) {
                    memcpy(evt->user_data + output_len, evt->data, evt->data_len);
                } else {
                    if (output_buffer == NULL) {
                        output_buffer = (char *) malloc(esp_http_client_get_content_length(evt->client));
                        output_len = 0;
                        if (output_buffer == NULL) {
                            ESP_LOGE(TAG, "Failed to allocate memory for output buffer");
                            return ESP_FAIL;
                        }
                    }
                    memcpy(output_buffer + output_len, evt->data, evt->data_len);
                }
                output_len += evt->data_len;
            }

            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            if (output_buffer != NULL) {
                // Response is accumulated in output_buffer. Uncomment the below line to print the accumulated response
                // ESP_LOG_BUFFER_HEX(TAG, output_buffer, output_len);
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
            int mbedtls_err = 0;
            esp_err_t err = esp_tls_get_and_clear_last_error(evt->data, &mbedtls_err, NULL);
            if (err != 0) {
                if (output_buffer != NULL) {
                    free(output_buffer);
                    output_buffer = NULL;
                }
                output_len = 0;
                ESP_LOGI(TAG, "Last esp error code: 0x%x", err);
                ESP_LOGI(TAG, "Last mbedtls failure: 0x%x", mbedtls_err);
            }
            break;
    }
    return ESP_OK;
}

void http_rest_camera_create() {
    strcpy(targetURL, NODE_SERVER_BASE_URL);
    strcat(targetURL, "camera/create");
    
    strcpy(postData, "{\"mac\":\"");
    strcat(postData, mac_str);
    strcat(postData, "\",\"ipv4\":\"");
    strcat(postData, ipv4_str);
    strcat(postData, "\"}");

    esp_http_client_config_t config = {
        .url = targetURL,
        .method = HTTP_METHOD_POST,
        .event_handler = http_event_handler,
        .user_data = local_response_buffer        // Pass address of local buffer to get response
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, postData, strlen(postData));

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d",
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));
    } else {
        ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(client);
}

size_t jpg_encode_stream(void * arg, size_t index, const void* data, size_t len) {
    jpg_chunking_t *j = (jpg_chunking_t *)arg;
    if(!index){
        j->len = 0;
    }
    if(httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK){
        return 0;
    }
    j->len += len;
    return len;
}

esp_err_t downloadJPG_httpd_handler(httpd_req_t *req) {
    esp_err_t res = ESP_OK;
    res = httpd_resp_set_type(req, "image/jpeg");
    if(res == ESP_OK){
        res = httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    }

    if(res == ESP_OK){
        if(fb->format == PIXFORMAT_JPEG){
            fb_len = fb->len;
            res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
        } else {
            jpg_chunking_t jchunk = {req, 0};
            res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk)?ESP_OK:ESP_FAIL;
            httpd_resp_send_chunk(req, NULL, 0);
            fb_len = jchunk.len;
        }
    }
    esp_camera_fb_return(fb);
    fb = NULL;
    return res;
}

esp_err_t captureJPG_httpd_handler(httpd_req_t *req) {
    esp_err_t res = ESP_OK;
    if(fb != NULL) {
        const char resp[] = "imageBuffered";
        httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
        return res;
    }
    fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    const char resp[] = "imageCaptured";
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return res;
}

esp_err_t iBeacon_httpd_handler(httpd_req_t *req) {
    char*  buf;
    size_t buf_len;
    char variable[32] = {0,};
    char value[32] = {0,};
    esp_err_t res = ESP_OK;
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = (char*)malloc(buf_len);
        if(!buf){
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            if (httpd_query_key_value(buf, "state", variable, sizeof(variable)) == ESP_OK) {
                switch(atoi(buf)) {
                    case 1: 
                        smoothedDistanceMeters = -255;
                        esp_ble_gap_start_scanning(0);
                        break;
                    case 2: 
                        esp_ble_gap_stop_scanning();
                        sprintf(postData, "%lf", smoothedDistanceMeters);
                        httpd_resp_send(req, postData, strlen(postData));
                        break;
                    case 3:
                        esp_ble_gap_start_advertising(&ble_adv_params);
                        break;
                    case 4:
                        esp_ble_gap_stop_advertising();
                        break;
                    default:
                        break;
                }
                free(buf);
                sprintf(postData, "success");
                httpd_resp_send(req, postData, strlen(postData));
                httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
                return res;
            }
        } 
        free(buf);
    }
    httpd_resp_send_404(req);
    return ESP_FAIL;
}

static const httpd_uri_t downloadJPG = {
    .uri       = "/downloadJPG",
    .method    = HTTP_GET,
    .handler   = downloadJPG_httpd_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t captureJPG = {
    .uri       = "/captureJPG",
    .method    = HTTP_GET,
    .handler   = captureJPG_httpd_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t iBeacon = {
    .uri       = "/iBeacon",
    .method    = HTTP_GET,
    .handler   = iBeacon_httpd_handler,
    .user_ctx  = NULL
};

httpd_handle_t start_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &downloadJPG);
        httpd_register_uri_handler(server, &captureJPG);
        httpd_register_uri_handler(server, &iBeacon);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

void stop_webserver(httpd_handle_t server) {
    // Stop the httpd server
    httpd_stop(server);
}

void disconnect_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        ESP_LOGI(TAG, "Stopping webserver");
        stop_webserver(*server);
        *server = NULL;
    }
}

void connect_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    ESP_LOGI(TAG, "CONNECT HANDLER BEGIN");
    // ESP_ERROR_CHECK(esp_read_mac(derived_mac_addr, ESP_MAC_WIFI_STA));
    // sprintf(mac_str, "%02x:%02x:%02x:%02x:%02x:%02x", derived_mac_addr[0], derived_mac_addr[1], derived_mac_addr[2], derived_mac_addr[3], derived_mac_addr[4], derived_mac_addr[5]);
    
    // ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
    // sprintf(ipv4_str, IPSTR, IP2STR(&event->ip_info.ip));

    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver();
    }
}

// void http_rest_with_url(void) {
//     char local_response_buffer[MAX_HTTP_OUTPUT_BUFFER] = {0};
//     /**
//      * NOTE: All the configuration parameters for http_client must be spefied either in URL or as host and path parameters.
//      * If host and path parameters are not set, query parameter will be ignored. In such cases,
//      * query parameter should be specified in URL.
//      *
//      * If URL as well as host and path parameters are specified, values of host and path will be considered.
//      */
//     esp_http_client_config_t config = {
//         .host = "httpbin.org",
//         .path = "/get",
//         .query = "esp",
//         .event_handler = _http_event_handler,
//         .user_data = local_response_buffer,        // Pass address of local buffer to get response
//     };
//     esp_http_client_handle_t client = esp_http_client_init(&config);

//     // GET
//     esp_err_t err = esp_http_client_perform(client);
//     if (err == ESP_OK) {
//         ESP_LOGI(TAG, "HTTP GET Status = %d, content_length = %d",
//                 esp_http_client_get_status_code(client),
//                 esp_http_client_get_content_length(client));
//     } else {
//         ESP_LOGE(TAG, "HTTP GET request failed: %s", esp_err_to_name(err));
//     }
//     ESP_LOG_BUFFER_HEX(TAG, local_response_buffer, strlen(local_response_buffer));

//     // POST
//     const char *post_data = "{\"field1\":\"value1\"}";
//     esp_http_client_set_url(client, "http://httpbin.org/post");
//     esp_http_client_set_method(client, HTTP_METHOD_POST);
//     esp_http_client_set_header(client, "Content-Type", "application/json");
//     esp_http_client_set_post_field(client, post_data, strlen(post_data));
//     err = esp_http_client_perform(client);
//     if (err == ESP_OK) {
//         ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d",
//                 esp_http_client_get_status_code(client),
//                 esp_http_client_get_content_length(client));
//     } else {
//         ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
//     }

//     //PUT
//     esp_http_client_set_url(client, "http://httpbin.org/put");
//     esp_http_client_set_method(client, HTTP_METHOD_PUT);
//     err = esp_http_client_perform(client);
//     if (err == ESP_OK) {
//         ESP_LOGI(TAG, "HTTP PUT Status = %d, content_length = %d",
//                 esp_http_client_get_status_code(client),
//                 esp_http_client_get_content_length(client));
//     } else {
//         ESP_LOGE(TAG, "HTTP PUT request failed: %s", esp_err_to_name(err));
//     }

//     //PATCH
//     esp_http_client_set_url(client, "http://httpbin.org/patch");
//     esp_http_client_set_method(client, HTTP_METHOD_PATCH);
//     esp_http_client_set_post_field(client, NULL, 0);
//     err = esp_http_client_perform(client);
//     if (err == ESP_OK) {
//         ESP_LOGI(TAG, "HTTP PATCH Status = %d, content_length = %d",
//                 esp_http_client_get_status_code(client),
//                 esp_http_client_get_content_length(client));
//     } else {
//         ESP_LOGE(TAG, "HTTP PATCH request failed: %s", esp_err_to_name(err));
//     }

//     //DELETE
//     esp_http_client_set_url(client, "http://httpbin.org/delete");
//     esp_http_client_set_method(client, HTTP_METHOD_DELETE);
//     err = esp_http_client_perform(client);
//     if (err == ESP_OK) {
//         ESP_LOGI(TAG, "HTTP DELETE Status = %d, content_length = %d",
//                 esp_http_client_get_status_code(client),
//                 esp_http_client_get_content_length(client));
//     } else {
//         ESP_LOGE(TAG, "HTTP DELETE request failed: %s", esp_err_to_name(err));
//     }

//     //HEAD
//     esp_http_client_set_url(client, "http://httpbin.org/get");
//     esp_http_client_set_method(client, HTTP_METHOD_HEAD);
//     err = esp_http_client_perform(client);
//     if (err == ESP_OK) {
//         ESP_LOGI(TAG, "HTTP HEAD Status = %d, content_length = %d",
//                 esp_http_client_get_status_code(client),
//                 esp_http_client_get_content_length(client));
//     } else {
//         ESP_LOGE(TAG, "HTTP HEAD request failed: %s", esp_err_to_name(err));
//     }

//     esp_http_client_cleanup(client);
// }
