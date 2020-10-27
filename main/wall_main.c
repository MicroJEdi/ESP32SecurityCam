#include <string.h>
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_camera.h"

#include "services/wifi_http_service.h"

#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1 //software reset will be performed
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27
#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22


// higher priority means more important
// range is 0 - 24
// #define MOVEMENT_PRIORITY 6
// #define CAPTURE_PRIORITY 6
// #define REFLECT_PRIORITY 6
// #define STORE_PRIORITY 6
// #define MOVEMENT_STACK_SIZE 1000
// #define CAPTURE_STACK_SIZE 1000
// #define REFLECT_STACK_SIZE 1000
// #define STORE_STACK_SIZE 1000
// #define MOVEMENT_THREASHOLD 100
// #define BUFFER_SIZE 20

// enum Status { INIT, CONFIG, IDLE, CAPTURE, PERSIST };

static const char *TAG = "WALL_MAIN";
// static enum Status deviceState = INIT;

static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,
    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,
    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG, // PIXFORMAT_ + YUV422|GRAYSCALE|RGB565|RGB888|JPEG
    .frame_size = FRAMESIZE_SVGA,    // FRAMESIZE_ + QVGA|PIXFORMAT_RGB565|SVGA|XGA|SXGA|UXGA
    .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 1       //if more than one, i2s runs in continuous mode. Use only with JPEG
};

// TaskHandle_t movementTask;
// TaskHandle_t captureTask;
// TaskHandle_t reflectTask;
// TaskHandle_t storeTask;

void app_main(void)
{
    static httpd_handle_t server = NULL;

    esp_err_t ret = esp_camera_init(&camera_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera Init Failed");
    }

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta();

    ble_ibeacon_init();

    server = start_webserver();

    http_rest_camera_create();

    // xTaskCreate(&movementHandler, "movement", MOVEMENT_STACK_SIZE, NULL, MOVEMENT_PRIORITY, &movementTask);
    // xTaskCreate(&capture Handler, "capture", CAPTURE_STACK_SIZE, NULL, CAPTURE_PRIORITY, &captureTask);
    // xTaskCreate(&reflectHandler, "reflect", REFLECT_STACK_SIZE, NULL, REFLECT_PRIORITY, &reflectTask);
    // xTaskCreate(&storeHandler, "store", STORE_STACK_SIZE, NULL, STORE_PRIORITY, &storeTask);
}