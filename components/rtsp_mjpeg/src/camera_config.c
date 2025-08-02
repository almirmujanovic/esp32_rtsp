#include "camera_config.h"
#include "esp_camera.h"
#include "esp_log.h"

static const char *TAG = "CAMERA_CONFIG";

esp_err_t init_camera(void) {
    camera_config_t config = {
        .pin_pwdn       = CAMERA_PIN_PWDN,
        .pin_reset      = CAMERA_PIN_RESET,
        .pin_xclk       = CAMERA_PIN_XCLK,
        .pin_sccb_sda   = CAMERA_PIN_SIOD,
        .pin_sccb_scl   = CAMERA_PIN_SIOC,

        .pin_d7         = CAMERA_PIN_D7,
        .pin_d6         = CAMERA_PIN_D6,
        .pin_d5         = CAMERA_PIN_D5,
        .pin_d4         = CAMERA_PIN_D4,
        .pin_d3         = CAMERA_PIN_D3,
        .pin_d2         = CAMERA_PIN_D2,
        .pin_d1         = CAMERA_PIN_D1,
        .pin_d0         = CAMERA_PIN_D0,

        .pin_vsync      = CAMERA_PIN_VSYNC,
        .pin_href       = CAMERA_PIN_HREF,
        .pin_pclk       = CAMERA_PIN_PCLK,

        .xclk_freq_hz   = 20000000,
        .ledc_timer     = LEDC_TIMER_0,
        .ledc_channel   = LEDC_CHANNEL_0,

        .pixel_format   = PIXFORMAT_JPEG,
        .frame_size     = CAMERA_FRAME_SIZE_ENUM,
        .jpeg_quality   = CAMERA_JPEG_QUALITY,
        .fb_count       = CAMERA_FB_COUNT,

#if defined(CONFIG_IDF_TARGET_ESP32S3)
        .fb_location    = CAMERA_FB_IN_DRAM,
#endif
        .grab_mode      = CAMERA_GRAB_WHEN_EMPTY,
    };

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Camera initialized successfully");
    return ESP_OK;
}