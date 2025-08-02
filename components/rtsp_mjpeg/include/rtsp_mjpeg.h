#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the RTSP MJPEG server (includes camera init)
 *
 * @param stack_size FreeRTOS stack size for the server task
 * @param priority   FreeRTOS priority for the server task
 * @return ESP_OK on success, or error code
 */
esp_err_t rtsp_mjpeg_server_start(size_t stack_size, UBaseType_t priority);

/**
 * @brief Stop the RTSP MJPEG server
 *
 * @return ESP_OK on success
 */
esp_err_t rtsp_mjpeg_server_stop(void);

#ifdef __cplusplus
}
#endif
