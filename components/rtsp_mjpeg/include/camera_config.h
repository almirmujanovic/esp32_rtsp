#pragma once
#include "esp_camera.h"

// Pin configuration for Freenove ESP32S3-EYE (edit if needed)
#define CAMERA_PIN_PWDN      -1
#define CAMERA_PIN_RESET     -1
#define CAMERA_PIN_XCLK      15
#define CAMERA_PIN_SIOD      4
#define CAMERA_PIN_SIOC      5

#define CAMERA_PIN_D0        11   // Y2 ↔ D0
#define CAMERA_PIN_D1        9    // Y3 ↔ D1
#define CAMERA_PIN_D2        8    // Y4 ↔ D2
#define CAMERA_PIN_D3        10   // Y5 ↔ D3
#define CAMERA_PIN_D4        12   // Y6 ↔ D4
#define CAMERA_PIN_D5        18   // Y7 ↔ D5
#define CAMERA_PIN_D6        17   // Y8 ↔ D6
#define CAMERA_PIN_D7        16   // Y9 ↔ D7

#define CAMERA_PIN_VSYNC     6
#define CAMERA_PIN_HREF      7
#define CAMERA_PIN_PCLK      13

#define CAMERA_JPEG_QUALITY  20   // Lower is better quality (try 20-40)
#define CAMERA_FB_COUNT      1

#define CAMERA_FRAME_SIZE_ENUM FRAMESIZE_QVGA
#define CAMERA_FRAME_WIDTH   320
#define CAMERA_FRAME_HEIGHT  240

esp_err_t init_camera(void);