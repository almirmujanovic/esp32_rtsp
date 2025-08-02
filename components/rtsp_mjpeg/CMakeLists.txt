idf_component_register(
    SRCS         "src/rtsp_mjpeg.c" "src/camera_config.c"
    INCLUDE_DIRS "include"
    REQUIRES     esp32-camera esp_jpeg esp_timer lwip
)
