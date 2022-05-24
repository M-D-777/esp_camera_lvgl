idf_component_register(
    SRCS "main.c" "app_camera_esp.c"
    INCLUDE_DIRS ""
    REQUIRES lvgl lvgl_esp32_drivers esp32-camera
)