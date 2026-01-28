idf_component_register(
    SRCS "esp32_s3_touch_lcd_3_5b.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES "driver" "esp_lcd" "esp_lvgl_port" "lvgl" "fatfs" "esp_io_expander"
    PRIV_REQUIRES "esp_timer" "vfs" "sdmmc" "spiffs" "esp_io_expander_tca9554"
)
