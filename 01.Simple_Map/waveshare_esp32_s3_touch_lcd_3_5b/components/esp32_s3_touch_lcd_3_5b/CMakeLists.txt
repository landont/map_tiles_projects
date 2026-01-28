idf_component_register(
    SRCS "esp32_s3_touch_lcd_3_5b.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES "driver" "esp_lcd" "esp_lvgl_port" "lvgl"
    PRIV_REQUIRES "esp_timer" "fatfs" "vfs" "sdmmc"
)
