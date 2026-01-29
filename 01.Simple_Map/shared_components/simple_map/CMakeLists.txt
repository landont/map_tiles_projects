idf_component_register(
    SRCS "simple_map.cpp"
    INCLUDE_DIRS "."
    REQUIRES "0015__map_tiles" "lvgl" "esp32_s3_touch_lcd_3_5b"
)