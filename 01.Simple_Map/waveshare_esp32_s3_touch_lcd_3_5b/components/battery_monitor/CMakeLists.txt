idf_component_register(
    SRCS "battery_monitor.cpp"
    INCLUDE_DIRS "."
    REQUIRES "driver" "XPowersLib" "esp32_s3_touch_lcd_3_5b" "simple_map"
)
