idf_component_register(
    SRCS "gps_lc76g.c" "gps_lc76g_i2c.c"
    INCLUDE_DIRS "."
    REQUIRES "driver" "esp_timer"
)
