idf_component_register(
    SRCS "gps_lc76g_uart.c"
    INCLUDE_DIRS "."
    REQUIRES "driver" "esp_timer"
)
