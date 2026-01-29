# Detect which BSP component is available by checking if the component directory exists
set(BSP_COMPONENT "")

# Get the project directory to search for components
idf_build_get_property(project_dir PROJECT_DIR)

# Check for known BSP components in the project's components directory
if(EXISTS "${project_dir}/components/esp32_s3_touch_lcd_3_5b")
    set(BSP_COMPONENT "esp32_s3_touch_lcd_3_5b")
elseif(EXISTS "${project_dir}/components/esp32_s3_touch_amoled_1_75")
    set(BSP_COMPONENT "esp32_s3_touch_amoled_1_75")
endif()

if(BSP_COMPONENT)
    message(STATUS "battery_monitor: Using BSP component '${BSP_COMPONENT}'")
    idf_component_register(
        SRCS "battery_monitor.cpp"
        INCLUDE_DIRS "."
        REQUIRES "driver" "XPowersLib" "${BSP_COMPONENT}" "simple_map" "gps_lc76g"
    )
else()
    message(WARNING "battery_monitor: No supported BSP component found")
    idf_component_register(
        SRCS "battery_monitor.cpp"
        INCLUDE_DIRS "."
        REQUIRES "driver" "XPowersLib" "simple_map" "gps_lc76g"
    )
endif()
