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
    message(STATUS "simple_map: Using BSP component '${BSP_COMPONENT}'")
    idf_component_register(
        SRCS "simple_map.cpp"
        INCLUDE_DIRS "."
        REQUIRES "0015__map_tiles" "lvgl" "${BSP_COMPONENT}"
    )
else()
    message(WARNING "simple_map: No supported BSP component found in ${project_dir}/components/")
    idf_component_register(
        SRCS "simple_map.cpp"
        INCLUDE_DIRS "."
        REQUIRES "0015__map_tiles" "lvgl"
    )
endif()
