# Detect which BSP component is available in the build
set(BSP_COMPONENT "")

# Check for known BSP components
idf_build_get_property(build_components BUILD_COMPONENTS)
if("esp32_s3_touch_lcd_3_5b" IN_LIST build_components)
    set(BSP_COMPONENT "esp32_s3_touch_lcd_3_5b")
elseif("esp32_s3_touch_amoled_1_75" IN_LIST build_components)
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
    message(WARNING "simple_map: No supported BSP component found")
    idf_component_register(
        SRCS "simple_map.cpp"
        INCLUDE_DIRS "."
        REQUIRES "0015__map_tiles" "lvgl"
    )
endif()
