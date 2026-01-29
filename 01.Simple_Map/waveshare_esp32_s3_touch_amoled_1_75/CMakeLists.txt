# Device A - Interactive Map Demo
cmake_minimum_required(VERSION 3.5)

# Add shared components from parent directory
set(EXTRA_COMPONENT_DIRS "../shared_components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

add_compile_options(-Wno-format)
project(simple_map_waveshare_esp32_s3_touch_amoled_1_75)

# Patch esp_lvgl_port touch handling to not crash on touch read errors
# The CST9217 touch controller can occasionally return errors that should be handled gracefully
set(TOUCH_FILE "${CMAKE_SOURCE_DIR}/managed_components/espressif__esp_lvgl_port/src/lvgl9/esp_lvgl_port_touch.c")
if(EXISTS "${TOUCH_FILE}")
    file(READ "${TOUCH_FILE}" TOUCH_CONTENT)
    string(FIND "${TOUCH_CONTENT}" "ESP_ERROR_CHECK(esp_lcd_touch_read_data" FOUND_POS)
    if(NOT FOUND_POS EQUAL -1)
        message(STATUS "Patching esp_lvgl_port_touch.c to handle touch read errors gracefully...")
        string(REPLACE
            "ESP_ERROR_CHECK(esp_lcd_touch_read_data(touch_ctx->handle));"
            "if (esp_lcd_touch_read_data(touch_ctx->handle) != ESP_OK) { return; }"
            TOUCH_CONTENT "${TOUCH_CONTENT}")
        file(WRITE "${TOUCH_FILE}" "${TOUCH_CONTENT}")
        message(STATUS "Patch applied successfully")
    endif()
endif()
