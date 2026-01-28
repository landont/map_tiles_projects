# ESP32-S3-Touch-LCD-3.5B - Interactive Map Demo
cmake_minimum_required(VERSION 3.5)

# Add shared components from parent directory
set(EXTRA_COMPONENT_DIRS "../shared_components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

add_compile_options(-Wno-format)
project(simple_map_waveshare_esp32_s3_touch_lcd_3_5b)
