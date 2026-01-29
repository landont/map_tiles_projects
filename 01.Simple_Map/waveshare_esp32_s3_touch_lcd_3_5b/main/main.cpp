#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "lvgl.h"
#include "bsp/esp-bsp.h"
#include "simple_map.hpp"
#include "battery_monitor.h"

static const char *TAG = "main";

extern "C" void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize I2C bus
    bsp_i2c_init();

    // Initialize battery monitor (AXP2101 PMIC)
    i2c_master_bus_handle_t i2c_handle = bsp_i2c_get_handle();
    if (i2c_handle != nullptr) {
        ret = battery_monitor_init(i2c_handle);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Battery monitor init failed, continuing without battery monitoring");
        }
    } else {
        ESP_LOGW(TAG, "I2C handle is null, skipping battery monitor init");
    }

    // Mount SD card
    esp_err_t err = bsp_sdcard_mount();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(err));
    }

    // Start display
    bsp_display_start();
    bsp_display_backlight_on();

    bsp_display_lock(0);

    // Initialize the simple map
    if (!SimpleMap::init(lv_screen_active())) {
        ESP_LOGE(TAG, "Failed to initialize map");
        bsp_display_unlock();
        return;
    }

    // Show a location (Acquaseria, Lake Como, Italy)
    SimpleMap::show_location(46.05503, 9.25880, 16);
    SimpleMap::center_map_on_gps();

    bsp_display_unlock();

    // Start battery monitoring task (update every 5 seconds)
    if (i2c_handle != nullptr) {
        battery_monitor_start(5000);
    }

    ESP_LOGI(TAG, "Map application started");
}
