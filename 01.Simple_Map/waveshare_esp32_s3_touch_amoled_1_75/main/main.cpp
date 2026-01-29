#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "lvgl.h"
#include "bsp/esp-bsp.h"
#include "simple_map.hpp"
#include "battery_monitor.h"
#include "gps_lc76g_i2c.h"

// Touch reset pin (GPIO 40)
#define TOUCH_RST_PIN GPIO_NUM_40

static const char *TAG = "GPS";

// GPS data callback - updates the map with GPS position
static void gps_callback(const gps_data_t *data, void *user_data) {
    if (data->valid) {
        // Log GPS data
        ESP_LOGI(TAG, "Fix: %.6f, %.6f | Alt: %.1fm | Speed: %.1f km/h | Sats: %d | HDOP: %.1f",
                 data->latitude, data->longitude, data->altitude,
                 data->speed_kmh, data->satellites_used, data->hdop);

        // Update map with GPS position
        if (bsp_display_lock(100)) {
            SimpleMap::set_gps_position(data->latitude, data->longitude, true);
            SimpleMap::update_gps_status(data->satellites_used, data->fix_type);
            bsp_display_unlock();
        }
    } else {
        // Log acquiring status
        ESP_LOGD(TAG, "Acquiring fix... Satellites visible: %d", data->satellites_visible);

        // No valid fix
        if (bsp_display_lock(100)) {
            SimpleMap::set_gps_position(0, 0, false);
            SimpleMap::update_gps_status(data->satellites_visible, 0);
            bsp_display_unlock();
        }
    }
}

extern "C" void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    bsp_i2c_init();

    esp_err_t err = bsp_sdcard_mount();
    if (err != ESP_OK) {
        printf("Failed to mount SD card, error: %s\n", esp_err_to_name(err));
    }

    esp_io_expander_handle_t expander = bsp_io_expander_init();
    if (expander == NULL) {
        printf("Failed to initialize IO expander\n");
        return;
    }

    // Reset LCD via IO expander pin 7
    esp_io_expander_set_dir(expander, IO_EXPANDER_PIN_NUM_7, IO_EXPANDER_OUTPUT);
    esp_io_expander_set_level(expander, IO_EXPANDER_PIN_NUM_7, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_io_expander_set_level(expander, IO_EXPANDER_PIN_NUM_7, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Reset touch controller via GPIO 40
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TOUCH_RST_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(TOUCH_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(TOUCH_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(200));  // Allow touch controller to initialize

    bsp_display_start();
    bsp_display_backlight_on();

    bsp_display_lock(0);

    // Initialize the simple map
    if (!SimpleMap::init(lv_screen_active())) {
        printf("Failed to initialize map\n");
        return;
    }

    // Show a location (Acquaseria, Lake Como, Italy)
    SimpleMap::show_location(46.05503, 9.25880, 16);
    SimpleMap::center_map_on_gps();

    bsp_display_unlock();

    // Initialize and start battery monitor
    i2c_master_bus_handle_t i2c_handle = bsp_i2c_get_handle();
    if (battery_monitor_init(i2c_handle) == ESP_OK) {
        battery_monitor_start(5000);  // Update every 5 seconds
    } else {
        printf("Battery monitor initialization failed (AXP2101 may not be present)\n");
    }

    // Initialize GPS module via I2C (LC76G on this board uses I2C, not UART)
    if (gps_i2c_init(i2c_handle) == ESP_OK) {
        gps_i2c_register_data_callback(gps_callback, NULL);
        gps_i2c_start(1000);  // Poll every 1 second (matches GPS update rate)
        printf("GPS initialized via I2C (addr: 0x50/0x54)\n");

        // Set GPS log level to DEBUG to see NMEA sentences
        esp_log_level_set("GPS_LC76G_I2C", ESP_LOG_DEBUG);
    } else {
        printf("GPS initialization failed\n");
    }
}