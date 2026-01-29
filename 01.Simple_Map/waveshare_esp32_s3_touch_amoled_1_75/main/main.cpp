#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "lvgl.h"
#include "bsp/esp-bsp.h"
#include "simple_map.hpp"
#include "battery_monitor.h"
#include "gps_lc76g.h"

// Touch reset pin (GPIO 40)
#define TOUCH_RST_PIN GPIO_NUM_40

// GPS UART pins for ESP32-S3-Touch-AMOLED-1.75
// Using GPIO 17 (TX) and GPIO 18 (RX) which are available
#define GPS_TX_PIN    17
#define GPS_RX_PIN    18

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

    // Initialize GPS module
    gps_config_t gps_config = {
        .uart_num = UART_NUM_2,
        .tx_pin = GPS_TX_PIN,
        .rx_pin = GPS_RX_PIN,
        .baud_rate = 9600,
        .update_rate_hz = 1
    };

    if (gps_init_with_config(&gps_config) == ESP_OK) {
        gps_register_data_callback(gps_callback, NULL);
        printf("GPS initialized on UART2 (TX: GPIO%d, RX: GPIO%d)\n", GPS_TX_PIN, GPS_RX_PIN);
    } else {
        printf("GPS initialization failed\n");
    }
}