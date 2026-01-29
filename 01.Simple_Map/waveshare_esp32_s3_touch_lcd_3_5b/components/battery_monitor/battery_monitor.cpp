#include "battery_monitor.h"
#include <stdio.h>
#include <cstring>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "bsp/esp32_s3_touch_lcd_3_5b.h"
#include "simple_map.hpp"

#define XPOWERS_CHIP_AXP2101
#include "XPowersLib.h"

static const char *TAG = "battery_monitor";

// AXP2101 I2C address
#define AXP2101_I2C_ADDR 0x34

// Global state
static XPowersPMU power;
static i2c_master_dev_handle_t i2c_device = nullptr;
static TaskHandle_t monitor_task_handle = nullptr;
static uint32_t update_interval = 5000;
static bool initialized = false;
static bool backlight_dimmed = false;
static const uint32_t BACKLIGHT_DIM_TIMEOUT_MS = 15000;  // 15 seconds

// I2C read/write callbacks for XPowersLib
static int pmu_register_read(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t len)
{
    if (len == 0 || data == nullptr || i2c_device == nullptr) {
        return -1;
    }

    esp_err_t ret = i2c_master_transmit_receive(i2c_device, &regAddr, 1, data, len, pdMS_TO_TICKS(1000));
    return (ret == ESP_OK) ? 0 : -1;
}

static int pmu_register_write_byte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t len)
{
    if (data == nullptr || i2c_device == nullptr) {
        return -1;
    }

    uint8_t *write_buffer = (uint8_t *)malloc(len + 1);
    if (!write_buffer) {
        return -1;
    }

    write_buffer[0] = regAddr;
    memcpy(write_buffer + 1, data, len);

    esp_err_t ret = i2c_master_transmit(i2c_device, write_buffer, len + 1, pdMS_TO_TICKS(1000));
    free(write_buffer);
    return (ret == ESP_OK) ? 0 : -1;
}

// Battery monitoring task
static void battery_monitor_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Battery monitor task started (interval: %lu ms)", update_interval);

    while (1) {
        if (initialized) {
            // Read battery status
            int percent = -1;
            bool charging = false;

            if (power.isBatteryConnect()) {
                percent = power.getBatteryPercent();
                uint16_t voltage = power.getBattVoltage();

                // Check charging status - isVbusIn indicates external power connected
                bool vbus_in = power.isVbusIn();
                bool is_charging_state = power.isCharging();
                bool is_discharging = power.isDischarge();

                // Use isVbusIn as primary indicator - if USB power connected, we're charging
                charging = vbus_in;

                ESP_LOGI(TAG, "Battery: %d%%, %umV, vbus=%d, charging=%d, discharging=%d -> %s",
                         percent, voltage, vbus_in, is_charging_state, is_discharging,
                         charging ? "CHARGING" : "ON BATTERY");
            } else {
                ESP_LOGD(TAG, "No battery connected");
            }

            // Update the SimpleMap battery indicator
            // Need to take LVGL mutex since we're updating UI from a task
            if (bsp_display_lock(100)) {
                SimpleMap::update_battery_indicator(percent, charging);
                bsp_display_unlock();
            } else {
                ESP_LOGW(TAG, "Failed to acquire LVGL mutex for battery update");
            }

            // Check for backlight dimming based on touch inactivity
            uint32_t current_time = esp_timer_get_time() / 1000;
            uint32_t last_touch = SimpleMap::get_last_touch_time();
            uint32_t idle_time = current_time - last_touch;

            if (idle_time >= BACKLIGHT_DIM_TIMEOUT_MS && !backlight_dimmed) {
                // Dim backlight to 75% after 15 seconds of inactivity
                bsp_display_brightness_set(75);
                backlight_dimmed = true;
                ESP_LOGI(TAG, "Backlight dimmed to 75%% (idle for %lu ms)", idle_time);
            } else if (idle_time < BACKLIGHT_DIM_TIMEOUT_MS && backlight_dimmed) {
                // Touch event restored brightness, reset flag
                backlight_dimmed = false;
                ESP_LOGI(TAG, "Backlight restored to 100%%");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(update_interval));
    }
}

esp_err_t battery_monitor_init(i2c_master_bus_handle_t i2c_bus_handle)
{
    if (initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing AXP2101 PMIC...");

    // Add AXP2101 device to I2C bus
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = AXP2101_I2C_ADDR;
    dev_cfg.scl_speed_hz = 400000;

    esp_err_t ret = i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &i2c_device);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add AXP2101 to I2C bus: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    // Initialize XPowersLib with our callbacks
    if (!power.begin(AXP2101_SLAVE_ADDRESS, pmu_register_read, pmu_register_write_byte)) {
        ESP_LOGE(TAG, "Failed to initialize AXP2101");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "AXP2101 initialized, chip ID: 0x%x", power.getChipID());

    // Configure PMIC for battery monitoring
    // Disable TS pin measurement (no battery temperature sensor)
    power.disableTSPinMeasure();

    // Enable battery detection and voltage measurement
    power.enableBattDetection();
    power.enableBattVoltageMeasure();
    power.enableVbusVoltageMeasure();
    power.enableSystemVoltageMeasure();

    // Set charging parameters (safe defaults)
    power.setPrechargeCurr(XPOWERS_AXP2101_PRECHARGE_50MA);
    power.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_200MA);
    power.setChargerTerminationCurr(XPOWERS_AXP2101_CHG_ITERM_25MA);
    power.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V1);

    // Log initial status
    if (power.isBatteryConnect()) {
        ESP_LOGI(TAG, "Battery connected: %d%%, %umV",
                 power.getBatteryPercent(), power.getBattVoltage());
    } else {
        ESP_LOGI(TAG, "No battery connected");
    }

    initialized = true;
    return ESP_OK;
}

esp_err_t battery_monitor_start(uint32_t update_interval_ms)
{
    if (!initialized) {
        ESP_LOGE(TAG, "Not initialized, call battery_monitor_init() first");
        return ESP_FAIL;
    }

    if (monitor_task_handle != nullptr) {
        ESP_LOGW(TAG, "Monitor task already running");
        return ESP_OK;
    }

    update_interval = update_interval_ms;

    BaseType_t result = xTaskCreate(
        battery_monitor_task,
        "battery_mon",
        8192,
        nullptr,
        5,  // Priority
        &monitor_task_handle
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create battery monitor task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Battery monitor started (update every %lu ms)", update_interval);
    return ESP_OK;
}

void battery_monitor_stop(void)
{
    if (monitor_task_handle != nullptr) {
        vTaskDelete(monitor_task_handle);
        monitor_task_handle = nullptr;
        ESP_LOGI(TAG, "Battery monitor stopped");
    }
}

int battery_monitor_get_percent(void)
{
    if (!initialized || !power.isBatteryConnect()) {
        return -1;
    }
    return power.getBatteryPercent();
}

bool battery_monitor_is_charging(void)
{
    if (!initialized) {
        return false;
    }
    return power.isCharging();
}

uint16_t battery_monitor_get_voltage(void)
{
    if (!initialized) {
        return 0;
    }
    return power.getBattVoltage();
}
