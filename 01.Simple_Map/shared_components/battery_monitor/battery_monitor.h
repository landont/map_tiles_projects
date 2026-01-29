#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the battery monitor (AXP2101 PMIC)
 *
 * @param i2c_bus_handle I2C bus handle from BSP
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t battery_monitor_init(i2c_master_bus_handle_t i2c_bus_handle);

/**
 * @brief Start the battery monitoring task
 *
 * Creates a FreeRTOS task that periodically reads battery status
 * and updates the SimpleMap battery indicator.
 *
 * @param update_interval_ms Update interval in milliseconds (default 5000)
 * @return ESP_OK on success, ESP_FAIL on error
 */
esp_err_t battery_monitor_start(uint32_t update_interval_ms);

/**
 * @brief Stop the battery monitoring task
 */
void battery_monitor_stop(void);

/**
 * @brief Get battery percentage
 *
 * @return Battery percentage (0-100), or -1 if not available
 */
int battery_monitor_get_percent(void);

/**
 * @brief Check if battery is charging
 *
 * @return true if charging, false otherwise
 */
bool battery_monitor_is_charging(void);

/**
 * @brief Get battery voltage in mV
 *
 * @return Battery voltage in millivolts
 */
uint16_t battery_monitor_get_voltage(void);

#ifdef __cplusplus
}
#endif
