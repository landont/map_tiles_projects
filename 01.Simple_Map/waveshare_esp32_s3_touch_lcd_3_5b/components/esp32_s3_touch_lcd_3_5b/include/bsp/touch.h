#pragma once

#include "esp_lcd_touch.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief BSP touch configuration structure
 */
typedef struct {
    void *dummy;    /*!< Placeholder for future configuration options */
} bsp_touch_config_t;

/**
 * @brief Create new touch device
 *
 * @param[in]  config      Touch configuration
 * @param[out] ret_touch   Touch device handle
 * @return
 *      - ESP_OK         On success
 *      - ESP_FAIL       Touch initialization failed
 */
esp_err_t bsp_touch_new(const bsp_touch_config_t *config, esp_lcd_touch_handle_t *ret_touch);

#ifdef __cplusplus
}
#endif
