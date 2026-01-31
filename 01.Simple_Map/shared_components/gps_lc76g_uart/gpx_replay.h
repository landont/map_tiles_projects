#pragma once

#include "esp_err.h"
#include "gps_lc76g_uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize GPX replay from file
 *
 * Loads and parses the GPX file from SD card.
 *
 * @param filepath Path to GPX file (e.g., "/sdcard/tracklog.gpx")
 * @return ESP_OK if file loaded successfully, ESP_FAIL otherwise
 */
esp_err_t gpx_replay_init(const char *filepath);

/**
 * @brief Start GPX replay
 *
 * Begins replaying trackpoints with timing matching the original recording.
 * Calls the registered GPS data callback for each point.
 *
 * @param callback GPS data callback to invoke for each trackpoint
 * @param user_data User data to pass to callback
 * @return ESP_OK if replay started, ESP_FAIL otherwise
 */
esp_err_t gpx_replay_start(gps_data_callback_t callback, void *user_data);

/**
 * @brief Stop GPX replay
 */
void gpx_replay_stop(void);

/**
 * @brief Check if GPX replay is active
 *
 * @return true if replaying, false otherwise
 */
bool gpx_replay_is_active(void);

/**
 * @brief Check if GPX file is loaded and ready
 *
 * @return true if file is loaded, false otherwise
 */
bool gpx_replay_is_loaded(void);

/**
 * @brief Free GPX replay resources
 */
void gpx_replay_deinit(void);

#ifdef __cplusplus
}
#endif
