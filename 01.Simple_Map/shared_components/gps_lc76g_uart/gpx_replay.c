/* GPX file replay for GPS testing - streaming version */

#include "gpx_replay.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "GPX_REPLAY";

// Trackpoint structure
typedef struct {
    double latitude;
    double longitude;
    float elevation;
    uint32_t timestamp;  // Seconds since midnight
} gpx_trackpoint_t;

// GPX replay state
static FILE *gpx_file = NULL;
static bool is_loaded = false;
static bool is_replaying = false;
static TaskHandle_t replay_task_handle = NULL;
static gps_data_callback_t replay_callback = NULL;
static void *replay_user_data = NULL;
static long first_trkpt_offset = 0;  // File offset of first trackpoint

// Parse ISO 8601 timestamp to seconds since midnight
static uint32_t parse_timestamp(const char *time_str)
{
    // Format: 2025-10-19T08:45:08Z
    int year, month, day, hour, minute, second;
    if (sscanf(time_str, "%d-%d-%dT%d:%d:%dZ", &year, &month, &day, &hour, &minute, &second) != 6) {
        return 0;
    }
    // Return seconds since midnight (for relative timing within a day)
    return hour * 3600 + minute * 60 + second;
}

// Read next trackpoint from file, returns false if EOF or error
static bool read_next_trackpoint(gpx_trackpoint_t *pt)
{
    if (!gpx_file) return false;

    char line[256];
    char lat_str[32] = {0}, lon_str[32] = {0};
    char ele_str[32] = {0}, time_str[32] = {0};
    bool in_trkpt = false;
    bool found_lat = false, found_lon = false;

    while (fgets(line, sizeof(line), gpx_file)) {
        // Look for trkpt start with lat/lon
        char *trkpt = strstr(line, "<trkpt");
        if (trkpt) {
            in_trkpt = true;
            // Extract lat
            char *lat_start = strstr(trkpt, "lat=\"");
            if (lat_start) {
                lat_start += 5;
                char *lat_end = strchr(lat_start, '"');
                if (lat_end) {
                    size_t len = lat_end - lat_start;
                    if (len < sizeof(lat_str)) {
                        strncpy(lat_str, lat_start, len);
                        lat_str[len] = '\0';
                        found_lat = true;
                    }
                }
            }
            // Extract lon
            char *lon_start = strstr(trkpt, "lon=\"");
            if (lon_start) {
                lon_start += 5;
                char *lon_end = strchr(lon_start, '"');
                if (lon_end) {
                    size_t len = lon_end - lon_start;
                    if (len < sizeof(lon_str)) {
                        strncpy(lon_str, lon_start, len);
                        lon_str[len] = '\0';
                        found_lon = true;
                    }
                }
            }
        }

        // Look for elevation
        if (in_trkpt) {
            char *ele_start = strstr(line, "<ele>");
            if (ele_start) {
                ele_start += 5;
                char *ele_end = strstr(ele_start, "</ele>");
                if (ele_end) {
                    size_t len = ele_end - ele_start;
                    if (len < sizeof(ele_str)) {
                        strncpy(ele_str, ele_start, len);
                        ele_str[len] = '\0';
                    }
                }
            }

            // Look for time
            char *time_start = strstr(line, "<time>");
            if (time_start) {
                time_start += 6;
                char *time_end = strstr(time_start, "</time>");
                if (time_end) {
                    size_t len = time_end - time_start;
                    if (len < sizeof(time_str)) {
                        strncpy(time_str, time_start, len);
                        time_str[len] = '\0';
                    }
                }
            }
        }

        // Look for trkpt end
        if (in_trkpt && strstr(line, "</trkpt>")) {
            if (found_lat && found_lon) {
                pt->latitude = atof(lat_str);
                pt->longitude = atof(lon_str);
                pt->elevation = ele_str[0] ? atof(ele_str) : 0;
                pt->timestamp = time_str[0] ? parse_timestamp(time_str) : 0;
                return true;
            }
            // Reset for next trackpoint
            in_trkpt = false;
            found_lat = false;
            found_lon = false;
            lat_str[0] = '\0';
            lon_str[0] = '\0';
            ele_str[0] = '\0';
            time_str[0] = '\0';
        }
    }

    return false;
}

// Find first trackpoint and save file position
static bool find_first_trackpoint(void)
{
    if (!gpx_file) return false;

    fseek(gpx_file, 0, SEEK_SET);

    char line[256];
    while (fgets(line, sizeof(line), gpx_file)) {
        if (strstr(line, "<trkpt")) {
            // Found first trackpoint, back up to start of this line
            first_trkpt_offset = ftell(gpx_file) - strlen(line);
            ESP_LOGI(TAG, "First trackpoint at offset %ld", first_trkpt_offset);
            fseek(gpx_file, first_trkpt_offset, SEEK_SET);
            return true;
        }
    }

    return false;
}

// Calculate heading between two points
static float calculate_heading(double lat1, double lon1, double lat2, double lon2)
{
    double dlon = (lon2 - lon1) * 0.0174533;
    double lat1_rad = lat1 * 0.0174533;
    double lat2_rad = lat2 * 0.0174533;

    double x = sin(dlon) * cos(lat2_rad);
    double y = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(dlon);

    double heading = atan2(x, y) * 57.2958;
    if (heading < 0) heading += 360;

    return (float)heading;
}

// Replay task
static void gpx_replay_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting GPX replay");

    gpx_trackpoint_t prev_pt = {0};
    gpx_trackpoint_t curr_pt = {0};
    uint32_t base_timestamp = 0;
    uint32_t start_time = esp_timer_get_time() / 1000;
    int point_count = 0;
    bool have_prev = false;

restart_replay:
    fseek(gpx_file, first_trkpt_offset, SEEK_SET);
    base_timestamp = 0;
    start_time = esp_timer_get_time() / 1000;
    point_count = 0;
    have_prev = false;

    while (is_replaying && read_next_trackpoint(&curr_pt)) {
        // Set base timestamp from first point
        if (base_timestamp == 0) {
            base_timestamp = curr_pt.timestamp;
        }

        // Calculate relative time for this point
        uint32_t rel_time_ms = (curr_pt.timestamp - base_timestamp) * 1000;

        // Wait until it's time to emit this point
        while (is_replaying) {
            uint32_t elapsed = (esp_timer_get_time() / 1000) - start_time;
            if (elapsed >= rel_time_ms) break;
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        if (!is_replaying) break;

        point_count++;

        // Build GPS data structure
        gps_data_t data = {0};
        data.valid = true;
        data.latitude = curr_pt.latitude;
        data.longitude = curr_pt.longitude;
        data.altitude = curr_pt.elevation;
        data.fix_type = GPS_FIX_3D;
        data.fix_quality = GPS_QUALITY_GPS;
        data.satellites_used = 8;
        data.satellites_visible = 12;
        data.hdop = 1.0;

        // Calculate speed and heading from previous point
        if (have_prev) {
            double dlat = curr_pt.latitude - prev_pt.latitude;
            double dlon = curr_pt.longitude - prev_pt.longitude;
            double dist_deg = sqrt(dlat * dlat + dlon * dlon);
            double dist_m = dist_deg * 111000;

            uint32_t dt = curr_pt.timestamp - prev_pt.timestamp;
            if (dt > 0) {
                data.speed_kmh = (dist_m / dt) * 3.6;
                data.speed_knots = data.speed_kmh / 1.852;
            }

            data.course = calculate_heading(prev_pt.latitude, prev_pt.longitude,
                                            curr_pt.latitude, curr_pt.longitude);
        }

        // Invoke callback
        if (replay_callback) {
            replay_callback(&data, replay_user_data);
        }

        // Log every 100 points
        if (point_count % 100 == 0) {
            ESP_LOGI(TAG, "Replay: %d points, pos: %.6f, %.6f",
                     point_count, curr_pt.latitude, curr_pt.longitude);
        }

        prev_pt = curr_pt;
        have_prev = true;
    }

    if (is_replaying) {
        ESP_LOGI(TAG, "GPX replay completed (%d points), looping...", point_count);
        goto restart_replay;
    }

    is_replaying = false;
    replay_task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t gpx_replay_init(const char *filepath)
{
    if (is_loaded) {
        gpx_replay_deinit();
    }

    gpx_file = fopen(filepath, "r");
    if (!gpx_file) {
        ESP_LOGE(TAG, "Failed to open GPX file: %s", filepath);
        return ESP_FAIL;
    }

    // Get file size
    fseek(gpx_file, 0, SEEK_END);
    long file_size = ftell(gpx_file);
    fseek(gpx_file, 0, SEEK_SET);

    ESP_LOGI(TAG, "Opened GPX file: %s (%ld bytes)", filepath, file_size);

    // Find first trackpoint
    if (!find_first_trackpoint()) {
        ESP_LOGE(TAG, "No trackpoints found in GPX file");
        fclose(gpx_file);
        gpx_file = NULL;
        return ESP_FAIL;
    }

    // Read first point to get start position
    gpx_trackpoint_t first_pt;
    if (read_next_trackpoint(&first_pt)) {
        ESP_LOGI(TAG, "First point: %.6f, %.6f", first_pt.latitude, first_pt.longitude);
    }

    is_loaded = true;
    return ESP_OK;
}

esp_err_t gpx_replay_start(gps_data_callback_t callback, void *user_data)
{
    if (!is_loaded || !gpx_file) {
        ESP_LOGE(TAG, "No GPX file loaded");
        return ESP_FAIL;
    }

    if (is_replaying) {
        ESP_LOGW(TAG, "Replay already in progress");
        return ESP_OK;
    }

    replay_callback = callback;
    replay_user_data = user_data;
    is_replaying = true;

    BaseType_t result = xTaskCreate(
        gpx_replay_task,
        "gpx_replay",
        4096,
        NULL,
        5,
        &replay_task_handle
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create replay task");
        is_replaying = false;
        return ESP_FAIL;
    }

    return ESP_OK;
}

void gpx_replay_stop(void)
{
    is_replaying = false;

    if (replay_task_handle) {
        vTaskDelay(pdMS_TO_TICKS(200));
        if (replay_task_handle) {
            vTaskDelete(replay_task_handle);
            replay_task_handle = NULL;
        }
    }

    ESP_LOGI(TAG, "GPX replay stopped");
}

bool gpx_replay_is_active(void)
{
    return is_replaying;
}

bool gpx_replay_is_loaded(void)
{
    return is_loaded;
}

void gpx_replay_deinit(void)
{
    gpx_replay_stop();

    if (gpx_file) {
        fclose(gpx_file);
        gpx_file = NULL;
    }

    is_loaded = false;
    first_trkpt_offset = 0;

    ESP_LOGI(TAG, "GPX replay deinitialized");
}
