/* GPX file replay for GPS testing */

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
    uint32_t timestamp;  // Seconds since start of track
} gpx_trackpoint_t;

// GPX replay state
static gpx_trackpoint_t *trackpoints = NULL;
static int num_trackpoints = 0;
static int max_trackpoints = 0;
static bool is_loaded = false;
static bool is_replaying = false;
static TaskHandle_t replay_task_handle = NULL;
static gps_data_callback_t replay_callback = NULL;
static void *replay_user_data = NULL;

// Parse ISO 8601 timestamp to seconds since epoch (simplified)
static uint32_t parse_timestamp(const char *time_str)
{
    // Format: 2025-10-19T08:45:08Z
    int year, month, day, hour, minute, second;
    if (sscanf(time_str, "%d-%d-%dT%d:%d:%dZ", &year, &month, &day, &hour, &minute, &second) != 6) {
        return 0;
    }

    // Simple conversion to seconds (not accurate for all dates, but good enough for relative timing)
    uint32_t days = (year - 2000) * 365 + (month - 1) * 30 + day;
    return days * 86400 + hour * 3600 + minute * 60 + second;
}

// Extract attribute value from XML tag
static bool get_xml_attr(const char *tag, const char *attr, char *value, size_t value_len)
{
    char search[32];
    snprintf(search, sizeof(search), "%s=\"", attr);

    const char *start = strstr(tag, search);
    if (!start) return false;

    start += strlen(search);
    const char *end = strchr(start, '"');
    if (!end) return false;

    size_t len = end - start;
    if (len >= value_len) len = value_len - 1;

    strncpy(value, start, len);
    value[len] = '\0';
    return true;
}

// Extract content between XML tags
static bool get_xml_content(const char *xml, const char *tag, char *value, size_t value_len)
{
    char open_tag[32], close_tag[32];
    snprintf(open_tag, sizeof(open_tag), "<%s>", tag);
    snprintf(close_tag, sizeof(close_tag), "</%s>", tag);

    const char *start = strstr(xml, open_tag);
    if (!start) return false;

    start += strlen(open_tag);
    const char *end = strstr(start, close_tag);
    if (!end) return false;

    size_t len = end - start;
    if (len >= value_len) len = value_len - 1;

    strncpy(value, start, len);
    value[len] = '\0';
    return true;
}

// Add a trackpoint to the array
static bool add_trackpoint(double lat, double lon, float ele, uint32_t timestamp)
{
    if (num_trackpoints >= max_trackpoints) {
        // Grow array
        int new_max = max_trackpoints == 0 ? 256 : max_trackpoints * 2;
        gpx_trackpoint_t *new_array = realloc(trackpoints, new_max * sizeof(gpx_trackpoint_t));
        if (!new_array) {
            ESP_LOGE(TAG, "Failed to allocate memory for trackpoints");
            return false;
        }
        trackpoints = new_array;
        max_trackpoints = new_max;
    }

    trackpoints[num_trackpoints].latitude = lat;
    trackpoints[num_trackpoints].longitude = lon;
    trackpoints[num_trackpoints].elevation = ele;
    trackpoints[num_trackpoints].timestamp = timestamp;
    num_trackpoints++;

    return true;
}

// Parse GPX file
static esp_err_t parse_gpx_file(const char *filepath)
{
    FILE *f = fopen(filepath, "r");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open GPX file: %s", filepath);
        return ESP_FAIL;
    }

    // Get file size
    fseek(f, 0, SEEK_END);
    long file_size = ftell(f);
    fseek(f, 0, SEEK_SET);

    ESP_LOGI(TAG, "Parsing GPX file: %s (%ld bytes)", filepath, file_size);

    // Read file in chunks and parse trackpoints
    char buffer[512];
    char trkpt_buffer[1024];
    int trkpt_idx = 0;
    bool in_trkpt = false;
    uint32_t first_timestamp = 0;

    while (fgets(buffer, sizeof(buffer), f)) {
        // Look for trkpt start
        char *trkpt_start = strstr(buffer, "<trkpt");
        if (trkpt_start) {
            in_trkpt = true;
            trkpt_idx = 0;
        }

        // Accumulate trkpt content
        if (in_trkpt) {
            size_t len = strlen(buffer);
            if (trkpt_idx + len < sizeof(trkpt_buffer) - 1) {
                strcpy(trkpt_buffer + trkpt_idx, buffer);
                trkpt_idx += len;
            }
        }

        // Look for trkpt end
        if (in_trkpt && strstr(buffer, "</trkpt>")) {
            in_trkpt = false;
            trkpt_buffer[trkpt_idx] = '\0';

            // Parse this trackpoint
            char lat_str[32], lon_str[32], ele_str[32], time_str[32];

            // Find the trkpt tag and extract lat/lon
            char *trkpt_tag = strstr(trkpt_buffer, "<trkpt");
            if (trkpt_tag && get_xml_attr(trkpt_tag, "lat", lat_str, sizeof(lat_str)) &&
                get_xml_attr(trkpt_tag, "lon", lon_str, sizeof(lon_str))) {

                double lat = atof(lat_str);
                double lon = atof(lon_str);
                float ele = 0;
                uint32_t timestamp = 0;

                if (get_xml_content(trkpt_buffer, "ele", ele_str, sizeof(ele_str))) {
                    ele = atof(ele_str);
                }

                if (get_xml_content(trkpt_buffer, "time", time_str, sizeof(time_str))) {
                    timestamp = parse_timestamp(time_str);
                    if (first_timestamp == 0) {
                        first_timestamp = timestamp;
                    }
                    timestamp -= first_timestamp;  // Convert to relative time
                }

                add_trackpoint(lat, lon, ele, timestamp);
            }
        }
    }

    fclose(f);

    ESP_LOGI(TAG, "Loaded %d trackpoints from GPX file", num_trackpoints);

    if (num_trackpoints > 0) {
        ESP_LOGI(TAG, "First point: %.6f, %.6f", trackpoints[0].latitude, trackpoints[0].longitude);
        ESP_LOGI(TAG, "Last point: %.6f, %.6f",
                 trackpoints[num_trackpoints-1].latitude,
                 trackpoints[num_trackpoints-1].longitude);
        ESP_LOGI(TAG, "Track duration: %lu seconds",
                 trackpoints[num_trackpoints-1].timestamp);
    }

    return num_trackpoints > 0 ? ESP_OK : ESP_FAIL;
}

// Calculate heading between two points
static float calculate_heading(double lat1, double lon1, double lat2, double lon2)
{
    double dlon = (lon2 - lon1) * 0.0174533;  // Convert to radians
    double lat1_rad = lat1 * 0.0174533;
    double lat2_rad = lat2 * 0.0174533;

    double x = sin(dlon) * cos(lat2_rad);
    double y = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(dlon);

    double heading = atan2(x, y) * 57.2958;  // Convert to degrees
    if (heading < 0) heading += 360;

    return (float)heading;
}

// Replay task
static void gpx_replay_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting GPX replay (%d points)", num_trackpoints);

    int current_point = 0;
    uint32_t start_time = esp_timer_get_time() / 1000;

    while (is_replaying && current_point < num_trackpoints) {
        uint32_t now = esp_timer_get_time() / 1000;
        uint32_t elapsed = now - start_time;

        // Find the point that matches current elapsed time
        while (current_point < num_trackpoints &&
               trackpoints[current_point].timestamp * 1000 <= elapsed) {

            // Build GPS data structure
            gps_data_t data = {0};
            data.valid = true;
            data.latitude = trackpoints[current_point].latitude;
            data.longitude = trackpoints[current_point].longitude;
            data.altitude = trackpoints[current_point].elevation;
            data.fix_type = GPS_FIX_3D;
            data.fix_quality = GPS_QUALITY_GPS;
            data.satellites_used = 8;  // Simulated
            data.satellites_visible = 12;
            data.hdop = 1.0;

            // Calculate speed from distance/time between points
            if (current_point > 0) {
                gpx_trackpoint_t *prev = &trackpoints[current_point - 1];
                gpx_trackpoint_t *curr = &trackpoints[current_point];

                // Simple distance calculation (approximate)
                double dlat = curr->latitude - prev->latitude;
                double dlon = curr->longitude - prev->longitude;
                double dist_deg = sqrt(dlat * dlat + dlon * dlon);
                double dist_m = dist_deg * 111000;  // Rough meters per degree

                uint32_t dt = curr->timestamp - prev->timestamp;
                if (dt > 0) {
                    data.speed_kmh = (dist_m / dt) * 3.6;
                    data.speed_knots = data.speed_kmh / 1.852;
                }

                // Calculate heading
                data.course = calculate_heading(prev->latitude, prev->longitude,
                                                 curr->latitude, curr->longitude);
            }

            // Invoke callback
            if (replay_callback) {
                replay_callback(&data, replay_user_data);
            }

            current_point++;
        }

        // Sleep a bit before checking again
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (current_point >= num_trackpoints) {
        ESP_LOGI(TAG, "GPX replay completed");

        // Loop back to start
        current_point = 0;
        start_time = esp_timer_get_time() / 1000;

        if (is_replaying) {
            ESP_LOGI(TAG, "Looping GPX replay...");
            // Recursively continue replay by restarting the task logic
            // For simplicity, just restart from beginning
            gpx_replay_task(NULL);
            return;
        }
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

    esp_err_t ret = parse_gpx_file(filepath);
    if (ret == ESP_OK) {
        is_loaded = true;
    }

    return ret;
}

esp_err_t gpx_replay_start(gps_data_callback_t callback, void *user_data)
{
    if (!is_loaded || num_trackpoints == 0) {
        ESP_LOGE(TAG, "No GPX data loaded");
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
        // Give task time to exit gracefully
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

    if (trackpoints) {
        free(trackpoints);
        trackpoints = NULL;
    }

    num_trackpoints = 0;
    max_trackpoints = 0;
    is_loaded = false;

    ESP_LOGI(TAG, "GPX replay deinitialized");
}
