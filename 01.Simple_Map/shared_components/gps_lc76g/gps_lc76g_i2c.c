/* LC76G GPS module implementation - I2C version */

#include "gps_lc76g_i2c.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "GPS_LC76G_I2C";

static bool gps_initialized = false;
static gps_data_t current_gps_data = {0};
static i2c_master_bus_handle_t i2c_bus_handle = NULL;  // Store bus handle for recovery
static i2c_master_dev_handle_t i2c_dev_write = NULL;  // Device handle for 0x50 (write)
static i2c_master_dev_handle_t i2c_dev_read = NULL;   // Device handle for 0x54 (read)
static TaskHandle_t gps_task_handle = NULL;
static int consecutive_errors = 0;
static SemaphoreHandle_t i2c_mutex = NULL;  // Mutex for I2C access
static gps_data_callback_t data_callback = NULL;
static void *data_callback_user_data = NULL;
static gps_nmea_callback_t nmea_callback = NULL;
static void *nmea_callback_user_data = NULL;
static uint32_t poll_interval = 500;

// NMEA parsing helpers
static uint8_t nmea_checksum(const char *sentence)
{
    uint8_t checksum = 0;
    const char *p = sentence;

    if (*p == '$') p++;

    while (*p && *p != '*') {
        checksum ^= *p;
        p++;
    }

    return checksum;
}

static bool nmea_verify_checksum(const char *sentence)
{
    const char *p = strchr(sentence, '*');
    if (!p) return false;

    uint8_t expected = nmea_checksum(sentence);
    uint8_t actual = (uint8_t)strtol(p + 1, NULL, 16);

    return expected == actual;
}

static double nmea_to_degrees(const char *coord, char direction)
{
    if (!coord || strlen(coord) < 4) return 0.0;

    char *dot = strchr(coord, '.');
    if (!dot) return 0.0;

    int deg_len = (direction == 'N' || direction == 'S') ? 2 : 3;

    char deg_str[4] = {0};
    strncpy(deg_str, coord, deg_len);
    double degrees = atof(deg_str);

    double minutes = atof(coord + deg_len);

    double result = degrees + (minutes / 60.0);

    if (direction == 'S' || direction == 'W') {
        result = -result;
    }

    return result;
}

static void parse_nmea_time(const char *time_str, gps_datetime_t *datetime)
{
    if (!time_str || strlen(time_str) < 6) return;

    char buf[3] = {0};

    buf[0] = time_str[0];
    buf[1] = time_str[1];
    datetime->hour = atoi(buf);

    buf[0] = time_str[2];
    buf[1] = time_str[3];
    datetime->minute = atoi(buf);

    buf[0] = time_str[4];
    buf[1] = time_str[5];
    datetime->second = atoi(buf);

    const char *dot = strchr(time_str, '.');
    if (dot) {
        datetime->millisecond = (uint16_t)(atof(dot) * 1000);
    }
}

static void parse_nmea_date(const char *date_str, gps_datetime_t *datetime)
{
    if (!date_str || strlen(date_str) < 6) return;

    char buf[3] = {0};

    buf[0] = date_str[0];
    buf[1] = date_str[1];
    datetime->day = atoi(buf);

    buf[0] = date_str[2];
    buf[1] = date_str[3];
    datetime->month = atoi(buf);

    buf[0] = date_str[4];
    buf[1] = date_str[5];
    datetime->year = 2000 + atoi(buf);
}

static bool get_nmea_field(const char *sentence, int field_num, char *buffer, size_t buffer_len)
{
    if (!sentence || !buffer || buffer_len == 0) return false;

    const char *p = sentence;
    int current_field = 0;

    while (*p && current_field < field_num) {
        if (*p == ',') current_field++;
        p++;
    }

    if (current_field != field_num) return false;

    size_t i = 0;
    while (*p && *p != ',' && *p != '*' && i < buffer_len - 1) {
        buffer[i++] = *p++;
    }
    buffer[i] = '\0';

    return i > 0;
}

static void parse_gga(const char *sentence)
{
    char field[32];

    if (get_nmea_field(sentence, 1, field, sizeof(field))) {
        parse_nmea_time(field, &current_gps_data.datetime);
    }

    if (get_nmea_field(sentence, 2, field, sizeof(field)) && strlen(field) > 0) {
        char dir[2];
        if (get_nmea_field(sentence, 3, dir, sizeof(dir))) {
            current_gps_data.latitude = nmea_to_degrees(field, dir[0]);
        }
    }

    if (get_nmea_field(sentence, 4, field, sizeof(field)) && strlen(field) > 0) {
        char dir[2];
        if (get_nmea_field(sentence, 5, dir, sizeof(dir))) {
            current_gps_data.longitude = nmea_to_degrees(field, dir[0]);
        }
    }

    if (get_nmea_field(sentence, 6, field, sizeof(field))) {
        current_gps_data.fix_quality = (gps_quality_t)atoi(field);
        current_gps_data.valid = (current_gps_data.fix_quality > GPS_QUALITY_INVALID);
    }

    if (get_nmea_field(sentence, 7, field, sizeof(field))) {
        current_gps_data.satellites_used = atoi(field);
    }

    if (get_nmea_field(sentence, 8, field, sizeof(field))) {
        current_gps_data.hdop = atof(field);
    }

    if (get_nmea_field(sentence, 9, field, sizeof(field))) {
        current_gps_data.altitude = atof(field);
    }

    if (current_gps_data.valid) {
        current_gps_data.last_update_ms = esp_timer_get_time() / 1000;
    }
}

static void parse_rmc(const char *sentence)
{
    char field[32];

    if (get_nmea_field(sentence, 1, field, sizeof(field))) {
        parse_nmea_time(field, &current_gps_data.datetime);
    }

    bool status_valid = false;
    if (get_nmea_field(sentence, 2, field, sizeof(field))) {
        status_valid = (field[0] == 'A');
        current_gps_data.valid = status_valid;
    }

    if (get_nmea_field(sentence, 3, field, sizeof(field)) && strlen(field) > 0) {
        char dir[2];
        if (get_nmea_field(sentence, 4, dir, sizeof(dir))) {
            current_gps_data.latitude = nmea_to_degrees(field, dir[0]);
        }
    }

    if (get_nmea_field(sentence, 5, field, sizeof(field)) && strlen(field) > 0) {
        char dir[2];
        if (get_nmea_field(sentence, 6, dir, sizeof(dir))) {
            current_gps_data.longitude = nmea_to_degrees(field, dir[0]);
        }
    }

    if (get_nmea_field(sentence, 7, field, sizeof(field))) {
        current_gps_data.speed_knots = atof(field);
        current_gps_data.speed_kmh = current_gps_data.speed_knots * 1.852f;
    }

    if (get_nmea_field(sentence, 8, field, sizeof(field))) {
        current_gps_data.course = atof(field);
    }

    if (get_nmea_field(sentence, 9, field, sizeof(field))) {
        parse_nmea_date(field, &current_gps_data.datetime);
    }

    current_gps_data.last_update_ms = esp_timer_get_time() / 1000;

    if (status_valid && data_callback) {
        data_callback(&current_gps_data, data_callback_user_data);
    }
}

static void parse_gsa(const char *sentence)
{
    char field[32];

    if (get_nmea_field(sentence, 2, field, sizeof(field))) {
        current_gps_data.fix_type = (gps_fix_type_t)atoi(field);
    }

    if (get_nmea_field(sentence, 15, field, sizeof(field))) {
        current_gps_data.pdop = atof(field);
    }

    if (get_nmea_field(sentence, 16, field, sizeof(field))) {
        current_gps_data.hdop = atof(field);
    }

    if (get_nmea_field(sentence, 17, field, sizeof(field))) {
        current_gps_data.vdop = atof(field);
    }
}

static void parse_gsv(const char *sentence)
{
    char field[32];

    if (get_nmea_field(sentence, 3, field, sizeof(field))) {
        current_gps_data.satellites_visible = atoi(field);
    }
}

static void parse_nmea_sentence(const char *sentence)
{
    ESP_LOGD(TAG, "NMEA: %s", sentence);

    if (!nmea_verify_checksum(sentence)) {
        ESP_LOGW(TAG, "Invalid checksum: %s", sentence);
        return;
    }

    if (nmea_callback) {
        nmea_callback(sentence, nmea_callback_user_data);
    }

    if (strstr(sentence, "GGA")) {
        parse_gga(sentence);
    } else if (strstr(sentence, "RMC")) {
        parse_rmc(sentence);

        if (data_callback && current_gps_data.valid) {
            data_callback(&current_gps_data, data_callback_user_data);
        }
    } else if (strstr(sentence, "GSA")) {
        parse_gsa(sentence);
    } else if (strstr(sentence, "GSV")) {
        parse_gsv(sentence);
    }
}

// Read NMEA data from LC76G via I2C
static esp_err_t gps_i2c_read_nmea(char *buffer, size_t buffer_size, size_t *bytes_read)
{
    esp_err_t ret;
    uint8_t read_length[4] = {0};

    *bytes_read = 0;

    if (i2c_dev_write == NULL || i2c_dev_read == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Take mutex to prevent I2C bus contention
    if (i2c_mutex && xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire I2C mutex");
        return ESP_ERR_TIMEOUT;
    }

    // Small delay to ensure I2C bus is idle
    vTaskDelay(pdMS_TO_TICKS(10));

    // Step 1: Send init command to write address (0x50)
    uint8_t init_cmd[] = {0x08, 0x00, 0x51, 0xAA, 0x04, 0x00, 0x00, 0x00};
    ret = i2c_master_transmit(i2c_dev_write, init_cmd, sizeof(init_cmd), pdMS_TO_TICKS(500));
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "Init cmd failed: %s", esp_err_to_name(ret));
        if (i2c_mutex) xSemaphoreGive(i2c_mutex);
        return ret;
    }

    // Allow GPS module time to process init command
    vTaskDelay(pdMS_TO_TICKS(100));

    // Step 2: Read data length (4 bytes) from read address (0x54)
    ret = i2c_master_receive(i2c_dev_read, read_length, 4, pdMS_TO_TICKS(500));
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "Read length failed: %s", esp_err_to_name(ret));
        if (i2c_mutex) xSemaphoreGive(i2c_mutex);
        return ret;
    }

    uint32_t data_len = read_length[0] | (read_length[1] << 8) | (read_length[2] << 16) | (read_length[3] << 24);
    ESP_LOGI(TAG, "Read length bytes: %02X %02X %02X %02X = %lu",
             read_length[0], read_length[1], read_length[2], read_length[3], data_len);

    if (data_len == 0) {
        // No data available yet
        if (i2c_mutex) xSemaphoreGive(i2c_mutex);
        return ESP_OK;
    }

    if (data_len > 2000) {
        ESP_LOGW(TAG, "Invalid data length: %lu (too large)", data_len);
        if (i2c_mutex) xSemaphoreGive(i2c_mutex);
        return ESP_OK;
    }

    if (data_len > buffer_size - 1) {
        data_len = buffer_size - 1;
    }

    ESP_LOGI(TAG, "GPS data available: %lu bytes", data_len);

    // Step 3: Send read command with length to write address (0x50)
    uint8_t read_cmd[8];
    read_cmd[0] = 0x00;
    read_cmd[1] = 0x20;
    read_cmd[2] = 0x51;
    read_cmd[3] = 0xAA;
    memcpy(&read_cmd[4], read_length, 4);

    ret = i2c_master_transmit(i2c_dev_write, read_cmd, sizeof(read_cmd), pdMS_TO_TICKS(500));
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "Failed to send read command: %s", esp_err_to_name(ret));
        if (i2c_mutex) xSemaphoreGive(i2c_mutex);
        return ret;
    }

    // Allow GPS module time to prepare data
    vTaskDelay(pdMS_TO_TICKS(100));

    // Step 4: Read NMEA data from read address (0x54)
    ret = i2c_master_receive(i2c_dev_read, (uint8_t *)buffer, data_len, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read NMEA data (%lu bytes): %s", data_len, esp_err_to_name(ret));
        if (i2c_mutex) xSemaphoreGive(i2c_mutex);
        return ret;
    }

    buffer[data_len] = '\0';
    *bytes_read = data_len;

    // Release mutex before logging
    if (i2c_mutex) xSemaphoreGive(i2c_mutex);

    ESP_LOGI(TAG, "Successfully read %lu bytes of NMEA data", data_len);

    return ESP_OK;
}

// GPS polling task
static void gps_poll_task(void *pvParameters)
{
    char *nmea_buffer = (char *)malloc(2048);
    if (!nmea_buffer) {
        ESP_LOGE(TAG, "Failed to allocate NMEA buffer");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "GPS poll task started (interval: %lu ms)", poll_interval);

    // Initial delay to let other I2C devices initialize
    vTaskDelay(pdMS_TO_TICKS(500));

    while (1) {
        size_t bytes_read = 0;
        esp_err_t ret = gps_i2c_read_nmea(nmea_buffer, 2047, &bytes_read);

        if (ret == ESP_OK && bytes_read > 0) {
            consecutive_errors = 0;  // Reset error counter on success

            // Log first 60 chars of data for debugging
            char preview[61];
            size_t preview_len = bytes_read < 60 ? bytes_read : 60;
            memcpy(preview, nmea_buffer, preview_len);
            preview[preview_len] = '\0';
            // Replace newlines with spaces for readable log
            for (size_t i = 0; i < preview_len; i++) {
                if (preview[i] == '\r' || preview[i] == '\n') preview[i] = ' ';
            }
            ESP_LOGI(TAG, "NMEA preview: %s...", preview);

            // Parse NMEA sentences (separated by newlines)
            int sentence_count = 0;
            char *line = strtok(nmea_buffer, "\r\n");
            while (line != NULL) {
                if (line[0] == '$') {
                    parse_nmea_sentence(line);
                    sentence_count++;
                }
                line = strtok(NULL, "\r\n");
            }
            ESP_LOGI(TAG, "Parsed %d NMEA sentences", sentence_count);
        } else if (ret == ESP_OK && bytes_read == 0) {
            // No data available, this is normal
            consecutive_errors = 0;
        } else {
            // Error occurred
            consecutive_errors++;
            ESP_LOGI(TAG, "GPS read error %d: %s", consecutive_errors, esp_err_to_name(ret));

            if (consecutive_errors >= 5) {
                ESP_LOGW(TAG, "Persistent GPS errors, performing full I2C recovery...");

                // Take mutex before recovery
                if (i2c_mutex) xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(1000));

                // Remove old device handles
                if (i2c_dev_read) {
                    i2c_master_bus_rm_device(i2c_dev_read);
                    i2c_dev_read = NULL;
                }
                if (i2c_dev_write) {
                    i2c_master_bus_rm_device(i2c_dev_write);
                    i2c_dev_write = NULL;
                }

                vTaskDelay(pdMS_TO_TICKS(100));

                // Reset bus via driver multiple times
                for (int i = 0; i < 3; i++) {
                    i2c_master_bus_reset(i2c_bus_handle);
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
                ESP_LOGI(TAG, "I2C bus reset 3x complete");

                // Recreate write device
                i2c_device_config_t dev_cfg_write = {
                    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                    .device_address = GPS_I2C_ADDR_WRITE,
                    .scl_speed_hz = 10000,
                };
                esp_err_t add_ret = i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg_write, &i2c_dev_write);
                if (add_ret == ESP_OK) {
                    ESP_LOGI(TAG, "Write device (0x%02X) recreated", GPS_I2C_ADDR_WRITE);
                } else {
                    ESP_LOGE(TAG, "Failed to recreate write device: %s", esp_err_to_name(add_ret));
                }

                vTaskDelay(pdMS_TO_TICKS(100));

                // Recreate read device
                i2c_device_config_t dev_cfg_read = {
                    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                    .device_address = GPS_I2C_ADDR_READ,
                    .scl_speed_hz = 10000,
                };
                add_ret = i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg_read, &i2c_dev_read);
                if (add_ret == ESP_OK) {
                    ESP_LOGI(TAG, "Read device (0x%02X) recreated", GPS_I2C_ADDR_READ);
                } else {
                    ESP_LOGE(TAG, "Failed to recreate read device: %s", esp_err_to_name(add_ret));
                }

                vTaskDelay(pdMS_TO_TICKS(200));

                if (i2c_mutex) xSemaphoreGive(i2c_mutex);

                consecutive_errors = 0;
                ESP_LOGI(TAG, "GPS recovery complete");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(poll_interval));
    }

    free(nmea_buffer);
    vTaskDelete(NULL);
}

esp_err_t gps_i2c_init(i2c_master_bus_handle_t i2c_bus)
{
    if (gps_initialized) {
        ESP_LOGW(TAG, "GPS already initialized");
        return ESP_OK;
    }

    if (i2c_bus == NULL) {
        ESP_LOGE(TAG, "I2C bus handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Initializing LC76G GNSS module via I2C...");

    // Save bus handle for potential recovery
    i2c_bus_handle = i2c_bus;

    // Create mutex for I2C access serialization
    if (i2c_mutex == NULL) {
        i2c_mutex = xSemaphoreCreateMutex();
        if (i2c_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create I2C mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    // Add GPS write device at address 0x50
    i2c_device_config_t dev_cfg_write = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = GPS_I2C_ADDR_WRITE,
        .scl_speed_hz = 10000,
    };

    esp_err_t ret = i2c_master_bus_add_device(i2c_bus, &dev_cfg_write, &i2c_dev_write);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add GPS write device: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Added write device at 0x%02X", GPS_I2C_ADDR_WRITE);

    // Add GPS read device at address 0x54
    i2c_device_config_t dev_cfg_read = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = GPS_I2C_ADDR_READ,
        .scl_speed_hz = 10000,
    };

    ret = i2c_master_bus_add_device(i2c_bus, &dev_cfg_read, &i2c_dev_read);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add GPS read device: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Added read device at 0x%02X", GPS_I2C_ADDR_READ);

    // Probe to verify GPS module exists at write address
    ret = i2c_master_probe(i2c_bus, GPS_I2C_ADDR_WRITE, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPS module not found at address 0x%02X: %s", GPS_I2C_ADDR_WRITE, esp_err_to_name(ret));
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "GPS module found at address 0x%02X", GPS_I2C_ADDR_WRITE);

    // Allow I2C devices to stabilize after creation
    vTaskDelay(pdMS_TO_TICKS(200));

    memset(&current_gps_data, 0, sizeof(current_gps_data));

    gps_initialized = true;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "GPS module initialized successfully!");
    ESP_LOGI(TAG, "I2C addresses: Write=0x%02X, Read=0x%02X", GPS_I2C_ADDR_WRITE, GPS_I2C_ADDR_READ);
    ESP_LOGI(TAG, "Waiting for GPS fix...");
    ESP_LOGI(TAG, "========================================");

    return ESP_OK;
}

esp_err_t gps_i2c_start(uint32_t poll_interval_ms)
{
    if (!gps_initialized) {
        ESP_LOGE(TAG, "GPS not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (gps_task_handle != NULL) {
        ESP_LOGW(TAG, "GPS task already running");
        return ESP_OK;
    }

    poll_interval = poll_interval_ms;

    BaseType_t result = xTaskCreate(
        gps_poll_task,
        "gps_poll",
        4096,
        NULL,
        10,
        &gps_task_handle
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create GPS poll task");
        return ESP_FAIL;
    }

    return ESP_OK;
}

void gps_i2c_stop(void)
{
    if (gps_task_handle != NULL) {
        vTaskDelete(gps_task_handle);
        gps_task_handle = NULL;
        ESP_LOGI(TAG, "GPS poll task stopped");
    }
}

esp_err_t gps_i2c_deinit(void)
{
    gps_i2c_stop();

    gps_initialized = false;
    data_callback = NULL;
    nmea_callback = NULL;

    ESP_LOGI(TAG, "GPS deinitialized");
    return ESP_OK;
}

esp_err_t gps_i2c_get_data(gps_data_t *data)
{
    if (!gps_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(data, &current_gps_data, sizeof(gps_data_t));
    return ESP_OK;
}

esp_err_t gps_i2c_register_data_callback(gps_data_callback_t callback, void *user_data)
{
    data_callback = callback;
    data_callback_user_data = user_data;
    return ESP_OK;
}

esp_err_t gps_i2c_register_nmea_callback(gps_nmea_callback_t callback, void *user_data)
{
    nmea_callback = callback;
    nmea_callback_user_data = user_data;
    return ESP_OK;
}

bool gps_i2c_has_fix(void)
{
    return gps_initialized && current_gps_data.valid;
}

gps_fix_type_t gps_i2c_get_fix_type(void)
{
    return current_gps_data.fix_type;
}

uint8_t gps_i2c_get_satellites_in_use(void)
{
    return current_gps_data.satellites_used;
}

uint32_t gps_i2c_get_time_since_update(void)
{
    if (!gps_initialized || current_gps_data.last_update_ms == 0) {
        return UINT32_MAX;
    }

    uint64_t now = esp_timer_get_time() / 1000;
    return (uint32_t)(now - current_gps_data.last_update_ms);
}

void* gps_i2c_get_mutex(void)
{
    return (void*)i2c_mutex;
}
