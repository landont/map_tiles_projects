/* LC76G GPS module implementation - UART version */

#include "gps_lc76g_uart.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "GPS_LC76G_UART";

#define UART_RX_BUF_SIZE    (1024)
#define NMEA_MAX_LEN        (256)

static bool gps_initialized = false;
static gps_data_t current_gps_data = {0};
static TaskHandle_t gps_task_handle = NULL;
static gps_data_callback_t data_callback = NULL;
static void *data_callback_user_data = NULL;
static gps_nmea_callback_t nmea_callback = NULL;
static void *nmea_callback_user_data = NULL;
static gps_error_callback_t error_callback = NULL;
static void *error_callback_user_data = NULL;

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

// Diagnostic counters
static uint32_t checksum_failures = 0;
static uint32_t valid_sentences = 0;
static uint32_t gga_count = 0;
static uint32_t rmc_count = 0;
static uint32_t gsa_count = 0;
static uint32_t gsv_count = 0;

static void parse_nmea_sentence(const char *sentence)
{
    ESP_LOGD(TAG, "NMEA: %s", sentence);

    if (!nmea_verify_checksum(sentence)) {
        checksum_failures++;
        // Log first 10 checksum failures with sentence type
        if (checksum_failures <= 10) {
            // Extract sentence type (e.g., GPRMC, GPGGA)
            char type[7] = {0};
            if (sentence[0] == '$' && strlen(sentence) > 6) {
                strncpy(type, sentence + 1, 6);
                char *comma = strchr(type, ',');
                if (comma) *comma = '\0';
            }
            ESP_LOGW(TAG, "Checksum fail #%lu [%s]: %.60s%s",
                     checksum_failures, type, sentence,
                     strlen(sentence) > 60 ? "..." : "");
        }
        return;
    }

    valid_sentences++;

    if (nmea_callback) {
        nmea_callback(sentence, nmea_callback_user_data);
    }

    if (strstr(sentence, "GGA")) {
        gga_count++;
        parse_gga(sentence);
    } else if (strstr(sentence, "RMC")) {
        rmc_count++;
        parse_rmc(sentence);

        if (data_callback && current_gps_data.valid) {
            data_callback(&current_gps_data, data_callback_user_data);
        }
    } else if (strstr(sentence, "GSA")) {
        gsa_count++;
        parse_gsa(sentence);
    } else if (strstr(sentence, "GSV")) {
        gsv_count++;
        parse_gsv(sentence);
    }
}

// GPS UART receive task
static void gps_uart_task(void *pvParameters)
{
    char nmea_buffer[NMEA_MAX_LEN];
    int nmea_idx = 0;
    uint8_t rx_byte;
    int consecutive_errors = 0;
    uint32_t last_data_time = 0;

    ESP_LOGI(TAG, "GPS UART task started");

    while (1) {
        int len = uart_read_bytes(GPS_UART_NUM, &rx_byte, 1, pdMS_TO_TICKS(100));

        if (len > 0) {
            consecutive_errors = 0;
            last_data_time = esp_timer_get_time() / 1000;

            if (rx_byte == '$') {
                // Start of new NMEA sentence
                nmea_idx = 0;
                nmea_buffer[nmea_idx++] = rx_byte;
            } else if (rx_byte == '\n' || rx_byte == '\r') {
                // End of NMEA sentence
                if (nmea_idx > 0) {
                    nmea_buffer[nmea_idx] = '\0';
                    if (nmea_buffer[0] == '$') {
                        sentences_received++;
                        parse_nmea_sentence(nmea_buffer);
                    }
                    nmea_idx = 0;
                }
            } else if (nmea_idx < NMEA_MAX_LEN - 1) {
                nmea_buffer[nmea_idx++] = rx_byte;
            }
        } else {
            // No data received - check for timeout
            uint32_t now = esp_timer_get_time() / 1000;

            if (last_data_time > 0 && (now - last_data_time) > 5000) {
                // No data for 5 seconds
                if (consecutive_errors == 0) {
                    consecutive_errors = 1;
                    ESP_LOGW(TAG, "No GPS data for 5 seconds - connection may be lost");

                    // Notify with invalid data
                    if (data_callback) {
                        gps_data_t stale_data = {0};
                        stale_data.valid = false;
                        data_callback(&stale_data, data_callback_user_data);
                    }
                }
            }
        }
    }

    vTaskDelete(NULL);
}

// Try to detect the correct baud rate by looking for valid NMEA data
static int detect_baud_rate(void)
{
    static const int baud_rates[] = {9600, 115200, 38400, 57600, 4800, 19200};
    static const int num_rates = sizeof(baud_rates) / sizeof(baud_rates[0]);

    uint8_t buffer[128];

    ESP_LOGI(TAG, "Auto-detecting GPS baud rate...");

    for (int i = 0; i < num_rates; i++) {
        int baud = baud_rates[i];

        // Change baud rate
        uart_set_baudrate(GPS_UART_NUM, baud);

        // Flush any garbage
        uart_flush_input(GPS_UART_NUM);
        vTaskDelay(pdMS_TO_TICKS(100));

        // Read data for up to 2 seconds
        int valid_chars = 0;
        int total_chars = 0;
        int dollar_signs = 0;
        uint32_t start = esp_timer_get_time() / 1000;

        while ((esp_timer_get_time() / 1000 - start) < 2000) {
            int len = uart_read_bytes(GPS_UART_NUM, buffer, sizeof(buffer) - 1, pdMS_TO_TICKS(100));
            if (len > 0) {
                buffer[len] = '\0';
                total_chars += len;

                for (int j = 0; j < len; j++) {
                    // Count valid ASCII printable chars and common NMEA chars
                    if ((buffer[j] >= 32 && buffer[j] <= 126) ||
                        buffer[j] == '\r' || buffer[j] == '\n') {
                        valid_chars++;
                    }
                    if (buffer[j] == '$') {
                        dollar_signs++;
                    }
                }
            }
        }

        if (total_chars > 0) {
            int valid_percent = (valid_chars * 100) / total_chars;

            // If >90% valid ASCII and we saw some $ signs, this is likely correct
            if (valid_percent > 90 && dollar_signs >= 2) {
                return baud;
            }
        }
    }

    ESP_LOGW(TAG, "Could not auto-detect baud rate, defaulting to %d", GPS_UART_BAUD);
    return GPS_UART_BAUD;
}

esp_err_t gps_uart_init(void)
{
    if (gps_initialized) {
        ESP_LOGW(TAG, "GPS UART already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing LC76G GNSS module (UART on GPIO%d)...", GPS_UART_RX_PIN);

    // Start with default baud rate for driver install
    uart_config_t uart_config = {
        .baud_rate = GPS_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_driver_install(GPS_UART_NUM, UART_RX_BUF_SIZE, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_param_config(GPS_UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART: %s", esp_err_to_name(ret));
        uart_driver_delete(GPS_UART_NUM);
        return ret;
    }

    // Only configure RX pin (GPS TX -> ESP32 RX)
    // TX pin not needed since we're only reading from GPS
    ret = uart_set_pin(GPS_UART_NUM, UART_PIN_NO_CHANGE, GPS_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        uart_driver_delete(GPS_UART_NUM);
        return ret;
    }

    // Auto-detect baud rate
    int detected_baud = detect_baud_rate();
    uart_set_baudrate(GPS_UART_NUM, detected_baud);

    // Flush buffer after baud rate detection
    uart_flush_input(GPS_UART_NUM);

    memset(&current_gps_data, 0, sizeof(current_gps_data));
    gps_initialized = true;

    ESP_LOGI(TAG, "GPS UART initialized on GPIO%d at %d baud", GPS_UART_RX_PIN, detected_baud);

    return ESP_OK;
}

esp_err_t gps_uart_start(void)
{
    if (!gps_initialized) {
        ESP_LOGE(TAG, "GPS not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (gps_task_handle != NULL) {
        ESP_LOGW(TAG, "GPS task already running");
        return ESP_OK;
    }

    BaseType_t result = xTaskCreate(
        gps_uart_task,
        "gps_uart",
        4096,
        NULL,
        10,
        &gps_task_handle
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create GPS UART task");
        return ESP_FAIL;
    }

    return ESP_OK;
}

void gps_uart_stop(void)
{
    if (gps_task_handle != NULL) {
        vTaskDelete(gps_task_handle);
        gps_task_handle = NULL;
        ESP_LOGI(TAG, "GPS UART task stopped");
    }
}

esp_err_t gps_uart_deinit(void)
{
    gps_uart_stop();

    if (gps_initialized) {
        uart_driver_delete(GPS_UART_NUM);
        gps_initialized = false;
    }

    data_callback = NULL;
    nmea_callback = NULL;

    ESP_LOGI(TAG, "GPS UART deinitialized");
    return ESP_OK;
}

esp_err_t gps_uart_get_data(gps_data_t *data)
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

esp_err_t gps_uart_register_data_callback(gps_data_callback_t callback, void *user_data)
{
    data_callback = callback;
    data_callback_user_data = user_data;
    return ESP_OK;
}

esp_err_t gps_uart_register_nmea_callback(gps_nmea_callback_t callback, void *user_data)
{
    nmea_callback = callback;
    nmea_callback_user_data = user_data;
    return ESP_OK;
}

esp_err_t gps_uart_register_error_callback(gps_error_callback_t callback, void *user_data)
{
    error_callback = callback;
    error_callback_user_data = user_data;
    return ESP_OK;
}

bool gps_uart_has_fix(void)
{
    return gps_initialized && current_gps_data.valid;
}

gps_fix_type_t gps_uart_get_fix_type(void)
{
    return current_gps_data.fix_type;
}

uint8_t gps_uart_get_satellites_in_use(void)
{
    return current_gps_data.satellites_used;
}

uint32_t gps_uart_get_time_since_update(void)
{
    if (!gps_initialized || current_gps_data.last_update_ms == 0) {
        return UINT32_MAX;
    }

    uint64_t now = esp_timer_get_time() / 1000;
    return (uint32_t)(now - current_gps_data.last_update_ms);
}
