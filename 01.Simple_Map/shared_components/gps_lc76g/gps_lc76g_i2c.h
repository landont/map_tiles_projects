/* LC76G GPS module interface - I2C version */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// I2C addresses for LC76G
#define GPS_I2C_ADDR_WRITE  0x50
#define GPS_I2C_ADDR_READ   0x54

#define GPS_MAX_NMEA_LEN        (256)
#define GPS_MAX_SATELLITES      (47)

typedef enum {
    GPS_FIX_NONE = 0,
    GPS_FIX_2D = 2,
    GPS_FIX_3D = 3,
} gps_fix_type_t;

typedef enum {
    GPS_QUALITY_INVALID = 0,
    GPS_QUALITY_GPS = 1,
    GPS_QUALITY_DGPS = 2,
    GPS_QUALITY_PPS = 3,
    GPS_QUALITY_RTK = 4,
    GPS_QUALITY_FLOAT_RTK = 5,
    GPS_QUALITY_ESTIMATED = 6,
    GPS_QUALITY_MANUAL = 7,
    GPS_QUALITY_SIMULATION = 8,
} gps_quality_t;

typedef struct {
    uint8_t prn;
    uint8_t elevation;
    uint16_t azimuth;
    uint8_t snr;
    bool in_use;
} gps_satellite_t;

typedef struct {
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millisecond;
    uint8_t day;
    uint8_t month;
    uint16_t year;
} gps_datetime_t;

typedef struct {
    double latitude;
    double longitude;
    float altitude;
    float speed_knots;
    float speed_kmh;
    float course;
    gps_fix_type_t fix_type;
    gps_quality_t fix_quality;
    uint8_t satellites_used;
    uint8_t satellites_visible;
    float hdop;
    float vdop;
    float pdop;
    gps_datetime_t datetime;
    bool valid;
    uint32_t last_update_ms;
} gps_data_t;

typedef void (*gps_data_callback_t)(const gps_data_t *data, void *user_data);
typedef void (*gps_nmea_callback_t)(const char *sentence, void *user_data);
typedef void (*gps_error_callback_t)(int error_count, void *user_data);
typedef void (*gps_reset_callback_t)(void *user_data);  // Hardware reset callback

// Initialize GPS - uses BSP's bsp_lc76g_get_nmea for I2C communication
esp_err_t gps_i2c_init(void);
esp_err_t gps_i2c_deinit(void);
esp_err_t gps_i2c_get_data(gps_data_t *data);
esp_err_t gps_i2c_register_data_callback(gps_data_callback_t callback, void *user_data);
esp_err_t gps_i2c_register_nmea_callback(gps_nmea_callback_t callback, void *user_data);
esp_err_t gps_i2c_register_error_callback(gps_error_callback_t callback, void *user_data);
esp_err_t gps_i2c_register_reset_callback(gps_reset_callback_t callback, void *user_data);

// Start/stop the GPS polling task
esp_err_t gps_i2c_start(uint32_t poll_interval_ms);
void gps_i2c_stop(void);

bool gps_i2c_has_fix(void);
gps_fix_type_t gps_i2c_get_fix_type(void);
uint8_t gps_i2c_get_satellites_in_use(void);
uint32_t gps_i2c_get_time_since_update(void);

// Get the I2C mutex for shared bus access (for battery monitor, etc.)
// Other I2C devices on the same bus should use this mutex to prevent conflicts
void* gps_i2c_get_mutex(void);

#ifdef __cplusplus
}
#endif
