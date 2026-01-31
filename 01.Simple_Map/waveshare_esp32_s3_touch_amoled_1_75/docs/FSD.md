# Functional Specification Document (FSD)
## ESP32-S3-Touch-AMOLED-1.75 Map Viewer Application

**Version:** 1.1
**Date:** January 31, 2026
**Project:** Simple Map Viewer for Waveshare ESP32-S3-Touch-AMOLED-1.75

---

## 1. Introduction

### 1.1 Purpose
This document specifies the functional requirements and system architecture for an interactive map viewer application running on the Waveshare ESP32-S3-Touch-AMOLED-1.75 development board. The application displays pre-rendered map tiles from an SD card and provides touch-based navigation with zoom controls on a high-contrast square AMOLED display.

### 1.2 Scope
The system provides:
- Interactive touch-scrollable map display on 466x466 AMOLED
- Real-time GPS position tracking via LC76G GNSS module (UART)
- Zoom level control via touch buttons (levels 12-18)
- Battery monitoring via AXP2101 PMIC (percentage, charging status)
- SD card-based tile storage and retrieval
- Audio capability for future enhancements

### 1.3 Definitions and Acronyms

| Term | Definition |
|------|------------|
| AMOLED | Active-Matrix Organic Light-Emitting Diode |
| BSP | Board Support Package |
| GNSS | Global Navigation Satellite System |
| GPS | Global Positioning System |
| LVGL | Light and Versatile Graphics Library |
| NMEA | National Marine Electronics Association (GPS sentence format) |
| PMIC | Power Management Integrated Circuit |
| PSRAM | Pseudo Static Random Access Memory |
| QSPI | Quad Serial Peripheral Interface |
| Tile | 256x256 pixel map image segment |
| UART | Universal Asynchronous Receiver/Transmitter |
| Zoom Level | Map detail level (higher = more detail) |

---

## 2. System Overview

### 2.1 System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        Application Layer                         │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │                      main.cpp                            │    │
│  │  - Initialize hardware (NVS, I2C, UART, SD, Display)    │    │
│  │  - Initialize GPS via UART (GPIO18)                     │    │
│  │  - Initialize battery monitor (AXP2101 via I2C)         │    │
│  │  - Reset LCD via IO expander, touch via GPIO            │    │
│  │  - Launch SimpleMap with GPS tracking                   │    │
│  └─────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────┘
                                │
        ┌───────────────────────┼───────────────────────┐
        ▼                       ▼                       ▼
┌─────────────────┐ ┌─────────────────────┐ ┌─────────────────────┐
│ SimpleMap       │ │ GPS Component       │ │ Battery Monitor     │
│ ┌─────────────┐ │ │ ┌─────────────────┐ │ │ ┌─────────────────┐ │
│ │simple_map   │ │ │ │gps_lc76g_uart   │ │ │ │battery_monitor  │ │
│ │- Map tiles  │ │ │ │- UART RX GPIO18 │ │ │ │- AXP2101 PMIC   │ │
│ │- Zoom btns  │ │ │ │- NMEA parsing   │ │ │ │- Battery %      │ │
│ │- GPS marker │ │ │ │- Fix callbacks  │ │ │ │- Charge state   │ │
│ │- Battery UI │ │ │ │- 9600 baud      │ │ │ │- Power button   │ │
│ └─────────────┘ │ │ └─────────────────┘ │ │ └─────────────────┘ │
└─────────────────┘ └─────────────────────┘ └─────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                    map_tiles Component                           │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │  0015/map_tiles (ESP Component Registry)                │    │
│  │  - Tile coordinate calculations                         │    │
│  │  - File I/O for tile loading                            │    │
│  │  - Image buffer management                              │    │
│  │  - GPS ↔ Tile coordinate conversion                     │    │
│  └─────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                  Board Support Package (BSP)                     │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │  esp32_s3_touch_amoled_1_75 Component                   │    │
│  │  - Display driver (SH8601 via QSPI)                     │    │
│  │  - Touch controller (CST9217 via I2C)                   │    │
│  │  - SD card interface (SDMMC 1-bit mode)                 │    │
│  │  - I2C bus management (new I2C master driver)           │    │
│  │  - IO expander (TCA9554) for LCD reset                  │    │
│  │  - Brightness control (via SH8601 command)              │    │
│  │  - Audio codec support (ES8311/ES7210)                  │    │
│  └─────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                      Hardware Layer                              │
│  ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐ ┌─────┐│
│  │ESP32-S3│ │ SH8601 │ │CST9217 │ │TCA9554 │ │AXP2101 │ │LC76G││
│  │  MCU   │ │ AMOLED │ │ Touch  │ │IO Exp  │ │ PMIC   │ │ GPS ││
│  └────────┘ └────────┘ └────────┘ └────────┘ └────────┘ └─────┘│
│  ┌────────┐                                                     │
│  │SD Card │                                                     │
│  │(FAT32) │                                                     │
│  └────────┘                                                     │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 Hardware Platform

#### 2.2.1 Waveshare ESP32-S3-Touch-AMOLED-1.75 Specifications

| Component | Specification |
|-----------|---------------|
| MCU | ESP32-S3 (Xtensa LX7 dual-core, 240 MHz) |
| Flash | 16 MB |
| PSRAM | 8 MB (Octal) |
| Display | 1.75" AMOLED, 466x466 pixels (square) |
| Display Controller | SH8601 (QSPI interface) |
| Touch Controller | CST9217 (I2C, up to 5 touch points) |
| SD Card Interface | SDMMC native 1-bit mode |
| IO Expander | TCA9554PWR (I2C address 0x20) |
| PMIC | AXP2101 (I2C address 0x34) |
| GPS Module | LC76G GNSS (UART, 9600 baud) |
| Audio Codec | ES8311 (DAC) / ES7210 (ADC) |

#### 2.2.2 Pin Configuration

| Function | GPIO | Notes |
|----------|------|-------|
| **I2C Bus** | | Touch, IO expander, audio codec |
| I2C SCL | GPIO 14 | 400 kHz |
| I2C SDA | GPIO 15 | |
| **QSPI Display** | | SH8601 AMOLED |
| LCD CS | GPIO 12 | |
| LCD PCLK | GPIO 38 | |
| LCD DATA0 | GPIO 4 | |
| LCD DATA1 | GPIO 5 | |
| LCD DATA2 | GPIO 6 | |
| LCD DATA3 | GPIO 7 | |
| LCD Reset | GPIO 39 | |
| LCD Backlight | N/A | AMOLED self-emissive |
| **Touch** | | CST9217 |
| Touch Reset | GPIO 40 | |
| Touch INT | N/C | Polling mode |
| **SD Card (SDMMC)** | | |
| SD CLK | GPIO 2 | |
| SD CMD | GPIO 1 | |
| SD D0 | GPIO 3 | |
| **GPS UART** | | LC76G GNSS |
| GPS UART RX | GPIO 18 | Receives GPS TX (hardware mod) |
| **I2S Audio** | | |
| I2S SCLK | GPIO 9 | |
| I2S MCLK | GPIO 42 | |
| I2S LCLK | GPIO 45 | |
| I2S DOUT | GPIO 8 | Speaker |
| I2S DIN | GPIO 10 | Microphone |
| Power Amp | GPIO 46 | |

---

## 3. Functional Requirements

### 3.1 Map Display Functions

#### FR-3.1.1 Tile Grid Display
- **Description:** Display a 5x4 grid of map tiles (20 tiles total)
- **Input:** Tile coordinates (X, Y) and zoom level
- **Output:** Visual map display on AMOLED
- **Behavior:**
  - Load tiles from SD card at `/sdcard/tiles1/{zoom}/{x}/{y}.bin`
  - Each tile is 256x256 pixels in RGB565 format
  - Grid creates a 1280x1024 pixel scrollable area (5 columns x 4 rows)
  - Square display optimizes tile alignment

#### FR-3.1.2 Touch Scrolling
- **Description:** Allow users to pan the map by touch dragging
- **Input:** Touch drag gesture (up to 5 touch points supported)
- **Output:** Map scrolls in the direction of the drag
- **Behavior:**
  - Smooth scrolling without momentum (momentum disabled)
  - When scroll reaches edge (>90 pixels beyond boundary), load adjacent tiles
  - Update GPS coordinates in real-time during scroll (50ms throttle)
  - Touch mirroring on X and Y axes

#### FR-3.1.3 Zoom Level Control
- **Description:** Change map zoom level between 12-18
- **Input:** Tap on zoom in (+) or zoom out (-) button
- **Output:** Map reloads at new zoom level
- **Behavior:**
  - "+" button increases zoom by 1 level (max 18)
  - "-" button decreases zoom by 1 level (min 12)
  - Map recenters on current GPS coordinates after zoom change
  - Loading popup displays during tile reload
  - Buttons disabled during tile loading to prevent rapid zoom changes

### 3.2 Coordinate Functions

#### FR-3.2.1 Internal GPS Tracking
- **Description:** Track current map center coordinates internally
- **Input:** Map scroll position
- **Output:** Latitude and longitude available via API
- **Behavior:**
  - Coordinates updated during scroll events (throttled to 50ms)
  - Available via `SimpleMap::get_current_location()` API
  - Used for centering map after zoom changes

#### FR-3.2.2 Coordinate Conversion
- **Description:** Convert between GPS coordinates and tile coordinates
- **Input:** Latitude, longitude, zoom level
- **Output:** Tile X, Y coordinates
- **Algorithm:** Standard Web Mercator projection (EPSG:3857)

### 3.3 User Interface Functions

#### FR-3.3.1 Zoom In Button
- **Description:** Circular button to increase zoom level by 1
- **Position:** Lower left corner of screen (15px from edges)
- **Size:** 50x50 pixels, circular (radius 25)
- **Appearance:** Dark gray background (80% opacity), white border, white "+" symbol
- **Behavior:**
  - Increases zoom level by 1 when tapped
  - Maximum zoom level: 18
  - Disabled during tile loading operations

#### FR-3.3.2 Zoom Out Button
- **Description:** Circular button to decrease zoom level by 1
- **Position:** Lower right corner of screen (15px from edges)
- **Size:** 50x50 pixels, circular (radius 25)
- **Appearance:** Dark gray background (80% opacity), white border, white "-" symbol
- **Behavior:**
  - Decreases zoom level by 1 when tapped
  - Minimum zoom level: 12
  - Disabled during tile loading operations

#### FR-3.3.3 Battery Status Indicator
- **Description:** Compact indicator showing battery level and charging status
- **Position:** Upper right corner of screen (5px from edges)
- **Size:** 67x24 pixels
- **Components:**
  - Colored status bar (18x16 pixels, left side)
  - Percentage label (e.g., "85%")
- **Color Coding:**
  - Green: Battery > 50%
  - Orange: Battery 21-50%
  - Red: Battery <= 20%
  - Blue: Charging (USB power connected)
  - Gray: Unknown/No battery
- **Update Frequency:** Placeholder for AXP2101 integration
- **API:** `SimpleMap::update_battery_indicator(int percent, bool is_charging)`

#### FR-3.3.4 Loading Indicator
- **Description:** Visual feedback during tile loading
- **Appearance:** Yellow pill-shaped popup with "Loading map..." text
- **Behavior:**
  - Appears when tile loading begins
  - Disappears when all tiles loaded
  - Prevents multiple simultaneous load operations

### 3.4 Display Brightness Functions

#### FR-3.4.1 AMOLED Brightness Control
- **Description:** Control display brightness via SH8601 command
- **Method:** Command 0x51 with brightness value (0-255)
- **Note:** No PWM backlight - AMOLED is self-emissive
- **Behavior:**
  - 100% brightness on initialization
  - Activity-based dimming supported via BSP

#### FR-3.4.2 Activity-Based Brightness
- **Description:** Dim display on touch inactivity
- **Input:** Touch events
- **Behavior:**
  - Reset brightness to 100% on any touch activity
  - Track last touch time for timeout-based dimming

### 3.5 GPS Functions

#### FR-3.5.1 GPS Position Tracking
- **Description:** Receive and process GPS position from LC76G module via UART
- **Input:** NMEA sentences from GPS module at 9600 baud on GPIO18
- **Output:** Latitude, longitude, altitude, speed, course, satellite count
- **Behavior:**
  - UART configured for RX-only operation (GPS TX → ESP32 RX)
  - Parse GGA, RMC, GSA, GSV NMEA sentences
  - Update map position when valid fix acquired
  - Heading indicator shown only when moving (speed > 1 km/h)

#### FR-3.5.2 GPS Status Display
- **Description:** Show GPS fix status on map display
- **Position:** Upper left corner of screen
- **Components:**
  - Satellite icon with count (e.g., "●12")
  - Fix type indicator (2D/3D)
- **Color Coding:**
  - Green: Valid 3D fix
  - Yellow: Valid 2D fix
  - Red: No fix / acquiring
  - Flashing: GPS error condition
- **API:** `SimpleMap::update_gps_status(int satellites, int fix_type)`

#### FR-3.5.3 GPS Error Handling
- **Description:** Handle GPS communication errors gracefully
- **Behavior:**
  - Track consecutive NMEA parse errors
  - Show error indicator after 10 consecutive errors
  - Clear error state when valid data received
  - Callback-based error notification to application
- **API:** `gps_uart_register_error_callback(callback, user_data)`

#### FR-3.5.4 GPS Marker Display
- **Description:** Show current GPS position on map
- **Appearance:** Circular marker with optional heading indicator
- **Behavior:**
  - Marker centered on GPS coordinates
  - Heading arrow shown when moving (speed > 1 km/h)
  - Map can optionally auto-center on GPS position
- **API:** `SimpleMap::set_gps_position(lat, lon, valid, heading)`

### 3.6 Battery Monitor Functions

#### FR-3.6.1 Battery Level Monitoring
- **Description:** Read battery status from AXP2101 PMIC
- **Input:** I2C communication with AXP2101 at address 0x34
- **Output:** Battery percentage (0-100%), charging status
- **Behavior:**
  - Periodic polling (configurable interval)
  - Update UI battery indicator with current level
  - Detect USB power connection for charging status

#### FR-3.6.2 Power Button Configuration
- **Description:** Configure AXP2101 power button behavior
- **Behavior:**
  - Short press: No action (reserved for future use)
  - Long press (configurable): Power off device
  - Power button IRQ handling

#### FR-3.6.3 Battery Task
- **Description:** FreeRTOS task for battery monitoring
- **Stack Size:** 8 KB
- **Priority:** Configurable (default: 5)
- **Update Interval:** Configurable (default: 5 seconds)
- **API:** `battery_monitor_init(i2c_handle)`

---

## 4. Non-Functional Requirements

### 4.1 Performance

| Metric | Requirement |
|--------|-------------|
| Tile load time | < 500ms for 20 tiles |
| Scroll response | < 50ms latency |
| GPS update rate | 20 Hz during scroll |
| Display refresh | AMOLED native (no tearing) |
| Memory usage | < 3 MB PSRAM for tiles |

### 4.2 Memory Configuration

| Resource | Allocation |
|----------|------------|
| Main task stack | 8,192 bytes |
| LVGL buffer | 466 x 100 pixels |
| Tile cache | 20 tiles x 131 KB = ~2.6 MB |
| Total PSRAM | 8 MB available |

### 4.3 Storage Requirements

| Item | Requirement |
|------|-------------|
| SD card format | FAT32 |
| Tile format | RGB565 binary (131,072 bytes/tile) |
| Tile path | `/sdcard/tiles1/{zoom}/{x}/{y}.bin` |
| Supported zoom levels | 12-18 |

---

## 5. Data Formats

### 5.1 Tile File Format

```
File: /sdcard/tiles1/{zoom}/{x}/{y}.bin
Size: 131,072 bytes (256 x 256 x 2)
Format: Raw RGB565 (16-bit per pixel)
Byte Order: Little-endian
```

### 5.2 Tile Coordinate System

```
Zoom Level 0: 1 tile covers entire world
Zoom Level N: 2^N x 2^N tiles cover entire world

Tile X: 0 at 180°W, increases eastward
Tile Y: 0 at 85.05°N, increases southward

Conversion formulas:
  tile_x = floor((lon + 180) / 360 * 2^zoom)
  tile_y = floor((1 - ln(tan(lat) + sec(lat)) / π) / 2 * 2^zoom)
```

---

## 6. System States

### 6.1 State Diagram

```
┌─────────────┐
│   STARTUP   │
│  - Init HW  │
│  - Reset LCD│
│  - Reset TP │
│  - Mount SD │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│    IDLE     │◄────────────────────┐
│  - Display  │                     │
│    map      │                     │
│  - Battery  │                     │
│    updates  │                     │
└──────┬──────┘                     │
       │                            │
       ├─────── Touch scroll ───────┤
       │                            │
       ▼                            │
┌─────────────┐                     │
│  SCROLLING  │                     │
│  - Update   │                     │
│    coords   │                     │
└──────┬──────┘                     │
       │                            │
       │ Scroll end at edge         │
       ▼                            │
┌─────────────┐                     │
│   LOADING   │─────────────────────┘
│  - Show     │
│    popup    │
│  - Load     │
│    tiles    │
└─────────────┘
```

### 6.2 State Descriptions

| State | Description | Exit Conditions |
|-------|-------------|-----------------|
| STARTUP | Hardware initialization, LCD/touch reset | Init complete → IDLE |
| IDLE | Map displayed, ready for interaction | Touch → SCROLLING, Button → LOADING |
| SCROLLING | User dragging map | Release + edge → LOADING, Release → IDLE |
| LOADING | Fetching tiles from SD | Load complete → IDLE |

---

## 7. API Reference

### 7.1 SimpleMap Public Interface

```cpp
class SimpleMap {
public:
    // Initialize map system on given LVGL screen
    static bool init(lv_obj_t* parent_screen);

    // Display map at GPS coordinates with zoom level
    static void show_location(double latitude, double longitude, int zoom_level = 16);

    // Update to new GPS position (keeps current zoom)
    static void update_location(double latitude, double longitude);

    // Change zoom level and reload tiles
    static void change_zoom_level(int new_zoom);

    // Center map view on current GPS coordinates
    static void center_map_on_gps();

    // Get current center coordinates
    static void get_current_location(double* latitude, double* longitude);

    // Get current zoom level
    static int get_current_zoom();

    // Update battery indicator (0-100 percent, -1 for unknown/no battery)
    static void update_battery_indicator(int percent, bool is_charging = false);

    // Set tile type (for multiple tile sets)
    static bool set_tile_type(int tile_type);
    static int get_tile_type();
    static int get_tile_type_count();

    // Activity tracking for brightness control
    static uint32_t get_last_touch_time();
    static void reset_activity_timer();

    // Cleanup and free resources
    static void cleanup();
};
```

### 7.2 BSP Public Interface

```c
// I2C Management
esp_err_t bsp_i2c_init(void);
esp_err_t bsp_i2c_deinit(void);
i2c_master_bus_handle_t bsp_i2c_get_handle(void);

// Display
lv_display_t *bsp_display_start(void);
lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg);
lv_indev_t *bsp_display_get_input_dev(void);
bool bsp_display_lock(uint32_t timeout_ms);
void bsp_display_unlock(void);

// Brightness (AMOLED via SH8601 command)
esp_err_t bsp_display_brightness_init(void);
esp_err_t bsp_display_brightness_set(int brightness_percent);
esp_err_t bsp_display_backlight_on(void);
esp_err_t bsp_display_backlight_off(void);

// SD Card
esp_err_t bsp_sdcard_mount(void);
esp_err_t bsp_sdcard_unmount(void);

// IO Expander
esp_io_expander_handle_t bsp_io_expander_init(void);

// Touch
esp_err_t bsp_touch_new(const bsp_touch_config_t *config,
                        esp_lcd_touch_handle_t *ret_touch);

// Audio (future use)
esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_config);
esp_codec_dev_handle_t bsp_audio_codec_speaker_init(void);
esp_codec_dev_handle_t bsp_audio_codec_microphone_init(void);
```

### 7.3 GPS UART Interface (gps_lc76g_uart)

```c
// GPS data types
typedef enum {
    GPS_FIX_NONE = 0,
    GPS_FIX_2D = 2,
    GPS_FIX_3D = 3
} gps_fix_type_t;

typedef struct {
    bool valid;              // True if GPS has valid fix
    double latitude;         // Decimal degrees (-90 to +90)
    double longitude;        // Decimal degrees (-180 to +180)
    float altitude;          // Meters above sea level
    float speed_kmh;         // Speed in km/h
    float course;            // Course/heading in degrees (0-360)
    gps_fix_type_t fix_type; // Fix type (none, 2D, 3D)
    int satellites_used;     // Number of satellites used in fix
    int satellites_visible;  // Number of satellites visible
    uint8_t hour, minute, second;  // UTC time
    uint8_t day, month;
    uint16_t year;
} gps_data_t;

// Callback types
typedef void (*gps_data_callback_t)(const gps_data_t *data, void *user_data);
typedef void (*gps_error_callback_t)(int error_count, void *user_data);

// Initialization and control
esp_err_t gps_uart_init(void);          // Initialize UART on GPIO18
esp_err_t gps_uart_start(void);         // Start GPS reception task
esp_err_t gps_uart_stop(void);          // Stop GPS reception task
esp_err_t gps_uart_deinit(void);        // Deinitialize UART

// Callback registration
esp_err_t gps_uart_register_data_callback(gps_data_callback_t callback, void *user_data);
esp_err_t gps_uart_register_error_callback(gps_error_callback_t callback, void *user_data);

// Data retrieval
esp_err_t gps_uart_get_last_data(gps_data_t *data);  // Get most recent GPS data
```

### 7.4 Battery Monitor Interface (battery_monitor)

```c
// Battery status structure
typedef struct {
    int percentage;          // 0-100, -1 if unknown
    bool is_charging;        // True if USB power connected
    float voltage_mv;        // Battery voltage in millivolts
} battery_status_t;

// Initialization
esp_err_t battery_monitor_init(i2c_master_bus_handle_t i2c_handle);

// Status retrieval
esp_err_t battery_monitor_get_status(battery_status_t *status);

// Internal: Update callback registered with SimpleMap
// Called automatically by battery monitor task
```

---

## 8. Dependencies

### 8.1 Software Dependencies

| Component | Version | Source |
|-----------|---------|--------|
| ESP-IDF | >= 5.3.0 | Espressif |
| LVGL | 9.3.0 | lvgl/lvgl |
| map_tiles | ^1.2.0 | 0015/map_tiles |
| esp_lcd_sh8601 | * | waveshare |
| esp_lcd_touch_cst9217 | * | waveshare |
| esp_lvgl_port | ^2 | espressif |
| esp_io_expander_tca9554 | ^2 | espressif |
| esp_codec_dev | ^1.3.3 | espressif |
| gps_lc76g_uart | local | shared_components |
| battery_monitor | local | shared_components |

### 8.2 Build Configuration

Key sdkconfig settings:
```ini
CONFIG_IDF_TARGET="esp32s3"
CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_OCT=y
CONFIG_SPIRAM_SPEED_80M=y
CONFIG_LV_COLOR_DEPTH_16=y
CONFIG_LV_COLOR_16_SWAP=y
CONFIG_LV_FONT_MONTSERRAT_14=y
CONFIG_LV_FONT_MONTSERRAT_16=y
CONFIG_BSP_DISPLAY_LVGL_BUF_HEIGHT=100
CONFIG_BSP_I2C_CLK_SPEED_HZ=400000
```

### 8.3 Build System Patches

The CMakeLists.txt includes an automatic patch for `esp_lvgl_port_touch.c` to handle CST9217 touch read errors gracefully instead of aborting.

---

## 9. Error Handling

### 9.1 Error Conditions

| Condition | Detection | Response |
|-----------|-----------|----------|
| SD card not mounted | bsp_sdcard_mount() returns error | Log error, display fails |
| Tile file not found | map_tiles_load_tile() returns false | Display black placeholder |
| PSRAM exhausted | Allocation failure | Graceful degradation with smaller grid |
| Display init failure | bsp_display_start() returns NULL | Log error, abort |
| Touch read error | esp_lcd_touch_read_data() error | Skip frame, retry next cycle |
| IO expander init failure | bsp_io_expander_init() returns NULL | Log error, abort |
| GPS UART init failure | gps_uart_init() returns error | Log error, continue without GPS |
| GPS no fix | data->valid == false | Show "acquiring" status, grey marker |
| GPS NMEA parse errors | 10+ consecutive errors | Show error indicator, invoke error callback |
| Battery monitor init failure | battery_monitor_init() returns error | Log error, show "unknown" battery status |
| AXP2101 communication error | I2C read/write failure | Retry, show "unknown" battery status |

### 9.2 Debug Output

Key log messages for troubleshooting:
```
"SimpleMap: Initialized with grid size %dx%d (%d tiles)"
"SimpleMap: Loading %dx%d grid at position (%d,%d) at zoom %d"
"SimpleMap: Tile loading completed in %lu ms"
"SimpleMap: Changing zoom from %d to %d"
"SimpleMap: Centered map on GPS %.6f, %.6f"
"Patching esp_lvgl_port_touch.c to handle touch read errors gracefully..."
"GPS UART initialized"
"GPS: Fix: %.6f, %.6f | Alt: %.1fm | Speed: %.1f km/h | Course: %.1f | Sats: %d"
"GPS: Acquiring fix... Satellites visible: %d"
"Battery: %d%% (charging: %s)"
"Battery monitor initialized"
```

---

## 10. Hardware Characteristics

### 10.1 AMOLED Display Advantages

| Feature | Benefit |
|---------|---------|
| Self-emissive pixels | Perfect blacks, infinite contrast |
| No backlight | Reduced power for dark content |
| Square format (466x466) | Optimal for map tile alignment |
| Wide viewing angles | Consistent colors at all angles |
| Fast response time | No motion blur during scrolling |

### 10.2 Touch Controller Features

| Feature | Specification |
|---------|---------------|
| Controller | CST9217 |
| Interface | I2C |
| Max touch points | 5 |
| Touch button support | 1 |
| Axis mirroring | X and Y |

---

## 11. Future Enhancements

### 11.1 Planned Features
- Audio feedback for user interactions
- Multiple tile layer support (street, satellite, terrain)
- Waypoint markers and route display
- Offline tile download via WiFi
- Gesture support (pinch-to-zoom)
- Low battery warnings and power saving mode
- GPS track recording and playback
- Compass heading integration

### 11.2 Hardware Compatibility
The SimpleMap component is designed for portability across ESP32 platforms:
- ESP32-S3-Touch-LCD-3.5B (320x480 LCD)
- ESP32-P4 boards (higher resolution displays)
- Other ESP32-S3 AMOLED displays
- Custom hardware with LVGL support

---

## Appendix A: File Structure

```
01.Simple_Map/
├── shared_components/                   # Shared across board variants
│   ├── simple_map/                      # Map display component
│   │   ├── CMakeLists.txt
│   │   ├── simple_map.cpp
│   │   └── simple_map.hpp
│   ├── gps_lc76g_uart/                  # GPS UART component
│   │   ├── CMakeLists.txt
│   │   ├── gps_lc76g_uart.c
│   │   └── gps_lc76g_uart.h
│   └── battery_monitor/                 # Battery monitor component
│       ├── CMakeLists.txt
│       ├── battery_monitor.c
│       └── battery_monitor.h
│
└── waveshare_esp32_s3_touch_amoled_1_75/  # Board-specific project
    ├── CMakeLists.txt                   # Build config with touch patch
    ├── sdkconfig
    ├── sdkconfig.defaults
    ├── docs/
    │   └── FSD.md                       # This document
    ├── main/
    │   ├── CMakeLists.txt
    │   ├── idf_component.yml
    │   └── main.cpp                     # Application entry point
    ├── components/
    │   └── esp32_s3_touch_amoled_1_75/  # Board Support Package
    │       ├── CMakeLists.txt
    │       ├── idf_component.yml
    │       ├── Kconfig
    │       ├── esp32_s3_touch_amoled_1_75.c
    │       ├── include/
    │       │   └── bsp/
    │       │       ├── esp32_s3_touch_amoled_1_75.h
    │       │       ├── esp-bsp.h
    │       │       ├── display.h
    │       │       ├── touch.h
    │       │       └── config.h
    │       └── priv_include/
    │           └── bsp_err_check.h
    └── managed_components/              # Auto-downloaded dependencies
        ├── espressif__esp_lvgl_port/
        ├── lvgl__lvgl/
        ├── 0015__map_tiles/
        ├── waveshare__esp_lcd_sh8601/
        └── waveshare__esp_lcd_touch_cst9217/
```

---

## Appendix B: Initialization Sequence

```
1. NVS Flash Init
   └── Initialize non-volatile storage

2. GPS UART Init
   ├── Configure UART1 on GPIO18 (RX only)
   ├── Set baud rate to 9600
   ├── Register GPS data callback
   ├── Register GPS error callback
   └── Start GPS reception task

3. SD Card Mount
   └── Mount FAT32 filesystem at /sdcard

4. I2C Bus Init (implicit via BSP)
   └── Configure GPIO 14/15 for touch, IO expander, battery

5. IO Expander Init (TCA9554)
   └── Initialize I2C device at 0x20

6. LCD Reset (via IO Expander Pin 7)
   ├── Set low for 100ms
   └── Set high, wait 100ms

7. Touch Reset (via GPIO 40)
   ├── Set low for 50ms
   └── Set high, wait 200ms

8. Display Start
   ├── Initialize QSPI bus
   ├── Configure SH8601 controller
   ├── Start LVGL port
   ├── Initialize touch input
   └── Set brightness to 100%

9. SimpleMap Init
   ├── Initialize map_tiles component
   ├── Create tile widget grid
   ├── Create zoom buttons
   ├── Create GPS status indicator
   ├── Create battery indicator
   └── Register touch callbacks

10. Show Initial Location
    ├── Set coordinates (Acquaseria, Lake Como)
    ├── Load map tiles
    ├── Center display on GPS position
    └── Enable GPS tracking mode

11. Battery Monitor Init
    ├── Initialize AXP2101 via I2C (0x34)
    ├── Configure power button behavior
    ├── Start battery monitoring task
    └── Begin periodic battery updates
```

---

## Appendix C: Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.1 | 2026-01-31 | Claude Opus 4.5 | Added GPS UART support (LC76G via GPIO18), AXP2101 battery monitoring, GPS/battery functional requirements and API documentation |
| 1.0 | 2026-01-29 | Claude Opus 4.5 | Initial release for AMOLED 1.75" board |
