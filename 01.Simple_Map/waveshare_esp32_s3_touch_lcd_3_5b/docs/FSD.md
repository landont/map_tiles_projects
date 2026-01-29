# Functional Specification Document (FSD)
## ESP32-S3-Touch-LCD-3.5B Map Viewer Application

**Version:** 1.0
**Date:** January 29, 2026
**Project:** Simple Map Viewer for Waveshare ESP32-S3-Touch-LCD-3.5B

---

## 1. Introduction

### 1.1 Purpose
This document specifies the functional requirements and system architecture for an interactive map viewer application running on the Waveshare ESP32-S3-Touch-LCD-3.5B development board. The application displays pre-rendered map tiles from an SD card and provides touch-based navigation with real-time GPS coordinate tracking.

### 1.2 Scope
The system provides:
- Interactive touch-scrollable map display
- Real-time GPS coordinate calculation from map position
- Zoom level control (levels 10-19)
- Manual coordinate entry via on-screen keyboard
- SD card-based tile storage and retrieval

### 1.3 Definitions and Acronyms

| Term | Definition |
|------|------------|
| BSP | Board Support Package |
| GPS | Global Positioning System |
| LVGL | Light and Versatile Graphics Library |
| PSRAM | Pseudo Static Random Access Memory |
| QSPI | Quad Serial Peripheral Interface |
| Tile | 256x256 pixel map image segment |
| Zoom Level | Map detail level (higher = more detail) |

---

## 2. System Overview

### 2.1 System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        Application Layer                         │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │                      main.cpp                            │    │
│  │  - Initialize hardware (NVS, I2C, SD card, Display)     │    │
│  │  - Launch SimpleMap with initial coordinates            │    │
│  └─────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                      SimpleMap Component                         │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │  simple_map.cpp / simple_map.hpp                        │    │
│  │  - Map container and tile widget management             │    │
│  │  - Input panel (lat/lon/zoom controls)                  │    │
│  │  - Scroll event handling and tile loading               │    │
│  │  - GPS coordinate conversion                            │    │
│  └─────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────┘
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
│  │  esp32_s3_touch_lcd_3_5b Component                      │    │
│  │  - Display driver (AXS15231B via QSPI)                  │    │
│  │  - Touch controller (AXS15231B via I2C)                 │    │
│  │  - SD card interface (SDMMC 1-bit mode)                 │    │
│  │  - I2C bus management                                   │    │
│  │  - IO expander (TCA9554) for LCD reset                  │    │
│  │  - Backlight PWM control                                │    │
│  └─────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                      Hardware Layer                              │
│  ┌────────────┐ ┌────────────┐ ┌────────────┐ ┌────────────┐   │
│  │  ESP32-S3  │ │ AXS15231B  │ │  TCA9554   │ │  SD Card   │   │
│  │   R8      │ │  Display   │ │ IO Expand  │ │  (FAT32)   │   │
│  └────────────┘ └────────────┘ └────────────┘ └────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 Hardware Platform

#### 2.2.1 Waveshare ESP32-S3-Touch-LCD-3.5B Specifications

| Component | Specification |
|-----------|---------------|
| MCU | ESP32-S3R8 (Xtensa LX7 dual-core, 240 MHz) |
| Flash | 16 MB |
| PSRAM | 8 MB (Octal) |
| Display | 3.5" IPS LCD, 320x480 pixels |
| Display Controller | AXS15231B (QSPI interface) |
| Touch Controller | Integrated in AXS15231B (I2C) |
| SD Card Interface | SDMMC native 1-bit mode |
| IO Expander | TCA9554PWR (I2C address 0x20) |

#### 2.2.2 Pin Configuration

| Function | GPIO | Notes |
|----------|------|-------|
| **I2C Bus** | | |
| I2C SCL | GPIO 7 | 400 kHz |
| I2C SDA | GPIO 8 | |
| **QSPI Display** | | |
| LCD CS | GPIO 12 | |
| LCD PCLK | GPIO 5 | 40 MHz |
| LCD DATA0 | GPIO 1 | |
| LCD DATA1 | GPIO 2 | |
| LCD DATA2 | GPIO 3 | |
| LCD DATA3 | GPIO 4 | |
| LCD Backlight | GPIO 6 | PWM controlled |
| LCD Reset | EXIO1 | Via IO expander |
| **SD Card (SDMMC)** | | |
| SD CLK | GPIO 11 | |
| SD CMD | GPIO 10 | |
| SD D0 | GPIO 9 | |

---

## 3. Functional Requirements

### 3.1 Map Display Functions

#### FR-3.1.1 Tile Grid Display
- **Description:** Display a configurable grid of map tiles (default 5x5)
- **Input:** Tile coordinates (X, Y) and zoom level
- **Output:** Visual map display on LCD
- **Behavior:**
  - Load tiles from SD card at `/sdcard/tiles1/{zoom}/{x}/{y}.bin`
  - Each tile is 256x256 pixels in RGB565 format
  - Grid creates a 1280x1280 pixel scrollable area (5x5 tiles)

#### FR-3.1.2 Touch Scrolling
- **Description:** Allow users to pan the map by touch dragging
- **Input:** Touch drag gesture
- **Output:** Map scrolls in the direction of the drag
- **Behavior:**
  - Smooth scrolling without momentum (momentum disabled)
  - When scroll reaches edge (>90 pixels beyond boundary), load adjacent tiles
  - Update GPS coordinates in real-time during scroll (50ms throttle)

#### FR-3.1.3 Zoom Level Control
- **Description:** Change map zoom level between 10-19
- **Input:** Tap on zoom in (+) or zoom out (-) button
- **Output:** Map reloads at new zoom level
- **Behavior:**
  - "+" button increases zoom by 1 level (max 19)
  - "-" button decreases zoom by 1 level (min 10)
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
  - Maximum zoom level: 19
  - Disabled during tile loading operations

#### FR-3.3.2 Zoom Out Button
- **Description:** Circular button to decrease zoom level by 1
- **Position:** Lower right corner of screen (15px from edges)
- **Size:** 50x50 pixels, circular (radius 25)
- **Appearance:** Dark gray background (80% opacity), white border, white "-" symbol
- **Behavior:**
  - Decreases zoom level by 1 when tapped
  - Minimum zoom level: 10
  - Disabled during tile loading operations

#### FR-3.3.3 Battery Status Indicator
- **Description:** Visual indicator showing battery level and charging status
- **Position:** Upper right corner of screen (10px from edges)
- **Size:** 70x30 pixels
- **Components:**
  - Battery icon (rectangle with fill level indicator)
  - Battery tip (small nub on right side)
  - Percentage label (e.g., "85%")
- **Color Coding:**
  - Green: Battery > 50%
  - Orange: Battery 21-50%
  - Red: Battery <= 20%
  - Blue: Charging
  - Gray: Unknown/No battery
- **API:** `SimpleMap::update_battery_indicator(int percent, bool is_charging)`

#### FR-3.3.4 Loading Indicator
- **Description:** Visual feedback during tile loading
- **Appearance:** Yellow pill-shaped popup with "Loading map..." text
- **Behavior:**
  - Appears when tile loading begins
  - Disappears when all tiles loaded
  - Prevents multiple simultaneous load operations

---

## 4. Non-Functional Requirements

### 4.1 Performance

| Metric | Requirement |
|--------|-------------|
| Tile load time | < 500ms for 25 tiles |
| Scroll response | < 50ms latency |
| GPS update rate | 20 Hz during scroll |
| Memory usage | < 4 MB PSRAM for tiles |

### 4.2 Memory Configuration

| Resource | Allocation |
|----------|------------|
| Main task stack | 10,240 bytes minimum |
| LVGL buffer | 320 x 100 pixels |
| Tile cache | 25 tiles x 131 KB = ~3.3 MB |
| Total PSRAM | 8 MB available |

### 4.3 Storage Requirements

| Item | Requirement |
|------|-------------|
| SD card format | FAT32 |
| Tile format | RGB565 binary (131,072 bytes/tile) |
| Tile path | `/sdcard/tiles1/{zoom}/{x}/{y}.bin` |
| Supported zoom levels | 10-19 |

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
│  - Mount SD │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│    IDLE     │◄────────────────────┐
│  - Display  │                     │
│    map      │                     │
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
| STARTUP | Hardware initialization | Init complete → IDLE |
| IDLE | Map displayed, awaiting input | Touch → SCROLLING, Button → LOADING |
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
    static void show_location(double latitude, double longitude, int zoom_level = 18);

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

// Backlight
esp_err_t bsp_display_brightness_init(void);
esp_err_t bsp_display_brightness_set(int brightness_percent);
esp_err_t bsp_display_backlight_on(void);
esp_err_t bsp_display_backlight_off(void);

// SD Card
esp_err_t bsp_sdcard_mount(void);
esp_err_t bsp_sdcard_unmount(void);

// IO Expander
esp_io_expander_handle_t bsp_io_expander_init(void);
```

---

## 8. Dependencies

### 8.1 Software Dependencies

| Component | Version | Source |
|-----------|---------|--------|
| ESP-IDF | >= 5.4.0 | Espressif |
| LVGL | 9.3.0 | lvgl/lvgl |
| map_tiles | ^1.2.0 | 0015/map_tiles |
| esp_lcd_axs15231b | ^2.0.2 | espressif |
| esp_lvgl_port | ^2.4.1 | espressif |
| esp_io_expander | ^1.0.0 | espressif |
| esp_io_expander_tca9554 | ^2.0.0 | espressif |

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
CONFIG_BSP_DISPLAY_LVGL_BUF_HEIGHT=100
CONFIG_ESP_MAIN_TASK_STACK_SIZE=10240
```

---

## 9. Error Handling

### 9.1 Error Conditions

| Condition | Detection | Response |
|-----------|-----------|----------|
| SD card not mounted | bsp_sdcard_mount() returns error | Log error, display fails |
| Tile file not found | map_tiles_load_tile() returns false | Display gray placeholder |
| Invalid coordinates | Validation in textarea_event_cb | Reject input, keep previous value |
| PSRAM exhausted | Allocation failure | Graceful degradation with smaller grid |
| Display init failure | bsp_display_start() returns NULL | Log error, abort |

### 9.2 Debug Output

Key log messages for troubleshooting:
```
"SimpleMap: Initialized with grid size %dx%d (%d tiles)"
"SimpleMap: Loading %dx%d grid at position (%d,%d) at zoom %d"
"SimpleMap: Tile loading completed in %lu ms"
"SimpleMap: Changing zoom from %d to %d"
"SimpleMap: Centered map on GPS %.6f, %.6f"
```

---

## 10. Future Enhancements

### 10.1 Planned Features
- GPS module integration for real-time location tracking
- Multiple tile layer support (street, satellite, terrain)
- Waypoint markers and route display
- Offline tile download via WiFi
- Gesture support (pinch-to-zoom)

### 10.2 Hardware Compatibility
The SimpleMap component is designed for portability across ESP32 platforms:
- ESP32-P4 boards (higher resolution displays)
- ESP32-S3 AMOLED displays
- Custom hardware with LVGL support

---

## Appendix A: File Structure

```
waveshare_esp32_s3_touch_lcd_3_5b/
├── CMakeLists.txt
├── partitions.csv
├── sdkconfig.defaults
├── docs/
│   └── FSD.md                          # This document
├── main/
│   ├── CMakeLists.txt
│   ├── idf_component.yml
│   └── main.cpp                        # Application entry point
└── components/
    └── esp32_s3_touch_lcd_3_5b/        # Board Support Package
        ├── CMakeLists.txt
        ├── idf_component.yml
        ├── Kconfig
        ├── esp32_s3_touch_lcd_3_5b.c   # BSP implementation
        ├── include/
        │   └── bsp/
        │       ├── esp32_s3_touch_lcd_3_5b.h
        │       ├── display.h
        │       ├── touch.h
        │       └── config.h
        └── priv_include/
            └── bsp_err_check.h
```

---

## Appendix B: Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2026-01-29 | Claude Opus 4.5 | Initial release |
