# Functional Specification Document (FSD)
## ESP32-S3 GPS Tracker with LoRa Communication and Map Display

**Version:** 1.0
**Date:** January 29, 2026
**Project:** Multi-Device GPS Tracking System with LoRa Communication

---

## 1. Introduction

### 1.1 Purpose
This document specifies the functional requirements and system architecture for a real-time GPS tracking application running on the Waveshare ESP32-S3-Touch-LCD-3.49 development board. The application provides multi-device location tracking using LoRa long-range communication, interactive map display with touch navigation, and proximity-based audio alarms.

### 1.2 Scope
The system provides:
- Real-time GPS position tracking via LC76G GNSS module
- LoRa-based device-to-device communication (3-5km range) via RYLR998 module
- Interactive touch-scrollable map display with multiple device markers
- Proximity alarm system with configurable thresholds
- Battery monitoring and power management
- Multi-device tracking (up to 10 remote devices)

### 1.3 Definitions and Acronyms

| Term | Definition |
|------|------------|
| GNSS | Global Navigation Satellite System |
| GPS | Global Positioning System |
| LoRa | Long Range radio communication |
| LVGL | Light and Versatile Graphics Library |
| NMEA | National Marine Electronics Association (GPS data format) |
| RSSI | Received Signal Strength Indicator |
| SNR | Signal-to-Noise Ratio |
| Tile | 256x256 pixel map image segment |
| Zoom Level | Map detail level (higher = more detail) |
| Haversine | Formula for calculating great-circle distances |

---

## 2. System Overview

### 2.1 System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           Application Layer                                  │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                           main.cpp                                   │    │
│  │  - Initialize hardware (LCD, I2C, SD card, Audio)                   │    │
│  │  - Launch FreeRTOS tasks (GPS, LoRa, Battery, Alarm, Power)        │    │
│  │  - Display splash screen and initialize map                         │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
        ┌───────────────────────────┼───────────────────────────┐
        ▼                           ▼                           ▼
┌───────────────────┐   ┌───────────────────┐   ┌───────────────────────────┐
│   UserMap Class   │   │    GPS Driver     │   │      LoRa Driver          │
│  ┌─────────────┐  │   │  ┌─────────────┐  │   │  ┌─────────────────────┐  │
│  │ user_map.cpp│  │   │  │gps_lc76g.c  │  │   │  │  lora_rylr998.c     │  │
│  │ - Map tiles │  │   │  │- NMEA parse │  │   │  │  - AT commands      │  │
│  │ - GPS marker│  │   │  │- Coordinates│  │   │  │  - TX/RX handling   │  │
│  │ - Remote    │  │   │  │- Satellites │  │   │  │  - Network config   │  │
│  │   markers   │  │   │  │- Fix status │  │   │  │  - GPS data extract │  │
│  │ - Zoom ctrl │  │   │  └─────────────┘  │   │  └─────────────────────┘  │
│  │ - Distance  │  │   └───────────────────┘   └───────────────────────────┘
│  │   calc      │  │               │                       │
│  └─────────────┘  │               │                       │
└───────────────────┘               │                       │
        │                           │                       │
        ▼                           ▼                       ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                          Component Layer                                     │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  ┌────────────────┐   │
│  │  map_tiles   │  │  audio_bsp   │  │  adc_bsp     │  │  device_config │   │
│  │  Component   │  │  Component   │  │  Component   │  │    Component   │   │
│  │  - Tile load │  │  - PCM play  │  │  - Battery   │  │  - Device ID   │   │
│  │  - Coord conv│  │  - I2S audio │  │    voltage   │  │  - Alarm thresh│   │
│  └──────────────┘  └──────────────┘  └──────────────┘  └────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                          Hardware Layer                                      │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌────────┐ ┌────────┐ │
│  │ ESP32-S3 │ │AXS15231B │ │  LC76G   │ │ RYLR998  │ │  Audio │ │SD Card │ │
│  │   MCU    │ │  LCD     │ │   GPS    │ │   LoRa   │ │ Codec  │ │(FAT32) │ │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘ └────────┘ └────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 2.2 Hardware Platform

#### 2.2.1 Waveshare ESP32-S3-Touch-LCD-3.49 Specifications

| Component | Specification |
|-----------|---------------|
| MCU | ESP32-S3 (Xtensa LX7 dual-core, 240 MHz) |
| Flash | 16 MB |
| PSRAM | 8 MB (Octal) |
| Display | 3.49" LCD Bar, 640x172 pixels |
| Display Controller | AXS15231B (8-bit parallel interface) |
| Touch Controller | FT5336 (I2C interface, address 0x3B) |
| SD Card Interface | SPI (SPI2_HOST) |
| GPS Module | LC76G GNSS (UART2, 115200 baud) |
| LoRa Module | RYLR998 (UART1, 115200 baud, 915 MHz) |
| Audio | I2S codec board (11025 Hz sample rate) |
| Power | AXP2101 PMIC with battery monitoring |

#### 2.2.2 Pin Configuration

| Function | GPIO | Notes |
|----------|------|-------|
| **I2C Bus (Touch)** | | FT5336 touch controller |
| I2C SCL | GPIO 18 | |
| I2C SDA | GPIO 17 | |
| **I2C Bus (ESP)** | | AXP2101 PMIC |
| I2C SCL | GPIO 48 | |
| I2C SDA | GPIO 47 | |
| **LCD (8-bit Parallel)** | | |
| LCD CS | GPIO 45 | |
| LCD RST | GPIO 4 | |
| LCD DC | GPIO 0 | |
| LCD WR | GPIO 35 | |
| LCD DATA0-7 | GPIO 36-43 | |
| LCD Backlight | GPIO 8 | PWM controlled |
| **GPS (UART2)** | | LC76G module |
| GPS TX | GPIO 5 | |
| GPS RX | GPIO 3 | |
| **LoRa (UART1)** | | RYLR998 module |
| LoRa TX | GPIO 1 | |
| LoRa RX | GPIO 2 | |
| **SD Card (SPI)** | | |
| SD CS | GPIO 9 | |
| SD MOSI | GPIO 11 | |
| SD MISO | GPIO 12 | |
| SD CLK | GPIO 10 | |
| **Power** | | |
| Power Button | GPIO 16 | Battery vs USB detection |
| **Audio (I2S)** | | Codec board |
| I2S BCK | GPIO 46 | |
| I2S WS | GPIO 14 | |
| I2S DATA | GPIO 13 | |

---

## 3. Functional Requirements

### 3.1 GPS Functions

#### FR-3.1.1 GPS Position Tracking
- **Description:** Continuously track device position using LC76G GNSS module
- **Input:** NMEA sentences from GPS module via UART
- **Output:** Latitude, longitude, altitude, speed, heading
- **Behavior:**
  - Parse GGA, RMC, GSA, GSV NMEA sentences
  - Update position at 1 Hz (configurable)
  - Track satellite count (up to 47 satellites)
  - Report fix quality (No fix, GPS, DGPS, RTK)
  - Calculate HDOP/VDOP precision values

#### FR-3.1.2 GPS Data Callback
- **Description:** Notify application of GPS updates
- **Input:** Parsed GPS data structure
- **Output:** Callback invocation with GPS data
- **Behavior:**
  - Register callback via `gps_lc76g_set_data_callback()`
  - Callback receives: lat, lon, altitude, speed, course, satellites, fix type

#### FR-3.1.3 Distance and Bearing Calculation
- **Description:** Calculate distance and bearing between two GPS coordinates
- **Algorithm:** Haversine formula for great-circle distance
- **Methods:**
  - `gps_lc76g_haversine_distance()` - returns distance in feet
  - `gps_lc76g_bearing()` - returns bearing in degrees

### 3.2 LoRa Communication Functions

#### FR-3.2.1 GPS Position Broadcasting
- **Description:** Transmit local GPS position to other devices
- **Frequency:** Every 10 seconds
- **Format:** `DeviceName,DeviceType,Latitude,Longitude`
- **Broadcast Address:** 6 (configurable)
- **Behavior:**
  - Only transmit when GPS has valid fix
  - Include device identification for marker differentiation
  - Use network ID and password for isolation

#### FR-3.2.2 Remote Position Reception
- **Description:** Receive GPS positions from remote devices
- **Input:** LoRa messages with GPS data payload
- **Output:** Parsed device position data
- **Behavior:**
  - Extract sender address, payload, RSSI, SNR
  - Parse CSV format: `Name,Type,Lat,Lon`
  - Update remote device markers on map
  - Track last update time per device

#### FR-3.2.3 LoRa Network Configuration
- **Description:** Configure LoRa module parameters
- **Parameters:**
  - Network ID: Isolate device groups
  - Password: "EEDCAA90" (default)
  - TX Power: Up to 22 dBm
  - Spreading Factor: SF5-SF12
  - Bandwidth and coding rate

### 3.3 Map Display Functions

#### FR-3.3.1 Tile Grid Display
- **Description:** Display map tiles in a scrollable view
- **Input:** Tile coordinates (X, Y) and zoom level
- **Output:** Visual map display on LCD
- **Behavior:**
  - Load tiles from SD card at `/sdcard/tiles1/{zoom}/{x}/{y}.bin`
  - Each tile is 256x256 pixels in RGB565 format
  - Display fits within 640x172 pixel screen

#### FR-3.3.2 Touch Scrolling
- **Description:** Allow users to pan the map by touch dragging
- **Input:** Touch drag gesture
- **Output:** Map scrolls in the direction of the drag
- **Behavior:**
  - Smooth scrolling with edge detection
  - Load adjacent tiles when reaching boundaries
  - Update display coordinates in real-time

#### FR-3.3.3 Zoom Level Control
- **Description:** Change map zoom level between 10-19
- **Default Zoom:** 18
- **Input:** Tap on zoom in (+) or zoom out (-) button
- **Output:** Map reloads at new zoom level
- **Behavior:**
  - "+" button increases zoom (max 19)
  - "-" button decreases zoom (min 10)
  - Map recenters on current GPS position after zoom change
  - Loading popup displays during tile reload

### 3.4 Device Marker Functions

#### FR-3.4.1 Local GPS Marker
- **Description:** Display current device position on map
- **Position:** Centered on screen
- **Appearance:** Device-specific icon and color
- **Behavior:**
  - Updates when GPS position changes
  - Shows direction arrow when GPS location is off-screen
  - Icon based on device type (Mom/Kid)

#### FR-3.4.2 Remote Device Markers
- **Description:** Display positions of remote tracked devices
- **Capacity:** Up to 10 simultaneous remote devices
- **Appearance:**
  - Mom device: Pink marker with specific icon
  - Kid device: Blue marker with specific icon
- **Behavior:**
  - Create marker when first receiving device position
  - Update marker position on subsequent messages
  - Show direction arrows when markers are off-screen
  - Display last update timestamp

#### FR-3.4.3 Off-Screen Direction Arrows
- **Description:** Indicate direction of off-screen markers
- **Appearance:** Arrows pointing toward off-screen devices
- **Position:** Edge of screen in direction of device
- **Behavior:**
  - Calculate bearing to each off-screen device
  - Update arrow positions as map moves

### 3.5 Proximity Alarm Functions

#### FR-3.5.1 Distance Threshold Monitoring
- **Description:** Monitor distance to remote devices against configured threshold
- **Default Threshold:** 100 feet (configurable per device)
- **Check Frequency:**
  - 1 second when alarm condition exists
  - 2 seconds when no alarm condition

#### FR-3.5.2 Audio Alarm Playback
- **Description:** Play audio alert when proximity threshold exceeded
- **Audio:** Embedded PCM audio (TF022 sound file)
- **Sample Rate:** 11025 Hz
- **Behavior:**
  - Trigger when any remote device exceeds distance threshold
  - Play via I2S audio codec
  - Continue monitoring while playing

#### FR-3.5.3 Distance Display
- **Description:** Show distance to closest remote device
- **Position:** Coordinate panel on map UI
- **Format:** Distance in feet
- **Update Rate:** Real-time with GPS updates

### 3.6 User Interface Functions

#### FR-3.6.1 Splash Screen
- **Description:** Display initialization screen during startup
- **Duration:** Until GPS lock acquired and map loaded
- **Components:**
  - Application logo/title
  - Loading spinner animation
  - GPS acquisition status

#### FR-3.6.2 Coordinate Panel
- **Description:** Display current GPS coordinates and status
- **Position:** Configurable panel on screen
- **Content:**
  - Latitude (6 decimal places)
  - Longitude (6 decimal places)
  - Distance to closest device (feet)
- **Layout:** Flex container with auto-positioning

#### FR-3.6.3 Battery Voltage Display
- **Description:** Show battery voltage in upper left corner
- **Format:** "X.XXV" voltage reading
- **Update Rate:** Every 2 seconds

#### FR-3.6.4 Zoom Buttons
- **Description:** Touch buttons for zoom control
- **Appearance:** "+" and "-" buttons
- **Behavior:**
  - Disabled during tile loading
  - Visual feedback on tap

#### FR-3.6.5 Settings Button
- **Description:** Access device configuration
- **Function:** Open settings menu (placeholder for future expansion)

#### FR-3.6.6 Notification Border
- **Description:** Status indicator border around map
- **Purpose:** Visual indication of system status
- **Colors:** Based on current state (normal, warning, alarm)

### 3.7 Power Management Functions

#### FR-3.7.1 Battery Monitoring
- **Description:** Monitor battery voltage via ADC
- **Update Rate:** Every 2 seconds
- **Output:** Voltage displayed on UI

#### FR-3.7.2 Power Source Detection
- **Description:** Detect battery vs USB power
- **Input:** GPIO 16 power button state
- **Behavior:**
  - Monitor power button for source changes
  - Update UI based on power source

#### FR-3.7.3 Backlight Dimming
- **Description:** Automatic backlight control
- **Behavior:**
  - Dim on touch inactivity
  - Full brightness on touch
  - PWM control via GPIO 8

---

## 4. Non-Functional Requirements

### 4.1 Performance

| Metric | Requirement |
|--------|-------------|
| GPS update rate | 1 Hz |
| LoRa broadcast interval | 10 seconds |
| LoRa range | 3-5 km typical |
| Tile load time | < 500ms per grid |
| UI response | < 50ms latency |
| Battery monitor update | 2 seconds |
| Alarm check interval | 1-2 seconds |

### 4.2 Memory Configuration

| Resource | Allocation |
|----------|------------|
| Main task stack | 8,192 bytes |
| GPS task stack | 4,096 bytes |
| LoRa RX task stack | 4,096 bytes |
| Battery monitor task stack | 8,192 bytes |
| Alarm monitor task stack | 4,096 bytes |
| LVGL buffer | Display-sized |
| PSRAM | 8 MB available |

### 4.3 Storage Requirements

| Item | Requirement |
|------|-------------|
| SD card format | FAT32 |
| Tile format | RGB565 binary (131,072 bytes/tile) |
| Tile path | `/sdcard/tiles1/{zoom}/{x}/{y}.bin` |
| Supported zoom levels | 10-19 |

### 4.4 Communication Specifications

| Parameter | GPS (LC76G) | LoRa (RYLR998) |
|-----------|-------------|----------------|
| Interface | UART2 | UART1 |
| Baud rate | 115200 | 115200 |
| Protocol | NMEA 0183 | AT Commands |
| Update rate | 1 Hz | Event-driven |
| Frequency | - | 915 MHz |

---

## 5. Data Formats

### 5.1 GPS NMEA Sentences

```
GGA - Global Positioning System Fix Data
RMC - Recommended Minimum Navigation Information
GSA - GPS DOP and Active Satellites
GSV - GPS Satellites in View
VTG - Track Made Good and Ground Speed
```

### 5.2 LoRa Message Format

```
Transmit:
  AT+SEND={address},{length},{payload}

Payload format:
  DeviceName,DeviceType,Latitude,Longitude
  Example: "Mom,0,37.123456,-122.654321"

Receive:
  +RCV={address},{length},{payload},{RSSI},{SNR}
```

### 5.3 Tile File Format

```
File: /sdcard/tiles1/{zoom}/{x}/{y}.bin
Size: 131,072 bytes (256 x 256 x 2)
Format: Raw RGB565 (16-bit per pixel)
Byte Order: Little-endian
```

### 5.4 Device Configuration

```
Device Types:
  DEVICE_TYPE_MOM = 0  (Pink marker)
  DEVICE_TYPE_KID = 1  (Blue marker)

Default Alarm Threshold: 100 feet
Network Password: "EEDCAA90"
Broadcast Address: 6
```

---

## 6. System States

### 6.1 State Diagram

```
┌──────────────────┐
│     STARTUP      │
│  - Init hardware │
│  - Init LVGL     │
│  - Show splash   │
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│  WAITING_GPS     │◄──────────────────────────────┐
│  - Show spinner  │                               │
│  - Parse NMEA    │                               │
└────────┬─────────┘                               │
         │ GPS Fix acquired                        │
         ▼                                         │
┌──────────────────┐                               │
│   MAP_LOADING    │                               │
│  - Load tiles    │                               │
│  - Show popup    │                               │
└────────┬─────────┘                               │
         │                                         │
         ▼                                         │
┌──────────────────┐     Touch scroll     ┌───────┴────────┐
│      IDLE        │◄────────────────────►│   SCROLLING    │
│  - Display map   │                      │  - Pan map     │
│  - Show markers  │                      │  - Load tiles  │
│  - Monitor GPS   │                      └────────────────┘
│  - Monitor LoRa  │
│  - Check alarms  │
└────────┬─────────┘
         │ Proximity threshold exceeded
         ▼
┌──────────────────┐
│    ALARMING      │
│  - Play audio    │
│  - Flash border  │
│  - 1s check rate │
└──────────────────┘
```

### 6.2 State Descriptions

| State | Description | Exit Conditions |
|-------|-------------|-----------------|
| STARTUP | Hardware initialization, LVGL setup | Init complete → WAITING_GPS |
| WAITING_GPS | Waiting for GPS satellite fix | GPS fix → MAP_LOADING |
| MAP_LOADING | Loading map tiles from SD card | Tiles loaded → IDLE |
| IDLE | Normal operation with map displayed | Touch → SCROLLING, Threshold → ALARMING |
| SCROLLING | User panning the map | Touch release → IDLE |
| ALARMING | Proximity alarm active | Distance < threshold → IDLE |

---

## 7. FreeRTOS Task Architecture

### 7.1 Task Overview

| Task | Priority | Stack | Purpose |
|------|----------|-------|---------|
| GPS Map Update | Normal | 4KB | Parse GPS, update map |
| GPS Broadcast | Normal | 4KB | Transmit position via LoRa |
| LoRa RX | Normal | 4KB | Receive remote positions |
| Battery Monitor | Low | 8KB | Read ADC, update display |
| Alarm Monitor | High | 4KB | Check proximity, play audio |
| Power Button | Normal | 4KB | Monitor power source |
| LVGL | Normal | 8KB | GUI rendering |

### 7.2 Synchronization

| Resource | Mechanism |
|----------|-----------|
| LVGL display | Mutex (bsp_display_lock/unlock) |
| GPS UART | Mutex |
| LoRa UART | Mutex |
| Remote device list | Mutex |
| LCD flush | Semaphore |

---

## 8. API Reference

### 8.1 UserMap Public Interface

```cpp
class UserMap {
public:
    // Initialize map system on given LVGL screen
    static bool init(lv_obj_t* parent_screen);

    // Display map at GPS coordinates with zoom level
    static void show_location(double latitude, double longitude, int zoom_level = 18);

    // Update local GPS marker position
    static void update_location(double latitude, double longitude);

    // Change zoom level and reload tiles
    static void change_zoom_level(int new_zoom);

    // Get current zoom level
    static int get_current_zoom();

    // Remote device marker management
    static void update_remote_marker(const char* device_id,
                                     double latitude, double longitude,
                                     int device_type);
    static void remove_remote_marker(const char* device_id);

    // Distance calculation
    static double get_distance_to_device(const char* device_id);
    static const char* get_closest_device_id();

    // Update battery display
    static void update_battery_display(float voltage);

    // Update distance label
    static void update_distance_label(double distance_feet);

    // Cleanup resources
    static void cleanup();
};
```

### 8.2 GPS Driver Interface

```c
// Initialize GPS module
esp_err_t gps_lc76g_init(uart_port_t uart_num, int tx_pin, int rx_pin);

// Start GPS processing task
esp_err_t gps_lc76g_start(void);

// Stop GPS processing
void gps_lc76g_stop(void);

// Get current GPS data
bool gps_lc76g_get_data(gps_data_t* data);

// Register data callback
void gps_lc76g_set_data_callback(gps_data_callback_t callback, void* user_data);

// Calculate distance between two points (returns feet)
double gps_lc76g_haversine_distance(double lat1, double lon1,
                                     double lat2, double lon2);

// Calculate bearing between two points (returns degrees)
double gps_lc76g_bearing(double lat1, double lon1,
                         double lat2, double lon2);
```

### 8.3 LoRa Driver Interface

```c
// Initialize LoRa module
esp_err_t lora_rylr998_init(uart_port_t uart_num, int tx_pin, int rx_pin);

// Configure network settings
esp_err_t lora_rylr998_set_network_id(uint32_t network_id);
esp_err_t lora_rylr998_set_password(const char* password);
esp_err_t lora_rylr998_set_address(uint16_t address);

// Send message
esp_err_t lora_rylr998_send(uint16_t address, const char* data, size_t len);

// Register receive callback
void lora_rylr998_set_rx_callback(lora_rx_callback_t callback, void* user_data);

// Parse GPS data from payload
bool lora_rylr998_parse_gps_payload(const char* payload,
                                    char* name, int* type,
                                    double* lat, double* lon);

// Start RX task
esp_err_t lora_rylr998_start_rx_task(void);
```

### 8.4 Device Configuration Interface

```c
// Get local device configuration
const char* device_config_get_name(void);
int device_config_get_type(void);
uint32_t device_config_get_color(void);

// Alarm threshold management
int device_config_get_alarm_threshold(void);
void device_config_set_alarm_threshold(int threshold_feet);

// Device type constants
#define DEVICE_TYPE_MOM 0
#define DEVICE_TYPE_KID 1
```

---

## 9. Dependencies

### 9.1 Software Dependencies

| Component | Version | Source |
|-----------|---------|--------|
| ESP-IDF | >= 5.0.4 | Espressif |
| LVGL | 9.3.0+ | lvgl/lvgl |
| map_tiles | ^1.2.0 | 0015/map_tiles |
| esp_lcd_axs15231b | ^1.0.1 | espressif |
| esp_io_expander_tca9554 | ^2.0.1 | espressif |

### 9.2 Build Configuration

Key sdkconfig settings:
```ini
CONFIG_IDF_TARGET="esp32s3"
CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_OCT=y
CONFIG_SPIRAM_SPEED_80M=y
CONFIG_LV_COLOR_DEPTH_16=y
CONFIG_LV_COLOR_16_SWAP=y
CONFIG_ESP_MAIN_TASK_STACK_SIZE=8192
```

---

## 10. Error Handling

### 10.1 Error Conditions

| Condition | Detection | Response |
|-----------|-----------|----------|
| GPS no fix | No valid NMEA data | Show spinner, wait for fix |
| GPS timeout | No data for 30 seconds | Log warning, continue waiting |
| LoRa TX failure | AT command error | Retry with backoff |
| LoRa RX timeout | No messages received | Continue listening |
| SD card not mounted | Mount failure | Log error, display fails |
| Tile file not found | File open error | Display gray placeholder |
| Audio playback failure | I2S error | Log error, continue monitoring |
| PSRAM exhausted | Allocation failure | Graceful degradation |

### 10.2 Debug Output

Key log messages:
```
"GPS: Fix acquired - Lat: %.6f, Lon: %.6f, Sats: %d"
"GPS: No fix - searching for satellites..."
"LoRa: TX -> Addr:%d, Data:%s"
"LoRa: RX <- Addr:%d, RSSI:%d, SNR:%d, Data:%s"
"LoRa: Remote device update - %s at %.6f, %.6f"
"Alarm: Distance to %s = %.0f ft (threshold: %d ft)"
"Alarm: TRIGGERED - playing audio"
"Battery: %.2fV"
```

---

## 11. Future Enhancements

### 11.1 Planned Features
- Geofence configuration and alerts
- Track history and breadcrumb trail
- WiFi connectivity for remote monitoring
- Multiple alarm thresholds per device
- Power saving modes (reduced GPS rate)
- OTA firmware updates
- Web-based configuration interface

### 11.2 Hardware Compatibility
The system architecture supports adaptation to:
- Different ESP32-S3 display boards
- Alternative GPS modules (u-blox, etc.)
- Different LoRa modules (SX1276, etc.)
- Higher resolution displays

---

## Appendix A: File Structure

```
02.ESP32-S3_Map_LoRa_GPS/
├── CMakeLists.txt
├── partitions.csv
├── sdkconfig.defaults
├── docs/
│   └── FSD.md                          # This document
├── main/
│   ├── CMakeLists.txt
│   ├── idf_component.yml
│   ├── main.cpp                        # Application entry point
│   ├── user_map.cpp                    # Map display component
│   ├── user_map.hpp                    # Map display header
│   ├── gps_lc76g.c                     # GPS driver implementation
│   ├── gps_lc76g.h                     # GPS driver header
│   ├── lora_rylr998.c                  # LoRa driver implementation
│   ├── lora_rylr998.h                  # LoRa driver header
│   ├── device_config.c                 # Device configuration
│   ├── device_config.h                 # Device configuration header
│   ├── multi_button.c                  # Button driver
│   └── multi_button.h                  # Button driver header
└── components/
    ├── i2c_bsp/                        # I2C Board Support Package
    ├── adc_bsp/                        # ADC for battery monitoring
    ├── sdcard_bsp/                     # SD card driver
    ├── lcd_bl_pwm_bsp/                 # LCD backlight PWM control
    ├── audio_bsp/                      # Audio driver
    ├── button_bsp/                     # Button input handling
    └── codec_board/                    # Audio codec support
```

---

## Appendix B: Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2026-01-29 | Claude Opus 4.5 | Initial release |
