# Map Tiles Projects - Example Repository

This repository contains example projects demonstrating the use of the **0015__map_tiles** component ([ESP Registry](https://components.espressif.com/components/0015/map_tiles)) for interactive map display on ESP32 devices.

## Repository Structure

This repository is organized into separate example projects, each showcasing different use cases and hardware configurations:

```
map_tiles_projects/
├── 01.Simple_Map/                  # Basic interactive map viewer with GPS navigation
│   ├── espressif_esp32_p4_function_ev_board/
│   ├── waveshare_esp32_p4_wifi6_touch_lcd_xc/
│   ├── waveshare_esp32_s3_touch_amoled_1_75/
│   └── shared_components/
└── 02.ESP32-S3_Map_LoRa_GPS/       # Map display with LoRa communication and GPS tracking
```

## Examples

### 01.Simple_Map

A comprehensive example demonstrating interactive map display with touch controls, real-time GPS coordinate tracking, zoom functionality, and manual coordinate entry. Supports multiple ESP32 development boards with different display configurations.

**[View Project →](01.Simple_Map/)**

### 02.ESP32-S3_Map_LoRa_GPS

An advanced example integrating map display with LoRa wireless communication and GPS tracking capabilities on ESP32-S3.

**[View Project →](02.ESP32-S3_Map_LoRa_GPS/)**

## About the Map Tiles Component

The **0015__map_tiles** component provides:
- Efficient map tile loading from SD card
- Support for standard tile formats (256x256 RGB565)
- Integration with LVGL 9.x graphics library
- Memory-optimized tile management
- Configurable zoom levels and tile providers

## Getting Started

Each example project contains:
- Complete ESP-IDF project structure
- Hardware-specific configurations
- Detailed README with setup instructions
- Example code demonstrating best practices

Navigate to any example folder to get started!

## Requirements

- ESP-IDF 5.4 or later
- LVGL 9.3 or later
- ESP32 device with display support
- SD card for map tile storage

## License

See [LICENSE](LICENSE) file for details.

## Contributing

More example projects will be added to demonstrate additional use cases and features of the map tiles library.
