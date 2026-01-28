# Simple Map Demo for Waveshare ESP32-S3-Touch-LCD-3.5B

This project demonstrates an interactive map display on the Waveshare ESP32-S3-Touch-LCD-3.5B development board.

## Hardware Specifications

- **Display**: 3.5" IPS TFT, 320x480 pixels, 262K colors
- **Display Controller**: AXS15231B (QSPI interface)
- **Touch**: Integrated capacitive touch in AXS15231B (I2C interface)
- **Processor**: ESP32-S3R8 (dual-core Xtensa LX7, up to 240MHz)
- **Memory**: 8MB PSRAM (Octal), 16MB Flash
- **Storage**: TF/SD card slot

## Pin Configuration

### QSPI Display Interface
| Function | GPIO |
|----------|------|
| LCD_CS   | GPIO12 |
| LCD_CLK  | GPIO5 |
| LCD_D0   | GPIO1 |
| LCD_D1   | GPIO2 |
| LCD_D2   | GPIO3 |
| LCD_D3   | GPIO4 |
| Backlight| GPIO6 |

### I2C Interface (Touch & Sensors)
| Function | GPIO |
|----------|------|
| SDA      | GPIO8 |
| SCL      | GPIO7 |

### SD Card (SD_MMC)
| Function | GPIO |
|----------|------|
| SD_D0    | GPIO11 |
| SD_CMD   | GPIO13 |
| SD_CLK   | GPIO14 |

## Prerequisites

- ESP-IDF v5.4.0 or later
- SD card with map tiles (see main project README for tile preparation)

## Building

```bash
cd waveshare_esp32_s3_touch_lcd_3_5b
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

## Map Tiles

Place your map tiles on the SD card in the following structure:
```
/sdcard/tiles1/
    /zoom_level/
        /x_tile/
            y_tile.png
```

## Features

- Pan the map by touch and drag
- Zoom controls via on-screen slider
- GPS coordinate input fields
- Loads map tiles from SD card

## References

- [Waveshare ESP32-S3-Touch-LCD-3.5B Wiki](https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-3.5B)
- [AXS15231B Display Driver](https://components.espressif.com/components/espressif/esp_lcd_axs15231b)
