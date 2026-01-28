/*
 * SPDX-FileCopyrightText: 2024
 * SPDX-License-Identifier: Apache-2.0
 *
 * BSP for Waveshare ESP32-S3-Touch-LCD-3.5B
 * Display: AXS15231B (320x480) via QSPI
 * Touch: Integrated in AXS15231B via I2C
 * SD Card: SPI mode with CS on IO expander
 */

#include <stdio.h>
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_vfs_fat.h"
#include "esp_spiffs.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_lcd_axs15231b.h"
#include "esp_io_expander_tca9554.h"

#include "bsp/esp32_s3_touch_lcd_3_5b.h"
#include "bsp_err_check.h"
#include "bsp/display.h"
#include "bsp/touch.h"

static const char *TAG = "ESP32-S3-Touch-LCD-3.5B";

static i2c_master_bus_handle_t i2c_handle = NULL;
static bool i2c_initialized = false;
static esp_io_expander_handle_t io_expander = NULL;
static lv_display_t *disp = NULL;
static lv_indev_t *disp_indev = NULL;
sdmmc_card_t *bsp_sdcard = NULL;
static esp_lcd_touch_handle_t tp = NULL;
static esp_lcd_panel_handle_t panel_handle = NULL;
static esp_lcd_panel_io_handle_t io_handle = NULL;

#define LCD_CMD_BITS (8)
#define LCD_PARAM_BITS (8)
#define LCD_LEDC_CH (CONFIG_BSP_DISPLAY_BRIGHTNESS_LEDC_CH)
#define LVGL_TICK_PERIOD_MS (CONFIG_BSP_DISPLAY_LVGL_TICK)
#define LVGL_MAX_SLEEP_MS (CONFIG_BSP_DISPLAY_LVGL_MAX_SLEEP)

/**************************************************************************************************
 *
 * I2C Function
 *
 **************************************************************************************************/
esp_err_t bsp_i2c_init(void)
{
    /* I2C was initialized before */
    if (i2c_initialized)
    {
        return ESP_OK;
    }

    i2c_master_bus_config_t i2c_bus_conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .sda_io_num = BSP_I2C_SDA,
        .scl_io_num = BSP_I2C_SCL,
        .i2c_port = BSP_I2C_NUM,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
        .trans_queue_depth = 0,
    };
    BSP_ERROR_CHECK_RETURN_ERR(i2c_new_master_bus(&i2c_bus_conf, &i2c_handle));

    i2c_initialized = true;

    return ESP_OK;
}

esp_err_t bsp_i2c_deinit(void)
{
    BSP_ERROR_CHECK_RETURN_ERR(i2c_del_master_bus(i2c_handle));
    i2c_initialized = false;
    return ESP_OK;
}

i2c_master_bus_handle_t bsp_i2c_get_handle(void)
{
    bsp_i2c_init();
    return i2c_handle;
}

/**************************************************************************************************
 *
 * IO Expander Function
 *
 **************************************************************************************************/
esp_io_expander_handle_t bsp_io_expander_init(void)
{
    BSP_ERROR_CHECK_RETURN_NULL(bsp_i2c_init());
    if (!io_expander)
    {
        BSP_ERROR_CHECK_RETURN_NULL(esp_io_expander_new_i2c_tca9554(i2c_handle, BSP_IO_EXPANDER_I2C_ADDRESS, &io_expander));
    }
    return io_expander;
}

/**************************************************************************************************
 *
 * SPIFFS Function
 *
 **************************************************************************************************/
esp_err_t bsp_spiffs_mount(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = CONFIG_BSP_SPIFFS_MOUNT_POINT,
        .partition_label = CONFIG_BSP_SPIFFS_PARTITION_LABEL,
        .max_files = CONFIG_BSP_SPIFFS_MAX_FILES,
        .format_if_mount_failed = false,
    };

    esp_err_t ret_val = esp_vfs_spiffs_register(&conf);

    BSP_ERROR_CHECK_RETURN_ERR(ret_val);

    size_t total = 0, used = 0;
    ret_val = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret_val != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret_val));
    }
    else
    {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    return ret_val;
}

esp_err_t bsp_spiffs_unmount(void)
{
    return esp_vfs_spiffs_unregister(CONFIG_BSP_SPIFFS_PARTITION_LABEL);
}

/**************************************************************************************************
 *
 * SD Card Function (SPI mode with IO expander CS)
 *
 **************************************************************************************************/

/* Custom SPI device handle for SD card with IO expander CS */
static spi_device_handle_t sd_spi_handle = NULL;

/* Callback to control CS via IO expander */
static void sd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    if (io_expander) {
        esp_io_expander_set_level(io_expander, BSP_SD_CS_EXIO, 0);  // CS low (active)
    }
}

static void sd_spi_post_transfer_callback(spi_transaction_t *t)
{
    if (io_expander) {
        esp_io_expander_set_level(io_expander, BSP_SD_CS_EXIO, 1);  // CS high (inactive)
    }
}

esp_err_t bsp_sdcard_mount(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "Initializing SD card (SPI mode)");

    /* Initialize IO expander first for CS control */
    if (!io_expander) {
        io_expander = bsp_io_expander_init();
        if (!io_expander) {
            ESP_LOGE(TAG, "Failed to initialize IO expander for SD CS");
            return ESP_FAIL;
        }
    }

    /* Configure EXIO3 as output for SD CS */
    esp_io_expander_set_dir(io_expander, 1 << BSP_SD_CS_EXIO, IO_EXPANDER_OUTPUT);
    esp_io_expander_set_level(io_expander, 1 << BSP_SD_CS_EXIO, 1);  // CS high (inactive)

    /* Initialize SPI bus for SD card */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = BSP_SD_MOSI,
        .miso_io_num = BSP_SD_MISO,
        .sclk_io_num = BSP_SD_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    ret = spi_bus_initialize(BSP_SD_SPI_NUM, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus for SD card: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Mount configuration */
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    /* Use a GPIO for CS since IO expander CS requires special handling */
    /* We'll use a workaround: configure SD SPI with a dummy CS and handle it manually */
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = BSP_SD_SPI_NUM;

    /* For IO expander CS, we need to use gpio_cs approach or handle manually */
    /* Using -1 for CS and handling via IO expander callbacks is complex */
    /* Alternative: Use a spare GPIO as CS if available, or handle SD init differently */

    /* Since CS is on IO expander, we'll try a different approach:
     * Use sdspi with slot config that uses software CS control */
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = GPIO_NUM_NC;  // We handle CS via IO expander
    slot_config.host_id = BSP_SD_SPI_NUM;

    ESP_LOGI(TAG, "Mounting SD card filesystem");

    /* For boards with IO expander CS, we need to manually toggle CS
     * This requires a custom approach or patching the sdspi driver
     * As a workaround, let's try using the raw SPI with manual CS control */

    /* Try mounting - this may fail without proper CS control */
    /* The esp_vfs_fat_sdspi_mount expects GPIO-based CS */
    /* For now, let's try with -1 and see if the card responds */
    ret = esp_vfs_fat_sdspi_mount(BSP_SD_MOUNT_POINT, &host, &slot_config, &mount_config, &bsp_sdcard);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "Note: SD CS is on IO expander (EXIO%d), which may require custom SPI handling", BSP_SD_CS_EXIO);
        spi_bus_free(BSP_SD_SPI_NUM);
        return ret;
    }

    ESP_LOGI(TAG, "SD card mounted successfully");
    return ESP_OK;
}

esp_err_t bsp_sdcard_unmount(void)
{
    esp_err_t ret = esp_vfs_fat_sdcard_unmount(BSP_SD_MOUNT_POINT, bsp_sdcard);
    spi_bus_free(BSP_SD_SPI_NUM);
    return ret;
}

/**************************************************************************************************
 *
 * Display Brightness Functions
 *
 **************************************************************************************************/
esp_err_t bsp_display_brightness_init(void)
{
    // Configure LEDC for backlight PWM control
    const ledc_channel_config_t lcd_backlight_channel = {
        .gpio_num = BSP_LCD_BACKLIGHT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LCD_LEDC_CH,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0,
        .flags.output_invert = 0,
    };
    const ledc_timer_config_t lcd_backlight_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    BSP_ERROR_CHECK_RETURN_ERR(ledc_timer_config(&lcd_backlight_timer));
    BSP_ERROR_CHECK_RETURN_ERR(ledc_channel_config(&lcd_backlight_channel));

    return ESP_OK;
}

esp_err_t bsp_display_brightness_set(int brightness_percent)
{
    if (brightness_percent < 0 || brightness_percent > 100)
    {
        ESP_LOGE(TAG, "Invalid brightness percentage. Should be between 0 and 100.");
        return ESP_ERR_INVALID_ARG;
    }

    // Calculate duty cycle (10-bit resolution: 0-1023)
    uint32_t duty = (brightness_percent * 1023) / 100;

    BSP_ERROR_CHECK_RETURN_ERR(ledc_set_duty(LEDC_LOW_SPEED_MODE, LCD_LEDC_CH, duty));
    BSP_ERROR_CHECK_RETURN_ERR(ledc_update_duty(LEDC_LOW_SPEED_MODE, LCD_LEDC_CH));

    return ESP_OK;
}

esp_err_t bsp_display_backlight_off(void)
{
    ESP_LOGI(TAG, "Backlight off");
    return bsp_display_brightness_set(0);
}

esp_err_t bsp_display_backlight_on(void)
{
    ESP_LOGI(TAG, "Backlight on");
    return bsp_display_brightness_set(100);
}

/**************************************************************************************************
 *
 * Display Panel Functions (AXS15231B via QSPI)
 *
 **************************************************************************************************/
esp_err_t bsp_display_new(const bsp_display_config_t *config, esp_lcd_panel_handle_t *ret_panel, esp_lcd_panel_io_handle_t *ret_io)
{
    esp_err_t ret = ESP_OK;

    ESP_LOGI(TAG, "Initialize QSPI bus for AXS15231B display");

    // Configure SPI bus for QSPI operation
    const spi_bus_config_t buscfg = AXS15231B_PANEL_BUS_QSPI_CONFIG(
        BSP_LCD_PCLK,
        BSP_LCD_DATA0,
        BSP_LCD_DATA1,
        BSP_LCD_DATA2,
        BSP_LCD_DATA3,
        BSP_LCD_H_RES * 80 * sizeof(uint16_t)
    );
    ESP_ERROR_CHECK(spi_bus_initialize(BSP_LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO));

    // Configure panel IO for QSPI
    const esp_lcd_panel_io_spi_config_t io_config = AXS15231B_PANEL_IO_QSPI_CONFIG(
        BSP_LCD_CS,
        NULL,  // callback
        NULL   // callback context
    );

    // Vendor-specific configuration for AXS15231B
    axs15231b_vendor_config_t vendor_config = {
        .init_cmds = NULL,
        .init_cmds_size = 0,
        .flags = {
            .use_qspi_interface = 1,
        },
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BSP_LCD_SPI_NUM, &io_config, &io_handle));

    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = BSP_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = BSP_LCD_BITS_PER_PIXEL,
        .vendor_config = &vendor_config,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_axs15231b(io_handle, &panel_config, &panel_handle));

    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    esp_lcd_panel_disp_on_off(panel_handle, true);

    if (ret_panel)
    {
        *ret_panel = panel_handle;
    }
    if (ret_io)
    {
        *ret_io = io_handle;
    }
    return ret;
}

/**************************************************************************************************
 *
 * Touch Functions (AXS15231B via I2C)
 *
 **************************************************************************************************/
esp_err_t bsp_touch_new(const bsp_touch_config_t *config, esp_lcd_touch_handle_t *ret_touch)
{
    /* Initialize I2C */
    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_init());

    /* Initialize touch via I2C */
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = BSP_LCD_H_RES,
        .y_max = BSP_LCD_V_RES,
        .rst_gpio_num = GPIO_NUM_NC,
        .int_gpio_num = BSP_LCD_TOUCH_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };

    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_AXS15231B_CONFIG();
    tp_io_config.scl_speed_hz = CONFIG_BSP_I2C_CLK_SPEED_HZ;

    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(i2c_handle, &tp_io_config, &tp_io_handle), TAG, "");
    return esp_lcd_touch_new_i2c_axs15231b(tp_io_handle, &tp_cfg, ret_touch);
}

/**************************************************************************************************
 *
 * LVGL Display Functions
 *
 **************************************************************************************************/
static lv_display_t *bsp_display_lcd_init(const bsp_display_cfg_t *cfg)
{
    bsp_display_config_t disp_config = {0};

    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_new(&disp_config, &panel_handle, &io_handle));

    int buffer_size = BSP_LCD_H_RES * LVGL_BUFFER_HEIGHT;
    if (cfg && cfg->buffer_size > 0) {
        buffer_size = cfg->buffer_size;
    }

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = buffer_size,
        .monochrome = false,
        .hres = BSP_LCD_H_RES,
        .vres = BSP_LCD_V_RES,
#if LVGL_VERSION_MAJOR >= 9
        .color_format = LV_COLOR_FORMAT_RGB565,
#endif
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
            .sw_rotate = true,
            .buff_dma = cfg ? cfg->flags.buff_dma : false,
#if CONFIG_BSP_DISPLAY_LVGL_PSRAM
            .buff_spiram = cfg ? cfg->flags.buff_spiram : true,
#else
            .buff_spiram = cfg ? cfg->flags.buff_spiram : false,
#endif
#if LVGL_VERSION_MAJOR >= 9
            .swap_bytes = true,
#endif
        }
    };

    return lvgl_port_add_disp(&disp_cfg);
}

static lv_indev_t *bsp_display_indev_init(lv_display_t *disp)
{
    BSP_ERROR_CHECK_RETURN_NULL(bsp_touch_new(NULL, &tp));
    assert(tp);

    /* Add touch input (for selected screen) */
    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp = disp,
        .handle = tp,
    };

    return lvgl_port_add_touch(&touch_cfg);
}

lv_display_t *bsp_display_start(void)
{
    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = BSP_LCD_DRAW_BUFF_SIZE,
        .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
        .flags = {
            .buff_dma = false,
            .buff_spiram = true,
        }
    };
    return bsp_display_start_with_config(&cfg);
}

lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg)
{
    assert(cfg != NULL);
    BSP_ERROR_CHECK_RETURN_NULL(lvgl_port_init(&cfg->lvgl_port_cfg));

    BSP_NULL_CHECK(disp = bsp_display_lcd_init(cfg), NULL);

    BSP_NULL_CHECK(disp_indev = bsp_display_indev_init(disp), NULL);

    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_brightness_init());

    return disp;
}

lv_indev_t *bsp_display_get_input_dev(void)
{
    return disp_indev;
}

void bsp_display_rotate(lv_display_t *disp, lv_disp_rotation_t rotation)
{
    lv_disp_set_rotation(disp, rotation);
}

bool bsp_display_lock(uint32_t timeout_ms)
{
    return lvgl_port_lock(timeout_ms);
}

void bsp_display_unlock(void)
{
    lvgl_port_unlock();
}
