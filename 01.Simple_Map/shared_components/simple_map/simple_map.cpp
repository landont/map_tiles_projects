#include "simple_map.hpp"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "bsp/esp-bsp.h"

// Static member definitions
lv_obj_t* SimpleMap::map_container = nullptr;
lv_obj_t* SimpleMap::map_group = nullptr;
lv_obj_t** SimpleMap::tile_widgets = nullptr;
int SimpleMap::grid_cols = 0;
int SimpleMap::grid_rows = 0;
int SimpleMap::tile_count = 0;
lv_obj_t* SimpleMap::zoom_in_btn = nullptr;
lv_obj_t* SimpleMap::zoom_out_btn = nullptr;
lv_obj_t* SimpleMap::battery_container = nullptr;
lv_obj_t* SimpleMap::battery_icon = nullptr;
lv_obj_t* SimpleMap::battery_label = nullptr;
lv_obj_t* SimpleMap::gps_container = nullptr;
lv_obj_t* SimpleMap::gps_icon = nullptr;
lv_obj_t* SimpleMap::gps_label = nullptr;
lv_obj_t* SimpleMap::gps_marker = nullptr;
lv_obj_t* SimpleMap::loading_popup = nullptr;
double SimpleMap::current_lat = 0.0;
double SimpleMap::current_lon = 0.0;
double SimpleMap::gps_lat = 0.0;
double SimpleMap::gps_lon = 0.0;
bool SimpleMap::gps_has_fix = false;
int SimpleMap::current_zoom = 15;
bool SimpleMap::initialized = false;
bool SimpleMap::is_loading = false;
bool SimpleMap::user_scrolled = false;
uint32_t SimpleMap::last_scroll_time = 0;
uint32_t SimpleMap::last_user_scroll_time = 0;
uint32_t SimpleMap::last_gps_update_time = 0;
uint32_t SimpleMap::last_touch_time = 0;
map_tiles_handle_t SimpleMap::map_handle = nullptr;
bool SimpleMap::is_round_display = false;

bool SimpleMap::init(lv_obj_t* parent_screen) {
    if (initialized) return true;

    // Detect display type (round if width == height)
    lv_display_t *disp = lv_display_get_default();
    lv_coord_t disp_width = lv_display_get_horizontal_resolution(disp);
    lv_coord_t disp_height = lv_display_get_vertical_resolution(disp);
    is_round_display = (disp_width == disp_height);
    printf("SimpleMap: Display %dx%d, round=%s\n", disp_width, disp_height, is_round_display ? "yes" : "no");

    // Initialize the map tiles component
    map_tiles_config_t config = {
        .base_path = "/sdcard",
        .tile_folders = {"tiles1"},
        .tile_type_count = 1,
        .grid_cols = 5,  // 5 columns
        .grid_rows = 4,  // 4 rows (5x4 grid = 20 tiles)
        .default_zoom = 18,
        .use_spiram = true,
        .default_tile_type = 0  // Start with tiles1
    };

    map_handle = map_tiles_init(&config);
    if (!map_handle) {
        printf("SimpleMap: Failed to initialize map tiles component\n");
        return false;
    }

    // Cache grid dimensions for performance
    map_tiles_get_grid_size(map_handle, &grid_cols, &grid_rows);
    tile_count = map_tiles_get_tile_count(map_handle);

    printf("SimpleMap: Initialized with grid size %dx%d (%d tiles)\n", grid_cols, grid_rows, tile_count);

    // Create scrollable map container (like map_scroll in map_display.cpp)
    map_container = lv_obj_create(parent_screen);
    lv_obj_set_size(map_container, disp_width, disp_height);  // Match device resolution
    lv_obj_center(map_container);
    lv_obj_clear_flag(map_container, LV_OBJ_FLAG_SCROLL_MOMENTUM);
    lv_obj_set_scroll_dir(map_container, LV_DIR_ALL);
    lv_obj_add_flag(map_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_border_width(map_container, 0, 0);
    lv_obj_set_style_radius(map_container, 0, 0);
    lv_obj_add_event_cb(map_container, map_scroll_event_cb, LV_EVENT_SCROLL_END, NULL);
    lv_obj_add_event_cb(map_container, map_scroll_event_cb, LV_EVENT_SCROLL, NULL);
    lv_obj_set_style_bg_color(map_container, lv_color_black(), 0);
    lv_obj_set_scrollbar_mode(map_container, LV_SCROLLBAR_MODE_OFF);

    // Create tile widgets
    create_tile_widgets();

    // Create zoom buttons
    create_zoom_buttons(parent_screen);

    // Update zoom button visibility based on initial zoom level
    update_zoom_buttons_visibility();

    // Create battery indicator
    create_battery_indicator(parent_screen);

    // Create GPS indicator (left of battery)
    create_gps_indicator(parent_screen);

    // Set initial battery value (placeholder - integrate AXP2101 for real values)
    // Call update_battery_indicator() from your app with real values from AXP2101 PMIC
    update_battery_indicator(-1, false);  // -1 = unknown/no battery connected
    printf("SimpleMap: Battery indicator initialized (call update_battery_indicator() with real values)\n");

    // Register touch event callback on parent screen to track activity for backlight dimming
    lv_obj_add_event_cb(parent_screen, touch_event_cb, LV_EVENT_PRESSED, NULL);
    last_touch_time = esp_timer_get_time() / 1000;  // Initialize to current time

    initialized = true;
    return true;
}

void SimpleMap::create_tile_widgets() {
    // Use cached grid dimensions for better performance
    if (grid_cols <= 0 || grid_rows <= 0 || tile_count <= 0) {
        printf("SimpleMap: Invalid grid dimensions cached\n");
        return;
    }

    // Allocate tile_widgets array if not already done
    if (!tile_widgets) {
        tile_widgets = (lv_obj_t**)calloc(tile_count, sizeof(lv_obj_t*));
        if (!tile_widgets) {
            printf("Failed to allocate tile_widgets array\n");
            return;
        }
    }

    // Create map group with full tile dimensions (like map_display.cpp)
    if (!map_group) {
        map_group = lv_obj_create(map_container);
        lv_obj_set_size(map_group, MAP_TILES_TILE_SIZE * grid_cols, MAP_TILES_TILE_SIZE * grid_rows);
        lv_obj_set_style_pad_all(map_group, 0, 0);
        lv_obj_set_style_border_width(map_group, 0, 0);
        lv_obj_clear_flag(map_group, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_pos(map_group, 0, 0);

        // Create grid of image widgets for tiles
        for (int i = 0; i < tile_count; i++) {
            tile_widgets[i] = lv_image_create(map_group);

            int row = i / grid_cols;
            int col = i % grid_cols;

            // Position using full TILE_SIZE (like map_display.cpp)
            lv_obj_set_pos(tile_widgets[i], col * MAP_TILES_TILE_SIZE, row * MAP_TILES_TILE_SIZE);
            lv_obj_set_size(tile_widgets[i], MAP_TILES_TILE_SIZE, MAP_TILES_TILE_SIZE);

            // Set default background while tiles load (black for missing tiles)
            lv_obj_set_style_bg_color(tile_widgets[i], lv_color_black(), 0);
            lv_obj_set_style_bg_opa(tile_widgets[i], LV_OPA_COVER, 0);
        }
    }
}

void SimpleMap::create_zoom_buttons(lv_obj_t* parent_screen) {
    // Create zoom in button (+)
    zoom_in_btn = lv_btn_create(parent_screen);
    lv_obj_set_size(zoom_in_btn, 50, 50);
    lv_obj_set_style_radius(zoom_in_btn, 25, 0);  // Circular button
    lv_obj_set_style_bg_color(zoom_in_btn, lv_color_make(50, 50, 50), 0);
    lv_obj_set_style_bg_opa(zoom_in_btn, LV_OPA_80, 0);
    lv_obj_set_style_border_color(zoom_in_btn, lv_color_white(), 0);
    lv_obj_set_style_border_width(zoom_in_btn, 2, 0);
    lv_obj_add_event_cb(zoom_in_btn, zoom_in_event_cb, LV_EVENT_CLICKED, NULL);

    // Position based on display type
    if (is_round_display) {
        // Round display: zoom in on left side, centered vertically
        lv_obj_align(zoom_in_btn, LV_ALIGN_LEFT_MID, 15, 0);
    } else {
        // Rectangular display: zoom in at bottom left
        lv_obj_align(zoom_in_btn, LV_ALIGN_BOTTOM_LEFT, 15, -15);
    }

    lv_obj_t* plus_label = lv_label_create(zoom_in_btn);
    lv_label_set_text(plus_label, "+");
    lv_obj_set_style_text_font(plus_label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(plus_label, lv_color_white(), 0);
    lv_obj_center(plus_label);

    // Create zoom out button (-)
    zoom_out_btn = lv_btn_create(parent_screen);
    lv_obj_set_size(zoom_out_btn, 50, 50);
    lv_obj_set_style_radius(zoom_out_btn, 25, 0);  // Circular button
    lv_obj_set_style_bg_color(zoom_out_btn, lv_color_make(50, 50, 50), 0);
    lv_obj_set_style_bg_opa(zoom_out_btn, LV_OPA_80, 0);
    lv_obj_set_style_border_color(zoom_out_btn, lv_color_white(), 0);
    lv_obj_set_style_border_width(zoom_out_btn, 2, 0);
    lv_obj_add_event_cb(zoom_out_btn, zoom_out_event_cb, LV_EVENT_CLICKED, NULL);

    // Position based on display type
    if (is_round_display) {
        // Round display: zoom out on right side, centered vertically
        lv_obj_align(zoom_out_btn, LV_ALIGN_RIGHT_MID, -15, 0);
    } else {
        // Rectangular display: zoom out at bottom right
        lv_obj_align(zoom_out_btn, LV_ALIGN_BOTTOM_RIGHT, -15, -15);
    }

    lv_obj_t* minus_label = lv_label_create(zoom_out_btn);
    lv_label_set_text(minus_label, "-");
    lv_obj_set_style_text_font(minus_label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(minus_label, lv_color_white(), 0);
    lv_obj_center(minus_label);
}

void SimpleMap::create_battery_indicator(lv_obj_t* parent_screen) {
    // Create battery container
    battery_container = lv_obj_create(parent_screen);
    lv_obj_set_size(battery_container, 67, 24);
    lv_obj_set_style_bg_color(battery_container, lv_color_make(0, 0, 0), 0);
    lv_obj_set_style_bg_opa(battery_container, LV_OPA_60, 0);
    lv_obj_set_style_border_width(battery_container, 1, 0);
    lv_obj_set_style_border_color(battery_container, lv_color_white(), 0);
    lv_obj_set_style_radius(battery_container, 4, 0);
    lv_obj_set_style_pad_all(battery_container, 2, 0);
    lv_obj_clear_flag(battery_container, LV_OBJ_FLAG_SCROLLABLE);

    // Position based on display type
    if (is_round_display) {
        // Round display: centered at top
        lv_obj_align(battery_container, LV_ALIGN_TOP_MID, 0, 15);
    } else {
        // Rectangular display: upper right
        lv_obj_align(battery_container, LV_ALIGN_TOP_RIGHT, -5, 5);
    }

    // Battery icon is now just a colored bar on the left
    battery_icon = lv_obj_create(battery_container);
    lv_obj_set_size(battery_icon, 18, 16);
    lv_obj_align(battery_icon, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_set_style_bg_color(battery_icon, lv_color_make(0, 200, 0), 0);  // Green
    lv_obj_set_style_bg_opa(battery_icon, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(battery_icon, 0, 0);
    lv_obj_set_style_radius(battery_icon, 2, 0);
    lv_obj_clear_flag(battery_icon, LV_OBJ_FLAG_SCROLLABLE);

    // Create battery percentage label - centered
    battery_label = lv_label_create(battery_container);
    lv_label_set_text(battery_label, "--%");
    lv_obj_set_style_text_color(battery_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(battery_label, &lv_font_montserrat_14, 0);
    lv_obj_align(battery_label, LV_ALIGN_RIGHT_MID, -2, 0);
}

void SimpleMap::create_gps_indicator(lv_obj_t* parent_screen) {
    // Create GPS container (left of battery indicator)
    gps_container = lv_obj_create(parent_screen);
    lv_obj_set_size(gps_container, 50, 24);
    lv_obj_set_style_bg_color(gps_container, lv_color_make(0, 0, 0), 0);
    lv_obj_set_style_bg_opa(gps_container, LV_OPA_60, 0);
    lv_obj_set_style_border_width(gps_container, 1, 0);
    lv_obj_set_style_border_color(gps_container, lv_color_white(), 0);
    lv_obj_set_style_radius(gps_container, 4, 0);
    lv_obj_set_style_pad_all(gps_container, 2, 0);
    lv_obj_clear_flag(gps_container, LV_OBJ_FLAG_SCROLLABLE);

    // Position based on display type (left of battery)
    if (is_round_display) {
        // Round display: left of centered battery
        lv_obj_align(gps_container, LV_ALIGN_TOP_MID, -60, 15);
    } else {
        // Rectangular display: left of battery in upper right
        lv_obj_align(gps_container, LV_ALIGN_TOP_RIGHT, -77, 5);
    }

    // GPS satellite icon (simple colored bar)
    gps_icon = lv_obj_create(gps_container);
    lv_obj_set_size(gps_icon, 14, 16);
    lv_obj_align(gps_icon, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_set_style_bg_color(gps_icon, lv_color_make(128, 128, 128), 0);  // Gray = no fix
    lv_obj_set_style_bg_opa(gps_icon, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(gps_icon, 0, 0);
    lv_obj_set_style_radius(gps_icon, 2, 0);
    lv_obj_clear_flag(gps_icon, LV_OBJ_FLAG_SCROLLABLE);

    // Create GPS satellite count label
    gps_label = lv_label_create(gps_container);
    lv_label_set_text(gps_label, "--");
    lv_obj_set_style_text_color(gps_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(gps_label, &lv_font_montserrat_14, 0);
    lv_obj_align(gps_label, LV_ALIGN_RIGHT_MID, -2, 0);

    // Create GPS marker (crosshair) on the map - initially hidden
    gps_marker = lv_obj_create(map_group);
    lv_obj_set_size(gps_marker, 20, 20);
    lv_obj_set_style_bg_opa(gps_marker, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(gps_marker, 0, 0);
    lv_obj_clear_flag(gps_marker, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(gps_marker, LV_OBJ_FLAG_HIDDEN);

    // Horizontal line of crosshair
    lv_obj_t* h_line = lv_obj_create(gps_marker);
    lv_obj_set_size(h_line, 20, 3);
    lv_obj_center(h_line);
    lv_obj_set_style_bg_color(h_line, lv_color_make(255, 0, 0), 0);  // Red crosshair
    lv_obj_set_style_bg_opa(h_line, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(h_line, 0, 0);
    lv_obj_set_style_radius(h_line, 0, 0);
    lv_obj_clear_flag(h_line, LV_OBJ_FLAG_SCROLLABLE);

    // Vertical line of crosshair
    lv_obj_t* v_line = lv_obj_create(gps_marker);
    lv_obj_set_size(v_line, 3, 20);
    lv_obj_center(v_line);
    lv_obj_set_style_bg_color(v_line, lv_color_make(255, 0, 0), 0);  // Red crosshair
    lv_obj_set_style_bg_opa(v_line, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(v_line, 0, 0);
    lv_obj_set_style_radius(v_line, 0, 0);
    lv_obj_clear_flag(v_line, LV_OBJ_FLAG_SCROLLABLE);

    printf("SimpleMap: GPS indicator initialized\n");
}

void SimpleMap::update_battery_indicator(int percent, bool is_charging) {
    printf("SimpleMap: update_battery_indicator called with percent=%d, is_charging=%d\n", percent, is_charging);

    if (!battery_icon || !battery_label) {
        printf("SimpleMap: ERROR - battery_icon=%p, battery_label=%p (null check failed)\n",
               (void*)battery_icon, (void*)battery_label);
        return;
    }

    // Update label
    if (percent < 0) {
        lv_label_set_text(battery_label, "--%");
        printf("SimpleMap: Battery label set to '--%%' (unknown)\n");
    } else {
        lv_label_set_text_fmt(battery_label, "%d%%", percent);
        printf("SimpleMap: Battery label set to '%d%%'\n", percent);
    }

    // Update status bar color based on level and charging state
    lv_color_t color;
    const char* color_name;

    if (is_charging) {
        color = lv_color_make(0, 150, 255);    // Blue for charging
        color_name = "blue (charging)";
    } else if (percent < 0) {
        color = lv_color_make(128, 128, 128);  // Gray for unknown
        color_name = "gray (unknown)";
    } else if (percent <= 20) {
        color = lv_color_make(255, 0, 0);      // Red for low
        color_name = "red (low)";
    } else if (percent <= 50) {
        color = lv_color_make(255, 165, 0);    // Orange for medium
        color_name = "orange (medium)";
    } else {
        color = lv_color_make(0, 200, 0);      // Green for good
        color_name = "green (good)";
    }

    printf("SimpleMap: Battery color set to %s\n", color_name);
    lv_obj_set_style_bg_color(battery_icon, color, 0);

    // Force UI update - invalidate container and trigger immediate refresh
    lv_obj_invalidate(battery_container);
    lv_refr_now(NULL);  // Force immediate screen refresh
    printf("SimpleMap: Battery indicator updated successfully\n");
}

void SimpleMap::set_gps_position(double latitude, double longitude, bool has_fix) {
    gps_lat = latitude;
    gps_lon = longitude;
    gps_has_fix = has_fix;

    if (has_fix) {
        // Update GPS marker position on the map
        update_gps_marker_position();

        // Check if we should auto-center on GPS
        check_auto_center();
    }

    // Show/hide GPS marker based on fix status
    if (gps_marker) {
        if (has_fix) {
            lv_obj_clear_flag(gps_marker, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(gps_marker, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

void SimpleMap::get_gps_position(double* latitude, double* longitude, bool* has_fix) {
    if (latitude) *latitude = gps_lat;
    if (longitude) *longitude = gps_lon;
    if (has_fix) *has_fix = gps_has_fix;
}

void SimpleMap::update_gps_status(int satellites, int fix_type) {
    if (!gps_icon || !gps_label) return;

    // Update satellite count label
    if (satellites >= 0) {
        lv_label_set_text_fmt(gps_label, "%d", satellites);
    } else {
        lv_label_set_text(gps_label, "--");
    }

    // Update GPS icon color based on fix type
    lv_color_t color;
    if (fix_type >= 3) {
        // 3D fix - green
        color = lv_color_make(0, 200, 0);
    } else if (fix_type >= 2) {
        // 2D fix - yellow
        color = lv_color_make(255, 200, 0);
    } else if (satellites > 0) {
        // Acquiring fix - orange
        color = lv_color_make(255, 128, 0);
    } else {
        // No signal - gray
        color = lv_color_make(128, 128, 128);
    }
    lv_obj_set_style_bg_color(gps_icon, color, 0);

    lv_obj_invalidate(gps_container);
}

void SimpleMap::show_gps_error() {
    if (!gps_icon || !gps_label) return;

    // Show "ERR" text in red
    lv_label_set_text(gps_label, "ERR");
    lv_obj_set_style_bg_color(gps_icon, lv_color_make(255, 0, 0), 0);

    lv_obj_invalidate(gps_container);
}

bool SimpleMap::is_user_scrolled() {
    return user_scrolled;
}

void SimpleMap::update_gps_marker_position() {
    if (!gps_marker || !map_handle || !gps_has_fix) return;

    // Convert GPS coordinates to tile coordinates
    double x, y;
    map_tiles_gps_to_tile_xy(map_handle, gps_lat, gps_lon, &x, &y);

    // Get current tile position (top-left of the grid)
    int tile_x, tile_y;
    map_tiles_get_position(map_handle, &tile_x, &tile_y);

    // Calculate pixel position relative to the map_group
    int marker_x = (int)((x - tile_x) * MAP_TILES_TILE_SIZE) - 10;  // Center the 20px marker
    int marker_y = (int)((y - tile_y) * MAP_TILES_TILE_SIZE) - 10;

    // Position the marker
    lv_obj_set_pos(gps_marker, marker_x, marker_y);
}

void SimpleMap::check_auto_center() {
    // Only auto-center if user has scrolled and we have GPS fix
    if (!user_scrolled) return;
    if (!gps_has_fix) return;

    uint32_t current_time = esp_timer_get_time() / 1000;
    uint32_t idle_time = current_time - last_user_scroll_time;

    if (idle_time >= AUTO_CENTER_TIMEOUT_MS) {
        printf("SimpleMap: Auto-centering on GPS (%.6f, %.6f) after %lu ms of inactivity\n",
               gps_lat, gps_lon, idle_time);

        // Update map center to GPS position
        current_lat = gps_lat;
        current_lon = gps_lon;

        // Recenter the map
        map_tiles_set_center_from_gps(map_handle, gps_lat, gps_lon);
        load_map_tiles();

        // Center the view after tiles load
        lv_timer_create([](lv_timer_t *t) {
            lv_timer_del(t);
            center_map_on_gps();
            update_gps_marker_position();
        }, 200, NULL);

        user_scrolled = false;
    }
}

void SimpleMap::zoom_in_event_cb(lv_event_t *e) {
    reset_activity_timer();  // Reset backlight timer on button press

    if (is_loading) return;

    int new_zoom = current_zoom + 1;
    if (new_zoom > 18) {
        printf("SimpleMap: Already at maximum zoom level (18)\n");
        return;
    }

    printf("SimpleMap: Zoom in button pressed, changing from %d to %d\n", current_zoom, new_zoom);
    change_zoom_level(new_zoom);
}

void SimpleMap::zoom_out_event_cb(lv_event_t *e) {
    reset_activity_timer();  // Reset backlight timer on button press

    if (is_loading) return;

    int new_zoom = current_zoom - 1;
    if (new_zoom < 12) {
        printf("SimpleMap: Already at minimum zoom level (12)\n");
        return;
    }

    printf("SimpleMap: Zoom out button pressed, changing from %d to %d\n", current_zoom, new_zoom);
    change_zoom_level(new_zoom);
}

void SimpleMap::update_zoom_buttons_visibility() {
    if (!zoom_in_btn || !zoom_out_btn) return;

    // Hide zoom in button at max zoom (18)
    if (current_zoom >= 18) {
        lv_obj_add_flag(zoom_in_btn, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_clear_flag(zoom_in_btn, LV_OBJ_FLAG_HIDDEN);
    }

    // Hide zoom out button at min zoom (12)
    if (current_zoom <= 12) {
        lv_obj_add_flag(zoom_out_btn, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_clear_flag(zoom_out_btn, LV_OBJ_FLAG_HIDDEN);
    }
}

void SimpleMap::touch_event_cb(lv_event_t *e) {
    reset_activity_timer();
}

uint32_t SimpleMap::get_last_touch_time() {
    return last_touch_time;
}

void SimpleMap::reset_activity_timer() {
    // Update last touch time
    last_touch_time = esp_timer_get_time() / 1000;

    // Set backlight to 100% on activity
    bsp_display_brightness_set(100);
}

bool SimpleMap::try_auto_center_on_gps() {
    // Only auto-center if user has scrolled away and we have GPS fix
    if (!user_scrolled || !gps_has_fix) {
        return false;
    }

    printf("SimpleMap: Auto-centering on GPS (%.6f, %.6f)\n", gps_lat, gps_lon);

    // Update map center to GPS position
    current_lat = gps_lat;
    current_lon = gps_lon;

    // Recenter the map
    map_tiles_set_center_from_gps(map_handle, gps_lat, gps_lon);
    load_map_tiles();

    // Center the view after tiles load
    lv_timer_create([](lv_timer_t *t) {
        lv_timer_del(t);
        center_map_on_gps();
        update_gps_marker_position();
    }, 200, NULL);

    user_scrolled = false;
    return true;
}

void SimpleMap::show_location(double latitude, double longitude, int zoom_level) {
    if (!initialized || !map_handle) return;

    // Update current position and zoom
    current_lat = latitude;
    current_lon = longitude;
    current_zoom = zoom_level;

    // Set zoom level in the component
    map_tiles_set_zoom(map_handle, zoom_level);

    // Set the tile center based on GPS coordinates
    map_tiles_set_center_from_gps(map_handle, latitude, longitude);

    load_map_tiles();
}

void SimpleMap::update_location(double latitude, double longitude) {
    show_location(latitude, longitude, current_zoom);
}

void SimpleMap::set_satellite_view(bool satellite) {
    if (!initialized || !map_handle) return;

    // Convert boolean to tile type (0 = regular, 1 = satellite)
    int tile_type = satellite ? 1 : 0;

    // Set tile type in the component
    if (map_tiles_set_tile_type(map_handle, tile_type)) {
        // Reload tiles with the new tile type
        load_map_tiles();
    }
}

bool SimpleMap::set_tile_type(int tile_type) {
    if (!initialized || !map_handle) return false;

    // Set tile type in the component
    if (map_tiles_set_tile_type(map_handle, tile_type)) {
        // Reload tiles with the new tile type
        load_map_tiles();
        return true;
    }
    return false;
}

int SimpleMap::get_tile_type() {
    if (!initialized || !map_handle) return -1;
    return map_tiles_get_tile_type(map_handle);
}

int SimpleMap::get_tile_type_count() {
    if (!initialized || !map_handle) return 0;
    return map_tiles_get_tile_type_count(map_handle);
}

const char* SimpleMap::get_tile_type_folder(int tile_type) {
    if (!initialized || !map_handle) return nullptr;
    return map_tiles_get_tile_type_folder(map_handle, tile_type);
}

void SimpleMap::load_map_tiles() {
    if (!initialized || !map_handle) return;

    // Check if already loading to prevent multiple simultaneous loads
    if (is_loading) {
        printf("SimpleMap: Already loading, skipping...\n");
        return;
    }

    is_loading = true;
    printf("SimpleMap: Starting tile loading...\n");

    // Show loading popup
    show_loading_popup();

    // Use a single-shot timer with longer delay to reduce CPU pressure
    lv_timer_create([](lv_timer_t *t) {
        lv_timer_del(t);  // Delete this timer immediately

        uint32_t start_time = esp_timer_get_time() / 1000;

        // Get current tile position
        int tx, ty;
        map_tiles_get_position(map_handle, &tx, &ty);

        printf("SimpleMap: Loading %dx%d grid at position (%d,%d) at zoom %d\n",
               grid_cols, grid_rows, tx, ty, map_tiles_get_zoom(map_handle));

        // Disable automatic invalidation during bulk updates
        lv_obj_add_flag(map_group, LV_OBJ_FLAG_HIDDEN);

        // Load grid of tiles - minimize style changes
        for (int row = 0; row < grid_rows; row++) {
            for (int col = 0; col < grid_cols; col++) {
                int index = row * grid_cols + col;
                int tile_x = tx + col;
                int tile_y = ty + row;

                // Load tile from SD card
                bool loaded = map_tiles_load_tile(map_handle, index, tile_x, tile_y);

                if (loaded) {
                    // Get the tile image data and set it
                    lv_image_dsc_t* tile_img = map_tiles_get_image(map_handle, index);
                    if (tile_img && tile_img->data) {
                        lv_image_set_src(tile_widgets[index], (const void *)tile_img);
                        // Only clear background if we have valid image
                        lv_obj_set_style_bg_opa(tile_widgets[index], LV_OPA_TRANSP, 0);
                    }
                } else {
                    // Missing tile - show solid black
                    lv_image_set_src(tile_widgets[index], NULL);
                    lv_obj_set_style_bg_color(tile_widgets[index], lv_color_black(), 0);
                    lv_obj_set_style_bg_opa(tile_widgets[index], LV_OPA_COVER, 0);
                }
            }
        }

        // Re-enable visibility and trigger single invalidation
        lv_obj_clear_flag(map_group, LV_OBJ_FLAG_HIDDEN);
        lv_obj_invalidate(map_group);  // Single invalidation instead of continuous ones

        uint32_t end_time = esp_timer_get_time() / 1000;
        printf("SimpleMap: Tile loading completed in %lu ms\n", end_time - start_time);

        // Hide loading popup
        hide_loading_popup();

        // Update GPS marker position after tiles are loaded
        update_gps_marker_position();

        // Clear loading flag
        is_loading = false;

    }, 50, NULL);  // Increased delay to reduce CPU pressure
}

void SimpleMap::map_scroll_event_cb(lv_event_t *e)
{
    if (!initialized || !map_handle) return;

    reset_activity_timer();  // Reset backlight timer on scroll

    lv_event_code_t event_code = lv_event_get_code(e);

    // Handle real-time GPS coordinate updates during scrolling
    if (event_code == LV_EVENT_SCROLL) {
        uint32_t current_time = esp_timer_get_time() / 1000;
        if (current_time - last_gps_update_time >= 50) {
            update_current_gps_from_map_center();
            last_gps_update_time = current_time;
        }

        // Mark that user has manually scrolled (for auto-center feature)
        if (!user_scrolled) {
            user_scrolled = true;
            printf("SimpleMap: User scroll detected\n");
        }
        last_user_scroll_time = current_time;

        return;
    }

    // Handle tile loading when scroll ends
    if (event_code == LV_EVENT_SCROLL_END) {
        // Don't process scroll events if already loading
        if (is_loading) {
            return;
        }

        // Add debouncing - ignore rapid scroll events
        uint32_t current_time = esp_timer_get_time() / 1000;
        if (current_time - last_scroll_time < 100) { // 100ms debounce
            return;
        }
        last_scroll_time = current_time;
        last_user_scroll_time = current_time;

        bool needUpdate = false;
        int tile_x, tile_y;
        map_tiles_get_position(map_handle, &tile_x, &tile_y);

        if (lv_obj_get_scroll_left(map_container) < -90)
        {
            printf("SimpleMap: at_left\n");
            tile_x -= 1;
            needUpdate = true;
        }
        else if (lv_obj_get_scroll_top(map_container) < -90)
        {
            printf("SimpleMap: at_top\n");
            tile_y -= 1;
            needUpdate = true;
        }
        else if (lv_obj_get_scroll_right(map_container) < -90)
        {
            printf("SimpleMap: at_right\n");
            tile_x += 1;
            needUpdate = true;
        }
        else if (lv_obj_get_scroll_bottom(map_container) < -90)
        {
            printf("SimpleMap: at_bottom\n");
            tile_y += 1;
            needUpdate = true;
        }

        if (needUpdate) {
            map_tiles_set_position(map_handle, tile_x, tile_y);
            load_map_tiles();
        }

        // Update GPS coordinates from current center (whether tiles updated or not)
        update_current_gps_from_map_center();
    }
}

void SimpleMap::show_loading_popup()
{
    if (loading_popup) return;

    // Thin fullscreen overlay (light opacity)
    loading_popup = lv_obj_create(lv_screen_active());
    lv_obj_remove_style_all(loading_popup);
    lv_obj_set_size(loading_popup, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(loading_popup, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(loading_popup, LV_OPA_40, 0); // lighter dim
    lv_obj_set_style_border_width(loading_popup, 0, 0);

    // Small pill card centered
    lv_obj_t* card = lv_obj_create(loading_popup);
    lv_obj_remove_style_all(card);
    lv_obj_set_size(card, 170, 42);
    lv_obj_center(card);

    // Simple, high-contrast look (no shadow/gradient)
    lv_obj_set_style_radius(card, 8, 0);
    lv_obj_set_style_bg_color(card, lv_palette_main(LV_PALETTE_YELLOW), 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_90, 0);
    lv_obj_set_style_border_width(card, 1, 0);
    lv_obj_set_style_border_color(card, lv_palette_darken(LV_PALETTE_GREY, 2), 0);
    lv_obj_set_style_pad_all(card, 8, 0);

    // Single static label
    lv_obj_t* label = lv_label_create(card);
    lv_label_set_text(label, "Loading map...");
    lv_obj_set_style_text_color(label, lv_color_black(), 0);
    lv_obj_center(label);

    lv_obj_move_foreground(loading_popup);
}

void SimpleMap::hide_loading_popup()
{
    if (loading_popup) {
        lv_obj_delete(loading_popup);
        loading_popup = nullptr;
    }
}

void SimpleMap::center_map_on_gps()
{
    if (!initialized || !map_handle || !map_group || !map_container) {
        printf("SimpleMap: Map not properly initialized for centering\n");
        return;
    }

    // Convert current GPS coordinates to tile coordinates
    double x, y;
    map_tiles_gps_to_tile_xy(map_handle, current_lat, current_lon, &x, &y);

    // Get current tile position (top-left of the 5x5 grid)
    int tile_x, tile_y;
    map_tiles_get_position(map_handle, &tile_x, &tile_y);

    // Calculate absolute pixel position of GPS coordinates
    int abs_px = (int)(x * MAP_TILES_TILE_SIZE);
    int abs_py = (int)(y * MAP_TILES_TILE_SIZE);

    // Calculate top-left pixel position of current tile grid
    int top_left_px_x = tile_x * MAP_TILES_TILE_SIZE;
    int top_left_px_y = tile_y * MAP_TILES_TILE_SIZE;

    // Calculate scroll position to center GPS coordinates in the map container
    int center_scroll_x = abs_px - top_left_px_x - lv_obj_get_width(map_container) / 2;
    int center_scroll_y = abs_py - top_left_px_y - lv_obj_get_height(map_container) / 2;

    // Apply the scroll to center the GPS coordinates
    lv_obj_scroll_to(map_container, center_scroll_x, center_scroll_y, LV_ANIM_OFF);

    // Update GPS marker position after centering
    update_gps_marker_position();

    printf("SimpleMap: Centered map on GPS %.6f, %.6f (scroll: %d, %d)\n",
           current_lat, current_lon, center_scroll_x, center_scroll_y);
}

void SimpleMap::update_current_gps_from_map_center()
{
    if (!initialized || !map_handle || !map_container) {
        return;
    }

    // Get current scroll position
    lv_coord_t scroll_x = lv_obj_get_scroll_x(map_container);
    lv_coord_t scroll_y = lv_obj_get_scroll_y(map_container);

    // Get screen center offset
    lv_coord_t screen_center_x = lv_obj_get_width(map_container) / 2;
    lv_coord_t screen_center_y = lv_obj_get_height(map_container) / 2;

    // Calculate the center pixel position relative to the map
    int center_pixel_x = scroll_x + screen_center_x;
    int center_pixel_y = scroll_y + screen_center_y;

    // Get current tile position (top-left of the grid)
    int tile_x, tile_y;
    map_tiles_get_position(map_handle, &tile_x, &tile_y);

    // Calculate top-left pixel position of current tile grid
    int top_left_px_x = tile_x * MAP_TILES_TILE_SIZE;
    int top_left_px_y = tile_y * MAP_TILES_TILE_SIZE;

    // Calculate absolute pixel position
    int abs_pixel_x = top_left_px_x + center_pixel_x;
    int abs_pixel_y = top_left_px_y + center_pixel_y;

    // Convert absolute pixel position to tile coordinates
    double tile_coord_x = (double)abs_pixel_x / MAP_TILES_TILE_SIZE;
    double tile_coord_y = (double)abs_pixel_y / MAP_TILES_TILE_SIZE;

    // Convert tile coordinates to GPS coordinates
    double new_lat, new_lon;
    map_tiles_tile_xy_to_gps(map_handle, tile_coord_x, tile_coord_y, &new_lat, &new_lon);

    // Update current GPS coordinates
    current_lat = new_lat;
    current_lon = new_lon;

    // GPS coordinates updated silently for better performance
}

void SimpleMap::change_zoom_level(int new_zoom) {
    if (!initialized || !map_handle) {
        printf("SimpleMap: Map not initialized, cannot change zoom\n");
        return;
    }

    if (new_zoom == current_zoom) {
        printf("SimpleMap: Zoom level already set to %d\n", new_zoom);
        return;
    }

    printf("SimpleMap: Changing zoom from %d to %d\n", current_zoom, new_zoom);

    // Set zoom level in the component first
    map_tiles_set_zoom(map_handle, new_zoom);

    // Update current zoom after setting it in the component
    current_zoom = new_zoom;

    // If we have GPS fix, center on GPS position; otherwise use current map center
    double center_lat = current_lat;
    double center_lon = current_lon;
    if (gps_has_fix) {
        center_lat = gps_lat;
        center_lon = gps_lon;
        current_lat = gps_lat;
        current_lon = gps_lon;
        user_scrolled = false;  // Reset scroll flag when zooming with GPS fix
        printf("SimpleMap: Zoom centering on GPS position (%.6f, %.6f)\n", gps_lat, gps_lon);
    }

    // Recalculate tile positions for the new zoom level
    map_tiles_set_center_from_gps(map_handle, center_lat, center_lon);

    // Force reload all tiles for the new zoom level
    load_map_tiles();

    // Center the map view after tiles are loaded
    lv_timer_create([](lv_timer_t *t) {
        lv_timer_del(t);
        center_map_on_gps();
        update_gps_marker_position();
        update_zoom_buttons_visibility();
        printf("SimpleMap: Zoom change completed, map centered at zoom level %d\n", current_zoom);
    }, 200, NULL);
}

void SimpleMap::get_current_location(double* latitude, double* longitude) {
    if (latitude) *latitude = current_lat;
    if (longitude) *longitude = current_lon;
}

int SimpleMap::get_current_zoom() {
    return current_zoom;
}

void SimpleMap::cleanup() {
    if (map_container) {
        lv_obj_delete(map_container);
        map_container = nullptr;
    }

    if (zoom_in_btn) {
        lv_obj_delete(zoom_in_btn);
        zoom_in_btn = nullptr;
    }

    if (zoom_out_btn) {
        lv_obj_delete(zoom_out_btn);
        zoom_out_btn = nullptr;
    }

    if (battery_container) {
        lv_obj_delete(battery_container);
        battery_container = nullptr;
    }

    if (gps_container) {
        lv_obj_delete(gps_container);
        gps_container = nullptr;
    }

    if (loading_popup) {
        lv_obj_delete(loading_popup);
        loading_popup = nullptr;
    }

    map_group = nullptr;  // Will be cleaned up when map_container is deleted
    battery_icon = nullptr;
    battery_label = nullptr;
    gps_icon = nullptr;
    gps_label = nullptr;
    gps_marker = nullptr;  // Cleaned up with map_group

    // Clean up the map tiles component
    if (map_handle) {
        map_tiles_cleanup(map_handle);
        map_handle = nullptr;
    }

    // Clean up tile widgets array
    if (tile_widgets) {
        free(tile_widgets);
        tile_widgets = nullptr;
    }

    // Reset cached grid dimensions
    grid_cols = 0;
    grid_rows = 0;
    tile_count = 0;

    // Reset loading flag
    is_loading = false;
    user_scrolled = false;
    last_scroll_time = 0;
    last_user_scroll_time = 0;
    last_gps_update_time = 0;
    gps_has_fix = false;

    initialized = false;
}
