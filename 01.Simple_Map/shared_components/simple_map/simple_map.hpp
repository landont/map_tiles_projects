#pragma once

#include "lvgl.h"
#include "map_tiles.h"

// Track log configuration
#define TRACK_LOG_MAX_POINTS 600      // 10 minutes at 1 point per second
#define TRACK_LOG_MIN_DISTANCE 1.0    // Minimum meters between track points
#define TRACK_LOG_DURATION_MS 600000  // 10 minutes in milliseconds

struct TrackPoint {
    double latitude;
    double longitude;
    uint32_t timestamp;  // Milliseconds since boot
};

class SimpleMap {
public:
    // Initialize the simple map system
    static bool init(lv_obj_t* parent_screen);

    // Display map at specific coordinates
    static void show_location(double latitude, double longitude, int zoom_level = 18);

    // Update location (for moving GPS)
    static void update_location(double latitude, double longitude);

    // Toggle between map and satellite view
    static void set_satellite_view(bool satellite);

    // Set specific tile type (0 = tiles1, 1 = tiles2, etc.)
    static bool set_tile_type(int tile_type);

    // Get current tile type
    static int get_tile_type();

    // Get available tile type count
    static int get_tile_type_count();

    // Get tile type folder name
    static const char* get_tile_type_folder(int tile_type);

    // Center map on current GPS coordinates with pixel precision
    static void center_map_on_gps();

    // Change zoom level and reload map tiles
    static void change_zoom_level(int new_zoom);

    // Get current GPS coordinates (latitude, longitude) at center of screen
    static void get_current_location(double* latitude, double* longitude);

    // Get current zoom level
    static int get_current_zoom();

    // Update battery indicator (0-100 percent, -1 for unknown/no battery)
    static void update_battery_indicator(int percent, bool is_charging = false);

    // Get last touch time (milliseconds since boot)
    static uint32_t get_last_touch_time();

    // Reset touch time and restore backlight (call on any user interaction)
    static void reset_activity_timer();

    // Try to auto-center map on GPS position (call periodically from battery monitor)
    // Returns true if map was re-centered
    static bool try_auto_center_on_gps();

    // GPS position tracking
    // Update GPS position from external GPS module (includes heading for direction arrow)
    static void set_gps_position(double latitude, double longitude, bool has_fix, float heading = -1.0f);

    // Get GPS position
    static void get_gps_position(double* latitude, double* longitude, bool* has_fix);

    // Clear track log history
    static void clear_track_log();

    // Update GPS status indicator (satellites in use, fix type)
    static void update_gps_status(int satellites, int fix_type);

    // Show GPS I2C error state (red indicator with "ERR" text)
    static void show_gps_error();

    // Check if user has manually scrolled the map
    static bool is_user_scrolled();

    // Cleanup
    static void cleanup();

private:
    static lv_obj_t* map_container;
    static lv_obj_t* map_group;
    static lv_obj_t** tile_widgets;
    static int grid_cols, grid_rows, tile_count;  // Cache grid dimensions
    static lv_obj_t* zoom_in_btn;
    static lv_obj_t* zoom_out_btn;
    static lv_obj_t* battery_container;
    static lv_obj_t* battery_icon;
    static lv_obj_t* battery_label;
    static lv_obj_t* gps_container;      // GPS status indicator container
    static lv_obj_t* gps_icon;           // GPS satellite icon
    static lv_obj_t* gps_label;          // GPS satellite count label
    static lv_obj_t* gps_marker;         // GPS position direction arrow marker
    static lv_obj_t* loading_popup;

    // Track log storage
    static TrackPoint track_log[TRACK_LOG_MAX_POINTS];
    static int track_log_count;          // Number of points in track log
    static int track_log_start;          // Index of oldest point (circular buffer)
    static lv_obj_t** track_dots;        // Array of yellow dot objects for track visualization
    static int track_dots_count;         // Number of allocated track dot objects
    static double current_lat;
    static double current_lon;
    static double gps_lat;               // Actual GPS latitude
    static double gps_lon;               // Actual GPS longitude
    static bool gps_has_fix;             // GPS has valid fix
    static float gps_heading;            // GPS heading/course in degrees (0-360, -1 = unknown)
    static int current_zoom;
    static bool initialized;
    static bool is_loading;  // Flag to prevent multiple simultaneous loads
    static bool user_scrolled;  // Flag indicating user has manually scrolled
    static bool is_programmatic_scroll;  // Flag to ignore scroll events during GPS centering
    static uint32_t last_scroll_time;  // Timestamp of last scroll event
    static uint32_t last_user_scroll_time;  // Timestamp of last user scroll
    static uint32_t last_gps_update_time;  // Timestamp of last GPS update
    static uint32_t last_touch_time;  // Timestamp of last touch event
    static map_tiles_handle_t map_handle;
    static bool is_round_display;  // True for round/square displays (width == height)

    // Auto-scroll back to GPS timeout (10 seconds)
    static const uint32_t AUTO_CENTER_TIMEOUT_MS = 10000;

    // Helper functions
    static void load_map_tiles();
    static void create_tile_widgets();
    static void map_scroll_event_cb(lv_event_t *e);
    static void create_zoom_buttons(lv_obj_t* parent_screen);
    static void create_battery_indicator(lv_obj_t* parent_screen);
    static void create_gps_indicator(lv_obj_t* parent_screen);
    static void update_gps_marker_position();
    static void update_gps_marker_rotation();
    static void add_track_point(double lat, double lon);
    static void prune_old_track_points();
    static void update_track_dots_positions();
    static void create_direction_arrow();
    static double distance_between(double lat1, double lon1, double lat2, double lon2);
    static void zoom_in_event_cb(lv_event_t *e);
    static void zoom_out_event_cb(lv_event_t *e);
    static void show_loading_popup();
    static void hide_loading_popup();
    static void update_current_gps_from_map_center();
    static void update_zoom_buttons_visibility();
    static void touch_event_cb(lv_event_t *e);
    static void check_auto_center();
};
