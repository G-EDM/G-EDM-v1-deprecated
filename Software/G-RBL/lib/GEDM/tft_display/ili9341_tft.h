#pragma once
/*
#  ██████        ███████ ██████  ███    ███  
# ██             ██      ██   ██ ████  ████  
# ██   ███ █████ █████   ██   ██ ██ ████ ██ 
# ██    ██       ██      ██   ██ ██  ██  ██ 
#  ██████        ███████ ██████  ██      ██ 
#
# This is a beta version for testing purposes.
# Only for personal use. Commercial use or redistribution without permission is prohibited. 
# Copyright (c) Roland Lautensack        
*/ 
#include "ili9341_pinout.h"
#include "ili9341_config.h"
#include <SPIFFS.h>
#include "FS.h"
#include <SPI.h>
#include <TFT_eSPI.h>
#include "gedm_sensors/sensors.h"
#include "pwm_controller/pwm_controller.h"
#include "gedm_api/api.h"
#include "gedm_api/probing_routines.h"
#include "sd_card/filehandler.h"
#include "gedm_spindle/gedm_spindle.h"
#include "gedm_planner/Planner.h"

extern TFT_eSPI tft;

extern TaskHandle_t ui_task_handle;

void IRAM_ATTR ui_task( void * parameter );

class G_EDM_UI_CONTROLLER : public G_EDM_SENSORS, public G_EDM_PWM_CONTROLLER, public G_EDM_SPINDLE, public G_EDM_PROBE_ROUTINES {

private:
    int probe_dimension;
    int probe_backup_frequency;
    void draw_toggle_probe_dimension( void );
    bool disable_tft_calibration;
    bool probe_prepare( int disable_index, bool is_3d = false );
    void probe_done( void );
    int spark_indicator_bar_width; // cache the width of the indicator bar to reduce redraws if nothing has changed
    int button_width;
    int button_height;
    int button_margin;
    String keyboard_value;
    float keyboard_initial_value;
    String keyboard_initial_value_alpha;
    int number_of_pages;
    int active_page;
    float z_pos;
    //float z_stop;
    int ms_edm;
    int ms_travel;
    int short_iterations;
    int total_iterations;
    unsigned long total_time;
    unsigned long travel_time;
    float total_workload_travel;
    int debug_bg_color;
    bool has_first_stroke;
    float drop_min;
    float drop_max;
    float drop_short;
    float voltage_min;
    //float short_circuit_mm_up;
    //float reamer_travel_mm;
    //float reamer_duration;
    //int operation_mode;
    //float flush_retract_after;
    //float flush_retract_mm;
    //bool disable_spark_for_flushing;
    //int flush_offset_steps;
    bool has_sd_card;
    String selected_list_item;
    String gcode_file;
    String active_profile;
    unsigned long start_time;
    bool motion_input_is_blocked(void);
    int spark_on_off_indicator_icon_last_state;

    int motion_tab_active;
    String last_settings_copy; // to prevent unnecessary SD writings we compare the settings. If we already stored them we skip the writing.

public:
    G_EDM_UI_CONTROLLER();
    TFT_eSPI tft;
    void sinker_drill_single_axis( void );
    void sd_card_task( void );
    void wire_gcode_task( void );
    bool open_gcode_file( void );
    bool is_ready;
    G_FILEHANDLER *filehandler;
    void init( void );
    String get_mpos_string( void );
    String convert_frequency_units( int frequency );
    unsigned long last_settings_write;
    void pre_process_defaults( void );
    void set_reprobe_point( void );
    void reset_defaults( void );
    void draw_set_reprobe_button( void );
    void process_overlay_wire_spindle_speed_buttons( void );
    void change_wire_spindle_speed( void );
    void update_display_minimal( void );
    void draw_homing_buttons( bool enabled );
    void start_ui_task( void );
    void reset_flush_retract_timer( void );
    bool check_if_time_to_flush( void );
    void render_page( int page_num, bool redraw_full );
    void draw_page( int page_num, bool redraw_full );
    void init(bool disable_calibration );
    void set_filehandler(  G_FILEHANDLER *ptrfilehandler );
    bool get_is_ready( void );
    void set_is_ready( void );
    void touch_calibrate( void );
    void loader_screen(void);
    bool keyboard_is_active;
    void draw_interface( void );
    void draw_spark_on_off_indicator_icon( void );
    void redraw_vsense( void );
    void draw_mpos( void );
    void draw_period_wave( void );
    void draw_motion_navigation_buttons( void );
    void draw_mode_buttons( void );
    float get_keyboard_result( void );
    String get_keyboard_result_alpha( void );
    void close_keyboard( void );
    void open_keyboard( float value, String text, String infotext );
    void open_keyboard_alpha( String value, String text, String infotext );
    void map_keys( int x, int y );
    void draw_keyboard_result( void );
    String convert_timings( float seconds );
    void add_page( void );
    void next_page( void );
    void previous_page( void );
    int get_active_page( void );
    void set_active_page( int value );
    void set_debug_bg_color( int num );
    void set_z_pos( float value );
    void set_z_stop( float value );
    void set_ms_edm( int value );
    void set_ms_travel( int value );
    void set_total_iterations( int total_iterations );
    void set_short_iterations( int short_iterations );
    void set_total_time( unsigned long total_time );
    void set_travel_time( unsigned long travel_time );
    void set_workload_travel( float total_workload_travel );
    void set_drop_range_noload( float value );
    void set_drop_min_max( float min, float max );
    void set_drop_short( float value );
    void set_voltage_min( float value );
    void set_z_no_home( bool value );
    void set_short_circuit_mm_up( float _mm );
    void set_reamer_travel_mm( float _mm );
    void set_reamer_duration( float _seconds );
    void set_operation_mode( int mode );
    void set_flush_retract_after( float _milliseconds );
    void set_flush_retract_mm( float _mm );
    void set_disable_spark_for_flushing( bool disable );
    void set_flush_offset_steps( int _steps );
    void set_active_profile( String profile );
    void render_page_settings_menu( void );
    void render_page_front( void );
    void update_process_meta( void );
    void render_page_process_overlay( void );
    void process_overlay_reprobe_confirm( void );
    void process_overlay_reprobe_button( void );
    void process_overlay_menu( void );
    void process_overlay_coords( void );
    void process_overlay_pause_button( void );
    void render_page_reprobe_confirmation( void );
    void render_page_settings_pwm( void );
    void render_page_settings_flushing( void );
    void render_page_settings_spark( void );
    void render_page_settings_mode( void );
    void render_page_settings_motion( void );
    void render_page_settings_sd( String active_profile );
    void render_page_settings_menu_sd_card( bool sd_card_has, bool force_redraw );
    void render_page_settings_sd_gcode( void );
    void render_page_settings_sd_profiles( void );
    void render_page_settings_sd_add_entry( String entry, int row );
    void draw_navigation_buttons( int current_page, int total_pages );
    void render_page_settings_sd_redraw_canvas( void );
    bool get_touch( uint16_t *x, uint16_t *y );
    void select_list_begin( int num_files, char files[][100] );
    String get_selected_item(  void );
    void unselected_item(  void );
    void set_gcode_file( String filename );
    String get_gcode_file( void );
    void stop_spi(void);
    void start_spi(void);
    void monitor_touch_input(void);
    void update_setting( String setting, String value );
    String* split(String& v, char delimiter, int& length);
    void load_settings( String file_name );
    void save_settings( String file_name );
    bool ready_to_run( void );
    void start_process(void);
    void reset_after_job(void);
    void set_motion_tab_active( int active_tab );
    void draw_probing_buttons( int active = 0 );
    void draw_motion_tab( void );
    bool machine_is_homed( void );
    void edit_pwm_frequency( void );
    void edit_pwm_duty( void );
    void edit_setpoint_min( void );
    void edit_setpoint_max( void );
    void edit_max_feed( int axis );
    void edit_max_feed_xy( void );
    void edit_rapid_delay( void );
    bool gcode_job_running( void );
    float get_z_stop( void );
    int get_ms_edm( void );
    int get_ms_travel( void );
    void draw_sdcard_button( int _state );
    void sd_card_handler( void );
    void alarm_handler( void );

};


extern G_EDM_UI_CONTROLLER ui_controller;