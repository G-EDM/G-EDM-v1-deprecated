#pragma once
/*
#  ██████        ███████ ██████  ███    ███  
# ██             ██      ██   ██ ████  ████  
# ██   ███ █████ █████   ██   ██ ██ ████ ██ 
# ██    ██       ██      ██   ██ ██  ██  ██ 
#  ██████        ███████ ██████  ██      ██ 
#
# This is a beta version for testing purposes.
# Only for personal and private use. Commercial use or redistribution without permission is prohibited. 
# Copyright (c) Roland Lautensack        
*/ 
#include "config/definitions.h"
#include <driver/adc.h>
#include <esp_attr.h>
#include <esp32-hal.h>

extern TaskHandle_t vsense_task;
extern TaskHandle_t start_stop_task;
//extern TaskHandle_t limit_switch_task;


void IRAM_ATTR update_vsense_task( void * parameter );
void IRAM_ATTR update_start_stop_task( void * parameter );
//void IRAM_ATTR update_limit_switch_task( void * parameter );


class G_EDM_SENSORS {
private:
    bool has_init;
    int      multisample_counts[3] = {0,0,0};
    uint32_t multisample_buffer[3][FILTER_LEN];
    int pin_high_at;
    uint64_t last_vSense_read_internal;
    int last_motion_plan;
    int counter[4];

public:
    G_EDM_SENSORS();

    float confirm_state( int state );    
    float source_voltage;
    float reference_voltage;
    float feedback_voltage;
    int vSense_analog;
    int probe_positive_count;
    float current_voltage_drop;
    bool block_core_0_access;

    uint32_t read_multisample(int raw, int map_id);
    void run_sensor_service( void );
    float calulate_source_voltage( void );
    void update_feedback_data( void );
    float IRAM_ATTR get_drop_in_percent( bool _update );
    void IRAM_ATTR update_vsense( int num_samples = 0 );
    float get_feedback_voltage( void );
    float get_source_voltage( void );
    int get_vsense_raw( void );
    void set_analog( int analog_raw );
    void set_analog_timestamp( void );
    bool edm_start_stop( void );
    bool limit_is_touched( void );
    bool continue_edm_process( void );
    void generate_reference_voltage( void );
    bool start_edm_process( void );
    void collect_probe_samples( void );
    bool is_probing( void );
    int get_motion_plan( void );
    int get_raw_plan( void );
    bool get_motion_plan_samples( int plan, int samples );

};
