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
#include <WString.h>
#include <InputBuffer.h>  
#include "Config.h"
#include <Serial.h>
#include <Error.h>
#include <Probe.h>
#include <System.h>
#include <Motors/Motors.h>


namespace api{

    /** push a cmd to the input buffer **/
    extern void push_cmd(const char *text, bool block_until_idle = false);
    /** execute a realtime command **/
    //extern void execute_realtime_command(Cmd cmd);    
    /** homing routine **/
    extern float* get_wpos( void );
    extern void home_machine( void );
    extern void set_reprobe_points( void );
    extern void set_reprobe_point_x( float x );
    extern void set_reprobe_point_y( float y );
    extern void home_x( void );
    extern void home_y( void );
    extern void home_z( void );
    extern void force_home_z( void );
    extern void block_while_homing( int axis_index );
    /** activate the steppers and unlock the machine **/
    extern void unlock_machine( void );
    /** run a gcode file from SD **/
    extern void run_sd_card_file( String file_path );
    /** get the current poisiton im steps **/
    extern int32_t get_step_position( int axis );
    /** cancel running job **/
    extern void cancel_running_job( void );
    extern bool machine_is_idle( void );
    extern void block_while_not_idle( void );
    extern void probe_xy_center( void );
    extern void probe_x( float mm );
    extern void probe_y( float mm );
    extern void probe_z( float mm );
    extern void probe_block( void );
    extern void show_probe_points( void );
    extern void reset_probe_points( void );
    extern void set_current_position_as_probe_position( void );
    extern bool machine_is_fully_probed( void );
    extern void execute_retraction( void );
    extern void execute_wait( void );
    extern void execute_resume( void );
    extern void lower_feedrate_realtime( void );
    extern void higher_feedrate_realtime( void );
    extern bool machine_is_fully_homed( void );

    extern void update_step_position( int axis, int steps );
    extern void jog_axis(    float mm, const char *axis_name, int speed );
    extern void jog_up(      float mm );
    extern void jog_down(    float mm );
    extern void jog_back(    float mm );
    extern void jog_forward( float mm );
    extern void jog_left(    float mm );
    extern void jog_right(   float mm );
    extern void step_up( void );
    extern void step_down( void );
    extern void lock_while_z_moves( void );

};

