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

#include "definitions.h"

int DEFAULT_STEP_DELAY_EDM   = 150;
int DEFAULT_STEP_DELAY_RAPID = 80;

int wire_spindle_speed       = 1920; // should convert to 9rpm at 64 microsteps
float max_feeds[N_AXIS]      = { 200.0, 200.0, 100.0 };
uint16_t max_feeds_micros[N_AXIS] = { 0,0,0 }; // used to store the calculated microseconds for the max feed speeds

float z_axis_max_feedrate   = 20.0;
float pwm_frequency         = 20000.0; // 5khz - 10khz are common for rough cuts
float pwm_duty              = 30.0;    // the deeper the hole gets the higher the duty should be. I used 80% for a deep hole.
int operation_mode          = 1;       // 1 = drill/sinker, 2 = reamer (just a constant up/down movement without progressing down)
float z_stop_after_mm           = 10.0;  // the maximum travel into the workpiece; Note: EDM has electrode wear of x percent and setting this to 1mm doesn't produce a 1mm deep hole
bool  z_no_home                 = true;  // set to true if no homing is needed and edm process starts without homing
float source_voltage_min        = 20.0;  // the minimum input voltage, EDM button won't do anything until this voltage is delivered from the PSU
//float reference_voltage  = 0.0; // we store the feedback voltage here to lock it in when the EDM starts
DMA_ATTR bool lock_reference_voltage = false;
float vsense_drop_range_min          = 5.0;  // target range bottom. We try to be between min and max and hodl there
float vsense_drop_range_max          = 20.0; // target range top. A default range from min+10 it could be min+40. 
                                             // That produces lesser motion but a little motion is not a bad thing
                                             // for a more steady burn use min+20/30/40
float vSense_drop_range_noload = 10.0;  // no load if drop is below this percent; Pin reading is noisy and can deliver huge failures. 
                                        // this can be a real problem while probing. It get's worse with lower voltages
                                        // example: At 25v the vSense Feedback is about 1v. 0.1v are 10%. And the spikes can be worse than that.
                                        // with the 0.1uf capacitor this should be no problem anymore
                                        // if probing fails the value can be changed in the software
float VSENSE_DROP_RANGE_SHORT = 65.0;   // voltage drops above this value are concidered a full short circuit
float pwm_duty_probing        = 10.0;
int   pwm_frequency_probing   = 70000;

float electrode_up_offset_mm  = 2.0;
float electrode_down_probe_mm = 0.5; 

bool protocol_ready = false;
DMA_ATTR bool is_between_steps                   = false;
DMA_ATTR bool force_redraw                       = false;
DMA_ATTR bool edm_process_done                   = false;       // if set to true in the edm process the task gets stopped from within the mainloop and also disables the spark PWM
DMA_ATTR bool edm_process_is_running             = false; // this is true if the machine actually is doing work
DMA_ATTR int stop_edm_task                       = 0;
DMA_ATTR int start_edm_task                      = 0;
DMA_ATTR volatile unsigned long _millis          = 0.0; // storing the current millis
DMA_ATTR volatile unsigned long interface_timer  = 0.0; // this is used for the interval to update the interface
DMA_ATTR volatile bool limit_touched             = false;
DMA_ATTR volatile bool probe_touched             = false;
DMA_ATTR volatile float position_target[6]       = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
DMA_ATTR volatile bool has_last_position         = false;
DMA_ATTR volatile int motion_plan                = 0;
DMA_ATTR volatile bool cutting_depth_reached     = false;
DMA_ATTR volatile bool take_over_z               = false;
DMA_ATTR volatile bool enable_correction         = false;

DMA_ATTR volatile unsigned long flush_retract_timer = 0.0;

int   finish_after_mm            = 2; // f the tool moves 2mm without load it considers the process as done
int   flush_offset_steps         = 0; // after moving the axis up for flushing it goes back down to the start position minus the offset steps
float flush_retract_mm           = 5.0;
float flush_retract_after        = 0.0; // retract tool after x ms to allow flushing; set to 0.0 to disable;
bool  disable_spark_for_flushing = true;
bool  enable_spindle             = true;
float tool_diameter              = 0.0; // just a default tool diamter set to 0.0 if gcode takes care of it
bool simulate_gcode              = false; // runs the gcode file without moving z


DMA_ATTR volatile float reamer_travel_mm = 10.0;  // the travel distance for the reamer up/down oscillation. 
                                // NOTE: In reamer moder the axis is homed and then moves down the reamer_travel_mm!!
                                // This is to not overschoot the home switch. It is necessary to make sure the axis can move the 
                                // given mm down without crashing.
DMA_ATTR volatile float reamer_duration  = 0.0;   // the duration of the reasming process in seconds . 0.0 means it runs until manually stopped

char get_axis_name(uint8_t axis) {
    switch (axis) {
        case X_AXIS:
            return 'X';
        case Y_AXIS:
            return 'Y';
        case Z_AXIS:
            return 'Z';
        case A_AXIS:
            return 'A';
        case B_AXIS:
            return 'B';
        case C_AXIS:
            return 'C';
        default:
            return '?';
    }
}
