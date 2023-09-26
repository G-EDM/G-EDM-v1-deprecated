#pragma once

#include <stdint.h>
#include "esp_attr.h"

/*
#
#  ██████        ███████ ██████  ███    ███  
# ██             ██      ██   ██ ████  ████  
# ██   ███ █████ █████   ██   ██ ██ ████ ██ 
# ██    ██       ██      ██   ██ ██  ██  ██ 
#  ██████        ███████ ██████  ██      ██ 
#
# Configuration file
#
*/   


/**
  * 
  * Pinout
  * 
  * (The display pinout is not included here. The TFT pins are configured within the platform.io file)
  * 
  * The complete Pinout can be found in here:
  * /build-tutorial/ESP32-Pinout.txt
  * 
  * Note: vSense, Limits and Start/Stop don't use ADC Channels 
  * So changing the Pins here won't work. The ADC channels need to be changed in the sensros.cpp file
  * in this case
  * 
  * A list with the GPIOs and the corresponding ADC channel can be found here:
  * https://github.com/espressif/esp-idf/blob/e9d442d2b72/components/soc/esp32/include/soc/adc_channel.h
  * 
  **/
#define GEDM_PWM_PIN           22          // PWM for the pulsegenerator
#define GEDM_A_STEP_PIN        12          // spindle step pin
#define GEDM_A_DIRECTION_PIN   1           // spindle dir pin. Note hti s is the TX1 Pin. If UART is used it will changed the Pin state while communicating!
#define X_STEP_PIN             GPIO_NUM_27
#define X_DIRECTION_PIN        GPIO_NUM_26
#define X_LIMIT_PIN            GPIO_NUM_39
#define Y_STEP_PIN             GPIO_NUM_13
#define Y_DIRECTION_PIN        GPIO_NUM_14
#define Y_LIMIT_PIN            GPIO_NUM_39
#define Z_STEP_PIN             GPIO_NUM_25
#define Z_DIRECTION_PIN        GPIO_NUM_33
#define Z_LIMIT_PIN            GPIO_NUM_39
#define STEPPERS_DISABLE_PIN   GPIO_NUM_32 // one disable pin for all
#define STEPPERS_LIMIT_ALL_PIN GPIO_NUM_39 // one limit pin for all

// changing microsteps via software for Z is deprecated
// somewhere very far in the future those two Pins will be used for other stuff
#define Z_STEPPER_MS2_PIN      16          // only Z can use software controlled microstepping
#define Z_STEPPER_MS1_PIN      17          // only Z can use software controlled microstepping



const int X_AXIS = 0;  // Axis indexing value.
const int Y_AXIS = 1;
const int Z_AXIS = 2;
#define A_AXIS 3
#define B_AXIS 4
#define C_AXIS 5
#ifndef N_AXIS
#    define N_AXIS 3
#endif

/**
 * HOMING
 * 
 * Seek and feedrate is not used
 * 
 **/
#define DEFAULT_HOMING_ENABLE         1
#define DEFAULT_HOMING_FEED_RATE      50.0  // mm/min
#define DEFAULT_HOMING_SEEK_RATE      200.0 // mm/min
#define DEFAULT_HOMING_DEBOUNCE_DELAY 100   // msec (0-65k)
#define DEFAULT_HOMING_PULLOFF        2.0   // mm
#define DEFAULT_HOMING_DIR_MASK       0     // homing all axis into positive direction
//#define DEFAULT_HOMING_DIR_MASK (1<<Y_AXIS) 
//#define DEFAULT_HOMING_DIR_MASK (1<<X_AXIS) // x axis homing in inverted direction ( table seeks the switch to the right )
//#define DEFAULT_HOMING_DIR_MASK ((1<<X_AXIS)|(1<<Y_AXIS))
//#define HOMING_FORCE_SET_ORIGIN
//HOMING_FORCE_ORIGIN

/**
 * SPEEDS
 * 
 * At the moment it doesn't use mm based feedrates
 * GCode F feedrates are ginored
 * It uses different stepdelays for different tasks
 * Only Z uses a max feedrate that it doesn't exceeds
 **/
extern int DEFAULT_STEP_DELAY_RAPID;
extern int DEFAULT_STEP_DELAY_EDM;
#define DEFAULT_STEP_DELAY             150
#define DEFAULT_STEP_DELAY_PROBING     800
#define DEFAULT_STEP_DELAY_EDM_REPROBE 250
#define DEFAULT_STEP_DELAY_EDM_RETRACT 150
#define DEFAULT_STEP_DELAY_HOMING_FEED 400
#define DEFAULT_STEP_DELAY_HOMING_SEEK 100

extern float max_feeds[N_AXIS];
extern uint16_t max_feeds_micros[N_AXIS];

#define REPORT_ECHO_RAW_LINE_RECEIVED // debug only
#define MACHINE_NAME    "G-EDM"
#define ROOT_FOLDER     "/gedm3"
#define SETTINGS_FOLDER "/settings"
#define GCODE_FOLDER    "/gcode"
#define ENABLE_SD_CARD
#define STEP_PULSE_DELAY 0


/**
 Setting Value 	Mask 	Invert X 	Invert Y 	Invert Z
0 	00000000 	N 	N 	N
1 	00000001 	Y 	N 	N
2 	00000010 	N 	Y 	N
3 	00000011 	Y 	Y 	N
4 	00000100 	N 	N 	Y
5 	00000101 	Y 	N 	Y
6 	00000110 	N 	Y 	Y
7 	00000111 	Y 	Y 	Y
 **/
//#define DEFAULT_DIRECTION_INVERT_MASK 4 // uint8_t
#define DEFAULT_DIRECTION_INVERT_MASK 6 // uint8_t
//#define DEFAULT_DIRECTION_INVERT_MASK 7 // uint8_t

/**
  *
  * Z Axis
  *
  **/
//#define DEFAULT_HOMING_CYCLE_2 bit(Z_AXIS)
#define DEFAULT_Z_STEPS_PER_MM 4096.0 // belt driven uses a different formular: fullstepsPerRev*microsteps/(pitch*pulleyTeeth)
#define DEFAULT_Z_MAX_RATE 200.0 // mm/min
#define DEFAULT_Z_ACCELERATION 100.0 // mm/sec^2
#define DEFAULT_Z_MAX_TRAVEL 300.0 // mm NOTE: Must be a positive value.
#define STEPPER_MOTOR_MICROSTEPS_EDM 64 // default
#define STEPPER_MOTOR_MICROSTEPS_TRAVEL 32 // not used at the moment
#define Z_AXIS_JOG_SPEED 200
#define DEFAULT_Z_MAX_TRAVEL 300.0 // mm NOTE: Must be a positive value.


/**
  *
  * X Axis
  *
  **/
#define DEFAULT_HOMING_CYCLE_0 bit(X_AXIS)
#define DEFAULT_X_STEPS_PER_MM 3840.0//=MS32//960.0=MS8 // mm/min 5mm ballscrew 1:3 pulley reduction 8 microsteps ( fullstepsPerRev * microsteps * gearRation / ballscrew pitch ) 200 * 8 * 3 / 5
#define DEFAULT_X_MAX_RATE 200.0 // mm/min
#define DEFAULT_X_ACCELERATION 100.0 // mm/sec^2
#define DEFAULT_X_MAX_TRAVEL 240.0 // mm NOTE: Must be a positive value.
#define X_AXIS_JOG_SPEED 200
#define GEDM_DEFAULT_X_MICROSTEPS 32


/**
  *
  * Y Axis
  *
  **/
#define DEFAULT_HOMING_CYCLE_1 bit(Y_AXIS)
#define DEFAULT_Y_STEPS_PER_MM 3840.0
#define DEFAULT_Y_MAX_RATE 200.0 // mm/min
#define DEFAULT_Y_ACCELERATION 100.0 // mm/sec^2
#define DEFAULT_Y_MAX_TRAVEL 106.0 // mm NOTE: Must be a positive value.
#define Y_AXIS_JOG_SPEED 200
#define GEDM_DEFAULT_Y_MICROSTEPS 32


/**
  *
  * A axis / Spindle/Wire stepper 
  * 
  **/
#define DEFAULT_A_MICROSTEPS 64
#define DEFAULT_A_MAX_FREQUENCY 5000


// HARD_LIMIT_FORCE_STATE_CHECK
#define DEFAULT_SOFT_LIMIT_ENABLE 1 
#define DEFAULT_HARD_LIMIT_ENABLE 1 
#define DISABLE_LIMIT_PIN_PULL_UP
#define ENABLE_SOFTWARE_DEBOUNCE
#define DEFAULT_INVERT_LIMIT_PINS 0
#define DEFAULT_INVERT_ST_ENABLE 0
//#define LIMITS_TWO_SWITCHES_ON_AXES
#define DEFAULT_STEP_PULSE_MICROSECONDS 4
#define DEFAULT_STEPPER_IDLE_LOCK_TIME 255 // stay on
#define DEFAULT_SPINDLE_BIT_PRECISION 12
#define DEFAULT_SPINDLE_RPM_MIN   0.0
#define DEFAULT_SPINDLE_RPM_MAX   1023.0
#define DEFAULT_SPINDLE_MIN_VALUE 0.0
#define DEFAULT_SPINDLE_MAX_VALUE 1023.0
#define DEFAULT_SPINDLE_FREQ      20000.0
#define SPINDLE_PWM_BASE_FREQ     20000.0
#define DEFAULT_LASER_FULL_POWER  1023
#define DEFAULT_LASER_MODE        0
#define DEFAULT_SPINDLE_OFF_VALUE 0.0



#define FILTER_LEN 5 // number of samples for multisampling. Not used for vSense
#define EDM_NO_PROBE false
#define OPTIMIZE_MICROSTEPPING true // deprecated;
#define PWM_FREQUENCY_MAX 500000 // The PWM function used currently can only be as high as 70khz.
                                // The duty resolution sets the limit. Lower resolution would allow higher values.
                                // Above 50khz tests showed switch losses with the current setup I use
#define PWM_FREQUENCY_MIN 1000  // same as above. The PWM function can only get as low as 1000. in theory it goes lower but throws errors below 1000. Still seems to work if lower.
#define PWM_DUTY_MAX 90.0       // 100% should never be used. Maximum 80% to give a small off time.
/** custom PWM for probing process  **/
#define PROBE_SAMPLES 5 // the signal is noisy. To prevent false positives we take multiple samples
                         // after the probing thinks it touched the workpiece.
#define PROBE_SPEED 60.0 // mm/min; speed used until the first contact (short) was made
#define PROBE_RETRACTION_MM 1.0
/** servo feedback **/
#define VOLTAGE_DIVIDER_R1 10000
#define VOLTAGE_DIVIDER_R2 470 // 470 // the value of this resistor defines the maximum vSense voltage
#define VSENSE_SAMPLES 5 // the number of samples taken from the pin for each update of the vSense value
#define INTERFACE_INTERVAL 50.0             // faster if not in EMD process
#define INTERFACE_INTERVAL_WHILE_EDM 100.0 // lesser updates if in EDM process since the updates interupt our step pulses
#define LOADER_DELAY 2000.0


// https://github.com/G-EDM/G-EDM/#donations
#define EXTENDED_FEATURES true
#define ENABLE_SERIAL false

extern int wire_spindle_speed; // frequency in hz
extern float pwm_frequency; // 5khz - 10khz are common for rough cuts
extern float pwm_duty;      // the deeper the hole gets the higher the duty should be. I used 80% for a deep hole.
extern int operation_mode; // 1 = drill/sinker, 2 = reamer (just a constant up/down movement without progressing down), 3 = 3D floating engraver, 4 = 2D wire
extern int finish_after_mm; // f the tool moves 2mm without load it considers the process as done
extern float z_stop_after_mm; // the maximum travel into the workpiece; Note: EDM has electrode wear of x percent and setting this to 1mm doesn't produce a 1mm deep hole
extern bool z_no_home; // set to true if no homing is needed and edm process starts without homing
extern float source_voltage_min; // the minimum input voltage, EDM button won't do anything until this voltage is delivered from the PSU
extern DMA_ATTR bool lock_reference_voltage;
extern float vsense_drop_range_min; // target range bottom. We try to be between min and max and hodl there
extern float vsense_drop_range_max; // target range top. A default range from min+10 it could be min+40.
                                    // That produces lesser motion but a little motion is not a bad thing
                                    // for a more steady burn use min+20/30/40
extern float vSense_drop_range_noload; // no load if drop is below this percent; Pin reading is noisy and can deliver huge failures.
                                       // this can be a real problem while probing. It get's worse with lower voltages
                                       // example: At 25v the vSense Feedback is about 1v. 0.1v are 10%. And the spikes can be worse than that.
                                       // with the 0.1uf capacitor this should be no problem anymore
                                       // if probing fails the value can be changed in the software
extern float VSENSE_DROP_RANGE_SHORT;  // voltage drops above this value are concidered a full short circuit
extern float pwm_duty_probing;
extern int pwm_frequency_probing;
extern float electrode_up_offset_mm;   // distance above the probe position where the electrode up movement stops
extern float electrode_down_probe_mm;  // electrode down moves quickly down until there are given mm left to the target position and then starts reprobing
                                       // this increases speed a little and helps prevent some unexpected problems like false positives in deeper cuts etc 
extern bool protocol_ready;
extern DMA_ATTR bool is_between_steps;
extern DMA_ATTR bool force_redraw;
extern DMA_ATTR bool edm_process_done;       // if set to true in the edm process the task gets stopped from within the mainloop and also disables the spark PWM
extern DMA_ATTR bool edm_process_is_running; // this is true if the machine actually is doing work
extern DMA_ATTR int stop_edm_task;
extern DMA_ATTR int start_edm_task;
extern DMA_ATTR volatile unsigned long _millis; // storing the current millis
extern DMA_ATTR volatile unsigned long interface_timer; // this is used for the interval to update the interface
extern DMA_ATTR volatile bool limit_touched;
extern DMA_ATTR volatile bool probe_touched;
extern DMA_ATTR volatile float position_target[6];
extern DMA_ATTR volatile bool has_last_position;
extern DMA_ATTR volatile int motion_plan;
extern DMA_ATTR volatile bool cutting_depth_reached;
extern DMA_ATTR volatile bool take_over_z;
extern DMA_ATTR volatile bool enable_correction;
extern DMA_ATTR volatile unsigned long flush_retract_timer;
extern int   flush_offset_steps; // after moving the axis up for flushing it goes back down to the start position minus the offset steps
extern float flush_retract_mm;
extern float flush_retract_after; // retract tool after x ms to allow flushing; set to 0.0 to disable;
extern bool  disable_spark_for_flushing;
extern bool  enable_spindle;
extern float tool_diameter;
extern bool simulate_gcode; // if enabled the gcode will only move XY and won't react to up/down commands
extern DMA_ATTR volatile float reamer_travel_mm; // the travel distance for the reamer up/down oscillation.
                               // NOTE: In reamer moder the axis is homed and then moves down the reamer_travel_mm!!
                               // This is to not overschoot the home switch. It is necessary to make sure the axis can move the
                               // given mm down without crashing.
extern DMA_ATTR volatile float reamer_duration;  // the duration of the reasming process in seconds . 0.0 means it runs until manually stopped

extern char get_axis_name(uint8_t axis);
