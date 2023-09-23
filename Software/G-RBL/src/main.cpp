// Remove all serial.prints


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

//################################################
// Main includes
//################################################
#include <driver/adc.h>
#include "gedm_main.h"
#include "config/definitions.h"
#include "sd_card/filehandler.h"

//################################################
// GRBL_ESP32
//################################################
#include "Grbl.h"

//#include <Arduino.h>

String active_profile = "None";

/** task related **/
TaskHandle_t vsense_task     = NULL;
TaskHandle_t start_stop_task = NULL;
TaskHandle_t ui_task_handle  = NULL;

void start_services(){
  // main tasks for the UI and touch
  ui_controller.start_ui_task();
  // sensor service includes the vSense feedback and the EDM start stop switch
  ui_controller.run_sensor_service();
}

void update_ui_controller(){
    ui_controller.set_voltage_min( source_voltage_min );
    ui_controller.set_drop_min_max( vsense_drop_range_min, vsense_drop_range_max );
    ui_controller.set_drop_short(VSENSE_DROP_RANGE_SHORT);
    ui_controller.set_drop_min_max( vsense_drop_range_min, vsense_drop_range_max );
    ui_controller.set_voltage_min( source_voltage_min );
    ui_controller.set_ms_edm( STEPPER_MOTOR_MICROSTEPS_EDM );
    ui_controller.set_ms_travel( STEPPER_MOTOR_MICROSTEPS_TRAVEL );
    ui_controller.set_min_max( PWM_FREQUENCY_MIN, PWM_FREQUENCY_MAX );
    ui_controller.change_pwm_frequency( ( int ) pwm_frequency );
    ui_controller.update_duty( pwm_duty );
    ui_controller.set_pwm_pin( GEDM_PWM_PIN );
    ui_controller.setup_pwm_channel();
    ui_controller.disable_spark_generator();
}


void setup() {
    if( ENABLE_SERIAL ){
      Serial.begin(9600);
    }
    disableCore0WDT();
    disableCore1WDT();
    grbl_init();
    ui_controller.init();
    ui_controller.set_filehandler( &filehandler );
    update_ui_controller();
    start_services();
    ui_controller.set_is_ready();
}

void loop(){
  run_once();
  vTaskDelay(2000);
}

