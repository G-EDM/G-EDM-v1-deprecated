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
#include "pwm_controller.h"
#include <stdio.h>
//#include <HardwareSerial.h>


void G_EDM_PWM_CONTROLLER::setup_pwm_channel(){
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, pwm_pin);
    mcpwm_config_t pwm_config = {};
    pwm_config.frequency      = pwm_frequency;
    pwm_config.cmpr_a         = 0;
    pwm_config.cmpr_b         = 0;
    pwm_config.counter_mode   = MCPWM_UP_COUNTER;
    pwm_config.duty_mode      = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);
}

G_EDM_PWM_CONTROLLER::G_EDM_PWM_CONTROLLER(){
  freq_max           = 20000;
  freq_min           = 1000;    
  pwm_frequency      = 5000;
  pwm_duty_cycle_percent = 0;
  pwm_duty_cycle     = 0;
  pwm_max_duty_cycle = 1023;//255;//@8bit 1023@@10bit resolution
  duty_cycle_percent = 0.0;
  pwm_period         = 0.0;
  pwm_t_on           = 0.0;
  pwm_t_off          = 0.0;
  spark_generator_is_running = false;
};
void G_EDM_PWM_CONTROLLER::set_pwm_pin( int pin ){
  pwm_pin = pin;
}
bool G_EDM_PWM_CONTROLLER::pwm_is_enabled(){
  return spark_generator_is_running;
}
int G_EDM_PWM_CONTROLLER::get_freq(){
  return pwm_frequency;
}
int G_EDM_PWM_CONTROLLER::get_duty(){
  return pwm_duty_cycle;
}
float G_EDM_PWM_CONTROLLER::get_duty_percent(){
  return duty_cycle_percent;
}
float G_EDM_PWM_CONTROLLER::get_period(){
  return pwm_period;
}
float G_EDM_PWM_CONTROLLER::get_t_on(){
  return pwm_t_on;
}
float G_EDM_PWM_CONTROLLER::get_t_off(){
  return pwm_t_off;
}    
void G_EDM_PWM_CONTROLLER::set_min_max( int _min, int _max ){
  freq_min = _min;
  freq_max = _max;
}
void G_EDM_PWM_CONTROLLER::change_pwm_frequency( int freq ){
  if( freq < freq_min ){
    freq = freq_min;  
  } else if( freq > freq_max ){
    freq = freq_max;
  }
  pwm_frequency = freq;
  mcpwm_set_frequency(MCPWM_UNIT_1, MCPWM_TIMER_0, freq);

}
void G_EDM_PWM_CONTROLLER::update_duty( float duty_percent ){
  //int duty = int( ( float ) pwm_max_duty_cycle / 100.0 * duty_percent );
  //change_pwm_duty( duty );
  change_pwm_duty( duty_percent );
  //pwm_duty_cycle = duty;
  pwm_duty_cycle_percent = duty_percent;
  update_values();
}
void G_EDM_PWM_CONTROLLER::change_pwm_duty( int duty ){
  if( ! spark_generator_is_running && ! lock_reference_voltage ){
    duty = 0;
  }
  mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, float( duty ) );
}
void G_EDM_PWM_CONTROLLER::pwm_off(){
  change_pwm_duty( 0 );
}
void G_EDM_PWM_CONTROLLER::pwm_on(){
  change_pwm_frequency( pwm_frequency );
  change_pwm_duty( pwm_duty_cycle_percent );
}
void G_EDM_PWM_CONTROLLER::update_values(){      
  duty_cycle_percent = pwm_duty_cycle_percent;//100.0 / float( pwm_max_duty_cycle ) * float( pwm_duty_cycle );
  pwm_period         = 1.0   / float( pwm_frequency );
  pwm_t_on           = pwm_period / 100.0 * float( duty_cycle_percent );
  pwm_t_off          = pwm_period - pwm_t_on;
  pwm_period         = pwm_period;
}
void G_EDM_PWM_CONTROLLER::toggle_pwm_on_off(){
  return spark_generator_is_running ? disable_spark_generator() : enable_spark_generator();
}
void G_EDM_PWM_CONTROLLER::disable_spark_generator(){
    //api::push_cmd("M3");
    pwm_off();
    spark_generator_is_running = false;
}
void G_EDM_PWM_CONTROLLER::enable_spark_generator(){
    spark_generator_is_running = true;
    pwm_on();
    spark_generator_is_running = true;
}

void G_EDM_PWM_CONTROLLER::probe_mode_on(){
  lock_reference_voltage = true;
  duty_percent_backup = get_duty_percent();
  frequency_backup    = get_freq();
  change_pwm_frequency( pwm_frequency_probing );
  update_duty( pwm_duty_probing );
  pwm_on();
}
void G_EDM_PWM_CONTROLLER::probe_mode_off(){
  lock_reference_voltage = false;
  change_pwm_frequency( frequency_backup );
  update_duty( duty_percent_backup );
  pwm_off();
}
