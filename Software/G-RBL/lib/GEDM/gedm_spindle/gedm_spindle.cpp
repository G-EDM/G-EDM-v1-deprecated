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
#include "gedm_spindle.h"
#include "config/definitions.h"
#include <Arduino.h>


G_EDM_SPINDLE gedm_spindle = G_EDM_SPINDLE();

G_EDM_SPINDLE::G_EDM_SPINDLE(){}

void G_EDM_SPINDLE::setup_spindle( int _dir_pin, int _step_pin ){
    default_frequency = 5000;
    dir_pin      = _dir_pin;
    step_pin     = _step_pin;
    frequency    = default_frequency;
    dir_inverted = false;
    pinMode(dir_pin, OUTPUT);
    digitalWrite(dir_pin,LOW);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, step_pin);
    mcpwm_config_t pwm_config = {};
    pwm_config.frequency      = frequency;
    pwm_config.cmpr_a         = 0;
    pwm_config.cmpr_b         = 0;
    pwm_config.counter_mode   = MCPWM_UP_COUNTER;
    pwm_config.duty_mode      = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    stop_spindle();
}
int G_EDM_SPINDLE::rpm_to_frequency( float rpm ){
    return round(rpm/60.0*float(200*DEFAULT_A_MICROSTEPS) );//0.46/60*(200*64)
}
float G_EDM_SPINDLE::frequency_to_rpm( float _frequency ){
    return _frequency*60/(200*DEFAULT_A_MICROSTEPS);//0.46/60*(200*64)
}
void G_EDM_SPINDLE::reset_spindle(){
    set_speed( default_frequency );
}
int G_EDM_SPINDLE::get_speed(){
    return frequency;
}
bool G_EDM_SPINDLE::set_speed( int __frequency ){
    if( frequency == __frequency ){ return true; }
    frequency = __frequency;
    mcpwm_set_frequency( MCPWM_UNIT_0, MCPWM_TIMER_0, __frequency );
    if( !is_running ){ return false; }
    mcpwm_set_duty( MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0 );
    return true;
}
void G_EDM_SPINDLE::start_spindle(){
    if( is_running ){ return; }
    mcpwm_set_frequency( MCPWM_UNIT_0, MCPWM_TIMER_0, frequency );
    mcpwm_set_duty( MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0 );
    is_running = true;
}
void G_EDM_SPINDLE::stop_spindle(){
    if( ! is_running ){ return; }
    //mcpwm_set_frequency( MCPWM_UNIT_0, MCPWM_TIMER_2, 0 );
    mcpwm_set_duty( MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0 );
    is_running = false;
}
void G_EDM_SPINDLE::set_spindle_direction( bool direction ){
}
bool G_EDM_SPINDLE::spindle_is_running(){
    return is_running;
}
void G_EDM_SPINDLE::reverse_direction(){
    if( dir_inverted ){
        digitalWrite(dir_pin,LOW);
        dir_inverted = false;
    } else{
        digitalWrite(dir_pin,HIGH);
        dir_inverted = true;
    }
}
bool G_EDM_SPINDLE::dir_is_inverted(){
    return dir_inverted;
}