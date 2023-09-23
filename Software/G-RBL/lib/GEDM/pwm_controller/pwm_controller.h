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
//#include "driver/ledc.h"

#include "config/definitions.h"
#include "driver/mcpwm.h"
//#include <soc/mcpwm_reg.h>
//#include <soc/mcpwm_struct.h>


/**
 * PWM controller class
 **/
class G_EDM_PWM_CONTROLLER
{
private:
  int pwm_pin;  // PWM+ output pin
  int freq_max;
  int freq_min;
  int pwm_frequency;
  int pwm_duty_cycle;
  int pwm_max_duty_cycle; // 255;//@8bit 1023@@10bit resolution
  float duty_cycle_percent;
  float pwm_period;
  float pwm_t_on;
  float pwm_t_off;
  bool spark_generator_is_running; // this only marks if the spark engine is running. Even with a duty of 0 it can be running
  //ledc_timer_config_t   spark_pwm_timer;
  //ledc_channel_config_t spark_pwm_channel;
  float duty_percent_backup;
  int frequency_backup;

public:
  G_EDM_PWM_CONTROLLER(void);
  void setup_pwm_channel(void);
  bool pwm_is_enabled(void);
  int get_freq(void);
  int get_duty(void);
  float get_duty_percent(void);
  float get_period(void);
  float get_t_on(void);
  float get_t_off(void);
  void set_min_max(int _min, int _max);
  void set_pwm_pin( int pin );
  void change_pwm_frequency(int freq);
  void change_pwm_duty(int duty);
  void pwm_off(void);
  void pwm_on(void);
  void update_values(void);
  void toggle_pwm_on_off(void);
  void disable_spark_generator(void);
  void enable_spark_generator(void);
  void update_duty(float duty_percent);

  void probe_mode_on( void );
  void probe_mode_off( void );

};
