/*
#  ██████        ███████ ██████  ███    ███  
# ██             ██      ██   ██ ████  ████  
# ██   ███ █████ █████   ██   ██ ██ ████ ██ 
# ██    ██       ██      ██   ██ ██  ██  ██ 
#  ██████        ███████ ██████  ██      ██ 
#
#      
*/ 
#include "gedm_stepper.h"
    
rmt_item32_t G_EDM_STEPPER::rmtItem[2];
rmt_config_t G_EDM_STEPPER::rmtConfig;


bool G_EDM_STEPPER::set_homing_mode(bool isHoming){
  return true;
}


// Get an RMT channel number
// returns RMT_CHANNEL_MAX for error
rmt_channel_t G_EDM_STEPPER::get_next_RMT_chan_num() {
    static uint8_t next_RMT_chan_num = uint8_t(RMT_CHANNEL_0);  // channels 0-7 are valid
    if (next_RMT_chan_num < RMT_CHANNEL_MAX) {
        next_RMT_chan_num++;
    }
    return rmt_channel_t(next_RMT_chan_num);
}


G_EDM_STEPPER::G_EDM_STEPPER(uint8_t axis_index, uint8_t step_pin, uint8_t dir_pin) :
    //Motor(axis_index)
    _axis_index(axis_index % MAX_AXES)
    ,_dual_axis_index(axis_index / MAX_AXES)
    , _step_pin(step_pin)
    , _dir_pin(dir_pin) 
{

    _rmt_chan_num = get_next_RMT_chan_num();

}
void G_EDM_STEPPER::init() {
    _direction = 2; // initial

    if( ! fixed_microsteps ){
        pinMode( ms1_pin ,OUTPUT );
        pinMode( ms2_pin ,OUTPUT ); 
    }

    configure_microsteps();
    read_settings();
}
void G_EDM_STEPPER::read_settings() { init_step_dir_pins(); }
void G_EDM_STEPPER::init_step_dir_pins() {
    _invert_step_pin = bitnum_istrue(DEFAULT_STEPPING_INVERT_MASK, _axis_index);
    _invert_dir_pin  = bitnum_istrue(DEFAULT_DIRECTION_INVERT_MASK, _axis_index);
    pinMode(_dir_pin, OUTPUT);
    rmtConfig.rmt_mode                       = RMT_MODE_TX;
    rmtConfig.clk_div                        = 20;
    rmtConfig.mem_block_num                  = 2;
    rmtConfig.tx_config.loop_en              = false;
    rmtConfig.tx_config.carrier_en           = false;
    rmtConfig.tx_config.carrier_freq_hz      = 0;
    rmtConfig.tx_config.carrier_duty_percent = 50;
    rmtConfig.tx_config.carrier_level        = RMT_CARRIER_LEVEL_LOW;
    rmtConfig.tx_config.idle_output_en       = true;
    auto stepPulseDelay                      = STEP_PULSE_DELAY;
    rmtItem[0].duration0                     = stepPulseDelay < 1 ? 1 : stepPulseDelay * 4;
    rmtItem[0].duration1                     = 4 * DEFAULT_STEP_PULSE_MICROSECONDS;
    rmtItem[1].duration0                     = 0;
    rmtItem[1].duration1                     = 0;
    if (_rmt_chan_num == RMT_CHANNEL_MAX) {
        return;
    }
    rmt_set_source_clk(_rmt_chan_num, RMT_BASECLK_APB);
    rmtConfig.channel              = _rmt_chan_num;
    rmtConfig.tx_config.idle_level = _invert_step_pin ? RMT_IDLE_LEVEL_HIGH : RMT_IDLE_LEVEL_LOW;
    rmtConfig.gpio_num             = gpio_num_t(_step_pin);
    rmtItem[0].level0              = rmtConfig.tx_config.idle_level;
    rmtItem[0].level1              = !rmtConfig.tx_config.idle_level;
    rmt_config(&rmtConfig);
    rmt_fill_tx_items(rmtConfig.channel, &rmtItem[0], rmtConfig.mem_block_num, 0);
    //pinMode(_disable_pin, OUTPUT);
}
void G_EDM_STEPPER::step() {
    RMT.conf_ch[_rmt_chan_num].conf1.mem_rd_rst = 1;
    RMT.conf_ch[_rmt_chan_num].conf1.tx_start   = 1;
}
void G_EDM_STEPPER::unstep(){}
int G_EDM_STEPPER::set_direction(bool dir) { 
    if( dir == _direction ){
        return 0;
    } 
    _direction = dir;
    int delay = 100;
    digitalWrite(_dir_pin, dir ^ _invert_dir_pin); 
    delayMicroseconds(delay);
    return delay;
}
void G_EDM_STEPPER::set_disable(bool disable) { 
}

void G_EDM_STEPPER::steps_down( int steps ){

}
void G_EDM_STEPPER::steps_up( int steps ){

}

void G_EDM_STEPPER::set_ms1_pin( int pin ){
  ms1_pin = pin;
}
void G_EDM_STEPPER::set_ms2_pin( int pin ){
  ms2_pin = pin;
}
/** this disables runtime changes of microsteps for this axis **/
void G_EDM_STEPPER::set_fixed_microsteps( bool _fixed_microsteps ){
  fixed_microsteps = _fixed_microsteps;
}
void G_EDM_STEPPER::set_microsteps( int _microsteps){
    microsteps = _microsteps;
}
void G_EDM_STEPPER::set_microsteps_travel( int microsteps ){
  microsteps_travel = microsteps;
}
void G_EDM_STEPPER::set_microsteps_edm( int microsteps ){
  microsteps_edm = microsteps;
}
/** changing microsteps from software is only supported for the z axis **/
void G_EDM_STEPPER::configure_microsteps(){
  if( ! fixed_microsteps ){
    if( microsteps == 8 ){
      digitalWrite( ms2_pin, LOW );
      digitalWrite( ms1_pin, LOW );
    } else if( microsteps == 16 ){
      digitalWrite( ms2_pin, HIGH );
      digitalWrite( ms1_pin, HIGH );
    } else if( microsteps == 32 ){
      digitalWrite( ms2_pin, LOW );
      digitalWrite( ms1_pin, HIGH );
    } else if( microsteps == 64 ){
      digitalWrite( ms2_pin, HIGH );
      digitalWrite( ms1_pin, LOW );
    }
    vTaskDelay(50);
  }
  calc_microsteps_per_rev();
}
void G_EDM_STEPPER::calc_microsteps_per_rev(){
  //microsteps_per_rev      = round( steps_per_rev * microsteps );
  //microsteps_per_gear_rev = round( microsteps_per_rev * gear_ratio );
}

void G_EDM_STEPPER::set_steps_per_mm( int steps ){
    steps_per_mm = steps;
}
int G_EDM_STEPPER::get_steps_per_mm(){
    return steps_per_mm;
}

int G_EDM_STEPPER::get_step_delay_for_speed( float mm_min ){
  // mm per single step
  double mm_per_step = 1.0 / ( double ) steps_per_mm;
  double delay = mm_per_step / ( double ) mm_min * 60000000;
  return round( delay );
}