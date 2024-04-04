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
#include "sensors.h"
#include "gedm_api/api.h"
#include "tft_display/ili9341_tft.h"
//#include <HardwareSerial.h> // just for debugging

G_EDM_SENSORS::G_EDM_SENSORS(){
    last_vSense_read_internal = ( uint64_t ) esp_timer_get_time();
    memset(counter, 0, sizeof(counter));
}


/** 
  * returns 
  * 0 = drop is below setpoint -> move
  * 1 = drop is within setpoint range at bottom end ( closer to noload than short circuit ) 
  * 2 = drop is within setpoint range at upper end ( closer to short circuit than noload )
  * 3 = drop is above setpoint or shorted
  * This raw plan does not validate the feedback
  * it reads the adc and returns the plan based on it without caring about noise
  **/
int G_EDM_SENSORS::get_raw_plan(){
    bool retract_only = false;
    int  plan         = 0;
    uint64_t last_reading_micros = ( uint64_t ) esp_timer_get_time() - last_vSense_read_internal;
    //update_vsense();
    if( last_reading_micros > 500 ){
      //update_vsense();
      retract_only = true;
    }
    current_voltage_drop = get_drop_in_percent( true );
    if( current_voltage_drop < vsense_drop_range_min ){
        if( retract_only ){ 
            plan = 1; 
        } else{ plan = 0; }
    } else if( current_voltage_drop > vsense_drop_range_max ){
        if( current_voltage_drop > VSENSE_DROP_RANGE_SHORT ){
            plan = 3;
        } else{ plan = 3; }
    } else if( vsense_drop_range_max - current_voltage_drop < current_voltage_drop - vsense_drop_range_min ){
        plan = 2;
    } else { 
        plan = 1;
    }
    return plan;
}


int G_EDM_SENSORS::get_motion_plan(){
    bool retract_only = false;
    int  plan         = get_raw_plan();


    return plan;
    ++counter[plan]; // update the multisample buffer

     counter[1] = 0; // not used
     counter[2] = 0; // not used

     if( plan == 0 ){
        counter[1] = 0;
        counter[2] = 0;
        counter[3] = 0;
        return plan;
     } else if( plan == 1 ){
         counter[0] = 0;
         counter[2] = 0;
         counter[3] = 0;
         return plan;
     } else if( plan == 2 ){
         counter[0] = 0;
         counter[1] = 0;
         counter[3] = 0;
         return plan;
     } else {
         counter[1] = 0;
         counter[2] = 0;
         counter[0] = 0;

         if( counter[plan] >= 3 ){ // confirm short circuit for x rounds
            return plan;
         } return 2; // hold until confirmed?

     }

    return plan;
}

void confirm_state( int state ){
}

void G_EDM_SENSORS::run_sensor_service()
{

    pin_high_at = 3700; // adc reading for digital high set close to max (4096) to compensate noise

    adc1_config_width(ADC_WIDTH_12Bit);

    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_11db);
    /** start vsense task **/
    xTaskCreatePinnedToCore(update_vsense_task, "update_vsense_task", 1400, this, 4, &vsense_task, 0);

    /** EDM start/stop switch **/
    adc1_config_channel_atten( ADC1_CHANNEL_0, ADC_ATTEN_11db );
    xTaskCreatePinnedToCore( update_start_stop_task, "update_start_stop_task", 1400, this, 1, &start_stop_task, 0 );

}

/** multisamples the vSense feedback and creates an average feedback voltage as reference **/
void G_EDM_SENSORS::generate_reference_voltage(){
    float tmp_ref_voltage = 0.0;
    int count             = 50;
    for( int i = 0; i < count; ++i ){
      tmp_ref_voltage += feedback_voltage;
      vTaskDelay( 10 );  
    }
    reference_voltage = tmp_ref_voltage / float( count );  
}

float G_EDM_SENSORS::calulate_source_voltage()
{
    //60/2.8*1.4
    return SUPPLY_VOLTAGE_MAX / VSENSE_MAX * feedback_voltage;
}
// calculates the feedback and source voltage based on the raw adc reading
void IRAM_ATTR G_EDM_SENSORS::update_feedback_data()
{
    //feedback_voltage = 3.3 / 4094.0 * float(vSense_analog);
    if ( ! edm_process_is_running && ! lock_reference_voltage )
    {
        // update_vsense();
        source_voltage    = calulate_source_voltage();
        reference_voltage = feedback_voltage;
    }
}
// Get the voltage drop in percent
float IRAM_ATTR G_EDM_SENSORS::get_drop_in_percent(bool _update)
{
    float voltage = 0.0;
    if (_update){
        update_feedback_data();
    }
    if (reference_voltage <= 0.0)
    {
        return 0.0;
    }
    /** calculate how much percent the voltage dropped **/
    float drop_in_percent = 100.0 - (100.0 / reference_voltage * feedback_voltage);
    return drop_in_percent < 0.0 ? 0.0 : drop_in_percent;
}

void IRAM_ATTR G_EDM_SENSORS::update_vsense( int num_samples )
{
    if( block_core_0_access ){
        // if num samples is a custom value it will consider
        // it as a reading from core 1 and locks access for core 0
        // until the reading is finished
        return;
    }
    int adc_reading = 0;
    num_samples     = 2;
    for (int i = 0; i < num_samples; ++i)
    {
        adc_reading += adc1_get_raw(ADC1_CHANNEL_6);
    }
    adc_reading /= num_samples;
    set_analog(adc_reading);
    set_analog_timestamp();
    block_core_0_access = false;
}

int G_EDM_SENSORS::get_vsense_raw()
{
    return vSense_analog;
}
void G_EDM_SENSORS::set_analog(int analog_raw)
{
    vSense_analog    = analog_raw;

    feedback_voltage = 3.3 / 4094.0 * float( read_multisample( vSense_analog, 1 ) );
    //feedback_voltage = 3.3 / 4094.0 * float(vSense_analog);
}
float G_EDM_SENSORS::get_feedback_voltage()
{
    return feedback_voltage;
}
float G_EDM_SENSORS::get_source_voltage()
{
    return source_voltage;
}
void G_EDM_SENSORS::set_analog_timestamp()
{
    last_vSense_read_internal = ( uint64_t ) esp_timer_get_time();
}

bool G_EDM_SENSORS::limit_is_touched(){
    int raw          = adc1_get_raw( ADC1_CHANNEL_3 ); 
    return read_multisample( raw, 0 ) > pin_high_at ? true : false;
}

/** check the state of the on/off switch. Returns true if switch is on. False if off. **/
bool G_EDM_SENSORS::edm_start_stop(){
  int raw     = adc1_get_raw( ADC1_CHANNEL_0 );  
  int sampled =  read_multisample( raw, 2 );
  return sampled > pin_high_at ? true : false;
}
bool G_EDM_SENSORS::continue_edm_process(){
  return stop_edm_task? false : true;
}

bool G_EDM_SENSORS::start_edm_process(){
  if( 
    sys_rt_exec_alarm != ExecAlarm::None 
    || sys.gedm_stop_process 
    || sys_rt_exec_state.bit.motionCancel
){
    return false;
  } return start_edm_task;
}

bool G_EDM_SENSORS::is_probing(){
    return sys_probe_state == Probe::Active ? true : false;
}

void G_EDM_SENSORS::collect_probe_samples(){

    if( probe_touched ){
        vTaskDelay(1);
        return;
    }

    if( get_drop_in_percent( true ) < vSense_drop_range_noload ){

      /** reset counter **/
      probe_positive_count = 0;
      probe_touched        = false;
        
    } else{

        sys.edm_pause_motion_probe = true;

        while( ++probe_positive_count <= PROBE_SAMPLES ){

            update_vsense();

            if( get_drop_in_percent( true ) < vSense_drop_range_noload ){
                probe_touched = false;
                break;
            }

            //vTaskDelay(1);

        }

        if( probe_positive_count >= PROBE_SAMPLES ){
          /** should be valid **/
          probe_touched = true;
        }
    
    }

    sys.edm_pause_motion_probe = false;

}


/** 
  * map_id 0 = limit switches 
  **/
uint32_t G_EDM_SENSORS::read_multisample(int raw, int map_id){
  int i = 0;
  uint32_t Sum = 0;
  multisample_buffer[map_id][multisample_counts[map_id]++] = raw;
  if(multisample_counts[map_id] == FILTER_LEN){
    multisample_counts[map_id] = 0;
  }
  for(i=0; i<FILTER_LEN; i++){
    Sum += multisample_buffer[map_id][i];
  }
  return (Sum/FILTER_LEN);
}





/**
 * Task on core 0 with lower priority that
 * checks if the start/stop button is off or on
 **/
void IRAM_ATTR update_start_stop_task(void *parameter)
{

  G_EDM_SENSORS *sensors = (G_EDM_SENSORS *)parameter;

  for (;;)
  {

        if (sensors->edm_start_stop())
        {
            stop_edm_task = 0;
            if (!edm_process_is_running)
            {
                start_edm_task = 1;
            }
            if( !edm_process_is_running && sys_rt_exec_state.bit.motionCancel ){
                sys_rt_exec_state.bit.motionCancel = false;
                force_redraw = true;
            }
            sys_rt_exec_state.bit.motionCancel = false;
        }
        else
        {
            start_edm_task = 0;
            if (edm_process_is_running)
            {
                stop_edm_task = 1;
            }
            if( !edm_process_is_running && !sys_rt_exec_state.bit.motionCancel ){
                sys_rt_exec_state.bit.motionCancel = true;
                force_redraw = true;
            }
            sys_rt_exec_state.bit.motionCancel = true;
        }
        vTaskDelay(300);
  }
  vTaskDelete(NULL);

}

/**
 * Task on core 0 to update the vSense value
 * Runs slower if EDM process is not running
 **/
void IRAM_ATTR update_vsense_task(void *parameter)
{

    G_EDM_SENSORS *sensors = (G_EDM_SENSORS *)parameter;

    for (;;)
    {

        sensors->update_vsense();

        /** if not in EDM process add a little delay to reduce computing **/
        if ( ! edm_process_is_running && ! sensors->is_probing() )
        {
            vTaskDelay(10); // 100 = 10 times per second
        }
        else
        {
            if( sensors->is_probing() ){

                while( sensors->is_probing() ){

                    sensors->update_vsense();
                    sensors->collect_probe_samples();
                    vTaskDelay(1); 

                }

            }
            vTaskDelay(1); // 5 = 167/s
        }
        
    }
    vTaskDelete(NULL);

}


/**
 * Task to check the limit switches
 **/
void IRAM_ATTR update_limit_switch_task(void *parameter)
{

    G_EDM_SENSORS *sensors = (G_EDM_SENSORS *)parameter;

    for (;;)
    { 
        limit_touched = sensors->limit_is_touched();
        vTaskDelay( sys.state == State::Homing ? 50 : 100 ); 
    }
    vTaskDelete(NULL);

}
