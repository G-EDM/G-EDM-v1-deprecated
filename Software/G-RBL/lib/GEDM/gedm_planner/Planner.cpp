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


/**
 * 
 * Warning!
 * The wire code with history retractions is not working at the moment
 * Don't use it. 
 * 
 **/
#include "Grbl.h"
#include <stdlib.h>  

#include "tft_display/ili9341_tft.h"


G_EDM_PLANNER planner = G_EDM_PLANNER();

G_EDM_PLANNER::G_EDM_PLANNER(){
    enable_wire_mode         = false;
    z_cutting_depth          = 0.0;
    deepest_step_pos         = 0;
    short_circuit_retraction = false;
    default_pl_data = &default_plan_data;
    memset(default_pl_data, 0, sizeof(plan_line_data_t));
    memset(total_mm_work, 0, sizeof(total_mm_work));
    default_pl_data->motion                 = {};
    default_pl_data->motion.systemMotion    = 1;
    default_pl_data->process_marker         = 1;
    position_history_reset();
    init_ready = true;
    reset_planner_state();
}



/**
  * Pausing and Resume 
  **/
bool G_EDM_PLANNER::get_is_paused(){
    return is_paused;
}
void G_EDM_PLANNER::pause(){
    if( is_paused ){ return; }
    is_paused = true;
    ui_controller.pwm_off();
    ui_controller.stop_spindle(); // don't waste the precious wire
    while( sys.edm_pause_motion ){
        if( sys.abort || sys_rt_exec_state.bit.motionCancel ){ break; }
        vTaskDelay(100);
    }
    resume();
}
void G_EDM_PLANNER::resume(){
    if( ! is_paused ){ return; }
    if( enable_spindle ){
        ui_controller.start_spindle();
    }
    is_paused = false;
    ui_controller.pwm_on();
}

/** 
  * adds the step delay and signals core 0 that it has some time to do something if needed 
  **/
bool G_EDM_PLANNER::add_step_delay( int _step_delay, bool enable_motion_plan ){
    microseconds     = ( uint64_t ) esp_timer_get_time();

    int micros_since = microseconds - last_step_timestamp;
    _step_delay     -= micros_since;
    _step_delay      = MAX(30,_step_delay);
    bool _success    = true;
    //_step_delay -= micros_since;
    if( _step_delay < 30 ){
        _step_delay = 30; // to be safe
    }
    int step_end_delay = 20;
    int first_step_delay = _step_delay - step_end_delay;
    if( first_step_delay <= 0 ){
        first_step_delay = _step_delay;
        step_end_delay   = 0;
    }
    is_between_steps = true;
    delayMicroseconds( first_step_delay );
    is_between_steps = false;
    delayMicroseconds( step_end_delay );
    last_step_timestamp = ( uint64_t ) esp_timer_get_time();
    return _success;
}


/** 
  * called while stepping and if probing is active checks the probe state and 
  * inserts a pause until a positive is confirmed 
  * see sensors.cpp for more details
  **/
bool G_EDM_PLANNER::probe_check( Line_Config &line ){
    if (sys_probe_state == Probe::Active) {
        if( line.tmp > 0 ){
            --line.tmp;
        }
        if( line.tmp == 0 ){
            line.step_delay = DEFAULT_STEP_DELAY_PROBING;
        }
        ui_controller.pwm_on(); // make sure PWM is on
        line.step_delay = DEFAULT_STEP_DELAY_PROBING;
        /** to get a more sensitive probing we pause the motion until the probing is confirmed and resume if it was a false positive **/
        while( 
            sys.edm_pause_motion_probe 
            //|| ui_controller.get_motion_plan() >= 2 
        ){
            if( sys.abort || sys_rt_exec_state.bit.motionCancel ){ break; }
            vTaskDelay(10);
            if( line.step_delay != DEFAULT_STEP_DELAY_PROBING ){
                line.step_delay *= 3;
                line.tmp = 500; // keep slow speed for x steps
            }
        }

        if( probe_state_monitor() ){
            ui_controller.pwm_off();
            sys_probe_state = Probe::Off;
            return true;
        }
    }
    return false;
}
bool G_EDM_PLANNER::is_ready(){
    return init_ready ? true : false;
}
void G_EDM_PLANNER::update_max_feed(){
    G_EDM_STEPPER* _stepper;
    for( int i = 0; i < N_AXIS; ++i )
    {
        _stepper = motor_manager.get_motor( i );
        int max_feed_step_delay = _stepper->get_step_delay_for_speed( max_feeds[i] );
        max_feeds_micros[ i ] = max_feed_step_delay;
        last_step_micros[i] = ( uint64_t ) esp_timer_get_time();
        //Serial.println("Step delay for: " + String( max_feeds[i] ) +"mm/min -> "+ String( max_feed_step_delay ) );
    }
}
int G_EDM_PLANNER::get_current_round(){
    return gcode_round;
}
void G_EDM_PLANNER::next_gcode_round(){
    // next file pass
    if( gcode_round == 1 ){
        //Serial.println( "Initial round finished:" );
        if( sys.gedm_wire_gcode_task ){
            // wire only does one round
            // process finished
            sys.edm_process_finished = true;
        }
    }
    ++gcode_round;
}


void print_target( float* target ){
    String t = "    Target: ";
    for( int i = 0; i < N_AXIS; ++i ){
        t += String( i ) + ": " + String( target[i],3)+" ";
    }
    //Serial.println( t );
}



/** 
 * Use with care! It is for a special case and should never be used for other purposes.The step daly it out of sync with the default step delay math. 
 * For example: If Z is moved with a custom step and after that z is moved with the normal line stepping
 * It may happen that there is a zero step delay between the two steps
 **/
void G_EDM_PLANNER::stepper_step_custom( int axis, bool dir, int _step_delay ){
    G_EDM_STEPPER* _stepper = motor_manager.get_motor( axis );
    if( sys_position[axis] > (DEFAULT_HOMING_PULLOFF*-1) * axis_settings[ axis ]->steps_per_mm->get() ){
        return;
    }
    int dir_delay = _stepper->set_direction( dir ); // added 50mikros after a dirchange!

    if( _step_delay > 0 ){
        add_step_delay( _step_delay );
    }
    _stepper->step();
    if ( dir ) {
        sys_position[axis]--;
    } else {
        sys_position[axis]++;
    }
    last_step_micros[axis] = ( uint64_t ) esp_timer_get_time();
    last_step_timestamp    = ( uint64_t ) esp_timer_get_time();
    return;
}



/** 
  * if possible hammer the position vertically until the short circuit position is free 
  * if the max number of rounds are exceeded is switches pwm on and off for some rounds and then goes on
  * with the normale process 
  **/
void G_EDM_PLANNER::unshort_hard( Line_Config &line ){
    line.failure_step_pos = sys_position[Z_AXIS];
    line.failure_step_pos++; // remove a step
    bool recovered = false;
    int max_rounds  = 15;
    int rounds_done = 0;
    while( !recovered && motion_plan >= 2 ){
        if( sys.abort || sys_rt_exec_state.bit.motionCancel ){
            recovered = false;
            break;
        }
        motion_plan = ui_controller.get_motion_plan(); 
        floating_edm_engraver( DEFAULT_STEP_DELAY_EDM_REPROBE, line );
        if( sys_position[Z_AXIS ] == line.failure_step_pos ){
            recovered = true;
        } else{
            recovered = false;
        }
        if( ++rounds_done > max_rounds ){
            if( motion_plan == 2 ){
                for(int i=0; i < 10; ++i ){
                    ui_controller.pwm_off();
                    vTaskDelay(1);
                    ui_controller.pwm_on();
                }
            }
            //Serial.println("Unshort failure position failed!");
            recovered = false;
            ui_controller.pwm_on();
            break;
        }
    }
}




bool G_EDM_PLANNER::floating_edm_engraver( int _step_delay, Line_Config &line ){
    
    int steps      = 0;
    bool direction = false;

    switch ( motion_plan )
    {
        case 3:
            line.skip_feed     = true;
            line.skip_floating = true; // step up
            // direction false for up and true for down
            ui_controller.pwm_off();
            if( line.setpoint_step_position != 0 ){
                steps = MAX( 20, sys_position[Z_AXIS]*-1+line.setpoint_step_position);
                if( steps > 40 ){ steps = 40; }
            } 
            steps = MAX(20,steps);
            for( int i = 0; i < steps; ++i ){
                stepper_step_custom( Z_AXIS, false, DEFAULT_STEP_DELAY_EDM_RETRACT ); // extra steps up
            }
            ui_controller.pwm_on();
            motion_plan = ui_controller.get_motion_plan(); 
            last_workload_at = sys_position[Z_AXIS]; 
        break;

        case 2:
            line.skip_feed     = true;
            line.skip_floating = true; // step up
            // direction false for up and true for down
            if( line.setpoint_step_position != 0 ){
                steps = MAX( 10, sys_position[Z_AXIS]*-1+line.setpoint_step_position);
                if( steps > 20 ){ steps = 20; }
            } 
            steps = MAX(10,steps);
            for( int i = 0; i < steps; ++i ){
                stepper_step_custom( Z_AXIS, false, DEFAULT_STEP_DELAY_EDM ); // extra steps up
            }
            motion_plan = ui_controller.get_motion_plan(); 
            last_workload_at = sys_position[Z_AXIS]; 
        break;

        case 1:
            line.skip_floating = true;
            line.setpoint_step_position = sys_position[Z_AXIS];
        break;

        default:
            if( line.last_motion_plan != motion_plan ){ // skip the down movement until it is confirmed in the enxt round
                line.skip_floating = true;
            } else{
                direction  = true; // step down
            }
        break;

    }
    
    line.last_motion_plan = motion_plan;
    
    /** Floating the z axis **/
    if( !line.skip_floating ){

        if( line.failure_step_pos != 0 && sys_position[Z_AXIS] <= line.failure_step_pos  ){
            // maybe remove this?
            // if a line created a short this failure position is the max depth for the rest of the line
            //return false;
        }
        
        microseconds = ( uint64_t ) esp_timer_get_time();

        if( direction ){
            if(

                // conditions for a single step down
                !work_is_at_cutting_depth_limit() 
                && !no_workload_finish_reached()
                //&& ( line.setpoint_step_position == 0 || line.setpoint_step_position < sys_position[Z_AXIS] )
                && ( microseconds - last_step_micros[Z_AXIS] > max_feeds_micros[ Z_AXIS ] )

            ){ stepper_step_custom( Z_AXIS, direction, _step_delay ); }
        }
        
    }

    return false;

}

bool G_EDM_PLANNER::pre_process_floating( int _step_delay, Line_Config &line ){
    motion_plan = ui_controller.get_motion_plan(); 
        if( line.floating_enabled ){
        if( motion_plan >= 2 ){
            unshort_hard( line );
        } else if( motion_plan <= 2 ){
            floating_edm_engraver( _step_delay, line );
        }
    } 
    return true;
}
bool G_EDM_PLANNER::process_wire( Line_Config &line ){

    bool _success = true;
    motion_plan   = ui_controller.get_motion_plan(); 

    if( motion_plan == 0 ){
        //ui_controller.set_speed( ui_controller.rpm_to_frequency( 6 ) );
    } else{
        //ui_controller.set_speed( wire_spindle_speed );
    }

    
    
    if( short_circuit_retraction ){
        // short circuit retraction only needs to 
        // move until the short is gone
        line.step_delay = MAX( DEFAULT_STEP_DELAY_EDM, max_feeds_micros[ Z_AXIS ] );
        if( motion_plan <= 2 ){
            return false;
        }
        ui_controller.pwm_off();
        vTaskDelay(1);
        ui_controller.pwm_on();
        vTaskDelay(1);

    } else {

    if( motion_plan == 3 ){
        int count = 0;
        // hard short or above setpoint
        while(++count<20){
            vTaskDelay(1);
            ui_controller.pwm_off();
            vTaskDelay(1);
            ui_controller.pwm_on();
        }
        return false;
    }
    // 1 and 2 are both within the setpoint range
    // if the drop is below the setpoint the machine moves
    // and holds as long as it is within the setpoint range
    else if( motion_plan == 2 ){
        // within setpoint at higher end
        // fully skip any feed
        // this can quickly change to a short circuit
        // the step delay is used as a normal delay
        // even if the motion is skipped
        // to react fast it uses a fast delay in this case
        line.skip_feed = true;
        line.step_delay = 50;//max_feeds_micros[ Z_AXIS ];
    } else if( motion_plan == 1 ){
        // within setpoint at lower end
        line.step_delay = 50;//2* MAX( DEFAULT_STEP_DELAY_EDM, max_feeds_micros[ Z_AXIS ] );
        line.skip_feed = true; // temporary for testing ( 1 and 2 are both in setpoint. 1 = closer to the bottom setpoint while 2 = closer to the top setpoint )
    } else{
        // below setpoint
        line.step_delay = MAX( DEFAULT_STEP_DELAY_EDM, max_feeds_micros[ Z_AXIS ] );
        line.skip_feed = false;
    }
    }

    return _success;

}



void G_EDM_PLANNER::wire_line_end( Line_Config &line ){
    // wait until the feedback doesn't drop anymore 
    // to ensure the wire is set into the final line position
    int raw_vsense      = ui_controller.get_vsense_raw(); 
    int last_vsense_raw = raw_vsense;
    int rounds          = 100;
    int probes          = 0;
    // the raw vsense raises with lesser workload
    // if the raw vsense stops raising the eroding finished
    // even with no sparks the vsense will keep raising due to ecm effect
    // to prevent a super slow ecm raising it will compare the current vsense to the previous one
    // with a few percent added
    while( --rounds>0 ){
        if( 
            sys.abort 
            || sys_rt_exec_state.bit.motionCancel 
            || probes >= 4
         ){
            break;
        }

        if( float(raw_vsense) <= ( float(last_vsense_raw) + float(last_vsense_raw)*0.05 ) ){
            ++probes;
        } else{
            probes=0;
        }
        // still droping
        vTaskDelay(10);
        last_vsense_raw = raw_vsense;
        raw_vsense = ui_controller.get_vsense_raw();
    }
}





/**
  * Takes the target positions and creates the
  * step/direction bits
  */
bool G_EDM_PLANNER::move_line( float* target, Line_Config &line ){
    if( sys_rt_exec_state.bit.motionCancel ){
        return false;
    }

    String debug = String(position_history_index_current);
    if( _state.z_axis_is_up ){ debug += " z_axis_is_up "; }
    if( line.floating_enabled ){ debug += " enable_floating_z "; }
    if( line.enable_flushing ){ debug += " enable_flushing "; }
    if( short_circuit_retraction ){ debug += " short_circuit_retraction "; }
    //Serial.println( debug );
    debug="";
    print_target( target );

    line.last_motion_plan       = 0;
    line.setpoint_step_position = 0;
    line.failure_step_pos       = 0;
    line.step_bits              = 0;
    line.direction_bits         = 0;
    line.step_event_count       = 0;

    int32_t target_steps[MAX_N_AXIS], position_steps[MAX_N_AXIS];
    long    count    = 0;
    bool    _success = true;
    int current_step_delay = line.step_delay; // backup

    sys.gedm_stepper_disable_z = line.floating_enabled;

    /** Ensure Z axis does not overshoot the cutting limit and is 
        set to the current position for floating z motion **/
    memcpy(position_steps, sys_position, sizeof(sys_position));
    for( int axis=0; axis<N_AXIS; ++axis ) {
        line.line_math[axis].counter = 0.0;
        if( line.ignore_z_motion
            && axis == Z_AXIS 
        ){
            // overwrite the Z position with the current system position
            // to prevent step generation for z
            target_steps[axis] = lround( sys_position[Z_AXIS] );
        } else{
            target_steps[axis] = lround(target[axis] * axis_settings[axis]->steps_per_mm->get());
        }
        line.line_math[axis].steps    = labs(target_steps[axis] - position_steps[axis]);
        line.step_event_count         = MAX( line.step_event_count, line.line_math[axis].steps );
        line.line_math[axis].delta    = (target_steps[axis] - position_steps[axis]) / axis_settings[axis]->steps_per_mm->get();
        if (line.line_math[axis].delta < 0.0) {
            line.direction_bits |= bit(axis);
        }
    }


    if( line.step_event_count <= 0 ){
        if( sys.gedm_single_axis_drill_task ){
            line.step_event_count = 1; // dirty hackaround for now
        } else{
            return true;
        }
    }

    // set stepper direction pins
    motor_manager.motors_direction( line.direction_bits );

    /** the final loop to apply the move the line **/
    while(  count < line.step_event_count ){

        // reset defaults
        line.step_delay       = current_step_delay; // restore previous step delay
        line.step_bits        = 0;                         // reset step bits
        line.skip_floating    = false;
        line.skip_feed        = false;
        line.failure_step_pos = 0;// 3D floating specific; z position that triggered a short

        // insert pause on request
        if( sys.edm_pause_motion ){
            pause();
        }

        // default break conditions for abort,probe,limits
        if( 
            sys.abort 
            || sys_rt_exec_state.bit.motionCancel 
            || probe_check( line ) 
            || ( ! line.ignore_limit_switch && limits_get_state() )
         ){
            _success = false;
            break;
        }

        if( line.motion_plan_enabled ){

            if( sys.gedm_wire_gcode_task ){

                _success = process_wire( line );
                if( ! _success ){
                    break;
                }

            } else{

                if( line.enable_flushing ){
                    do_flush_if_needed();
                }

                _success = pre_process_floating( 0, line );

                if( ! _success ){
                    add_step_delay(30);
                    break;
                }

            }

        }


        /** If XY feed is skipped or in drill mode where no XY exists the round continues without stepping XY **/
        if( line.skip_feed || line.ignore_xy_motion ){
            add_step_delay( line.step_delay, false );
            if( sys.gedm_single_axis_drill_task ){
                if( 
                    no_workload_finish_reached()
                    || work_is_at_cutting_depth_limit()
                ){
                    sys.edm_process_finished = true;
                    count                    = line.step_event_count;
                    break;
                }
            }
            continue;
        }

        microseconds = ( uint64_t ) esp_timer_get_time();
        if( sys.gedm_wire_gcode_task && ! line.ignore_feed_limit ){
            if( line.step_delay < max_feeds_micros[ Z_AXIS ] ){
                line.step_delay = max_feeds_micros[ Z_AXIS ];
            }
        }


        bool delay_success = add_step_delay( line.step_delay, line.floating_enabled ); 

        // update the step position for each axis
        // this loops over all axis and does one single step
        for (int axis = 0; axis < N_AXIS; axis++) {
            line.line_math[axis].counter += line.line_math[axis].steps;
            if(line.line_math[axis].counter > line.step_event_count) { // "">" only on grbls code? may it be ">="
                line.step_bits |= bit(axis);
                line.line_math[axis].counter -= line.step_event_count;
                    if (line.direction_bits & bit(axis)) {
                        sys_position[axis]--;
                    } else {
                        sys_position[axis]++;
                    }
            }
        }
        // do the step
        //last_step_timestamp = ( uint64_t ) esp_timer_get_time();
        motor_manager.motors_step( line.step_bits );
        ++count;

    }

    if( _success && !short_circuit_retraction && sys.gedm_wire_gcode_task ){

        wire_line_end( line );

    }


    sys.gedm_stepper_disable_z = false; // restore

    if( line.setpoint_step_position != 0 ){
        // update the z history to a position without short circuits
        position_history[ position_history_index_current ][Z_AXIS] = lround( 1/axis_settings[Z_AXIS]->steps_per_mm->get()*line.setpoint_step_position );
    }

    return _success;

}






/** 
  * breaks are ignored in forward direction but prevent the history from moving back 
  * This is just a cheap and easy way to prevent M3/M4 up/downs from becoming a problem
  * Z axis is dispatched in floating operation and would not follow the UP/down path in the history
  * That would result in a crash. So easy way to prevent this it to add a break and 
  * stop the history from retractring further back
  * A break is basically just a position block with all positive coords
  * The only motion that uses positive targets is the homing motion
  * therefore while homing breaks are ignored
  * in normal operation there are only negative targets (all negative space)
  * Make sure your machine is set to all negative space!
  **/
void G_EDM_PLANNER::push_break_to_position_history(){
    float target[MAX_N_AXIS];
    for( int axis = 0; axis < MAX_N_AXIS; ++axis ){target[axis] = 1.0;}
    push_to_position_history( target, false, 0 );
}
/** used only for homing to ignore breaks while seeking the limit switches in positive direction **/
void G_EDM_PLANNER::set_ignore_breakpoints( bool ignore_break ){
    position_history_ignore_breakpoints = ignore_break;
}
bool G_EDM_PLANNER::position_history_is_break( float* target ){
    // positive positions are interpreted as invalid
    // only while homing they are allowed
    if( position_history_ignore_breakpoints ){
        return false;
    }
    bool is_history_break = false; 
    for( int i = 0; i < N_AXIS; ++i ){
        if( target[i] > 0.0 ){
            is_history_break = true;
            break;
        }
    }
    return is_history_break;
}
uint16_t G_EDM_PLANNER::push_to_position_history( float* target, bool override, int override_index ){
    // move one step forward
    if( ! override ){ position_history_get_next(); }
    int index = override ? override_index : position_history_index;
    memcpy( position_history[ index ], target, sizeof(target[0]) * N_AXIS );
    return index;
}
uint16_t G_EDM_PLANNER::position_history_get_previous(){
    if (position_history_index == 0) {
        position_history_index = POSITION_HISTORY_LENGTH;
    }
    position_history_index--;
    return position_history_index;
}
uint16_t G_EDM_PLANNER::position_history_get_next(){
    position_history_index++;
    if (position_history_index == POSITION_HISTORY_LENGTH) {
        position_history_index = 0;
    }
    return position_history_index;
}
uint16_t G_EDM_PLANNER::position_history_work_get_previous( bool peek ){
    uint16_t index = position_history_index_current;
    if (index == 0) {
        index = POSITION_HISTORY_LENGTH;
    }
    index--;
    if( ! peek ){
        position_history_index_current = index;
    }
    return index;
}
uint16_t G_EDM_PLANNER::position_history_work_get_next( bool peek ){
    uint16_t index = position_history_index_current;
    index++;
    if (index == POSITION_HISTORY_LENGTH) {
        index = 0;
    }
    if( ! peek ){
        position_history_index_current = index;
    }
    return index;
}
/**
  * In reverse mode this check if the previous work index is a allowed to be used
  * if the previous index is the real final index the history run a full cycle backwards
  * 
  * In forward mode it check if the next work index is the final index
  **/
bool G_EDM_PLANNER::position_history_is_at_final_index(){
    return position_history_index == position_history_index_current ? true : false;
}
bool G_EDM_PLANNER::future_position_history_is_at_final_index( bool reverse ){
    if( reverse ){
        uint16_t previous_w_index = position_history_work_get_previous( true );
        if( previous_w_index == position_history_index ){
            return true;
        } return false;
    }
    uint16_t next_w_index = position_history_work_get_next( true );
    if( next_w_index == position_history_index ){
        return true;
    } return false;
}
uint16_t G_EDM_PLANNER::get_current_work_index(){
    return position_history_index_current;
}

void G_EDM_PLANNER::history_backup_recover_target(){
    float* current_position = system_get_mpos();
    memcpy( history_recover_target, current_position, sizeof(current_position[0]) * N_AXIS );
    position_history_recover_index = position_history_index_current;
    recovered = false;
}
bool G_EDM_PLANNER::position_history_move_recover_trigger( Line_Config &line ){

    return false;

}
bool G_EDM_PLANNER::position_history_move_forward( bool no_motion, Line_Config &line ){
    // get the previous position object
    if( !position_history_is_at_final_index() ){
        position_history_work_get_next( false ); 
    }
    if( position_history_is_break( position_history[position_history_index_current] ) ){
        return true;
    }
    bool _success = move_line( position_history[position_history_index_current], line );

    return _success;
}

/** this function exits if a short is canceled and also changes the z axis position **/
bool G_EDM_PLANNER::position_history_move_back(){
    //float probe_pos = lround( 1/axis_settings[Z_AXIS]->steps_per_mm->get()*sys_probe_position_final[Z_AXIS] );
    // get the previous position object
    bool _success = true;
    // make sure it is not a break point
    bool previous_is_final    = future_position_history_is_at_final_index( true );
    uint16_t index_w_previous = position_history_work_get_previous( true ); // peek the previous index without changing the work index
    bool is_stop = ( 
        ( previous_is_final )
        || position_history_is_break( position_history[index_w_previous] ) // check if the previous index is a block position
    ) ? true : false;
    if( !is_stop ){
        position_history_work_get_previous( false );
    }

    Line_Config history_back_line;
    history_back_line.step_delay          = DEFAULT_STEP_DELAY_EDM;
    history_back_line.motion_plan_enabled = true;
    history_back_line.ignore_z_motion     = true;
    history_back_line.ignore_feed_limit   = true;

    _success = move_line( position_history[position_history_index_current], history_back_line );

    return _success;
}


/**
  * HISTORY  
  * This is the core of the history
  **/


void G_EDM_PLANNER::position_history_reset(){
    max_reverse_depth        = 2; // max two lines back in wire mode retractions
    has_reverse              = 0;

    position_history_index_current = 1; 
    position_history_index         = 0;
    position_history_is_between    = false;
    planner_is_synced              = false;

    memset(position_history, 0, sizeof(position_history[0]));
    float _target[MAX_N_AXIS];
    push_break_to_position_history();
    _state.z_axis_is_up = false;
    position_history_index_current = position_history_index;
}
bool G_EDM_PLANNER::position_history_sync( Line_Config &line ){

    // it still moves the full line back
    // or maybe repeats the line somehow
    sys.gedm_planner_sync       = true;
    short_circuit_retraction    = false;
    bool _success               = true;
    bool motion_ready           = false;
    bool current_is_last_block  = false;
    int  direction              = 0; // o = forward, 1 = backward, //2 = move to short trigger block
    int  last_direction         = 0; 
    bool enable_history         = line.enable_position_history; // history is only useful for wire edm    

    motion_ready = position_history_is_at_final_index();

    while( !motion_ready ){

        motion_ready = false;

        if( sys.abort || sys_rt_exec_state.bit.motionCancel || sys.gedm_stop_process ){ 
            _success = false;
            break; 
        }

        if( enable_history ){

            // default direction is forward
            // if history is enabled it can retract backwards
            // only used for 2D wire; 3D floating doesn't use history
            // Note: in a backward retraction the line backwards is canceled as soon as the short
            // circuit is canceled. It results in a success=false
            // the line was not fully finished and therefore returns false
            // but this is actually a success in cancelling the short circuit
            direction = _success 
                         ? get_success_direction( last_direction ) 
                         : get_failure_direction( last_direction );

        }

        // move in the wanted direction
        _success = process_direction( direction, line );
        last_direction = direction;
        current_is_last_block = position_history_is_at_final_index();

        // break conditions
        if( _success ){
            if( direction == 0 ){
                // this is the same for motion with and without history
                // if the last blocks was succesfull in forward direction
                // it is finished
                if( current_is_last_block ){
                    motion_ready = true;
                    break;
                }
            }
        } else {
            if( ! enable_history && current_is_last_block ){
                // if it is a non history motion and reached the last block
                // exit and return the success state
                // if hiostory is enabled continue the loop
                motion_ready = true;
                break;
            }
        }
    }
    //Serial.println("@all sync");
    sys.gedm_planner_sync      = 0;
    sys.gedm_recover_motion    = false;
    sys.gedm_retraction_motion = false;
    short_circuit_retraction   = false;
    recovered                  = true;
    // to be safe that history is synced
    // even after a hard motion cancel etc.
    position_history_index_current = position_history_index;
    return _success;
}

/**
  * 
  * Success and failure only refers to the last line
  * If a line was finished it is a success
  * if a line was not finished it is a failure
  * A backward motion to cancel short circuits don't need to run the full line
  * So a failure in a backward motion always indicates that the short was canceled before the line finished
  * 
  **/
int G_EDM_PLANNER::get_success_direction( int last_direction ){
    // last motion executed succesfull
    switch (last_direction){
        case 0:
            // last success motion was in forward direction
            // this can be a normal feed or a recover forward motion 
            // if this is still within a recovery it needs some extra checks
            // to keep it in recovery until at the initial position
            //Serial.println( "    Forward success" );
            return 0; // keep going forward
            break;
    
        case 1:
            // last success motion was backward
            // this motion was not enough to cancel a short circuit
            // a short was not canceled and needs more retraction
            //Serial.println("    Short not canceled");
            if( ++has_reverse > max_reverse_depth ){
                return 0; // force forward
            }
            return 1;
            break;
    }
    return 0;
}
int G_EDM_PLANNER::get_failure_direction( int last_direction ){
    // last motion failed
    switch (last_direction){
        case 0:
            // last failed motion was in forward direction
            // this indicates that a normal feed motion 
            // or a forward recover motion created a short
            // a recover motion follows a retraction with the goal to get back to the initial position
            //Serial.println( "    Forward shorted" );
            return 1; // no matter the details the next move is backwards/retraction
            break;

        case 1:
            // last failed motion in backward direction
            // this motion canceled a short circuit
            // there are no other options for a failed backward motion except the 
            // successfull cancelation of short circuits
            // the backward line was not fully drawn since the short was gone somewhere within the line
            //Serial.println("    Short canceled");
            vTaskDelay(1);
            return 0; // back to forward???
            break;
    }
    return 1;
}
bool G_EDM_PLANNER::process_direction( int direction, Line_Config &line ){
    bool _success = true;
    switch (direction){
        case 0:
            has_reverse = 0;
            return position_history_move_forward( false, line );
            break;
        case 1:
            sys.gedm_retraction_motion = true;
            short_circuit_retraction   = true;
            _success = position_history_move_back();
            sys.gedm_retraction_motion = false;
            short_circuit_retraction   = false;
            return _success;
            break;
    }
    return false;
}















void G_EDM_PLANNER::configure(){
    if( ! sys_probed_axis[Z_AXIS] ){
        sys_probe_position_final[Z_AXIS] = sys_position[Z_AXIS];
    }
    _state.z_axis_is_up    = false;
    is_paused              = false;
    gcode_round            = 1; // counter for the number of file repeats
    deepest_step_pos       = 0;
    last_workload_at       = sys_probe_position_final[Z_AXIS];
    no_workload_steps_to_finish = round( finish_after_mm * axis_settings[ Z_AXIS ]->steps_per_mm->get() );
    //flushing_total_steps  = round( flush_retract_mm * axis_settings[ Z_AXIS ]->steps_per_mm->get() );
    ui_controller.reset_flush_retract_timer();
    memset(total_mm_work, 0, sizeof(total_mm_work));
};
/**
  * return true if z moved down a given amount of mm without having a workload 
  * this should be used with care since noise can easily generate a spike that may be interpreted as workload!
  **/
bool G_EDM_PLANNER::no_workload_finish_reached(){
    if( sys_position[Z_AXIS] <= ( last_workload_at - no_workload_steps_to_finish ) ){
        return true;
    }
    return false;
}
/** 
  * set the cutting depth relative to work zero in mm 
  * 0.0 = no limit 
  * this function doesn't care if the limit set is out of range! #todo
  **/
void G_EDM_PLANNER::set_cutting_depth_limit( float _mm ){
    z_cutting_depth = _mm;
    if( _mm == 0.0 ){
        cutting_depth_step_final = 0; // disabled   
        return; 
    }
    int cutting_depth_steps  = round( _mm * axis_settings[ Z_AXIS ]->steps_per_mm->get() ); 
    cutting_depth_step_final = sys_probe_position_final[Z_AXIS] - cutting_depth_steps;
    cutting_depth_mpos_final = system_convert_axis_steps_to_mpos( cutting_depth_step_final, Z_AXIS );
}
bool G_EDM_PLANNER::work_is_at_cutting_depth_limit(){
    if( cutting_depth_step_final != 0 ){
        if( sys_position[Z_AXIS] <= cutting_depth_step_final ){
            return true;
        }
    }
    return false;
}


void G_EDM_PLANNER::reset_planner_state(){
    _state.z_axis_is_up = false;
    set_ignore_breakpoints( false );
    set_cutting_depth_limit( 0.0 );
}
void G_EDM_PLANNER::do_flush_if_needed(){
    if( ui_controller.check_if_time_to_flush() ){
        float flush_offset_mm = 1.0 / axis_settings[Z_AXIS]->steps_per_mm->get() * float( flush_offset_steps );
        retraction( Z_AXIS, flush_retract_mm, flush_offset_mm, false, disable_spark_for_flushing );
        ui_controller.reset_flush_retract_timer();
    } 
}





/** 
  * Takes the target and line config, pushes the line to the history and syncs the history
  **/
bool G_EDM_PLANNER::process_stage( float* target, Line_Config &line ){
    // push the target to the history buffer
    uint16_t index = push_to_position_history( target, false, 0 );
    bool _success = position_history_sync( line );
    if( ! _success ){
        float* current_position = system_get_mpos();
        memcpy(target, current_position, sizeof(current_position[0])*MAX_N_AXIS );
        push_to_position_history( target, true, index );
    }
    return _success;
}

/** 
  * This is the main gateway for lines
  * All normal lines are passed through this except for some special motions
  **/
uint8_t G_EDM_PLANNER::plan_history_line( float* target, plan_line_data_t* pl_data ) {


    bool _success = true;
    float* current_position;
    float __target[MAX_N_AXIS];

    // exit clean on aborts and motionstops
    if( 
        sys.abort 
        || sys_rt_exec_state.bit.motionCancel 
        || sys.edm_process_finished 
    ){
        idle_timer = millis();
        current_position = system_get_mpos();
        memcpy(target, current_position, sizeof(current_position[0])*MAX_N_AXIS );
        gc_sync_position();
        sys.gedm_planner_line_running = false;
        return false;
    }

    sys.state                     = State::Cycle;
    sys.gedm_planner_line_running = true; 

    Line_Config line;

    // change the step delay / speed for this line; This may be overwritten in the process and is just a default value
    if( pl_data->step_delay ){
        line.step_delay = pl_data->step_delay;
    } else{
        line.step_delay = DEFAULT_STEP_DELAY_RAPID;
    }

    // set the default line configuration
    line.ignore_limit_switch = pl_data->use_limit_switches ? false : true; // defaults is no limits except for homing!

    if( 
        ! pl_data->motion.systemMotion && // not a system motion //maybe deprecated here
        ( 
            sys.gedm_floating_z_gcode_task     // is a 3D floating gcode line
            || sys.gedm_single_axis_drill_task // or a drill/sinker single line
            || sys.gedm_wire_gcode_task
        ) 
    ){

        // this line is within the EDM process
        // it can be a travel motion or a normal work motion
        // M3/M4 up/down is not passed through this function
        // so this can only be a travel or work movement
        // G0 is a rapid move and normally only the G0 travels should 
        // be flagged as rapids in the process
        // and normally there should have been a M3 / Z up command before
        // if it is a 3D floating gcode process

        if( sys.gedm_floating_z_gcode_task ){

            if( pl_data->motion.rapidMotion == 1 ){

                // should be a travel move
                line.ignore_z_motion = true; // just to be safe ignore z

            } else {

                // should be a work move
                line.ignore_z_motion     = true;
                line.floating_enabled    = true;
                line.motion_plan_enabled = true;
                line.is_work_motion      = true;
                line.step_delay          = DEFAULT_STEP_DELAY_EDM;

            }

        } else if( sys.gedm_single_axis_drill_task ){

            line.ignore_z_motion     = true;
            line.floating_enabled    = true;
            line.motion_plan_enabled = true;
            line.is_work_motion      = true;
            line.step_delay          = DEFAULT_STEP_DELAY_EDM;

            line.enable_flushing  = true;
            line.ignore_xy_motion = true;

        } else if( sys.gedm_wire_gcode_task ){

            //Serial.println("Wire line");

            line.enable_position_history = true;

            line.step_delay          = DEFAULT_STEP_DELAY_EDM;
            line.ignore_z_motion     = true;
            line.motion_plan_enabled = true;
            line.is_work_motion      = true; // Note: In wire mode all motions in the gcode file are work motion
                                             // rapid moves are considered work motions too
                                             // everything needed before the file is done via the UI



        }

        if( simulate_gcode ){
            line.ignore_z_motion = true;
        }

    } 

    _success = process_stage( target, line );

    gc_sync_position(); 
    idle_timer                    = millis();
    sys.gedm_planner_line_running = false;

    return _success;
}



/**
  * 
  * CUSTOM MOTIONS
  * 
  * These are custom motions that are used within the process
  * and are not meant to be used directly!
  * They may not work if called outside of a running job 
  * 
  **/

/**
  * Used to retract Z for flushing
  **/
void G_EDM_PLANNER::retraction( int stepper, float travel_mm, float offset_mm, bool oneway, bool disable_pwm ){
    if( disable_pwm ){ ui_controller.pwm_off(); }
    // get the current position and save the position for backup usage
    float* current_position = system_get_mpos();
    float target[MAX_N_AXIS];
    float backup_position[MAX_N_AXIS];
    memcpy( target, current_position, sizeof(current_position[0]) * N_AXIS );
    memcpy( backup_position, current_position, sizeof(current_position[0]) * N_AXIS );
    float probe_position_mpos = system_convert_axis_steps_to_mpos( sys_probe_position_final[stepper], stepper);
    float __target = probe_position_mpos+travel_mm;
    if( __target > DEFAULT_HOMING_PULLOFF*-1.0 ){
        __target = DEFAULT_HOMING_PULLOFF*-1.0;
        if( target[Z_AXIS] > __target ){
            __target = target[Z_AXIS];
        }
    }
    target[Z_AXIS] = __target;

    Line_Config retraction_line; // default config

    move_line( target, retraction_line );
    // re-enable PWM and exit if this was a one way motion
    if( disable_pwm ){ 
        ui_controller.pwm_on(); 
    }
    backup_position[Z_AXIS] += offset_mm;
    // return to offset position
    move_line( backup_position, retraction_line );
    return; 
}
/**
  * Used to move Z while waiting for reprobe confirmation
  **/
bool G_EDM_PLANNER::z_axis_move_mm( float mm ){
    if( mm == 0.0 ){ return false; }
    float* current_position = system_get_mpos();
    float target[MAX_N_AXIS];
    memcpy( target, current_position, sizeof(current_position[0]) * N_AXIS );
    target[Z_AXIS] += mm;
    if( limitsCheckTravel( target ) ){ return false; }
    Line_Config move_mm_line; // default config
    bool _success = move_line( target, move_mm_line );
    return _success;
}


/** 
  * called between lines! curently in the protocol loop 
  **/
bool G_EDM_PLANNER::reprobing_routine(){
    // turn off sd card readings
    bool axis_was_up = _state.z_axis_is_up;

    // move axis up
    if( ! _state.z_axis_is_up ){
        z_axis_up();
    }

    float* current_position = system_get_mpos();
    float target[MAX_N_AXIS];
    float backup_position[MAX_N_AXIS];
    memcpy( target,          current_position, sizeof(current_position[0]) * N_AXIS );
    memcpy( backup_position, current_position, sizeof(current_position[0]) * N_AXIS );
    if( sys.gedm_probe_position_x == 0 && sys.gedm_probe_position_y == 0 ){
        // no probe points set
        // falling back to 0,0 workposition
        float* work_position = current_position;// system_get_mpos();
        float work_position_copy[MAX_N_AXIS];
        mpos_to_wpos( work_position );
        memcpy( work_position_copy, work_position, sizeof(work_position[0]) * N_AXIS );
        //target[X_AXIS] = gc_state.position[X_AXIS] - gc_state.coord_offset[X_AXIS] - work_position_copy[X_AXIS];
        //target[Y_AXIS] = gc_state.position[Y_AXIS] - gc_state.coord_offset[Y_AXIS] - work_position_copy[Y_AXIS];
        if( work_position_copy[X_AXIS] < 0 ){
            target[X_AXIS] += (work_position_copy[X_AXIS]*-1);
        } else{
            target[X_AXIS] -= work_position_copy[X_AXIS];
        }
        if( work_position_copy[Y_AXIS] < 0 ){
            target[Y_AXIS] += (work_position_copy[Y_AXIS]*-1);
        } else{
            target[Y_AXIS] -= work_position_copy[Y_AXIS];
        }
    } else{
        target[X_AXIS] = sys.gedm_probe_position_x;
        target[Y_AXIS] = sys.gedm_probe_position_y;
    }
    sys.gedm_reprobe_motion = true;

    /** 
      * Move to XY probe position with axis up 
      **/
    Line_Config reprobe_line; // default config

    bool _success = move_line( target, reprobe_line ); 
    sys.gedm_reprobe_motion = false;
    is_between_steps        = true;
    // wait for reprobe confirmation
    sys.gedm_reprobe_block = true;
    while( sys.gedm_reprobe_block || sys.edm_pause_motion ){
        if( sys.abort || sys_rt_exec_state.bit.motionCancel ){
            break;
        }
        // wait for confirmation
        vTaskDelay(10);
    }
    // since this routine is called from within a planner line request
    // the gc_position is not updated yet and still thinks it is a the previous position
    // sync the current position
    gc_sync_position();
    // send the G10 command to reset the new Z position
    char command_buf[20];
    sprintf(command_buf,"G10 P1 L20 Z0\r" );
    execute_line( command_buf );
    // update some variables and recalculate the cutting depth limit etc.
    // calculate the offset from the old probe and workload positions
    int offset_steps = MAX( 0, sys_probe_position_final[Z_AXIS] + (last_workload_at*-1) );
    sys_probe_position_final[Z_AXIS] = sys_position[Z_AXIS];
    last_workload_at                 = sys_probe_position_final[Z_AXIS] - offset_steps;
    set_cutting_depth_limit( z_cutting_depth );
    // move Z axis up to the travel offset position
    _state.z_axis_is_up = false;
    z_axis_up();
    is_between_steps = false;
    // write the new z position to the backup position
    // and move back to the initial XY position
    current_position = system_get_mpos();
    memcpy( target, current_position, sizeof(current_position[0]) * N_AXIS );
    backup_position[Z_AXIS] = target[Z_AXIS];
    sys.gedm_reprobe_motion = true;
    /**
      * Reprobe move
      **/
    _success = move_line( backup_position, reprobe_line ); // again without adding it to the history
    sys.gedm_reprobe_motion = false;
    sys.gedm_insert_probe   = false;
    force_redraw            = true;
    is_between_steps        = true;  
    // move z back down if ti was down
    if( ! axis_was_up ){
        z_axis_down();
    }
    return _success;
}

/**
  * Used to move Z up via M3 command
  **/
void G_EDM_PLANNER::z_axis_up(){
    if( sys.gedm_wire_gcode_task ){
        return;
    }
    if( simulate_gcode || _state.z_axis_is_up ){
        ui_controller.pwm_off();
        push_break_to_position_history();
        _state.z_axis_is_up = true;
        return;
    }
    Line_Config zup_line; // default config

    float* current_position = system_get_mpos();
    float target[MAX_N_AXIS];
    memcpy( target, current_position, sizeof(current_position[0]) * N_AXIS );
    // get the probe position in mpos
    float probe_position_mpos = system_convert_axis_steps_to_mpos( sys_probe_position_final[Z_AXIS], Z_AXIS);
    float mm_up_wanted        = abs( MIN( 0, target[Z_AXIS] - probe_position_mpos ) ) + electrode_up_offset_mm;

    float __target = probe_position_mpos+electrode_up_offset_mm;
    if( __target > DEFAULT_HOMING_PULLOFF*-1.0 ){
        __target = DEFAULT_HOMING_PULLOFF*-1.0;
        if( target[Z_AXIS] > __target ){
            __target = target[Z_AXIS];
        }
    }

    target[Z_AXIS] = __target;//get_mm_up_possible( mm_up_wanted );

    ui_controller.pwm_off();

    process_stage( target, zup_line );

    _state.z_axis_is_up    = true;
    push_break_to_position_history(); // set a stop block to the history to prevent it from retracting there
}
/**
  * Used to move Z down via M4 command
  **/
void G_EDM_PLANNER::z_axis_down(){
    if( sys.gedm_wire_gcode_task ){
        return;
    }
    if( simulate_gcode ){
        _state.z_axis_is_up = false;
        return;
    }
    Line_Config zdown_line;

    // backup some stuff
    zdown_line.step_delay = DEFAULT_STEP_DELAY_EDM_REPROBE; // lower the speed

    float* current_position = system_get_mpos();
    float target[MAX_N_AXIS];
    memcpy( target, current_position, sizeof(current_position[0]) * N_AXIS );
    float travel_limit = 0.0;
    float probe_mpos = system_convert_axis_steps_to_mpos( sys_probe_position_final[Z_AXIS], Z_AXIS);
    // this is just to set a rough travel target
    // the real target is determined through probing
    if( last_workload_at != 0.0 && last_workload_at != sys_probe_position_final[Z_AXIS] ){
        // move to last workload if it is not the initial probe point 
        travel_limit = system_convert_axis_steps_to_mpos( last_workload_at, Z_AXIS);
    } else if( z_cutting_depth != 0.0 ){
        // if a cutting depth is set move until the cuttin depth is reached
        //travel_limit = probe_mpos - z_cutting_depth; // maybe dangerous to do
        travel_limit = probe_mpos;
    } else{
        // no travel limit set and last work load is probe position
        // this can be a reprobe position too where there is already
        travel_limit = probe_mpos;
    }
    target[Z_AXIS] = travel_limit;
    // reenable PWM
    ui_controller.pwm_on();
    // enable probing
    set_probe_direction( false );
    sys_probe_state = Probe::Active;
    probe_touched   = false;

    _state.z_axis_is_up = false;

    process_stage( target, zdown_line );
    
    // it does probe but it keeps some limits and sometimes doesn't touch the probe
    // this limti is to keep the process a little even
    // turn off probe after the line is finished
    // probe disables the pwm on success and it needs to be reactivated
    sys_probe_state = Probe::Off;
    //vTaskDelay(1);
    ui_controller.pwm_on();
    push_break_to_position_history(); // set a stop point after electrode moved down
}



/**
 * 
 * 
 * Todo:
 * Test 3d floating simulation
 * 
 * Test 3D floating
 * Test 3d floating reprobing
 * Test 3d floating up/down
 * 
 * 2D wire history redo
 * 
 * restore feedrate for normal milling operation
 * 
 * 
 * force 2d probing in wire mode
 * use max feed rate for z as max feed for xy in wire mode 
 * 
*/
