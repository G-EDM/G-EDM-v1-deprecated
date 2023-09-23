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
#include "api.h"

#include "sd_card/filehandler.h"
#include "Report.h"
#include "GCode.h"

namespace api{

    int32_t get_step_position( int axis ){
      //const int X_AXIS = 0;  // Axis indexing value.
      //const int Y_AXIS = 1;
      //const int Z_AXIS = 2;
      return sys_position[axis];
    }
    void push_cmd(const char *text, bool block_until_idle ){ 
        WebUI::inputBuffer.push(text); 
        vTaskDelay(50);
        if( block_until_idle ){
            vTaskDelay(50);
            block_while_not_idle();
        }
    }
    void lock_while_z_moves(){}
    void unlock_machine(){
        /** enable steppers and unlock machine state**/
        push_cmd("$X\r");
        vTaskDelay(50);
    }

    bool machine_is_idle(){
        return ( ( sys.state == State::Alarm || sys.state == State::Idle ) && ! sys.gedm_planner_line_running) ? true : false;
    }
    void block_while_not_idle(){
        while( ! machine_is_idle() ){
            if( sys.abort || sys.gedm_stop_process || sys_rt_exec_state.bit.motionCancel ){
                break;
            }
            vTaskDelay(50);
        }
    }
    void probe_block(){
        idle_timer = millis();
        while( ! sys.probe_succeeded || sys_probe_state == Probe::Active ){
            if( sys.gedm_stop_process || sys.abort || sys_rt_exec_state.bit.motionCancel ){ break; }
            vTaskDelay(500);
        }
        idle_timer = millis();
    }
    void block_while_homing( int axis_index ){
        while( ! sys_axis_homed[axis_index] || sys.gedm_planner_line_running ){
            if( sys.abort || sys_rt_exec_state.bit.motionCancel ){ break; }
            vTaskDelay(10);
        }
        if( machine_is_fully_homed() ){
            //push_cmd("G90 G54 X-30 Y-30\r\n");
            push_cmd("G28.1\r\n");                 // set home position
        }
        block_while_not_idle();
    }



    void cancel_running_job(){
      sys_rt_exec_state.bit.motionCancel = true;
      execute_realtime_command(Cmd::Reset);
      filehandler.closeFile();
    }


    void show_positions(){
      float* current_position = system_get_mpos();
      for( int i = 0; i < 3 ; ++i ){
        //Serial.println( "MPos-" + String(i) + ": " + String( current_position[i] ) );
      }
      mpos_to_wpos(current_position);
      for( int i = 0; i < 3; ++i ){
        //Serial.println( "WPos-" + String(i) + ": " + String( current_position[i] ) );
      }
    }

    float* get_wpos(){
        float* current_position = system_get_mpos();
        mpos_to_wpos(current_position);
        return current_position;
    }


    bool machine_is_fully_homed(){

        bool fully_homed = true;

        for( int i = 0; i < N_AXIS; ++i ){
            if( i == Z_AXIS && z_no_home ){
                // skip Z if homing is not wanted. Electrode wears and if it starts to get shorter homing may be a problem.
                continue;
            }
            if( ! sys_axis_homed[i] ){
                fully_homed = false;
            }
        }

        return fully_homed;

    }




    void home_machine(){
        if( ! z_no_home ){
            home_z();
        } 
        home_x();
        home_y();
        block_while_not_idle();
    }
    void home_x(){
        sys_axis_homed[X_AXIS] = false;
        push_cmd( "$HX\r" );
        block_while_homing( X_AXIS );
        vTaskDelay(500);
    }
    void home_y(){
        sys_axis_homed[Y_AXIS] = false;
        push_cmd( "$HY\r" );
        block_while_homing( Y_AXIS );
        vTaskDelay(500);
    }
    void home_z(){
        sys_axis_homed[Z_AXIS] = false;
        push_cmd( "$HZ\r" );
        block_while_homing( Z_AXIS );
        vTaskDelay(500);
    }

    void force_home_z(){ // don't use!
        if( sys_position[Z_AXIS] <= (DEFAULT_HOMING_PULLOFF*-1) * DEFAULT_Z_STEPS_PER_MM ){
            return;
        }
        sys_position[Z_AXIS] = (DEFAULT_HOMING_PULLOFF*-1) * DEFAULT_Z_STEPS_PER_MM; 
        gc_sync_position();
    }


    void reset_probe_points(){
        for( int i = 0; i < N_AXIS; ++i ){
            sys_probed_axis[i] = false;
        }
    }
    void set_current_position_as_probe_position(){
        sys.probe_succeeded = true;
        memcpy(sys_probe_position, sys_position, sizeof(sys_position));
        for( int i = 0; i < N_AXIS; ++i ){
            sys_probed_axis[i]          = true;
            sys_probe_position_final[i] = sys_probe_position[i];
        }
        push_cmd("G91 G10 P1 L20 X0 Y0 Z0\r\n");
        vTaskDelay(500);
    }
    bool machine_is_fully_probed(){
        bool is_fully_probed = true;
        for( int i = 0; i < N_AXIS; ++i ){
            if( ! sys_probed_axis[i] ){
                is_fully_probed = false;
            }
        }
        return is_fully_probed;
    }
    void show_probe_points(){
        for( int i = 0; i < N_AXIS; ++i ){
            //Serial.println( String(i) + " probed at: " + String( sys_probe_position_final[i] ) );
        }
    }


    void set_reprobe_points(){
        float* current_position = system_get_mpos();
        float position_copy[MAX_N_AXIS];
        memcpy(position_copy, current_position, sizeof(current_position[0]) * N_AXIS);
        set_reprobe_point_x( position_copy[ X_AXIS ] );
        set_reprobe_point_y( position_copy[ Y_AXIS ] );
    }
    void set_reprobe_point_x( float x ){
        sys.gedm_probe_position_x = x;
    }
    void set_reprobe_point_y( float y ){
        sys.gedm_probe_position_y = y;
    }


    /** probe inside a hole or pocket and find the center position **/
    void probe_xy_center( void ){



    }


    void jog_axis( float mm, const char *axis_name, int speed ){
        if(  sys.gedm_planner_line_running ){
          sys_rt_exec_state.bit.motionCancel = true;
          while( sys.gedm_planner_line_running ){
            vTaskDelay(1);
            sys_rt_exec_state.bit.motionCancel = true;
          }
        }
        block_while_not_idle();
        sys_rt_exec_state.bit.motionCancel = false;
        /** cancel current jog motion if there is any **/
        char command_buf[40];
        sprintf(command_buf,"$J=G91 G21 %s%.5f F%d\r\n", axis_name, mm, speed);    
        /** push new jog command to input buffer **/
        push_cmd( command_buf );
    }
    void jog_up( float mm ){
        jog_axis( mm, "Z", Z_AXIS_JOG_SPEED );
    }
    void jog_down( float mm ){
        jog_axis( (mm*-1), "Z", Z_AXIS_JOG_SPEED );
    }
    void jog_forward( float mm ){
        jog_axis( mm, "Y", X_AXIS_JOG_SPEED );
    }
    void jog_back( float mm ){
        jog_axis( (mm*-1), "Y", X_AXIS_JOG_SPEED );
    }
    void jog_left( float mm ){
        jog_axis( mm, "X", X_AXIS_JOG_SPEED );
    }
    void jog_right( float mm ){
        jog_axis( (mm*-1), "X", X_AXIS_JOG_SPEED );
    }



};