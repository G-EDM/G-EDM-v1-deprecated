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
#include "GCode.h"
#include "shared.h"


/*
typedef struct {
    int   operation_mode;       // 0=defailt, 1=3D floating_edm_engraver, 2=drill/sinker, 3=2d wire
    bool  ignore_limit_switch;  // grbl has a background task running for limits, but the new planner is not async and while homing it needs a custom limits check
    bool  ignore_xy_motion;     // used in drill/sinker mode to fully skip xy motion
    bool  overwrite_z_position; // if true the line planner will set the current position as the target position to not produce any steps
    bool  enable_floating;
    bool  enable_position_history;
    int   current_step_delay;
    bool  enable_flushing;
    bool  is_custom_motion; // used to tell the line planner to skip some functions like reprobing, flushing etc. custom motion are up/down/flushing/reprobing etc
} planner_config;
*/
typedef struct {
  float delta   = 0;
  long  steps   = 0;
  long  counter = 0;
} Axis_Direct;

typedef struct {
    int     tmp                     = 0; // used for random stuff
    bool    ignore_limit_switch     = 1;  // grbl has a background task running for limits, but the new planner is not async and while homing it needs a custom limits check
    bool    ignore_xy_motion        = 0;  
    bool    ignore_z_motion         = 0;
    bool    ignore_feed_limit       = 0;
    bool    overwrite_z_position    = 0;  // if true the line planner will set the current position as the target position to not produce any steps
    bool    enable_position_history = 0;
    bool    enable_flushing         = 0;
    bool    skip_feed               = 0; 
    bool    skip_floating           = 0;
    bool    motion_plan_enabled     = 0;
    bool    floating_enabled        = 0;
    bool    is_work_motion          = 0;
    int     step_delay              = DEFAULT_STEP_DELAY_RAPID;
    int     last_motion_plan        = 0;
    int     setpoint_step_position  = 0;
    int     failure_step_pos        = 0;
    long    step_event_count        = 0;
    int     line_has_first_contact  = 0;
    uint8_t direction_bits          = 0;
    uint8_t step_bits               = 0;
    Axis_Direct line_math[N_AXIS];
} Line_Config;


static const uint16_t LINE_HISTORY_LENGTH = 4096;
static const uint16_t POSITION_HISTORY_LENGTH = 1024;

typedef struct {
    bool  z_axis_is_up;
} planner_state;


class G_EDM_PLANNER{

    private:

      float get_mm_up_possible( float mm_up_wanted );

      planner_state _state;

      float total_mm_work[MAX_N_AXIS];



      void stepper_step_custom( int axis, bool dir, int _step_delay );

      uint64_t last_step_micros[MAX_N_AXIS];
      uint64_t last_step_timestamp;
      uint64_t microseconds;

      bool short_circuit_retraction;
      bool short_circuit_recover;
      int max_reverse_depth;
      int has_reverse;

      int32_t* recover_target;
      uint16_t short_trigger_target_index;
      bool recovered;

      plan_line_data_t  default_plan_data;
      plan_line_data_t* default_pl_data;

      float    history_recover_target[MAX_N_AXIS];
      uint16_t position_history_recover_index;
      float    position_history[ POSITION_HISTORY_LENGTH ][MAX_N_AXIS];
      uint16_t position_history_index;
      uint16_t position_history_index_current;
      uint16_t position_history_final_index; 
      bool     position_history_is_between;
      bool     position_history_is_forward;
      bool     planner_is_synced;
      bool     position_history_ignore_breakpoints;
      uint16_t position_history_recover_fail_count;
      bool     history_is_recovering;

      bool move_line( float* target, Line_Config &line );
      bool process_stage( float* target, Line_Config &line );

      uint16_t push_to_position_history( float* target, bool override, int override_index );
      uint16_t position_history_get_previous( void ); // used by the ringbuffer internally
      uint16_t position_history_get_next( void );     // used by the ringbuffer internally
      uint16_t position_history_work_get_previous( bool peek = false ); // used by the user
      uint16_t position_history_work_get_next( bool peek = false );     // used by the user
      bool     position_history_is_break( float* target );
      bool     future_position_history_is_at_final_index( bool reverse = false );
      bool     position_history_is_at_final_index( void );
      bool     position_history_move_back( void );
      bool     position_history_move_forward( bool no_motion, Line_Config &line );
      bool     position_history_move_recover_trigger( Line_Config &line );
      bool     position_history_change_history( void );
      void     history_backup_recover_target( void );
      bool     position_history_move_to_custom( void );
      bool     current_position_is_at_target( void );
      int      get_success_direction( int last_direction );
      int      get_failure_direction( int last_direction );
      bool     process_direction( int direction, Line_Config &line );
      bool     probe_check( Line_Config &line );

      int last_motion_plan;
      bool is_paused;

      int    last_workload_at;
      int    travel_in_first_pass; // monitor the travel of Z in the first pass in steps
      int    deepest_step_pos;         // deepest travel of z into the work piece in the first pass
      bool   enable_wire_mode;
      int    step_limit;
      int    no_workload_steps_to_finish;
      int    flushing_total_steps;
      bool   stop_on_contact;
      int    cutting_depth_step_final;
      float  cutting_depth_mpos_final;
      float  z_cutting_depth;
      int    gcode_round;
      bool init_ready;
      void pause( void );
      void resume( void );
      void prepare( float* target );
      bool add_step_delay( int _step_delay, bool enable_motion_plan = false );
      bool cutting_depth_reached( void );
      bool no_workload_finish_reached( void );
 
    public:
      G_EDM_PLANNER( void );
      void reset_planner_state( void );
      void position_history_reset( void );
      void push_break_to_position_history( void );
      void push_current_mpos_to_position_history( void );
      int position_history_undo( int count );
      bool position_history_sync_new( void );
      bool position_history_sync( Line_Config &line );
      uint16_t get_current_work_index( void );
      void set_ignore_breakpoints( bool ignore_break );
      void update_max_feed( void );
      bool is_ready( void );
      void configure( void );
      void cancel_xyz_short( void );
      bool get_is_paused( void );
      void next_gcode_round( void );
      void set_cutting_depth_limit( float _mm );
      bool work_is_at_cutting_depth_limit( void );
      void do_flush_if_needed( void );
      uint8_t plan_history_line( float* target, plan_line_data_t* pl_data );
      bool floating_edm_engraver( int _step_delay, Line_Config &line );
      bool pre_process_floating( int _step_delay, Line_Config &line );
      bool process_wire( Line_Config &line );
      void unshort_hard( Line_Config &line );
      void wire_line_end( Line_Config &line );

      void retraction( int stepper, float travel_mm, float offset_mm, bool oneway, bool disable_pwm );
      void z_axis_up();   // acts like a pen up to change the toolpath (M3 GCode) It uses the probe position + the offset
      void z_axis_down(); // acts like pen down after a change of the toolpath (M4 GCode)
      bool z_axis_move_mm( float mm );
      bool reprobing_routine( void );
      int get_current_round( void );
      float get_total_travel_from_probe_position( void );

};

extern G_EDM_PLANNER planner;