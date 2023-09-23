#pragma once
/*
#  ██████        ███████ ██████  ███    ███  
# ██             ██      ██   ██ ████  ████  
# ██   ███ █████ █████   ██   ██ ██ ████ ██ 
# ██    ██       ██      ██   ██ ██  ██  ██ 
#  ██████        ███████ ██████  ██      ██ 
#   
*/ 
#include <cstdint>
#include <Config.h>
#include <config/definitions.h>
#include <driver/rmt.h>
#include <NutsBolts.h>

class G_EDM_STEPPER {

    public:

        G_EDM_STEPPER(uint8_t axis_index, uint8_t step_pin, uint8_t dir_pin);
        void init();
        bool set_homing_mode(bool isHoming);
        int set_direction(bool); // returns the number of microseconds added after a dir change
        void step();
        void unstep();
        void read_settings();
        void init_step_dir_pins();
        void set_disable(bool);
        void steps_down(int steps);
        void steps_up(int steps);
        void restore_grbl(void);
        void move_to( float mm, int *counter );
        
        void set_steps_per_mm( int steps);
        int get_steps_per_mm( void );

        void set_ms1_pin( int pin);
        void set_ms2_pin( int pin);
        void set_fixed_microsteps( bool _fixed_microsteps );
        void set_microsteps( int _microsteps);
        void set_microsteps_travel( int microsteps );
        void set_microsteps_edm( int microsteps );

        void move_to_previous_fullstep( void );
        void move_to_next_fullstep( void );
        void change_microsteps( int ms );
        void configure_microsteps( void );
        bool step_position_is_at_fullstep( void );
        void calc_microsteps_per_rev(void);

        int get_step_delay_for_speed( float mm_min );


    protected:
        uint8_t _axis_index;       // X_AXIS, etc
        uint8_t _dual_axis_index;  // 0 = primary 1=ganged

        int steps_per_mm;

        rmt_channel_t _rmt_chan_num;
        bool    _invert_step_pin;
        bool    _invert_dir_pin;
        uint8_t _step_pin;
        uint8_t _dir_pin;
        uint8_t _disable_pin;
        int _direction;

        int ms1_pin; 
        int ms2_pin; 
        bool fixed_microsteps;
        int microsteps;
        int microsteps_travel;
        int microsteps_edm;



    private:
        static rmt_channel_t get_next_RMT_chan_num();
        static rmt_item32_t  rmtItem[2];
        static rmt_config_t  rmtConfig;
        bool grbl_direction_state; // used to restore the wanted grbl state after we are done
};