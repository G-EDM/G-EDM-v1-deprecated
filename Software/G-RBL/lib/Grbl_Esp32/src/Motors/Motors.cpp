/*
	Motors.cpp
	Part of Grbl_ESP32
	2020 -	Bart Dring
	Grbl is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.
	Grbl is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.
	You should have received a copy of the GNU General Public License
	along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
	TODO
		Make sure public/private/protected is cleaned up.
		Only a few Unipolar axes have been setup in init()
		Get rid of Z_SERVO, just reply on Z_SERVO_PIN
		Class is ready to deal with non SPI pins, but they have not been needed yet.
			It would be nice in the config message though
	Testing
		Done (success)
			3 Axis (3 Standard Steppers)
			MPCNC (ganged with shared direction pin)
			TMC2130 Pen Laser (trinamics, stallguard tuning)
			Unipolar
		TODO
			4 Axis SPI (Daisy Chain, Ganged with unique direction pins)
	Reference
		TMC2130 Datasheet https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC2130_datasheet.pdf
    
    2023 - Roland Lautensack (G-EDM) This file was heavily edited and may no longer be compatible with the default grbl

*/



#include "Motors/Motors.h"
#include "gedm_motor/gedm_stepper.h"

#include "Grbl.h"

bool sys_axis_homed[MAX_N_AXIS]; 

G_EDM_STEPPER* myMotor[MAX_AXES];


G_STEPPER_MANAGER motor_manager = G_STEPPER_MANAGER();

G_STEPPER_MANAGER::G_STEPPER_MANAGER(){}

void G_STEPPER_MANAGER::init() {

    ignore_disable = false; 
    n_axis         = N_AXIS;

    /** create the steppers **/
    myMotor[X_AXIS] = new G_EDM_STEPPER(X_AXIS, X_STEP_PIN, X_DIRECTION_PIN);
    myMotor[Y_AXIS] = new G_EDM_STEPPER(Y_AXIS, Y_STEP_PIN, Y_DIRECTION_PIN);
    myMotor[Z_AXIS] = new G_EDM_STEPPER(Z_AXIS, Z_STEP_PIN, Z_DIRECTION_PIN);
    
    /** stepper configuration **/

    // X Axis
    myMotor[X_AXIS]->set_microsteps(        GEDM_DEFAULT_X_MICROSTEPS );
    myMotor[X_AXIS]->set_microsteps_travel( GEDM_DEFAULT_X_MICROSTEPS );
    myMotor[X_AXIS]->set_microsteps_edm(    GEDM_DEFAULT_X_MICROSTEPS );
    myMotor[X_AXIS]->set_fixed_microsteps(  true );
    myMotor[X_AXIS]->set_steps_per_mm(      DEFAULT_X_STEPS_PER_MM );

    // Y Axis
    myMotor[Y_AXIS]->set_microsteps(        GEDM_DEFAULT_Y_MICROSTEPS );
    myMotor[Y_AXIS]->set_microsteps_travel( GEDM_DEFAULT_Y_MICROSTEPS );
    myMotor[Y_AXIS]->set_microsteps_edm(    GEDM_DEFAULT_Y_MICROSTEPS );
    myMotor[Y_AXIS]->set_fixed_microsteps(  true );
    myMotor[Y_AXIS]->set_steps_per_mm(      DEFAULT_Y_STEPS_PER_MM );

    // Z Axis
    myMotor[Z_AXIS]->set_ms2_pin(           Z_STEPPER_MS2_PIN );
    myMotor[Z_AXIS]->set_ms1_pin(           Z_STEPPER_MS1_PIN );
    myMotor[Z_AXIS]->set_microsteps(        STEPPER_MOTOR_MICROSTEPS_EDM );
    myMotor[Z_AXIS]->set_microsteps_travel( STEPPER_MOTOR_MICROSTEPS_TRAVEL );
    myMotor[Z_AXIS]->set_microsteps_edm(    STEPPER_MOTOR_MICROSTEPS_EDM );
    myMotor[Z_AXIS]->set_fixed_microsteps(  false );
    myMotor[Z_AXIS]->set_steps_per_mm(      DEFAULT_Z_STEPS_PER_MM );


    if (STEPPERS_DISABLE_PIN != UNDEFINED_PIN) {
        pinMode(STEPPERS_DISABLE_PIN, OUTPUT); 
    }
    // certain motors need features to be turned on. Check them here
    for (uint8_t axis = X_AXIS; axis < n_axis; axis++) {
        myMotor[axis]->init();
    }
}

int G_STEPPER_MANAGER::convert_mm_to_steps( float mm, uint8_t axis){
    return round( axis_settings[axis]->steps_per_mm->get() * mm );
}

G_EDM_STEPPER* G_STEPPER_MANAGER::get_motor( int axis ) {
    return myMotor[axis];
}
void G_STEPPER_MANAGER::restore(){
    ignore_disable = false;
}
void G_STEPPER_MANAGER::set_ignore_disable( bool ignore ){
    ignore_disable = ignore;
}

void G_STEPPER_MANAGER::custom_step_axis( int axis_index, bool dir ){
    myMotor[axis_index]->set_direction( dir );
    myMotor[axis_index]->step();
}
void G_STEPPER_MANAGER::motors_set_disable(bool disable, uint8_t mask) {
    if( disable && ignore_disable ){ return; }
    static bool    prev_disable = true;
    static uint8_t prev_mask    = 0;

    if ((disable == prev_disable) && (mask == prev_mask)) {
        return;
    }

    prev_disable = disable;
    prev_mask    = mask;

    if (DEFAULT_INVERT_ST_ENABLE) {
        disable = !disable;  // Apply pin invert.
    }

    // global disable.
    digitalWrite(STEPPERS_DISABLE_PIN, disable);
    
    for (uint8_t axis = 0; axis < n_axis; axis++) {
        myMotor[axis]->set_disable( disable );
    }

    // Add an optional delay for stepper drivers. that need time
    // Some need time after the enable before they can step.
    auto wait_disable_change = DEFAULT_STEP_ENABLE_DELAY;
    if (wait_disable_change != 0) {
        auto disable_start_time = esp_timer_get_time() + wait_disable_change;
        while ((esp_timer_get_time() - disable_start_time) < 0) {
            NOP();
        }
    }
}

void G_STEPPER_MANAGER::motors_read_settings() {
    //grbl_msg_sendf(MsgLevel::Info, "Read Settings");
    for (uint8_t axis = X_AXIS; axis < n_axis; axis++) {
        myMotor[axis]->read_settings();
    }
}

// use this to tell all the motors what the current homing mode is
// They can use this to setup things like Stall
uint8_t G_STEPPER_MANAGER::motors_set_homing_mode(uint8_t homing_mask, bool isHoming) {
    uint8_t can_home = 0;
    for (uint8_t axis = X_AXIS; axis < n_axis; axis++) {
        if (bitnum_istrue(homing_mask, axis)) {
            if( ! isHoming ){
                if( sys_rt_exec_state.bit.motionCancel ){ continue; }
                sys_axis_homed[axis] = true;
            } else{
                sys_axis_homed[axis] = false;
            }
            if ( myMotor[axis]->set_homing_mode(isHoming) ) {
                bitnum_true(can_home, axis);
            }
            //myMotor[axis][1]->set_homing_mode(isHoming);
        }
    }
    return can_home;
}



bool G_STEPPER_MANAGER::motors_direction(uint8_t dir_mask) {

    myMotor[X_AXIS]->set_direction( bitnum_istrue( dir_mask, X_AXIS ) );
    myMotor[Y_AXIS]->set_direction( bitnum_istrue( dir_mask, Y_AXIS ) );
    if( sys.gedm_stepper_disable_z ){ return true; }
    myMotor[Z_AXIS]->set_direction( bitnum_istrue( dir_mask, Z_AXIS ) );
    return true;

}

void G_STEPPER_MANAGER::motors_step(uint8_t step_mask) {
    for (uint8_t axis = X_AXIS; axis < n_axis; axis++) {
        if( sys.gedm_stepper_disable_z && axis == Z_AXIS ){ 
            // if floating z is enabled ignore all z moves
            continue; 
        }
        if (bitnum_istrue(step_mask, axis)) {
            myMotor[axis]->step();
        }
    }
}
// Turn all stepper pins off
void G_STEPPER_MANAGER::motors_unstep() {
    /*for (uint8_t axis = X_AXIS; axis < n_axis; axis++) {
        myMotor[axis]->unstep();
    }*/
}