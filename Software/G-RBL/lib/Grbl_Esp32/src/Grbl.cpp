/*
  Grbl.cpp - Initialization and main loop for Grbl
  Part of Grbl
  Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC

	2018 -	Bart Dring This file was modifed for use on the ESP32
					CPU. Do not use this with Grbl for atMega328P

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
*/


//################################################
// GRBL_ESP32
//################################################
#include "Grbl.h"


void grbl_init() {

    client_init();

    disableCore0WDT();
    disableCore1WDT();
    settings_init();
    //stepper_init();
    system_ini();     // Configure pinout pins and pin-change interrupt (Renamed due to conflict with esp32 files)
    motor_manager.init();
    memset(sys_position, 0, sizeof(sys_position));  // Clear machine position.
    machine_init();                                 // weak definition in Grbl.cpp does nothing
    sys.state = State::Idle;

#ifdef HOMING_INIT_LOCK
    if (homing_enable->get()) {
        sys.state = State::Alarm;
    }
#endif
    //Spindles::Spindle::select();

    WebUI::inputBuffer.begin();

    grbl_msg_sendf(MsgLevel::Info, MACHINE_NAME);  

}

static void reset_variables() {
    
    // Reset system variables.
    State prior_state = sys.state;
    memset(&sys, 0, sizeof(system_t));  // Clear system struct variable.
    sys.state             = prior_state;
    sys.f_override        = FeedOverride::Default;              // Set to 100%
    sys.r_override        = RapidOverride::Default;             // Set to 100%
    sys.spindle_speed_ovr = SpindleSpeedOverride::Default;      // Set to 100%
    memset(sys_probe_position, 0, sizeof(sys_probe_position));  // Clear probe position.

    sys_probe_state                      = Probe::Off;
    sys_rt_exec_state.value              = 0;
    sys_rt_exec_state.bit.retraction     = 0;
    sys_rt_exec_accessory_override.value = 0;
    sys_rt_exec_alarm                    = ExecAlarm::None;
    cycle_stop                           = false;
    sys_rt_f_override                    = FeedOverride::Default;
    sys_rt_r_override                    = RapidOverride::Default;
    sys_rt_s_override                    = SpindleSpeedOverride::Default;

    client_reset_read_buffer(CLIENT_ALL);
    gc_init();  // Set g-code parser to default state
    //spindle->stop();
    limits_init();
    probe_init();
    //st_reset();
    gc_sync_position();
    report_init_message(CLIENT_ALL);
    sys_pl_data_inflight = NULL;
}

void run_once() {
    
    reset_variables();
    // Start Grbl main loop. Processes program inputs and executes them.
    // This can exit on a system abort condition, in which case run_once()
    // is re-executed by an enclosing loop.
    protocol_main_loop();
    
}

void __attribute__((weak)) machine_init() {}

void __attribute__((weak)) display_init() {}

void __attribute__((weak)) user_m30() {}

void __attribute__((weak)) user_tool_change(uint8_t new_tool) {}



/*bool kinematics_pre_homing(uint8_t cycle_mask) {
    return false;  // finish normal homing cycle
}

void kinematics_post_homing(){}*/
