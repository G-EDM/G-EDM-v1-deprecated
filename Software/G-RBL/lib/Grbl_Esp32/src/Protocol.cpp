/*
  Protocol.cpp - controls Grbl execution protocol and procedures
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

  2023 - Roland Lautensack (G-EDM) This file was heavily edited and may no longer be compatible with the default grbl

*/
#include "Grbl.h"

static void protocol_exec_rt_suspend();

static char line[LINE_BUFFER_SIZE];     // Line to be executed. Zero-terminated.

typedef struct {
    char buffer[LINE_BUFFER_SIZE];
    int  len;
    int  line_number;
} client_line_t;
client_line_t client_lines;

static void empty_line() {
    client_line_t* cl = &client_lines;
    cl->len           = 0;
    cl->buffer[0]     = '\0';
}

Error add_char_to_line(char c) {
    client_line_t* cl = &client_lines;
    // Simple editing for interactive input
    if (c == '\b') {
        // Backspace erases
        if (cl->len) {
            --cl->len;
            cl->buffer[cl->len] = '\0';
        }
        return Error::Ok;
    }
    if (cl->len == (LINE_BUFFER_SIZE - 1)) {
        return Error::Overflow;
    }
    if (c == '\r' || c == '\n') {
        cl->len = 0;
        cl->line_number++;
        return Error::Eol;
    }
    cl->buffer[cl->len++] = c;
    cl->buffer[cl->len]   = '\0';
    return Error::Ok;
}

Error execute_line(char* line) {
    grbl_send(line);
    Error result = Error::Ok;
    if (line[0] == 0) {
        return Error::Ok;
    }
    if (line[0] == '$' || line[0] == '[') {
        return system_execute_line(line);
    }
    if (sys.state == State::Alarm || sys.state == State::Jog) {
        return Error::SystemGcLock;
    }
    return gc_execute_line(line);
}


/*
  GRBL PRIMARY LOOP:
*/
void protocol_main_loop() {
    client_reset_read_buffer(CLIENT_ALL);
    empty_line();
    //uint8_t client = CLIENT_SERIAL; // default client
    // Perform some machine checks to make sure everything is good to go.
    // Check for and report alarm state after a reset, error, or an initial power up.
    // NOTE: Sleep mode disables the stepper drivers and position can't be guaranteed.
    // Re-initialize the sleep state as an ALARM mode to ensure user homes or acknowledges.
    if (sys.state == State::Alarm || sys.state == State::Sleep) {
        report_feedback_message(Message::AlarmLock);
        sys.state = State::Alarm;  // Ensure alarm state is set.
    } else {
        // Check if the safety door is open.
        sys.state = State::Idle;
        // All systems go!
        system_execute_startup(line);  // Execute startup script.
    }
    // ---------------------------------------------------------------------------------
    // Primary loop! Upon a system abort, this exits back to main() to reset the system.
    // This is also where Grbl idles while waiting for something to do.
    // ---------------------------------------------------------------------------------
    protocol_ready = true;
    int c;
    for (;;) {


        /*if( sys.gedm_undo_moves != 0 ){
            // remove; for testing only!
            planner.position_history_undo( sys.gedm_undo_moves );
        }
        if( sys.gedm_planner_sync == 1 ){
            planner.position_history_sync();

        }*/

        if( sys.gedm_insert_probe ){
            planner.reprobing_routine();
        }

        if( sys.gedm_retraction_motion ){
            vTaskDelay(1);
            continue;
        }


        if (SD_ready_next) {
            idle_timer = millis();
            char fileLine[255];
            if (filehandler.readFileLine(fileLine, 255)) {
                SD_ready_next = false;
                report_status_message( execute_line(fileLine) );
            } else {
                filehandler.reset_and_repeat_file();
                planner.next_gcode_round();
            }
        }

        // Receive one line of incoming serial data, as the data becomes available.
        // Filtering, if necessary, is done later in gc_execute_line(), so the
        // filtering is the same with serial and file input.
        char* line;

        while ((c = client_read(CLIENT_INPUT)) != -1) {
            Error res = add_char_to_line(c);
            switch (res) {
                case Error::Ok:
                    break;
                case Error::Eol:
                    protocol_execute_realtime();  // Runtime command check point.
                    if (sys.abort) {
                        return;  // Bail to calling function upon system abort
                    }
                    line = client_lines.buffer;
                    execute_line(line);
                    empty_line();
                    idle_timer = millis();
                    break;
                case Error::Overflow:
                    report_status_message(Error::Overflow);
                    empty_line();
                    break;
                default:
                    break;
            }
        }  

        

        //protocol_execute_realtime();  // Runtime command check point.
        if (sys.abort) {
            return;  // Bail to main() program loop to reset system.
        }

        if( millis() - idle_timer > 1000 ){

            idle_timer = millis();
            
            if( ! sys.gedm_planner_line_running ){
                sys.state = State::Idle;
            } else{
                sys.state = State::Cycle;
            }

        }


        protocol_execute_realtime();
        vTaskDelay(1);

    }
    return; /* Never reached */
}


void protocol_execute_realtime() {
    protocol_exec_rt_system();
    if (sys.suspend.value) {
        protocol_exec_rt_suspend();
    }
}


void protocol_exec_rt_system() {
    ExecAlarm alarm = sys_rt_exec_alarm;  // Temp variable to avoid calling volatile multiple times.
    if (alarm != ExecAlarm::None) {       // Enter only if an alarm is pending
        // System alarm. Everything has shutdown by something that has gone severely wrong. Report
        // the source of the error to the user. If critical, Grbl disables by entering an infinite
        // loop until system reset/abort.
        sys.state = State::Alarm;  // Set system alarm state
        report_alarm_message(alarm);
        // Halt everything upon a critical event flag. Currently hard and soft limits flag this.
        if ((alarm == ExecAlarm::HardLimit) || (alarm == ExecAlarm::SoftLimit)) {
            report_feedback_message(Message::CriticalEvent);

            //sys_rt_exec_state.bit.reset = false;  // Disable any existing reset
            //do {
                
                // Block everything, except reset and status reports, until user issues reset or power
                // cycles. Hard limits typically occur while unattended or not paying attention. Gives
                // the user and a GUI time to do what is needed before resetting, like killing the
                // incoming stream. The same could be said about soft limits. While the position is not
                // lost, continued streaming could cause a serious crash if by chance it gets executed.
            //} while (!sys_rt_exec_state.bit.reset);
        }
        sys_rt_exec_alarm = ExecAlarm::None;
    }
    ExecState rt_exec_state;
    rt_exec_state.value = sys_rt_exec_state.value;  // Copy volatile sys_rt_exec_state.
    if (rt_exec_state.value != 0 || cycle_stop) {   // Test if any bits are on
        // Execute system abort.
        if (rt_exec_state.bit.reset) {
            sys.abort = true;  // Only place this is set true.
            return;            // Nothing else to do but exit.
        }

        // NOTE: Once hold is initiated, the system immediately enters a suspend state to block all
        // main program processes until either reset or resumed. This ensures a hold completes safely.
        if (rt_exec_state.bit.motionCancel || rt_exec_state.bit.sleep) {
            // State check for allowable states for hold methods.
            if (!(sys.state == State::Alarm || sys.state == State::CheckMode)) {
                // If in CYCLE or JOG states, immediately initiate a motion HOLD.
                if (sys.state == State::Cycle || sys.state == State::Jog) {
                    if (!(sys.suspend.bit.motionCancel || sys.suspend.bit.jogCancel)) {  // Block, if already holding.
                        sys.step_control             = {};
                        sys.step_control.executeHold = true;  // Initiate suspend state with active flag.
                        if (sys.state == State::Jog) {        // Jog cancelled upon any hold event, except for sleeping.
                            if (!rt_exec_state.bit.sleep) {
                                sys.suspend.bit.jogCancel = true;
                            }
                        }
                    }
                }
                // If IDLE, Grbl is not in motion. Simply indicate suspend state and hold is complete.
                if (sys.state == State::Idle) {
                    sys.suspend.value            = 0;
                    sys.suspend.bit.holdComplete = true;
                }
                // Execute and flag a motion cancel with deceleration and return to idle. Used primarily by probing cycle
                // to halt and cancel the remainder of the motion.
                if (rt_exec_state.bit.motionCancel) {
                    // MOTION_CANCEL only occurs during a CYCLE, but a HOLD and SAFETY_DOOR may been initiated beforehand
                    // to hold the CYCLE. Motion cancel is valid for a single planner block motion only, while jog cancel
                    // will handle and clear multiple planner block motions.
                    if (sys.state != State::Jog) {
                        sys.suspend.bit.motionCancel = true;  // NOTE: State is State::Cycle.
                    }
                    //sys_rt_exec_state.bit.motionCancel = false;
                }

            }
            if (rt_exec_state.bit.sleep) {
                if (sys.state == State::Alarm) {
                    sys.suspend.bit.holdComplete    = true;
                }
                sys.state                   = State::Sleep;
                sys_rt_exec_state.bit.sleep = false;
            }
        }
        // Execute a cycle start by starting the stepper interrupt to begin executing the blocks in queue.
        if (rt_exec_state.bit.cycleStart) {
            // Block if called at same time as the hold commands: feed hold, motion cancel, and safety door.
            // Ensures auto-cycle-start doesn't resume a hold without an explicit user-input.
            if (!( rt_exec_state.bit.motionCancel )) {

            }
            sys_rt_exec_state.bit.cycleStart = false;
        }
        if (cycle_stop) {
            // Reinitializes the cycle plan and stepper system after a feed hold for a resume. Called by
            // realtime command execution in the main program, ensuring that the planner re-plans safely.
            // NOTE: Bresenham algorithm variables are still maintained through both the planner and stepper
            // cycle reinitializations. The stepper path should continue exactly as if nothing has happened.
            // NOTE: cycle_stop is set by the stepper subsystem when a cycle or feed hold completes.
            if ((sys.state == State::Hold  || sys.state == State::Sleep) && !(sys.soft_limit) &&
                !(sys.suspend.bit.jogCancel)) {
                // Hold complete. Set to indicate ready to resume.  Remain in HOLD or DOOR states until user
                // has issued a resume command or reset.

                if (sys.step_control.executeHold) {
                    sys.suspend.bit.holdComplete = true;
                }
                sys.step_control.executeHold      = false;
                sys.step_control.executeSysMotion = false;
            } else {
                // Motion complete. Includes CYCLE/JOG/HOMING states and jog cancel/motion cancel/soft limit events.
                // NOTE: Motion and jog cancel both immediately return to idle after the hold completes.
                if (sys.suspend.bit.jogCancel) {  // For jog cancel, flush buffers and sync positions.
                    sys.step_control = {};
                    //st_reset();
                    gc_sync_position();
                }

                    sys.suspend.value = 0;
                    sys.state         = State::Idle;
            }
            cycle_stop = false;
        }
    }
    // Execute overrides.
    if ((sys_rt_f_override != sys.f_override) || (sys_rt_r_override != sys.r_override)) {
        sys.f_override         = sys_rt_f_override;
        sys.r_override         = sys_rt_r_override;
        sys.report_ovr_counter = 0;  // Set to report change immediately
    }

    // NOTE: Unlike motion overrides, spindle overrides do not require a planner reinitialization.
    if (sys_rt_s_override != sys.spindle_speed_ovr) {
        sys.step_control.updateSpindleRpm = true;
        sys.spindle_speed_ovr             = sys_rt_s_override;
        sys.report_ovr_counter            = 0;  // Set to report change immediately
        // If spinlde is on, tell it the rpm has been overridden
        if (gc_state.modal.spindle != SpindleState::Disable) {
            //spindle->set_rpm(gc_state.spindle_speed);
        }
    }

    if (sys_rt_exec_accessory_override.bit.spindleOvrStop) {
        sys_rt_exec_accessory_override.bit.spindleOvrStop = false;
        // Spindle stop override allowed only while in HOLD state.
        // NOTE: Report counters are set in spindle_set_state() when spindle stop is executed.
        if (sys.state == State::Hold) {
            if (sys.spindle_stop_ovr.value == 0) {
                sys.spindle_stop_ovr.bit.initiate = true;
            } else if (sys.spindle_stop_ovr.bit.enabled) {
                sys.spindle_stop_ovr.bit.restore = true;
            }
        }
    }


    // Reload step segment buffer
    switch (sys.state) {
        case State::Cycle:
        case State::Hold:
        case State::Homing:
        case State::Sleep:
        case State::Jog:
            break;
        default:
            break;
    }
}





bool is_retracting = false;

static void protocol_exec_rt_suspend() {

}
