/*
  MotionControl.cpp - high level interface for issuing motion commands
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

  2023 - Roland Lautensack (G-EDM) This file was edited and may no longer be compatible with the default grbl

*/

//#include "Grbl.h"
#include "MotionControl.h"

// M_PI is not defined in standard C/C++ but some compilers
// support it anyway.  The following suppresses Intellisense
// problem reports.
#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif

bool mc_line(float* target, plan_line_data_t* pl_data) {

    bool submitted_result = false;
    // store the plan data so it can be cancelled by the protocol system if needed
    sys_pl_data_inflight = pl_data;

    // If in check gcode mode, prevent motion by blocking planner. Soft limits still work.
    if (sys.state == State::CheckMode) {
        sys_pl_data_inflight = NULL;
        return submitted_result;
    }

    // Plan and queue motion into planner buffer
    // uint8_t plan_status; // Not used in normal operation.
    if (sys_pl_data_inflight == pl_data) {
        planner.plan_history_line(target, pl_data);
        submitted_result = true;
    }
    sys_pl_data_inflight = NULL;
    return submitted_result;
}

bool __attribute__((weak)) cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position) {
    return mc_line(target, pl_data);
}

bool __attribute__((weak)) kinematics_pre_homing(uint8_t cycle_mask) {
    return false;  // finish normal homing cycle
}

void __attribute__((weak)) kinematics_post_homing() {}

void __attribute__((weak)) motors_to_cartesian(float* cartesian, float* motors, int n_axis) {
    memcpy(cartesian, motors, n_axis * sizeof(motors[0]));
}

void __attribute__((weak)) forward_kinematics(float* position) {}


void mc_arc(float*            target,
            plan_line_data_t* pl_data,
            float*            position,
            float*            offset,
            float             radius,
            uint8_t           axis_0,
            uint8_t           axis_1,
            uint8_t           axis_linear,
            uint8_t           is_clockwise_arc) {
    float center_axis0 = position[axis_0] + offset[axis_0];
    float center_axis1 = position[axis_1] + offset[axis_1];
    float r_axis0      = -offset[axis_0];  // Radius vector from center to current location
    float r_axis1      = -offset[axis_1];
    float rt_axis0     = target[axis_0] - center_axis0;
    float rt_axis1     = target[axis_1] - center_axis1;

    float previous_position[MAX_N_AXIS] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    uint16_t n;
    auto n_axis = N_AXIS;
    for (n = 0; n < n_axis; n++) {
        previous_position[n] = position[n];
    }
    // CCW angle between position and target from circle center. Only one atan2() trig computation required.
    float angular_travel = atan2(r_axis0 * rt_axis1 - r_axis1 * rt_axis0, r_axis0 * rt_axis0 + r_axis1 * rt_axis1);
    if (is_clockwise_arc) {  // Correct atan2 output per direction
        if (angular_travel >= -ARC_ANGULAR_TRAVEL_EPSILON) {
            angular_travel -= 2 * M_PI;
        }
    } else {
        if (angular_travel <= ARC_ANGULAR_TRAVEL_EPSILON) {
            angular_travel += 2 * M_PI;
        }
    }
    // NOTE: Segment end points are on the arc, which can lead to the arc diameter being smaller by up to
    // (2x) arc_tolerance. For 99% of users, this is just fine. If a different arc segment fit
    // is desired, i.e. least-squares, midpoint on arc, just change the mm_per_arc_segment calculation.
    // For the intended uses of Grbl, this value shouldn't exceed 2000 for the strictest of cases.
    uint16_t segments = floor(fabs(0.5 * angular_travel * radius) / sqrt(arc_tolerance->get() * (2 * radius - arc_tolerance->get())));
    if (segments) {
        // Multiply inverse feed_rate to compensate for the fact that this movement is approximated
        // by a number of discrete segments. The inverse feed_rate should be correct for the sum of
        // all segments.
        if (pl_data->motion.inverseTime) {
            pl_data->feed_rate *= segments;
            pl_data->motion.inverseTime = 0;  // Force as feed absolute mode over arc segments.
        }
        float theta_per_segment  = angular_travel / segments;
        float linear_per_segment = (target[axis_linear] - position[axis_linear]) / segments;

        // Computes: cos_T = 1 - theta_per_segment^2/2, sin_T = theta_per_segment - theta_per_segment^3/6) in ~52usec
        float cos_T = 2.0 - theta_per_segment * theta_per_segment;
        float sin_T = theta_per_segment * 0.16666667 * (cos_T + 4.0);
        cos_T *= 0.5;
        float    sin_Ti;
        float    cos_Ti;
        float    r_axisi;
        uint16_t i;
        uint8_t  count             = 0;
        float    original_feedrate = pl_data->feed_rate;  // Kinematics may alter the feedrate, so save an original copy
        for (i = 1; i < segments; i++) {                  // Increment (segments-1).
            if (count < N_ARC_CORRECTION) {
                // Apply vector rotation matrix. ~40 usec
                r_axisi = r_axis0 * sin_T + r_axis1 * cos_T;
                r_axis0 = r_axis0 * cos_T - r_axis1 * sin_T;
                r_axis1 = r_axisi;
                count++;
            } else {
                // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments. ~375 usec
                // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
                cos_Ti  = cos(i * theta_per_segment);
                sin_Ti  = sin(i * theta_per_segment);
                r_axis0 = -offset[axis_0] * cos_Ti + offset[axis_1] * sin_Ti;
                r_axis1 = -offset[axis_0] * sin_Ti - offset[axis_1] * cos_Ti;
                count   = 0;
            }
            // Update arc_target location
            position[axis_0] = center_axis0 + r_axis0;
            position[axis_1] = center_axis1 + r_axis1;
            position[axis_linear] += linear_per_segment;
            pl_data->feed_rate = original_feedrate;  // This restores the feedrate kinematics may have altered
            limitsCheckSoft(position);
            cartesian_to_motors(position, pl_data, previous_position);
            previous_position[axis_0]      = position[axis_0];
            previous_position[axis_1]      = position[axis_1];
            previous_position[axis_linear] = position[axis_linear];
            // Bail mid-circle on system abort. Runtime command check already performed by mc_line.
            if (sys.abort) {
                return;
            }
        }
    }
    // Ensure last segment arrives at target location.
    limitsCheckSoft(target);
    cartesian_to_motors(target, pl_data, previous_position);
}

// Execute dwell in seconds.
bool mc_dwell(int32_t milliseconds) {
    if (milliseconds <= 0 || sys.state == State::CheckMode) {
        return false;
    }
    return delay_msec(milliseconds, DwellMode::Dwell);
}

// return true if the mask has exactly one bit set,
// so it refers to exactly one axis
static bool mask_is_single_axis(uint8_t axis_mask) {
    // This code depends on the fact that, for binary numberes
    // with only one bit set - and only for such numbers -
    // the bits in one less than the number are disjoint
    // with that bit.  For a number like B100, if you
    // subtract one, the low order 00 bits will have to
    // borrow from the high 1 bit and thus clear it.
    // If any lower bits are 1, then there will be no
    // borrow to clear the highest 1 bit.
    return axis_mask && ((axis_mask & (axis_mask - 1)) == 0);
}

static bool axis_is_squared(uint8_t axis_mask) {
    // Squaring can only be done if it is the only axis in the mask
    if (axis_mask & homing_squared_axes->get()) {
        if (mask_is_single_axis(axis_mask)) {
            return true;
        }
        grbl_msg_sendf(MsgLevel::Info, "Cannot multi-axis home with squared axes. Homing normally");
        return false;
    }

    return false;
}


#    define BACKUP_STEPPER(save_stepper)
#    define RESTORE_STEPPER(save_stepper)

// Perform homing cycle to locate and set machine zero. Only '$H' executes this command.
// NOTE: There should be no motions in the buffer and Grbl must be in an idle state before
// executing the homing cycle. This prevents incorrect buffered plans after homing.
void mc_homing_cycle(uint8_t cycle_mask) {
    bool no_cycles_defined = true;

    if (user_defined_homing(cycle_mask)) {
        return;
    }

    // This give kinematics a chance to do something before normal homing
    // if it returns true, the homing is canceled.
    if (kinematics_pre_homing(cycle_mask)) {
        return;
    }
    // Check and abort homing cycle, if hard limits are already enabled. Helps prevent problems
    // with machines with limits wired on both ends of travel to one limit pin.
    // TODO: Move the pin-specific LIMIT_PIN call to Limits.cpp as a function.
#ifdef LIMITS_TWO_SWITCHES_ON_AXES
    if (limits_get_state()) {
        mc_reset();  // Issue system reset and ensure spindle and .. are shutdown.
        sys_rt_exec_alarm = ExecAlarm::HardLimit;
        return;
    }
#endif
    limits_disable();  // Disable hard limits pin change register for cycle duration
    // -------------------------------------------------------------------------------------
    // Perform homing routine. NOTE: Special motion case. Only system reset works.
    n_homing_locate_cycle = NHomingLocateCycle;
#ifdef HOMING_SINGLE_AXIS_COMMANDS
    /*
    if (cycle_mask) { limits_go_home(cycle_mask); } // Perform homing cycle based on mask.
    else
    */
    if (cycle_mask) {
        if (!axis_is_squared(cycle_mask)) {
            limits_go_home(cycle_mask);  // Homing cycle 0
        } else {
            n_homing_locate_cycle = 0;  // don't do a second touch cycle
            limits_go_home(cycle_mask);
            n_homing_locate_cycle = NHomingLocateCycle;  // restore to default value
            limits_go_home(cycle_mask);
            limits_go_home(cycle_mask);
        }
    }  // Perform homing cycle based on mask.
    else
#endif
    {
        for (int cycle = 0; cycle < MAX_N_AXIS; cycle++) {
            auto homing_mask = homing_cycle[cycle]->get();
            if (homing_mask) {  // if there are some axes in this cycle
                no_cycles_defined = false;
                if (!axis_is_squared(homing_mask)) {
                    limits_go_home(homing_mask);  // Homing cycle 0
                } else {
                    n_homing_locate_cycle = 0;  // don't do a second touch cycle
                    limits_go_home(homing_mask);
                    n_homing_locate_cycle = NHomingLocateCycle;  // restore to default value
                    limits_go_home(homing_mask);
                    limits_go_home(homing_mask);
                }
            }
        }
        if (no_cycles_defined) {
            report_status_message(Error::HomingNoCycles);
        }
    }
    protocol_execute_realtime();  // Check for reset and set system abort.
    if (sys.abort) {
        return;  // Did not complete. Alarm state set by mc_alarm.
    }
    // Homing cycle complete! Setup system for normal operation.
    // -------------------------------------------------------------------------------------
    // Sync gcode parser and planner positions to homed position.
    gc_sync_position();
    // This give kinematics a chance to do something after normal homing
    kinematics_post_homing();
    // If hard limits feature enabled, re-enable hard limits pin change register after homing cycle.
    limits_init();
}

// Perform tool length probe cycle. Requires probe switch.
// NOTE: Upon probe failure, the program will be stopped and placed into ALARM state.
GCUpdatePos mc_probe_cycle(float* target, plan_line_data_t* pl_data, uint8_t parser_flags) {
    // TODO: Need to update this cycle so it obeys a non-auto cycle start.
    if (sys.state == State::CheckMode) {
#ifdef SET_CHECK_MODE_PROBE_TO_START
        return GCUpdatePos::None;
#else
        return GCUpdatePos::Target;
#endif
    }

    if (sys.abort) {
        return GCUpdatePos::None;  // Return if system reset has been issued.
    }

    // Initialize probing control variables
    uint8_t is_probe_away = bit_istrue(parser_flags, GCParserProbeIsAway);
    uint8_t is_no_error   = bit_istrue(parser_flags, GCParserProbeIsNoError);
    sys.probe_succeeded   = false;  // Re-initialize probe history before beginning cycle.
    set_probe_direction(is_probe_away);
    // After syncing, check if probe is already triggered. If so, halt and issue alarm.
    // NOTE: This probe initialization error applies to all probing cycles.
    if (probe_get_state() ^ is_probe_away) {  // Check probe pin state.
        sys_rt_exec_alarm = ExecAlarm::ProbeFailInitial;
        protocol_execute_realtime();
        return GCUpdatePos::None;       // Nothing else to do but bail.
    }
    // Setup and queue probing motion. Auto cycle-start should not start the cycle.
    limitsCheckSoft(target);
    sys_probe_state = Probe::Active;
    cartesian_to_motors(target, pl_data, gc_state.position);
    sys_rt_exec_state.bit.cycleStart = true;
    // Probing cycle complete!
    // Set state variables and error out, if the probe failed and cycle with error is enabled.
    if (sys_probe_state == Probe::Active) {
        if (is_no_error) {
            memcpy(sys_probe_position, sys_position, sizeof(sys_position));
        } else {
            sys.gedm_stop_process = true;
            sys_rt_exec_alarm = ExecAlarm::ProbeFailContact;
        }
    } else {
        sys.probe_succeeded = true;  // Indicate to system the probing cycle completed successfully.
    }
    sys_probe_state = Probe::Off;  // Ensure probe state monitor is disabled.
    protocol_execute_realtime();   // Check and execute run-time commands
    if (sys.probe_succeeded) {
        return GCUpdatePos::System;  // Successful probe cycle.
    } else {
        return GCUpdatePos::Target;  // Failed to trigger probe within travel. With or without error.
    }
}


// Method to ready the system to reset by setting the realtime reset command and killing any
// active processes in the system. This also checks if a system reset is issued while Grbl
// is in a motion state. If so, kills the steppers and sets the system alarm to flag position
// lost, since there was an abrupt uncontrolled deceleration. Called at an interrupt level by
// realtime abort command and hard limits. So, keep to a minimum.
void mc_reset() {
    grbl_msg_sendf(MsgLevel::Debug, "mc_reset()");
    // Only this function can set the system reset. Helps prevent multiple kill calls.
    if (!sys_rt_exec_state.bit.reset) {
        sys_rt_exec_state.bit.reset = true;
        //spindle->stop();

        // do we need to stop a running SD job?
        if (filehandler.get_sd_state(false) == SDState::BusyPrinting) {
            //Report print stopped
            report_feedback_message(Message::SdFileQuit);
            //filehandler.closeFile();
        }
        // Kill steppers only if in any motion state, i.e. cycle, actively holding, or homing.
        // NOTE: If steppers are kept enabled via the step idle delay setting, this also keeps
        // the steppers enabled by avoiding the go_idle call altogether, unless the motion state is
        // violated, by which, all bets are off.
        if ((sys.state == State::Cycle || sys.state == State::Homing || sys.state == State::Jog) ||
            (sys.step_control.executeHold || sys.step_control.executeSysMotion)) {
            if (sys.state == State::Homing) {
                if (sys_rt_exec_alarm == ExecAlarm::None) {
                    sys_rt_exec_alarm = ExecAlarm::HomingFailReset;
                }
            } else {
                sys_rt_exec_alarm = ExecAlarm::AbortCycle;
            }
            //st_go_idle();  // Force kill steppers. Position has likely been lost.
        }

    }
}
