#include "Grbl.h"
#include "Config.h"


// Declare system global variable structure
system_t               sys;

int32_t                sys_position[MAX_N_AXIS];        // Real-time machine (aka home) position vector in steps.
int32_t                sys_probe_position[MAX_N_AXIS];  // Last probe position in machine coordinates and steps.
int32_t                sys_probe_position_final[MAX_N_AXIS];
bool                   sys_probed_axis[MAX_N_AXIS];

volatile ExecAlarm sys_last_alarm;

volatile Probe         sys_probe_state;                 // Probing state value.  Used to coordinate the probing cycle with stepper ISR.
volatile ExecState     sys_rt_exec_state;  // Global realtime executor bitflag variable for state management. See EXEC bitmasks.
volatile ExecAlarm     sys_rt_exec_alarm;  // Global realtime executor bitflag variable for setting various alarms.
volatile ExecAccessory sys_rt_exec_accessory_override;  // Global realtime executor bitflag variable for spindle/... overrides.
volatile bool          cycle_stop;                      // For state transitions, instead of bitflag
volatile void*         sys_pl_data_inflight;  // holds a plan_line_data_t while cartesian_to_motors has taken ownership of a line motion
#ifdef DEBUG
volatile bool sys_rt_exec_debug;
#endif
volatile Percent sys_rt_f_override;  // Global realtime executor feedrate override percentage
volatile Percent sys_rt_r_override;  // Global realtime executor rapid override percentage
volatile Percent sys_rt_s_override;  // Global realtime executor spindle override percentage

xQueueHandle control_sw_queue;    // used by control switch debouncing
bool         debouncing = false;  // debouncing in process

void system_ini() {  // Renamed from system_init() due to conflict with esp32 files

}

unsigned long idle_timer = millis();


void system_flag_wco_change() {
    sys.report_wco_counter = 0;
}

/** this is not accurate used only for rough calculations! **/
int system_convert_mm_to_steps(float mm, uint8_t idx) {
    int steps = int( mm * axis_settings[idx]->steps_per_mm->get() );
    return steps;
}



float system_convert_axis_steps_to_mpos(int32_t steps, uint8_t idx) {
    float pos;
    float steps_per_mm = axis_settings[idx]->steps_per_mm->get();
    pos                = steps / steps_per_mm;
    return pos;
}

// Returns machine position of axis 'idx'. Must be sent a 'step' array.
// NOTE: If motor steps and machine position are not in the same coordinate frame, this function
//   serves as a central place to compute the transformation.
void system_convert_array_steps_to_mpos(float* position, int32_t* steps) {
    auto  n_axis = N_AXIS;
    float motors[n_axis];
    for (int idx = 0; idx < n_axis; idx++) {
        motors[idx] = (float)steps[idx] / axis_settings[idx]->steps_per_mm->get();
    }
    motors_to_cartesian(position, motors, n_axis);
    // mtoc is just memcpy(cartesian, motors, n_axis * sizeof(motors[0]));
}
float* system_get_mpos() {
    static float position[MAX_N_AXIS];
    system_convert_array_steps_to_mpos(position, sys_position);
    return position;
};


/*
    This returns an unused pwm channel.
    The 8 channels share 4 timers, so pairs 0,1 & 2,3 , etc
    have to be the same frequency. The spindle always uses channel 0
    so we start counting from 2.

    There are still possible issues if requested channels use different frequencies
    TODO: Make this more robust.
*/
int8_t sys_get_next_PWM_chan_num() {
    static uint8_t next_PWM_chan_num = 2;  // start at 2 to avoid spindle
    if (next_PWM_chan_num < 8) {           // 7 is the max PWM channel number
        return next_PWM_chan_num++;
    } else {
        grbl_msg_sendf(MsgLevel::Error, "Error: out of PWM channels");
        return -1;
    }
}

/*
		Calculate the highest precision of a PWM based on the frequency in bits

		80,000,000 / freq = period
		determine the highest precision where (1 << precision) < period
	*/
uint8_t sys_calc_pwm_precision(uint32_t freq) {
    uint8_t precision = 0;

    // increase the precision (bits) until it exceeds allow by frequency the max or is 16
    while ((1 << precision) < (uint32_t)(80000000 / freq) && precision <= 16) {  // TODO is there a named value for the 80MHz?
        precision++;
    }

    return precision - 1;
}
