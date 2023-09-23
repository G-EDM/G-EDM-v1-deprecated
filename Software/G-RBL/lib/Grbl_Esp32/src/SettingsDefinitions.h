#pragma once

extern AxisSettings* x_axis_settings;
extern AxisSettings* y_axis_settings;
extern AxisSettings* z_axis_settings;
extern AxisSettings* a_axis_settings;
extern AxisSettings* b_axis_settings;
extern AxisSettings* c_axis_settings;
extern AxisSettings* axis_settings[];

extern StringSetting* startup_line_0;
extern StringSetting* startup_line_1;
extern StringSetting* build_info;

extern IntSetting* pulse_microseconds;
extern IntSetting* stepper_idle_lock_time;
extern IntSetting* direction_delay_microseconds;
extern IntSetting* enable_delay_microseconds;

extern AxisMaskSetting* step_invert_mask;
extern AxisMaskSetting* dir_invert_mask;
extern AxisMaskSetting* homing_dir_mask;
extern AxisMaskSetting* homing_squared_axes;
extern AxisMaskSetting* homing_cycle[MAX_N_AXIS];

extern FlagSetting* step_enable_invert;
extern FlagSetting* limit_invert;
extern FlagSetting* probe_invert;
extern FlagSetting* report_inches;
extern FlagSetting* soft_limits;
extern FlagSetting* hard_limits;
extern FlagSetting* homing_enable;
extern FlagSetting* laser_mode;
extern IntSetting*  laser_full_power;

extern IntSetting*   status_mask;
extern FloatSetting* junction_deviation;
extern FloatSetting* arc_tolerance;

extern FloatSetting* homing_feed_rate;
extern FloatSetting* homing_seek_rate;
extern FloatSetting* homing_debounce;
extern FloatSetting* homing_pulloff;

extern AxisMaskSetting* stallguard_debug_mask;

extern EnumSetting* message_level;
