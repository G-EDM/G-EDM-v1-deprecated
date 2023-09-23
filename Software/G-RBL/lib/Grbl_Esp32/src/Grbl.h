#pragma once

/*
  Grbl.h - Header for system level commands and real-time processes
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

  2023 - Roland Lautensack (G-EDM) This file was heavily edited and may no longer be compatible with the default grbl

*/

// Grbl versioning system
const char* const GRBL_VERSION       = "1.3a";
const char* const GRBL_VERSION_BUILD = "20211103";

//#include <Arduino.h>
//#include <EEPROM.h>
//#include <driver/rmt.h>
//#include <esp_task_wdt.h>
//#include <freertos/task.h>
//#include <Preferences.h>
//#include <driver/timer.h>
#include "Config.h"
#include "NutsBolts.h"
#include "Error.h"
#include "Probe.h"
#include "System.h"
#include "GCode.h"
#include "gedm_planner/Planner.h"
#include "Limits.h"
#include "MotionControl.h"
#include "Protocol.h"
#include "Serial.h"
#include "Report.h"
#include "Pins.h"
#include "Motors/Motors.h"
#include "Jog.h"
#include "InputBuffer.h"
#include "Settings.h"
#include "SettingsDefinitions.h"
#include "sd_card/filehandler.h"

void grbl_init();
void run_once();
void machine_init();                      // weak definition in Grbl.cpp
void display_init();                      // weak definition in Grbl.cpp
void user_m30();                          // weak definition in Grbl.cpp/
void user_tool_change(uint8_t new_tool);  // weak definition in Grbl.cpp
bool user_defined_homing(uint8_t cycle_mask);  // weak definition in Limits.cpp
bool cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position);
bool kinematics_pre_homing(uint8_t cycle_mask);
void kinematics_post_homing();
bool limitsCheckTravel(float* target);  // weak in Limits.cpp; true if out of range
void motors_to_cartesian(float* cartestian, float* motors, int n_axis);  // weak definition