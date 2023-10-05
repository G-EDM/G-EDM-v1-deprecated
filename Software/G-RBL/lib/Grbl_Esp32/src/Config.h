#pragma once

/*
  Config.h - compile time configuration
  Part of Grbl

  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC
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

// This file contains compile-time configurations for Grbl's internal system. For the most part,
// users will not need to directly modify these, but they are here for specific needs, i.e.
// performance tuning or adjusting to non-typical machines.

// IMPORTANT: Any changes here requires a full re-compiling of the source code to propagate them.

/*
ESP 32 Notes

Some features should not be changed. See notes below.

*/

#include <Arduino.h>

#include "NutsBolts.h"

// It is no longer necessary to edit this file to choose
// a machine configuration; edit machine.h instead
// machine.h is #included below, after some definitions
// that the machine file might choose to undefine.

// Note: HOMING_CYCLES are now settings
#define SUPPORT_TASK_CORE 1  // Reference: CONFIG_ARDUINO_RUNNING_CORE = 1

// Inverts pin logic of the control command pins based on a mask. This essentially means you can use
// normally-closed switches on the specified pins, rather than the default normally-open switches.
// The mask order is ...
// Macro3 | Macro2 | Macro 1| Macr0 |Cycle Start | Feed Hold | Reset | Safety Door
// For example B1101 will invert the function of the Reset pin.
#define INVERT_CONTROL_PIN_MASK B00001111

// #define ENABLE_CONTROL_SW_DEBOUNCE     // Default disabled. Uncomment to enable.
#define CONTROL_SW_DEBOUNCE_PERIOD 32  // in milliseconds default 32 microseconds

#define USE_RMT_STEPS

// Include the file that loads the machine-specific config file.
// machine.h must be edited to choose the desired file.
//#include "Machines/gedm.h"
#include "config/definitions.h"

#ifndef GRBL_SPI_FREQ
// You can override these by defining them in a board file.
// To override, you must set all of them
//-1 means use the default board pin
#    define GRBL_SPI_SS -1
#    define GRBL_SPI_MOSI -1
#    define GRBL_SPI_MISO -1
#    define GRBL_SPI_SCK -1
#    define GRBL_SPI_FREQ 4000000
#endif

// ESP32 CPU Settings
const uint32_t fTimers = 80000000;  // a reference to the speed of ESP32 timers

// =============== Don't change or comment these out ======================
// They are for legacy purposes and will not affect your I/O

const int STEP_MASK = B111111;

const int PROBE_MASK = 1;

const int MAX_N_AXIS = 6;

// Number of axes defined (steppers, servos, etc) (valid range: 3 to 6)
// Even if your machine only uses less than the minimum of 3, you should select 3


#ifndef LIMIT_MASK
#    define LIMIT_MASK B0
#endif

// Serial baud rate
// OK to change, but the ESP32 boot text is 115200, so you will not see that is your
// serial monitor, sender, etc uses a different value than 115200
#define BAUD_RATE 9600

//Connect to your local AP with these credentials
//#define CONNECT_TO_SSID  "your SSID"
//#define SSID_PASSWORD  "your SSID password"
//CONFIGURE_EYECATCH_BEGIN (DO NOT MODIFY THIS LINE)

//#define ENABLE_WIFI  //enable wifi
//#define ENABLE_HTTP                //enable HTTP and all related services
//#define ENABLE_OTA                 //enable OTA
//#define ENABLE_MDNS                //enable mDNS discovery
//#define ENABLE_SSDP                //enable UPNP discovery
//#define ENABLE_NOTIFICATIONS       //enable notifications


// Captive portal is used when WiFi is in access point mode.  It lets the
// WebUI come up automatically in the browser, instead of requiring the user
// to browse manually to a default URL.  It works like airport and hotel
// WiFi that takes you a special page as soon as you connect to that AP.
#define ENABLE_CAPTIVE_PORTAL

// Warning! The current authentication implementation is too weak to provide
// security against an attacker, since passwords are stored and transmitted
// "in the clear" over unsecured channels.  It should be treated as a
// "friendly suggestion" to prevent unwitting dangerous actions, rather than
// as effective security against malice.
// #define ENABLE_AUTHENTICATION
//CONFIGURE_EYECATCH_END (DO NOT MODIFY THIS LINE)

//Radio Mode
const int ESP_RADIO_OFF = 0;
const int ESP_WIFI_STA  = 1;
const int ESP_WIFI_AP   = 2;
const int ESP_BT        = 3;

//Default mode

#    undef ENABLE_NOTIFICATIONS
     const int DEFAULT_RADIO_MODE = ESP_RADIO_OFF;


// Define realtime command special characters. These characters are 'picked-off' directly from the
// serial read data stream and are not passed to the grbl line execution parser. Select characters
// that do not and must not exist in the streamed GCode program. ASCII control characters may be
// used, if they are available per user setup. Also, extended ASCII codes (>127), which are never in
// GCode programs, maybe selected for interface programs.
// NOTE: If changed, manually update help message in report.c.

// NOTE: All override realtime commands must be in the extended ASCII character set, starting
// at character value 128 (0x80) and up to 255 (0xFF). If the normal set of realtime commands,
// such as status reports, feed hold, reset, and cycle start, are moved to the extended set
// space, serial.c's RX ISR will need to be modified to accommodate the change.

enum class Cmd : uint8_t {
    Reset                 = 0x18,  // Ctrl-X
    StatusReport          = '?',
    CycleStart            = '~',
    JogCancel             = 0x85,
    DebugReport           = 0x86,  // Only when DEBUG enabled, sends debug report in '{}' braces.
    FeedOvrReset          = 0x90,  // Restores feed override value to 100%.
    FeedOvrCoarsePlus     = 0x91,
    FeedOvrCoarseMinus    = 0x92,
    FeedOvrFinePlus       = 0x93,
    FeedOvrFineMinus      = 0x94,
    RapidOvrReset         = 0x95,  // Restores rapid override value to 100%.
    RapidOvrMedium        = 0x96,
    RapidOvrLow           = 0x97,
    RapidOvrExtraLow      = 0x98,  // *NOT SUPPORTED*
    SpindleOvrReset       = 0x99,  // Restores spindle override value to 100%.
    SpindleOvrCoarsePlus  = 0x9A,  // 154
    SpindleOvrCoarseMinus = 0x9B,
    SpindleOvrFinePlus    = 0x9C,
    SpindleOvrFineMinus   = 0x9D,
    SpindleOvrStop        = 0x9E,
    FlushRetract          = 0x87,
    ShortRetract          = 0x88,
    DefaultWait           = 0x89,
    Probe                 = 0x8a,
};

// If homing is enabled, homing init lock sets Grbl into an alarm state upon power up. This forces
// the user to perform the homing cycle (or override the locks) before doing anything else. This is
// mainly a safety feature to remind the user to home, since position is unknown to Grbl.
#define HOMING_INIT_LOCK  // Comment to disable

// Number of homing cycles performed after when the machine initially jogs to limit switches.
// This help in preventing overshoot and should improve repeatability. This value should be one or
// greater.
static const uint8_t NHomingLocateCycle = 1;  // Integer (1-128)

// Enables single axis homing commands. $HX, $HY, and $HZ for X, Y, and Z-axis homing. The full homing
// cycle is still invoked by the $H command. This is disabled by default. It's here only to address
// users that need to switch between a two-axis and three-axis machine. This is actually very rare.
// If you have a two-axis machine, DON'T USE THIS. Instead, just alter the homing cycle for two-axes.
#define HOMING_SINGLE_AXIS_COMMANDS  // Default disabled. Uncomment to enable.

// Number of blocks Grbl executes upon startup. These blocks are stored in non-volatile storage.
// and addresses are defined in settings.h. With the current settings, up to 2 startup blocks may
// be stored and executed in order. These startup blocks would typically be used to set the GCode
// parser state depending on user preferences.
#define N_STARTUP_LINE 2  // Integer (1-2)

// Number of floating decimal points printed by Grbl for certain value types. These settings are
// determined by realistic and commonly observed values in CNC machines. For example, position
// values cannot be less than 0.001mm or 0.0001in, because machines can not be physically more
// precise this. So, there is likely no need to change these, but you can if you need to here.
// NOTE: Must be an integer value from 0 to ~4. More than 4 may exhibit round-off errors.
// ESP32 Note: These are mostly hard coded, so these values will not change anything

// If your machine has two limits switches wired in parallel to one axis, you will need to enable
// this feature. Since the two switches are sharing a single pin, there is no way for Grbl to tell
// which one is enabled. This option only effects homing, where if a limit is engaged, Grbl will
// alarm out and force the user to manually disengage the limit switch. Otherwise, if you have one
// limit switch for each axis, don't enable this option. By keeping it disabled, you can perform a
// homing cycle while on the limit switch and not have to move the machine off of it.
// #define LIMITS_TWO_SWITCHES_ON_AXES

// Allows GRBL to track and report gcode line numbers.  Enabling this means that the planning buffer
// goes from 16 to 15 to make room for the additional line number data in the plan_block_t struct
// #define USE_LINE_NUMBERS // Disabled by default. Uncomment to enable.

// Upon a successful probe cycle, this option provides immediately feedback of the probe coordinates
// through an automatically generated message. If disabled, users can still access the last probe
// coordinates through Grbl '$#' print parameters.
//#define MESSAGE_PROBE_COORDINATES  // Enabled by default. Comment to disable.


// This option causes the feed hold input to act as a safety door switch. A safety door, when triggered,
// immediately forces a feed hold and then safely de-energizes the machine. Resuming is blocked until
// the safety door is re-engaged. When it is, Grbl will re-energize the machine and then resume on the
// previous tool path, as if nothing happened.
#define ENABLE_SAFETY_DOOR_INPUT_PIN  // ESP32 Leave this enabled for now .. code for undefined not ready

// Inverts select limit pin states based on the following mask. This effects all limit pin functions,
// such as hard limits and homing. However, this is different from overall invert limits setting.
// This build option will invert only the limit pins defined here, and then the invert limits setting
// will be applied to all of them. This is useful when a user has a mixed set of limit pins with both
// normally-open(NO) and normally-closed(NC) switches installed on their machine.
// NOTE: PLEASE DO NOT USE THIS, unless you have a situation that needs it.
// #define INVERT_LIMIT_PIN_MASK (bit(X_AXIS)|bit(Y_AXIS)) // Default disabled. Uncomment to enable.


// When Grbl powers-cycles or is hard reset with the Arduino reset button, Grbl boots up with no ALARM
// by default. This is to make it as simple as possible for new users to start using Grbl. When homing
// is enabled and a user has installed limit switches, Grbl will boot up in an ALARM state to indicate
// Grbl doesn't know its position and to force the user to home before proceeding. This option forces
// Grbl to always initialize into an ALARM state regardless of homing or not. This option is more for
// OEMs and LinuxCNC users that would like this power-cycle behavior.
// #define FORCE_INITIALIZATION_ALARM // Default disabled. Uncomment to enable.

// At power-up or a reset, Grbl will check the limit switch states to ensure they are not active
// before initialization. If it detects a problem and the hard limits setting is enabled, Grbl will
// simply message the user to check the limits and enter an alarm state, rather than idle. Grbl will
// not throw an alarm message.
//#define CHECK_LIMITS_AT_INIT

// ---------------------------------------------------------------------------------------
// ADVANCED CONFIGURATION OPTIONS:

// Enables code for debugging purposes. Not for general use and always in constant flux.
// #define DEBUG // Uncomment to enable. Default disabled.

// Configure rapid, feed, and spindle override settings. These values define the max and min
// allowable override values and the coarse and fine increments per command received. Please
// note the allowable values in the descriptions following each define.
namespace FeedOverride {
    const int Default         = 100;  // 100%. Don't change this value.
    const int Max             = 200;  // Percent of programmed feed rate (100-255). Usually 120% or 200%
    const int Min             = 10;   // Percent of programmed feed rate (1-100). Usually 50% or 1%
    const int CoarseIncrement = 10;   // (1-99). Usually 10%.
    const int FineIncrement   = 1;    // (1-99). Usually 1%.
};
namespace RapidOverride {
    const int Default  = 100;  // 100%. Don't change this value.
    const int Medium   = 50;   // Percent of rapid (1-99). Usually 50%.
    const int Low      = 25;   // Percent of rapid (1-99). Usually 25%.
    const int ExtraLow = 5;    // Percent of rapid (1-99). Usually 5%.  Not Supported
};

namespace SpindleSpeedOverride {
    const int Default         = 100;  // 100%. Don't change this value.
    const int Max             = 200;  // Percent of programmed spindle speed (100-255). Usually 200%.
    const int Min             = 10;   // Percent of programmed spindle speed (1-100). Usually 10%.
    const int CoarseIncrement = 10;   // (1-99). Usually 10%.
    const int FineIncrement   = 1;    // (1-99). Usually 1%.
}

// When a M2 or M30 program end command is executed, most GCode states are restored to their defaults.
// This compile-time option includes the restoring of the feed, rapid, and spindle speed override values
// to their default values at program end.
#define RESTORE_OVERRIDES_AFTER_PROGRAM_END  // Default enabled. Comment to disable.

// The status report change for Grbl v1.1 and after also removed the ability to disable/enable most data
// fields from the report. This caused issues for GUI developers, who've had to manage several scenarios
// and configurations. The increased efficiency of the new reporting style allows for all data fields to
// be sent without potential performance issues.
// NOTE: The options below are here only provide a way to disable certain data fields if a unique
// situation demands it, but be aware GUIs may depend on this data. If disabled, it may not be compatible.
#define REPORT_FIELD_BUFFER_STATE        // Default enabled. Comment to disable.
#define REPORT_FIELD_PIN_STATE           // Default enabled. Comment to disable.
#define REPORT_FIELD_CURRENT_FEED_SPEED  // Default enabled. Comment to disable.
#define REPORT_FIELD_WORK_COORD_OFFSET   // Default enabled. Comment to disable.
#define REPORT_FIELD_OVERRIDES           // Default enabled. Comment to disable.
#define REPORT_FIELD_LINE_NUMBERS        // Default enabled. Comment to disable.

// Some status report data isn't necessary for realtime, only intermittently, because the values don't
// change often. The following macros configures how many times a status report needs to be called before
// the associated data is refreshed and included in the status report. However, if one of these value
// changes, Grbl will automatically include this data in the next status report, regardless of what the
// count is at the time. This helps reduce the communication overhead involved with high frequency reporting
// and agressive streaming. There is also a busy and an idle refresh count, which sets up Grbl to send
// refreshes more often when its not doing anything important. With a good GUI, this data doesn't need
// to be refreshed very often, on the order of a several seconds.
// NOTE: WCO refresh must be 2 or greater. OVR refresh must be 1 or greater.
const int REPORT_OVR_REFRESH_BUSY_COUNT = 20;  // (1-255)
const int REPORT_OVR_REFRESH_IDLE_COUNT = 10;  // (1-255) Must be less than or equal to the busy count
const int REPORT_WCO_REFRESH_BUSY_COUNT = 30;  // (2-255)
const int REPORT_WCO_REFRESH_IDLE_COUNT = 10;  // (2-255) Must be less than or equal to the busy count

// The temporal resolution of the acceleration management subsystem. A higher number gives smoother
// acceleration, particularly noticeable on machines that run at very high feedrates, but may negatively
// impact performance. The correct value for this parameter is machine dependent, so it's advised to
// set this only as high as needed. Approximate successful values can widely range from 50 to 200 or more.
// NOTE: Changing this value also changes the execution time of a segment in the step segment buffer.
// When increasing this value, this stores less overall time in the segment buffer and vice versa. Make
// certain the step segment buffer is increased/decreased to account for these changes.
const int ACCELERATION_TICKS_PER_SECOND = 100;

// Sets the maximum step rate allowed to be written as a Grbl setting. This option enables an error
// check in the settings module to prevent settings values that will exceed this limitation. The maximum
// step rate is strictly limited by the CPU speed and will change if something other than an AVR running
// at 16MHz is used.
// NOTE: For now disabled, will enable if flash space permits.
// #define MAX_STEP_RATE_HZ 30000 // Hz

// By default, Grbl sets all input pins to normal-high operation with their internal pull-up resistors
// enabled. This simplifies the wiring for users by requiring only a switch connected to ground,
// although its recommended that users take the extra step of wiring in low-pass filter to reduce
// electrical noise detected by the pin. If the user inverts the pin in Grbl settings, this just flips
// which high or low reading indicates an active signal. In normal operation, this means the user
// needs to connect a normal-open switch, but if inverted, this means the user should connect a
// normal-closed switch.
// The following options disable the internal pull-up resistors, sets the pins to a normal-low
// operation, and switches must be now connect to Vcc instead of ground. This also flips the meaning
// of the invert pin Grbl setting, where an inverted setting now means the user should connect a
// normal-open switch and vice versa.
// NOTE: All pins associated with the feature are disabled, i.e. XYZ limit pins, not individual axes.
// WARNING: When the pull-ups are disabled, this requires additional wiring with pull-down resistors!
//#define DISABLE_LIMIT_PIN_PULL_UP
//#define DISABLE_PROBE_PIN_PULL_UP
//#define DISABLE_CONTROL_PIN_PULL_UP

// Sets which axis the tool length offset is applied. Assumes the spindle is always parallel with
// the selected axis with the tool oriented toward the negative direction. In other words, a positive
// tool length offset value is subtracted from the current location.
const int TOOL_LENGTH_OFFSET_AXIS = Z_AXIS;  // Default z-axis. Valid values are X_AXIS, Y_AXIS, or Z_AXIS.

// With this enabled, Grbl sends back an echo of the line it has received, which has been pre-parsed (spaces
// removed, capitalized letters, no comments) and is to be immediately executed by Grbl. Echoes will not be
// sent upon a line buffer overflow, but should for all normal lines sent to Grbl. For example, if a user
// sendss the line 'g1 x1.032 y2.45 (test comment)', Grbl will echo back in the form '[echo: G1X1.032Y2.45]'.
// Only GCode lines are echoed, not command lines starting with $ or [ESP.
// NOTE: Only use this for debugging purposes!! When echoing, this takes up valuable resources and can effect
// performance. If absolutely needed for normal operation, the serial write buffer should be greatly increased
// to help minimize transmission waiting within the serial write protocol.
//#define REPORT_ECHO_LINE_RECEIVED // Default disabled. Uncomment to enable.

// This is similar to REPORT_ECHO_LINE_RECEIVED and subject to all its caveats,
// but instead of echoing the pre-parsed line, it echos the raw line exactly as
// received, including not only GCode lines, but also $ and [ESP commands.
//#define REPORT_ECHO_RAW_LINE_RECEIVED // Default disabled. Uncomment to enable.

// Minimum planner junction speed. Sets the default minimum junction speed the planner plans to at
// every buffer block junction, except for starting from rest and end of the buffer, which are always
// zero. This value controls how fast the machine moves through junctions with no regard for acceleration
// limits or angle between neighboring block line move directions. This is useful for machines that can't
// tolerate the tool dwelling for a split second, i.e. 3d printers or laser cutters. If used, this value
// should not be much greater than zero or to the minimum value necessary for the machine to work.
const double MINIMUM_JUNCTION_SPEED = 0.0;  // (mm/min)

// Sets the minimum feed rate the planner will allow. Any value below it will be set to this minimum
// value. This also ensures that a planned motion always completes and accounts for any floating-point
// round-off errors. Although not recommended, a lower value than 1.0 mm/min will likely work in smaller
// machines, perhaps to 0.1mm/min, but your success may vary based on multiple factors.
const double MINIMUM_FEED_RATE = 1.0;  // (mm/min)

// Number of arc generation iterations by small angle approximation before exact arc trajectory
// correction with expensive sin() and cos() calcualtions. This parameter maybe decreased if there
// are issues with the accuracy of the arc generations, or increased if arc execution is getting
// bogged down by too many trig calculations.
const int N_ARC_CORRECTION = 12;  // Integer (1-255)

// The arc G2/3 GCode standard is problematic by definition. Radius-based arcs have horrible numerical
// errors when arc at semi-circles(pi) or full-circles(2*pi). Offset-based arcs are much more accurate
// but still have a problem when arcs are full-circles (2*pi). This define accounts for the floating
// point issues when offset-based arcs are commanded as full circles, but get interpreted as extremely
// small arcs with around machine epsilon (1.2e-7rad) due to numerical round-off and precision issues.
// This define value sets the machine epsilon cutoff to determine if the arc is a full-circle or not.
// NOTE: Be very careful when adjusting this value. It should always be greater than 1.2e-7 but not too
// much greater than this. The default setting should capture most, if not all, full arc error situations.
const double ARC_ANGULAR_TRAVEL_EPSILON = 5E-7;  // Float (radians)

// Time delay increments performed during a dwell. The default value is set at 50ms, which provides
// a maximum time delay of roughly 55 minutes, more than enough for most any application. Increasing
// this delay will increase the maximum dwell time linearly, but also reduces the responsiveness of
// run-time command executions, like status reports, since these are performed between each dwell
// time step. Also, keep in mind that the Arduino delay timer is not very accurate for long delays.
const int DWELL_TIME_STEP = 50;  // Integer (1-255) (milliseconds)

// For test use only. This uses the ESP32's RMT peripheral to generate step pulses
// It allows the use of the STEP_PULSE_DELAY (see below) and it automatically ends the
// pulse in one operation.
// Dir Pin  ____|--------------------
// Step Pin _______|--|____________
// While this is experimental, it is intended to be the future default method after testing
//#define USE_RMT_STEPS

// STEP_PULSE_DELAY is now a setting...$Stepper/Direction/Delay

// The number of linear motions in the planner buffer to be planned at any give time. The vast
// majority of RAM that Grbl uses is based on this buffer size. Only increase if there is extra
// available RAM, like when re-compiling for a Mega2560. Or decrease if the Arduino begins to
// crash due to the lack of available RAM or if the CPU is having trouble keeping up with planning
// new incoming motions as they are executed.
// #define BLOCK_BUFFER_SIZE 16 // Uncomment to override default in planner.h.

// Governs the size of the intermediary step segment buffer between the step execution algorithm
// and the planner blocks. Each segment is set of steps executed at a constant velocity over a
// fixed time defined by ACCELERATION_TICKS_PER_SECOND. They are computed such that the planner
// block velocity profile is traced exactly. The size of this buffer governs how much step
// execution lead time there is for other Grbl processes have to compute and do their thing
// before having to come back and refill this buffer, currently at ~50msec of step moves.
// #define SEGMENT_BUFFER_SIZE 6 // Uncomment to override default in stepper.h.

// Line buffer size from the serial input stream to be executed. Also, governs the size of
// each of the startup blocks, as they are each stored as a string of this size.
// NOTE: 80 characters is not a problem except for extreme cases, but the line buffer size
// can be too small and GCode blocks can get truncated. Officially, the GCode standards
// support up to 256 characters.
// #define LINE_BUFFER_SIZE 80  // Uncomment to override default in protocol.h

// Serial send and receive buffer size. The receive buffer is often used as another streaming
// buffer to store incoming blocks to be processed by Grbl when its ready. Most streaming
// interfaces will character count and track each block send to each block response. So,
// increase the receive buffer if a deeper receive buffer is needed for streaming and avaiable
// memory allows. The send buffer primarily handles messages in Grbl. Only increase if large
// messages are sent and Grbl begins to stall, waiting to send the rest of the message.
// NOTE: Grbl generates an average status report in about 0.5msec, but the serial TX stream at
// 115200 baud will take 5 msec to transmit a typical 55 character report. Worst case reports are
// around 90-100 characters. As long as the serial TX buffer doesn't get continually maxed, Grbl
// will continue operating efficiently. Size the TX buffer around the size of a worst-case report.
// #define RX_BUFFER_SIZE 128 // (1-254) Uncomment to override defaults in serial.h
// #define TX_BUFFER_SIZE 100 // (1-254)

// A simple software debouncing feature for hard limit switches. When enabled, the limit
// switch interrupt unblock a waiting task which will recheck the limit switch pins after
// a short delay. Default disabled
//#define ENABLE_SOFTWARE_DEBOUNCE // Default disabled. Uncomment to enable.
const int DEBOUNCE_PERIOD = 32;  // in milliseconds default 32 microseconds

// Configures the position after a probing cycle during Grbl's check mode. Disabled sets
// the position to the probe target, when enabled sets the position to the start position.
// #define SET_CHECK_MODE_PROBE_TO_START // Default disabled. Uncomment to enable.

// Force Grbl to check the state of the hard limit switches when the processor detects a pin
// change inside the hard limit ISR routine. By default, Grbl will trigger the hard limits
// alarm upon any pin change, since bouncing switches can cause a state check like this to
// misread the pin. When hard limits are triggered, they should be 100% reliable, which is the
// reason that this option is disabled by default. Only if your system/electronics can guarantee
// that the switches don't bounce, we recommend enabling this option. This will help prevent
// triggering a hard limit when the machine disengages from the switch.
// NOTE: This option has no effect if SOFTWARE_DEBOUNCE is enabled.
// #define HARD_LIMIT_FORCE_STATE_CHECK // Default disabled. Uncomment to enable.

// Adjusts homing cycle search and locate scalars. These are the multipliers used by Grbl's
// homing cycle to ensure the limit switches are engaged and cleared through each phase of
// the cycle. The search phase uses the axes max-travel setting times the SEARCH_SCALAR to
// determine distance to look for the limit switch. Once found, the locate phase begins and
// uses the homing pull-off distance setting times the LOCATE_SCALAR to pull-off and re-engage
// the limit switch.
// NOTE: Both of these values must be greater than 1.0 to ensure proper function.
// #define HOMING_AXIS_SEARCH_SCALAR  1.5 // Uncomment to override defaults in limits.c.
// #define HOMING_AXIS_LOCATE_SCALAR  10.0 // Uncomment to override defaults in limits.c.

// Enable the '$RST=*', '$RST=$', and '$RST=#' eeprom restore commands. There are cases where
// these commands may be undesirable. Simply comment the desired macro to disable it.
#define ENABLE_RESTORE_WIPE_ALL          // '$RST=*' Default enabled. Comment to disable.
#define ENABLE_RESTORE_DEFAULT_SETTINGS  // '$RST=$' Default enabled. Comment to disable.
#define ENABLE_RESTORE_CLEAR_PARAMETERS  // '$RST=#' Default enabled. Comment to disable.

// Additional settings have been added to the original set that you see with the $$ command
// Some senders may not be able to parse anything different from the original set
// You can still set these like $33=5000, but you cannot read them back.
// Default is off to limit support issues...you can enable here or in your machine definition file
// #define SHOW_EXTENDED_SETTINGS

// Writing to non-volatile storage (NVS) can take a long time and interfere with timely instruction
// execution, causing problems for the stepper ISRs and serial comm ISRs and subsequent loss of
// stepper position and serial data. This configuration option forces the planner buffer to completely
// empty whenever the NVS is written, to prevent any chance of lost steps.
// It doesn't prevent loss of serial Rx data, especially if a GUI is premptively filling up the
// serial Rx buffer.  GUIs should detect GCodes that write to NVS - notably G10,G28.1,G30.1 -
// and wait for an 'ok' before sending more data.
// NOTE: Most setting changes - $ commands - are blocked when a job is running. Coordinate setting
// GCode commands (G10,G28/30.1) are not blocked, since they are part of an active streaming job.
// This option forces a planner buffer sync only with such GCode commands.
#define FORCE_BUFFER_SYNC_DURING_NVS_WRITE  // Default enabled. Comment to disable.

// In Grbl v0.9 and prior, there is an old outstanding bug where the `WPos:` work position reported
// may not correlate to what is executing, because `WPos:` is based on the GCode parser state, which
// can be several motions behind. This option forces the planner buffer to empty, sync, and stop
// motion whenever there is a command that alters the work coordinate offsets `G10,G43.1,G92,G54-59`.
// This is the simplest way to ensure `WPos:` is always correct. Fortunately, it's exceedingly rare
// that any of these commands are used need continuous motions through them.
#define FORCE_BUFFER_SYNC_DURING_WCO_CHANGE  // Default enabled. Comment to disable.

// By default, Grbl disables feed rate overrides for all G38.x probe cycle commands. Although this
// may be different than some pro-class machine control, it's arguable that it should be this way.
// Most probe sensors produce different levels of error that is dependent on rate of speed. By
// keeping probing cycles to their programmed feed rates, the probe sensor should be a lot more
// repeatable. If needed, you can disable this behavior by uncommenting the define below.
// #define ALLOW_FEED_OVERRIDE_DURING_PROBE_CYCLES // Default disabled. Uncomment to enable.



// This option will automatically disable the laser during a feed hold by invoking a spindle stop
// override immediately after coming to a stop. However, this also means that the laser still may
// be reenabled by disabling the spindle stop override, if needed. This is purely a safety feature
// to ensure the laser doesn't inadvertently remain powered while at a stop and cause a fire.
//#define DISABLE_LASER_DURING_HOLD  // Default enabled. Comment to disable.

// Enables a piecewise linear model of the spindle PWM/speed output. Requires a solution by the
// 'fit_nonlinear_spindle.py' script in the /doc/script folder of the repo. See file comments
// on how to gather spindle data and run the script to generate a solution.
// #define ENABLE_PIECEWISE_LINEAR_SPINDLE  // Default disabled. Uncomment to enable.

// N_PIECES, RPM_MAX, RPM_MIN, RPM_POINTxx, and RPM_LINE_XX constants are all set and given by
// the 'fit_nonlinear_spindle.py' script solution. Used only when ENABLE_PIECEWISE_LINEAR_SPINDLE
// is enabled. Make sure the constant values are exactly the same as the script solution.
// NOTE: When N_PIECES < 4, unused RPM_LINE and RPM_POINT defines are not required and omitted.
/*
#define N_PIECES 4  // Integer (1-4). Number of piecewise lines used in script solution.
#define RPM_MAX  11686.4  // Max RPM of model. $30 > RPM_MAX will be limited to RPM_MAX.
#define RPM_MIN  202.5    // Min RPM of model. $31 < RPM_MIN will be limited to RPM_MIN.
*/

const int N_PIECES = 3;

const double RPM_MAX = 23935.2;
const double RPM_MIN = 2412.2;





// Grbl generic default settings. Should work across different machines.
#ifndef DEFAULT_STEP_PULSE_MICROSECONDS
#    define DEFAULT_STEP_PULSE_MICROSECONDS 3  // $0
#endif

#ifndef DEFAULT_STEP_ENABLE_DELAY
#    define DEFAULT_STEP_ENABLE_DELAY 0
#endif

#ifndef STEP_PULSE_DELAY
#    define STEP_PULSE_DELAY 0
#endif

#ifndef DEFAULT_STEPPER_IDLE_LOCK_TIME
#    define DEFAULT_STEPPER_IDLE_LOCK_TIME 250  // $1 msec (0-254, 255 keeps steppers enabled)
#endif

#ifndef DEFAULT_STEPPING_INVERT_MASK
#    define DEFAULT_STEPPING_INVERT_MASK 0  // $2 uint8_t
#endif

#ifndef DEFAULT_DIRECTION_INVERT_MASK
#    define DEFAULT_DIRECTION_INVERT_MASK 0  // $3 uint8_
#endif

#ifndef DEFAULT_INVERT_ST_ENABLE
#    define DEFAULT_INVERT_ST_ENABLE 0  // $4 boolean
#endif

#ifndef DEFAULT_INVERT_LIMIT_PINS
#    define DEFAULT_INVERT_LIMIT_PINS 1  // $5 boolean
#endif

#ifndef DEFAULT_INVERT_PROBE_PIN
#    define DEFAULT_INVERT_PROBE_PIN 0  // $6 boolean
#endif

#ifndef DEFAULT_STATUS_REPORT_MASK
#    define DEFAULT_STATUS_REPORT_MASK 1  // $10
#endif

#ifndef DEFAULT_VERBOSE_ERRORS
#    define DEFAULT_VERBOSE_ERRORS 0
#endif

#ifndef DEFAULT_JUNCTION_DEVIATION
#    define DEFAULT_JUNCTION_DEVIATION 0.01  // $11 mm
#endif

#ifndef DEFAULT_ARC_TOLERANCE
#    define DEFAULT_ARC_TOLERANCE 0.002  // $12 mm
#endif

#ifndef DEFAULT_REPORT_INCHES
#    define DEFAULT_REPORT_INCHES 0  // $13 false
#endif

#ifndef DEFAULT_SOFT_LIMIT_ENABLE
#    define DEFAULT_SOFT_LIMIT_ENABLE 0  // $20 false
#endif

#ifndef DEFAULT_HARD_LIMIT_ENABLE
#    define DEFAULT_HARD_LIMIT_ENABLE 0  // $21 false
#endif

#ifndef DEFAULT_HOMING_ENABLE
#    define DEFAULT_HOMING_ENABLE 0  // $22 false
#endif

#ifndef DEFAULT_HOMING_DIR_MASK
#    define DEFAULT_HOMING_DIR_MASK 3  // $23 move positive dir Z, negative X,Y
#endif

#ifndef DEFAULT_HOMING_FEED_RATE
#    define DEFAULT_HOMING_FEED_RATE 200.0  // $24 mm/min
#endif

#ifndef DEFAULT_HOMING_SEEK_RATE
#    define DEFAULT_HOMING_SEEK_RATE 2000.0  // $25 mm/min
#endif

#ifndef DEFAULT_HOMING_DEBOUNCE_DELAY
#    define DEFAULT_HOMING_DEBOUNCE_DELAY 250  // $26 msec (0-65k)
#endif

#ifndef DEFAULT_HOMING_PULLOFF
#    define DEFAULT_HOMING_PULLOFF 1.0  // $27 mm
#endif

#ifndef DEFAULT_HOMING_SQUARED_AXES
#    define DEFAULT_HOMING_SQUARED_AXES 0
#endif

#ifndef DEFAULT_HOMING_CYCLE_0
#    define DEFAULT_HOMING_CYCLE_0 bit(Z_AXIS)
#endif

#ifndef DEFAULT_HOMING_CYCLE_1
#    define DEFAULT_HOMING_CYCLE_1 (bit(X_AXIS) | bit(Y_AXIS))
#endif

#ifndef DEFAULT_HOMING_CYCLE_2
#    define DEFAULT_HOMING_CYCLE_2 0
#endif

#ifndef DEFAULT_HOMING_CYCLE_3
#    define DEFAULT_HOMING_CYCLE_3 0
#endif

#ifndef DEFAULT_HOMING_CYCLE_4
#    define DEFAULT_HOMING_CYCLE_4 0
#endif

#ifndef DEFAULT_HOMING_CYCLE_5
#    define DEFAULT_HOMING_CYCLE_5 0
#endif

// ======== SPINDLE STUFF ====================
#ifndef SPINDLE_TYPE
//#    define SPINDLE_TYPE SpindleType::NONE
#endif

#ifndef DEFAULT_SPINDLE_RPM_MIN          // $31
#    define DEFAULT_SPINDLE_RPM_MIN 0.0  // rpm
#endif

#ifndef DEFAULT_LASER_MODE        // $32
#    define DEFAULT_LASER_MODE 0  // false
#endif

#ifndef DEFAULT_LASER_FULL_POWER
#    define DEFAULT_LASER_FULL_POWER 1000
#endif

#ifndef DEFAULT_SPINDLE_RPM_MAX             // $30
#    define DEFAULT_SPINDLE_RPM_MAX 1000.0  // rpm
#endif

#ifndef DEFAULT_SPINDLE_FREQ
#    define DEFAULT_SPINDLE_FREQ 5000.0  // $33 Hz (extended set)
#endif

#ifndef DEFAULT_SPINDLE_OFF_VALUE
#    define DEFAULT_SPINDLE_OFF_VALUE 0.0  // $34 Percent of full period(extended set)
#endif

#ifndef DEFAULT_SPINDLE_MIN_VALUE
#    define DEFAULT_SPINDLE_MIN_VALUE 0.0  // $35 Percent of full period (extended set)
#endif

#ifndef DEFAULT_SPINDLE_MAX_VALUE
#    define DEFAULT_SPINDLE_MAX_VALUE 100.0  // $36 Percent of full period (extended set)
#endif

#ifndef DEFAULT_SPINDLE_DELAY_SPINUP
#    define DEFAULT_SPINDLE_DELAY_SPINUP 0
#endif


#ifndef DEFAULT_SPINDLE_DELAY_SPINDOWN
#    define DEFAULT_SPINDLE_DELAY_SPINDOWN 0
#endif

#ifndef DEFAULT_INVERT_SPINDLE_OUTPUT_PIN
#    define DEFAULT_INVERT_SPINDLE_OUTPUT_PIN 0
#endif

#ifndef DEFAULT_INVERT_SPINDLE_ENABLE_PIN
#    define DEFAULT_INVERT_SPINDLE_ENABLE_PIN 0
#endif

#ifndef DEFAULT_SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED
#    define DEFAULT_SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED 0
#endif

// ================  user settings =====================
#ifndef DEFAULT_USER_INT_80
#    define DEFAULT_USER_INT_80 0  // $80 User integer setting
#endif

#ifndef DEFAULT_USER_INT_81
#    define DEFAULT_USER_INT_81 0  // $81 User integer setting
#endif

#ifndef DEFAULT_USER_INT_82
#    define DEFAULT_USER_INT_82 0  // $82 User integer setting
#endif

#ifndef DEFAULT_USER_INT_83
#    define DEFAULT_USER_INT_83 0  // $83 User integer setting
#endif

#ifndef DEFAULT_USER_INT_84
#    define DEFAULT_USER_INT_84 0  // $84 User integer setting
#endif

#ifndef DEFAULT_USER_FLOAT_90
#    define DEFAULT_USER_FLOAT_90 0.0  // $90 User integer setting
#endif

#ifndef DEFAULT_USER_FLOAT_91
#    define DEFAULT_USER_FLOAT_91 0.0  // $92 User integer setting
#endif

#ifndef DEFAULT_USER_FLOAT_92
#    define DEFAULT_USER_FLOAT_92 0.0  // $92 User integer setting
#endif

#ifndef DEFAULT_USER_FLOAT_93
#    define DEFAULT_USER_FLOAT_93 0.0  // $93 User integer setting
#endif

#ifndef DEFAULT_USER_FLOAT_94
#    define DEFAULT_USER_FLOAT_94 0.0  // $94 User integer setting
#endif

// =========== 	AXIS RESOLUTION ======

#ifndef DEFAULT_X_STEPS_PER_MM
#    define DEFAULT_X_STEPS_PER_MM 100.0
#endif
#ifndef DEFAULT_Y_STEPS_PER_MM
#    define DEFAULT_Y_STEPS_PER_MM 100.0
#endif
#ifndef DEFAULT_Z_STEPS_PER_MM
#    define DEFAULT_Z_STEPS_PER_MM 100.0
#endif
#ifndef DEFAULT_A_STEPS_PER_MM
#    define DEFAULT_A_STEPS_PER_MM 100.0
#endif
#ifndef DEFAULT_B_STEPS_PER_MM
#    define DEFAULT_B_STEPS_PER_MM 100.0
#endif
#ifndef DEFAULT_C_STEPS_PER_MM
#    define DEFAULT_C_STEPS_PER_MM 100.0
#endif

// ============ AXIS MAX SPPED =========

#ifndef DEFAULT_X_MAX_RATE
#    define DEFAULT_X_MAX_RATE 1000.0  // mm/min
#endif
#ifndef DEFAULT_Y_MAX_RATE
#    define DEFAULT_Y_MAX_RATE 1000.0  // mm/min
#endif
#ifndef DEFAULT_Z_MAX_RATE
#    define DEFAULT_Z_MAX_RATE 1000.0  // mm/min
#endif
#ifndef DEFAULT_A_MAX_RATE
#    define DEFAULT_A_MAX_RATE 1000.0  // mm/min
#endif
#ifndef DEFAULT_B_MAX_RATE
#    define DEFAULT_B_MAX_RATE 1000.0  // mm/min
#endif
#ifndef DEFAULT_C_MAX_RATE
#    define DEFAULT_C_MAX_RATE 1000.0  // mm/min
#endif

// ============== Axis Acceleration =========
#define SEC_PER_MIN_SQ (60.0 * 60.0)  // Seconds Per Minute Squared, for acceleration conversion
// Default accelerations are expressed in mm/sec^2
#ifndef DEFAULT_X_ACCELERATION
#    define DEFAULT_X_ACCELERATION 200.0
#endif
#ifndef DEFAULT_Y_ACCELERATION
#    define DEFAULT_Y_ACCELERATION 200.0
#endif
#ifndef DEFAULT_Z_ACCELERATION
#    define DEFAULT_Z_ACCELERATION 200.0
#endif
#ifndef DEFAULT_A_ACCELERATION
#    define DEFAULT_A_ACCELERATION 200.0
#endif
#ifndef DEFAULT_B_ACCELERATION
#    define DEFAULT_B_ACCELERATION 200.0
#endif
#ifndef DEFAULT_C_ACCELERATION
#    define DEFAULT_C_ACCELERATION 200.0
#endif

// ========= AXIS MAX TRAVEL ============

#ifndef DEFAULT_X_MAX_TRAVEL
#    define DEFAULT_X_MAX_TRAVEL 300.0  // $130 mm NOTE: Must be a positive value.
#endif
#ifndef DEFAULT_Y_MAX_TRAVEL
#    define DEFAULT_Y_MAX_TRAVEL 300.0  // mm NOTE: Must be a positive value.
#endif
#ifndef DEFAULT_Z_MAX_TRAVEL
#    define DEFAULT_Z_MAX_TRAVEL 300.0  // mm NOTE: Must be a positive value.
#endif
#ifndef DEFAULT_A_MAX_TRAVEL
#    define DEFAULT_A_MAX_TRAVEL 300.0  // mm NOTE: Must be a positive value.
#endif
#ifndef DEFAULT_B_MAX_TRAVEL
#    define DEFAULT_B_MAX_TRAVEL 300.0  // mm NOTE: Must be a positive value.
#endif
#ifndef DEFAULT_C_MAX_TRAVEL
#    define DEFAULT_C_MAX_TRAVEL 300.0  // mm NOTE: Must be a positive value.
#endif

#ifndef DEFAULT_X_HOMING_MPOS
#    define DEFAULT_X_HOMING_MPOS 0.0
#endif
#ifndef DEFAULT_Y_HOMING_MPOS
#    define DEFAULT_Y_HOMING_MPOS 0.0
#endif
#ifndef DEFAULT_Z_HOMING_MPOS
#    define DEFAULT_Z_HOMING_MPOS 0.0
#endif
#ifndef DEFAULT_A_HOMING_MPOS
#    define DEFAULT_A_HOMING_MPOS 0.0
#endif
#ifndef DEFAULT_B_HOMING_MPOS
#    define DEFAULT_B_HOMING_MPOS 0.0
#endif
#ifndef DEFAULT_C_HOMING_MPOS
#    define DEFAULT_C_HOMING_MPOS 0.0
#endif

#ifndef DEFAULT_HOMING_CYCLE_0
#    define DEFAULT_HOMING_CYCLE_0 bit(Z_AXIS)
#endif

#ifndef DEFAULT_HOMING_CYCLE_1
#    define DEFAULT_HOMING_CYCLE_1 (bit(X_AXIS) | bit(Y_AXIS))
#endif

#ifndef DEFAULT_HOMING_CYCLE_2
#    define DEFAULT_HOMING_CYCLE_2 0
#endif

#ifndef DEFAULT_HOMING_CYCLE_3
#    define DEFAULT_HOMING_CYCLE_3 0
#endif

#ifndef DEFAULT_HOMING_CYCLE_4
#    define DEFAULT_HOMING_CYCLE_4 0
#endif

#ifndef DEFAULT_HOMING_CYCLE_5
#    define DEFAULT_HOMING_CYCLE_5 0
#endif

// ========== Motor current (SPI Drivers ) =============
#ifndef DEFAULT_X_CURRENT
#    define DEFAULT_X_CURRENT 0.8  // $140 current in amps (extended set)
#endif
#ifndef DEFAULT_Y_CURRENT
#    define DEFAULT_Y_CURRENT 0.8  // $141 current in amps (extended set)
#endif
#ifndef DEFAULT_Z_CURRENT
#    define DEFAULT_Z_CURRENT 0.8  // $142 current in amps (extended set)
#endif
#ifndef DEFAULT_A_CURRENT
#    define DEFAULT_A_CURRENT 0.8  // $143 current in amps (extended set)
#endif
#ifndef DEFAULT_B_CURRENT
#    define DEFAULT_B_CURRENT 0.8  // $144 current in amps (extended set)
#endif
#ifndef DEFAULT_C_CURRENT
#    define DEFAULT_C_CURRENT 0.8  // $145 current in amps (extended set)
#endif

// ========== Motor hold current (SPI Drivers ) =============

#ifndef DEFAULT_X_HOLD_CURRENT
#    define DEFAULT_X_HOLD_CURRENT 0.4  // $150 current in amps (extended set)
#endif
#ifndef DEFAULT_Y_HOLD_CURRENT
#    define DEFAULT_Y_HOLD_CURRENT 0.4  // $151 current in amps (extended set)
#endif
#ifndef DEFAULT_Z_HOLD_CURRENT
#    define DEFAULT_Z_HOLD_CURRENT 0.4  // $152 current in amps (extended set)
#endif
#ifndef DEFAULT_A_HOLD_CURRENT
#    define DEFAULT_A_HOLD_CURRENT 0.4  // $153 current in amps (extended set)
#endif
#ifndef DEFAULT_B_HOLD_CURRENT
#    define DEFAULT_B_HOLD_CURRENT 0.4  // $154 current in amps (extended set)
#endif
#ifndef DEFAULT_C_HOLD_CURRENT
#    define DEFAULT_C_HOLD_CURRENT 0.4  // $154 current in amps (extended set)
#endif

// ========== Microsteps (SPI Drivers ) ================

#ifndef DEFAULT_X_MICROSTEPS
#    define DEFAULT_X_MICROSTEPS 16  // $160 micro steps (extended set)
#endif
#ifndef DEFAULT_Y_MICROSTEPS
#    define DEFAULT_Y_MICROSTEPS 16  // $161 micro steps (extended set)
#endif
#ifndef DEFAULT_Z_MICROSTEPS
#    define DEFAULT_Z_MICROSTEPS 16  // $162 micro steps (extended set)
#endif
#ifndef DEFAULT_A_MICROSTEPS
#    define DEFAULT_A_MICROSTEPS 16  // $163 micro steps (extended set)
#endif
#ifndef DEFAULT_B_MICROSTEPS
#    define DEFAULT_B_MICROSTEPS 16  // $164 micro steps (extended set)
#endif
#ifndef DEFAULT_C_MICROSTEPS
#    define DEFAULT_C_MICROSTEPS 16  // $165 micro steps (extended set)
#endif

// ========== Stallguard (SPI Drivers ) ================

#ifndef DEFAULT_X_STALLGUARD
#    define DEFAULT_X_STALLGUARD 16  // $170 stallguard (extended set)
#endif
#ifndef DEFAULT_Y_STALLGUARD
#    define DEFAULT_Y_STALLGUARD 16  // $171 stallguard (extended set)
#endif
#ifndef DEFAULT_Z_STALLGUARD
#    define DEFAULT_Z_STALLGUARD 16  // $172 stallguard (extended set)
#endif
#ifndef DEFAULT_A_STALLGUARD
#    define DEFAULT_A_STALLGUARD 16  // $173 stallguard (extended set)
#endif
#ifndef DEFAULT_B_STALLGUARD
#    define DEFAULT_B_STALLGUARD 16  // $174 stallguard (extended set)
#endif
#ifndef DEFAULT_C_STALLGUARD
#    define DEFAULT_C_STALLGUARD 16  // $175 stallguard (extended set)
#endif

// ==================  pin defaults ========================

// Here is a place to default pins to UNDEFINED_PIN.
// This can eliminate checking to see if the pin is defined because
// the overridden pinMode and digitalWrite functions will deal with it.

#ifndef SDCARD_DET_PIN
#    define SDCARD_DET_PIN UNDEFINED_PIN
#endif

#ifndef STEPPERS_DISABLE_PIN
#    define STEPPERS_DISABLE_PIN UNDEFINED_PIN
#endif

#ifndef X_DISABLE_PIN
#    define X_DISABLE_PIN UNDEFINED_PIN
#endif
#ifndef Y_DISABLE_PIN
#    define Y_DISABLE_PIN UNDEFINED_PIN
#endif
#ifndef Z_DISABLE_PIN
#    define Z_DISABLE_PIN UNDEFINED_PIN
#endif
#ifndef A_DISABLE_PIN
#    define A_DISABLE_PIN UNDEFINED_PIN
#endif
#ifndef B_DISABLE_PIN
#    define B_DISABLE_PIN UNDEFINED_PIN
#endif
#ifndef C_DISABLE_PIN
#    define C_DISABLE_PIN UNDEFINED_PIN
#endif

#ifndef X2_DISABLE_PIN
#    define X2_DISABLE_PIN UNDEFINED_PIN
#endif
#ifndef Y2_DISABLE_PIN
#    define Y2_DISABLE_PIN UNDEFINED_PIN
#endif
#ifndef Z2_DISABLE_PIN
#    define Z2_DISABLE_PIN UNDEFINED_PIN
#endif
#ifndef A2_DISABLE_PIN
#    define A2_DISABLE_PIN UNDEFINED_PIN
#endif
#ifndef B2_DISABLE_PIN
#    define B2_DISABLE_PIN UNDEFINED_PIN
#endif
#ifndef C2_DISABLE_PIN
#    define C2_DISABLE_PIN UNDEFINED_PIN
#endif

#ifndef X_LIMIT_PIN
#    define X_LIMIT_PIN UNDEFINED_PIN
#endif
#ifndef Y_LIMIT_PIN
#    define Y_LIMIT_PIN UNDEFINED_PIN
#endif
#ifndef Z_LIMIT_PIN
#    define Z_LIMIT_PIN UNDEFINED_PIN
#endif
#ifndef A_LIMIT_PIN
#    define A_LIMIT_PIN UNDEFINED_PIN
#endif
#ifndef B_LIMIT_PIN
#    define B_LIMIT_PIN UNDEFINED_PIN
#endif
#ifndef C_LIMIT_PIN
#    define C_LIMIT_PIN UNDEFINED_PIN
#endif
#ifndef X2_LIMIT_PIN
#    define X2_LIMIT_PIN UNDEFINED_PIN
#endif
#ifndef Y2_LIMIT_PIN
#    define Y2_LIMIT_PIN UNDEFINED_PIN
#endif
#ifndef Z2_LIMIT_PIN
#    define Z2_LIMIT_PIN UNDEFINED_PIN
#endif
#ifndef A2_LIMIT_PIN
#    define A2_LIMIT_PIN UNDEFINED_PIN
#endif
#ifndef B2_LIMIT_PIN
#    define B2_LIMIT_PIN UNDEFINED_PIN
#endif
#ifndef C2_LIMIT_PIN
#    define C2_LIMIT_PIN UNDEFINED_PIN
#endif

// assigned all MS3 (microstep pin 3) to UNDEFINED_PIN

#ifndef X_STEPPER_MS3
#    define X_STEPPER_MS3 UNDEFINED_PIN
#endif
#ifndef Y_STEPPER_MS3
#    define Y_STEPPER_MS3 UNDEFINED_PIN
#endif
#ifndef Z_STEPPER_MS3
#    define Z_STEPPER_MS3 UNDEFINED_PIN
#endif
#ifndef A_STEPPER_MS3
#    define A_STEPPER_MS3 UNDEFINED_PIN
#endif
#ifndef B_STEPPER_MS3
#    define B_STEPPER_MS3 UNDEFINED_PIN
#endif
#ifndef C_STEPPER_MS3
#    define C_STEPPER_MS3 UNDEFINED_PIN
#endif
#ifndef X2_STEPPER_MS3
#    define X2_STEPPER_MS3 UNDEFINED_PIN
#endif
#ifndef Y2_STEPPER_MS3
#    define Y2_STEPPER_MS3 UNDEFINED_PIN
#endif
#ifndef Z2_STEPPER_MS3
#    define Z2_STEPPER_MS3 UNDEFINED_PIN
#endif
#ifndef A2_STEPPER_MS3
#    define A2_STEPPER_MS3 UNDEFINED_PIN
#endif
#ifndef B2_STEPPER_MS3
#    define B2_STEPPER_MS3 UNDEFINED_PIN
#endif
#ifndef C2_STEPPER_MS3
#    define C2_STEPPER_MS3 UNDEFINED_PIN
#endif

#ifndef SERVO_TIMER_INTERVAL
#    define SERVO_TIMER_INTERVAL 75.0  // Hz This is the update inveral in milliseconds
#endif

#ifndef DYNAMIXEL_TXD
#    define DYNAMIXEL_TXD UNDEFINED_PIN
#endif

#ifndef DYNAMIXEL_RXD
#    define DYNAMIXEL_RXD UNDEFINED_PIN
#endif

#ifndef DYNAMIXEL_RTS
#    define DYNAMIXEL_RTS UNDEFINED_PIN
#endif

// ================ User Digital I/O ==============================

#ifndef USER_DIGITAL_PIN_0
#    define USER_DIGITAL_PIN_0 UNDEFINED_PIN
#endif

#ifndef USER_DIGITAL_PIN_1
#    define USER_DIGITAL_PIN_1 UNDEFINED_PIN
#endif

#ifndef USER_DIGITAL_PIN_2
#    define USER_DIGITAL_PIN_2 UNDEFINED_PIN
#endif

#ifndef USER_DIGITAL_PIN_3
#    define USER_DIGITAL_PIN_3 UNDEFINED_PIN
#endif

// ================ User Analog I/O ==============================

#ifndef USER_ANALOG_PIN_0
#    define USER_ANALOG_PIN_0 UNDEFINED_PIN
#endif

#ifndef USER_ANALOG_PIN_1
#    define USER_ANALOG_PIN_1 UNDEFINED_PIN
#endif

#ifndef USER_ANALOG_PIN_2
#    define USER_ANALOG_PIN_2 UNDEFINED_PIN
#endif

#ifndef USER_ANALOG_PIN_3
#    define USER_ANALOG_PIN_3 UNDEFINED_PIN
#endif

#ifndef PROBE_PIN
#    define PROBE_PIN UNDEFINED_PIN
#endif

#ifndef USER_ANALOG_PIN_0_FREQ
#    define USER_ANALOG_PIN_0_FREQ 5000
#endif
#ifndef USER_ANALOG_PIN_1_FREQ
#    define USER_ANALOG_PIN_1_FREQ 5000
#endif
#ifndef USER_ANALOG_PIN_2_FREQ
#    define USER_ANALOG_PIN_2_FREQ 5000
#endif
#ifndef USER_ANALOG_PIN_3_FREQ
#    define USER_ANALOG_PIN_3_FREQ 5000
#endif

#ifndef DEFAULT_USER_MACRO0
#    define DEFAULT_USER_MACRO0 ""
#endif

#ifndef DEFAULT_USER_MACRO1
#    define DEFAULT_USER_MACRO1 ""
#endif

#ifndef DEFAULT_USER_MACRO2
#    define DEFAULT_USER_MACRO2 ""
#endif

#ifndef DEFAULT_USER_MACRO3
#    define DEFAULT_USER_MACRO3 ""
#endif
