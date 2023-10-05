/*
  Report.cpp - reporting and messaging methods
  Part of Grbl

  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC

	2018 -	Bart Dring This file was modified for use on the ESP32
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
#include <map>

#ifdef REPORT_HEAP
EspClass esp;
#endif
const int DEFAULTBUFFERSIZE = 64;

void grbl_send(const char* text) {
    if( ENABLE_SERIAL ){
        Serial.println( String( text) );
    }
}

// This is a formating version of the grbl_send(CLIENT_ALL,...) function that work like printf
void grbl_sendf(const char* format, ...) {
    char    loc_buf[64];
    char*   temp = loc_buf;
    va_list arg;
    va_list copy;
    va_start(arg, format);
    va_copy(copy, arg);
    size_t len = vsnprintf(NULL, 0, format, arg);
    va_end(copy);
    if (len >= sizeof(loc_buf)) {
        temp = new char[len + 1];
        if (temp == NULL) {
            return;
        }
    }
    len = vsnprintf(temp, len + 1, format, arg);
    grbl_send(temp);
    va_end(arg);
    if (temp != loc_buf) {
        delete[] temp;
    }
}
// Use to send [MSG:xxxx] Type messages. The level allows messages to be easily suppressed
void grbl_msg_sendf(MsgLevel level, const char* format, ...) {

    if (message_level != NULL) {  // might be null before messages are setup
        if (level > static_cast<MsgLevel>(message_level->get())) {
            return;
        }
    }

    char    loc_buf[100];
    char*   temp = loc_buf;
    va_list arg;
    va_list copy;
    va_start(arg, format);
    va_copy(copy, arg);
    size_t len = vsnprintf(NULL, 0, format, arg);
    va_end(copy);
    if (len >= sizeof(loc_buf)) {
        temp = new char[len + 1];
        if (temp == NULL) {
            return;
        }
    }
    len = vsnprintf(temp, len + 1, format, arg);
    grbl_sendf("[MSG:%s]\r\n", temp);
    va_end(arg);
    if (temp != loc_buf) {
        delete[] temp;
    }
}

//function to notify
void grbl_notify(const char* title, const char* msg) {

}

void grbl_notifyf(const char* title, const char* format, ...) {
    char    loc_buf[64];
    char*   temp = loc_buf;
    va_list arg;
    va_list copy;
    va_start(arg, format);
    va_copy(copy, arg);
    size_t len = vsnprintf(NULL, 0, format, arg);
    va_end(copy);
    if (len >= sizeof(loc_buf)) {
        temp = new char[len + 1];
        if (temp == NULL) {
            return;
        }
    }
    len = vsnprintf(temp, len + 1, format, arg);
    grbl_notify(title, temp);
    va_end(arg);
    if (temp != loc_buf) {
        delete[] temp;
    }
}

static const int coordStringLen = 20;
static const int axesStringLen  = coordStringLen * MAX_N_AXIS;

// formats axis values into a string and returns that string in rpt
// NOTE: rpt should have at least size: axesStringLen
static void report_util_axis_values(float* axis_value, char* rpt) {
    uint8_t     idx;
    char        axisVal[coordStringLen];
    float       unit_conv = 1.0;      // unit conversion multiplier..default is mm
    const char* format    = "%4.3f";  // Default - report mm to 3 decimal places
    rpt[0]                = '\0';
    if (report_inches->get()) {
        unit_conv = 1.0 / MM_PER_INCH;
        format    = "%4.4f";  // Report inches to 4 decimal places
    }
    auto n_axis = N_AXIS;
    for (idx = 0; idx < n_axis; idx++) {
        snprintf(axisVal, coordStringLen - 1, format, axis_value[idx] * unit_conv);
        strcat(rpt, axisVal);
        if (idx < (n_axis - 1)) {
            strcat(rpt, ",");
        }
    }
}

// This version returns the axis values as a String
static String report_util_axis_values(const float* axis_value) {
    String  rpt = "";
    uint8_t idx;
    char    axisVal[coordStringLen];
    float   unit_conv = 1.0;  // unit conversion multiplier..default is mm
    int     decimals  = 3;    // Default - report mm to 3 decimal places
    if (report_inches->get()) {
        unit_conv = 1.0 / MM_PER_INCH;
        decimals  = 4;  // Report inches to 4 decimal places
    }
    auto n_axis = N_AXIS;
    for (idx = 0; idx < n_axis; idx++) {
        rpt += String(axis_value[idx] * unit_conv, decimals);
        if (idx < (n_axis - 1)) {
            rpt += ",";
        }
    }
    return rpt;
}


void report_status_message(Error status_code) {

    switch (status_code) {
        case Error::Ok:  // Error::Ok
            if (filehandler.get_sd_state(false) == SDState::BusyPrinting) {
                SD_ready_next = true;  
            } else {
                grbl_send("ok");
            }
            break;
        default:
            // do we need to stop a running SD job?
            if (filehandler.get_sd_state(false) == SDState::BusyPrinting) {
                if (status_code == Error::GcodeUnsupportedCommand) {
                    grbl_sendf("error:%d in SD file at line %d\r\n", status_code, filehandler.sd_get_current_line_number());
                    // don't close file
                    SD_ready_next = true; 
                } else {
                    grbl_notifyf("SD print error", "Error:%d during SD file at line: %d", status_code, filehandler.sd_get_current_line_number());
                    grbl_sendf("error:%d in SD file at line %d\r\n", status_code, filehandler.sd_get_current_line_number());
                    filehandler.closeFile();
                }
                return;
            }
    }
}

void report_alarm_message(ExecAlarm alarm_code) {
    sys_last_alarm = alarm_code;
    grbl_sendf("ALARM:%d\r\n", static_cast<int>(alarm_code));  // OK to send to all clients
    delay_ms(500);                                                         // Force delay to ensure message clears serial write buffer.
}
std::map<Message, const char*> MessageText = {
    { Message::CriticalEvent, "Reset to continue" },
    { Message::AlarmLock, "'$H'|'$X' to unlock" },
    { Message::AlarmUnlock, "Caution: Unlocked" },
    { Message::Enabled, "Enabled" },
    { Message::Disabled, "Disabled" },
    { Message::CheckLimits, "Check limits" },
    { Message::ProgramEnd, "Program End" },
    { Message::RestoreDefaults, "Restoring defaults" },
    { Message::SpindleRestore, "Restoring spindle" },
    { Message::SleepMode, "Sleeping" },
    // { Message::SdFileQuit, "Reset during SD file at line: %d" },
};

void report_feedback_message(Message message) {  // ok to send to all clients
#if defined(ENABLE_SD_CARD)
    if (message == Message::SdFileQuit) {
        grbl_notifyf("SD print canceled", "Reset during SD file at line: %d", filehandler.sd_get_current_line_number());
        grbl_msg_sendf(MsgLevel::Info, "Reset during SD file at line: %d", filehandler.sd_get_current_line_number());

    } else
#endif  //ENABLE_SD_CARD
    {
        auto it = MessageText.find(message);
        if (it != MessageText.end()) {
            grbl_msg_sendf(MsgLevel::Info, it->second);
        }
    }
}

// Welcome message
void report_init_message(uint8_t client) {
    grbl_sendf("\r\nLet there be sparks...\r\n", GRBL_VERSION);
}

// Prints current probe parameters. Upon a probe command, these parameters are updated upon a
// successful probe or upon a failed probe with the G38.3 without errors command (if supported).
// These values are retained until Grbl is power-cycled, whereby they will be re-zeroed.
void report_probe_parameters(uint8_t client) {
    // Report in terms of machine position.
    char probe_rpt[(axesStringLen + 13 + 6 + 1)];  // the probe report we are building here
    char temp[axesStringLen];
    strcpy(probe_rpt, "[PRB:");  // initialize the string with the first characters
    // get the machine position and put them into a string and append to the probe report
    float print_position[MAX_N_AXIS];
    system_convert_array_steps_to_mpos(print_position, sys_probe_position);
    report_util_axis_values(print_position, temp);
    strcat(probe_rpt, temp);
    // add the success indicator and add closing characters
    sprintf(temp, ":%d]\r\n", sys.probe_succeeded);
    strcat(probe_rpt, temp);
    grbl_send(probe_rpt);  // send the report
}



// Prints specified startup line
void report_startup_line(uint8_t n, const char* line, uint8_t client) {
    grbl_sendf("$N%d=%s\r\n", n, line);  // OK to send to all
}

void report_execute_startup_message(const char* line, Error status_code) {
    grbl_sendf(">%s:", line);  // OK to send to all
    report_status_message(status_code);
}



// Prints the character string line Grbl has received from the user, which has been pre-parsed,
// and has been sent into protocol_execute_line() routine to be executed by Grbl.
void report_echo_line_received(char* line, uint8_t client) {
    grbl_sendf("[echo: %s]\r\n", line);
}


void report_gcode_comment(char* comment) {
    char          msg[80];
    const uint8_t offset = 4;  // ignore "MSG_" part of comment
    uint8_t       index  = offset;
    if (strstr(comment, "MSG")) {
        while (index < strlen(comment)) {
            msg[index - offset] = comment[index];
            index++;
        }
        msg[index - offset] = 0;  // null terminate
        grbl_msg_sendf(MsgLevel::Info, "GCode Comment...%s", msg);
    }
}

void report_machine_type(uint8_t client) {
    grbl_msg_sendf(MsgLevel::Info, "Using machine:%s", MACHINE_NAME);
}

/*
    Print a message in hex format
    Ex: report_hex_msg(msg, "Rx:", 6);
    Would would print something like ... [MSG Rx: 0x01 0x03 0x01 0x08 0x31 0xbf]
*/
void report_hex_msg(char* buf, const char* prefix, int len) {
    char report[200];
    char temp[20];
    sprintf(report, "%s", prefix);
    for (int i = 0; i < len; i++) {
        sprintf(temp, " 0x%02X", buf[i]);
        strcat(report, temp);
    }

    grbl_msg_sendf(MsgLevel::Info, "%s", report);
}

void report_hex_msg(uint8_t* buf, const char* prefix, int len) {
    char report[200];
    char temp[20];
    sprintf(report, "%s", prefix);
    for (int i = 0; i < len; i++) {
        sprintf(temp, " 0x%02X", buf[i]);
        strcat(report, temp);
    }

    grbl_msg_sendf(MsgLevel::Info, "%s", report);
}

char* report_state_text() {
    static char state[10];

    switch (sys.state) {
        case State::Idle:
            break;
        case State::Cycle:
            break;
        case State::Hold:
            if (!(sys.suspend.bit.jogCancel)) {
                sys.suspend.bit.holdComplete ? strcpy(state, "Hold:0") : strcpy(state, "Hold:1");
                break;
            }  // Continues to print jog state during jog cancel.
        case State::Jog:
            break;
        case State::Homing:
            break;
        case State::Alarm:
            break;
        case State::CheckMode:
            break;
        case State::Sleep:
            break;
    }
    return state;
}

char report_get_axis_letter(uint8_t axis) {
    return get_axis_name( axis );
}

char* reportAxisLimitsMsg(uint8_t axis) {
    static char msg[40];
    sprintf(msg, "Limits(%0.3f,%0.3f)", limitsMinPosition(axis), limitsMaxPosition(axis));
    return msg;
}

char* reportAxisNameMsg(uint8_t axis, uint8_t dual_axis) {
    static char name[10];
    sprintf(name, "%c%c Axis", report_get_axis_letter(axis), dual_axis ? '2' : ' ');
    return name;
}

char* reportAxisNameMsg(uint8_t axis) {
    static char name[10];
    sprintf(name, "%c  Axis", report_get_axis_letter(axis));
    return name;
}


void mpos_to_wpos(float* position) {
    float* wco    = get_wco();
    auto n_axis = N_AXIS;
    for (int idx = 0; idx < n_axis; idx++) {
        position[idx] -= wco[idx];
    }
}

float* get_wco() {
    static float wco[MAX_N_AXIS];
    auto n_axis = N_AXIS;
    for (int idx = 0; idx < n_axis; idx++) {
        // Apply work coordinate offsets and tool length offset to current position.
        wco[idx] = gc_state.coord_system[idx] + gc_state.coord_offset[idx];
        if (idx == TOOL_LENGTH_OFFSET_AXIS) {
            wco[idx] += gc_state.tool_length_offset;
        }
    }
    return wco;
}
