/*
  Serial.cpp - Header for system level commands and real-time processes
  Part of Grbl
  Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC

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

  What is going on here?

  Original Grbl only supports communication via serial port. That is why this
  file is call serial.cpp. Grbl_ESP32 supports many "clients".

  Clients are sources of commands like the serial port or a bluetooth connection.
  Multiple clients can be active at a time. If a client asks for status, only the client will
  receive the reply to the command.

  The serial port acts as the debugging port because it is always on and does not
  need to be reconnected after reboot. Messages about the configuration and other events
  are sent to the serial port automatically, without a request command. These are in the
  [MSG: xxxxxx] format. Gcode senders are should be OK with this because Grbl has always
  send some messages like this.

  Important: It is up user that the clients play well together. Ideally, if one client
  is sending the gcode, the others should just be doing status, feedhold, etc.

  Clients send gcode, grbl commands ($$, [ESP...], etc) and realtime commands (?,!.~, etc)
  Gcode and Grbl commands are a string of printable characters followed by a '\r' or '\n'
  Realtime commands are single characters with no '\r' or '\n'

  After sending a gcode or grbl command, you must wait for an OK to send another.
  This is because only a certain number of commands can be buffered at a time.
  Grbl will tell you when it is ready for another one with the OK.

  Realtime commands can be sent at any time and will acted upon very quickly.
  Realtime commands can be anywhere in the stream.

  To allow the realtime commands to be randomly mixed in the stream of data, we
  read all clients as fast as possible. The realtime commands are acted upon and the other charcters are
  placed into a client_buffer[client].

  The main protocol loop reads from client_buffer[]

  2023 - Roland Lautensack (G-EDM) This file was heavily edited and may no longer be compatible with the default grbl

*/

#include "Grbl.h"

portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;

static TaskHandle_t clientCheckTaskHandle = 0;

WebUI::InputBuffer client_buffer[CLIENT_COUNT];  // create a buffer for each client

// Returns the number of bytes available in a client buffer.
uint8_t client_get_rx_buffer_available(uint8_t client) {
    return 128 - Serial.available();
    //    return client_buffer[client].availableforwrite();
}

void client_init() {

    client_reset_read_buffer(CLIENT_ALL);
    Serial.write("\r\n");  // create some white space after ESP32 boot info
    //return;

    clientCheckTaskHandle = 0;
    // create a task to check for incoming data
    // For a 4096-word stack, uxTaskGetStackHighWaterMark reports 244 words available
    // after WebUI attaches.
    xTaskCreatePinnedToCore(clientCheckTask,    // task
                            "clientCheckTask",  // name for task
                            8192,               // size of task stack
                            NULL,               // parameters
                            1,                  // priority
                            &clientCheckTaskHandle,
                            SUPPORT_TASK_CORE  // must run the task on same core
                                               // core
    );
}

static uint8_t getClientChar(uint8_t* data) {
    int res;
    if (WebUI::inputBuffer.available()) {
        *data = WebUI::inputBuffer.read();
        return CLIENT_INPUT;
    }

    return CLIENT_ALL;
}

// this task runs and checks for data on all interfaces
// REaltime stuff is acted upon, then characters are added to the appropriate buffer
void clientCheckTask(void* pvParameters) {
    uint8_t            data = 0;
    uint8_t            client;  // who sent the data
    static UBaseType_t uxHighWaterMark = 0;
    while (true) {  // run continuously
        while ((client = getClientChar(&data)) != CLIENT_ALL) {
            // Pick off realtime command characters directly from the serial stream. These characters are
            // not passed into the main buffer, but these set system state flag bits for realtime execution.
            if (is_realtime_command(data)) {
                execute_realtime_command(static_cast<Cmd>(data));
            } else {
                //if (filehandler.get_sd_state(false) < SDState::Busy) {
                    vTaskEnterCritical(&myMutex);
                    client_buffer[client].write(data);
                    vTaskExitCritical(&myMutex);
               /* } else {
                    if (data == '\r' || data == '\n') {
                        grbl_sendf("error %d\r\n", Error::AnotherInterfaceBusy);
                        grbl_msg_sendf(MsgLevel::Info, "SD card job running");
                    }
                }*/
            }
        }  // if something available

        vTaskDelay(1 / portTICK_RATE_MS);  // Yield to other tasks

        static UBaseType_t uxHighWaterMark = 0;

    }
}

void client_reset_read_buffer(uint8_t client) {
    client_buffer[CLIENT_INPUT].begin();
}

// Fetches the first byte in the client read buffer. Called by protocol loop.
int client_read(uint8_t client) {
    vTaskEnterCritical(&myMutex);
    int data = client_buffer[client].read();
    vTaskExitCritical(&myMutex);
    return data;
}

// checks to see if a character is a realtime character
bool is_realtime_command(uint8_t data) {
    if (data >= 0x80) {
        return true;
    }
    auto cmd = static_cast<Cmd>(data);
    return cmd == Cmd::Reset || cmd == Cmd::StatusReport || cmd == Cmd::CycleStart;
}


// Act upon a realtime character
void execute_realtime_command(Cmd command) {}

void client_write(uint8_t client, const char* text) {
    if (client == CLIENT_INPUT) {
        return;
    }
    if (client == CLIENT_SERIAL || client == CLIENT_ALL) {
        Serial.write(text);
    }
}
