/*
#  ██████        ███████ ██████  ███    ███  
# ██             ██      ██   ██ ████  ████  
# ██   ███ █████ █████   ██   ██ ██ ████ ██ 
# ██    ██       ██      ██   ██ ██  ██  ██ 
#  ██████        ███████ ██████  ██      ██ 
#
# This is a beta version for testing purposes.
# Only for personal use. Commercial use or redistribution without permission is prohibited. 
# Copyright (c) Roland Lautensack        
*/ 
#include "ili9341_tft.h"
#include "Motors/Motors.h"
#include <nvs.h>
#include "Protocol.h"

/**
 * 
 * GEDM specific GCodes
 *
 * Warning: This is Frankenstein G-RBL EDM. It doesn't work with gcode senders and the GCode does need to only contain the raw XY path and tool up/down for path changes.
 *
 * In drill/sinker mode no gcode is used and no homing of xy is done to allow easy positioning and just drill/sink down.
 *
 * In wire Mode the X and Y axis retract if needed. ( work in progress. Don't use it for now. )
 *
 * In floating Z mode a rod is used as electrode that floats on the surface of the workpiece and constantly tries
 * to progress down while the gcode file is looping. So any probing or homing commands in the file would repeat after every round if not removed.
 *
 * M7-9 toogle the different modes on and off. If the machine was set to wire edm and is then switched
 * to a different mode it is not needed to turn the previous mode off. A new mode will disable the others.
 * 
 * M7 = toggle EDM drill/sinker mode on/off
 * M8 = toggle EDM floating z axis on/off
 * M9 = toggle Wire EDM with XY retractions on/off
 * 
 * M3-4 commands are ignored in wire mode. In wire mode every command in the gcode file is interpreted as a work motion
 * A G0 command produces the same result as a G1 command. Z motion is fully disabled even if there is any in the gcode!
 * 
 * M3 = Move z up for toolpath change, it disables the PWM to prevent unwanted eroding while moving up
 * M4 = move z back down after a toolpath change, re-enables PWM and moves down until contact is made.
 *
 * While a motion is done it blocks the gcode parser from parsing new commands.
 * 
 **/
TaskHandle_t drill_task_handle = NULL;

G_EDM_UI_CONTROLLER ui_controller;




/** runs once after the ui is ready **/
void run_once_if_ready(G_EDM_UI_CONTROLLER *ui_controller)
{
  /** enable machine on startup **/
  api::unlock_machine();
  api::reset_probe_points();
  api::push_cmd("G91 G10 L20 P1 X0Y0Z0\r\n");
  api::push_cmd("G91 G54 G21\r\n");
  ui_controller->reset_defaults();
  sys_axis_homed[X_AXIS] = false;
  sys_axis_homed[Y_AXIS] = false;
  sys_axis_homed[Z_AXIS] = false;
  ui_controller->setup_spindle(GEDM_A_DIRECTION_PIN, GEDM_A_STEP_PIN);
  planner.position_history_reset();
  planner.update_max_feed();
}

/**
 * Starting the UI on core 0
 **/
void IRAM_ATTR ui_task(void *parameter)
{

  G_EDM_UI_CONTROLLER *ui_controller = (G_EDM_UI_CONTROLLER *)parameter;

  ui_controller->init(DISABLE_TFT_CALIBRATION);
  ui_controller->filehandler->set_spi_instance( ui_controller->tft.getSPIinstance() );
  ui_controller->filehandler->filehandler_initialize(true);

  /** wait until ui is ready **/
  while (!ui_controller->get_is_ready())
  {
    vTaskDelay(100);
  }

  /** restore the last session **/
  ui_controller->filehandler->create_directory_tree(3);
  ui_controller->load_settings("last_settings");

  /** draw frontpage **/
  ui_controller->render_page(1, true);
  ui_controller->last_settings_write = millis();

  bool has_test = false;

  /** the main UI loop task **/
  for (;;)
  {

    _millis = millis();

    if (ui_controller->is_probing())
    {
      vTaskDelay(1000);
      continue;
    }

    if (sys.state == State::Alarm) {
      ui_controller->alarm_handler();
    }

    if (!edm_process_is_running)
    {

      if (force_redraw)
      {
        if( ui_controller->get_active_page() != 1 ){
          ui_controller->render_page( ui_controller->get_active_page(), true );
          vTaskDelay(1);
          force_redraw = false;
        } else {
          ui_controller->draw_interface();
          vTaskDelay(1);
          ui_controller->render_page_front();
          vTaskDelay(1);
          force_redraw = false;
        }
      }

      if (ui_controller->ready_to_run())
      {

        /**
         * it gets here after an SD job is finished too and needs a
         * check if this is just the end of an SD job
         **/
        if (ui_controller->filehandler->has_job_finished())
        {
          /** ensure that the machine is finished **/
          if (api::machine_is_idle())
          {
            ui_controller->reset_after_job();
          }
          continue; // mashine still working
        }

        /** everything ready to run **/
        ui_controller->start_process();
      }

      ui_controller->monitor_touch_input();
      vTaskDelay(1);
      ui_controller->update_feedback_data();
      vTaskDelay(1);

      if (ui_controller->get_active_page() == 1)
      {
        ui_controller->draw_mpos();
        vTaskDelay(1);
        ui_controller->redraw_vsense();
        vTaskDelay(1);
        ui_controller->draw_spark_on_off_indicator_icon();
        vTaskDelay(1);
        ui_controller->draw_motion_navigation_buttons();
        vTaskDelay(1);
        ui_controller->sd_card_handler();
        vTaskDelay(1);
      }
      else
      {
        ui_controller->last_settings_write = millis();
      }

      if (!has_test)
      {
        has_test = true;
        run_once_if_ready(ui_controller);
      }
    }
    else
    {

      /** EDM process is running **/
      if (!ui_controller->start_edm_process() || sys_rt_exec_alarm != ExecAlarm::None)
      {
        Serial.println("EDM stop forced");
        /** process stopped **/
        ui_controller->reset_after_job();
        vTaskDelay(100);
        continue;
      }

      if (sys.edm_process_finished)
      {
        Serial.println("EDM stop normal");
        ui_controller->reset_after_job();
        vTaskDelay(100);
        continue;
      }

      ui_controller->update_vsense();

    
      /** only redraw the vsense based values **/
      if (sys.gedm_reprobe_block)
      {
        ui_controller->process_overlay_reprobe_confirm();
      }
      else
      {

        if (force_redraw)
        {
          ui_controller->render_page(9, true);
          force_redraw = false;
        }
        else
        {
          ui_controller->update_display_minimal();
        }
      }
    }

    vTaskDelay((edm_process_is_running ? (planner.get_is_paused() ? INTERFACE_INTERVAL : INTERFACE_INTERVAL_WHILE_EDM) : INTERFACE_INTERVAL));
  }
  vTaskDelete(NULL);
}


void G_EDM_UI_CONTROLLER::sd_card_handler(){
    if (!filehandler->is_locked_for_ui())
    {
      filehandler->sd_card_refresh();
      render_page_settings_menu_sd_card(filehandler->get_has_sd_card(), false);

      if (millis() - last_settings_write > 4000.0)
      {
        // update last settings every x seconds if on the frontpage
        // if nothing has changed it will skip
        save_settings("last_settings");
        last_settings_write = millis();
      }
    }
}

void G_EDM_UI_CONTROLLER::alarm_handler(){
  if (enable_spindle)
  {
    stop_spindle();
  }
  vTaskDelay(200);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_RED);
  tft.drawString("Alarm!", 10, 50, 2);
  tft.setTextSize(1);
   tft.setTextColor(TFT_LIGHTGREY);
  String error = "";
  int error_code = static_cast<int>( sys_last_alarm );

  String alarm_msg = "Something failed";

  switch (error_code){
      case 1:
          alarm_msg = "Hard limit was triggered";
      break;
      case 2:
          alarm_msg = "Target out of reach";
      break; 
      case 3:
          alarm_msg = "Reset while in motion";
      break;
      case 4:          
      case 5:
          alarm_msg = "Probe fail";
      break;  
      case 6:
      case 7: 
      case 8:
      case 9:
          alarm_msg = "Homing fail";
      break;  
  }

  tft.drawString("Alarm Code: " + String( error_code ), 10, 100, 2);
  tft.drawString("Alarm MSG: "  + alarm_msg, 10, 120, 2);
  tft.drawString("Touch to reset", 10, 140, 2);
  sys_rt_exec_alarm = ExecAlarm::None;
  //sys.state         = sys.gedm_planner_line_running?State::Cycle:State::Idle;

  uint16_t x, y;
  while (!get_touch(&x, &y))
  {
    vTaskDelay(100);
  }
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_LIGHTGREY);
  //tft.drawString("Resetting.....", 10, 50, 2);
  sys.abort = true;
  //sys.state = State::Idle;
  int cux = 0;
  while (sys.abort)
  {
    tft.fillCircle( 320/2, 240/2, cux, TFT_GREENYELLOW);
    if( ++cux > 120 ){
      tft.fillScreen(TFT_BLACK);
      cux= 1;
    }
    vTaskDelay(10);
  }

  reset_after_job();
  protocol_execute_realtime();
  vTaskDelay(1);
}

void G_EDM_UI_CONTROLLER::wire_gcode_task(){
  /** basic settings for the process **/
  motor_manager.set_ignore_disable(true);
  planner.set_cutting_depth_limit( 0.0 ); // normally not needed but reset the z limit anyway
  api::push_cmd("G54\r");
  planner.configure();
  api::push_cmd("M9\r"); 
  vTaskDelay(1000);
  // set wire speed
  set_speed( wire_spindle_speed );
  if ( enable_spindle && ! simulate_gcode )
  {
    // spindle should alway run in wire edm to pull the wire
    // but for testing it may be useful to allow the spindle to be off
    start_spindle();
  }
  pwm_on();
  render_page(9, true);
  SD_ready_next = true; // start
}

/**
 * Drill/Sinker mode
 **/
void G_EDM_UI_CONTROLLER::sd_card_task()
{
  /** basic settings for the process **/
  motor_manager.set_ignore_disable(true);
  /** set the cutting depth in steps after probing **/
  float cutting_depth = get_z_stop();
  planner.set_cutting_depth_limit(cutting_depth > 0.0 ? cutting_depth : 0.0);
  /** probe the workpiece **/
  api::push_cmd("G54\r");
  planner.configure();
  api::push_cmd("M8\r"); // send M8 to toggle gcode mode this needs to be done after the probing stuff and before the first gcode line is loaded from file
  vTaskDelay(1000);
  if (enable_spindle)
  {
    start_spindle();
  }
  /** pwm is disabled after the probe. Reenable it **/
  pwm_on();
  reset_flush_retract_timer();
  render_page(9, true);
  SD_ready_next = true; // start
}

/**
 * Drill/Sinker mode
 * This mode does not use any XY motion and does not home X or Y
 * It is used to drill holes and simple z axis based sinker operations
 **/
//void IRAM_ATTR sinker_drill_task_single_axis(void *parameter)
void G_EDM_UI_CONTROLLER::sinker_drill_single_axis()
{
  /** basic settings for the process **/
  motor_manager.set_ignore_disable(true);
  /** in sinker/drill mode only z axis is homed if not disabled **/
  if (!z_no_home)
  {
    api::home_z();
  } else{
    api::force_home_z();
  }
  if (enable_spindle)
  {
    start_spindle();
  }
  /** backup frequency and duty and change the values for the probing cycle **/
  pwm_on();
  probe_mode_on();
  generate_reference_voltage();
  probe_z(-1.0);
  probe_mode_off();
  /** set the cutting depth in steps after probing **/
  float cutting_depth = get_z_stop();
  planner.set_cutting_depth_limit(cutting_depth > 0.0 ? cutting_depth : 0.0);
  planner.configure();
  api::push_cmd("G10 P1 L20 X0Y0\r");
  api::push_cmd("M7\r"); // send M7 to toggle drill/sinker mode this needs to be done after the probing stuff!
  vTaskDelay(500);
  /** pwm is disabled after the probe. Reenable it **/
  pwm_on();
  reset_flush_retract_timer();
  //api::push_cmd("G1 Z-5 F30\r");
  float* current_position = system_get_mpos();
  float max_travel = ( DEFAULT_Z_MAX_TRAVEL - DEFAULT_HOMING_PULLOFF + (current_position[Z_AXIS]>=0?0:current_position[Z_AXIS]) ) * -1;
  float target_depth = current_position[Z_AXIS] -= cutting_depth - 2.0;
  if( cutting_depth <= 0.0 ){
    // no limits, drilling until no more workload is recognized
    target_depth = max_travel;
  }
  if( target_depth <= max_travel ){
    // target is out of range, using the max possible travel instead
    target_depth = max_travel;
  }
  char command_buf_down[40];
  sprintf(command_buf_down, "G90 G1 Z%.5f F30\r", target_depth);
  api::push_cmd(command_buf_down);
}

/**
 * Reamer mode
 * This mode does not use any XY motion and does not home X or Y
 * In reamer mode the Z axis just constantly moves up and down with spindle on or off
 * It can use a timer to stop the operation after a given amount of time and the distance of the up/down movement can be set too.
 * The movement starts with a down movement from the current Z axis position. No homing is used.
 * This mode is used to even out and enlarge deepholes
 * Reamer mode doesn't care about short circuits!
 **/
void IRAM_ATTR reamer_task_single_axis(void *parameter)
{

  G_EDM_UI_CONTROLLER *ui_controller = (G_EDM_UI_CONTROLLER *)parameter;

  if (enable_spindle)
  {
    ui_controller->start_spindle();
  }

  vTaskDelay(500);

  ui_controller->pwm_on();

  unsigned long start_time = millis();
  bool is_moving_down = false;
  char command_buf_down[40];
  char command_buf_up[40];

  sprintf(command_buf_down, "$J=G91 G21 Z%.5f F60\r\n", reamer_travel_mm * -1);
  sprintf(command_buf_up, "$J=G91 G21 Z%.5f F60\r\n", reamer_travel_mm);

  for (;;)
  {

    vTaskDelay(10);

    // ui_controller->update_display_minimal();

    if (
        sys.abort || !ui_controller->start_edm_process() || (reamer_duration > 0.0 && float((millis() - start_time) / 1000.0) >= reamer_duration))
    {
      break;
    }

    if (sys.gedm_planner_line_running)
    {

      /** axis is moving. Skipping this round **/
      continue;
    }
    else
    {

      if (is_moving_down)
      {

        api::push_cmd(command_buf_up);
        is_moving_down = false;
        vTaskDelay(500);
      }
      else
      {

        api::push_cmd(command_buf_down);
        is_moving_down = true;
        vTaskDelay(500);
      }
    }

    //_millis = millis();
    // ui_controller->update_vsense();
  }

  ui_controller->pwm_off();
  sys.edm_process_finished = true;

  /** todo: move back up after job is done **/
  // ui_controller->reset_after_job();
  vTaskDelete(NULL);
}

void G_EDM_UI_CONTROLLER::pre_process_defaults()
{
  planner.push_break_to_position_history();
  api::unlock_machine();
  edm_process_is_running = true;
  render_page(9, true);
  generate_reference_voltage();
}

void G_EDM_UI_CONTROLLER::start_process(void)
{

  if (operation_mode == 1)
  {
    pre_process_defaults();
    sinker_drill_single_axis();
    //xTaskCreatePinnedToCore(sinker_drill_task_single_axis, "drill_task", 2500, this, 5, &drill_task_handle, 0);
  }
  else if (operation_mode == 2)
  {
    pre_process_defaults();
    xTaskCreatePinnedToCore(reamer_task_single_axis, "reamer_task", 2500, this, 5, &drill_task_handle, 0);
  }
  else if (operation_mode == 3)
  {

    /** gcode from file **/
    // gcode mode requires the machine to be fully homed
    // if no probe position is set we just set the current position as G54 0,0 position
    if (!api::machine_is_fully_homed())
    {
      return;
    }

    if (!api::machine_is_fully_probed())
    {
      // if the machine is not probed
      // we just set the current position of the unprobed axes
      // to the probe position#
      for (int i = 0; i < N_AXIS; ++i)
      {
        if (!sys_probed_axis[i])
        {

          if (i == X_AXIS)
          {
            api::push_cmd("G10 P1 L20 X0\r");
          }
          else if (i == Y_AXIS)
          {
            api::push_cmd("G10 P1 L20 Y0\r");
          }
          else if (i == Z_AXIS)
          {
            api::push_cmd("G10 P1 L20 Z0\r");
          }
          sys_probed_axis[i] = true;
          sys_probe_position_final[i] = sys_position[i];
        }
      }
      vTaskDelay(1000);
    }

    /** if a gcode file is set time to run it **/
    if (gcode_file.length() <= 0)
    {
      Serial.println("No gcode file");
      /** no file specified **/
    }
    else
    {

      open_gcode_file();
      sd_card_task();

      //xTaskCreatePinnedToCore(sd_card_task, "sd_task", 2500, this, 5, &drill_task_handle, 0);
    }
  } else if( operation_mode == 4 ){

    if (!api::machine_is_fully_homed())
    {
      Serial.println("Machine not fully homed");
      return;
    }

    if( ! sys_probed_axis[X_AXIS] || ! sys_probed_axis[Y_AXIS] ){
      // if the machine is not probed
      // we just set the current position of the unprobed axes
      // to the probe position#
      for (int i = 0; i < N_AXIS; ++i)
      {
        if (!sys_probed_axis[i])
        {

          if (i == X_AXIS)
          {
            api::push_cmd("G10 P1 L20 X0\r");
          }
          else if (i == Y_AXIS)
          {
            api::push_cmd("G10 P1 L20 Y0\r");
          }
          else if (i == Z_AXIS)
          {
            api::push_cmd("G10 P1 L20 Z0\r");
          }
          sys_probed_axis[i]          = true;
          sys_probe_position_final[i] = sys_position[i];
        }
      }
      vTaskDelay(1000);
    }
    if (gcode_file.length() <= 0)
    {
      Serial.println("No gcode file");
      /** no file specified **/
    } else{
      open_gcode_file();
      wire_gcode_task();
    }

  }
}


bool G_EDM_UI_CONTROLLER::open_gcode_file()
{
    String path = String(ROOT_FOLDER) + String(GCODE_FOLDER) + "/" + gcode_file;
    char file[255];
    path.toCharArray(file, 200);
    while (filehandler->get_is_busy())
    {
      Serial.println("Filehandler is busy");
      if (sys.abort || !start_edm_process())
      {
        break;
      }
      vTaskDelay(100);
    }
    pre_process_defaults();
    filehandler->closeFile();
    filehandler->openFile(file);
    return true;
}

/**
 * this is only used for floating Z gcode tasks
 * the reprobe point is the point where z axis reprobes the work piece
 * in the running process
 **/
void G_EDM_UI_CONTROLLER::set_reprobe_point()
{

  api::set_reprobe_points();
}

void G_EDM_UI_CONTROLLER::render_page_reprobe_confirmation()
{
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(TFT_LIGHTGREY);
  tft.setTextColor(TFT_GREEN);
  tft.drawString("Step 1", 10, 10, 2);
  tft.drawString("Step 2 ( optional )", 10, 100, 2);
  tft.setTextColor(TFT_LIGHTGREY);
  tft.drawString("Loosen the drill chuck and let the", 10, 40, 2);
  tft.drawString("electrode slide on top of the work piece", 10, 60, 2);
  tft.drawString("Use arrows to move Z up or down", 10, 130, 2);
  tft.drawString("to regain some lost travel if needed", 10, 150, 2);
  tft.fillTriangle(290, 110, 280, 130, 300, 130, TFT_GREEN);
  tft.fillTriangle(280, 140, 290, 160, 300, 140, TFT_GREEN);
  tft.setTextColor(TFT_BLACK);
  tft.fillRoundRect(10, 190, 300, 40, 5, TFT_OLIVE);
  tft.drawString("Confirm new Z0 G54 work position", 20, 200, 2);
}

void G_EDM_UI_CONTROLLER::process_overlay_reprobe_confirm()
{
  render_page_reprobe_confirmation();

  bool done = false;
  uint16_t x, y;
  bool block_touch = false;

  while (!done)
  {

    if (sys.abort || !sys.gedm_reprobe_block || !start_edm_process())
    {
      break;
    }

    if (get_touch(&x, &y))
    {

      if (block_touch)
      {
        /** debounce **/
        continue;
      }

      block_touch = true;

      if (x > 280 && x < 300 && y > 110 && y < 130)
      {

        open_keyboard(0.0, "Move Z up (mm)", "Move Z axis up\r" + get_mpos_string());
        planner.z_axis_move_mm(get_keyboard_result());
        render_page_reprobe_confirmation();
      }
      else if (x > 280 && x < 300 && y > 140 && y < 160)
      {

        open_keyboard(0.0, "Move Z down (mm)", "Move Z axis down\r" + get_mpos_string());
        planner.z_axis_move_mm(get_keyboard_result() * -1);
        render_page_reprobe_confirmation();
      }
      else if (x > 10 && x < 310 && y > 190 && y < 230)
      {

        render_page(9, true);
        done = true;
        sys.gedm_reprobe_block = false;
      }
    }
    else
    {
      block_touch = false;
    }

    vTaskDelay(1);
  }
}

void G_EDM_UI_CONTROLLER::update_display_minimal()
{
  // core 0 is needed to update the feedback data
  // display access is heavy and is only used within
  // the stepper motor delay
  // probing needs high speed feedbacks and display stuff off
  if ((!sys.edm_pause_motion && !is_between_steps) || is_probing())
  {
    return;
  }
  update_vsense();

  /** check for touch **/
  uint16_t x, y;
  bool block_touch = false;
  bool redraw = false;
  bool is_paused = planner.get_is_paused();
  bool recover = true;

  if (get_touch(&x, &y))
  {
    if (block_touch || sys.gedm_reprobe_motion)
    {
      /** debounce **/
      return;
    }
    if (!is_paused)
    {
      /** pause process on touch events **/
      sys.edm_pause_motion = true;
    }
    block_touch = true;
    if (x > 250 && x < 310 && y > 170 && y < 230)
    {
      // pause process
      sys.edm_pause_motion = is_paused ? false : true;
      recover = false;
      process_overlay_pause_button();
    }
    else if (x > 0 && x < 220 && y > 100 && y < 125)
    {
      /** change PWM frequency **/
      edit_pwm_frequency();
      redraw = true;
    }
    else if (x > 0 && x < 220 && y > 125 && y < 150)
    {
      /** change PWM duty **/
      edit_pwm_duty();
      redraw = true;
    }
    else if (x > 0 && x < 220 && y > 150 && y < 175)
    {
      /** change setpoint min **/
      edit_setpoint_min();
      redraw = true;
    }
    else if (x > 0 && x < 220 && y > 175 && y < 200)
    {
      /** change setpoint max **/
      edit_setpoint_max();
      redraw = true;
    }

    else if (x > 0 && x < 220 && y > 200 && y < 225)
    {
        // changing feedrate
        edit_max_feed( Z_AXIS );
        redraw = true;
    }
    else if (operation_mode == 3)
    {


      // floating z gcode process
      if (x > 250 && x < 310 && y > 100 && y < 160)
      {
        // force reprobe with the next line
        if (sys.gedm_insert_probe)
        {
          sys.gedm_insert_probe = false; // undo
        }
        else
        {
          sys.gedm_insert_probe = true;
        }
        redraw = true;
      }

    } else if( operation_mode == 4 )
    {
      // wire mode
      if (x > 250 && x < 310 && y > 100 && y < 160)
      {
        change_wire_spindle_speed();
        set_speed(wire_spindle_speed); // update speed
        redraw = true;
      }


    }

    /** block until untouched **/
    while (get_touch(&x, &y))
    {
      update_vsense();
      vTaskDelay(1);
    }
    if (!is_paused && recover)
    {
      /** restore **/
      sys.edm_pause_motion = false;
    }
    if (redraw)
    {
      render_page(9, true);
    }
  }
  else
  {

    process_overlay_coords();
    update_vsense();
    update_process_meta();
  }

  update_vsense();
}

void G_EDM_UI_CONTROLLER::edit_rapid_delay(){
    open_keyboard( (float)DEFAULT_STEP_DELAY_RAPID, "Step delay used for rapid moves (ms)", "Rapid motion speed\rHigher values produce slower motions");
    int delay = MAX(10,round( get_keyboard_result() ));
    DEFAULT_STEP_DELAY_RAPID = delay;
}
void G_EDM_UI_CONTROLLER::edit_max_feed_xy(){
    open_keyboard( (float)DEFAULT_STEP_DELAY_EDM, "Step delay in EDM process (ms)", "The step delay used in the EDM process\rHigher values produce slower motions");
    int delay = MAX(DEFAULT_STEP_DELAY_RAPID,round( get_keyboard_result() ));
    DEFAULT_STEP_DELAY_EDM = delay;
}

void G_EDM_UI_CONTROLLER::edit_max_feed( int axis ){    
    open_keyboard( max_feeds[axis], "Max "+String( get_axis_name( axis ) )+" axis Feedrate (mm/min)", "The maximum feedrate used for "+String( get_axis_name( axis ) )+" in the\rEDM process");
    float mm_min = get_keyboard_result();
    max_feeds[axis] = mm_min;
    planner.update_max_feed();
}

void G_EDM_UI_CONTROLLER::edit_pwm_frequency()
{
  open_keyboard(get_freq(), "PWM Frequency (hz)", "PWM Frequency in hz");
  change_pwm_frequency((int)get_keyboard_result());
};
void G_EDM_UI_CONTROLLER::edit_pwm_duty()
{
  open_keyboard(get_duty_percent(), "PWM duty (%)", "PWM Duty cycle in percent");
  float new_duty = get_keyboard_result();
  if (new_duty > PWM_DUTY_MAX)
  {
    new_duty = PWM_DUTY_MAX;
  }
  update_duty(new_duty);
};
void G_EDM_UI_CONTROLLER::edit_setpoint_min()
{
  open_keyboard(vsense_drop_range_min, "Setpoint minimum (%)", "If voltage drop is below this value the \rtool moves further");
  float new_setpoint_min = get_keyboard_result();
  /** setpoint range min is toolow **/
  if (new_setpoint_min <= 0)
  {
    new_setpoint_min = 1;
  }
  /** if setpoint range min is higher then setpoint range max we use the setpoint range max minus 1 **/
  if (new_setpoint_min >= vsense_drop_range_max)
  {
    new_setpoint_min = vsense_drop_range_max - 1.0;
  }
  vsense_drop_range_min = new_setpoint_min;
};
void G_EDM_UI_CONTROLLER::edit_setpoint_max()
{
  open_keyboard(vsense_drop_range_max, "Setpoint maximum (%)", "If voltage drops more then this percentage the \rtool retracts");
  float new_setpoint_max = get_keyboard_result();
  /** if setpoint range max is lower then setpoint range min we use the setpoint range min plus 1 **/
  if (new_setpoint_max <= vsense_drop_range_min)
  {
    new_setpoint_max = vsense_drop_range_min + 1.0;
  }
  if (new_setpoint_max > 100.0)
  {
    new_setpoint_max = 100.0;
  }
  vsense_drop_range_max = new_setpoint_max;
};

void G_EDM_UI_CONTROLLER::reset_flush_retract_timer()
{
  flush_retract_timer = millis();
}

bool G_EDM_UI_CONTROLLER::check_if_time_to_flush()
{

  if (flush_retract_after > 0.0 && (millis() - flush_retract_timer >= flush_retract_after))
  {

    reset_flush_retract_timer();
    return true;
  }

  return false;
}

void G_EDM_UI_CONTROLLER::reset_defaults()
{
  probe_dimension                 = 0;
  sys_probe_state                 = Probe::Off;
  sys_rt_exec_alarm               = ExecAlarm::None;
  sys.gedm_retraction_motion      = false;
  sys.gedm_reprobe_block          = false;
  sys.gedm_reprobe_motion         = false;
  sys.gedm_single_axis_drill_task = false;
  sys.gedm_floating_z_gcode_task  = false;
  sys.gedm_stepper_disable_z      = false;
  sys.gedm_wire_gcode_task        = false;
  sys.edm_process_finished        = false;
  sys.edm_pause_motion            = false;
  sys.edm_pause_motion_probe      = false;
  sys.gedm_insert_probe           = false;
  sys.probe_succeeded             = true;
  sys.gedm_reset_protocol         = false;
  sys.gedm_stop_process           = false;
  sys.gedm_probe_position_x       = 0.0;
  sys.gedm_probe_position_y       = 0.0;
  enable_correction               = false;
  motion_plan                     = 0;
  has_last_position               = false;
  probe_touched                   = false;
  limit_touched                   = false;
  edm_process_is_running          = false;
  sys_rt_exec_state.bit.motionCancel = true;
  sys_rt_exec_state.bit.reset        = false;
  api::block_while_not_idle();
  sys.abort                         = false;
  force_redraw                      = true;
  planner.reset_planner_state();
  last_settings_write = millis();
  sys.state = State::Idle;
  api::push_cmd("G91 G54 G21\r\n");
}

void G_EDM_UI_CONTROLLER::reset_after_job()
{
  //sys.abort = true;
  sys.gedm_stop_process              = true;
  sys_rt_exec_state.bit.motionCancel = true;
  filehandler->closeFile();
  disable_spark_generator();
  stop_spindle();
  reset_spindle();
  motor_manager.set_ignore_disable(false);
  motor_manager.restore();
  // protocol_ready = false;
  //api::cancel_running_job();
  vTaskDelay(100);
  planner.position_history_reset();

  // set_gcode_file("");
  filehandler->reset_job_finished();
  sys.state         = State::Idle;
  sys_rt_exec_alarm = ExecAlarm::None;
  sys_rt_exec_state.bit.reset = false;
  api::block_while_not_idle();
  render_page(1, true);
  force_redraw = true;
  api::unlock_machine();
  disable_spark_generator();
  api::block_while_not_idle();
  reset_defaults();
  render_page(1,true);
}

G_EDM_UI_CONTROLLER::G_EDM_UI_CONTROLLER() {}

bool G_EDM_UI_CONTROLLER::ready_to_run()
{

  if (
      sys.abort
      /** if on/off switch is off **/
      || !start_edm_process()
      /** if pwm is off **/
      || !pwm_is_enabled()
      /** if edm is already running **/
      || edm_process_is_running
      /** if source voltage is below min voltage **/
      || (get_source_voltage() < source_voltage_min)

  )
  {
    return false;
  }
  vTaskDelay(50);
  return true;
}

void G_EDM_UI_CONTROLLER::init()
{
  set_motion_tab_active(1);
  spark_on_off_indicator_icon_last_state = -1;
  spark_indicator_bar_width = 0;
  is_ready = false;
  keyboard_is_active = false;
  button_width = 45;
  button_height = 45;
  button_margin = 5;
  keyboard_value = "";
  keyboard_initial_value = 0.0;
  active_page = 0;
  has_sd_card = false;
  gcode_file = "";
  TFT_eSPI tft = TFT_eSPI();
}

bool G_EDM_UI_CONTROLLER::gcode_job_running()
{

  return filehandler->is_locked_for_ui();
}

bool G_EDM_UI_CONTROLLER::get_is_ready()
{
  if (
      !is_ready
      //|| !planner.is_ready()
      || !protocol_ready || !filehandler->get_is_ready() || (millis() - start_time) < LOADER_DELAY)
  {
    return false;
  }
  return true;
}
void G_EDM_UI_CONTROLLER::set_is_ready()
{
  vTaskDelay(500);
  is_ready = true;
}

void G_EDM_UI_CONTROLLER::start_ui_task()
{
  xTaskCreatePinnedToCore(ui_task, "ui_task", 10240, this, 1, &ui_task_handle, !xPortGetCoreID());
}

/**
 *
 * Render the pages based on their pagenumber
 *
 **/
void G_EDM_UI_CONTROLLER::render_page(int value, bool redraw_full)
{
  //value = 9; operation_mode=4; // force to show the process overlay page
  int active_page = get_active_page();
  if (active_page != value)
  {
    redraw_full = true;
  }
  set_active_page(value);
  // ui_controller->set_drop( sensors.get_drop_in_percent( false ) );
  set_drop_min_max(vsense_drop_range_min, vsense_drop_range_max);
  set_voltage_min(source_voltage_min);
  set_drop_short(VSENSE_DROP_RANGE_SHORT);
  // set_z_no_home( z_no_home );
  // set_reamer_travel_mm( reamer_travel_mm );
  // set_reamer_duration( reamer_duration );
  // set_operation_mode( operation_mode );
  // set_flush_retract_after( flush_retract_after );
  // set_flush_retract_mm( flush_retract_mm );
  // set_disable_spark_for_flushing( disable_spark_for_flushing );
  // set_flush_offset_steps( flush_offset_steps );
  set_active_profile(active_profile);
  draw_page(value, redraw_full);
  vTaskDelay(200);
}

void G_EDM_UI_CONTROLLER::set_filehandler(G_FILEHANDLER *ptrfilehandler)
{

  filehandler = ptrfilehandler;
}




void G_EDM_UI_CONTROLLER::init(bool disable_calibration)
{
  disable_tft_calibration = disable_calibration;
  start_time = millis();
  last_settings_copy = "";
  disable_spark_generator();
  tft.init();
  vTaskDelay(1);
  tft.setRotation(3);
  vTaskDelay(1);
  tft.fillScreen(TFT_BLACK);
  vTaskDelay(1);
  tft.setCursor(0, 0, 4);
  vTaskDelay(1);
  touch_calibrate();
  vTaskDelay(1);
  loader_screen();
  vTaskDelay(1);
  tft.setCursor(0, 0, 4);
}

void G_EDM_UI_CONTROLLER::loader_screen()
{
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(3);
  tft.setTextColor(TFT_OLIVE);
  tft.drawString("GEDM", 20, 20, 2);
  tft.setTextSize(2);
  tft.setTextColor(TFT_PURPLE);
  tft.drawString("It's goblin goood...", 20, 70, 2);
  tft.setTextColor(TFT_DARKCYAN);
  tft.setTextSize(1);
  tft.drawString("Powered by TFT_eSPI and G-RBL", 20, 120, 2);
  tft.drawString("Credits:", 20, 160, 2);
  tft.drawString("8kate, Teelicht", 20, 180, 2);
  tft.drawString("Ethan Hall, Alex Treseder", 20, 200, 2);  
  

}

void G_EDM_UI_CONTROLLER::touch_calibrate()
{
  if (disable_tft_calibration)
  {
    return;
  }
  uint16_t calData[5];
  uint8_t calDataOK = 0;
  if (!SPIFFS.begin())
  {
    SPIFFS.format();
    SPIFFS.begin();
  }
  if (SPIFFS.exists(TOUCH_CALIBRATION_FILE))
  {
    if (REPEAT_CAL)
    {
      SPIFFS.remove(TOUCH_CALIBRATION_FILE);
    }
    else
    {
      File f = SPIFFS.open(TOUCH_CALIBRATION_FILE, "r");
      if (f)
      {
        if (f.readBytes((char *)calData, 14) == 14)
          calDataOK = 1;
        f.close();
      }
    }
  }
  if (calDataOK && !REPEAT_CAL)
  {
    tft.setTouch(calData);
  }
  else
  {
    // data not valid so recalibrate
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(20, 0);
    tft.setTextFont(2);
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.println("Touch corners as indicated");
    tft.setTextFont(1);
    tft.println();
    if (REPEAT_CAL)
    {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.println("Set REPEAT_CAL to false to stop this running again!");
    }
    tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.println("Calibration complete!");
    File f = SPIFFS.open(TOUCH_CALIBRATION_FILE, "w");
    if (f)
    {
      f.write((const unsigned char *)calData, 14);
      f.close();
    }
  }
}

void G_EDM_UI_CONTROLLER::change_wire_spindle_speed(){
  open_keyboard( frequency_to_rpm( wire_spindle_speed ), "Wire spindle speed (rpm)", "Speed of the spindle stepper in wire mode\rRotations per minute");
  wire_spindle_speed = rpm_to_frequency( ( get_keyboard_result() ) );
  if( wire_spindle_speed > DEFAULT_A_MAX_FREQUENCY ){
    wire_spindle_speed = DEFAULT_A_MAX_FREQUENCY;
  }
}

void G_EDM_UI_CONTROLLER::stop_spi(void)
{
  tft.getSPIinstance().end();
}
void G_EDM_UI_CONTROLLER::start_spi(void)
{
  tft.getSPIinstance().begin();
}

String G_EDM_UI_CONTROLLER::get_gcode_file(void)
{
  return gcode_file;
}
bool G_EDM_UI_CONTROLLER::get_touch(uint16_t *x, uint16_t *y)
{
  return tft.getTouch(x, y, 500);
}
float G_EDM_UI_CONTROLLER::get_z_stop()
{
  return z_stop_after_mm;
}
int G_EDM_UI_CONTROLLER::get_ms_edm()
{
  return ms_edm;
}
int G_EDM_UI_CONTROLLER::get_ms_travel()
{
  return ms_travel;
}

void G_EDM_UI_CONTROLLER::set_gcode_file(String filename)
{
  if (filename.length() <= 0)
  {
    if (operation_mode > 2)
    {
      // operation_mode = 1; // default to drill mode only after gcode job is done
    }
  }
  else
  {
    //operation_mode = 3;set_operation_mode
  }
  gcode_file = filename;
}
void G_EDM_UI_CONTROLLER::set_debug_bg_color(int num)
{
  debug_bg_color = num;
};
void G_EDM_UI_CONTROLLER::set_workload_travel(float total_workload_travel)
{
  this->total_workload_travel = total_workload_travel;
}
void G_EDM_UI_CONTROLLER::set_total_iterations(int total_iterations)
{
  this->total_iterations = total_iterations;
}
void G_EDM_UI_CONTROLLER::set_short_iterations(int short_iterations)
{
  this->short_iterations = short_iterations;
}
void G_EDM_UI_CONTROLLER::set_total_time(unsigned long total_time)
{
  this->total_time = total_time;
}
void G_EDM_UI_CONTROLLER::set_travel_time(unsigned long travel_time)
{
  this->travel_time = travel_time;
}
void G_EDM_UI_CONTROLLER::set_z_pos(float value)
{
  z_pos = value;
}
void G_EDM_UI_CONTROLLER::set_z_stop(float value)
{
  z_stop_after_mm = value; // global
}
void G_EDM_UI_CONTROLLER::set_ms_edm(int value)
{
  ms_edm = value;
}
void G_EDM_UI_CONTROLLER::set_ms_travel(int value)
{
  ms_travel = value;
}
void G_EDM_UI_CONTROLLER::set_drop_min_max(float min, float max)
{
  drop_min = min;
  drop_max = max;
}
void G_EDM_UI_CONTROLLER::set_drop_short(float value)
{
  drop_short = value;
}
void G_EDM_UI_CONTROLLER::set_voltage_min(float value)
{
  voltage_min = value;
}
void G_EDM_UI_CONTROLLER::set_z_no_home(bool value)
{
  z_no_home = value;
}
void G_EDM_UI_CONTROLLER::set_reamer_travel_mm(float _mm)
{
  reamer_travel_mm = _mm;
}
void G_EDM_UI_CONTROLLER::set_reamer_duration(float _seconds)
{
  reamer_duration = _seconds;
}
void G_EDM_UI_CONTROLLER::set_operation_mode(int mode)
{
  operation_mode = mode;
  if( mode > 2 && ! filehandler->get_has_sd_card() ){
    mode = 1; // fallback if no sd card is inserted
  }
  if( mode == 4 ){
    probe_dimension = 1;
  } else{
    probe_dimension = 0;
  }
}
void G_EDM_UI_CONTROLLER::set_flush_retract_after(float _milliseconds)
{
  flush_retract_after = _milliseconds;
}
void G_EDM_UI_CONTROLLER::set_flush_retract_mm(float _mm)
{
  flush_retract_mm = _mm;
}
void G_EDM_UI_CONTROLLER::set_disable_spark_for_flushing(bool disable)
{
  disable_spark_for_flushing = disable;
}
void G_EDM_UI_CONTROLLER::set_flush_offset_steps(int _steps)
{
  flush_offset_steps = _steps;
}
void G_EDM_UI_CONTROLLER::set_active_profile(String profile)
{
  this->active_profile = profile;
}

int G_EDM_UI_CONTROLLER::get_active_page()
{
  return active_page;
}
void G_EDM_UI_CONTROLLER::set_active_page(int page_num)
{
  active_page = page_num;
}
/** converting seconds to different units s/ms/us/ns **/
String G_EDM_UI_CONTROLLER::convert_timings(float seconds)
{
  if (seconds > 60)
  {
    return String(seconds / 60.0) + "min";
  }
  else if (seconds > 0.1)
  {
    return String(seconds) + "s";
  }
  else if (seconds > 0.001)
  {
    return String(seconds * 1000.0) + "ms";
  }
  else if (seconds > 0.000001)
  {
    return String(seconds * 1000000.0) + "us";
  }
  else
  {
    return String(seconds * 1000000000.0) + "ns";
  }
}
float G_EDM_UI_CONTROLLER::get_keyboard_result()
{
  return keyboard_value.length() > 0 ? keyboard_value.toFloat() : keyboard_initial_value;
}
String G_EDM_UI_CONTROLLER::get_keyboard_result_alpha()
{
  return keyboard_value.length() > 0 ? String(keyboard_value) : "";
}
void G_EDM_UI_CONTROLLER::close_keyboard()
{
  keyboard_is_active = false;
  has_first_stroke = false;
  spark_indicator_bar_width = 0;
}
void G_EDM_UI_CONTROLLER::open_keyboard_alpha(String value, String text, String infotext)
{
  if (keyboard_is_active)
  {
    return;
  }
  has_first_stroke = false;
  tft.fillScreen(TFT_BLACK);
  tft.fillScreen(TFT_BLACK);
  keyboard_value = String(value);
  keyboard_initial_value_alpha = value;
  keyboard_is_active = true;
  int w = 30;
  int h = 30;
  int margin = 5;
  int px = margin;
  int py = margin;
  int t = 0;
  int c = 0;
  const char *data_sets[] = {"1", "2", "3", "4", "5", "6", "7", "8", "9", "0", "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z", "-", "_"};
  tft.setTextSize(1);
  tft.setTextColor(TFT_BLACK);
  while (t++ <= 37)
  {
    vTaskDelay(1);
    if (c++ >= 10)
    {
      c = 1;
      py = py + h + 1;
      px = margin;
    }
    tft.fillRect(px, py, w, h, TFT_WHITE);
    tft.drawString(String(data_sets[t - 1]), px + 10, py + 10, 2);
    px = px + 1 + w;
  }
  /** delete button **/
  tft.fillRect(px, py, w * 2 + 1, h, TFT_RED);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("DEL", px + 20, py + 10, 2);
  /** text field **/
  py = py + h + 1;
  tft.fillRect(margin, py, 309, h + 10, TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.drawString(keyboard_value, margin + 10, py + 10, 2);
  tft.setTextColor(TFT_YELLOW);
  tft.drawString(text, margin + 10, py + 40, 2);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  /** ok button **/
  py = py + h + 1;
  tft.fillRect(margin, py + 1 + 40, 248, h, TFT_MAROON);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("Cancel", margin + 90, py + 48, 2);
  tft.fillRect(254, py + 1 + 40, 60, h, TFT_GREEN);
  tft.setTextColor(TFT_BLACK);
  tft.drawString("OK", 253 + 25, py + 48, 2);
  bool block_touch = false;
  uint16_t x, y;
  while (keyboard_is_active)
  {
    vTaskDelay(1);
    if (tft.getTouch(&x, &y, 500))
    {
      if (block_touch)
      {
        continue;
      }
      block_touch = true;
      if (x >= 5 && x <= 35 && y >= 5 && y <= 35)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "1";
      }
      else if (x >= 36 && x <= 66 && y >= 5 && y <= 35)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "2";
      }
      else if (x >= 67 && x <= 97 && y >= 5 && y <= 35)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "3";
      }
      else if (x >= 98 && x <= 128 && y >= 5 && y <= 35)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "4";
      }
      else if (x >= 129 && x <= 159 && y >= 5 && y <= 35)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "5";
      }
      else if (x >= 160 && x <= 190 && y >= 5 && y <= 35)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "6";
      }
      else if (x >= 191 && x <= 221 && y >= 5 && y <= 35)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "7";
      }
      else if (x >= 222 && x <= 252 && y >= 5 && y <= 35)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "8";
      }
      else if (x >= 253 && x <= 283 && y >= 5 && y <= 35)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "9";
      }
      else if (x >= 284 && x <= 314 && y >= 5 && y <= 35)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "0";
      }
      else if (x >= 5 && x <= 35 && y >= 36 && y <= 66)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "A";
      }
      else if (x >= 36 && x <= 66 && y >= 36 && y <= 66)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "B";
      }
      else if (x >= 67 && x <= 97 && y >= 36 && y <= 66)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "C";
      }
      else if (x >= 98 && x <= 128 && y >= 36 && y <= 66)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "D";
      }
      else if (x >= 129 && x <= 159 && y >= 36 && y <= 66)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "E";
      }
      else if (x >= 160 && x <= 190 && y >= 36 && y <= 66)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "F";
      }
      else if (x >= 191 && x <= 221 && y >= 36 && y <= 66)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "G";
      }
      else if (x >= 222 && x <= 252 && y >= 36 && y <= 66)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "H";
      }
      else if (x >= 253 && x <= 283 && y >= 36 && y <= 66)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "I";
      }
      else if (x >= 284 && x <= 314 && y >= 36 && y <= 66)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "J";
      }
      else if (x >= 5 && x <= 35 && y >= 67 && y <= 97)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "K";
      }
      else if (x >= 36 && x <= 66 && y >= 67 && y <= 97)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "L";
      }
      else if (x >= 67 && x <= 97 && y >= 67 && y <= 97)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "M";
      }
      else if (x >= 98 && x <= 128 && y >= 67 && y <= 97)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "N";
      }
      else if (x >= 129 && x <= 159 && y >= 67 && y <= 97)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "O";
      }
      else if (x >= 160 && x <= 190 && y >= 67 && y <= 97)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "P";
      }
      else if (x >= 191 && x <= 221 && y >= 67 && y <= 97)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "Q";
      }
      else if (x >= 222 && x <= 252 && y >= 67 && y <= 97)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "R";
      }
      else if (x >= 253 && x <= 283 && y >= 67 && y <= 97)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "S";
      }
      else if (x >= 284 && x <= 314 && y >= 67 && y <= 97)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "T";
      }
      else if (x >= 5 && x <= 35 && y >= 98 && y <= 128)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "U";
      }
      else if (x >= 36 && x <= 66 && y >= 98 && y <= 128)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "V";
      }
      else if (x >= 67 && x <= 97 && y >= 98 && y <= 128)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "W";
      }
      else if (x >= 98 && x <= 128 && y >= 98 && y <= 128)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "X";
      }
      else if (x >= 129 && x <= 159 && y >= 98 && y <= 128)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "Y";
      }
      else if (x >= 160 && x <= 190 && y >= 98 && y <= 128)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "Z";
      }
      else if (x >= 191 && x <= 221 && y >= 98 && y <= 128)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "-";
      }
      else if (x >= 222 && x <= 252 && y >= 98 && y <= 128)
      {
        if (!has_first_stroke)
        {
          has_first_stroke = true;
          keyboard_value = "";
        }
        keyboard_value += "_";
      }
      else if (x >= 253 && x <= 313 && y >= 98 && y <= 128)
      {
        // delete
        keyboard_value = "";
      }
      else if (x >= 5 && x <= 254 && y >= 201 && y <= 240)
      {
        // cancel
        keyboard_value = "";
        break;
      }
      else if (x >= 253 && y >= 201 && y <= 240)
      {
        // ok
        break;
      }
      else
      {
        continue;
      }
      tft.fillRect(5, 129, 309, 40, TFT_BLACK);
      tft.setTextColor(TFT_WHITE);
      tft.drawString(keyboard_value, 15, 139, 2);
    }
    else
    {
      block_touch = false;
    }
    vTaskDelay(1);
  }
  close_keyboard();
}
void G_EDM_UI_CONTROLLER::open_keyboard(float value, String text, String infotext)
{
  if (keyboard_is_active)
  {
    return;
  }
  tft.fillScreen(TFT_BLACK);
  has_first_stroke = false;
  keyboard_value = String(value);
  keyboard_initial_value = value;
  keyboard_is_active = true;
  int w = button_width;
  int h = button_height;
  int margin = button_margin;
  tft.setTextSize(2);
  tft.setTextColor(TFT_BLACK);
  tft.fillRect(0, 0, 320, 2 * margin + h, TFT_BLACK);
  // 0-9
  tft.fillRect(margin, margin, w, h, TFT_WHITE);
  tft.drawString("1", margin + w / 2, margin, 2);
  tft.fillRect(2 * margin + w, margin, w, h, TFT_WHITE);
  tft.drawString("2", 2 * margin + w + w / 2, margin, 2);
  tft.fillRect(3 * margin + 2 * w, margin, w, h, TFT_WHITE);
  tft.drawString("3", 3 * margin + 2 * w + w / 2, margin, 2);
  tft.fillRect(4 * margin + 3 * w, margin, w, h, TFT_WHITE);
  tft.drawString("4", 4 * margin + 3 * w + w / 2, margin, 2);
  tft.fillRect(5 * margin + 4 * w, margin, w, h, TFT_WHITE);
  tft.drawString("5", 5 * margin + 4 * w + w / 2, margin, 2);
  tft.fillRect(margin, 2 * margin + h, w, h, TFT_WHITE);
  tft.drawString("6", margin + w / 2, 2 * margin + h, 2);
  tft.fillRect(2 * margin + w, 2 * margin + h, w, h, TFT_WHITE);
  tft.drawString("7", 2 * margin + w + w / 2, 2 * margin + h, 2);
  tft.fillRect(3 * margin + 2 * w, 2 * margin + h, w, h, TFT_WHITE);
  tft.drawString("8", 3 * margin + 2 * w + w / 2, 2 * margin + h, 2);
  tft.fillRect(4 * margin + 3 * w, 2 * margin + h, w, h, TFT_WHITE);
  tft.drawString("9", 4 * margin + 3 * w + w / 2, 2 * margin + h, 2);
  tft.fillRect(5 * margin + 4 * w, 2 * margin + h, w, h, TFT_WHITE);
  tft.drawString("0", 5 * margin + 4 * w + w / 2, 2 * margin + h, 2);
  // undo
  tft.fillRect(6 * margin + 5 * w, margin, w, h, TFT_RED);
  tft.drawString("DEL", 6 * margin + 5 * w + 2, 2 * margin, 2);
  // ,
  tft.fillRect(6 * margin + 5 * w, 2 * margin + h, w, h, TFT_WHITE);
  tft.drawString(",", 6 * margin + 5 * w + w / 2, 2 * margin + h, 2);
  // result field
  draw_keyboard_result();
  // done
  tft.fillRect(5 * margin + 4 * w, 3 * margin + 2 * h, 2 * w + margin, h, TFT_GREEN);
  tft.drawString("OK", 5 * margin + 4 * w + w - margin, 3 * margin + 2 * h + margin, 2);
  // textbox title
  // tft.fillRect(margin, 4 * margin + 3 * h, 6 * w + 5 * margin, 2 * h, TFT_YELLOW);
  // tft.setTextColor(TFT_BLACK);
  tft.setTextColor(TFT_GREEN);

  tft.setTextSize(1);
  tft.drawString(text, 2 * margin, 5 * margin + 3 * h, 2);

  tft.setTextColor(TFT_LIGHTGREY);

  // infotext
  if (infotext != "")
  {
    int size;
    String *t = split(infotext, '\r', size);

    if (size <= 1)
    {
      tft.drawString(infotext, 2 * margin, 5 * margin + 3 * h + 20, 2);
    }
    else
    {

      for (int i = 0; i < size; i++)
      {
        tft.drawString(t[i], 2 * margin, 5 * margin + 3 * h + 20 + (i * 20), 2);
      }
    }

    delete[] t;
  }

  bool block_touch = false;
  uint16_t x, y;
  while (keyboard_is_active)
  {
    vTaskDelay(1);
    if (tft.getTouch(&x, &y, 500))
    {
      if (block_touch)
      {
        continue;
      }
      block_touch = true;
      map_keys(x, y);
    }
    else
    {
      block_touch = false;
    }
    vTaskDelay(1);
  }
  close_keyboard();
}
void G_EDM_UI_CONTROLLER::draw_keyboard_result()
{
  int w = button_width;
  int h = button_height;
  int margin = button_margin;
  tft.setTextSize(2);
  tft.setTextColor(TFT_BLACK);
  tft.fillRect(margin, 3 * margin + 2 * h, 4 * w + 3 * margin, h, TFT_WHITE);
  tft.drawString(keyboard_value, 2 * margin, 4 * margin + 2 * h, 2);
}
void G_EDM_UI_CONTROLLER::map_keys(int x, int y)
{
  int w = button_width;
  int h = button_height;
  int margin = button_margin;
  int r = 500;
  if (x >= margin && x <= margin + w && y >= margin && y <= margin + h)
  {
    // 1
    r = 1;
  }
  else if (x >= 2 * margin + w && x <= 2 * margin + 2 * w && y >= margin && y <= margin + h)
  {
    // 2
    r = 2;
  }
  else if (x >= 3 * margin + 2 * w && x <= 3 * margin + 3 * w && y >= margin && y <= margin + h)
  {
    // 3
    r = 3;
  }
  else if (x >= 4 * margin + 3 * w && x <= 4 * margin + 4 * w && y >= margin && y <= margin + h)
  {
    // 4
    r = 4;
  }
  else if (x >= 5 * margin + 4 * w && x <= 5 * margin + 5 * w && y >= margin && y <= margin + h)
  {
    // 5
    r = 5;
  }
  else if (x >= 6 * margin + 5 * w && x <= 6 * margin + 6 * w && y >= margin && y <= margin + h)
  {
    // del
    r = 100;
  }
  else if (x >= margin && x <= margin + w && y >= 2 * margin + h && y <= 2 * margin + 2 * h)
  {
    // 6
    r = 6;
  }
  else if (x >= 2 * margin + w && x <= 2 * margin + 2 * w && y >= 2 * margin + h && y <= 2 * margin + 2 * h)
  {
    // 7
    r = 7;
  }
  else if (x >= 3 * margin + 2 * w && x <= 3 * margin + 3 * w && y >= 2 * margin + h && y <= 2 * margin + 2 * h)
  {
    // 8
    r = 8;
  }
  else if (x >= 4 * margin + 3 * w && x <= 4 * margin + 4 * w && y >= 2 * margin + h && y <= 2 * margin + 2 * h)
  {
    // 9
    r = 9;
  }
  else if (x >= 5 * margin + 4 * w && x <= 5 * margin + 5 * w && y >= 2 * margin + h && y <= 2 * margin + 2 * h)
  {
    // 0
    r = 0;
  }
  else if (x >= 6 * margin + 5 * w && x <= 6 * margin + 6 * w && y >= 2 * margin + h && y <= 2 * margin + 2 * h)
  {
    // ,
    r = 200;
  }
  else if (x >= 4 * margin + 3 * w && x <= 6 * margin + 6 * w && y >= 3 * margin + 2 * h && y <= 3 * margin + 3 * h)
  {
    // ok
    r = 300;
  }
  if (r == 100)
  {
    // delete
    keyboard_value = "";
  }
  else if (r == 300)
  {
    // done
    close_keyboard();
    return;
  }
  else if (r == 200)
  {
    if (!has_first_stroke)
    {
      has_first_stroke = true;
      keyboard_value = "";
    }
    if (keyboard_value.indexOf(".") == -1)
    {
      keyboard_value += ".";
    }
  }
  else if (r == 500)
  {
    // nothing
  }
  else
  {
    if (!has_first_stroke)
    {
      has_first_stroke = true;
      keyboard_value = "";
    }
    keyboard_value += String(r);
  }
  draw_keyboard_result();
}

void G_EDM_UI_CONTROLLER::draw_spark_on_off_indicator_icon()
{
  if (keyboard_is_active)
  {
    return;
  }
  int state; // 0=gray 1=green 2=red

  if (pwm_is_enabled())
  {
    state = 2; // allow disable
  }
  else
  {
    state = 1; // allow enable
  }

  if (get_source_voltage() < source_voltage_min)
  {
    state = 0; // voltage too low
  }

  if (state == spark_on_off_indicator_icon_last_state)
  {
    // no changes
    return;
  }

  spark_on_off_indicator_icon_last_state = state;

  tft.fillRect(249, 169, 62, 62, TFT_BLACK);
  tft.setTextSize(1);

  switch (state)
  {

  case 0:
    tft.setTextColor(TFT_BLACK);
    tft.fillRoundRect(250, 170, 60, 60, 5, TFT_DARKGREY);
    tft.drawString("EDM", 255, 180, 2);
    tft.drawString("START", 255, 200, 2);
    break;

  case 1:
    tft.setTextColor(TFT_BLACK);
    tft.fillRoundRect(250, 170, 60, 60, 5, TFT_OLIVE);
    tft.drawString("EDM", 255, 180, 2);
    tft.drawString("START", 255, 200, 2);
    break;

  case 2:
    tft.setTextColor(TFT_WHITE);
    tft.fillRoundRect(250, 170, 60, 60, 5, TFT_RED);
    tft.drawString("EDM", 255, 180, 2);
    tft.drawString("STOP", 255, 200, 2);
    break;
  }

  vTaskDelay(1);
}
void G_EDM_UI_CONTROLLER::render_page_settings_menu()
{
  /** buttons **/
  tft.fillRoundRect(10, 10, 80, 30, 5, TFT_OLIVE);
  tft.fillRoundRect(10, 49, 80, 30, 5, TFT_OLIVE);
  tft.fillRoundRect(10, 88, 80, 30, 5, TFT_OLIVE);
  tft.fillRoundRect(10, 127, 80, 30, 5, TFT_OLIVE);
  tft.fillRoundRect(10, 166, 80, 30, 5, TFT_OLIVE);
  /** text **/
  tft.setTextSize(1);
  tft.setTextColor(TFT_BLACK);
  tft.drawString("PWM", 20, 15, 2);
  tft.drawString("Flushing", 20, 54, 2);
  tft.drawString("Spark", 20, 93, 2);
  tft.drawString("Mode", 20, 132, 2);
  tft.drawString("Motion", 20, 171, 2);

  render_page_settings_menu_sd_card(has_sd_card, true);

  vTaskDelay(1);
}

void G_EDM_UI_CONTROLLER::draw_sdcard_button( int _state ){

  if (keyboard_is_active || active_page != 1)
  {
    return;
  }

  tft.setTextSize(1);

  switch (_state)
  {
    case 0:
      tft.setTextColor(TFT_WHITE);
      tft.fillRoundRect(10, 205, 80, 30, 5, TFT_MAROON);
    break;
  
    case 1:
      tft.setTextColor(TFT_WHITE);
      tft.fillRoundRect(10, 205, 80, 30, 5, TFT_DARKGREY);
    break;

    case 2:
      tft.setTextColor(TFT_BLACK);
      tft.fillRoundRect(10, 205, 80, 30, 5, TFT_OLIVE);
    break;

    default:
      tft.setTextColor(TFT_BLACK);
      tft.fillRoundRect(10, 205, 80, 30, 5, TFT_OLIVE);
    break;

  }

  tft.drawString("SD_Card", 20, 210, 2);
  draw_mode_buttons();

}

void G_EDM_UI_CONTROLLER::render_page_settings_menu_sd_card(bool sd_card_has, bool force_redraw)
{

  bool is_locked_for_ui = filehandler->is_locked_for_ui();

  if (is_locked_for_ui)
  {
    sd_card_has = false;
  }

  if (has_sd_card == sd_card_has && !force_redraw)
  {
    return;
  }

  if (!sd_card_has)
  {
    set_gcode_file("");
  }

  has_sd_card = sd_card_has;
  tft.setTextSize(1);
  if (!has_sd_card)
  {
    draw_sdcard_button( is_locked_for_ui ? 1 : 0 );
  }
  else
  {
    draw_sdcard_button( 2 );
  }
  tft.drawString("SD_Card", 20, 210, 2);
  vTaskDelay(1);
}
/** Draws the values for the input PSU voltage and the vSense voltage reading **/
void G_EDM_UI_CONTROLLER::redraw_vsense()
{
  if (keyboard_is_active || active_page != 1)
  {
    return;
  }
  tft.fillRect(140, 0, 60, 26, TFT_BLACK);
  tft.fillRect(267, 0, 60, 26, TFT_BLACK);
  int color = (get_source_voltage() < voltage_min) ? TFT_RED : TFT_GREEN;
  tft.setTextColor(color);
  tft.setTextSize(1);
  tft.drawString(String(get_source_voltage()) + "v", 145, 10, 2);
  tft.drawString(String(get_feedback_voltage()) + "v", 270, 10, 2);
  vTaskDelay(1);
}
/** Draws the indicatorbar **/
void G_EDM_UI_CONTROLLER::draw_mpos()
{
  if (keyboard_is_active || active_page != 1)
  {
    return;
  }
  tft.fillRect(109, 34, 211, 26, TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(TFT_LIGHTGREY);
  tft.drawString(get_mpos_string(), 110, 35, 2);
  vTaskDelay(1);
}
/** PWM wave **/
void G_EDM_UI_CONTROLLER::draw_period_wave()
{
  if (keyboard_is_active)
  {
    return;
  }
  String text = convert_frequency_units(get_freq()) + " @ " + String(get_duty_percent()) + "%";
  float max_width_for_duty = 200.0;
  tft.fillRect(109, 61, 202, 32, TFT_BLACK);
  tft.fillRect(99, 215, 150, 20, TFT_BLACK);
  if (!pwm_is_enabled())
  {
    tft.setTextSize(1);
    tft.setTextColor(TFT_RED);
    tft.drawString("PWM OFF " + text, 110, 71, 2);
  }
  else
  {
    // get the length of the duty line
    int duty_line_length = round(float(get_duty_percent()) / 100.0 * max_width_for_duty);
    if (duty_line_length <= 10)
    {
      duty_line_length = 10;
    }
    // raising edge
    tft.drawLine(110, 62, 110, 92, TFT_BLUE);
    // t_on
    tft.drawLine(110, 62, 110 + duty_line_length, 62, TFT_BLUE);
    // falling edge
    tft.drawLine(110 + duty_line_length, 62, 110 + duty_line_length, 92, TFT_BLUE);
    // t_off
    tft.drawLine(110 + duty_line_length, 92, 110 + max_width_for_duty, 92, TFT_BLUE);
  }
  tft.setTextSize(1);
  tft.setTextColor(TFT_OLIVE);
  tft.setTextColor(TFT_PURPLE);


  //tft.drawString(convert_frequency_units(get_freq()) + " @ " + String(get_duty_percent()) + "% Duty", 110, 105, 2);
  //tft.drawString("Setpoint: " + String(drop_min) + "% - " + String(drop_max) + "%", 110, 125, 2);

  if ( operation_mode == 3 || operation_mode == 4 )
  {
    tft.setTextColor(TFT_LIGHTGREY);
    tft.drawString("gCode: " + (gcode_file.length() > 0 ? gcode_file : "No file selected"), 110, 145, 2);
  }

  vTaskDelay(1);
}


void G_EDM_UI_CONTROLLER::draw_mode_buttons()

{

  if (keyboard_is_active || active_page != 1)
  {
    return;
  }

  int width  = 45;
  int height = 30;
  int margin = 5;
  int active_color = TFT_GREENYELLOW;
      tft.setTextColor(TFT_BLACK);
      tft.fillRoundRect(110, 105, width, height, 5, operation_mode==1?active_color:TFT_OLIVE);
      tft.fillRoundRect(110+margin+width, 105, width, height, 5, operation_mode==2?active_color:TFT_OLIVE);
      tft.drawString("Drill", 118,         111, 2);
      tft.drawString("Ream",  118+5+width, 111, 2);
      if( ! filehandler->get_has_sd_card() ){
          tft.setTextColor(TFT_LIGHTGREY);
          tft.fillRoundRect(110+2*margin+2*width, 105, width, height, 5, TFT_DARKGREY);
          tft.fillRoundRect(110+3*margin+3*width, 105, width, height, 5, TFT_DARKGREY);
      } else{
          tft.fillRoundRect(110+2*margin+2*width, 105, width, height, 5, operation_mode==3?active_color:TFT_OLIVE);
          tft.fillRoundRect(110+3*margin+3*width, 105, width, height, 5, operation_mode==4?active_color:(EXTENDED_FEATURES?TFT_OLIVE:TFT_DARKGREY));
      }

      tft.drawString("3D", 117+2*5+2*width, 105, 2);
      tft.drawString("Float", 117+2*5+2*width, 117, 2);
      tft.drawString("2D",  119+3*5+3*width, 105, 2);
      tft.drawString("Wire",  119+3*5+3*width, 117, 2);

}



void G_EDM_UI_CONTROLLER::draw_interface()
{
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  /** input voltage and vSense voltage **/
  tft.setTextColor(TFT_OLIVE);
  tft.drawString("PSU", 110, 10, 2);
  tft.drawString("vSense", 215, 10, 2);
  render_page_settings_menu();
  vTaskDelay(1);
}
bool G_EDM_UI_CONTROLLER::motion_input_is_blocked()
{
  if ( edm_process_is_running 
       || sys_rt_exec_state.bit.motionCancel 
       || sys.state == State::CheckMode )
  {
    return true;
  }

  return false;
}
void G_EDM_UI_CONTROLLER::draw_motion_navigation_buttons()
{

  /** motion controller **/
  int color = TFT_GREEN;
  if (
      motion_input_is_blocked()
  )
  {
    color = TFT_DARKGREY;
  }
  // tft.setTextColor( TFT_DARKGREEN );
  tft.fillTriangle(220, 180, 210, 200, 230, 200, color);
  tft.fillTriangle(210, 210, 220, 230, 230, 210, color);
  tft.fillTriangle(150, 180, 140, 200, 160, 200, color);
  tft.fillTriangle(140, 210, 150, 230, 160, 210, color);
  tft.fillTriangle(170, 195, 170, 215, 190, 205, color);
  tft.fillTriangle(110, 205, 130, 215, 130, 195, color);
}
void G_EDM_UI_CONTROLLER::render_page_front()
{
  // tft.fillScreen(TFT_BLACK);
  /** feedback status **/
  /** basic infos **/
  /**  **/
  spark_on_off_indicator_icon_last_state = -1; // reset
  draw_spark_on_off_indicator_icon();
  draw_mpos();
  redraw_vsense();
  draw_period_wave();
  draw_motion_navigation_buttons();
  draw_mode_buttons();
}
void G_EDM_UI_CONTROLLER::render_page_settings_pwm()
{
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(TFT_OLIVE);
  tft.drawString("PWM Settings", 10, 10, 2);
  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKGREEN);
  tft.drawString("Frequency", 20, 40, 2);
  tft.drawString("Duty Cycle", 20, 70, 2);
  tft.setTextSize(1);
  tft.setTextColor(TFT_OLIVE);
  tft.drawString("Period", 20, 110, 2);
  tft.drawString("T_ON", 20, 130, 2);
  tft.drawString("T_OFF", 20, 150, 2);
  String unit;
  float value;
  tft.setTextSize(1);
  tft.setTextColor(TFT_GREEN);
  tft.drawString(convert_frequency_units(get_freq()), 100, 40, 2);
  tft.drawString(String(get_duty_percent()) + "%", 100, 70, 2);
  tft.setTextSize(1);
  tft.drawString(convert_timings(get_period()), 100, 110, 2);
  tft.drawString(convert_timings(get_t_on()), 100, 130, 2);
  tft.drawString(convert_timings(get_t_off()), 100, 150, 2);
  tft.setTextSize(1);
  tft.setTextColor(TFT_BLACK);
  tft.fillRoundRect(10, 190, 300, 40, 5, TFT_OLIVE);
  tft.drawString("Done", 20, 200, 2);
}
void G_EDM_UI_CONTROLLER::render_page_settings_flushing()
{
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(TFT_OLIVE);
  tft.drawString("Flushing Settings", 10, 10, 2);
  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKGREEN);
  tft.drawString("Interval", 20, 40, 2);
  tft.drawString("Distance", 20, 70, 2);
  tft.drawString("Disable spark", 20, 100, 2);
  tft.drawString("Offset Steps", 20, 130, 2);
  tft.setTextColor(TFT_GREEN);
  tft.drawString(String(flush_retract_after) + "ms", 120, 40, 2);
  tft.drawString(String(flush_retract_mm) + "mm", 120, 70, 2);
  tft.drawString(String(disable_spark_for_flushing ? "YES" : "NO"), 120, 100, 2);
  tft.drawString(String(flush_offset_steps), 120, 130, 2);
  tft.setTextSize(1);
  tft.setTextColor(TFT_BLACK);
  tft.fillRoundRect(10, 190, 300, 40, 5, TFT_OLIVE);
  tft.drawString("Done", 20, 200, 2);
}

void G_EDM_UI_CONTROLLER::render_page_settings_spark()
{
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(TFT_OLIVE);
  tft.drawString("Spark Settings", 10, 10, 2);
  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKGREEN);
  tft.drawString("Setpoint Min", 20, 40, 2);
  tft.drawString("Setpoint Max", 20, 70, 2);
  tft.drawString("Minimum PSU voltage", 20, 100, 2);
  tft.setTextColor(TFT_GREEN);
  tft.drawString(String(drop_min) + "%", 160, 40, 2);
  tft.drawString(String(drop_max) + "%", 160, 70, 2);
  tft.drawString(String(voltage_min) + "v", 160, 100, 2);
  tft.setTextSize(1);
  tft.setTextColor(TFT_BLACK);
  tft.fillRoundRect(10, 190, 300, 40, 5, TFT_OLIVE);
  tft.drawString("Done", 20, 200, 2);
}
void G_EDM_UI_CONTROLLER::render_page_settings_mode()
{
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(TFT_OLIVE);
  tft.drawString("Mode Settings", 10, 10, 2);
  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKGREEN);
  tft.drawString("Mode", 20, 40, 2);
  tft.setTextColor(TFT_GREEN);
  if (operation_mode == 1)
  {
    tft.drawString("Drill / Sinker", 100, 40, 2);
    tft.setTextColor(TFT_DARKGREEN);
    tft.drawString("Cutting depth", 20, 80, 2);
    tft.setTextColor(TFT_GREEN);
    tft.drawString(String(z_stop_after_mm) + "mm", 160, 80, 2);
  }
  else if (operation_mode == 2)
  {
    tft.drawString("Reamer", 100, 40, 2);
    tft.setTextColor(TFT_DARKGREEN);
    tft.drawString("Oscillation distance", 20, 80, 2);
    tft.drawString("Duration", 20, 110, 2);
    tft.setTextColor(TFT_GREEN);
    tft.drawString(String(reamer_travel_mm) + "mm", 160, 80, 2);
    tft.drawString(String(reamer_duration) + "s", 160, 110, 2);
  }
  else if (operation_mode == 3)
  {
    tft.drawString("3D Floating GCode", 100, 40, 2);
    tft.setTextColor(TFT_DARKGREEN);
    tft.drawString("Cutting depth", 20, 80, 2);
    tft.drawString("Simulate GCode", 20, 110, 2);
    tft.setTextColor(TFT_GREEN);
    tft.drawString(String(z_stop_after_mm) + "mm", 160, 80, 2);
    tft.drawString((simulate_gcode ? "YES" : "NO"), 160, 110, 2);
  }
  else if (operation_mode == 4)
  {
    tft.drawString("2D Wire GCode", 100, 40, 2);
    tft.setTextColor(TFT_DARKGREEN);
    tft.drawString("Wire spindle", 20, 80, 2);
    tft.drawString("Simulate GCode", 20, 110, 2);
    tft.setTextColor(TFT_GREEN);
    tft.drawString(String(frequency_to_rpm(wire_spindle_speed)) + "rpm", 160, 80, 2);
    tft.drawString((simulate_gcode ? "YES" : "NO"), 160, 110, 2);

  }


  if (operation_mode <= 4)
  {
    tft.setTextColor(TFT_DARKGREEN);
    tft.drawString("Enable spindle", 20, 140, 2);
    tft.setTextColor(TFT_GREEN);
    tft.drawString(enable_spindle ? "ON" : "OFF", 160, 140, 2);
  }
  tft.setTextSize(1);
  tft.setTextColor(TFT_BLACK);
  tft.fillRoundRect(10, 190, 300, 40, 5, TFT_OLIVE);
  tft.drawString("Done", 20, 200, 2);
}

void G_EDM_UI_CONTROLLER::draw_homing_buttons(bool enabled)
{

  tft.fillRect(10, 129, 300, 42, TFT_LIGHTGREY);

  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(1);

  if (sys_axis_homed[X_AXIS])
  {
    tft.drawString("Homed!", 95, 100, 2);
  }
  if (sys_axis_homed[Y_AXIS])
  {
    tft.drawString("Homed!", 165, 100, 2);
  }
  if (sys_axis_homed[Z_AXIS] || z_no_home)
  {
    tft.drawString("Homed!", 235, 100, 2);
  }

  if (!enabled||sys_rt_exec_state.bit.motionCancel)
  {
    tft.fillRoundRect(20, 130, 60, 40, 5, TFT_DARKGREY);
    tft.fillRoundRect(90, 130, 60, 40, 5, TFT_DARKGREY);
    tft.fillRoundRect(160, 130, 60, 40, 5, TFT_DARKGREY);
    tft.fillRoundRect(230, 130, 60, 40, 5, TFT_DARKGREY);
  }
  else
  {
    tft.fillRoundRect(20, 130, 60, 40, 5, TFT_OLIVE);
    tft.fillRoundRect(90, 130, 60, 40, 5, TFT_OLIVE);
    tft.fillRoundRect(160, 130, 60, 40, 5, TFT_OLIVE);
    tft.fillRoundRect(230, 130, 60, 40, 5, TFT_OLIVE);
  }
  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(2);
  tft.drawString("ZXY", 27, 132, 2);
  tft.drawString("X", 112, 132, 2);
  tft.drawString("Y", 182, 132, 2);
  tft.drawString("Z", 252, 132, 2);
}

String G_EDM_UI_CONTROLLER::convert_frequency_units(int frequency)
{
  String text;
  if (frequency > 1000000)
  {
    return String(float(frequency) / 1000000.0) + "MHz";
  }
  else if (frequency > 1000)
  {
    return String(float(frequency) / 1000.0) + "KHz";
  }
  else
  {
    return String(float(frequency)) + "Hz";
  }
}


void G_EDM_UI_CONTROLLER::draw_toggle_probe_dimension()
{
  tft.setTextSize(1);
  tft.setTextColor(( probe_dimension == 0 ? TFT_WHITE : TFT_BLACK ));
  tft.fillRoundRect(155, 125, 37, 45, 5, ( probe_dimension == 0 ? TFT_DARKCYAN : TFT_GREEN ) );
  tft.drawString( ( probe_dimension == 0 ? "3D" : "2D"), 165, 140, 2);
}



void G_EDM_UI_CONTROLLER::draw_set_reprobe_button()
{
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE);
  tft.fillRoundRect(200, 125, 100, 45, 5, TFT_OLIVE);
  tft.drawString("Reprobe MPos", 205, 130, 2);
  tft.drawString("X" + String(sys.gedm_probe_position_x) + " Y" + String(sys.gedm_probe_position_y), 205, 145, 2);
}

void G_EDM_UI_CONTROLLER::draw_probing_buttons( int active )
{

  if( active != 0 ){
    render_page_settings_motion();
  }

  draw_toggle_probe_dimension();

  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKGREEN);
  tft.drawString("Duty", 160, 50, 2);
  tft.drawString("Frequency", 160, 75, 2);
  tft.drawString("Trigger", 160, 100, 2);

  draw_set_reprobe_button();

  tft.setTextColor(TFT_MAROON);
  tft.drawString(String(pwm_duty_probing) + "%", 230, 50, 2);
  tft.drawString(convert_frequency_units(pwm_frequency_probing), 230, 75, 2);
  tft.drawString(String(vSense_drop_range_noload) + "%", 230, 100, 2);

  if (!machine_is_homed())
  {

    tft.setTextSize(1);
    tft.setTextColor(TFT_BLACK);
  }

  if (get_source_voltage() >= source_voltage_min && !motion_input_is_blocked()) 
  {

    sys_axis_homed[X_AXIS] = true;
    sys_axis_homed[Y_AXIS] = true;

    int active_color_one = TFT_DARKCYAN;
    if( probe_dimension == 1 ){
      active_color_one = TFT_GREEN;
    }

    tft.fillRect(20, 50, 35, 35, (active != 1 && sys_axis_homed[X_AXIS] && sys_axis_homed[Y_AXIS] && (sys_axis_homed[Z_AXIS] || z_no_home)) ? active_color_one : TFT_DARKGREY); //
    tft.fillRect(56, 50, 35, 35, (active != 2 && sys_axis_homed[Y_AXIS] && (sys_axis_homed[Z_AXIS] || z_no_home)) ? TFT_OLIVE : TFT_DARKGREY);                           // y
    tft.fillRect(92, 50, 35, 35, (active != 3 && sys_axis_homed[X_AXIS] && sys_axis_homed[Y_AXIS] && (sys_axis_homed[Z_AXIS] || z_no_home)) ? active_color_one : TFT_DARKGREY); //

    tft.fillRect(20, 86, 35, 35, (active != 4 && sys_axis_homed[X_AXIS] && (sys_axis_homed[Z_AXIS] || z_no_home)) ? TFT_OLIVE : TFT_DARKGREY); // x
    
    if( probe_dimension == 0 ){
        tft.fillRect(56, 86, 35, 35, (active != 5 && ( sys_axis_homed[Z_AXIS] || z_no_home ) ) ? TFT_OLIVE : TFT_DARKGREY);                             // z
    } else if( probe_dimension == 1 ){
      // 2d center finder
      //tft.fillCircle(73, 103, 17, TFT_BLACK);
      //tft.fillCircle(73, 103, 10, TFT_WHITE);
      tft.fillRect(56, 86, 35, 35, (active != 5 ? TFT_OLIVE : TFT_DARKGREY ) ); // z
      tft.fillRect(59, 89, 29, 29, TFT_WHITE); // z
 

    }
    
    
    
    tft.fillRect(92, 86, 35, 35, (active != 6 && sys_axis_homed[X_AXIS] && (sys_axis_homed[Z_AXIS] || z_no_home)) ? TFT_OLIVE : TFT_DARKGREY); // x

    tft.fillRect(20, 122, 35, 35, (active != 7 && sys_axis_homed[X_AXIS] && sys_axis_homed[Y_AXIS] && (sys_axis_homed[Z_AXIS] || z_no_home)) ? active_color_one : TFT_DARKGREY); //
    tft.fillRect(56, 122, 35, 35, (active != 8 && sys_axis_homed[Y_AXIS] && (sys_axis_homed[Z_AXIS] || z_no_home)) ? TFT_OLIVE : TFT_DARKGREY);                           // y
    tft.fillRect(92, 122, 35, 35, (active != 9 && sys_axis_homed[X_AXIS] && sys_axis_homed[Y_AXIS] && (sys_axis_homed[Z_AXIS] || z_no_home)) ? active_color_one : TFT_DARKGREY); //

    tft.fillCircle(20, 50, 3, TFT_BLACK);
    tft.fillCircle(73, 50, 3, TFT_BLACK);
    tft.fillCircle(127, 50, 3, TFT_BLACK);

    tft.fillCircle(20, 103, 3, TFT_BLACK);
    tft.fillCircle(73, 103, 3, TFT_BLACK);
    tft.fillCircle(127, 103, 3, TFT_BLACK);

    tft.fillCircle(20, 156, 3, TFT_BLACK);
    tft.fillCircle(73, 156, 3, TFT_BLACK);
    tft.fillCircle(127, 156, 3, TFT_BLACK);
  }
  else
  {

    tft.setTextSize(1);
    tft.setTextColor(TFT_BLACK);
    if( motion_input_is_blocked() ){
        tft.drawString("Motion turned off!", 20, 50, 2);
    } else{
        tft.drawString("Voltage too low", 20, 50, 2);
    }
    tft.drawString("Probing disabled", 20, 70, 2);
  }
}

bool G_EDM_UI_CONTROLLER::machine_is_homed()
{

  if (
      sys_axis_homed[X_AXIS] && sys_axis_homed[Y_AXIS] && (sys_axis_homed[Z_AXIS] || z_no_home))
  {

    return true;
  }

  return false;
}

void G_EDM_UI_CONTROLLER::draw_motion_tab()
{

  tft.setTextSize(1);
  tft.fillRect(0, 39, 320, 206, TFT_BLACK);
  tft.fillRect(10, 40, 300, 135, TFT_LIGHTGREY); // content wrapper

  if (motion_tab_active == 1)
  {
    tft.fillRect(10, 10, 100, 30, TFT_LIGHTGREY); // tab homing
    tft.setTextColor(TFT_LIGHTGREY);
    tft.drawString("Probing", 130, 15, 2);
    tft.drawString("More", 230, 15, 2);
    tft.setTextColor(TFT_BLACK);
    tft.drawString("Homing", 30, 15, 2);
  }
  else if (motion_tab_active == 2)
  {
    tft.fillRect(110, 10, 100, 30, TFT_LIGHTGREY); // tab probing
    tft.setTextColor(TFT_LIGHTGREY);
    tft.drawString("Homing", 30, 15, 2);
    tft.drawString("More", 230, 15, 2);
    tft.setTextColor(TFT_BLACK);
    tft.drawString("Probing", 130, 15, 2);
  }
  else if (motion_tab_active == 3)
  {
    tft.fillRect(210, 10, 100, 30, TFT_LIGHTGREY); // tab tool
    tft.setTextColor(TFT_LIGHTGREY);
    tft.drawString("Homing", 30, 15, 2);
    tft.drawString("Probing", 130, 15, 2);
    tft.setTextColor(TFT_BLACK);
    tft.drawString("More", 230, 15, 2);
  }
}

String G_EDM_UI_CONTROLLER::get_mpos_string()
{
  float *mpos = system_get_mpos();
  String mpos_string = "";
  for (int axis = 0; axis < N_AXIS; ++axis)
  {
    switch (axis)
    {
    case X_AXIS:
      mpos_string += "X" + String(mpos[axis]) + " ";
      break;
    case Y_AXIS:
      mpos_string += "Y" + String(mpos[axis]) + " ";
      break;
    case Z_AXIS:
      mpos_string += "Z" + String(mpos[axis]) + " ";
      break;
    case A_AXIS:
      mpos_string += "A" + String(mpos[axis]) + " ";
      break;
    case B_AXIS:
      mpos_string += "B" + String(mpos[axis]) + " ";
      break;
    case C_AXIS:
      mpos_string += "C" + String(mpos[axis]) + " ";
      break;
    default:
      mpos_string += "?" + String(mpos[axis]) + " ";
      break;
    }
  }

  return mpos_string;
}

void G_EDM_UI_CONTROLLER::process_overlay_coords()
{
  tft.fillRect(29, 26, 291, 72, TFT_BLACK);
  float *mpos = api::get_wpos();
  tft.setTextColor(TFT_GREENYELLOW);
  tft.setTextSize(1);
  tft.drawString(String(mpos[0], 3), 30, 30, 2);
  tft.drawString(String(mpos[1], 3), 30, 50, 2);
  tft.drawString(String(mpos[2], 3), 30, 70, 2);
  mpos = system_get_mpos();
  tft.drawString(String(mpos[0], 3), 100, 30, 2);
  tft.drawString(String(mpos[1], 3), 100, 50, 2);
  tft.drawString(String(mpos[2], 3), 100, 70, 2);
}
void G_EDM_UI_CONTROLLER::process_overlay_wire_spindle_speed_buttons()
{
  if (operation_mode != 4)
  {
    /** this is gcode wire specific **/
    return;
  }
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE);
  tft.fillRoundRect(250, 100, 60, 60, 5, TFT_OLIVE);
  tft.drawString("Spindle", 255, 105, 2);
  tft.drawString(String( frequency_to_rpm(wire_spindle_speed),1 ), 255, 125, 2);
  tft.drawString("rpm", 255, 140, 2);

}
void G_EDM_UI_CONTROLLER::process_overlay_reprobe_button()
{
  if (operation_mode != 3)
  {
    /** this is gcode floating z specific **/
    return;
  }
  tft.setTextSize(1);
  tft.setTextColor(sys.gedm_insert_probe ? TFT_WHITE : TFT_BLACK);
  tft.fillRoundRect(250, 100, 60, 60, 5, sys.gedm_insert_probe ? TFT_MAROON : TFT_OLIVE);
  tft.drawString(sys.gedm_insert_probe ? "Reprobe" : "Reprobe", 255, 110, 2);
  tft.drawString(sys.gedm_insert_probe ? "Active" : "Insert", 255, 130, 2);
}
void G_EDM_UI_CONTROLLER::process_overlay_menu()
{
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("Frequency",      10, 110, 2);
  tft.drawString("Duty",           10, 135, 2);
  tft.drawString("Setpoint Min",   10, 160, 2);
  tft.drawString("Setpoint Max",   10, 185, 2);
  tft.drawString("Feedrate MAX", 10, 210, 2);
    
  tft.setTextColor(TFT_GREEN);
  tft.drawString(String(convert_frequency_units(get_freq())), 120, 110, 2);
  tft.drawString(String(get_duty_percent()) + "%",            120, 135, 2);
  tft.drawString(String(drop_min) + "%",                      120, 160, 2);
  tft.drawString(String(drop_max) + "%",                      120, 185, 2);
  tft.drawString(String(max_feeds[Z_AXIS],3) + "mm/min",      120, 210, 2);
}

void G_EDM_UI_CONTROLLER::process_overlay_pause_button()
{
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE);
  tft.fillRoundRect(250, 170, 60, 60, 5, sys.edm_pause_motion ? TFT_OLIVE : TFT_RED);
  tft.drawString(sys.edm_pause_motion ? "Paused" : "Pause", 255, 200, 2);
  tft.drawString("EDM", 255, 180, 2);
}

void G_EDM_UI_CONTROLLER::update_process_meta()
{
  tft.setTextSize(1);
  tft.setTextColor(TFT_GREENYELLOW);
  tft.drawString(String(get_feedback_voltage(), 2) + "v", 190, 30, 2);
  tft.drawString(String(planner.get_current_round()), 260, 30, 2);
  String plan = "";
  switch (motion_plan)
  {
      case 0:
      plan = "Feed@";
      break;
      case 1:
      plan = "Hold@";
      break;
      case 2:
      plan = "Backoff@";
      break;
      case 3:
      plan = "Undo@";
      break;
  }
  plan += String( int( planner.get_current_work_index()) );
  tft.drawString( "Plan: "+  plan, 190, 50, 2);

  tft.drawString( String( sys.gedm_retraction_motion ? "Back" : "Forward" ), 190, 70, 2);

}

void G_EDM_UI_CONTROLLER::render_page_process_overlay()
{
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE);
  process_overlay_pause_button();
  tft.setTextColor(TFT_LIGHTGREY);
  tft.drawString("WPos", 30, 10, 2);
  tft.drawString("MPos", 100, 10, 2);
  tft.drawString("X", 10, 30, 2);
  tft.drawString("Y", 10, 50, 2);
  tft.drawString("Z", 10, 70, 2);

  tft.drawString("vSense", 190, 10, 2);
  tft.drawString("Round", 260, 10, 2);

  process_overlay_coords();
  process_overlay_menu();
  process_overlay_reprobe_button();
  process_overlay_wire_spindle_speed_buttons();
  update_process_meta();
}

void G_EDM_UI_CONTROLLER::render_page_settings_motion()
{

  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  // tft.setTextColor( TFT_OLIVE );
  // tft.drawString( "Motion Settings", 10, 10, 2);
  // tft.setTextSize(1);

  draw_motion_tab();

  if (motion_tab_active == 1)
  {

    tft.setTextColor(TFT_DARKGREEN);
    tft.drawString("Home Z", 20, 50, 2);
    tft.drawString("Press to move to origin", 20, 75, 2);
    tft.setTextColor(TFT_MAROON);
    tft.drawString(String(z_no_home ? "NO" : "YES"), 100, 50, 2);
    draw_homing_buttons(true);
  }
  else if (motion_tab_active == 2)
  {
    draw_probing_buttons();
  }
  else if (motion_tab_active == 3)
  {
    tft.setTextColor(TFT_DARKGREEN);
    tft.drawString("Tool diameter", 20, 50, 2);
    tft.drawString("Spindle", 20, 75, 2);
    tft.drawString("Feedrate MAX", 20, 100, 2);
    tft.drawString("Stepdelay EDM", 20, 125, 2);
    tft.drawString("Stepdelay Rapids", 20, 150, 2);
    tft.setTextColor(TFT_MAROON);
    tft.drawString(String(tool_diameter) + "mm", 140, 50, 2);
    tft.drawString( spindle_is_running() ? "Stop" : "Start", 140, 75, 2);
    tft.drawString(String(max_feeds[Z_AXIS],3) + "mm/min", 140, 100, 2);
    tft.drawString(String(DEFAULT_STEP_DELAY_EDM) + "ms", 140, 125, 2);
    tft.drawString(String(DEFAULT_STEP_DELAY_RAPID) + "ms", 140, 150, 2);
  }

  tft.setTextSize(1);
  tft.setTextColor(TFT_BLACK);
  tft.fillRoundRect(10, 190, 300, 40, 5, TFT_OLIVE);
  tft.drawString("Done", 20, 200, 2);
}

void G_EDM_UI_CONTROLLER::render_page_settings_sd(String active_profile)
{
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(TFT_OLIVE);
  tft.drawString("SD Card", 10, 10, 2);
  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKGREEN);
  tft.drawString("Save current burn profile", 20, 40, 2);
  tft.drawString("Burn profile", 20, 70, 2);
  tft.drawString("gCode file", 20, 100, 2);

  tft.setTextColor(TFT_OLIVE);
  tft.setTextSize(1);
  tft.setTextColor(TFT_BLACK);
  tft.fillRoundRect(10, 190, 300, 40, 5, TFT_OLIVE);
  tft.drawString("Done", 20, 200, 2);
  tft.setTextColor(TFT_WHITE);
  tft.drawString(active_profile, 120, 70, 2);
  tft.drawString((gcode_file.length() > 0 ? gcode_file : "None"), 120, 100, 2);
  if (gcode_file.length() > 0)
  {

    tft.setTextColor(TFT_RED);
    tft.drawString("Unload gCode", 20, 140, 2);
  }
}
void G_EDM_UI_CONTROLLER::render_page_settings_sd_profiles()
{
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(TFT_OLIVE);
  tft.drawString("Burn profiles", 10, 10, 2);
}
void G_EDM_UI_CONTROLLER::render_page_settings_sd_gcode()
{
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(TFT_OLIVE);
  tft.drawString("gCode", 10, 10, 2);
}
void G_EDM_UI_CONTROLLER::render_page_settings_sd_add_entry(String entry, int row)
{
  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKGREEN);
  int margin_top = 40;
  int row_height = 30;
  int y = margin_top + (row_height * row);
  tft.drawString(entry, 20, y, 2);
}
void G_EDM_UI_CONTROLLER::render_page_settings_sd_redraw_canvas()
{
  tft.fillRect(0, 39, 320, 160, TFT_BLACK);
}
void G_EDM_UI_CONTROLLER::draw_navigation_buttons(int current_page, int total_pages)
{
  tft.fillRect(0, 194, 320, 46, TFT_BLACK);
  if (total_pages <= 1)
  {
    // return;
  }
  if (current_page > 1)
  {
    // highlight backbutton
    tft.fillRect(90, 200, 50, 35, TFT_OLIVE);
    tft.drawLine(100, 218, 125, 208, TFT_BLACK);
    tft.drawLine(100, 218, 125, 228, TFT_BLACK);
  }
  else
  {
    tft.fillRect(90, 200, 50, 35, TFT_MAROON);
    tft.drawLine(100, 218, 125, 208, TFT_BLACK);
    tft.drawLine(100, 218, 125, 228, TFT_BLACK);
  }
  if (current_page == total_pages)
  {
    // no more forward
    tft.fillRect(265, 200, 50, 35, TFT_MAROON);
    tft.drawLine(305, 218, 280, 208, TFT_BLACK);
    tft.drawLine(305, 218, 280, 228, TFT_BLACK);
  }
  else
  {
    tft.fillRect(265, 200, 50, 35, TFT_OLIVE);
    tft.drawLine(305, 218, 280, 208, TFT_BLACK);
    tft.drawLine(305, 218, 280, 228, TFT_BLACK);
  }
  tft.fillRect(5, 200, 80, 35, TFT_MAROON);
  tft.setTextColor(TFT_WHITE);
  tft.drawString(String(current_page) + " / " + String(total_pages), 185, 210, 2);
  tft.drawString("Close", 30, 208, 2);
}

void G_EDM_UI_CONTROLLER::select_list_begin(int num_files, char files[][100])
{

  int current_page = 1;
  int batch_size = 5;
  int current = 0;
  int from = 0;
  int to = batch_size;
  int i = 0;
  bool has_changed = true;
  bool finish = false;
  int total_pages = num_files / batch_size;
  bool block_touch = false;
  uint16_t ix, iy;

  if (float(num_files) / float(batch_size) != total_pages)
  {
    ++total_pages;
  }

  if (num_files <= 0)
  {

    render_page_settings_sd_add_entry("No files found. Touch to return.", 0);

    vTaskDelay(200);

    while (true)
    {

      if (get_touch(&ix, &iy))
      {

        break;
      }
    }
  }
  else
  {

    while (true)
    {

      if (has_changed)
      {
        /** delete current canvas ( fill black below title ) **/
        render_page_settings_sd_redraw_canvas();
      }

      /** add list items **/
      while (current < batch_size && i < num_files)
      {
        render_page_settings_sd_add_entry(String(files[i]), current);
        ++current;
        ++i;
        has_changed = true;
      }

      if (has_changed)
      {
        draw_navigation_buttons(current_page, total_pages);
        has_changed = false;
      }

      block_touch = false;

      while (true)
      {

        if (get_touch(&ix, &iy))
        {

          if (block_touch)
          {
            continue;
          }

          block_touch = true;

          if (ix >= 5 && ix <= 85 && iy >= 200 && iy <= 240)
          {

            // close
            finish = true;
            break;
          }
          else if (current_page > 1 && ix >= 90 && ix <= 140 && iy >= 200 && iy <= 240)
          {

            // previous page
            --current_page;
            i = i - (current + 1) - batch_size;
            current = 0;
            if (i < 0)
            {
              i = 0;
            }

            has_changed = true;
            vTaskDelay(50);
            break;
          }
          else if (total_pages > 1 && current_page < total_pages && ix >= 265 && ix <= 315 && iy >= 200 && iy <= 240)
          {

            // next page
            ++current_page;
            current = 0;
            has_changed = true;
            vTaskDelay(50);
            break;
          }
          else if (ix > 0 && ix < 320 && iy > 40 && iy < 200)
          {

            // touch within list canvas
            // row height is 30px and the first entry is 40px below the top.
            int selected_row = (iy - 40) / 30;
            float f_selected_row = (float(ix) - 40.0) / 30.0;

            if (selected_row != f_selected_row)
            {
              ++selected_row;
            }

            /** the current index "i" is the last entry on the current page **/
            int array_item = i - 1 - current + selected_row;
            selected_list_item = files[array_item];
            finish = true;
            break;
          }

          // end if xy
        }

        block_touch = false;
      }

      if (finish)
      {
        break;
      }
    }
  }
}

String G_EDM_UI_CONTROLLER::get_selected_item()
{
  return selected_list_item;
}
void G_EDM_UI_CONTROLLER::unselected_item()
{
  selected_list_item = "";
}


void G_EDM_UI_CONTROLLER::probe_done(){
  if( enable_spindle ){
    stop_spindle();
    set_speed(probe_backup_frequency);
  }  
  probe_mode_off();
}
bool G_EDM_UI_CONTROLLER::probe_prepare( int disable_index, bool is_3d ){

    draw_probing_buttons( disable_index );
    /*vTaskDelay(500);

    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(TFT_LIGHTGREY);
    tft.drawString("Probing" + String( is_3d ? "3D" : "2D" ),10,30,2);
    tft.setTextSize(1);

    if( is_3d ){
      tft.setTextColor(TFT_RED);
      tft.drawString("This probe contains Z motions",10,80,2);
      tft.drawString("and is not compatible with wire EDM!",10,100,2);
    }
    else {
      tft.setTextColor(TFT_GREENYELLOW);
      tft.drawString("This probe is safe to use",10,80,2);
      tft.drawString("with wire EDM!",10,100,2);
    }*/

    probe_mode_on();
    generate_reference_voltage();
    probe_backup_frequency = get_speed();
    if( enable_spindle ){
      set_speed(60);
      start_spindle();
    }

    return true;

}
void G_EDM_UI_CONTROLLER::draw_page(int page_num, bool redraw_full)
{

  if (page_num == 1)
  {
    /** Front page **/
    if (redraw_full)
    {
      draw_interface();
    }
    render_page_front();
  }
  else if (page_num == 2)
  {
    /** Front page **/
    if (redraw_full)
    {
      draw_interface();
    }
    render_page_front();
  }
  else if (page_num == 3)
  {
    /** PWM settings page **/
    render_page_settings_pwm();
  }
  else if (page_num == 4)
  {
    /** Flushing settings page **/
    render_page_settings_flushing();
  }
  else if (page_num == 5)
  {
    /** Spark settings page **/
    render_page_settings_spark();
  }
  else if (page_num == 6)
  {
    /** Mode settings page **/
    render_page_settings_mode();
  }
  else if (page_num == 7)
  {
    /** Motion settings page **/
    // set_motion_tab_active( 1 );
    render_page_settings_motion();
  }
  else if (page_num == 8)
  {
    /** Motion settings page **/
    render_page_settings_sd(active_profile);
  }
  else if (page_num == 9)
  {
    render_page_process_overlay();
  }
}

void G_EDM_UI_CONTROLLER::set_motion_tab_active(int active_tab)
{

  motion_tab_active = active_tab;
}

/**
 * Monitoring touch events
 * and map them to the callback functions
 **/
void G_EDM_UI_CONTROLLER::monitor_touch_input()
{

  uint16_t x, y;
  bool redraw = false;
  bool block_touch = false;

  if (get_touch(&x, &y))
  {

    if (block_touch)
    {
      /** debounce **/
      return;
    }

    block_touch = true;

    if (active_page == 1)
    {

      /** frontpage **/
      if (x >= 10 && x <= 80 && y >= 10 && y <= 45)
      {

        /** show pwm settings page **/
        render_page(3, true);
      }
      else if (x >= 10 && x <= 80 && y >= 49 && y <= 81)
      {

        /** show flushing settings page **/
        render_page(4, true);
      }
      else if (x >= 10 && x <= 80 && y >= 88 && y <= 123)
      {

        /** show spark settings page **/
        render_page(5, true);
      }
      else if (x >= 10 && x <= 80 && y >= 127 && y <= 162)
      {

        /** show mode settings page **/
        render_page(6, true);
      }
      else if (x >= 10 && x <= 85 && y >= 166 && y <= 201)
      {

        /** show motion settings page **/
        render_page(7, true);
      }
      else if (x >= 10 && x <= 85 && y >= 205 && y <= 240)
      {

        if (!filehandler->is_locked_for_ui())
        {

          filehandler->sd_card_refresh();

          if (filehandler->get_has_sd_card())
          {

            // show save/load/restore settings page
            render_page(8, true);
          }
          else
          {

            // render_page( 1, false );
          }
        }
      }
      else if (x >= 240 && y >= 160)
      {
        /** PWM on/off button **/
        /** disable touch deactivation if edm process is running **/
        if (!edm_process_is_running)
        {
          if (get_source_voltage() < source_voltage_min)
          {
            disable_spark_generator();
          }
          else
          {
            toggle_pwm_on_off();
          }
          redraw = true;
        }
      }
      else if (x >= 210 && x <= 230 && y >= 180 && y <= 200)
      {
        /** move Z up **/
        if (!motion_input_is_blocked())
        {
          open_keyboard(0.0, "Jog Z up (mm)", "Jogging Z up\r" + get_mpos_string());
          api::jog_up(get_keyboard_result());
          redraw = true;
        }
      }
      else if (x >= 210 && x <= 230 && y >= 210 && y <= 230)
      {
        /** move Z down **/
        if (!motion_input_is_blocked())
        {
          open_keyboard(0.0, "Jog Z down (mm)", "Jogging Z down\r" + get_mpos_string());
          api::jog_down(get_keyboard_result());
          redraw = true;
        }
      }
      else if (x >= 140 && x <= 160 && y >= 180 && y <= 200)
      {
        /** move Y up **/
        if (!motion_input_is_blocked())
        {
          open_keyboard(0.0, "Jog Y up (mm)", "Jogging Y up\r" + get_mpos_string());
          api::jog_back(get_keyboard_result());
          redraw = true;
        }
      }
      else if (x >= 140 && x <= 160 && y >= 210 && y <= 230)
      {
        /** move X down **/
        if (!motion_input_is_blocked())
        {
          open_keyboard(0.0, "Jog Y down (mm)", "Jogging Y down\r" + get_mpos_string());
          api::jog_forward(get_keyboard_result());
          redraw = true;
        }
      }
      else if (x >= 110 && x <= 130 && y >= 195 && y <= 215)
      {
        /** move X left **/
        if (!motion_input_is_blocked())
        {
          open_keyboard(0.0, "Jog X left (mm)", "Jogging X left\r" + get_mpos_string());
          api::jog_left(get_keyboard_result());
          redraw = true;
        }
      }
      else if (x >= 170 && x <= 190 && y >= 195 && y <= 215)
      {
        /** move X down **/
        if (!motion_input_is_blocked())
        {
          open_keyboard(0.0, "Jog X right (mm)", "Jogging X right\r" + get_mpos_string());
          api::jog_right(get_keyboard_result());
          redraw = true;
        }
      }

      else if (x >= 110 && x <= 155 && y >= 111 && y <= 141)
      {
        /** drill mode **/
        set_operation_mode(1);
        redraw = true;
      }
      else if (x >= 160 && x <= 205 && y >= 111 && y <= 141)
      {
        /** reamer mode **/
        set_operation_mode(2);
        redraw = true;
      } 
      else if (x >= 210 && x <= 255 && y >= 111 && y <= 141)
      {
        if( filehandler->get_has_sd_card() ){
            /** 3d float mode **/
            set_operation_mode(3);
            redraw = true;
        }
      } 
      else if (x >= 260 && x <= 305 && y >= 111 && y <= 141)
      {

        if( EXTENDED_FEATURES && filehandler->get_has_sd_card() ){
            /** 2d wire mode **/
            set_operation_mode(4);
            redraw = true;
        }


      } 

      if (redraw)
      {

        render_page(1, true);
      }
    }
    else if (active_page == 2)
    {

      /** benchmark page is touched. Back to the frontpage **/
      render_page(1, true);
    }
    else if (active_page == 3)
    {

      /** PWM settings page **/
      if (x >= 10 && x <= 310 && y >= 190 && y <= 230)
      {

        /** done/back **/
        render_page(1, true);
      }
      else if (x >= 0 && x <= 320 && y >= 30 && y <= 55)
      {

        /** change PWM frequency **/
        edit_pwm_frequency();
        redraw = true;
      }
      else if (x >= 0 && x <= 160 && y >= 60 && y <= 85)
      {

        /** change PWM duty **/
        edit_pwm_duty();
        redraw = true;
      }

      if (redraw)
      {
        render_page(active_page, true);
      }
    }
    else if (active_page == 4)
    {

      /** Flushing settings page **/
      if (x >= 10 && x <= 310 && y >= 190 && y <= 230)
      {
        /** done/back **/
        render_page(1, true);
      }
      else if (x >= 0 && x <= 320 && y >= 30 && y <= 55)
      {

        /** change flushing movement interval in ms **/
        open_keyboard(flush_retract_after, "F_int (ms) (1000ms=1s)", "Up/Down flushingmovement interval in ms");
        flush_retract_after = get_keyboard_result();

        redraw = true;
      }
      else if (x >= 0 && x <= 320 && y >= 60 && y <= 85)
      {

        /** change flushing movement distance in mm **/
        open_keyboard(flush_retract_mm, "F_dis (mm)", "Tool moves given mm up while flushing");
        flush_retract_mm = get_keyboard_result();

        redraw = true;
      }
      else if (x >= 0 && x <= 320 && y >= 90 && y <= 115)
      {

        disable_spark_for_flushing = !disable_spark_for_flushing;
        redraw = true;
      }
      else if (x >= 0 && x <= 320 && y >= 120 && y <= 145)
      {
        /** change flushing movement offset steps **/
        open_keyboard(flush_offset_steps, "Offset steps (steps)", "Axis moves back to flushing start position \rminus given offset steps");
        int steps = int(round(get_keyboard_result()));

        flush_offset_steps = steps;

        redraw = true;
      }

      if (redraw)
      {
        render_page(active_page, true);
      }
    }
    else if (active_page == 5)
    {

      /**Spark settings page **/
      if (x >= 10 && x <= 310 && y >= 190 && y <= 230)
      {

        /** done/back **/
        render_page(1, true);
      }
      else if (x >= 0 && x <= 320 && y >= 30 && y <= 55)
      {
        /** changing setpoint range min **/
        edit_setpoint_min();
        redraw = true;
      }
      else if (x >= 0 && x <= 320 && y >= 60 && y <= 85)
      {
        /** changing setpoint range max **/
        edit_setpoint_max();
        redraw = true;
      }
      else if (x >= 0 && x <= 320 && y >= 90 && y <= 115)
      {
        /** changing the minimum inputvoltage. Makes debugging sometimes easier if the process starts even at 0v. **/
        open_keyboard(source_voltage_min, "Minimum Inputvoltage (v)", "If voltage supplied from PSU is below this \rvalue the process won't start");
        float new_voltage_min = get_keyboard_result();

        source_voltage_min = new_voltage_min;

        redraw = true;
      }

      if (redraw)
      {
        render_page(active_page, true);
      }
    }
    else if (active_page == 6)
    {

      /** Mode settings page **/
      if (x >= 10 && x <= 310 && y >= 190 && y <= 230)
      {

        /** done/back **/
        render_page(1, true);
      }
      else if (x >= 0 && x <= 320 && y >= 30 && y <= 55)
      {

        /** changing the operation mode **/
        ++operation_mode;

        if ( operation_mode > 4 
             || ( operation_mode > 2 && ! filehandler->get_has_sd_card() )
             || ( operation_mode > 3 && !EXTENDED_FEATURES ) 
           )
        {
          set_operation_mode(1); // jump back to first
        }
        set_operation_mode( operation_mode );

        redraw = true;
      }
      else if (x >= 0 && x <= 320 && y >= 70 && y <= 95)
      {

        if (operation_mode == 1 || operation_mode == 3)
        {

          /** change Z travel limit; 0 means no limits **/
          open_keyboard(z_stop_after_mm, "Z_STOP (mm)", "Tool travels given mm counting from the \rfirst contact");
          set_z_stop(get_keyboard_result());

          redraw = true;
        }
        else if (operation_mode == 2)
        {

          open_keyboard(reamer_travel_mm, "REAMER_DIS (mm)", "Tool oscillates given mm up and down");
          reamer_travel_mm = get_keyboard_result();

          redraw = true;
        }
        else if (operation_mode == 4)
        {
          change_wire_spindle_speed();
          redraw = true;
        }
      }
      else if (x >= 0 && x <= 320 && y >= 100 && y <= 125)
      {

        if (operation_mode == 2)
        {

          open_keyboard(reamer_duration, "Reamer duration (s)", "Total duration of the reaming process in \rseconds");
          reamer_duration = get_keyboard_result();

          redraw = true;
        }
        else if ( operation_mode == 3 || operation_mode == 4 )
        {
          simulate_gcode = !simulate_gcode;
          redraw = true;
        }
      }
      else if (x >= 0 && x <= 320 && y >= 130 && y <= 155)
      {

        if (operation_mode <= 4)
        {
          enable_spindle = !enable_spindle;
          redraw = true;
        }
      }

      if (redraw)
      {
        render_page(active_page, true);
      }
    }
    else if (active_page == 7)
    {

      /** Motion settings page **/
      if (x >= 10 && x <= 300 && y >= 190 && y <= 230)
      {

        /** done/back **/
        render_page(1, true);
      }
      else if (x >= 10 && x <= 100 && y >= 10 && y <= 40)
      {

        // homing tab
        if (motion_tab_active != 1)
        {
          set_motion_tab_active(1);
          redraw = true;
        }
      }
      else if (x >= 100 && x <= 200 && y >= 10 && y <= 40)
      {

        // probing tab
        if (motion_tab_active != 2)
        {
          set_motion_tab_active(2);
          redraw = true;
        }
      }
      else if (x >= 200 && x <= 300 && y >= 10 && y <= 40)
      {

        // probing tab
        if (motion_tab_active != 3)
        {
          set_motion_tab_active(3);
          redraw = true;
        }
      }

      if (motion_tab_active == 1)
      {

        /** homing tab octive**/
        if (x >= 0 && x <= 320 && y >= 40 && y <= 65)
        {

          /** toggle Z axis no home **/
          z_no_home = !z_no_home;
          redraw = true;
        }

        else if(x >= 0 && x <= 320 && y >= 65 && y <= 90 && !motion_input_is_blocked()){
          // move to 0,0 WPos
          api::push_cmd("G90 G0 X0 Y0\r\n");
          redraw = true;
        }

        else if (x >= 20 && x <= 80 && y >= 130 && y <= 170 && !motion_input_is_blocked() )
        {

          for (int i = 0; i < N_AXIS; ++i)
          {
            sys_axis_homed[i] = false;
          }

          draw_homing_buttons(false);
          api::home_z();
          api::home_x();
          api::home_y();
          redraw = true;
        }

        else if (x >= 90 && x <= 150 && y >= 130 && y <= 170 && !motion_input_is_blocked())
        {

          draw_homing_buttons(false);
          api::home_x();
          redraw = true;
        }
        else if (x >= 160 && x <= 220 && y >= 130 && y <= 170 && !motion_input_is_blocked())
        {

          draw_homing_buttons(false);
          api::home_y();
          redraw = true;
        }
        else if (x >= 230 && x <= 290 && y >= 130 && y <= 170 && !motion_input_is_blocked())
        {

          draw_homing_buttons(false);
          api::home_z();
          redraw = true;
        }
      }
      else if (motion_tab_active == 2)
      {

        /** probing tab active **/
        if (x >= 160 && x <= 320 && y >= 40 && y <= 65)
        {

          // change probing duty
          open_keyboard(pwm_duty_probing, "Probing PWM duty (%)", "Duty cycle used for probing");
          pwm_duty_probing = get_keyboard_result();
          if (pwm_duty_probing > PWM_DUTY_MAX)
          {
            pwm_duty_probing = PWM_DUTY_MAX;
          }
          redraw = true;
        }
        else if (x >= 160 && x <= 320 && y >= 65 && y <= 90)
        {

          // change probing frequency
          open_keyboard(pwm_frequency_probing, "Probing PWM frequency (hz)", "PWM frequency used for probing");
          pwm_frequency_probing = int(get_keyboard_result());
          if (pwm_frequency_probing < PWM_FREQUENCY_MIN)
          {
            pwm_frequency_probing = PWM_FREQUENCY_MIN;
          }
          if (pwm_frequency_probing > PWM_FREQUENCY_MAX)
          {
            pwm_frequency_probing = PWM_FREQUENCY_MAX;
          }
          redraw = true;
        }
        else if (x >= 160 && x <= 320 && y >= 90 && y <= 115)
        {
          /** changing the minimum inputvoltage. Makes debugging sometimes easier if the process starts even at 0v. **/
          open_keyboard(vSense_drop_range_noload, "Voltage drop for probe (%)", "A voltagedrop above this value is considered a contact");
          float new_drop_noload = get_keyboard_result();

          if (new_drop_noload <= 5.0)
          {
            new_drop_noload = 10.0;
          }
          else if (new_drop_noload >= 50.0)
          {
            new_drop_noload = 50.0;
          }

          vSense_drop_range_noload = new_drop_noload;

          redraw = true;
        }
        else if (x >= 200 && x <= 320 && y >= 125 && y <= 170)
        {

          set_reprobe_point();
          draw_set_reprobe_button();
        }
        else if (x >= 155 && x <= 192 && y >= 125 && y <= 170)
        {
          if( ++probe_dimension > 1 ){
            probe_dimension = 0;
          }
          if( operation_mode == 4 ){
            // in 2D wire mode no 3D probe is allowed as it would crash the wire extension
            probe_dimension = 1; // force 2D probe 
          }
          draw_probing_buttons();
          //draw_toggle_probe_dimension();
        }
        else if (get_source_voltage() >= source_voltage_min && !motion_input_is_blocked())
        {

          if (x >= 56 && x <= 91 && y >= 86 && y <= 121 && (sys_axis_homed[Z_AXIS] || z_no_home))
          {
            probe_prepare(5,probe_dimension==1?false:true);
            /** probing only z **/
            if( probe_dimension == 0 ){
              probe_z(-1.0);
            } else if( probe_dimension == 1 ){
              center_finder_2d();
            }
            probe_done();
            redraw = true;
          }
          else if (x >= 56 && x <= 91 && y >= 50 && y <= 85 && (sys_axis_homed[Y_AXIS] && (sys_axis_homed[Z_AXIS] || z_no_home)))
          {
            probe_prepare(2);
            probe_y(-1.0, true);
            probe_done();
            redraw = true;
          }
          else if (x >= 56 && x <= 91 && y >= 122 && y <= 157 && (sys_axis_homed[Y_AXIS] && (sys_axis_homed[Z_AXIS] || z_no_home)))
          {
            probe_prepare(8);
            probe_y(1.0, true);
            probe_done();
            redraw = true;
          }
          else if (x >= 92 && x <= 127 && y >= 86 && y <= 121 && (sys_axis_homed[X_AXIS] && (sys_axis_homed[Z_AXIS] || z_no_home)))
          {
            probe_prepare(6);
            probe_x(-1.0, true);
            probe_done();
            redraw = true;
          }
          else if (x >= 20 && x <= 55 && y >= 86 && y <= 121 && (sys_axis_homed[X_AXIS] && (sys_axis_homed[Z_AXIS] || z_no_home)))
          {
            probe_prepare(4);
            probe_x(1.0, true);
            probe_done();
            redraw = true;
          }
          else if (x >= 92 && x <= 127 && y >= 50 && y <= 85 && (sys_axis_homed[X_AXIS] && sys_axis_homed[Y_AXIS] && (sys_axis_homed[Z_AXIS] || z_no_home)))
          {
            probe_prepare(3,probe_dimension==1?false:true);
            if( probe_dimension == 0 ){
                // 3D probing
                right_back_edge_3d();
            } else if( probe_dimension == 1 ){
                // 2D probing: doesn't involve any Z motions
                right_back_edge_2d();
            }
            probe_done();
            redraw = true;
          }
          else if (x >= 92 && x <= 127 && y >= 122 && y <= 157 && (sys_axis_homed[X_AXIS] && sys_axis_homed[Y_AXIS] && (sys_axis_homed[Z_AXIS] || z_no_home)))
          {
            probe_prepare(9,probe_dimension==1?false:true);
            if( probe_dimension == 0 ){
              // 3D probing
              right_front_edge_3d();
            } else if( probe_dimension == 1 ){
              // 2D probing: doesn't involve any Z motions
              right_front_edge_2d();
            }
            probe_done();
            redraw = true;
          }
          else if (x >= 20 && x <= 55 && y >= 50 && y <= 85 && (sys_axis_homed[X_AXIS] && sys_axis_homed[Y_AXIS] && (sys_axis_homed[Z_AXIS] || z_no_home)))
          {
            probe_prepare(1,probe_dimension==1?false:true);
            if( probe_dimension == 0 ){
                // 3D probing
                left_back_edge_3d();
            } else if( probe_dimension == 1 ){
              // 2D probing: doesn't involve any Z motions
              left_back_edge_2d();
            }

            probe_done();
            redraw = true;
          }
          else if (x >= 20 && x <= 55 && y >= 122 && y <= 157 && (sys_axis_homed[X_AXIS] && sys_axis_homed[Y_AXIS] && (sys_axis_homed[Z_AXIS] || z_no_home)))
          {
            probe_prepare(7,probe_dimension==1?false:true);
            if( probe_dimension == 0 ){
                // 3D probing: First probe Z then X and Y
                left_front_edge_3d();
            } else if( probe_dimension == 1 ){
              // 2D probing: doesn't involve any Z motions
              left_front_edge_2d();
            }
            probe_done();
            redraw = true;
          }
        }
      }
      else if (motion_tab_active == 3)
      {

        if (x >= 0 && x <= 320 && y >= 40 && y <= 65)
        {
          open_keyboard(tool_diameter, "Tool diameter (mm)", "Diameter of the tool used");
          tool_diameter = get_keyboard_result();
          redraw = true;
        }
        else if (x >= 0 && x <= 320 && y >= 65 && y <= 90)
        {
          if (spindle_is_running())
          {
            stop_spindle();
          }
          else
          {
            start_spindle();
          }
          while (get_touch(&x, &y))
          {
            vTaskDelay(10);
          }
          redraw = true;
        }
        else if(x >= 0 && x <= 320 && y >= 90 && y <= 115)
        {
          edit_max_feed( Z_AXIS );
          redraw = true;
        }
        else if(x >= 0 && x <= 320 && y >= 110 && y <= 140)
        {
          edit_max_feed_xy();
          redraw = true;
        }
        else if(x >= 0 && x <= 320 && y >= 140 && y <= 175)
        {
          edit_rapid_delay();
          redraw = true;
        }
      }

      if (redraw)
      {
        render_page(active_page, true);
      }
    }
    else if (active_page == 8)
    {

      if (!filehandler->get_has_sd_card())
      {
        render_page(1, true);
        return;
      }

      /** settings manager page **/
      if (x >= 10 && x <= 310 && y >= 190 && y <= 230)
      {

        /** done/back **/
        render_page(1, true);
      }
      else if (x >= 0 && x <= 320 && y >= 30 && y <= 55)
      {

        String file_name = "burn-profile";

        /** save current settings to sd card file **/
        open_keyboard_alpha(file_name, "Enter filename or press OK for autonaming", "");
        file_name = get_keyboard_result_alpha();

        if (file_name == "" || file_name.length() <= 0)
        {
        }
        else
        {

          String folder = filehandler->get_folder_settings();

          if (SD.exists(folder + "/" + file_name + ".pro"))
          {
            int i = 0;

            while (filehandler->get_file_exists(folder + "/" + file_name + "-" + String(i) + ".pro"))
            {
              ++i;
            }

            file_name = file_name + "-" + String(i);
          }

          save_settings(file_name);
          active_profile = file_name;
        }

        redraw = true;
      }
      else if (x >= 0 && x <= 320 && y >= 60 && y <= 85)
      {

        /** load settings from sd card file **/
        render_page_settings_sd_profiles();

        /** count the profile files (subfolders are ignored!) **/
        int num_files = filehandler->count_files_in_folder_by_extension(filehandler->get_folder_settings(), ".pro");

        char files[num_files][100];

        // load the filenames
        filehandler->get_files_in_folder_by_extension(filehandler->get_folder_settings(), ".pro", files);

        select_list_begin(num_files, files);

        String item = get_selected_item();

        if (item.length() > 0)
        {
          load_settings(item);
        }

        unselected_item();

        filehandler->close_current_folder();

        redraw = true;
      }
      else if (x >= 0 && x <= 320 && y >= 90 && y <= 115)
      {

        /** load settings from sd card file **/
        render_page_settings_sd_gcode();

        /** count the profile files (subfolders are ignored!) **/
        int num_files = filehandler->count_files_in_folder_by_extension(filehandler->get_folder_gcode(), ".gcode");

        char files[num_files][100];

        // load the filenames
        filehandler->get_files_in_folder_by_extension(filehandler->get_folder_gcode(), ".gcode", files);

        select_list_begin(num_files, files);

        String item = get_selected_item();

        if (item.length() > 0)
        {
          set_gcode_file(item);
        }

        unselected_item();

        filehandler->close_current_folder();

        redraw = true;
      }
      else if (x >= 0 && x <= 320 && y >= 130 && y <= 155)
      {

        if (get_gcode_file().length() > 0)
        {

          set_gcode_file("");
        }

        redraw = true;
      }

      if (redraw)
      {
        render_page(active_page, true);
      }
    }
  }
  else
  {

    block_touch = false;
  }
}

String *G_EDM_UI_CONTROLLER::split(String &v, char delimiter, int &length)
{
  length = 1;
  bool found = false;
  for (int i = 0; i < v.length(); i++)
  {
    if (v[i] == delimiter)
    {
      length++;
      found = true;
    }
  }
  if (found)
  {
    String *valores = new String[length];
    int i = 0;
    for (int itemIndex = 0; itemIndex < length; itemIndex++)
    {
      for (; i < v.length(); i++)
      {

        if (v[i] == delimiter)
        {
          i++;
          break;
        }
        valores[itemIndex] += v[i];
      }
    }
    return valores;
  }
  return nullptr;
}
void G_EDM_UI_CONTROLLER::load_settings(String file_name)
{

  String file_path = filehandler->get_folder_settings() + "/" + file_name;

  if (file_path.indexOf(".pro") == -1 && file_path.indexOf(".PRO") == -1)
  {
    file_path += ".PRO";
  }
  String settings = filehandler->get_file_contents(file_path);
  if (settings.length() <= 0)
  {
    return;
  }
  int qtde;
  String *t = split(settings, ';', qtde);
  String setting;
  String value;
  for (int i = 0; i < qtde; i++)
  {
    setting = t[i].substring(0, t[i].indexOf(":"));
    value = t[i].substring(t[i].indexOf(":") + 1, t[i].length());
    update_setting(setting, value);
  }
  delete[] t;
  active_profile = file_name;
}

/**
 * All this String stuff is ugly
 * May change later to use NVS or Spiffs since SD card access is slow
 **/
void G_EDM_UI_CONTROLLER::save_settings(String file_name)
{
  String settings = "";
  settings += "1:" + String(get_freq()) + ";";
  settings += "2:" + String(get_duty_percent()) + ";";
  settings += "3:" + String(flush_retract_after) + ";";
  settings += "4:" + String(flush_retract_mm) + ";";
  settings += "5:" + String(disable_spark_for_flushing ? "true" : "false") + ";";
  settings += "6:" + String(flush_offset_steps) + ";";
  settings += "7:" + String(vsense_drop_range_min) + ";";
  settings += "8:" + String(vsense_drop_range_max) + ";";
  settings += "9:" + String(source_voltage_min) + ";";
  settings += "10:" + String(operation_mode) + ";";
  settings += "11:" + String(get_z_stop()) + ";";
  settings += "12:" + String(reamer_travel_mm) + ";";
  settings += "13:" + String(reamer_duration) + ";";
  settings += "14:" + String(z_no_home ? "true" : "false") + ";";
  settings += "15:" + String(vSense_drop_range_noload) + ";";
  settings += "16:" + String(enable_spindle ? "true" : "false") + ";";
  settings += "17:" + String(tool_diameter) + ";";
  settings += "18:" + String(simulate_gcode ? "true" : "false") + ";";
  settings += "19:" + String(pwm_frequency_probing) + ";";
  settings += "20:" + String(pwm_duty_probing) + ";";
  settings += "21:" + String(max_feeds[Z_AXIS]) + ";";
  settings += "22:" + String(wire_spindle_speed) + ";";

  String file_path = filehandler->get_folder_settings() + "/" + file_name;

  if (file_path.indexOf(".pro") == -1 && file_path.indexOf(".PRO") == -1)
  {
    file_path += ".PRO";
  }
  if (last_settings_copy == "")
  {
    last_settings_copy = settings;
  }
  if (file_name == "last_settings" && last_settings_copy == settings)
  {
    return;
  }
  last_settings_copy = settings;
  draw_sdcard_button( 1 );
  bool success = filehandler->write_file_contents(file_path, settings);
  if( ! success ){
    //last_settings_copy = "failed";
  }
  render_page_settings_menu_sd_card(filehandler->get_has_sd_card(), true);
}

void G_EDM_UI_CONTROLLER::update_setting(String setting, String value)
{
  if (setting == "1")
  {
    change_pwm_frequency(value.toInt());
  }
  else if (setting == "2")
  {
    update_duty(value.toFloat());
  }
  else if (setting == "3")
  {
    flush_retract_after = value.toFloat();
  }
  else if (setting == "4")
  {
    flush_retract_mm = value.toFloat();
  }
  else if (setting == "5")
  {
    disable_spark_for_flushing = value == "true" ? true : false;
  }
  else if (setting == "6")
  {
    flush_offset_steps = value.toInt();
  }
  else if (setting == "7")
  {
    vsense_drop_range_min = value.toFloat();
  }
  else if (setting == "8")
  {
    vsense_drop_range_max = value.toFloat();
  }
  else if (setting == "9")
  {
    source_voltage_min = value.toFloat();
  }
  else if (setting == "10")
  {
    set_operation_mode( value.toInt() );
  }
  else if (setting == "11")
  {
    set_z_stop(value.toFloat());
  }
  else if (setting == "12")
  {
    reamer_travel_mm = value.toFloat();
  }
  else if (setting == "13")
  {
    reamer_duration = value.toFloat();
  }
  else if (setting == "14")
  {
    z_no_home = value == "true" ? true : false;
    //(z_no_home);
  }
  else if (setting == "15")
  {
    vSense_drop_range_noload = value.toFloat();
  }
  else if (setting == "16")
  {
    enable_spindle = value == "true" ? true : false;
  }
  else if (setting == "17")
  {
    tool_diameter = value.toFloat();
  }
  else if (setting == "18")
  {
    // simulate_gcode = value == "true" ? true : false;
  }
  else if (setting == "19")
  {
    pwm_frequency_probing = value.toFloat();
  }
  else if (setting == "20")
  {
    pwm_duty_probing = value.toFloat();
  }
  else if (setting == "21")
  {
    max_feeds[Z_AXIS] = value.toFloat();
  }
  else if(setting == "22")
  {
    wire_spindle_speed = value.toInt();
  }
  // force_redraw = true;
}

