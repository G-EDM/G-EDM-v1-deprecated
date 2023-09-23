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


#include "sd_card/filehandler.h"

SDState sd_state          = SDState::Idle;
bool SD_ready_next        = false;
bool filehandler_is_ready = false;

G_FILEHANDLER filehandler;

G_FILEHANDLER::G_FILEHANDLER()
{
  filehandler_is_ready = false;
  locked_for_ui        = false;
  job_finished         = false;
}

void G_FILEHANDLER::filehandler_initialize(bool enable)
{
  if( is_locked_for_ui() ){ return; }
  sd_state             = SDState::NotPresent;
  filehandler_is_ready = false;
  ignore_sd            = enable ? false : true;
  folder_root          = String(ROOT_FOLDER);
  folder_settings      = folder_root + String(SETTINGS_FOLDER);
  folder_gcode         = folder_root + String(GCODE_FOLDER);
  line_number          = 0;
  error_code           = 0;
  has_sd_card          = false;
  if (ignore_sd)
  {
    filehandler_is_ready = true;
    sd_state = SDState::Idle;
    return;
  }

  if (!SD.begin(SS, file_spi))
  {
    error_code           = 1;
    filehandler_is_ready = true;
    sd_state = SDState::Idle;
    return;
  } 

  //if ( SD.cardSize() <= 0){
    //SD.end();
    //filehandler_initialize(true);
  //}

  create_directory_tree( 3 );

  has_sd_card = true;
  filehandler_is_ready = true;
  sd_state = SDState::Idle;

}

void G_FILEHANDLER::set_spi_instance( SPIClass &_spi ){
  file_spi = _spi;
}
/**
 *
 * Check if the directory treee exist on the SD card and create it if needed
 *
 **/
void G_FILEHANDLER::create_directory_tree( int rounds )
{

  if( ! has_sd_card || ignore_sd ){
    return;
  }

  /** check if directory tree exists and create it if needed **/
  if (!SD.exists(folder_root))
  {
    SD.mkdir(folder_root);
  }
  if (!SD.exists(folder_settings))
  {
    SD.mkdir(folder_settings);
  }
  if (!SD.exists(folder_gcode))
  {
    SD.mkdir(folder_gcode);
  }


  /*if (!SD.exists(folder_root))
  {
  } else{
  }*/

};


/** Store settings **/
bool G_FILEHANDLER::write_file_contents(String file_path, String _data)
{
  if( ignore_sd || is_locked_for_ui() ){ 
    return false; 
  }
  // if no root folder exists or is not found reinitialize the sd card
  if (!SD.exists(folder_root) || SD.cardSize() <= 0)
  {
    SD.end();
    filehandler_initialize(true);
  }
  sd_state = SDState::BusyCustom;
  if (SD.exists(file_path))
  {
    SD.remove(file_path);
  }
  int rounds   = 2;
  bool success = false;
  while( ! success && --rounds >= 0 ){
      current_file = SD.open(file_path, FILE_WRITE);
      if (current_file)
      {
        current_file.print(_data);
        current_file.flush();
      } 
      if( current_file.size() <= 0 || ! current_file.available() ){
          current_file.close();
          SD.end();
          filehandler_initialize(true);
      } else{
        success = true;
        current_file.close();
    }
  }
  sd_state = SDState::Idle;
  return success;
};


bool G_FILEHANDLER::get_file_exists(String full_path)
{
  return SD.exists(full_path);
}



/** Load settings from file **/
String G_FILEHANDLER::get_file_contents(String file_path)
{
  if (ignore_sd || is_locked_for_ui() )
  {
    return "";
  }
  Serial.print(file_path);

  if (!SD.exists(folder_root) || SD.cardSize() <= 0)
  {
    SD.end();
    filehandler_initialize(true);
  }

  if (!SD.exists(file_path))
  {
    /** no sd card or file missing **/
    return "";
  }
  sd_state = SDState::BusyCustom;
  current_file = SD.open(file_path, FILE_READ);
  String data = "";
  if (current_file)
  {
    while (current_file.available())
    {
      data += String((char)current_file.read());
    }
    current_file.close();
  }
  sd_state = SDState::Idle;
  return data;
};









/**
 * 
 * These two functions are exclusive for the gcode reading 
 * 
 **/
boolean G_FILEHANDLER::openFile(const char *path)
{
  current_file = SD.open(path, FILE_READ);
  if (!current_file)
  {
    return false;
  }
  job_finished           = false;
  sd_state = SDState::BusyPrinting;
  SD_ready_next          = false; 
  sd_current_line_number = 0;
  return true;
}
boolean G_FILEHANDLER::closeFile()
{
  if (!current_file)
  {
    //force_redraw           = true;
    SD_ready_next          = false;
    job_finished           = true;
    sd_current_line_number = 0;
    sd_state = SDState::Idle;
    return false;
  }
  sd_state = SDState::Idle;
  SD_ready_next          = false;
  sd_current_line_number = 0;
  current_file.close();
  job_finished = true;
  return true;
}










void G_FILEHANDLER::reset_job_finished()
{
  job_finished = false;
}
bool G_FILEHANDLER::has_job_finished()
{
  return job_finished;
}

bool G_FILEHANDLER::is_locked_for_ui()
{
  return sd_state == SDState::BusyPrinting ? true : false;
}
bool G_FILEHANDLER::get_is_busy(){
  return sd_state != SDState::Idle ? true : false;
}


boolean G_FILEHANDLER::readFileLine(char *line, int maxlen){
  if (!current_file)
  {
    return false;
  }
  sd_current_line_number += 1;
  int len = 0;
  while (current_file.available())
  {
    if (len >= maxlen)
    {
      return false;
    }
    char c = current_file.read();
    if (c == '\n' || c == '\r')
    {
      break;
    }
    line[len++] = c;
  }
  line[len] = '\0';
  return len || current_file.available();

}

/** if we use a floating z axis we repeat the 2D path until the cutting depth matches the settings **/
bool G_FILEHANDLER::reset_and_repeat_file()
{

  if (!current_file)
  {
    return false;
  }

  if (current_file.seek(0)) // reset the file position
  {
    sd_current_line_number = 0; // reset the current line number
    return true;
  }
  else
  {
    return false;
  }
}
uint32_t G_FILEHANDLER::sd_get_current_line_number()
{
  return sd_current_line_number;
}
SDState G_FILEHANDLER::get_sd_state(bool refresh)
{
  if (!((sd_state == SDState::NotPresent) || (sd_state == SDState::Idle)))
  {
    return sd_state;
  }
  if (!refresh)
  {
    return sd_state;
  }
  last_refresh = 0.0;
  sd_card_refresh();
  return sd_state;
}

void G_FILEHANDLER::sd_get_current_filename(char *name)
{
  if (current_file)
  {
    strcpy(name, current_file.name());
  }
  else
  {
    name[0] = 0;
  }
}






bool G_FILEHANDLER::get_is_ready()
{
  return filehandler_is_ready;
}
void G_FILEHANDLER::sd_card_refresh()
{
  if( ignore_sd || is_locked_for_ui() ){ 
    return; 
  }
  if (millis() - last_refresh > 10000)
  {
    last_refresh = millis();
    if (!SD.exists(folder_root) || SD.cardSize() <= 0)
    {
      if( current_file ){
        current_file.close();
      }

      SD.end();
      filehandler_initialize(true);
    }
  }
}

bool G_FILEHANDLER::get_has_sd_card()
{
  return ignore_sd ? false : has_sd_card;
}
bool G_FILEHANDLER::read_line(char *line, int maxlen)
{
  line_number += 1;
  int len = 0;
  while (current_file.available())
  {
    if (len >= maxlen)
    {
      return false;
    }
    char c = current_file.read();
    if (c == '\n')
    {
      break;
    }
    line[len++] = c;
  }
  line[len] = '\0';
  return len || current_file.available();
}

int G_FILEHANDLER::get_error_code()
{
  return error_code;
}
void G_FILEHANDLER::close_current_folder()
{
  current_file.close();
  line_number = 0;
  sd_state = SDState::Idle;
}
void G_FILEHANDLER::open_folder(String folder)
{
  sd_state = SDState::BusyCustom;
  current_file = SD.open(folder);
  current_file.rewindDirectory();
}

String G_FILEHANDLER::get_next_filename(String extension)
{
  if( is_locked_for_ui() ){ 
    return ""; 
  }
  File entry = current_file.openNextFile();
  if (!entry)
  {
    return "";
  }
  String extension_upper = extension;
  extension_upper.toUpperCase();
  while (
      entry &&
      (
        (String(entry.name()).indexOf(extension) == -1 && String(entry.name()).indexOf(extension_upper) == -1) ||
        entry.isDirectory()
       )
      )
  {
    entry = current_file.openNextFile();
  }
  String file_name = entry ? String(entry.name()) : "";
  String file_name_no_path = file_name;
  if (file_name.lastIndexOf("/") != -1)
  {
    file_name_no_path = file_name.substring(file_name.lastIndexOf("/") + 1, file_name.length());
  }
  entry.close();
  return file_name_no_path;
}

int G_FILEHANDLER::count_files_in_folder_by_extension(String folder, String extension)
{
  if( is_locked_for_ui() ){ 
    return 0; 
  }
  int num_files = 0;
  String file_name;
  open_folder(folder);
  while (true)
  {
    file_name = get_next_filename(extension);
    if (file_name.length() <= 0)
    {
      break;
    }
    ++num_files;
  }
  close_current_folder();
  return num_files;
}

void G_FILEHANDLER::get_files_in_folder_by_extension(String folder, String extension, char files[][100])
{
  if( is_locked_for_ui() ){ 
    return; 
  }
  /** rewind directory **/
  open_folder(folder);
  String file_name;
  int index = 0;
  while (true)
  {
    file_name = get_next_filename(extension);
    if (file_name.length() <= 0)
    {
      break;
    }
    char c[100];
    file_name.toCharArray(c, sizeof(c));
    memcpy(files[index], c, sizeof(files[index]));
    ++index;
  }
  close_current_folder();
}

String G_FILEHANDLER::get_card_type()
{
  card_type = SD.cardType();
  if (card_type == CARD_NONE)
  {
    error_code = 2;
    return "No SD card attached";
  }
  if (card_type == CARD_MMC)
  {
    return "MMC";
  }
  else if (card_type == CARD_SD)
  {
    return "SDSC";
  }
  else if (card_type == CARD_SDHC)
  {
    return "SDHC";
  }
  else
  {
    return "???";
  }
}

String G_FILEHANDLER::get_folder_settings()
{
  return String(folder_settings);
}

String G_FILEHANDLER::get_folder_gcode()
{
  return String(folder_gcode);
}




int G_FILEHANDLER::open_file(String folder, String file)
{
  if( is_locked_for_ui() ){ return 1; }
  close_current_folder();
  String file_path = folder + "/" + file;
  if ( is_locked_for_ui() || !SD.exists(file_path) )
  {
    /** no sd card **/
    return 1;
  }
  sd_state = SDState::BusyCustom;
  current_file = SD.open(file_path, FILE_READ);
  if (!current_file)
  {
    return 2;
  }
  return 0;
}
