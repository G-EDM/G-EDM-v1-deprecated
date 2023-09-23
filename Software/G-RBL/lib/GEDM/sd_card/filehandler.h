#pragma once
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
#include <WString.h>
#include <FS.h>
#include <SD.h>
#include "config/definitions.h"
#include <stdint.h>
#include "shared.h"


extern bool    SD_ready_next;  // Grbl has processed a line and is waiting for another
extern uint8_t SD_client;

extern bool filehandler_is_ready;

class G_FILEHANDLER
{
private:
    bool ready;
    bool lock;
    File     current_file;
    int      error_code;
    uint64_t card_size;
    uint8_t  card_type;
    String folder_root;
    String folder_settings;
    String folder_gcode;
    bool has_sd_card;
    unsigned long last_refresh;
    bool is_in_use;
    bool ignore_sd;
    String get_card_type( void );
    int line_number;
    uint32_t sd_current_line_number; // stores the most recent line number read from the SD
    bool locked_for_ui;
    bool job_finished;

public:
    G_FILEHANDLER(void);
    bool is_locked_for_ui( void );
    bool get_is_ready( void );
    void create_directory_tree( int rounds ); 
    void filehandler_initialize( bool ignore );   
    void sd_card_refresh( void );
    bool get_has_sd_card( void );
    bool get_is_busy( void );
    bool write_file_contents( String file_path, String _data );    
    String get_file_contents(  String file_path  );    
    String get_folder_settings( void );
    String get_folder_gcode( void );
    void open_folder( String folder );
    void close_current_folder( void );
    String get_next_filename( String extension );
    void storeBenchmarks( void );    
    void listDir( const char * dirname, uint8_t levels );
    void createDir( const char * path );
    void removeDir( const char * path );
    void writeFile( const char * path, const char * message );
    void appendFile( const char * path, const char * message );
    void renameFile( const char * path1, const char * path2 );
    void deleteFile( const char * path );
    void testFileIO( const char * path );
    int get_error_code( void );
    int count_files_in_folder_by_extension( String folder, String extension );
    void get_files_in_folder_by_extension( String folder, String extension, char files[][100] );
    bool read_line(char* line, int maxlen);
    int open_file( String folder, String file );
    bool get_file_exists( String full_path );
    bool has_gcode_running( void );
    bool has_job_finished( void );
    void reset_job_finished( void );
    bool reset_and_repeat_file( void );

    SPIClass file_spi;
    void set_spi_instance( SPIClass &_spi );


SDState  get_sd_state(bool refresh);
SDState  set_sd_state(SDState state);
boolean  openFile(const char* path);
boolean  closeFile();
boolean  readFileLine(char* line, int len);
void     readFile(const char* path);
uint32_t sd_get_current_line_number();
void     sd_get_current_filename(char* name);






};


extern G_FILEHANDLER filehandler;