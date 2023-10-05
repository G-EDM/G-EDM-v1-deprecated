#pragma once

/***
   * 
   * ILI9341 2.4" Touch TFT display
   * MISO MOSI and SCKL are shared between the TFT, Touch and SD card
   * All three devices share one SPI bus and are selected for data io via their chipselect pins (CS, T_CS, SD_CS)
   * 
   * VSS and LED Pins are connected to 3.3v
   * GND is connected to gnd
   * T_IRQ is not connected
   * The below pinout is set in the user_config.h file within the TFT_eSPI library folder
   * 
   *     /libraries/TFT_eSPI/user_config.h
   * 
   * Changes done below won't do anything. This is just for information.
   * 
   ***/
/*
#define GEDM_PIN_MISO  19 // T_DO, SD_MISO, SDO<MISO>
#define GEDM_PIN_MOSI  23 // T_DIN, SD_MOSI, SDI<MOSI>
#define GEDM_PIN_SCLK  18 // T_CLK, SD_SCK, SCK
#define GEDM_PIN_CS    2
#define GEDM_PIN_RST   4
#define GEDM_PIN_T_CS  21
#define GEDM_PIN_DC    15
#define GEDM_PIN_SD_CS 5
*/
