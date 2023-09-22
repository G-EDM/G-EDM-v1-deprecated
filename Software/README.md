# Let there be sparks... ⚡⚡⚡

```diff
  ██████        ███████ ██████  ███    ███  
 ██             ██      ██   ██ ████  ████  
 ██   ███ █████ █████   ██   ██ ██ ████ ██ 
 ██    ██       ██      ██   ██ ██  ██  ██ 
  ██████        ███████ ██████  ██      ██ 
```

 
 
# Software section ( coming soon )

    * G-EDM GRBL Fork to control a G-EDM with one or three axis
    * Freecad post processor to create GCode for the G-EDM with Freecads Path workbench
    * GRBL-Plotter configuration files to create GCode with the GRBL-Plotter



# How to install G-RBL

The G-RBL code is build with VisualStudioCode and needs the platform.io (PlatformIO IDE) extension.
After installing PlatformIO in VisualStudio just open the project folder and press the upload button. It should compile G-RBL and flash it to the connected ESP32. It may be needed to press the Boot button on the ESP to enter the flashing mode. As soon as it starts uploading the button can be released. 

Note: After the Software is flashed to the ESP it reboots and enters a calibration mode for the TFT display. If something fails with the initial calibration it is needed to reflash the software two times. 

If a recalibration is needed the file ili9341_config.h needs to be changed and the line:

    #define REPEAT_CAL false

needs to be set to:

    #define REPEAT_CAL true

After the recalivration is done this change needs to be reversed.



# Stay informed:

[>>> Follow the project on Youtube <<<](https://www.youtube.com/@G-EDM/videos)

[>>> Follow the project on Hackaday <<<](https://hackaday.io/project/190371-g-edm)

[>>> Join the discord <<<](https://discord.gg/9cTsyDkEbe)



