**Table of Contents**

[Introduction](#Introduction)\
[Compiling and building](#Compiling)\
[version 4.32](#version432)\
[version 4.31](#version431)\
[version 4.3](#version43)\
[version 4.2](#version42)\
[version 4.01](#version401)\
[version 4.0](#version40)\
[version 3.91](#version391)\
[version 3.71](#version371)\
[Version 3.7](#version37)\
[Version 3.5](#version35)\
[Version 3.4](#version34)\
[Known issues](#Known)\
[License](#License)


Find more information at:
https://wiki.deomgevingsmonitor.nl/index.php/Main_Page

## Introduction <a name="Introduction"></a>
This release makes it possible to operate the Omgevings Monitor (OM) independently of USB power for a longer period >> days. Needless to write, this depends on the ambient temperature and the amount of daylight that the solar panel receives. In battery operation the status LED will flash briefly 3 to 4 times while measurements are being carried out. Green means the battery has sufficient charge. If the LED flashes red during measurement the remaining charge level is approximately 20%, the battery needs to be charged. The measurements are carried out once every 15 minutes in battery operation.

It is possible to use the OM as a handheld by using the BOOT0 button in case of battery operation. The OM then wakes up from energy saving mode. This becomes visible by the lighting of the LEDs. By pressing the user button, the LEDs go out again and the OM starts operating in energy saving mode again. This can take up to about 45 seconds.\
If the battery is low, the system will enter a very low energy state and will not operate normal until the battery is charged. By pressing the reset button while charging, the device becomes operational again. 
As long as there is some energy in the battery, it is possible that the device will start up briefly and then go back into battery protection mode.

The dB LED has the following meaning, more or less in the order of the rainbow:
- dBA >= 90 white
- dBA >= 80 - dBA < 90 red
- dBA >= 70 - dBA < 80 yellow
- dBA >= 60 - dBA < 70 green
- dBA >= 50 - dBA < 60 light blue
- dBA >= 40 - dBA < 50 blue
- dBA >= 35 - dBA < 40 purple
- dBA < 35 LED off, equals to noise level of the microphone

## Compiling and building <a name="Compiling"></a>
The project was developed with STM32CubeIDE 1.16.1, but also builds in STM32CubeIDE Version: 1.17.0

The project has two build configurations: Debug & Release.\
Select the option under Project -> Build configurations -> Set Active

For installation the STM32CubeIDE follow the procedure under ["Compileren en bouwen"](https://wiki.deomgevingsmonitor.nl/index.php/Programmeeromgeving).

The installation of the sources in the IDE differs slightly from the procedure described on the previously mentioned page.

Under Ubuntu Linux I encountered problems downloading the necessary firmware automatically. The possibility to login by selecting Help -> STM32Cube updates -> Connection to myST -> Enter myST account information shows no dialog. If necessary, download and install it manually in advance. The firmware can be found under ["get stm32l072RZTX firmware"](https://www.st.com/en/embedded-software/stm32cubel0.html#get-software). Install first the "STM32Cube MCU Package for STM32L0 series 1.20.0" in the folder "~/STM32Cube/Repository". Second copy the "Patch for STM32CubeL0" over the installad package. If you know the solution for configuring the STM32CubeIDE so that it automatically downloads the necessary packages, please mention it in an issue of this repo.

Get the sources by using git or download the software as a zip file. Here the procedure is described based on a zip file.

Place the unzipped sources in your working directory of STM32CubeIDE.

For clearity rename the source folder to MJSGadget - dB meter. 

Start STM32CubeIDE\
If this is the first time that STM32CubeIDE is used, choose the 'Import Project' option. Otherwise
- Select File -> Open Projects from File System. Select your source folder MJSGadget - dB Meter. 
- Uncheck the "Search for nested projects". Leave the rest of the settings in the "Import Projects from File System or Archive" unchanged. 
- Click Finish

If you are under Windows and not logged in to ST in the IDE, login to myST first. (Under Ubuntu Linux this is an issue for me. For this reason, the firmware was manually installed earlier in this procedure.)

In the Project Explorer double click on the file 'MJSGadget  - dB meter.ioc'.* (On linux, when the Pinout view is not shown, close the tab and reopen the ioc file.)\
When the 'New STM32Cube firmware version available' dialog is shown, choose 'Continue'.\
After the 'MJSGadget - dB meter.ioc' window has opened, click (in the left column) on 'Middleware and Software Packs'.\
Click on USB_DEVICE. If everything is correct, there is a green check mark in front of USB_DEVICE. 

Click in the ribbon of the IDE on the button 'Device Configuration Tool Code Generation' (icon with yellow wheel) 

Click Yes in the "Open Associated Perpective?"dialog.

After the configuration process unfortunately, a number of necessary files are deleted. 

Therefore, perform the following steps:

- Go to the folder C:\Users\<user>\STM32Cube\Repository\STM32Cube_FW_L0_V1.12.2\Drivers\CMSIS\DSP
- Select the folder Include and use Ctrl-C to copy the contents
- In the Project Explorer select the folder Drivers/CMSIS/DSP and use Ctrl-V
- Go to the folder C:\Users\<user>\STM32Cube\Repository\STM32Cube_FW_L0_V1.12.2\Drivers\CMSIS
- Select the folder Lib and use Ctrl-C to copy the contents.
- In the Project Explorer select the folder Drivers/CMSIS and use Ctrl-V
- (For Linux the folders can be found under ~/STM32Cube/Repository/STM32Cube_FW_L0_V1.12.0/Drivers/CMSIS.)

Execute Project -> Clean.

Check 'Start a build immediately' and choose 'Clean'.

## version 4.32 <a name="version432"></a>
Optimizing the datagram to be send and when operating on battery send only PM when PM measurement is active. Preventing cleared values upload during the cycle PM measurement is not executed. NOx values are only uploaded when operated on USB power.

## version 4.31 <a name="version431"></a>
Building on fresh Ubuntu verified. Readme instruction supplemented and corrected. Some files renamed to lower case and aligned with the sourcefiles.

## version 4.3 <a name="version43"></a>
Various fixes to better handle error handling in different conditions. Feedback when the user button is pressed by the VOC LED, which then lights up white briefly. The 128x64 OLED display is optionally added to the I2C2 bus. This option is disabled by default in the build.

## version 4.2 <a name="version42"></a>
Optional display can be compiled for local display of values. (still in development) The define directive "#define SSD1306" is located at about line 24 in ssd1306_128x64_i2c.h

## version 4.01 <a name="version401"></a>
Correction for LED color on low battery and some clean up.

## version 4.0 <a name="version40"></a>
USB monitoring is possible bij defining USBLOGGING, approximately at line 55, in usbd_cdc_if.h. This has an effect on the inrush current that is inexplicable to me, making it impossible for me to measure the current consumption during code execution and during stop mode. For this reason, the directive is disabled by default.

## version 3.91 <a name="version391"></a>
fix voor local build. The project was nested with another project in the debug configuration. This has been fixed.

## version 3.71 <a name="version371"></a>
fix for reset timeoutcntr

## version 3.7 <a name="version37"></a>
When USB powered and a sen54 or sen55 is connected the on-board SGP40 is disabled. VOC is handled by the sen54/55 module. Due to the VOC algorithm of the sen5x the value is reset after every powercycle. Therefore the SGP40 is enabled when operated on batterypower. When a sen54 or sen55 is attached, a mode is added that the PM measurement is disabled, but the VOC index, temperature and NOx is measured. This mode is selected by press and hold the userbuttton for 2 seconds. This mode uses less power than continious PM measurement, but it is not an energy saving mode. The battery is empty in less than a day. When the capacity is below approx. 40% the sen5x is switched off.  The VOC LED flashes green when selected and red when PM enabled again. When operating on battery the sensors are switched off before sending the datagram by WiFi. Status turns white when PM measurement is executed.

## version 3.5 <a name="version35"></a>
Source code parts moved from main.x to measurement.x. Auto reboot on incorrect start-up, shown by 3 red LEDs short after booting. After firmware update please reset 2 times with 10 second pause for the 2nd reset. Otherwise the current consumption in stopmode is about 12mA instead of 2.3mA. Who can explain this difference or find a solution?

## version 3.4 <a name="version34"></a>
NOx and VOC from sen55 is included in measurement. For this it is necessary that the sen55 has a longer runtime, this needs a USB powersupply. NOx is after 6 hours of operation reliable, VOC after about an hour. With battery operation the values for NOx and VOC reads zero from the sen55.

## Known issues <a name="Known"></a>
- If the OM runs on battery power and the boot button is pressed, it can take up to half a minute before the device is fully operational and the LED display starts when using a Sensirion sen5x sensor.
- If 3 red LEDs illuminate repeatedly during start-up, the reset button must be pressed for approximately 3 seconds. This ensures that the ESP has sufficient time to reset. Then check whether the WiFi connection still works correctly. The OM must have uploaded the first values for temperature and humidity within one minute. If in doubt, the procedure for connecting to the home network can be carried out again.
- The system must be reset twice to reach optimal energy saving mode.
- If the user button is pressed to change the LED mode, it may take up to 30 seconds for the system to enter standby mode when a sen5x is attached. This is not really an issue because the system is waiting for a particle measurement.

## License <a name="License"></a>
Parts are licensed under GNU GENERAL PUBLIC LICENSE Version 3 and GNU AFFERO GENERAL PUBLIC LICENSE Version 3AGPL-3.0