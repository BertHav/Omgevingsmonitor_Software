Find more information at:
https://wiki.deomgevingsmonitor.nl/index.php/Main_Page

# General
This release makes it possible to operate the Omgevings Monitor (OM) independently of USB power for a longer period >> days. Needless to write, this depends on the ambient temperature and the amount of daylight that the solar panel receives. In battery operation the status LED will flash briefly 3 to 4 times while measurements are being carried out. Green means the battery has sufficient charge. If the LED flashes red during measaurement the remaining charge level is approximately 20%, the battery needs to be charged. The measurements are carried out once every 15 minutes in battery operation.

It is possible to use the OM as a handheld by using the BOOT0 button in case of battery operation. The OM then wakes up from energy saving mode. This becomes visible by the lighting of the LEDs. By pressing the user button, the LEDs go out again and the OM starts operating in energy saving mode again. 

If the battery is low, the system will enter a very low energy state and will not operate normal until the battery is charged. By pressing the reset button while charging, the device becomes operational again. 
As long as there is some energy in the battery, it is possible that the device will start up briefly and then go back into battery protection mode.

The dB LED has the following meaning, more or less in the order of the rainbow:
dBA >= 90 white
dBA >= 80 && dBA < 90 red
dBA >= 70 && dBA < 80 yellow
dBA >= 60 && dBA < 70 green
dBA >= 50 && dBA < 60 light blue
dBA >= 40 && dBA < 50 blue
dBA >= 35 && dBA < 40 purple
dBA < 35 LED off, equals to noise level of the microphone


#### version 3.91 
fix voor local build. The project was nested with another project in the debug configuration. This has been fixed.

Follow the procedure under ["Compileren en bouwen"](https://wiki.deomgevingsmonitor.nl/index.php/Programmeeromgeving).

Start STM32CubeIDE, if you are not logged in to ST in the IDE, do that first. 

Below is the translation in English or something similar. 

In the Project Explorer double click on the file 'MJSGadget*.ioc'. 
Choose 'Continue' in the 'New STM32Cube firmware version available' dialog. 
After the MJSGadget*.ioc window has opened, click (in the left column) on 'Middleware and Software Packs'. 
Click on USB_DEVICE. If everything is correct, there is a green check mark in front of USB_DEVICE. 

Click on the button 'Device Configuration Tool Code Generation' (icon with yellow wheel) 

Unfortunately, a number of necessary files are deleted. 

Therefore, perform the following steps:

Go to the folder C:\Users\<user>\STM32Cube\Repository\STM32Cube_FW_L0_V1.12.2\Drivers\CMSIS\DSP
Select the folder Include and use Ctrl-C to copy the contents
In the Project Explorer select the folder Drivers/CMSIS/DSP and use Ctrl-V
Go to the folder C:\Users\<user>\STM32Cube\Repository\STM32Cube_FW_L0_V1.12.2\Drivers\CMSIS
Select the folder Lib and use Ctrl-C to copy the contents.
In the Project Explorer select the folder Drivers/CMSIS and use Ctrl-V
(For Linux the folders can be found under ~/STM32Cube/Repository/STM32Cube_FW_L0_V1.12.2/Drivers/CMSIS.)

Run Project -> Clean.
Check 'Start a build immediately' and choose 'Clean'.

#### version 3.71 
fix for reset timeoutcntr

#### Version 3.7 
When USB powered and a sen54 or sen55 is connected the on-board SGP40 is disabled. VOC is handled by the sen54/55 module. Due to the VOC algorithm of the sen5x the value is reset after every powercycle. Therefore the SGP40 is enabled when operated on batterypower. When a sen54 or sen55 is attached, a mode is added that the PM measurement is disabled, but the VOC index, temperature and NOx is measured. This mode is selected by press and hold the userbuttton for 2 seconds. This mode uses less power than continious PM measurement, but it is not an energy saving mode. The battery is empty in less than a day. When the capacity is below approx. 40% the sen5x is switched off.  The VOC LED flashes green when selected and red when PM enabled again. When operating on battery the sensors are switched off before sending the datagram by WiFi. Status turns white when PM measurement is executed.

#### Version 3.5 
Source code parts moved from main.x to measurement.x. Auto reboot on incorrect start-up, shown by 3 red LEDs short after booting. After firmware update please reset 2 times with 10 second pause for the 2nd reset. Otherwise the current consumption in stopmode is about 12mA instead of 2.3mA. Who can explain this difference or find a solution?

#### Version 3.4 
NOx and VOC from sen55 is included in measurement. For this it is necessary that the sen55 has a longer runtime, this needs a USB powersupply. NOx is after 6 hours of operation reliable, VOC after about an hour. With battery operation the values for NOx and VOC reads zero from the sen55.
