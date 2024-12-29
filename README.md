Find more information at:
https://wiki.deomgevingsmonitor.nl/index.php/Main_Page

Version 3.4 NOx and VOC from sen55 is included in measurement. For this it is necessary that the sen55 has a longer runtime, this needs a USB powersupply. NOx is after 6 hours of operation reliable, VOC after about an hour. With battery operation the values for NOx and VOC reads zero from the sen55.

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

