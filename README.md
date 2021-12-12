# ESP8266_MPU9250_MS5611_VARIO

DIY friendly, cheap, fast and accurate audio-only variometer using widely available off-the-shelf modules, so minimal assembly required.

WARNING : This project is no longer being maintained. The [ESP8266_BLUETOOTH_AUDIO_VARIO](https://github.com/har-in-air/ESP8266_BLUETOOTH_AUDIO_VARIO) project has
up-to-date algorithm and feature implementations.

Features :
1. uses earthframe-z acceleration computed from MPU9250 data, and z altitude calculated from MS5611 barometric pressure, for a "zero-lag" variometer response
2. zeroes beeps, for weak lift, indicated with short pulses and long intervals.
3. climbtone beeps, for lift strong enough that you should try to core the lift, indicated with longer beeps and shorter intervals.
4. sinktone beeps, for sink rates significantly more than glider sink rate in still air, indicated with descending frequency long beeps.
5. offscale tones generated for climbrates > +10m/s or sinkrates > -10m/s
6. vario will automatically go into a low-power drain sleep state if there is no vertical motion for a while. This is in case you forgot to power off your vario after landing. Power off and on again to recover from this state.
7. easy to assemble using widely available, off-the-shelf modules
8. cheap !! Takes advantage of the popularity of the modules used in this project - you can source them for just a few $ (worldwide shipping included) on ebay and aliexpress.
9. configurable via webpage form with the vario acting as a WiFi access point and HTTP web server.
10. 3D printer files for an appropriate case for the vario can be found at https://github.com/antoine5974/esp8266-vario-3D-printed-casing, thanks to Antoine Lomberty.

Demo video at https://youtu.be/v9U-UKarfZY . Note that for this demo, the vario was configured to show off the filter responsiveness, not for flying.

Software development platform used : Arduino 1.8.2 on Windows 10 x64. See https://github.com/esp8266/Arduino for instructions on how to install ESP8266 SDK/libraries.

You can find a comprehensive guide to the ESP8266 at https://tttapa.github.io/ESP8266/Chap01%20-%20ESP8266.html

For a demonstration of the effectiveness of the Kalman sensor fusion algorithm used in this vario, check out the /docs
directory in the project https://github.com/har-in-air/ESP32_IMU_BARO_GPS_LOGGER

ESP8285 ESP-M2 module purchased from Aliexpress for US$1.80 including shipping. It was advertised as an ESP-M1 with 16Mbit flash, but the silkscreen on the back of the board is marked "ESP-M2 E". And the ESP8285 has an 8Mbit (1Mbyte) flash. You can use any ESP8266 module that brings out the gpio pins that are used in this project, e.g. the ESP-12E or ESP-12F. 

I used an ESP8285 (ESP8266 with on-chip flash memory) module because it is literally thumbnail sized. An ESP8266 module would not have fit in the small plastic case I used for the project. If you do use ESP8266 instead of ESP8285, select "Generic ESP8266 module" for Arduino board type.  If you decide to use a development board like the Wemos D1 or Nodemcu, bear in mind that the onboard USB-UART interface will draw current all the time, so the auto-power down feature will be compromised, as will the active power consumption.  I measured ~26mA current draw in active vario mode with the ESP8285 configured for 80MHz clock. 

I found the CJMCU-117 MPU9250+MS5611 module on ebay for US$3.16 including shipping, when MS5611 modules were at least US$7 ! Unfortunately, prices for this module have gone up now. The MPU9250 magnetometer isn't used in this project. I used this module because it was the cheapest IMU module available at the time with an MS5611 barometer.  I replaced the 10K I2C pullup resistors with 3K3 resistors for a more reliable interface with 400kHz I2C clock. I pulled up the PS and NCS pins to the CJMCU-117 LDO regulator output, not the circuit schematic 3.3V. 

I did experiment with a BMP280 pressure sensor as it's much cheaper, but found the MS5611 to be better for a vario application. At comparable or better sampling rates, the BMP280 had occasional nasty high amplitude noise spikes even if the overall noise variance was similar for a large (e.g. ~1000) number of samples. And with the highest internal pressure oversampling to minimize noise, the sample interval is too long. No problem for an altimeter, but not for a variometer.

MCP73831 based USB Li-ion charger module, purchased from Electrodragon for US$0.50. You can also get cheap TP4056 based chargers. I cut the board in half to fit in the case I used (removing the battery connector on the charger module), and soldered the lipoly battery wires directly to the charger board. I also reduced the charging current to match the 520mAH battery I used, by replacing the 1K5 charging current set resistor with 3K. See the MCP73831T datasheet for resistor values to suit your charging requirement. Rule of thumb is max charging current < 0.5C, e.g. for a 500mAH battery, max charging current should be < 250mA. The battery charger module isn't shown in the circuit schematic, you just have to connect the module output to the battery terminals, permanently.

I used a separate 3.3V LDO voltage regulator for the ESP8285. If you take power from the CJMCU-117 LDO regulator output, there will be a clearly apparent increase in barometer sensor noise. I recommend the HT7333 for its low dropout voltage and low quiescent current draw. The AMS1117 regulator is unsuitable for this application.

Use a low-voltage, broadband piezo speaker like the Kingstate KPEG006 or similar part from PUI Audio. I found a cheap and loud piezo on Aliexpress (see the pics directory), but it's a bit large if you're looking to fit everything in the smallest case possible. Most passive piezo buzzers are meant to be used as alarms at a fixed resonant frequency, so are not a good match for this application. I'm fine with the volume as I use an open face helmet and don't like a loud vario. If you need more volume, you can use a dual inverting schmitt trigger chip e.g. SN74LVC2G14, connect the inverters in series to the pwm signal, and take the piezo connection from the two inverter outputs. With the inverter chip powered from VBAT, this will give you a push-pull configuration with a peak-to-peak driving voltage of ~8V (VBAT x 2) rather than 3.3V. You can get a little more fancy and use a spare gpio pin + nchan mosfet + pchan mosfet to power the inverter chip only when generating audio. Otherwise the piezo will always be polarized when the vario is on - that's not good for piezo working life. An alternative loud speaker option (that will draw more current) is to use a small 8ohm - 32ohm speaker (e.g. salvaged from a cellphone) with a mosfet drive - this option is included in the circuit schematic.

Any external USB-UART module (FTDI232, CH340, CP2102) can be used for flashing the ESP8285. I got a CP2102 module for about US$1 including shipping, from aliexpress. Ensure the USB-UART output logic level is at 3.3V. Only the TX, RX and GND pins of the USB-UART module should be connected to the ESP82xx. Connect TX / RX of the USB-UART module to RXD0 / TXD0 of the ESP82xx. 

If you use the ESP8285, make sure you select "Generic ESP8285 module" as board type in the Arduino IDE, as it uses a different internal SPI flash interface mode than the ESP8266. Set the flash partition option to (1M /64k SPIFFS) and use 115200 baudrate for flashing.

The MPU9250 accelerometer and gyro biases will have to be calibrated for your unit. The gyro is automatically calibrated each time on power on. The accelerometer needs to be manually calibrated before the vario can be used. Calibration parameters are saved to and retrieved from flash.

You will also have to map the sensor axes appropriately if you mount the sensor board in a different orientation. In my case the CJMCU-117 sensor board is horizontal, but upside down when the box is oriented normally with the piezo speaker on top. For debug purposes (testing yaw/pitch/roll calculations) the sensor board silkscreen +X points "forward", so the silkscreen +Y arrow points to the right. You should then get positive pitch when the box is rotated up (around the Y axis), positive roll when the box is rotated to the right (around the X axis), and positive yaw when the box is rotated flat (clockwise around the Z axis). Note that since the magnetometer isn't used the yaw value is initialized to 0 for whatever forward direction the unit is powered up with. 

## The PROGRAM/CONFIGURE/CALIBRATE button
There are two buttons on the vario. One is the ESP8266 hardware RESET button, and the other (PGM/CONF/CAL) button serves three functions.
1. (Program) Press and hold the PGM/CONF/CAL button, power on the vario and then release the button. Or, with the vario powered on, press and hold the PGM/CONF/CAL button and momentarily press the RESET button. Then release the PGM/CONF/CAL button. The ESP8266 is now ready for application code flashing. You can use the Arduino IDE to upload new code.
2. (Configure) Power on the vario and then (within a couple of seconds) press and hold the PGM/CONF/CAL button. Wait until you hear a low frequency tone start. Release the button, and the vario will set up in wifi configuration mode. Use your smartphone or PC to connect to the open WiFi access point "EspVarioConfig". Once connected, use a web browser to open the webpage at the url http://192.168.4.1 If you're on a PC with Windows, the url http://espvarioconfig.local will also work. Modify the configuration parameters and update the vario by pressing the 'SEND' button in the webpage form. You can reset the vario configuration parameters to 'factory defaults' by pressing the 'SET DEFAULTS' button in the webpage form. When you are done, switch off the vario and you are set to go with the new configuration. NOTE : Do not leave the vario in wifi configuration mode longer than necessary, the wifi radio will drain the vario battery pretty quickly !!
3. (Calibrate) The vario attempts to calibrate the gyro automatically each time on power up. However, the accelerometer needs to be calibrated manually. This is only required if you have never done this before, or if the calibration values in flash are corrupted. To calibrate the accelerometer, power on the vario and wait until you hear the battery voltage feedback beeps and then the countdown beeps for gyro calibration. When you hear the countdown beeps, press the PGM/CONF/CAL button. You should hear a confirmation tone, release the button and then you will have about 10 seconds to place the vario on a level surface, undisturbed, with the accelerometer z axis vertical (pointing up or down). You will hear a sequence of short pulses before calibration starts. Leave the vario undisturbed until accelerometer calibration is complete, and you're set.

## Normal running mode
When you power on the vario, after a few seconds you will hear the battery voltage feedback tones. 5 beeps for a fully charged battery, 1 beep if it needs to be charged before use.

Following this, if there are no issues with retrieving the calibration and configuration data from flash, or with sensor communication, you will hear a sequence of 10 beeps. This is the countdown to gyro calibration. Make sure the vario is left undisturbed at this point. When the calibration is done, you will hear a confirmation beep. If you cannot ensure the vario is undisturbed, no worries, it will use the last saved gyro calibration values. So if you have successfully calibrated the vario gyro in the past few days or in similar temperature conditions, it should be fine.

Following this, you will hear some strange squawking/squealing noises as the kalman filter estimates the accelerometer bias. When it has settled down, the vario will function as intended :-).

### Problems
If the non-volatile data in flash was corrupted, the vario will reset the configuration data to default values. But the accelerometer and gyroscope calibration data will be zeroed out, and now the accelerometer must be manually calibrated. This is also the case when you have programmed the flash for the first time, or if you have wiped the ESP8266 flash for some reason. 

Sometimes ESP8266 communication with the sensors may hang after reflashing the code. If you hear the error tones for mpu9250 or ms5611, just power cycle the vario, i.e. switch off, wait a few seconds for the supply bypass capacitors to discharge, and then switch on.

It's a good idea to have the Arduino serial monitor console open after flashing the code with the vario still connected via USB-UART cable (@115200 baud). You can monitor the debug print messages on the console. This is especially useful for manual accelerometer calibration for the first time, or if there are errors. You will see prompts for the required actions, this is easier than recognizing and responding to unfamiliar sequences of beeps ...
