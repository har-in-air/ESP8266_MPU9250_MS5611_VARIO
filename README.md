# ESP8266_MPU9250_MS5611_VARIO
DIY friendly, cheap, fast and accurate audio-only variometer using widely available off-the-shelf modules, so minimal assembly required.

Features :
1. uses earthframe-z acceleration computed from MPU9250 data, and z altitude calculated from MS5611 barometric pressure, for a "zero-lag" variometer response
2. lifty air beeps, for climbrates from +5cm/sec to +50cm/sec, indicated with short pulses and longer intervals.
3. climbtone beeps, for climbrates > +0.5m/s, indicated with longer beeps and shorter intervals.
4. sinktone generated for sinkrates > - 2.5m/s, indicated with continuous tones
5. offscale tones generated for climbrates > +10m/s or sinkrates > -10m/s
6. vario will automatically go into a low-power drain state if there is no vertical motion for 20minutes. This is in case you forgot to power off your vario after landing. Power off and on again to recover from this state.
7. easy to assemble using widely available, off-the-shelf modules
8. cheap !! Takes advantage of the popularity of the modules used in this project - you can source them for just a few $ (worldwide shipping included) on ebay and aliexpress.

Demo video at https://youtu.be/v9U-UKarfZY

Software development platform used : Arduino 1.8.2 on Windows 10 x64. See https://github.com/esp8266/Arduino for instructions on how to install ESP8266 SDK/libraries.

ESP8285 ESP-M2 module purchased from Aliexpress for US$1.80 including shipping. It was advertised as an ESP-M1 with 16Mbit flash, but the silkscreen on the back of the board is marked "ESP-M2 E". And the ESP8285 has an 8Mbit flash. You can use any ESP8266 module that brings out the gpio pins that are used in this project, e.g. the ESP-12E or ESP-12F. You can find a comprehensive beginner's guide to the ESP8266 at https://tttapa.github.io/ESP8266/Chap01%20-%20ESP8266.html

I used an ESP8285 (ESP8266 with on-chip flash memory) module because it is literally thumbnail sized. An ESP8266 module would not have fit in the small plastic case I used for the project. If you do use ESP8266 instead of ESP8285, select "Generic ESP8266 module" for Arduino board type.  If you decide to use a development board like the Wemos D1 or Nodemcu, bear in mind that the onboard USB-UART interface will draw current all the time, so the auto-power down feature will be compromised, as will the active power consumption.  I measured ~26mA current draw in active vario mode with the ESP8285 configured for 80MHz clock. 

I found the CJMCU-117 MPU9250+MS5611 module on ebay for US$3.16 including shipping, when MS5611 modules were at least US$7 ! Unfortunately, prices for this module have gone up now. The MPU9250 magnetometer isn't used in this project. I used this module because it was the cheapest IMU module available at the time with an MS5611 barometer.  I replaced the 10K I2C pullup resistors with 3K3 resistors for a more reliable interface with 400kHz I2C clock. I pulled up the PS and NCS pins to the CJMCU-117 LDO regulator output, not the circuit schematic 3.3V.

MCP73831 based USB Li-ion charger module, purchased from Electrodragon for US$0.50. You can also get cheap TP4056 based chargers. I cut the board in half to fit in the case I used (removing the battery connector on the charger module), and soldered the lipoly battery wires directly to the charger board. I also reduced the charging current to match the 520mAH battery I used, by replacing the 1K5 charging current set resistor with 3K. See the MCP73831T datasheet for resistor values to suit your charging requirement. Rule of thumb is max charging current < 0.5C, e.g. for a 500mAH battery, max charging current should be < 250mA. The battery charger module isn't shown in the circuit schematic, you just have to connect the module output to the battery terminals, permanently.

I used a separate 3.3V LDO voltage regulator for the ESP8285. If you take power from the CJMCU-117 LDO regulator output, there will be a clearly apparent increase in barometer sensor noise. I recommend the HT7333 for low dropout voltage and low quiescent current draw. The AMS1117 regulator is unsuitable for this application.

Use a low-voltage, broadband piezo speaker like the Kingstate KPEG006 or similar part from PUI Audio. Most passive piezo buzzers are meant to be used as alarms at a fixed resonant frequency, so are not a good match for this application. I'm fine with the volume as I use an open face helmet and don't like a loud vario. If you need more volume, you can use a dual inverting schmitt trigger chip e.g. SN74LVC2G14, connect the inverters in series to the pwm signal, and take the piezo connection from the two inverter outputs. With the inverter chip powered from VBAT, this will give you a push-pull configuration with a peak-to-peak driving voltage of ~8V (VBAT x 2) rather than 3.3V. You can get a little more fancy and use a spare gpio pin + nchan mosfet + pchan mosfet to power the inverter chip only when generating audio. Otherwise the piezo will always be polarized when the vario is on - that's not good for piezo working life.

Any external USB-UART module (FTDI232, CH340, CP2102) can be used for flashing the ESP8285. I got a CP2102 module for about US$1 including shipping, from aliexpress. Ensure the USB-UART output logic level is at 3.3V. Only the TX, RX and GND pins of the USB-UART module should be connected to the ESP82xx. Connect TX / RX of the USB-UART module to RXD0 / TXD0 of the ESP82xx.  To put the ESP82xx in programming mode, power on the vario, press and hold the FLASH button, and then press the RESET button. I normally flash ESP8266 modules at 921600baud, but I had reliability problems flashing at more than 115200baud with the ESP8285. If using the ESP8285,  make sure you select "Generic ESP8285 module" as board type in the Arduino IDE, as it uses a different internal SPI flash interface mode than the ESP8266.

The MPU9250 accelerometer and gyro biases will have to be calibrated for your unit. The gyro is calibrated each time on power on. The accelerometer is calibrated if you press the FLASH/CALIBRATION button during the countdown beeps for the gyro calibration.  Calibration parameters are saved to and retrieved from flash. See the code comments for further details on how to set up the unit for 
calibration.

You can of course re-compile the firmware for different thresholds for sinktone, climbtone etc. The most significant configuration settings are in the header file config.h.

You will also have to map the sensor axes appropriately if you mount the sensor board in a different orientation. In my case the CJMCU-117 sensor board is horizontal, but upside down when the box is oriented normally with the piezo speaker on top. For debug purposes (testing yaw/pitch/roll calculations) the sensor board silkscreen +X points "forward", so the silkscreen +Y arrow points to the right. You should then get positive pitch when the box is rotated up (around the Y axis), positive roll when the box is rotated to the right (around the X axis), and positive yaw when the box is rotated flat (clockwise around the Z axis). Note that since the magnetometer isn't used the yaw value is initialized to 0 for whatever forward direction the unit is powered up with. 
