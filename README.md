# ESP8266_MPU9250_MS5611_VARIO
DIY friendly, cheap, accurate, fast responding audio variometer using easily available modules

Software development platform used : Arduino 1.8.2 on Windows 10 x64

ESP8285  ESP-M2 module purchased from Aliexpress for US$1.80 including shipping. Note that it was advertised as an ESP-M1 with 16Mbit flash, but the silkscreen on the back of the board shows ESP-M2, and the ESP8285 has an 8Mbit on-chip flash. You can use any ESP8266 module that brings out the gpio pins that are used in this project. If using ESP8266 instead of ESP8285, select "Generic 8266 module" in Arduino board type. If you decide to use a development board like the Wemos D1 or Nodemcu, bear in mind that the onboard USB-UART interface will draw current all the time, so the auto-power down feature will be compromised, as will the active power consumption.  I measured ~26mA current draw in active vario mode with the ESP8285 configured for 80MHz clock. 

CJMCU-117 MPU9250+MS5611 module, purchased from ebay for US$3.16 including shipping !! Prices seem to have gone up now. The magnetometer isn't used in this project, I used this module because it was the cheapest available at the time with an MS5611 barometer.  I replaced the 10K I2C pullup resistors with 3K3 resistors for a more reliable interface with 400kHz I2C clock. Also note I pulled up the PS and NCS pins to the CJMCU-117 LDO regulator output, not the circuit 3.3V.

Li-ion charger module, purchased from Electrodragon for US$0.50.  I cut the board in half to fit in the case I used, removing the battery connector, and soldered the lipoly battery wires directly to the charger board. I also reduced the charging current to match the 520mAH battery I used, by replacing the 1K5 charging current set resistor with 3K. See the MCP73831T datasheet for resistor values to suit your charging requirement. Rule of thumb is max charging current < 0.5C.

I used a separate LDO regulator for powering the ESP8285. If you take power from the CJMCU-117 LDO regulator output, there will be a clearly apparent increase in barometer sensor noise. I recommend HT7333 for low dropout voltage and low quiescent current draw. The AMS117 regulator is unsuitable for this application.

Use a broadband piezo speaker like the Kingstate KPEG006 or similar part from PUI Audio. Most passive piezo buzzers are meant to be used as alarms at a resonant frequency, so are not a good match for this application. I'm fine with the volume as i use an open face helmet and don't like a loud vario. If you need more volume, you can use a dual inverting schmitt trigger chip e.g. SN74LVC2G14, connect the inverters in series to the pwm signal, and take the piezo connection from the two inverter outputs. With the inverter chip powered from VBAT, this will give you a push-pull configuration with peak to peak driving voltage of ~8V (VBAT x 2) rather than 3.3V. You can get a little more fancy and use a spare gpio pin + nchan mosfet + pchan mosfet to power the logic gates only when generating audio. Otherwise the piezo will always be polarized when the vario is on - that's not good for piezo working life.

Any external USB-UART module (FTDI232, CH340, CP2102) can be used for flashing the ESP8285. Less than US$1 including shipping, from ebay, aliexpress etc. I had reliability problems flashing at more than 115200baud with the ESP8285 (I normally use 921600baud with ESP8266 modules). Connect TX / RX of the USB-UART module to RXD0 / TXD0 of the ESP8285. 

The MPU9250 will have to be calibrated for your use. You will also have to map the sensor axes appropriately if you mount the sensor board in a different orientation to what I've used in the case (horizontal, but upside down) when the case is in normal orientation with the piezo on top.
