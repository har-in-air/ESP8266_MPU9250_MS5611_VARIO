#include <Arduino.h>
#include <EEPROM.h>
#include "nvd.h"
#include "util.h"

// saving and retrieving non-volatile data (NVD) to / from
// ESP8266 flash. 

NVD nvd;

void nvd_Init(void)   {
	EEPROM.begin(NVD_SIZE_BYTES);
	for (int inx = 0; inx < NVD_SIZE_BYTES; inx++) {
		nvd.buf[inx] = EEPROM.read(inx);
		}
	uint16_t checkSum = nvd_CheckSum();	
	//Serial.printf("Calculated checkSum = 0x%04x\r\n", checkSum);	
	//Serial.printf("Saved checkSum = 0x%04x\r\n", nvd.params.checkSum);
	
    if ((checkSum ^ nvd.params.checkSum) == 0xFFFF) {
		Serial.printf("nvd checkSum ok\r\n");
		Serial.printf("axBias = %d\r\n", nvd.params.axBias);
		Serial.printf("ayBias = %d\r\n", nvd.params.ayBias);
		Serial.printf("azBias = %d\r\n", nvd.params.azBias);
		Serial.printf("gxBias = %d\r\n", nvd.params.gxBias);
		Serial.printf("gyBias = %d\r\n", nvd.params.gyBias);
		Serial.printf("gzBias = %d\r\n", nvd.params.gzBias);
		Serial.printf("calibrated = 0x%X\r\n", nvd.params.calibrated);
		}
   else  {
		Serial.printf("nvd bad checkSum, saving safe values to flash\r\n");
		nvd.params.axBias = 0;
		nvd.params.ayBias = 0;
		nvd.params.azBias = 0;
		nvd.params.gxBias = 0;
		nvd.params.gyBias = 0;
		nvd.params.gzBias = 0;
		nvd.params.calibrated = 0;
		checkSum = nvd_CheckSum();
		nvd.params.checkSum = ~checkSum;
	    for  (int inx = 0; inx < NVD_SIZE_BYTES; inx++){
			EEPROM.write(inx, nvd.buf[inx]);
			}
		EEPROM.commit();
		}
   EEPROM.end();
   }

   
uint16_t nvd_CheckSum(void) {
	uint16_t checkSum = 0;
	for (int inx = 0; inx < NVD_SIZE_BYTES-2; inx++) {
		checkSum += (uint16_t)nvd.buf[inx]; 
		}
	return checkSum;
	}
	

void nvd_SaveCalibrationParams(int16_t axb, int16_t ayb, int16_t azb, int16_t gxb, int16_t gyb, int16_t gzb, int16_t calibrated) {
	EEPROM.begin(NVD_SIZE_BYTES);
	nvd.params.axBias = axb;
	nvd.params.ayBias = ayb;
	nvd.params.azBias = azb;
	nvd.params.gxBias = gxb;
	nvd.params.gyBias = gyb;
	nvd.params.gzBias = gzb;
	nvd.params.calibrated = calibrated;
	uint16_t checkSum = nvd_CheckSum();
	nvd.params.checkSum = ~checkSum;
	for  (int inx = 0; inx < NVD_SIZE_BYTES; inx++){
		EEPROM.write(inx, nvd.buf[inx]);
		}
	EEPROM.commit();
	EEPROM.end();
	}	


