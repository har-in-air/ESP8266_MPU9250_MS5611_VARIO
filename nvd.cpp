#include <Arduino.h>
#include <EEPROM.h>
#include "nvd.h"
#include "util.h"

int gbNvdChange;
NVD_PARAMS nvd;

void nvd_Init(void)   {
	uint16_t checkSum;
	EEPROM.begin(NVD_SIZE_BYTES);
	uint8_t* pBuf = (uint8_t*)&nvd;
	for (int inx = 0; inx < NVD_SIZE_BYTES; inx++) {
		pBuf[inx] = EEPROM.read(inx);
		}
	checkSum = 0;
	for (int inx = 0; inx < NVD_SIZE_BYTES-2; inx++) {
		checkSum += (uint16_t)pBuf[inx]; 
		}
	Serial.printf("checkSum = 0x%04x\r\n", checkSum);	
	Serial.printf("Stored checkSum = 0x%04x\r\n", nvd.checkSum);
	
    if ((checkSum ^ nvd.checkSum) == 0xFFFF) {
		Serial.printf("nvd checkSum ok\r\n");
		Serial.printf("axBias = %d\r\n", nvd.axBias);
		Serial.printf("ayBias = %d\r\n", nvd.ayBias);
		Serial.printf("azBias = %d\r\n", nvd.azBias);
		Serial.printf("gxBias = %d\r\n", nvd.gxBias);
		Serial.printf("gyBias = %d\r\n", nvd.gyBias);
		Serial.printf("gzBias = %d\r\n", nvd.gzBias);
		Serial.printf("calibrated = 0x%X\r\n", nvd.calibrated);
		}
   else  {
		Serial.printf("nvd bad checkSum, writing default values\r\n");
		nvd.axBias = 0;
		nvd.ayBias = 0;
		nvd.azBias = 0;
		nvd.gxBias = 0;
		nvd.gyBias = 0;
		nvd.gzBias = 0;
		nvd.calibrated = 0;
		checkSum = 0;
		for (int inx = 0; inx < NVD_SIZE_BYTES-2; inx++) {
			checkSum += (uint16_t)pBuf[inx]; 
			}
		nvd.checkSum = ~checkSum;
	    for  (int inx = 0; inx < NVD_SIZE_BYTES; inx++){
			EEPROM.write(inx, pBuf[inx]);
			}
		EEPROM.commit();
		}
   EEPROM.end();
   }

void nvd_SaveCalibrationParams(int16_t axb, int16_t ayb, int16_t azb, int16_t gxb, int16_t gyb, int16_t gzb, int16_t calibrated) {
	EEPROM.begin(NVD_SIZE_BYTES);
	nvd.axBias = axb;
	nvd.ayBias = ayb;
	nvd.azBias = azb;
	nvd.gxBias = gxb;
	nvd.gyBias = gyb;
	nvd.gzBias = gzb;
	nvd.calibrated = calibrated;
	uint16_t checkSum = 0;
	uint8_t* pBuf = (uint8_t*) &nvd;
	for (int inx = 0; inx < NVD_SIZE_BYTES-2; inx++) {
		checkSum += (uint16_t)pBuf[inx]; 
		}
	nvd.checkSum = ~checkSum;
	for  (int inx = 0; inx < NVD_SIZE_BYTES; inx++){
		EEPROM.write(inx, pBuf[inx]);
		}
	EEPROM.commit();
	EEPROM.end();
	}	


