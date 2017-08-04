#include <Arduino.h>
#include <Wire.h>
#include "MPU9250.h"
#include "util.h"

MPU9250::MPU9250() {
	// run Wire.begin() in the main app as there could be other sensors (Ms5611 etc)
	
	// valid for FS = +/- 2G
    ax0g_ = 110;
    ay0g_ = -267;
    azp1g_ = 15853;
    az0g_ = azp1g_ - (int16_t)(1000.0f*MPU9250_2G_SENSITIVITY);
    aScale_ = 1.0f/MPU9250_2G_SENSITIVITY; // accelerometer values in milli-Gs
    
    // valid for FS = 500dps
    gxBias_ = 38; 
    gyBias_ = -18; 
    gzBias_ = 73; 
    gScale_  = 1.0f/MPU9250_500DPS_SENSITIVITY; // gyroscope values in deg/second
	}

void MPU9250::GetAccelGyroData(float* pAccelData, float* pGyroData) {
	uint8_t buf[14];
	int16_t raw[3];
	ReadBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 14, buf);
	raw[0] = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
	raw[1] = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
	raw[2] = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);	
	pAccelData[0] = (float)(raw[0] - ax0g_) * aScale_;
	pAccelData[1] = (float)(raw[1] - ay0g_) * aScale_;
	pAccelData[2] = (float)(raw[2] - az0g_) * aScale_;
	raw[0] = (int16_t)(((uint16_t)buf[8] << 8) | (uint16_t)buf[9]);
	raw[1] = (int16_t)(((uint16_t)buf[10] << 8) | (uint16_t)buf[11]);
	raw[2] = (int16_t)(((uint16_t)buf[12] << 8) | (uint16_t)buf[13]);	
    pGyroData[0] = (float)(raw[0] - gxBias_) * gScale_;
    pGyroData[1] = (float)(raw[1] - gyBias_) * gScale_;
    pGyroData[2] = (float)(raw[2] - gzBias_) * gScale_;
	}
	
	

	
int MPU9250::CheckID(void) {
	uint8_t whoami = ReadByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
	return (( whoami == 0x71) ? 1 : 0);
	}
	

void MPU9250::Sleep(void) {
  WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x40);
}

void MPU9250::ConfigAccelGyro(void) {
  // reset MPU9250, all registers to default settings
  WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80);
  delay(100); // Wait after reset
  // as per datasheet all registers are reset to 0 except WHOAMI and PWR_MGMT_1, 
  // so we assume reserved bits are 0
  // select best available clock source 
  WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
  delay(200);

  // fsync disabled, gyro bandwidth = 41Hz (with GYRO_CONFIG:fchoice = 11) 
  WriteByte(MPU9250_ADDRESS, CONFIG, 0x03);

  // output data rate = 1000Hz/5 = 200Hz
  WriteByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);

  // set gyro FS = 500dps, Fchoice = b11 (inverse of bits [1:0] = 00)
  //uint8_t c = ReadByte(MPU9250_ADDRESS, GYRO_CONFIG);
  //c &= 0x04; // Clear all bits except for reserved bit 2
  //c |= 0x08; // FS = 500dps  bits[4:3] = 01
  WriteByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x08 );

  // Set accelerometer FS = +/-2G 
  //c = ReadByte(MPU9250_ADDRESS, ACCEL_CONFIG);
  //c &= 0x07;  // Clear all bits except for reserved bits 2:0, for FS = 2G bits[4:3] = 00 
  WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);

  // set accelerometer BW = 41Hz
  //c = ReadByte(MPU9250_ADDRESS, ACCEL_CONFIG2);
  //c &= ~0xF0; // clear all bits except for reserved bits [7:4] 
  //c |=  0x03;  // accel_fchoice = 1 (inverse of bit 3), a_dlpf_cfg bits[2:0] = 011
  WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x03);

  // interrupt is active high, push-pull, 50uS pulse
  WriteByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x10);
  // Enable data ready interrupt on INT pin
  WriteByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);
  delay(100);
}


// place unit with sensor board in normal position, flat with 
// board +ve z axis pointing upwards
void MPU9250::CalibrateAccel(int numSamples){
	uint8_t buf[6];
	int16_t raw[3];
	int32_t aAccum[3] = {0,0,0};
	for (int inx = 0; inx < numSamples; inx++){
		ReadBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, buf);
		raw[0] = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
		raw[1] = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
		raw[2] = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);	
		aAccum[0] += (int32_t) raw[0];
		aAccum[1] += (int32_t) raw[1];
		aAccum[2] += (int32_t) raw[2];
		delay(5);
		}

	ax0g_ =  (int16_t)( aAccum[0] / numSamples);
	ay0g_ =  (int16_t)( aAccum[1] / numSamples);
	azp1g_ =  (int16_t)( aAccum[2] / numSamples);
    az0g_ = azp1g_ - (int16_t)(1000.0f*MPU9250_2G_SENSITIVITY);

    Serial.printf("axBias = %d\r\n", (int)ax0g_);
    Serial.printf("ayBias = %d\r\n", (int)ay0g_);
    Serial.printf("azp1g = %d\r\n", (int)azp1g_);
    Serial.printf("azBias = %d\r\n", (int)az0g_);
	}

#define MAX_EXPECTED_OFFSET_500DPS	100
	
void MPU9250::CalibrateGyro(int numSamples){
	uint8_t buf[6];
	int16_t raw[3];
	int32_t gAccum[3] = {0,0,0};
	int foundBadData;
	int numTries = 0;
	do {
		foundBadData = 0;
		for (int inx = 0; inx < numSamples; inx++){
			ReadBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, buf);
			raw[0] = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
			raw[1] = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
			raw[2] = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);	
			if ((ABS(raw[0]) > MAX_EXPECTED_OFFSET_500DPS) || (ABS(raw[1]) > MAX_EXPECTED_OFFSET_500DPS) || (ABS(raw[2]) > MAX_EXPECTED_OFFSET_500DPS)) {
				foundBadData = 1;
				break;
			}  
			gAccum[0]  += (int32_t) raw[0];
			gAccum[1]  += (int32_t) raw[1];
			gAccum[2]  += (int32_t) raw[2];
			delay(5);
			}
		} while (foundBadData && (++numTries < 10));

    if (!foundBadData) {		
		gxBias_ =  (int16_t)( gAccum[0] / numSamples);
		gyBias_ =  (int16_t)( gAccum[1] / numSamples);
		gzBias_ =  (int16_t)( gAccum[2] / numSamples);
		}

	Serial.printf("Num Tries = %d\r\n",numTries);
	Serial.printf("gxBias = %d\r\n",gxBias_);
	Serial.printf("gyBias = %d\r\n",gyBias_);
	Serial.printf("gzBias = %d\r\n",gzBias_);
	}


void MPU9250::WriteByte(uint8_t deviceAddress, uint8_t registerAddress, uint8_t d) {
  Wire.beginTransmission(deviceAddress);  
  Wire.write(registerAddress);
  Wire.write(d);           
  Wire.endTransmission();     
}

uint8_t MPU9250::ReadByte(uint8_t deviceAddress, uint8_t registerAddress){
  uint8_t d; 
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.endTransmission(false); // restart
  Wire.requestFrom(deviceAddress, (uint8_t) 1);
  d = Wire.read();
  return d;
}

int MPU9250::ReadBytes(uint8_t deviceAddress, uint8_t registerAddress,
                        uint8_t count, uint8_t * dest) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.endTransmission(false); // restart
  int cnt = 0;
  Wire.requestFrom(deviceAddress, count);
  while (Wire.available())  {
    dest[cnt++] = Wire.read();
	}
  return cnt;
}

