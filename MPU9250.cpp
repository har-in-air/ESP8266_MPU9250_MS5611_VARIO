#include <Arduino.h>
#include <Wire.h>
#include "MPU9250.h"
#include "VarioAudio.h"
#include "config.h"
#include "util.h"


MPU9250::MPU9250() {	
	// valid for accelerometer full scale = +/- 2G
    axBias_ = 105;
    ayBias_ = -273;
    azBias_ = -508;
    aScale_ = 1.0f/MPU9250_2G_SENSITIVITY; // accelerometer values in milli-Gs
    
    // valid for gyro full scale = 500dps
    gxBias_ = 42; 
    gyBias_ = -18; 
    gzBias_ = 73; 
    gScale_  = 1.0f/MPU9250_500DPS_SENSITIVITY; // gyroscope values in deg/second
	}

void MPU9250::GetAccelGyroData(float* pAccelData, float* pGyroData) {
	uint8_t buf[14];
	int16_t raw[3];
	ReadBytes(MPU9250_I2C_ADDRESS, ACCEL_XOUT_H, 14, buf);
	raw[0] = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
	raw[1] = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
	raw[2] = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);	
	pAccelData[0] = (float)(raw[0] - axBias_) * aScale_;
	pAccelData[1] = (float)(raw[1] - ayBias_) * aScale_;
	pAccelData[2] = (float)(raw[2] - azBias_) * aScale_;
	raw[0] = (int16_t)(((uint16_t)buf[8] << 8) | (uint16_t)buf[9]);
	raw[1] = (int16_t)(((uint16_t)buf[10] << 8) | (uint16_t)buf[11]);
	raw[2] = (int16_t)(((uint16_t)buf[12] << 8) | (uint16_t)buf[13]);	
    pGyroData[0] = (float)(raw[0] - gxBias_) * gScale_;
    pGyroData[1] = (float)(raw[1] - gyBias_) * gScale_;
    pGyroData[2] = (float)(raw[2] - gzBias_) * gScale_;
	}

	
int MPU9250::CheckID(void) {
	uint8_t whoami = ReadByte(MPU9250_I2C_ADDRESS, WHO_AM_I_MPU9250);
	return (( whoami == 0x71) ? 1 : 0);
	}
	
void MPU9250::SetCalibrationParams(NVD* pNVD) {
	axBias_ = pNVD->params.axBias;
	ayBias_ = pNVD->params.ayBias;
	azBias_ = pNVD->params.azBias;
	gxBias_ = pNVD->params.gxBias;
	gyBias_ = pNVD->params.gyBias;
	gzBias_ = pNVD->params.gzBias;
	Serial.printf("Using saved calibration parameters :\r\n");
	Serial.printf("Accel : axBias %d, ayBias %d, azBias %d\r\n",axBias_, ayBias_, azBias_);
	Serial.printf("Gyro : gxBias %d, gyBias %d, gzBias %d\r\n",gxBias_, gyBias_, gzBias_);
	}

void MPU9250::Sleep(void) {
	WriteByte(MPU9250_I2C_ADDRESS, PWR_MGMT_1, 0x40);
	}

void MPU9250::ConfigAccelGyro(void) {
	// reset MPU9250, all registers to default settings
	WriteByte(MPU9250_I2C_ADDRESS, PWR_MGMT_1, 0x80);
	delay(100); // Wait after reset
	// as per datasheet all registers are reset to 0 except WHOAMI and PWR_MGMT_1, 
	// so we assume reserved bits are 0
	// select best available clock source 
	WriteByte(MPU9250_I2C_ADDRESS, PWR_MGMT_1, 0x01);
	delay(200);

	// fsync disabled, gyro bandwidth = 41Hz (with GYRO_CONFIG:fchoice = 11) 
	WriteByte(MPU9250_I2C_ADDRESS, CONFIG, 0x03);

	// output data rate = 1000Hz/5 = 200Hz
	WriteByte(MPU9250_I2C_ADDRESS, SMPLRT_DIV, 0x04);

	// set gyro FS = 500dps, Fchoice = b11 (inverse of bits [1:0] = 00)
	WriteByte(MPU9250_I2C_ADDRESS, GYRO_CONFIG, 0x08 );

	// Set accelerometer FS = +/-2G 
	// for aFS = 2g, bits[4:3] = 00 
	WriteByte(MPU9250_I2C_ADDRESS, ACCEL_CONFIG, 0x00);

	// set accelerometer BW = 41Hz
	// accel_fchoice = 1 (inverse of bit 3), a_dlpf_cfg bits[2:0] = 011
	WriteByte(MPU9250_I2C_ADDRESS, ACCEL_CONFIG2, 0x03);

	// interrupt is active high, push-pull, 50uS pulse
	WriteByte(MPU9250_I2C_ADDRESS, INT_PIN_CFG, 0x10);
	// Enable data ready interrupt on INT pin
	WriteByte(MPU9250_I2C_ADDRESS, INT_ENABLE, 0x01);
	delay(100);
	}


// place unit so that the sensor board accelerometer +ve z axis points 
// vertically downwards. This is where the sensor z axis sees a static 
// acceleration of 1g. In this orientation the ax and ay values are 
// the offsets for a 0g environment. 
// Repeat this calibration a few times with the debug serial monitor to check the 
// consistency of the calibration offsets. The board MUST be in a 1g static acceleration 
// environment for this calibration, i.e. at rest, no vibrations etc.

#define ACCEL_NUM_AVG_SAMPLES	10
#define ACCEL_NUM_CALIB_TRIES	200

void MPU9250::CalibrateAccel(){
	uint8_t buf[6];
	int16_t ax,ay,az,ax0g,ay0g;
	int16_t az1g = 0;
	int32_t axAccum, ayAccum, azAccum;
	Serial.printf("\r\n");
	for (int cnt = 0; cnt < ACCEL_NUM_CALIB_TRIES; cnt++) {
		axAccum = ayAccum = azAccum = 0;
		for (int inx = 0; inx < ACCEL_NUM_AVG_SAMPLES; inx++){
			ReadBytes(MPU9250_I2C_ADDRESS, ACCEL_XOUT_H, 6, buf);
			ax = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
			ay = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
			az = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);	
			axAccum += (int32_t) ax;
			ayAccum += (int32_t) ay;
			azAccum += (int32_t) az;
			delay(5);
			}
		ax = (int16_t)(axAccum / ACCEL_NUM_AVG_SAMPLES);
		ay = (int16_t)(ayAccum / ACCEL_NUM_AVG_SAMPLES);
		az = (int16_t)(azAccum / ACCEL_NUM_AVG_SAMPLES);
		if (az > az1g) {
			az1g = az;
			ax0g = ax;
			ay0g = ay;
			}
		Serial.printf("%03d/%3d : ax %d  ay %d  az %d\r\n",cnt, ACCEL_NUM_CALIB_TRIES, ax, ay, az);
		}

	axBias_ = ax0g;
	ayBias_ = ay0g;
    azBias_ = az1g - (int16_t)(1000.0f*MPU9250_2G_SENSITIVITY);

    Serial.printf("axBias = %d\r\n", (int)axBias_);
    Serial.printf("ayBias = %d\r\n", (int)ayBias_);
    Serial.printf("azBias = %d\r\n", (int)azBias_);
	}

#define GYRO_NUM_CALIB_SAMPLES			50
	
int MPU9250::CalibrateGyro(void){
	uint8_t buf[6];
	int16_t gx,gy,gz;
	int32_t gxAccum, gyAccum, gzAccum;
	int foundBadData;
	int numTries = 1;
	do {
		delay(500);
		foundBadData = 0;
		gxAccum = gyAccum = gzAccum = 0;
		for (int inx = 0; inx < GYRO_NUM_CALIB_SAMPLES; inx++){
			ReadBytes(MPU9250_I2C_ADDRESS, GYRO_XOUT_H, 6, buf);
			gx = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
			gy = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
			gz = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);	
			// if a larger than expected gyro bias is measured, assume the unit was disturbed and try again after a short delay, upto 10 times
			if ((ABS(gx) > GYRO_MAX_EXPECTED_OFFSET_500DPS) || (ABS(gy) > GYRO_MAX_EXPECTED_OFFSET_500DPS) || (ABS(gz) > GYRO_MAX_EXPECTED_OFFSET_500DPS)) {
				foundBadData = 1;
				// generate a low tone pulse each time calibration fails. If you hear this when the unit is undisturbed,
				// you probably need to increase GYRO_MAX_EXPECTED_OFFSET_500DPS. 
				audio.GenerateTone(200, 300); 
				break;
				}  
			gxAccum  += (int32_t) gx;
			gyAccum  += (int32_t) gy;
			gzAccum  += (int32_t) gz;
			delay(5);
			}
		} while (foundBadData && (++numTries < 10));

	// update gyro biases only if calibration succeeded, else use the last saved values from flash memory. Valid scenario for
	// gyro calibration failing is when you turn on the unit while flying. So not a big deal.
    if (!foundBadData) {		
		gxBias_ =  (int16_t)( gxAccum / GYRO_NUM_CALIB_SAMPLES);
		gyBias_ =  (int16_t)( gyAccum / GYRO_NUM_CALIB_SAMPLES);
		gzBias_ =  (int16_t)( gzAccum / GYRO_NUM_CALIB_SAMPLES);		
		}

	Serial.printf("Num Tries = %d\r\n",numTries);
	Serial.printf("gxBias = %d\r\n",gxBias_);
	Serial.printf("gyBias = %d\r\n",gyBias_);
	Serial.printf("gzBias = %d\r\n",gzBias_);
	return (foundBadData ? 0 : 1);
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

