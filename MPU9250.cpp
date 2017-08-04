#include <Arduino.h>
#include <Wire.h>
#include "MPU9250.h"


MPU9250::MPU9250() {
	// run Wire.begin() in the main app as there could be other sensors (Ms5611 etc)
	
	// valid for FS = +/- 2G
    ax0g_ = 66;
    ay0g_ = -185;
    azp1g_ = 15620;
    az0g_ = azp1g_ - (int16_t)(1000.0f*MPU9250_2G_SENSITIVITY);
    aScale_ = 1.0f/MPU9250_2G_SENSITIVITY; // accelerometer values in milli-Gs
    
    // valid for FS = 250dps
    gxBias_ = 82; 
    gyBias_ = -36; 
    gzBias_ = 111; 
    gScale_  = 1.0f/MPU9250_250DPS_SENSITIVITY; // gyroscope values in deg/second
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
	

void MPU9250::Sleep() {
  WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x40);
}

void MPU9250::ConfigGyroAccel(int aFS, int gFS) {
  // set reset bit bit (7), all registers in default settings
  WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80);
  delay(100); // Wait for all registers to reset

  // Get stable time source
  // Auto select clock source to be PLL gyroscope reference if ready else
  WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
  delay(200);

  // Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz,
  // respectively;
  // minimum delay time for this setting is 5.9 ms, which means sensor fusion
  // update rates cannot be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  WriteByte(MPU9250_ADDRESS, CONFIG, 0x03);

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  // Use a 200 Hz rate; a rate consistent with the filter update rate
  // determined inset in CONFIG above.
  WriteByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are
  // left-shifted into positions 4:3

  uint8_t c = ReadByte(MPU9250_ADDRESS, GYRO_CONFIG);
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x02; // Clear Fchoice bits [1:0]
  c = c & ~0x18; // Clear AFS bits [4:3]
  c = c | (GFS_250DPS << 3); // Set full scale range for the gyro
  // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of
  // GYRO_CONFIG
  // c =| 0x00;
  WriteByte(MPU9250_ADDRESS, GYRO_CONFIG, c );

  // Set accelerometer full-scale range configuration
  c = ReadByte(MPU9250_ADDRESS, ACCEL_CONFIG);
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | (AFS_2G << 3); // Set full scale range for the accelerometer
  WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, c);

  c = ReadByte(MPU9250_ADDRESS, ACCEL_CONFIG2);
  c &= ~0x0F; 
  c |=  0x0b;  // Set accel_fchoice_b (bit 3) and bandwidth to 41 Hz
  WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c);
  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
  // but all these rates are further reduced by a factor of 5 to 200 Hz because
  // of the SMPLRT_DIV setting


  // interrupt is active high, push-pull, pulse (not latched), cleared on any data read
  WriteByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x10);
  // Enable data ready interrupt
  WriteByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);
  delay(100);
}


void MPU9250::CalibrateGyroAccel(){
	uint8_t data[12];
	int inx, packet_count, fifo_count;

	// reset device
	// Write a one to bit 7 reset bit; toggle reset device
	WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80);
	delay(100);

	// get stable time source; Auto select clock source to be PLL gyroscope
	// reference if ready else use the internal oscillator, bits 2:0 = 001
	WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
	WriteByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
	delay(200);

	// Configure device for bias calculation
	// Disable all interrupts
	WriteByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);
	// Disable FIFO
	WriteByte(MPU9250_ADDRESS, FIFO_EN, 0x00);
	// Turn on internal clock source
	WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);
	// Disable I2C master
	WriteByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00);
	// Disable FIFO and I2C master modes
	WriteByte(MPU9250_ADDRESS, USER_CTRL, 0x00);
	// Reset FIFO and DMP
	WriteByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);
	delay(15);

	// Configure MPU6050 gyro and accelerometer for bias calculation
	// Set low-pass filter to 188 Hz
	WriteByte(MPU9250_ADDRESS, CONFIG, 0x01);
	// Set sample rate to 1 kHz
	WriteByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);
	// Set gyro full-scale to 250 degrees per second, maximum sensitivity
	WriteByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);
	// Set accelerometer full-scale to 2 g, maximum sensitivity
	WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	WriteByte(MPU9250_ADDRESS, USER_CTRL, 0x40);  // Enable FIFO
	// Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in
	// MPU-9150)
	WriteByte(MPU9250_ADDRESS, FIFO_EN, 0x78);
	delay(40);  // accumulate 40 samples in 40 milliseconds = 480 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	// Disable gyro and accelerometer sensors for FIFO
	WriteByte(MPU9250_ADDRESS, FIFO_EN, 0x00);
	// Read FIFO sample count
	ReadBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]);
	fifo_count = (int)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
	// How many sets of full gyro and accelerometer data for averaging
	packet_count = fifo_count/12;
	Serial.printf("Number packets in FIFO = %d\r\n", packet_count);
	
	int32_t gAccum[3] = {0,0,0};
	int32_t aAccum[3] = {0,0,0};
	int16_t accel_temp[3];
	int16_t gyro_temp[3];
	for (inx = 0; inx < packet_count; inx++){
		ReadBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]);
		accel_temp[0] = (int16_t) (((uint16_t)data[0] << 8) | (uint16_t)data[1]);
		accel_temp[1] = (int16_t) (((uint16_t)data[2] << 8) | (uint16_t)data[3]);
		accel_temp[2] = (int16_t) (((uint16_t)data[4] << 8) | (uint16_t)data[5]);

		aAccum[0] += (int32_t) accel_temp[0];
		aAccum[1] += (int32_t) accel_temp[1];
		aAccum[2] += (int32_t) accel_temp[2];

		gyro_temp[0]  = (int16_t) (((uint16_t)data[6] << 8) | (uint16_t)data[7]);
		gyro_temp[1]  = (int16_t) (((uint16_t)data[8] << 8) | (uint16_t)data[9]);
		gyro_temp[2]  = (int16_t) (((uint16_t)data[10] << 8) | (uint16_t)data[11]);

		gAccum[0]  += (int32_t) gyro_temp[0];
		gAccum[1]  += (int32_t) gyro_temp[1];
		gAccum[2]  += (int32_t) gyro_temp[2];
		}

	ax0g_ =  (int16_t)( aAccum[0] / packet_count);
	ay0g_ =  (int16_t)( aAccum[1] / packet_count);
	azp1g_ =  (int16_t)( aAccum[2] / packet_count);
    az0g_ = azp1g_ - (int16_t)(1000.0f*MPU9250_2G_SENSITIVITY);

    Serial.printf("Accelerometer Calibration Data\r\n");
    Serial.printf("axBias = %d\r\n", (int)ax0g_);
    Serial.printf("ayBias = %d\r\n", (int)ay0g_);
    Serial.printf("azBias = %d\r\n", (int)az0g_);
	
	gxBias_ =  (int16_t)( gAccum[0] / packet_count);
	gyBias_ =  (int16_t)( gAccum[1] / packet_count);
	gzBias_ =  (int16_t)( gAccum[2] / packet_count);

    Serial.printf("Gyro Calibration Data\r\n");
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

