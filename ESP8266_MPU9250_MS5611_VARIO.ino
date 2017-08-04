// CJMCU-117 interface
// VCC  VBAT
// GND  GND
// SDO 	x
// INT  GPIO15
// SCL  GPIO5
// SDA  GPIO4
// PS   3V3 (ms5611 i2c mode). Take the 3V3 connection from the LDO regulator output on the CJMCU-117
// CSB  GND (ms5611 i2c slave addr lsb = 0)
// NCS  3V3 (mpu9250 i2c mode)
// AD0  GND (mpu9250 i2c slave address = 0x68)

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <math.h>
#include "MahonyAHRS.h"
#include "MPU9250.h"
#include "MS5611.h"
#include "KalmanVario.h"
#include "VarioAudio.h"
#include "util.h"
#include "config.h"

extern "C" {
#include "user_interface.h"
}

uint32_t timePreviousUs;
uint32_t timeNowUs;
float timeDeltaSecs;

float accel[3];
float gyro[3];
float kfAltitudeCm = 0.0f;
float kfClimbrateCps  = 0.0f;
float accelAccumulator = 0.0f;

int pinSDA = 4;
int pinSCL = 5;
int pinDRDYInt = 15;

volatile int drdyCounter;
volatile int drdyFlag;
int sampleCounter;
int timeoutSeconds;

MPU9250 imu;
MS5611 baro;
KalmanVario kf;
VarioAudio audio;

#ifdef IMU_DEBUG
static uint32_t cctMarker;
#define  cctTicksPerUs 80 // 80MHz clock
#define SystemCoreClockHz 80000000


static inline uint32_t cct_ccount(void) {
  uint32_t r;
  asm volatile ("rsr %0, ccount" : "=r"(r));
  return r;
  }

void cct_DelayUs(uint32_t us) {
  volatile uint32_t waitCycleCount = (cctTicksPerUs * us ) + cct_ccount();
  do  {} while (cct_ccount()  < waitCycleCount);
  }


uint32_t  cct_IntervalUs(uint32_t before, uint32_t after) {
  return  (before <= after ?
    ((after - before)+cctTicksPerUs/2)/cctTicksPerUs :
    (after + (0xFFFFFFFF - before) + cctTicksPerUs/2)/cctTicksPerUs);
}

float  cct_IntervalSecs(uint32_t before, uint32_t after) {
  return  (before <= after ?
    (float)(after - before)/(float)SystemCoreClockHz :
    (float)(after + (0xFFFFFFFF - before))/(float)SystemCoreClockHz);
}

void cct_SetMarker(void) {
  cctMarker = cct_ccount();
}

uint32_t cct_ElapsedTimeUs(void) {
  uint32_t now = cct_ccount();
  return  (cctMarker <= now ?
    ((now - cctMarker)+cctTicksPerUs/2)/cctTicksPerUs :
    (now + (0xFFFFFFFF - cctMarker) + cctTicksPerUs/2)/cctTicksPerUs);
}

#endif

int batteryVoltage(void) {
	int bv = 0;
	for (int inx = 0; inx < 4; inx++) {
		bv += analogRead(A0);
		delay(1);
		}
	bv /= 4;
	// potential divider with 120K and 33K to scale 4.2V down to < 1.0V for the ESP8266 ADC
	// actual measurement 0.854V with battery voltage = 4.0V => actual scale up from resistive divider = 4.0/0.854 = 4.6838
	// adc isn't very accurate either, experimental correction factor ~ 1.04, so effective scale up is 1.04 * 4.6838 = 4.8712
	return (int) (bv*48.712f/1023.0f + 0.5f); //  voltage x 10
	}  

void audio_indicateBatteryVoltage(int bv) {
    int numBeeps;
    if (bv >= 40) numBeeps = 5;
    else
    if (bv >= 39) numBeeps = 4;
    else
    if (bv >= 37) numBeeps = 3;
    else
    if (bv >= 36) numBeeps = 2;
    else  numBeeps = 1;
    while (numBeeps--) {
        audio.GenerateTone(1000, 300);
        delay(300);
        }
    }  

void setup() {
	delay(10);
	WiFi.disconnect(); 
	WiFi.mode(WIFI_OFF);
	WiFi.forceSleepBegin(); // turn off radio to minimize current draw
	delay(100); // delay(1) is required, delay some more so battery voltage can settle 
	Serial.begin(115200);
	Serial.printf("\r\nESP8266 MPU9250 MS5611 VARIO %s %s\r\n", __DATE__, __TIME__);
	Wire.begin(pinSDA, pinSCL);
	Wire.setClock(400000); // set clock frequency AFTER Wire.begin()!!

	audio.Config(13); // generate square wave audio tones on GPIO13
	int bv = batteryVoltage();
	Serial.printf("battery voltage = %d.%dV\r\n", bv/10, bv%10 );
	audio_indicateBatteryVoltage(bv);
	 
	drdyCounter = 0;
	drdyFlag = 0;
	pinMode(pinDRDYInt, INPUT);
	attachInterrupt(digitalPinToInterrupt(pinDRDYInt), DRDYInterruptHandler, RISING);
	
	if (!imu.CheckID()) {
		Serial.printf("Unable to communicate with mpu9250, abort!\r\n");
		Serial.flush();
		audio.IndicateFault(200);
		abort();
		}
	imu.ConfigAccelGyro();
	delay(3000); // allow time for unit to be left at rest so gyro can be calibrated
#ifdef IMU_CALIBRATE	
    // to calibrate the accelerometer, the sensor board needs to be flat, with the MPU9250 +z axis
	// pointing up.
	imu.CalibrateAccel(50);
#endif
	// try to calibrate gyro each time on power up. if the unit is not at rest, give up
	// and use the default offsets that were computed offline.
	Serial.printf("Calibrating gyro\r\n");
	imu.CalibrateGyro(50);
	baro.Reset();
	delay(100);
	baro.Config();
	baro.AveragedSample(4);
	kf.Config(KF_ZMEAS_VARIANCE, KF_ZACCEL_VARIANCE, KF_ACCELBIAS_VARIANCE, baro.zCmAvg_, 0.0f, 0.0f);
 
	baro.InitializeSampleStateMachine();
	initTime();
	accelAccumulator = 0.0f;
	sampleCounter = 0;
	timeoutSeconds = 0;
	}
	
void DRDYInterruptHandler() {
    drdyFlag = 1;
	drdyCounter++;
	}	

void initTime() {
	timeNowUs = timePreviousUs = micros();
	}

void updateTime(){
	timeNowUs = micros();
	timeDeltaSecs = ((timeNowUs - timePreviousUs) / 1000000.0f);
	timePreviousUs = timeNowUs;
	}


void loop(){
	if ( drdyFlag ) { // new data sample ready, mpu9250 configured for 200Hz ODR => 5mS sample interval
		drdyFlag = 0;
		updateTime();
#ifdef IMU_DEBUG		
		cct_SetMarker();
#endif		
		imu.GetAccelGyroData(accel, gyro); // get accelerometer samples (ax,ay,az) in milli-Gs, gyroscope samples (gx,gy,gz) in deg/second
        // CJMCU-117 board is placed upside down in the case. Assume the power switch side of the case points forward (north, +X), 
		// right is east (+Y), down is +Z  in the  (North-East-Down) right-handed coordinate frame we are using in the AHRS algorithm.
		// Required mapping from sensor samples to NED frame is : gxned = gx, gyned = gy, gzned = gz, axned = -ax, ayned = -ay, azned = -az
		imu_MahonyAHRSupdateIMU(timeDeltaSecs, gyro[0]*DEG_TO_RAD, gyro[1]*DEG_TO_RAD, gyro[2]*DEG_TO_RAD, -accel[0], -accel[1], -accel[2]);
		float gravityCompensatedAccel = imu_GravityCompensatedAccel(-accel[0], -accel[1], -accel[2], q0, q1, q2, q3);
		accelAccumulator += gravityCompensatedAccel; // one sample every 5mS, accumulate

		sampleCounter++;
		if (sampleCounter >= 2) { // 10mS sampling period for MS5611, alternating between pressure sample and temperature samples
			sampleCounter = 0;
			int zMeasurementAvailable = baro.SampleStateMachine(); // one altitude sample calculated for every new pair of pressure & temperature samples
			if ( zMeasurementAvailable ) { 
				float avgAccel = accelAccumulator / 4.0f; // average acceleration over 20mS interval between new z samples
				kf.Update(baro.zCmSample_,avgAccel, 0.020f, &kfAltitudeCm, &kfClimbrateCps);
				accelAccumulator = 0.0f;
				int32_t audioCps =  kfClimbrateCps >= 0.0f ? (int32_t)(kfClimbrateCps+0.5f) : (int32_t)(kfClimbrateCps-0.5f);
				if (ABS(audioCps) > SLEEP_THRESHOLD_CPS) { // reset sleep timeout counter if there is significant vertical movement
					timeoutSeconds = 0;
					}
				if (timeoutSeconds >= SLEEP_TIMEOUT_SECONDS) {
					Serial.print("Timeout without activity, put MPU9250 and ESP8266 to sleep to minimize current draw\r\n");
					Serial.flush();
					audio.IndicateFault(1000);
					audio.SetFrequency(0);
					imu.Sleep(); // put MPU9250 in sleep mode
					ESP.deepSleep(0); // esp8266 in sleep can only recover with a reset/power cycle
					}  	
				CLAMP(audioCps, -1000, 1000); // clamp climbrate to +/- 10m/sec
				audio.VarioBeep(audioCps);                
				}
			}
#ifdef IMU_DEBUG			
		uint32_t elapsedUs =  cct_ElapsedTimeUs(); // check  worst case time  taken in reading and processing the data, should be less than 5mS
#endif
		if (drdyCounter >= 200) {
			drdyCounter = 0;
			timeoutSeconds++; // 200 x 5mS = 1 second
#ifdef IMU_DEBUG
			float yaw, pitch, roll;
			imu_Quaternion2YawPitchRoll(q0,q1,q2,q3, &yaw, &pitch, &roll);
			Serial.printf("\r\nYaw = %d Pitch = %d Roll = %d\r\n", (int)yaw, (int)pitch, (int)roll);
			Serial.printf("Baro Alt = %d, kfAlt = %d, kfVario = %d\r\n",(int)baro.zCmSample_, (int)kfAltitudeCm, (int)kfClimbrateCps);
			Serial.printf("us = %d\r\n", elapsedUs);
#endif			
			}
		} 
	}




