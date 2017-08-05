// CJMCU-117 MPU9250+MS5611 circuit interface
//
// VCC  VBAT (via ferrite bead)
// GND  GND
// SCL  GPIO5
// SDA  GPIO4
// NCS  3V3 (mpu9250 i2c mode, connect to CJMCU-117 LDO regulator output)
// AD0  GND (mpu9250 i2c slave addr lsb = 0)
// INT  GPIO15
// SDO 	x
// CSB  GND (ms5611 i2c slave addr lsb = 0)
// PS   3V3 (ms5611 i2c mode, connect to CJMCU-117 LDO regulator output)

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <EEPROM.h>
#include <math.h>
#include "MahonyAHRS.h"
#include "MPU9250.h"
#include "MS5611.h"
#include "KalmanVario.h"
#include "VarioAudio.h"
#include "util.h"
#include "config.h"
#include "cct.h"
#include "nvd.h"

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
float zAccelAccumulator = 0.0f;

int pinSDA = 4;
int pinSCL = 5;
int pinDRDYInt = 15;
int pinAudio = 13;
int pinBtn = 0;

volatile int drdyCounter;
volatile int drdyFlag;
int sampleCounter;
int timeoutSeconds;

MPU9250 imu;
MS5611 baro;
KalmanVario kf;
VarioAudio audio;


int batteryVoltage(void) {
	int adcSample = 0;
	for (int inx = 0; inx < 4; inx++) {
		adcSample += analogRead(A0);
		delay(1);
		}
	adcSample /= 4;
	// potential divider with 120K and 33K to scale 4.2V down to < 1.0V for the ESP8266 ADC
	// actual measurement 0.854V with battery voltage = 4.0V => actual scale up from resistive divider = 4.0/0.854 = 4.6838
	// adc isn't very accurate either, experimental correction factor ~ 1.04, so effective scale up is 1.04 * 4.6838
	return (int) ((adcSample*1.04f*4.6838f*10.0f)/1023.0f + 0.5f); //  voltage x 10
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
	
void powerDown() {
	audio.SetFrequency(0); // switch off pwm audio 
	imu.Sleep(); // put MPU9250 in sleep mode
	ESP.deepSleep(0); // esp8266 in sleep can only recover with a reset/power cycle
	}

void setup() {
	delay(10);
	WiFi.disconnect(); 
	WiFi.mode(WIFI_OFF);
	WiFi.forceSleepBegin(); // switch off radio to minimize current draw
	delay(100); // delay(1) is required, additional delay so battery voltage can settle 
	Serial.begin(115200);
	Serial.printf("\r\nESP8266 MPU9250 MS5611 VARIO %s %s\r\n", __DATE__, __TIME__);
	Wire.begin(pinSDA, pinSCL);
	Wire.setClock(400000); // set clock frequency AFTER Wire.begin()
	audio.Config(pinAudio); 
	int bv = batteryVoltage();
	Serial.printf("battery voltage = %d.%dV\r\n", bv/10, bv%10 );
	audio_indicateBatteryVoltage(bv);

    nvd_Init();
	if (nvd.axBias == 0 && nvd.ayBias == 0 && nvd.azBias == 0) {
		audio.IndicateFault(3000);
		}
	imu.SetCalibrationParams(&nvd);
	
	drdyCounter = 0;
	drdyFlag = 0;
	pinMode(pinDRDYInt, INPUT); // connected to INT output of CJMCU-117
	attachInterrupt(digitalPinToInterrupt(pinDRDYInt), DRDYInterruptHandler, RISING);
	
	if (!imu.CheckID()) {
		Serial.printf("Unable to communicate with mpu9250, abort!\r\n");
		Serial.flush();
		audio.IndicateFault(200);
		powerDown();
		}
	imu.ConfigAccelGyro();
	// try to calibrate gyro each time on power up. if the unit is not at rest, give up
	// and use the last saved gyro offsets
	// allow a few seconds for unit to be left undisturbed so gyro can be calibrated
	pinMode(pinBtn, INPUT);
	int bCalibrateAccelerometer = 0;
	for (int inx = 0; inx < 10; inx++) {
		delay(500); 
		audio.GenerateTone(800,50);
		if (digitalRead(pinBtn) == 0) {
			delay(100);
			if (digitalRead(pinBtn) == 0) {
				bCalibrateAccelerometer = 1;
				break;
				}
			}
		}
	if (bCalibrateAccelerometer) {	
		audio.GenerateTone(1000, 3000);
		// allow time for the unit to be placed in calibration position with the accelerometer +z pointing downwards
		for (int inx = 0; inx < 50; inx++) {
			delay(200); 
			audio.GenerateTone(800,50);
			}
		Serial.printf("Calibrating accel & gyro\r\n");
		imu.CalibrateAccel();
		imu.CalibrateGyro();
		nvd_SaveCalibrationParams(imu.axBias_,imu.ayBias_,imu.azBias_,imu.gxBias_,imu.gyBias_,imu.gzBias_,NVD_CALIBRATED);
		}
	imu.CalibrateGyro();
	// indicate calibration complete
	audio.GenerateTone(800, 1000);
	delay(1000);			
	baro.Reset();
	delay(100);
	baro.Config();
	baro.AveragedSample(4); 
	baro.InitializeSampleStateMachine();

	kf.Config(KF_ZMEAS_VARIANCE, KF_ZACCEL_VARIANCE, KF_ACCELBIAS_VARIANCE, baro.zCmAvg_, 0.0f, 0.0f);
	initTime();
	zAccelAccumulator = 0.0f;
	sampleCounter = 0;
	timeoutSeconds = 0;
	}
	


void loop(){
	if ( drdyFlag ) { // new data sample ready, mpu9250 configured for 200Hz ODR => 5mS sample interval
		drdyFlag = 0;
		updateTime();
#ifdef IMU_DEBUG		
		cct_SetMarker();
#endif		
		imu.GetAccelGyroData(accel, gyro); // accelerometer samples (ax,ay,az) in milli-Gs, gyroscope samples (gx,gy,gz) in deg/second
        // CJMCU-117 board is placed upside down in the case. We arbitrarily decide that the CJMCU-117 board 
		// silkscreen +X points "forward" or "north"  (in our case, the side with the power switch), 
		// silkscreen +Y points "right" or "east", silkscreen +Z points down. This is the North-East-Down (NED) 
		// right-handed coordinate frame used in our AHRS algorithm implementation.
		// The required mapping from sensor samples to NED frame for our specific board orientation is : 
		// gxned = gx, gyned = gy, gzned = gz
		// axned = -ax, ayned = -ay, azned = -az
		imu_MahonyAHRSupdateIMU(timeDeltaSecs, gyro[0]*DEG_TO_RAD, gyro[1]*DEG_TO_RAD, gyro[2]*DEG_TO_RAD, -accel[0], -accel[1], -accel[2]);
		float gravityCompensatedAccel = imu_GravityCompensatedAccel(-accel[0], -accel[1], -accel[2], q0, q1, q2, q3);
		zAccelAccumulator += gravityCompensatedAccel; // one earth-z acceleration value computed every 5mS, accumulate

		sampleCounter++;
		if (sampleCounter >= 2) { // 10mS elapsed, this is the sampling period for MS5611, 
			sampleCounter = 0;    // alternating between pressure and temperature samples
			int zMeasurementAvailable = baro.SampleStateMachine(); // one z (altitude) sample calculated for every new pair of pressure & temperature samples
			if ( zMeasurementAvailable ) { 
				float zAccelAverage = zAccelAccumulator / 4.0f; // average earth-z acceleration over the 20mS interval between z samples
				kf.Update(baro.zCmSample_, zAccelAverage, 0.020f, &kfAltitudeCm, &kfClimbrateCps);
				zAccelAccumulator = 0.0f;
				int32_t audioCps =  kfClimbrateCps >= 0.0f ? (int32_t)(kfClimbrateCps+0.5f) : (int32_t)(kfClimbrateCps-0.5f);
				if (ABS(audioCps) > SLEEP_THRESHOLD_CPS) { // reset sleep timeout watchdog if there is significant vertical motion
					timeoutSeconds = 0;
					}
				if (timeoutSeconds >= SLEEP_TIMEOUT_SECONDS) {
					Serial.print("Timed out, put MPU9250 and ESP8266 to sleep to minimize current draw\r\n");
					Serial.flush();
					audio.IndicateFault(1000);
					powerDown();
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
			timeoutSeconds++; // 200 * 5mS = 1 second
#ifdef IMU_DEBUG
			float yaw, pitch, roll;
			imu_Quaternion2YawPitchRoll(q0,q1,q2,q3, &yaw, &pitch, &roll);
			// Pitch is positive for clockwise rotation about the +Y axis
			// Roll is positive for clockwise rotation about the +X axis
			// Yaw is positive for clockwise rotation about the +Z axis
			// Magnetometer isn't used, so yaw is initialized to 0 for the "forward" direction of the case on power up.
			Serial.printf("\r\nYaw = %d Pitch = %d Roll = %d\r\n", (int)yaw, (int)pitch, (int)roll);
			Serial.printf("Baro Alt = %d, kfAlt = %d, kfVario = %d\r\n",(int)baro.zCmSample_, (int)kfAltitudeCm, (int)kfClimbrateCps);
			Serial.printf("us = %d\r\n", elapsedUs);
#endif			
			}
		} 
	}




