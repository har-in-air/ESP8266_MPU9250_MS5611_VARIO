#ifndef CONFIG_H_
#define CONFIG_H_

// Kalman filter configuration
#define KF_ZMEAS_VARIANCE       400.0f
#define KF_ZACCEL_VARIANCE      1000.0f
#define KF_ACCELBIAS_VARIANCE   1.0f

// Power-down timeout. Here we power down if the
// vario does not see any climb or sink rate more than
// 50cm/sec, for 20 minutes.
#define SLEEP_TIMEOUT_SECONDS   1200 // 20 minutes
#define SLEEP_THRESHOLD_CPS		50

// vario thresholds in cm/sec for generating different
// audio tones. Between the sink threshold and the zero threshold,
// the vario is quiet
#define CLIMB_THRESHOLD     5
#define ZERO_THRESHOLD	    5
#define SINK_THRESHOLD      -100

// change these parameters based on the frequency bandwidth of the speaker

#define VARIO_MAX_FREQHZ      4000
#define VARIO_XOVER_FREQHZ    2000
#define VARIO_MIN_FREQHZ      200

#define VARIO_SINK_FREQHZ     400
#define VARIO_TICK_FREQHZ     200

// audio feedback tones
#define BATTERY_TONE_FREQHZ			400
#define CALIB_TONE_FREQHZ			800
#define MPU9250_ERROR_TONE_FREQHZ	200
#define MS5611_ERROR_TONE_FREQHZ	2500

// if you find that gyro calibration fails when you leave
// the unit undisturbed, possibly your unit has an MPU9250 device
// with a larger gyro bias. In that case try increasing this
// threshold maybe 10% at a time, until you find the calibration
// works consistently.

#define GYRO_MAX_EXPECTED_OFFSET_500DPS		100


// print useful information to the serial port for 
// verifying correct operation. Comment out to prevent
// data being spewed out continuously.
#define IMU_DEBUG


#endif