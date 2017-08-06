#ifndef CONFIG_H_
#define CONFIG_H_

#define KF_ZMEAS_VARIANCE       400.0f
#define KF_ZACCEL_VARIANCE      1000.0f
#define KF_ACCELBIAS_VARIANCE   1.0f

#define SLEEP_TIMEOUT_SECONDS   1200 // 20 minutes
#define SLEEP_THRESHOLD_CPS		50

#define CLIMB_THRESHOLD     50
#define ZERO_THRESHOLD	    5
#define SINK_THRESHOLD      -250

#define BATTERY_TONE_FREQHZ			400
#define CALIB_TONE_FREQHZ			800
#define MPU9250_ERROR_TONE_FREQHZ	200
#define MS5611_ERROR_TONE_FREQHZ	2500

#define IMU_DEBUG


#endif