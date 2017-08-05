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

#define IMU_DEBUG


#endif