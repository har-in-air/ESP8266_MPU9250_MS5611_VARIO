#ifndef CONFIG_H_
#define CONFIG_H_

////////////////////////////////////////////////////////////////////
// WEB CONFIGURATION PARAMETER DEFAULTS AND LIMITS

// vario thresholds in cm/sec for generating different
// audio tones. Between the sink threshold and the zero threshold,
// the vario is quiet

#define VARIO_CLIMB_THRESHOLD_CPS_DEFAULT  50
#define VARIO_CLIMB_THRESHOLD_CPS_MIN   20
#define VARIO_CLIMB_THRESHOLD_CPS_MAX   100

#define VARIO_ZERO_THRESHOLD_CPS_DEFAULT  5
#define VARIO_ZERO_THRESHOLD_CPS_MIN    -20
#define VARIO_ZERO_THRESHOLD_CPS_MAX    20

#define VARIO_SINK_THRESHOLD_CPS_DEFAULT  -200
#define VARIO_SINK_THRESHOLD_CPS_MIN    -400
#define VARIO_SINK_THRESHOLD_CPS_MAX    -100


// When generating climbtones, the vario allocates most of the speaker 
// frequency bandwidth to climbrates below this crossover threshold 
// so you have more frequency discrimination. So set the crossover threshold 
// to the average thermal core climbrate you expect for the site and conditions.
#define VARIO_CROSSOVER_CPS_DEFAULT     400
#define VARIO_CROSSOVER_CPS_MIN         300
#define VARIO_CROSSOVER_CPS_MAX         800

// Kalman filter configuration
#define KF_ACCEL_VARIANCE_DEFAULT     1000
#define KF_ACCEL_VARIANCE_MIN       100
#define KF_ACCEL_VARIANCE_MAX       10000

#define KF_ZMEAS_VARIANCE_DEFAULT    300
#define KF_ZMEAS_VARIANCE_MIN        100
#define KF_ZMEAS_VARIANCE_MAX        400

// Sleep timeout. The vario will go into sleep mode
// if it does not detect climb or sink rates more than
// the specified threshold, for the specified minutes.
// You can only exit the sleep mode by power cycling the unit.
#define SLEEP_TIMEOUT_MINUTES_DEFAULT   15
#define SLEEP_TIMEOUT_MINUTES_MIN       5
#define SLEEP_TIMEOUT_MINUTES_MAX       30

// audio feedback tones
#define BATTERY_TONE_HZ_DEFAULT       400
#define CALIBRATING_TONE_HZ_DEFAULT   800
#define UNCALIBRATED_TONE_HZ_DEFAULT  2000
#define MPU9250_ERR_TONE_HZ_DEFAULT   200 
#define MS5611_ERR_TONE_HZ_DEFAULT    2500

// if you find that gyro calibration fails when you leave
// the unit undisturbed, possibly your unit has an MPU9250 device
// with a larger gyro bias. In that case try increasing this
// limit  until you find the calibration works consistently.

#define GYRO_OFFSET_LIMIT_1000DPS_DEFAULT   	50
#define GYRO_OFFSET_LIMIT_1000DPS_MIN       	25
#define GYRO_OFFSET_LIMIT_1000DPS_MAX		      200


///////////////////////////////////////////////////////////////////////////////
// COMPILED CONFIGURATION PARAMETERS ( cannot be changed with web configuration )

// change these parameters based on the frequency bandwidth of the speaker
#define VARIO_SPKR_MIN_FREQHZ      	200
#define VARIO_SPKR_MAX_FREQHZ       3200

// Three octaves (2:1) of frequency for climbrates below crossoverCps,
// and one octave of frequency for climbrates above crossoverCps.
// This gives you more frequency discrimination for climbrates below
// crossoverCps
#define VARIO_CROSSOVER_FREQHZ    	1600

// uncomment this if you want the current beep/tone to be interrupted and
// a new tone generated when there is a 'significant' change in climb/sink rate
// this will give you faster apparent vario response, but could also be 
// confusing/irritating if you are in choppy air
//#define VARIO_INTERRUPT_BEEPS

// this is the 'significant change' threshold that is checked when 
// VARIO_INTERRUPT_BEEPS is enabled
#define VARIO_DISCRIMINATION_THRESHOLD_CPS    25

#define KF_ACCELBIAS_VARIANCE   1.0f

#define SLEEP_THRESHOLD_CPS    50

// print debug information to the serial port for different code modules

// these #defines can be left uncommented after debugging, as the enclosed
// debug prints do not appear in the critical run-time loop
#define MAIN_DEBUG
#define KF_DEBUG
#define VARIO_DEBUG
#define NVD_DEBUG
#define MPU9250_DEBUG
#define MS5611_DEBUG
#define WEBCFG_DEBUG

// !! ensure these #defines are commented out after debugging, as the 
// enclosed debug prints are in the critical run-time loop.
//#define IMU_DEBUG
//#define CCT_DEBUG

#endif
