#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include "config.h"
#include "nvd.h"
#include "util.h"
#include "wificonfig.h"

extern "C" {
#include <user_interface.h>
}

String szVarioClimbThresholdCps;
String szVarioZeroThresholdCps;
String szVarioSinkThresholdCps;
String szVarioCrossoverCps;

String szKFAccelVariance;
String szKFZMeasVariance;

String szBatteryToneHz;
String szUncalibratedToneHz;
String szCalibratingToneHz;
String szMPU9250ErrorToneHz;
String szMS5611ErrorToneHz;

String szSleepTimeoutMinutes;
String szGyroOffsetLimit1000DPS;

String szPageContent;

MDNSResponder mdns;
ESP8266WebServer server(80);

static void wificonfig_GeneratePage();
static void wificonfig_HandleRoot();
static void wificonfig_ReturnFail(String msg);
static void wificonfig_ReturnOK();
static void wificonfig_HandleSubmit();
static void wificonfig_HandleNotFound();
static void wificonfig_HandleDefaults();
static void wificonfig_UpdateTextFields();

void wificonfig_UpdateTextFields() {
  szVarioClimbThresholdCps = String(nvd.params.vario.climbThresholdCps);
  szVarioZeroThresholdCps = String(nvd.params.vario.zeroThresholdCps);
  szVarioSinkThresholdCps = String(nvd.params.vario.sinkThresholdCps);
  szVarioCrossoverCps = String(nvd.params.vario.crossoverCps);

  szKFAccelVariance = String(nvd.params.kf.accelVariance);
  szKFZMeasVariance = String(nvd.params.kf.zMeasVariance);

  szBatteryToneHz = String(nvd.params.alarm.batteryToneHz);
  szUncalibratedToneHz = String(nvd.params.alarm.uncalibratedToneHz);
  szCalibratingToneHz = String(nvd.params.alarm.calibratingToneHz);
  szMPU9250ErrorToneHz = String(nvd.params.alarm.mpu9250ErrorToneHz);
  szMS5611ErrorToneHz = String(nvd.params.alarm.ms5611ErrorToneHz);
  
  szGyroOffsetLimit1000DPS = String(nvd.params.misc.gyroOffsetLimit1000DPS);
  szSleepTimeoutMinutes = String(nvd.params.misc.sleepTimeoutMinutes);
  }

void wificonfig_GeneratePage() {
  szPageContent = "<!DOCTYPE HTML>\r\n<HTML><P>";
  szPageContent += "<HEAD><STYLE>input[type=number] {width: 50px;border: 2px solid red;border-radius: 4px;}</STYLE></HEAD><BODY>";
  szPageContent += "<FORM action=\"/\" method=\"post\"><P>";
  szPageContent += "<fieldset><legend>Vario</legend>";  
  szPageContent += "Climb Threshold [20 ... 100] <INPUT type=\"number\" name=\"varioClimbThreshold\" value=\"" + szVarioClimbThresholdCps + "\">cm/s<BR>";
  szPageContent += "Zero Threshold [-20 ... 20] <INPUT type=\"number\" name=\"varioZeroThreshold\" value=\"" + szVarioZeroThresholdCps + "\">cm/s<BR>";
  szPageContent += "Sink Threshold [-400 ... -100] <INPUT type=\"number\" name=\"varioSinkThreshold\" value=\"" + szVarioSinkThresholdCps + "\">cm/s<BR>";
  szPageContent += "Crossover climbrate [300 ... 700] <INPUT type=\"number\" name=\"varioCrossoverCps\" value=\"" + szVarioCrossoverCps + "\">cm/s<BR>";
  szPageContent += "</fieldset><fieldset><legend>Kalman Filter</legend>";
  szPageContent += "Acceleration Variance [100 ... 10000] <INPUT type=\"number\" name=\"kfAccelVar\" value=\"" + szKFAccelVariance + "\"><BR>";
  szPageContent += "Altitude Noise Variance [100 ... 400] <INPUT type=\"number\" name=\"kfZMeasVar\" value=\"" + szKFZMeasVariance + "\"><BR>";
  szPageContent += "</fieldset><fieldset><legend>Feedback</legend>";
  szPageContent += "Battery Tone [200 ... 4000] <INPUT type=\"number\" name=\"batteryTone\" value=\"" + szBatteryToneHz + "\">Hz<BR>";
  szPageContent += "Uncalibrated Tone [200 ... 4000] <INPUT type=\"number\" name=\"uncalibratedTone\" value=\"" + szUncalibratedToneHz + "\">Hz<BR>";
  szPageContent += "Calibrating Tone [200 ... 4000] <INPUT type=\"number\" name=\"calibratingTone\" value=\"" + szCalibratingToneHz + "\">Hz<BR>";
  szPageContent += "MPU9250 Error Tone [200 ... 4000] <INPUT type=\"number\" name=\"mpu9250ErrorTone\" value=\"" + szMPU9250ErrorToneHz + "\">Hz<BR>";
  szPageContent += "MS5611 Error Tone [200 ... 4000] <INPUT type=\"number\" name=\"ms5611ErrorTone\" value=\"" + szMS5611ErrorToneHz + "\">Hz<BR>";
  szPageContent += "</fieldset><fieldset><legend>Miscellaneous</legend>";
  szPageContent += "Sleep Timeout [5 ... 30] <INPUT type=\"number\" name=\"sleepTimeout\" value=\"" + szSleepTimeoutMinutes + "\">min<BR>";
  szPageContent += "Gyro Offset Limit [100 ... 500] <INPUT type=\"number\" name=\"gyroOffsetLimit\" value=\"" + szGyroOffsetLimit1000DPS + "\"><BR>";
  szPageContent += "</fieldset><INPUT type = \"submit\" value = \"SEND\"> ";
  szPageContent += "</P></FORM><BR><a href=\"defaults\"><BUTTON>SET DEFAULTS</BUTTON></a></BODY></HTML>";
  }

void wificonfig_HandleRoot(){
  /*
  if (server.hasArg("varioClimbThreshold") || 
		server.hasArg("varioZeroThreshold") || 
		server.hasArg("varioSinkThreshold") || 
		server.hasArg("varioCrossoverCps") ||
    server.hasArg("kfAccelVar") ||
    server.hasArg("kfZMeasVar") ||
    server.hasArg("batteryTone") ||
    server.hasArg("uncalibratedTone") ||
    server.hasArg("calibratingTone") ||
    server.hasArg("mpu9250ErrorTone") ||
    server.hasArg("ms5611ErrorTone") ||
    server.hasArg("sleepTimeout") ||
    server.hasArg("gyroOffsetLimit") 
		) {
    wificonfig_HandleSubmit();
    }
    */
  wificonfig_HandleSubmit();
  wificonfig_GeneratePage();
  server.send(200, "text/html", szPageContent);
  }

void wificonfig_ReturnFail(String msg){
  server.sendHeader("Connection", "close");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(500, "text/plain", msg + "\r\n");
  }
  

void wificonfig_HandleSubmit() {
  int bChanged = 0;
  if (server.hasArg("varioClimbThreshold")) {
    szVarioClimbThresholdCps = server.arg("varioClimbThreshold");
    bChanged = 1;
#ifdef WEBCFG_DEBUG    
    Serial.print("varioClimbThreshold : "); Serial.println(szVarioClimbThresholdCps);
#endif
    }
  if (server.hasArg("varioZeroThreshold")) {
    szVarioZeroThresholdCps = server.arg("varioZeroThreshold");
    bChanged = 1;
#ifdef WEBCFG_DEBUG    
    Serial.print("varioZeroThreshold : ");Serial.println( szVarioZeroThresholdCps);
#endif    
    }
  if (server.hasArg("varioSinkThreshold")) {
    szVarioSinkThresholdCps = server.arg("varioSinkThreshold");
    bChanged = 1;
#ifdef WEBCFG_DEBUG    
    Serial.print("varioSinkThreshold : ");Serial.println( szVarioSinkThresholdCps);
#endif
    }
  if (server.hasArg("varioCrossoverCps")) {
    szVarioCrossoverCps = server.arg("varioCrossoverCps");
    bChanged = 1;
#ifdef WEBCFG_DEBUG    
    Serial.print("varioCrossoverCps : ");Serial.println( szVarioCrossoverCps);
#endif
    }
  if (server.hasArg("kfAccelVar")) {
    szKFAccelVariance = server.arg("kfAccelVar");
    bChanged = 1;
#ifdef WEBCFG_DEBUG    
    Serial.print("kfAccelVariance : ");Serial.println( szKFAccelVariance);
#endif
    }
  if (server.hasArg("kfZMeasVar")) {
    szKFZMeasVariance = server.arg("kfZMeasVar");
    bChanged = 1;
#ifdef WEBCFG_DEBUG    
    Serial.print("kfZMeasVariance : ");Serial.println(szKFZMeasVariance);
#endif
    }

  if (server.hasArg("batteryTone")) {
    szBatteryToneHz = server.arg("batteryTone");
    bChanged = 1;
#ifdef WEBCFG_DEBUG    
    Serial.print("batteryToneHz : ");Serial.println(szBatteryToneHz);
#endif
    }
  if (server.hasArg("uncalibratedTone")) {
    szUncalibratedToneHz = server.arg("uncalibratedTone");
    bChanged = 1;
#ifdef WEBCFG_DEBUG    
    Serial.print("UncalibratedToneHz : ");Serial.println(szUncalibratedToneHz);
#endif
    }
  if (server.hasArg("calibratingTone")) {
    szCalibratingToneHz = server.arg("calibratingTone");
    bChanged = 1;
#ifdef WEBCFG_DEBUG    
    Serial.print("CalibratingToneHz : ");Serial.println(szCalibratingToneHz);
#endif
    }
  if (server.hasArg("mpu9250ErrorTone")) {
    szMPU9250ErrorToneHz = server.arg("mpu9250ErrorTone");
    bChanged = 1;
#ifdef WEBCFG_DEBUG    
    Serial.print("mpu9250ErrorToneHz : ");Serial.println(szMPU9250ErrorToneHz);
#endif
    }
  if (server.hasArg("ms5611ErrorTone")) {
    szMS5611ErrorToneHz = server.arg("ms5611ErrorTone");
    bChanged = 1;
#ifdef WEBCFG_DEBUG    
    Serial.print("ms5611ErrorToneHz : ");Serial.println(szMS5611ErrorToneHz);
#endif
    }
  if (server.hasArg("sleepTimeout")) {
    szSleepTimeoutMinutes = server.arg("sleepTimeout");
    bChanged = 1;
#ifdef WEBCFG_DEBUG    
    Serial.print("sleepTimeoutMinutes : ");Serial.println(szSleepTimeoutMinutes);
#endif
    }
  if (server.hasArg("gyroOffsetLimit")) {
    szGyroOffsetLimit1000DPS = server.arg("gyroOffsetLimit");
    bChanged = 1;
#ifdef WEBCFG_DEBUG    
    Serial.print("gyroOffsetLimit1000DPS : ");Serial.println(szGyroOffsetLimit1000DPS);
#endif
    }

  if (bChanged) {
    VARIO_PARAMS vario;
    vario.climbThresholdCps = szVarioClimbThresholdCps.toInt();
    CLAMP(vario.climbThresholdCps, VARIO_CLIMB_THRESHOLD_CPS_MIN, VARIO_CLIMB_THRESHOLD_CPS_MAX);
    vario.zeroThresholdCps = szVarioZeroThresholdCps.toInt();
    CLAMP(vario.zeroThresholdCps, VARIO_ZERO_THRESHOLD_CPS_MIN, VARIO_ZERO_THRESHOLD_CPS_MAX);
    vario.sinkThresholdCps = szVarioSinkThresholdCps.toInt();
    CLAMP(vario.sinkThresholdCps, VARIO_SINK_THRESHOLD_CPS_MIN, VARIO_SINK_THRESHOLD_CPS_MAX);
    vario.crossoverCps = szVarioCrossoverCps.toInt();
    CLAMP(vario.crossoverCps, VARIO_CROSSOVER_CPS_MIN, VARIO_CROSSOVER_CPS_MAX);

    KALMAN_FILTER_PARAMS kf;
    kf.accelVariance = szKFAccelVariance.toInt();
    CLAMP(kf.accelVariance, KF_ACCEL_VARIANCE_MIN, KF_ACCEL_VARIANCE_MAX);
    kf.zMeasVariance = szKFZMeasVariance.toInt();
    CLAMP(kf.zMeasVariance, KF_ZMEAS_VARIANCE_MIN, KF_ZMEAS_VARIANCE_MAX);

    ALARM_PARAMS alarm;
    alarm.batteryToneHz = szBatteryToneHz.toInt();
    CLAMP(alarm.batteryToneHz, VARIO_SPKR_MIN_FREQHZ, VARIO_SPKR_MAX_FREQHZ );
    alarm.uncalibratedToneHz = szUncalibratedToneHz.toInt();
    CLAMP(alarm.uncalibratedToneHz, VARIO_SPKR_MIN_FREQHZ, VARIO_SPKR_MAX_FREQHZ );
    alarm.calibratingToneHz  = szCalibratingToneHz.toInt();
    CLAMP(alarm.calibratingToneHz, VARIO_SPKR_MIN_FREQHZ, VARIO_SPKR_MAX_FREQHZ );
    alarm.mpu9250ErrorToneHz = szMPU9250ErrorToneHz.toInt();
    CLAMP(alarm.mpu9250ErrorToneHz, VARIO_SPKR_MIN_FREQHZ, VARIO_SPKR_MAX_FREQHZ );
    alarm.ms5611ErrorToneHz  = szMS5611ErrorToneHz.toInt();
    CLAMP(alarm.ms5611ErrorToneHz, VARIO_SPKR_MIN_FREQHZ, VARIO_SPKR_MAX_FREQHZ );

    MISC_PARAMS misc;
    misc.sleepTimeoutMinutes  = szSleepTimeoutMinutes.toInt();
    CLAMP(misc.sleepTimeoutMinutes, SLEEP_TIMEOUT_MINUTES_MIN, SLEEP_TIMEOUT_MINUTES_MAX );
    misc.gyroOffsetLimit1000DPS  = szGyroOffsetLimit1000DPS.toInt();
    CLAMP(misc.gyroOffsetLimit1000DPS, GYRO_OFFSET_LIMIT_1000DPS_MIN, GYRO_OFFSET_LIMIT_1000DPS_MAX);

    nvd_SaveConfigurationParams(&vario, &alarm, &kf, &misc);
    wificonfig_UpdateTextFields();
    }
  }
  
void wificonfig_HandleDefaults() {
    VARIO_PARAMS vario;
    vario.climbThresholdCps = VARIO_CLIMB_THRESHOLD_CPS_DEFAULT;
    vario.zeroThresholdCps = VARIO_ZERO_THRESHOLD_CPS_DEFAULT;
    vario.sinkThresholdCps = VARIO_SINK_THRESHOLD_CPS_DEFAULT ;
    vario.crossoverCps =  VARIO_CROSSOVER_CPS_DEFAULT;

    KALMAN_FILTER_PARAMS kf;
    kf.accelVariance = KF_ACCEL_VARIANCE_DEFAULT;
    kf.zMeasVariance = KF_ZMEAS_VARIANCE_DEFAULT;

    ALARM_PARAMS alarm;
    alarm.batteryToneHz = BATTERY_TONE_HZ_DEFAULT ;
    alarm.uncalibratedToneHz = UNCALIBRATED_TONE_HZ_DEFAULT;
    alarm.calibratingToneHz  = CALIBRATING_TONE_HZ_DEFAULT ;
    alarm.mpu9250ErrorToneHz = MPU9250_ERR_TONE_HZ_DEFAULT;
    alarm.ms5611ErrorToneHz  = MS5611_ERR_TONE_HZ_DEFAULT;

    MISC_PARAMS misc;
    misc.sleepTimeoutMinutes  = SLEEP_TIMEOUT_MINUTES_DEFAULT;
    misc.gyroOffsetLimit1000DPS  = GYRO_OFFSET_LIMIT_1000DPS_DEFAULT;

    nvd_SaveConfigurationParams(&vario,&alarm,&kf,&misc);
    wificonfig_UpdateTextFields();
    wificonfig_GeneratePage();
    server.send(200, "text/html", szPageContent);  
    }



void wificonfig_HandleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (int i = 0; i < server.args(); i++){
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
    }
  server.send(404, "text/plain", message);
  }

void wificonfig_wifiOn() {
  wifi_fpm_do_wakeup();
  wifi_fpm_close();
  delay(100);
  }

#define FPM_SLEEP_MAX_TIME 0xFFFFFFF


void wificonfig_wifiOff() {
  wifi_station_disconnect();
  wifi_set_opmode(NULL_MODE);
  wifi_set_sleep_type(MODEM_SLEEP_T);
  wifi_fpm_open();
  wifi_fpm_do_sleep(FPM_SLEEP_MAX_TIME);
  delay(100);
  }

void wificonfig_SetupAPWebServer() {
  wificonfig_wifiOn();
#ifdef WEBCFG_DEBUG    
  Serial.println("Setting up access point \"espVarioConfig\" with no password");
#endif  
  boolean result = WiFi.softAP("espVarioConfig", "");
#ifdef WEBCFG_DEBUG    
  if(result == true){
    Serial.println("AP setup OK");
    }
  else{
    Serial.println("AP setup failed!");
    }    
#endif  
  
  IPAddress myIP = WiFi.softAPIP();  
#ifdef WEBCFG_DEBUG    
  Serial.print("Access Point IP address: ");Serial.println(myIP);
#endif  
  if (mdns.begin("espvarioconfig", myIP)) {
#ifdef WEBCFG_DEBUG    
    Serial.println("MDNS responder started");
#endif  
    }

  wificonfig_UpdateTextFields();
  server.on("/", wificonfig_HandleRoot);
  server.on("/defaults", wificonfig_HandleDefaults);
  server.onNotFound(wificonfig_HandleNotFound);
  
  server.begin();
#ifdef WEBCFG_DEBUG    
  Serial.print("Connect to http://espvarioconfig.local or http://");Serial.println(myIP);
#endif
  }

void wificonfig_HandleClient() {
  server.handleClient();
  }
    
