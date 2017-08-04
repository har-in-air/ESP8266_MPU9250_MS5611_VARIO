#include <Arduino.h>
#include "Wire.h"
#include "MS5611.h"


MS5611::MS5611() {}

void MS5611::Config(void) {
	paSample_ = 0.0f;
	zCmSample_ = 0.0f;
	celsiusSample_ = 0;
	zCmAvg_ = 0.0f;

	ReadCoefficients();
    //Serial.printf("\r\nCalib Coeffs : %d %d %d %d %d %d\r\n",cal_[0],cal_[1],cal_[2],cal_[3],cal_[4],cal_[5]);
    tref_ = ((int64_t)cal_[4])<<8;
    offT1_ = ((int64_t)cal_[1])<<16;
    sensT1_ = ((int64_t)cal_[0])<<15;
	}




#if 0

#define MAX_TEST_SAMPLES    100
extern char gszBuf[];
static float pa[MAX_TEST_SAMPLES];
static float z[MAX_TEST_SAMPLES];

void MS5611::Test(int nSamples) {
	int n;
    float paMean, zMean, zVariance, paVariance;
    paMean = 0.0f;
    zMean = 0.0f;
    paVariance = 0.0f;
    zVariance = 0.0f;
    for (n = 0; n < nSamples; n++) {
	    TriggerTemperatureSample();
	    delay(MS5611_SAMPLE_PERIOD_MS);
	    D2_ = ReadSample();
	    CalculateTemperatureCx10();
		TriggerPressureSample();
		delay(MS5611_SAMPLE_PERIOD_MS);
		D1_ = ReadSample();
		pa[n] = CalculatePressurePa();
        z[n] =  Pa2Cm(pa[n]);
        paMean += pa[n];
        zMean += z[n];
        }
    paMean /= nSamples;
    zMean /= nSamples;
    Serial.printf("paMean = %dPa, zMean = %dcm\r\n",(int)paMean,(int)zMean);
    for (n = 0; n < nSamples; n++) {
        paVariance += (pa[n]-paMean)*(pa[n]-paMean);
        zVariance += (z[n]-zMean)*(z[n]-zMean);
        //Serial.printf("%d %d\r\n",(int)pa[n],(int)z[n]);
       }
    paVariance /= (nSamples-1);
    zVariance /= (nSamples-1);
    Serial.printf("\r\npaVariance %d  zVariance %d\r\n",(int)paVariance, (int)zVariance);
	}
#endif

void MS5611::AveragedSample(int nSamples) {
	int32_t tc,tAccum,n;
    float pa,pAccum;
	pAccum = 0.0f;
    tAccum = 0;
	n = 2;
    while (n--) {
    	TriggerTemperatureSample();
    	delay(MS5611_SAMPLE_PERIOD_MS);
		D2_ = ReadSample();
		TriggerPressureSample();
		delay(MS5611_SAMPLE_PERIOD_MS);
		D1_ = ReadSample();
		}
	
	n = nSamples;
    while (n--) {
    	TriggerTemperatureSample();
    	delay(MS5611_SAMPLE_PERIOD_MS);
		D2_ = ReadSample();
		CalculateTemperatureCx10();
		TriggerPressureSample();
		delay(MS5611_SAMPLE_PERIOD_MS);
		D1_ = ReadSample();
		pa = CalculatePressurePa();
		pAccum += pa;
		tAccum += tempCx100_;
		}
	tc = tAccum/nSamples;
	celsiusSample_ = (tc >= 0 ?  (tc+50)/100 : (tc-50)/100);
	paSample_ = (pAccum+nSamples/2)/nSamples;
	zCmAvg_ = zCmSample_ = Pa2Cm(paSample_);
#if 1
   Serial.printf("Tavg : %dC\r\n",celsiusSample_);
   Serial.printf("Pavg : %dPa\r\n",(int)paSample_);
   Serial.printf("Zavg : %dcm\r\n",(int)zCmAvg_);
#endif

	}
	
	

/// Fast Lookup+Interpolation method for converting pressure readings to altitude readings.
#include "pztbl.h"

float MS5611::Pa2Cm(float paf)  {
   	int32_t pa,inx,pa1,z1,z2;
    float zf;
    pa = (int32_t)(paf);

   	if (pa > PA_INIT) {
      	zf = (float)(gPZTbl[0]);
      	}
   	else {
      	inx = (PA_INIT - pa)>>10;
      	if (inx >= PZLUT_ENTRIES-1) {
         	zf = (float)(gPZTbl[PZLUT_ENTRIES-1]);
         	}
      	else {
         	pa1 = PA_INIT - (inx<<10);
         	z1 = gPZTbl[inx];
         	z2 = gPZTbl[inx+1];
         	zf = (float)(z1) + ( ((float)pa1-paf)*(float)(z2-z1))/1024.0f;
         	}
      	}
   	return zf;
   	}

void MS5611::CalculateTemperatureCx10(void) {
	dT_ = (int64_t)D2_ - tref_;
	tempCx100_ = 2000 + ((dT_*((int32_t)cal_[5]))>>23);
	}


float MS5611::CalculatePressurePa(void) {
	float pa;
    int64_t offset, sens,offset2,sens2,t2;
	offset = offT1_ + ((((int64_t)cal_[3])*dT_)>>7);
	sens = sensT1_ + ((((int64_t)cal_[2])*dT_)>>8);
    if (tempCx100_ < 2000) { // correction for temperature < 20C
        t2 = ((dT_*dT_)>>31); 
        offset2 = (5*(tempCx100_-2000)*(tempCx100_-2000))/2;
        sens2 = offset2/2;
        } 
    else {
        t2 = 0;
        sens2 = 0;
        offset2 = 0;
        }
    tempCx100_ -= t2;
    offset -= offset2;
    sens -= sens2;
	pa = (((float)((int64_t)D1_ * sens))/2097152.0f - (float)offset) / 32768.0f;
	return pa;
	}


/// Trigger a pressure sample with max oversampling rate
void MS5611::TriggerPressureSample(void) {
	Wire.beginTransmission(MS5611_I2C_ADDRESS);  
	Wire.write(MS5611_CONVERT_D1 | MS5611_ADC_4096);  //  pressure conversion, max oversampling
	Wire.endTransmission();  
   }

/// Trigger a temperature sample with max oversampling rate
void MS5611::TriggerTemperatureSample(void) {
	Wire.beginTransmission(MS5611_I2C_ADDRESS);  
	Wire.write(MS5611_CONVERT_D2 | MS5611_ADC_4096);   //  temperature conversion, max oversampling
	Wire.endTransmission();  
   }

uint32_t MS5611::ReadSample(void)	{
	Wire.beginTransmission(MS5611_I2C_ADDRESS);  // Initialize the Tx buffer
	Wire.write(0x00);                        // Put ADC read command in Tx buffer
	Wire.endTransmission(false);        // Send the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(MS5611_I2C_ADDRESS, 3);     // Read three bytes from slave PROM address 
	int inx = 0;
	uint8_t buf[3];
	while (Wire.available()) {
        buf[inx++] = Wire.read(); 
		}          
	uint32_t w = (((uint32_t)buf[0])<<16) | (((uint32_t)buf[1])<<8) | (uint32_t)buf[2];
	return w;
   }


void MS5611::InitializeSampleStateMachine(void) {
   TriggerTemperatureSample();
   sensorState = MS5611_READ_TEMPERATURE;
   }

int MS5611::SampleStateMachine(void) {
   if (sensorState == MS5611_READ_TEMPERATURE) {
      D2_ = ReadSample();
      TriggerPressureSample();
      //DBG_1(); // turn on the debug pulse for timing the critical computation
      CalculateTemperatureCx10();
      //celsiusSample_ = (tempCx100_ >= 0? (tempCx100_+50)/100 : (tempCx100_-50)/100);
      paSample_ = CalculatePressurePa();
	  zCmSample_ = Pa2Cm(paSample_);
      //DBG_0();
	  sensorState = MS5611_READ_PRESSURE;
      return 1;  // 1 => new altitude sample is available
      }
   else
   if (sensorState == MS5611_READ_PRESSURE) {
      D1_ = ReadSample();
      TriggerTemperatureSample();
      sensorState = MS5611_READ_TEMPERATURE;
      return 0; // 0 => intermediate state
      }
   return 0;    
   }

void MS5611::Reset() {
	Wire.beginTransmission(MS5611_I2C_ADDRESS);  
	Wire.write(MS5611_RESET);                
	Wire.endTransmission();                  
    }
   
	
void MS5611::ReadCoefficients(void)        {
    for (int inx = 0; inx < 6; inx++) {
		Wire.beginTransmission(MS5611_I2C_ADDRESS); 
        Wire.write(0xA2 + inx*2); // skip the factory data in addr A0, and the checksum at last addr
        Wire.endTransmission(false); // restart
        Wire.requestFrom(MS5611_I2C_ADDRESS, 2); 
		int cnt = 0;
		uint8_t buf[2];
		while (Wire.available()) {
			buf[cnt++] = Wire.read(); 
			}  
		cal_[inx] = (((uint16_t)buf[0])<<8) | (uint16_t)buf[1];
		}
    }
   
   
