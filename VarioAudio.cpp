#include <Arduino.h>
#include "VarioAudio.h"
#include "config.h"
#include "util.h"



void VarioAudio::Config(int pinPWM) {
	discrimThreshold_ 	=  CLIMB_DISCRIMINATION_THRESHOLD;
	sinkToneCps_       	= SINK_THRESHOLD;
	climbToneCps_      	= CLIMB_THRESHOLD;
	liftyAirToneCps_   	= ZERO_THRESHOLD;
    varioState_ 		= VARIO_STATE_QUIET;
	beepPeriodTicks_	= 0;
	beepEndTick_ 		= 0;
	beepCps_ 			= 0;
	varioCps_ 			= 0;
	freqHz_  			= 0;
	pinPWM_ 			= pinPWM;
	analogWrite(pinPWM_, 0);
    }

void VarioAudio::VarioBeep(int32_t nCps) {
   int32_t newFreqHz = 0;

   if (
            (beepPeriodTicks_ <= 0) 
       ||   ((tick_ >= beepPeriodTicks_/2) && (ABS(nCps - varioCps_) > discrimThreshold_))
       ||   ((nCps >= climbToneCps_) && (varioCps_ < climbToneCps_)) 
       ||   ((nCps >= liftyAirToneCps_) && (varioCps_ < liftyAirToneCps_)) 
       ) {
      varioCps_ = nCps;
      // if sinking much faster than glider sink rate, generate continuous tone alarm
      if (varioCps_ <= sinkToneCps_) {
         varioState_ = VARIO_STATE_SINK;
         beepCps_ = varioCps_;
         if (beepCps_ < -VARIO_MAX_CPS) {
			beepCps_ = -VARIO_MAX_CPS;
			}
		 tick_ = 0;
		 if (beepCps_ == -VARIO_MAX_CPS) {
    		beepPeriodTicks_ = 10;
    		beepEndTick_ = 10;
    		newFreqHz = offScaleLoTone_[0];
    		freqHz_ = newFreqHz;
            SetFrequency(freqHz_);
            }
         else {
            beepPeriodTicks_ = 20;
            beepEndTick_  = 18;
            newFreqHz = VARIO_SINK_FREQHZ + ((sinkToneCps_ - beepCps_)*(VARIO_MAX_FREQHZ - VARIO_SINK_FREQHZ))/(VARIO_MAX_CPS+ sinkToneCps_);
            CLAMP(newFreqHz,VARIO_MIN_FREQHZ,VARIO_MAX_FREQHZ);
            freqHz_ = newFreqHz;
            SetFrequency(freqHz_);
         	}
        }
      //if climbing, generate beeps
      else {
         if (varioCps_ >= climbToneCps_) {
            varioState_ = VARIO_STATE_CLIMB;
            beepCps_ = varioCps_;
            if (beepCps_ > VARIO_MAX_CPS) {
				beepCps_ = VARIO_MAX_CPS;
				}
		    tick_ = 0;
			if (beepCps_ == VARIO_MAX_CPS) {
    		    beepPeriodTicks_ = 10;
    		    beepEndTick_ = 10;
    		    newFreqHz = offScaleHiTone_[0];
    		    freqHz_ = newFreqHz;
                SetFrequency(freqHz_);
                }
            else {
           	    int index = beepCps_/100;
           	    if (index > 9) index = 9;
           	    beepPeriodTicks_ = beepTbl_[index].periodTicks;
           	    beepEndTick_ = beepTbl_[index].endTick;
         	    if (beepCps_ > VARIO_XOVER_CPS) {
                    newFreqHz = VARIO_XOVER_FREQHZ + ((beepCps_ - VARIO_XOVER_CPS)*(VARIO_MAX_FREQHZ - VARIO_XOVER_FREQHZ))/(VARIO_MAX_CPS - VARIO_XOVER_CPS);
                    }
                else {
                    newFreqHz = VARIO_MIN_FREQHZ + (beepCps_*(VARIO_XOVER_FREQHZ - VARIO_MIN_FREQHZ))/VARIO_XOVER_CPS;
                    }
                CLAMP(newFreqHz,VARIO_MIN_FREQHZ,VARIO_MAX_FREQHZ);
                freqHz_ = newFreqHz;
                SetFrequency(freqHz_);
                }
            }
         else   // in "lifty-air" band, indicate with a ticking sound with longer interval
         if (varioCps_ >= liftyAirToneCps_) {
            varioState_ = VARIO_STATE_LIFTY_AIR;
    		beepCps_ = varioCps_;
    		tick_ = 0;
    		beepPeriodTicks_ = 30;
    		beepEndTick_ = 2;
    		newFreqHz = VARIO_TICK_FREQHZ + (beepCps_*(VARIO_XOVER_FREQHZ - VARIO_TICK_FREQHZ))/VARIO_XOVER_CPS;
            CLAMP(newFreqHz,VARIO_TICK_FREQHZ,VARIO_MAX_FREQHZ);
            freqHz_ = newFreqHz;
    		SetFrequency(freqHz_);  // higher frequency as you approach climb threshold
            }

      // not sinking enough to trigger alarm,  be quiet
         else{
            varioState_ = VARIO_STATE_QUIET;
            tick_ = 0;
            beepPeriodTicks_ = 0;
            beepEndTick_  = 0;
            freqHz_ = 0;
            SetFrequency(freqHz_);
            }
         }
      }
   else{
      tick_++;
      beepPeriodTicks_--;
      newFreqHz = freqHz_;
      if (tick_ >= beepEndTick_){ // shut off tone
         newFreqHz = 0;
         }
	  else
	  if (beepCps_ == VARIO_MAX_CPS) {
    	  newFreqHz = offScaleHiTone_[tick_];
         }
      else
	  if (beepCps_ == -VARIO_MAX_CPS) {
    	  newFreqHz = offScaleLoTone_[tick_];
          }
     else
	 if (varioState_ == VARIO_STATE_SINK) {
    	 newFreqHz = freqHz_ - 10;
         }
     if (newFreqHz != freqHz_) {
         freqHz_ = newFreqHz;
         SetFrequency(freqHz_);
         }
	  }
   }

void VarioAudio::SetFrequency(int32_t fHz) {
	if (fHz ) {
		analogWriteFreq(fHz);
		analogWrite(pinPWM_, 512);
		}
	else {
		analogWrite(pinPWM_, 0);
		}
	}


void VarioAudio::GenerateTone(int32_t fHz, int ms) {
    SetFrequency(fHz);
    delay(ms);
    SetFrequency(0);
    }

