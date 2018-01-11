#include "Arduino.h"
#include "audio.h"

static int pinPWM_;

void audio_Config(int pinPWM) {
  pinPWM_       = pinPWM;
  analogWrite(pinPWM_, 0);
  }

void audio_SetFrequency(int32_t freqHz) {
	if (freqHz > 0) {
		analogWriteFreq(freqHz);
		analogWrite(pinPWM_, 512); // generate square wave with frequency fHz and 50% duty cycle (512/1023 ~= 0.5)
		}
	else {
		analogWrite(pinPWM_, 0); // turn off output
		}
	}


void audio_GenerateTone(int32_t freqHz, int ms) {
    audio_SetFrequency(freqHz);
    delay(ms);
    audio_SetFrequency(0);
    }
