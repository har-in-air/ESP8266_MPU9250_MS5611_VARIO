#ifndef VARIO_AUDIO_H_
#define VARIO_AUDIO_H_


// climb audio beep period and "duty cycle"
typedef struct BEEP_ {
    int periodTicks;
    int endTick;
} BEEP;

#define VARIO_MAX_CPS         1000

// more audio discrimination for climbrates under this threshold
#define VARIO_XOVER_CPS       300



#define VARIO_STATE_SINK    	11
#define VARIO_STATE_QUIET   	22
#define VARIO_STATE_LIFTY_AIR	33
#define VARIO_STATE_CLIMB   	44

#define CLIMB_DISCRIMINATION_THRESHOLD 25



class VarioAudio {
	
public :
VarioAudio(){};

void Config(int pinPWM);
void VarioBeep(int32_t cps);
void SetFrequency(int32_t freqHz);
void GenerateTone( int32_t freqHz, int ms);

private :
int32_t discrimThreshold_;
int32_t beepCps_;
int32_t varioCps_;
int32_t freqHz_;
int32_t sinkToneCps_;
int32_t climbToneCps_;
int32_t liftyAirToneCps_;

int beepPeriodTicks_;
int beepEndTick_;
int tick_;
int varioState_;

int pinPWM_;

int offScaleHiTone_[10]  = {2000,1000,2000,1000,2000,1000,2000,1000,2000,1000};
int offScaleLoTone_[10]  = {650,600,550,500,450,400,350,300,250,200};
// table for beep duration and repeat rate based on vertical speed
BEEP beepTbl_[10] = {
// repetition rate saturates at 8m/s
{13,4},
{11,4},
{9,4},
{8,4},
{7,4},
{6,4},
{5,3},
{4,2},
{3,1},
{3,1},
};	
};

#endif