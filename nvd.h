#ifndef NVD_H_
#define NVD_H_

#define NVD_SIZE_BYTES	16
#define NVD_CALIBRATED	0x600D

typedef struct NVD_PARAMS_ {
	int16_t  axBias;
	int16_t  ayBias;
	int16_t  azBias;
	int16_t  gxBias;
	int16_t  gyBias;
	int16_t  gzBias;
	int16_t  calibrated;
	uint16_t checkSum;
} NVD_PARAMS;

extern NVD_PARAMS nvd;


void nvd_Init(void);
void nvd_SaveCalibrationParams(int16_t axb, int16_t ayb, int16_t azb, int16_t gxb, int16_t gyb, int16_t gzb, int16_t calibrated);


#endif

