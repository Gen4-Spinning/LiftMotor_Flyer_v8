/*
 * GB.h
 *
 *  Created on: Mar 30, 2023
 *      Author: harsha
 */

#ifndef GB_H_
#define GB_H_

#include "stdio.h"
#include "stdint.h"

//THESE CONSTANTS COME FROM THE TIM3 - SET as 40ns Per Clk count
//V8 -  encoder gearbox ratio is 80.6
#define GB_ENCODER_SINGLE_ROTATION_IN_TIM3CNTS 45265 // without the 176 counts
#define THROW_AWAY_COUNTS 176
#define GB_ENCODER_TIM3CNTS_PER_MM 140


#define GB_REQUEST_NONE 0
#define GB_REQUEST_STOP 1
#define GB_REQUEST_CONTINUOUS 2
#define GB_REQUEST_SINGLE_SHOT 3
#define GB_REQUEST_SET_HOMING_POS 4

#define HOMING_DONE 1
#define HOMING_SETUP_DONE 2
#define HOMING_SETUP_FAILED 3

#define MAX_BINS 21
#define BIN_SIZE 15



typedef struct GB_struct {
	uint16_t rawPWM_cnts;

	uint16_t PWM_cnts;
	uint16_t prevPWM_cnts;
	uint16_t useCnts;
	uint8_t readFirstTime;

	uint8_t badreadingcounter;
	uint16_t PWM_Period;
	float PWM_dutyCycle;
	float correctedPosition;
	float prevAbsPosition;
	float deltaAbsPosition;

	float RawPosition;

	//float absPosition;
	//encoder health.
	uint8_t firstReading;

	char ErrorFlag;

	char MB_request;  // mainboard Request. only valid in IDLE state
	char MB_requestTimer;

	char outOfBounds;
	char overRideBounds;

} GB_TypeDef;

typedef struct Error{
	  float bin_sums[MAX_BINS];
	    int bin_counts[MAX_BINS];
	    int bin_min_enc[MAX_BINS];
	    int bin_max_enc[MAX_BINS];
	    float bin_means[MAX_BINS];
	    int prev_bin;
	    float binErrorTable[MAX_BINS];
	    float final_bin_errors[MAX_BINS];
}Error_TypeDef;

extern GB_TypeDef GB;
extern Error_TypeDef EGB;


void init_GB(GB_TypeDef *gb);
uint8_t check_GB_Encoder_Health(void);
void init_EG(Error_TypeDef *eg);
void Answer_GB_Request(void);
void CalculateGB_deltaPosition(GB_TypeDef *gb);


#endif /* GB_H_ */
