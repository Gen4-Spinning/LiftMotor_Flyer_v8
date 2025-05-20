/*
 * PosPts.h
 *
 *  Created on: 31-Mar-2023
 *      Author: harsha
 */

#ifndef POSPTS_H_
#define POSPTS_H_

#include "GB.h"
#include "Constants.h"

#define MAX_MOVEMENT_BELOW_HOMING 5
#define MAX_MOVEMENT_ABOVE_HOMING 285

#define HOMING_POSITION_DUTY_CYCLE 7.0f
#define HOMING_EXTRA_PERCENTAGE 1.0f

#define HOMING_VELOCITY 2 // mm/s

typedef struct PosPointsStruct {

	uint16_t homingPositionCnts;
	float homingPositionDutyCycle;
	uint16_t maxLimit_positionCnts;
	uint16_t minLimit_positionCnts;

	float homingDistance;
	float encPos_homingDistance;
	uint16_t homingDirection;
	float homingTime;
	uint8_t homingControlType;
	uint16_t alreadyAtHome_Flag;
} PosPoints;

extern PosPoints posPts;

void initPosPts(PosPoints *p);
void setupLeadScrewLimitsAndHoming(PosPoints *p,uint16_t zeroPosition);
uint8_t checkWithinLeadScrewLimits(PosPoints *p,uint16_t currentPosition);
void calculateHomeMove(PosPoints *p,uint16_t currentPosition);
void resetHomeCalculations(PosPoints *p);
uint8_t checkHomingPosition(uint16_t homingPosCnts);



#endif /* POSPTS_H_ */
