/*
 * PosPts.c
 *
 *  Created on: 31-Mar-2023
 *      Author: harsha
 */

#include "PosPts.h"
#include "math.h"

void initPosPts(PosPoints *p){
	p->homingDirection = 0;
	p->homingDistance = 0;
	p->homingTime = 0;

	// we set Homing position such that the pwm duty cycle is 12.5%
	p->homingPositionCnts = 0;
	p->homingPositionDutyCycle = 0;
	p->maxLimit_positionCnts = 0;
	p->minLimit_positionCnts = 0;

	p->alreadyAtHome_Flag = 0;
	p->homingControlType = 0;
}

void setupLeadScrewLimitsAndHoming(PosPoints *p,uint16_t homingPos){
	p->homingPositionCnts = homingPos;
	p->homingPositionDutyCycle =( (float)homingPos/GB_ENCODER_SINGLE_ROTATION_IN_TIM3CNTS) * 100.0f;
	p->minLimit_positionCnts = p->homingPositionCnts -  MAX_MOVEMENT_BELOW_HOMING * GB_ENCODER_TIM3CNTS_PER_MM ;
	p->maxLimit_positionCnts = p->homingPositionCnts +  MAX_MOVEMENT_ABOVE_HOMING * GB_ENCODER_TIM3CNTS_PER_MM ;
}

//return 1 if error
uint8_t checkWithinLeadScrewLimits(PosPoints *p,uint16_t currentPosition){
	if ((currentPosition >= p->minLimit_positionCnts) && (currentPosition <= p->maxLimit_positionCnts)){
		return 0;
	}
	return 1;
}

//return 1 if OK, 0 if bad
uint8_t checkHomingPosition(uint16_t homingPosCnts){
	uint16_t minCount = (HOMING_POSITION_DUTY_CYCLE - HOMING_EXTRA_PERCENTAGE)/100.0f  * GB_ENCODER_SINGLE_ROTATION_IN_TIM3CNTS;
	uint16_t maxCount = (HOMING_POSITION_DUTY_CYCLE + HOMING_EXTRA_PERCENTAGE)/100.0f  * GB_ENCODER_SINGLE_ROTATION_IN_TIM3CNTS;

	if ((homingPosCnts < minCount) || (homingPosCnts > maxCount)){
		return 0;
	}else{
		return 1;
	}
}


// if the
void calculateHomeMove(PosPoints *p,uint16_t currentPosition){
//	float deltaCnts = currentPosition - p->homingPositionCnts;
//	float delta_mm = (float)deltaCnts/GB_ENCODER_TIM3CNTS_PER_MM;
	float delta_mm = currentPosition;
	p->alreadyAtHome_Flag = 0;

// if the delta distance to move is less than 10mm, we let the home happen through open loop
//since the closed loop trajectory nos become very small.
	if (delta_mm < -0.5){
		p->homingDirection = MOVEUP;
		p->homingDistance = fabs(delta_mm);
		p->homingTime = p->homingDistance/HOMING_VELOCITY;
		if (delta_mm > -10){
			p->homingControlType = OPEN_LOOP;
		}else{
			p->homingControlType = CLOSED_LOOP;
		}
	}
	else if (delta_mm > 0.5){
		p->homingDirection = MOVEDOWN;
		p->homingDistance = fabs(delta_mm);
		p->homingTime = p->homingDistance/HOMING_VELOCITY;
		if (delta_mm < 10){
			p->homingControlType = OPEN_LOOP;
		}else{
			p->homingControlType = CLOSED_LOOP;
		}
	}
	else{
		p->alreadyAtHome_Flag = 1;
	}

}


void resetHomeCalculations(PosPoints *p){
	p->homingDirection = 0;
	p->homingDistance = 0;
	p->homingTime = 0;
	p->homingControlType = 0;
	p->alreadyAtHome_Flag = 0;
}
