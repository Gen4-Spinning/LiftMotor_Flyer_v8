/*
 * PosCntrl_CL.c
 *
 *  Created on: 31-Mar-2023
 *      Author: harsha
 */


#include "PosCntrl_CL.h"

void posCL_Reset(PosCL_TypeDef *p){
	p->moveTime = 0;
	p->moveDirection = 0;
	p->moveDist_mm = 0;

	p->GB_absEndPosition = 0;
	p->GB_absStartPosition = 0;
	p->GB_absCurrentPosition = 0;
	p->GBPoscurrentMoveDist_mm = 0;

	p->encPos_absEndPosition = 0;
	p->encPoscurrentMoveDist_mm = 0;

	p->PID_positioningErr = 0;
	p->remainingDistance_mm = 0;
	p->pkVelocity = 0 ;

	p->targetSettingError = 0;
	p->targetReached = 0;
}


void posCL_SetupMove(PosCL_TypeDef *p,float startPosition,float moveDist,uint8_t moveDir,float moveTime){
	p->moveTime = moveTime;
	p->moveDist_mm = moveDist;
	p->moveDirection = moveDir;
	p->GB_absStartPosition = startPosition;
	if (p->moveDirection == MOVEUP){
		p->GB_absEndPosition = startPosition + moveDist;
	}
	if (p->moveDirection == MOVEDOWN){
		p->GB_absEndPosition = startPosition - moveDist;
	}
	//TODO set the error if the limits are too much.

	p->GBPoscurrentMoveDist_mm = 0;
	p->PID_positioningErr = 0;
	p->remainingDistance_mm = p->moveDist_mm;
	p->targetReached = 0;
}

void posCL_SetupEncPosMove(PosCL_TypeDef *p,float startPosition,float moveDist,uint8_t moveDir){
	if (moveDir == MOVEUP){
		p->encPos_absEndPosition = startPosition + moveDist;
	}
	if (moveDir == MOVEDOWN){
		p->encPos_absEndPosition = startPosition - moveDist;
	}
}


void posCL_ClearMove(PosCL_TypeDef *p){
	p->moveTime = 0;
	p->moveDist_mm = 0;
	p->moveDirection = 0;
	p->PID_positioningErr = 0;
}

void posCL_setPkVelocity(PosCL_TypeDef *p,float velocity){
	p->pkVelocity = velocity;
}

void posCL_CalcMoveDistance(PosCL_TypeDef *p,float currentDist){

	p->GB_absCurrentPosition = currentDist;
	if (p->moveDirection == MOVEUP){
		p->GBPoscurrentMoveDist_mm = p->GB_absCurrentPosition - p->GB_absStartPosition;
	}else if(p->moveDirection == MOVEDOWN){
		p->GBPoscurrentMoveDist_mm = p->GB_absStartPosition - p->GB_absCurrentPosition;
	}
}

void posCL_updateMoveDists_FromBothEncoders(PosCL_TypeDef *p,float GB_currentDist,float encPoscurrentDist){
	p->encPoscurrentMoveDist_mm = encPoscurrentDist;
	p->remainingDistance_mm = p->moveDist_mm - p->encPoscurrentMoveDist_mm;
	posCL_CalcMoveDistance(p,GB_currentDist);
}


float RecalculateTime_OnResume(PosCL_TypeDef *p,PosController *pc){
	/* this uses the actual location, not the target location to calculate what
	 * should be the time, given the RU and RD to make the move.
	 */
	float steadyState_distance = 0;
	float steadyStateTime = 0;
	float totalTime = 0;

	float rampUp_distance = 0.5 * pc->strokeRUTime_ms/1000.0 * p->pkVelocity;
	float rampDown_distance = 0.5 * pc->strokeRDTime_ms/1000.0 * p->pkVelocity;
	//do we care if there is not enough space left for the lift to reach its top speed?
	// No not really. WE will handle it elsewhere.
	steadyState_distance = p->remainingDistance_mm - rampUp_distance - rampDown_distance;
	steadyStateTime = steadyState_distance/p->pkVelocity;
	totalTime = steadyStateTime + pc->strokeRUTime_ms/1000.0 + pc->strokeRDTime_ms/1000.0;
	return totalTime;
}


