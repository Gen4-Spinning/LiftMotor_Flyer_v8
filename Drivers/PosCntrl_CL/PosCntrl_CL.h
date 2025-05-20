/*
 * PosCntrl_CL.h
 *
 *  Created on: 31-Mar-2023
 *      Author: harsha
 */

#ifndef POSCNTRL_CL_H_
#define POSCNTRL_CL_H_

#include "stdio.h"
#include "Constants.h"
#include "Struct.h"
#include "PosCntl.h"

typedef struct PositioningClosedLoop_struct {
	float moveTime;
	uint8_t moveDirection;
	float moveDist_mm;

	float GB_absEndPosition;
	float GB_absStartPosition;
    float GB_absCurrentPosition;
    float GBPoscurrentMoveDist_mm;

    float encPos_absEndPosition;
    float encPoscurrentMoveDist_mm;

    float PID_positioningErr;
    float remainingDistance_mm;
    float pkVelocity;

    char targetSettingError;
    uint8_t targetReached;

} PosCL_TypeDef;


extern PosCL_TypeDef posCL;

void posCL_Reset(PosCL_TypeDef *p);
void posCL_SetupMove(PosCL_TypeDef *p,float startPosition,float moveDist,uint8_t moveDir,float moveTime);
void posCL_ClearMove(PosCL_TypeDef *p);
void posCL_CalcMoveDistance(PosCL_TypeDef *p,float currentDist);
void posCL_setPkVelocity(PosCL_TypeDef *p,float velocity);
float RecalculateTime_OnResume(PosCL_TypeDef *p,PosController *pc);

void posCL_SetupEncPosMove(PosCL_TypeDef *p,float startPosition,float moveDist,uint8_t moveDir);
void posCL_updateMoveDists_FromBothEncoders(PosCL_TypeDef *p,float GB_currentDist,float encPoscurrentDist);
#endif /* POSCNTRL_CL_H_ */
