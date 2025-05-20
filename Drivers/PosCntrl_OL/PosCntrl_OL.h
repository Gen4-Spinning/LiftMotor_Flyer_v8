/*
 * posCntrl_OL.h
 *
 *  Created on: 31-Mar-2023
 *      Author: harsha
 */

#ifndef POSCNTRL_OL_H_
#define POSCNTRL_OL_H_

#include "stdio.h"
#include "Constants.h"
#include "stdint.h"

#define MOTOR_ENCODER_MM_PER_ROT 0.16
#define MOTOR_ENCODER_CNTS_PER_MM 512  //2048 counts per rotation.

typedef struct PositioningOpenLoop_struct {
	uint16_t moveDuty;
	uint8_t moveDirection;
	float moveDist_mm;

	float GB_absEndPosition;
	float GB_absStartPosition;
    float GB_absCurrentPosition;

    float GBPoscurrentMoveDist_mm;

    float encPos_absEndPosition;
    float encPoscurrentMoveDist_mm;

    char targetSettingError;
    uint8_t targetReached;

} PosOL_TypeDef;

extern PosOL_TypeDef posOL;

void posOL_Reset(PosOL_TypeDef *p);
void posOL_SetupMove(PosOL_TypeDef *p,float startPosition,float moveDist,uint8_t moveDir,uint16_t moveDuty);
void posOL_ClearMove(PosOL_TypeDef *p);
void posOL_CheckTargetReached(PosOL_TypeDef *p);
void posOL_CalcMoveDistance(PosOL_TypeDef *p,float currentDist);

//MotorEncoder stuff
void posOL_CheckTargetReached_wMotorEncoder(PosOL_TypeDef *p,float strokeLength_fromMotorEncoder);
void posOL_updateMoveDists_FromBothEncoders(PosOL_TypeDef *p,float GB_currentDist,float encPoscurrentDist);
void posOL_SetupEncPosMove(PosOL_TypeDef *p,float startPosition,float moveDist,uint8_t moveDir);


#endif /* POSCNTRL_OL_H_ */
