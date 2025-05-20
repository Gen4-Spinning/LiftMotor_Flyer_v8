/*
 * EncPos.h
 *
 *  Created on: 19-Apr-2023
 *      Author: harsha
 */

#ifndef ENCPOS_H_
#define ENCPOS_H_

#include "EncSpeed.h"
#include "PosCntrl_CL.h"
#include "PosCntrl_OL.h"

#define LIFT_MOTOR_GB 9.0f
#define LEAD_SCREW_PITCH 4.0f

typedef struct EncPos{
	float rps;
	float strokeVelocity_mm_sec;
	float strokeMovement_mm;
	float strokeMovementMM_controlLoop;
	float error_with_GB_deltaSpeed;
	uint16_t errorWithGB_deltaSpeedCount;
	float error_with_GB_deltaPos;
	float absPosition;
}EncPos_TypeDef;

extern EncPos_TypeDef encPos;

void encPos_CalculateMovement(EncPos_TypeDef *encPos,EncSpeed_TypeDef *encSpeed);
void encPos_ZeroMovement(EncPos_TypeDef *encPos);
void encPos_CalculateErrorWithGB_OL(EncPos_TypeDef *encPos,PosOL_TypeDef *p);
void encPos_CalculateErrorWithGB_CL(EncPos_TypeDef *encPos,PosCL_TypeDef *p);
void encPos_CalculateAbsPosition(EncPos_TypeDef *encPos,uint8_t direction);
void encPos_ZeroAbsPosition(EncPos_TypeDef *e,float GB_absPosition);
void encPos_CalculateDeltaAbsPos_withGB(EncPos_TypeDef *e,float GB_deltaAbsPos,float GBabsPos);

#endif /* ENCODERPOSITION_ENCPOS_H_ */
