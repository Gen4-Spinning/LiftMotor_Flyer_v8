/*
 * EncPos.c
 *
 *  Created on: 19-Apr-2023
 *      Author: harsha
 */

#include "EncPos.h"
#include "Constants.h"

void encPos_CalculateMovement(EncPos_TypeDef *encPos,EncSpeed_TypeDef *encSpeed){
	encPos->rps = encSpeed->speedRPM/60.0;
	encPos->strokeVelocity_mm_sec = (encPos->rps/LIFT_MOTOR_GB) * LEAD_SCREW_PITCH;
	encPos->strokeMovementMM_controlLoop = encPos->strokeVelocity_mm_sec * TIM16_CONTROL_LOOP_CALLING_MS/1000.0f;
	encPos->strokeMovement_mm  += encPos->strokeMovementMM_controlLoop;

}

void encPos_CalculateAbsPosition(EncPos_TypeDef *encPos,uint8_t direction){
	if (direction == MOVEUP){
		encPos->absPosition += (encPos->strokeVelocity_mm_sec * TIM16_CONTROL_LOOP_CALLING_MS/1000.0f);
	}else if (direction == MOVEDOWN){
		encPos->absPosition -= (encPos->strokeVelocity_mm_sec  * TIM16_CONTROL_LOOP_CALLING_MS/1000.0f);
	}
}
void encPos_ZeroMovement(EncPos_TypeDef *encPos){
	encPos->strokeMovement_mm = 0;
	encPos->error_with_GB_deltaPos = 0;
}

void encPos_ZeroAbsPosition(EncPos_TypeDef *e,float GB_absPosition){
	e->absPosition = GB_absPosition;
}

void encPos_CalculateErrorWithGB_OL(EncPos_TypeDef *encPos,PosOL_TypeDef *p){
	encPos->error_with_GB_deltaPos = p->encPoscurrentMoveDist_mm - encPos->strokeMovement_mm;
}

void encPos_CalculateErrorWithGB_CL(EncPos_TypeDef *encPos,PosCL_TypeDef *p){
	encPos->error_with_GB_deltaPos = p->GBPoscurrentMoveDist_mm - encPos->strokeMovement_mm;
}

void encPos_CalculateDeltaAbsPos_withGB(EncPos_TypeDef *e,float GB_deltaAbsPos,float GBabsPos){
	e->error_with_GB_deltaSpeed = GB_deltaAbsPos -  e->strokeMovementMM_controlLoop;
	e->error_with_GB_deltaPos = GBabsPos -  e->absPosition;

}

