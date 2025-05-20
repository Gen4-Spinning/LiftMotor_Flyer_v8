/*
 * PosCntl.h
 *
 *  Created on: 23-May-2023
 *      Author: harsha
 */

#ifndef POSCNTL_H_
#define POSCNTL_H_

#include "stdio.h"
#include "stdint.h"

#define CALLING_TIME_MS 40
#define END_VELOCITY 1

#define VELOCITY_RAMPUP 1
#define VELOCITY_RAMPDOWN 2
#define VELOCITY_CRUISE 3
#define VELOCITY_ZERO 4
#define VELOCITY_STROKE_RD 5
#define VELOCITY_STROKE_RU 6
#define VELOCITY_INTERRUPTION_RESUME 7

/* when you start the lifts, you set the state to  stroke Running. the interrupt start the position loop
 * when it sees the stroke is not stroke IDLE. when the end dist is reached the pos loop sets
 * the state to stroke over. once you come out of stroke over, you clean up and then set the stroke to idle again.
 * when you pause, it is not considered as the stroke is over, but nothing happens because the stroke velocity is zero.
 */
#define STROKE_OVER 1
#define STROKE_IDLE 2
#define STROKE_RUNNING 3
#define STROKE_PAUSED 4

#define INTERRUPTION_RU 1
#define INTERRUPTION_RD 2

//type of cases to calculate remaining Time
#define FULL_STROKE_WITH_SLOW_RAMPUP 1
#define FULL_STROKE_WITH_FAST_RAMPUP 2
#define FULL_STROKE_WITH_INTERRUPTED_RESUME 3
#define RESUME_STROKE_WITH_RAMPUP 4
#define RAMPDOWN_MID_STROKE 5

#define ARGUMENT_UNUSED -1

typedef struct velocityStruct{
	char velocity_state;
	float dV_RU;
	float dV_RD;
	float dV_strokeRD;
	float dV_strokeRU;
}pVelocity;

typedef struct posStruct{
	float    dS;

	float    dist_cruisePhase;
	float    dist_RU;
	float    dist_RD;
	float    dist_strokeRD;
	float    dist_strokeRU;

	float    endDist_RU;
	float    endDist_cruise;
	float    endDist_total;
}pPosition;

typedef struct posController{
	float callingTimeSec_Const;

	uint16_t rampUpTime_ms;			// possible RU and RD and change layer times sent by the controller.
	uint16_t rampDownTime_ms;
	uint16_t strokeChangeTime_ms;

	char overallState;

	float strokeTotalDist;
	float strokeTotalTime;
	float strokeDirection;
	uint16_t strokeRUTime_ms; //for this stroke
	uint16_t strokeRDTime_ms;
	float strokePkVelocity;

	float currentDist;
	float currentTime;
	float currentVelocity;
	float currentStrokeTime;

	float remainingDist;
	float remainingTime;

	char interruption;
	char interruptionType;
	float interruptedVelocity;
	float interruption_dV;
	uint8_t interruptionResumeState;
	float pausedAtStrokeEnd;

	pVelocity pV;
	pPosition pS;

}PosController;

extern PosController PC;


void PC_Reset(PosController *pc);
void PC_Initialize(PosController *pc);

void PC_ExecPosition(PosController *pc);
void PC_ExecTime(PosController *pc);
void PC_ExecVelocity(PosController *pc);

void PC_SetupMove(PosController *p,float targetDistance, float targetTime,uint8_t direction);
void PC_SetupRampTimes(PosController *p,uint16_t rampUpms,uint16_t rampDown_ms,uint16_t layerChange_ms);
void PC_SetupStrokeTimes(PosController *p,uint16_t strokeRU,uint16_t strokeRD);

void PC_Start(PosController *p);
void PC_Start_NewLayer(PosController *p);


void PC_ResetInterruptVariables(PosController *pc);
void PC_ResumeInterruption(PosController *p);
void PC_SetupInterruptedRDMove(PosController *p);
void PC_SetupInterruptedRUMove(PosController *p);

void PC_CalculateCurrentStrokeTime(PosController *p,uint8_t caseType,float resumeAfterStopDist);

#endif /* POSCNTL_H_ */
