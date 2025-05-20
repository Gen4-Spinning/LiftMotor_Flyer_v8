/*
 * PosCtnl.c
 *
 *  Created on: 23-May-2023
 *      Author: harsha
 */


#include "PosCntl.h"
#include "math.h"
void PC_ExecVelocity(PosController *pc){
	/* from velocity Ramp Up go to ramp cruise when pk Velocity is reached
	 * in velocity cruise go to next state based on distance,
	 * in velocity zero dont do anything.
	 * in velocity ramp down go to zero velocity
	 * in velocity change layer, go to next state based on location.
	 *
	 * in the position loop you go from velocity cruise to velocity RD and then to
	 *  velocity zero depending on position
	 */
	if ((pc->pV.velocity_state == VELOCITY_CRUISE) || (pc->pV.velocity_state == VELOCITY_ZERO)){
		//dont do anything.
	}

	else if (pc->pV.velocity_state == VELOCITY_RAMPUP){
		if (pc->currentVelocity <= pc->strokePkVelocity){
			pc->currentVelocity += pc->pV.dV_RU;
			if (pc->currentVelocity > pc->strokePkVelocity){
				pc->currentVelocity = pc->strokePkVelocity;
				pc->pV.velocity_state = VELOCITY_CRUISE;
			}
		}
	}

	// reach the interrupted velocity and then handover to the rampUp
	// or ramp down states
	else if (pc->pV.velocity_state == VELOCITY_INTERRUPTION_RESUME){
		if (pc->currentVelocity <= pc->interruptedVelocity){
			pc->currentVelocity += pc->interruption_dV;
			if (pc->currentVelocity > pc->interruptedVelocity){
				pc->currentVelocity = pc->interruptedVelocity;
				pc->pV.velocity_state = pc->interruptionResumeState;
			}
		}
	}

	else if (pc->pV.velocity_state == VELOCITY_RAMPDOWN){
		if (pc->currentVelocity >= 0){
			pc->currentVelocity -= pc->pV.dV_RD;
			if (pc->currentVelocity < 0){
				pc->currentVelocity = 0;
				pc->pV.velocity_state = VELOCITY_ZERO;
				pc->overallState = STROKE_PAUSED;
			}
		}
	}

	else if (pc->pV.velocity_state == VELOCITY_STROKE_RD){
		if (pc->currentVelocity >= END_VELOCITY){
				pc->currentVelocity -= pc->pV.dV_strokeRD;
		}else{
			//end velocity is some residual velocity to ensure we reach the target dist
			pc->currentVelocity = END_VELOCITY;
		}
	}

	else if (pc->pV.velocity_state == VELOCITY_STROKE_RU){
		if (pc->currentVelocity <= pc->strokePkVelocity){
			pc->currentVelocity += pc->pV.dV_strokeRU;
			if (pc->currentVelocity > pc->strokePkVelocity){
				pc->currentVelocity = pc->strokePkVelocity;
				pc->pV.velocity_state = VELOCITY_CRUISE;
			}
		}
	}
	else{
	}

}

void PC_ExecTime(PosController *pc){
	if (pc->pV.velocity_state != VELOCITY_ZERO){
		pc->currentTime += pc->callingTimeSec_Const;
		pc->remainingTime -= pc->callingTimeSec_Const;
	}
}

void PC_ExecPosition(PosController *pc){
	pc->pS.dS =pc->currentVelocity * pc->callingTimeSec_Const;
	pc->currentDist += pc->pS.dS;
	pc->remainingDist = pc->strokeTotalDist - pc->currentDist;

	if (pc->currentDist >= pc->pS.endDist_total){
		pc->pV.velocity_state = VELOCITY_ZERO;
		pc->overallState = STROKE_OVER;
	}

	if (pc->currentDist >= pc->pS.endDist_cruise){
		/* here we have different cases based on whether we were cruising, ramping up or ramping down when
		 * we reached this position.RU and RD are cases of interruptions so we stroke some extra values.
		 * Cruising is the simple normal case.
		 * TODO: A special case borderline case is velocity zero, as we reach that state only when the target falls
		 * to zero, and if we also come into this loop, we shouldnt do anything. just means we are going to
		 * resume very close to the edge of the stroke.Set pausedAtStrokeEnd to indicate this.
		 */
		if (pc->pV.velocity_state == VELOCITY_RAMPUP){
			pc->interruption = 1;
			pc->interruptedVelocity = pc->currentVelocity;
			pc->interruptionType = INTERRUPTION_RU;

			pc->pV.velocity_state = VELOCITY_STROKE_RD;
		}
		else if (pc->pV.velocity_state == VELOCITY_RAMPDOWN){
			pc->interruption = 1;
			pc->interruptedVelocity = pc->currentVelocity;
			pc->interruptionType = INTERRUPTION_RD;

			pc->pV.velocity_state = VELOCITY_STROKE_RD;
		}
		else if (pc->pV.velocity_state == VELOCITY_CRUISE){
			pc->pV.velocity_state = VELOCITY_STROKE_RD;
		}
	}
}

//initialize the first time, resets the interrupt vars also
void PC_Initialize(PosController *pc){

	pc->callingTimeSec_Const = CALLING_TIME_MS/1000.0;
	pc->overallState = STROKE_IDLE;

	pc->strokeTotalDist=0;
	pc->strokeTotalTime=0;
	pc->strokeDirection=0;
	pc->strokePkVelocity=0;

	pc->currentDist=0;
	pc->currentTime=0;
	pc->currentVelocity=0;

	pc->remainingDist=0;
	pc->remainingTime=0;

	pc->interruption=0;
	pc->interruptionType=0;
	pc->interruptedVelocity=0;
	pc->interruption_dV=0;
	pc->interruptionResumeState = 0;
	pc->pausedAtStrokeEnd=0;

	pc->pV.velocity_state = VELOCITY_ZERO;
	pc->pS.dS = 0;

}
//to be called when the end Dist is reached. dont reset the interrupt variables
void PC_Reset(PosController *pc){
	pc->strokeTotalDist=0;
	pc->strokeTotalTime=0;
	pc->strokeDirection=0;
	pc->strokePkVelocity=0;

	pc->currentDist=0;
	pc->currentTime=0;
	pc->currentVelocity=0;

	pc->remainingDist=0;

	pc->overallState = STROKE_IDLE;
	pc->pV.velocity_state = VELOCITY_ZERO;

}


void PC_SetupRampTimes(PosController *p,uint16_t rampUpms,uint16_t rampDown_ms,uint16_t layerChange_ms){
	p->rampUpTime_ms = rampUpms;
	p->rampDownTime_ms = rampDown_ms;
	p->strokeChangeTime_ms = layerChange_ms;
	p->callingTimeSec_Const = (float)CALLING_TIME_MS/1000.0;
}

void PC_ResetInterruptVariables(PosController *pc){
	pc->interruption=0;
	pc->interruptionType=0;
	pc->interruptedVelocity=0;
	pc->interruption_dV=0;
	pc->interruptionResumeState = 0;
	pc->pausedAtStrokeEnd=0;
}


//Trapezoidal Trajectory ( No Jerk Control - No S profile )
void PC_SetupMove(PosController *p,float targetDistance, float targetTime,uint8_t direction){
	/*
	 * We calculate the pk velocity using the stroke defined Ru and RD times. This can be one of three
	 * -the RU and RD or the change layer times sent earlier by the controller. We then calculate the
	 * steps needed for each of the three times using that peak velocity, and from the deltaVelocity for each
	 * of the speeds. The position loop will choose the correct dV to use.
	 * If its end of cruise, it will use velocity change layer
	 * if its during a cruise and gets a ramp down command it will choose velocuty ramp down.
	 * Ramp up will always be velocity rampUp.
	 */
	p->strokeTotalDist = targetDistance;
    p->strokeTotalTime = targetTime;
    p->strokeDirection = direction;

    float steadyStateTime_s = p->strokeTotalTime - ((float)(p->strokeRUTime_ms)/1000) - ((float)(p->strokeRDTime_ms)/1000);

	float temp = (0.5*(float)(p->strokeRUTime_ms + p->strokeRDTime_ms)/1000) + steadyStateTime_s;
	p->strokePkVelocity = p->strokeTotalDist/temp;

	uint16_t rampUp_steps = p->rampUpTime_ms/CALLING_TIME_MS;
	uint16_t rampdown_steps = p->rampDownTime_ms/CALLING_TIME_MS;
	uint16_t strokeRD_steps = p->strokeRDTime_ms/CALLING_TIME_MS;
	uint16_t strokeRU_steps = p->strokeRUTime_ms/CALLING_TIME_MS;
	p->pV.dV_RU = p->strokePkVelocity/rampUp_steps;
	p->pV.dV_RD = p->strokePkVelocity/rampdown_steps;
	p->pV.dV_strokeRD = p->strokePkVelocity/strokeRD_steps;
	/*the 'this works' instead of this should be the right way of doing it
	 * we dont want to slow down too fast cos that causes a rise in voltage that blows up components
	 * so instead we use what has been showed to be working for the Change layer time.
	 */
	p->pV.dV_strokeRD = p->pV.dV_RD/4;//overwrite with what works
	p->pV.dV_strokeRU = p->strokePkVelocity/strokeRU_steps;

	p->pS.dist_RU = 0.5 * p->strokePkVelocity * (float)p->rampUpTime_ms/1000.0;
	p->pS.dist_RD = 0.5 * p->strokePkVelocity * (float)p->rampDownTime_ms/1000.0;
	p->pS.dist_strokeRD = 0.5 * p->strokePkVelocity * (float)p->strokeRDTime_ms/1000.0;
	p->pS.dist_strokeRU = 0.5 * p->strokePkVelocity * (float)p->strokeRUTime_ms/1000.0;
	p->pS.dist_cruisePhase = p->strokeTotalDist - p->pS.dist_strokeRU - p->pS.dist_strokeRD ;

	p->pS.endDist_cruise = p->pS.dist_cruisePhase + p->pS.dist_strokeRU;
	p->pS.endDist_total = p->strokeTotalDist;

}

void PC_SetupInterruptedRDMove(PosController *p){
	/* we need to go up to the interrupted velocity and then just ramp down at the same ramp rate as
	 * earlier.
	 */
	uint16_t rampSteps = p->strokeRDTime_ms/CALLING_TIME_MS;
	p->interruption_dV = p->interruptedVelocity/rampSteps;
	p->interruptionResumeState = VELOCITY_RAMPDOWN;
}

void PC_SetupInterruptedRUMove(PosController *p){
	/* we need to go up to the interrupted velocity and then just ramp down at the same ramp rate as
	 * earlier.
	 */
	uint16_t rampSteps = p->strokeRUTime_ms/CALLING_TIME_MS;
	p->interruption_dV = p->interruptedVelocity/rampSteps;
	p->interruptionResumeState = VELOCITY_RAMPUP;
}


void PC_SetupStrokeTimes(PosController *p,uint16_t strokeRU,uint16_t strokeRD){
	p->strokeRUTime_ms = strokeRU;
	p->strokeRDTime_ms = strokeRD;
}

void PC_Start(PosController *p){
	p->overallState = STROKE_RUNNING;
	p->pV.velocity_state = VELOCITY_RAMPUP;
}

void PC_Start_NewLayer(PosController *p){
	p->overallState = STROKE_RUNNING;
	p->pV.velocity_state = VELOCITY_STROKE_RU;
}

void PC_ResumeInterruption(PosController *p){
	p->overallState = STROKE_RUNNING;
	p->pV.velocity_state = VELOCITY_INTERRUPTION_RESUME;
}


float remainingdV;
float approxTime;
float approxDist;
float accel;
void PC_CalculateCurrentStrokeTime(PosController *p,uint8_t caseType,float resumeAfterStopDist){
	float dist_cruise = 0;
	if (caseType == FULL_STROKE_WITH_SLOW_RAMPUP){
		dist_cruise = p->strokeTotalDist - p->pS.dist_RU; //forget the RD, or correct for it some other day-
		p->currentStrokeTime = p->rampUpTime_ms/1000 + dist_cruise/p->strokePkVelocity;
	}else if (caseType == FULL_STROKE_WITH_FAST_RAMPUP){
		dist_cruise = p->strokeTotalDist - p->pS.dist_strokeRU;
		p->currentStrokeTime = p->strokeChangeTime_ms/1000 + dist_cruise/p->strokePkVelocity;
	}else if (caseType == FULL_STROKE_WITH_INTERRUPTED_RESUME){
		remainingdV = p->strokePkVelocity - p->interruptedVelocity;
		approxTime = remainingdV/p->callingTimeSec_Const; // not taking into account the strokeRU time
		approxDist = approxTime * remainingdV; // not taking into account dis we travel in stroke RU
		dist_cruise = p->strokeTotalDist - approxDist;
		p->currentStrokeTime = p->strokeRUTime_ms/1000 + approxTime + dist_cruise/p->strokePkVelocity;
	}
	else if (caseType == RESUME_STROKE_WITH_RAMPUP){
		// here in both cases we want the time till we reach the edge.
		if (resumeAfterStopDist < p->pS.dist_RU){
			accel = p->pV.dV_RU/p->callingTimeSec_Const;
			p->currentStrokeTime = (float)sqrt((double)(resumeAfterStopDist * 2 / accel));  // s = ut + 0.5*a *t*t
		}else{
			dist_cruise = resumeAfterStopDist - p->pS.dist_RU;
			p->currentStrokeTime = p->rampUpTime_ms/1000 + dist_cruise/p->strokePkVelocity;
		}
	}else if (caseType == RAMPDOWN_MID_STROKE){ // here we just want time to come to a stop, whether or not we touch the edge
		p->currentStrokeTime = (p->currentVelocity/p->pV.dV_RD)*p->callingTimeSec_Const;
	}
	p->remainingTime = p->currentStrokeTime;
}
