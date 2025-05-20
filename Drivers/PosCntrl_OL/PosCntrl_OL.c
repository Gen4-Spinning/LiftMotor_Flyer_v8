/*
 * posCntrl_OL.c
 *
 *  Created on: 31-Mar-2023
 *      Author: harsha
 */


#include <PosCntrl_OL.h>

void posOL_Reset(PosOL_TypeDef *p){
	p->moveDirection = 0;
	p->moveDuty = 0;
	p->moveDist_mm = 0;

	p->GB_absCurrentPosition = 0;
	p->GB_absEndPosition = 0;
	p->GB_absStartPosition = 0;

	p->targetReached = 0;
	p->targetSettingError = 0;
}

void posOL_SetupMove(PosOL_TypeDef *p,float startPosition,float moveDist,uint8_t moveDir,uint16_t moveDuty){
	p->moveDuty = moveDuty;
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

	p->targetReached = 0;
}

void posOL_SetupEncPosMove(PosOL_TypeDef *p,float startPosition,float moveDist,uint8_t moveDir){
	if (moveDir == MOVEUP){
		p->encPos_absEndPosition = startPosition + moveDist;
	}
	if (moveDir == MOVEDOWN){
		p->encPos_absEndPosition = startPosition - moveDist;
	}
}

void posOL_ClearMove(PosOL_TypeDef *p){
	p->moveDuty = 0;
	p->moveDist_mm = 0;
	p->moveDirection = 0;
}

void posOL_CheckTargetReached(PosOL_TypeDef *p){
	if (p->moveDirection == MOVEUP){
		if (p->GB_absCurrentPosition >= p-> GB_absEndPosition){
			p->targetReached = 1;
		}
	}
	else if (p->moveDirection == MOVEDOWN){
		if (p->GB_absCurrentPosition <= p-> GB_absEndPosition){
			p->targetReached = 1;
		}
	}
}

//For ring doubler where the GB was nonlinear with distance.
void posOL_CheckTargetReached_wMotorEncoder(PosOL_TypeDef *p,float strokeLength_fromMotorEncoder){
	if (strokeLength_fromMotorEncoder >= p->moveDist_mm){
		p->targetReached = 1;
	}
}

void posOL_CalcMoveDistance(PosOL_TypeDef *p,float currentDist){
	p->GB_absCurrentPosition = currentDist;
	if (p->moveDirection == MOVEUP){
		p->GBPoscurrentMoveDist_mm = p->GB_absCurrentPosition - p->GB_absStartPosition ;
	}else if(p->moveDirection == MOVEDOWN){
		p->GBPoscurrentMoveDist_mm = p->GB_absStartPosition - p->GB_absCurrentPosition;
	}
}

void posOL_updateMoveDists_FromBothEncoders(PosOL_TypeDef *p,float GB_currentDist,float encPoscurrentDist){
	p->encPoscurrentMoveDist_mm = encPoscurrentDist;
	posOL_CalcMoveDistance(p,GB_currentDist);
}


