/*
 * Console.c
 *
 *  Created on: Mar 16, 2023
 *      Author: harsha
 */


#include "Console.h"
#include "EepromFns.h"

#include "EncoderCalibration.h"
#include "EncoderFns.h"

#include "GB.h"
#include "EncPos.h"
#include "EncSpeed.h"

#include "PosCntrl_CL.h"
#include "PosCntrl_OL.h"
#include "PosCntl.h"
#include "PosPts.h"

#include "Constants.h"
#include "LiftRamp.h"

#include "main.h"
#include "stdio.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim17;

void Console_LiftOL_Start(void){
	LRM.controlType = OPEN_LOOP;
	LRM.runType = LOCAL_DEBUG;
	posOL_SetupMove(&posOL, GB.RawPosition,LRM.distance,LRM.direction,LRM.duty);//set up GB end Pos
	posOL_SetupEncPosMove(&posOL,encPos.absPosition,LRM.distance,LRM.direction); //set up delta Encoder end pos
	encPos_ZeroMovement(&encPos);
	SetupLiftRampDuty(&liftRampDuty,LRM.duty,2000,2000,RUN_FOREVER,400,800);
	StartSixSectorObj_v8_Correction(&sixSectorCntrl_Obj,&sixSectorObj,LRM.direction); // lift Down is CW, lift UP is CCW
	StartLiftRampDuty(&liftRampDuty);
	T.tim16_20msTimer  =0;
	T.tim16_oneSecTimer = 0;
}

void Console_LiftOL_Stop(void){
  StopSixSectorObj(&sixSectorCntrl_Obj,&sixSectorObj);
  StopLiftRampDuty(&liftRampDuty);
}

void Console_LiftCL_Start(void){
	LRM.runType = LOCAL_DEBUG;
	LRM.controlType = CLOSED_LOOP;
	S.motorState = RUN_STATE;
	posCL_SetupMove(&posCL,GB.correctedPosition,LRM.distance,LRM.direction,LRM.time);
	posCL_SetupEncPosMove(&posCL,encPos.absPosition,LRM.distance,LRM.direction);
	encPos_ZeroMovement(&encPos);
	PC_SetupRampTimes(&PC,10000,10000,1000);
	PC_SetupStrokeTimes(&PC, 2000,2000);
	PC_SetupMove(&PC, LRM.distance, LRM.time, LRM.direction);
	posCL_setPkVelocity(&posCL,PC.strokePkVelocity);
	StartSixSectorObj_v8_Correction(&sixSectorCntrl_Obj,&sixSectorObj,LRM.direction);
	PC_Start(&PC);
	T.tim16_20msTimer  =0;
	T.tim16_oneSecTimer = 0;
}

void Console_LiftCL_Stop(void){
	StopSixSectorObj(&sixSectorCntrl_Obj,&sixSectorObj);
	PC_Reset(&PC);
	S.recievedStopCommand =1;
}

int waitForNoInput(void){
	int a;
	int rc;
	while ((rc = scanf("%d", &a)) == 0)
		{
			// clear what is left, the * means only match and discard:
			scanf("%*[^\n]");
			// input was not a number, ask again:
			printf("\r\n Not a number! Try Again ");
		}
	if (rc == EOF)
	{
		printf("\r\n No number found");
		return BADCONSOLE_VAL;
	}
	else
	{
		printf("\r\n You entered %d.", a);
		return a;
	}
	return BADCONSOLE_VAL;
}




uint8_t configurePIDSettings(void){
	int noEntered;
	uint8_t firstTime = 1;
	uint16_t minLimit=0;
	uint16_t maxLimit=0;
	uint8_t  pidQueryNo=0;
	uint8_t newPIDquery = 0;
	uint8_t insidePIDLoop =0;

	float Kp_new=0,Ki_new=0;
	uint8_t FF_factor_new=0,startOffsetNew=0;
	uint8_t eepromWrite=0,eepromWriteSuccess=0;
	while(1){
		if (firstTime == 1){
			printf("\r\n***MOTOR PID SETTINGS***");
			printf("\r\n Enter new PID parameter values in the order asked.To skip entering a value enter -1");
			printf("\r\n To start, press 1 or enter -1 to exit this menu");
			firstTime = 0;
			insidePIDLoop = 0;
		}

		noEntered = waitForNoInput();

		if (eepromWrite == 1){
			if (noEntered == 1){
				eepromWriteSuccess  = writePWMSettingsToEEPROM_Manual(Kp_new,Ki_new,FF_factor_new,startOffsetNew);
				if (eepromWriteSuccess){
					printf("\r\n Success! Written into Eeprom! Restart Motor for new values to take effect");
				}else{
					printf("\r\n Fail! Not Written into Eeprom!");
				}
				printf("\r\n ------------------");
				eepromWrite = 0;
				firstTime = 1;
			}else if (noEntered == 0){
				printf("\r\n Not writing into Eeprom!");
				eepromWrite = 0;
				firstTime = 1;
			}
			noEntered = 0;
		}

		else if (insidePIDLoop == 0){
			if (noEntered == 1){
				printf("\r\n -----Kp----");
				printf("\r\n Current Kp value :%5f. Enter new Value : (0-300)",sV.Kp);
				minLimit = 0;
				maxLimit = 300;
				pidQueryNo = 1;
				insidePIDLoop = 1;
			}

			if (noEntered == -1){
			  printf("\r\n");
			  break;
			}
		}else{
			if (noEntered == -1){
				if (pidQueryNo!=5){
					printf("\r\n Skipping parameter");
					if (pidQueryNo == 1){
						Kp_new = sV.Kp;
					}else if (pidQueryNo == 2){
						Ki_new = sV.Ki;
					}else if (pidQueryNo == 3){
						FF_factor_new = sV.ff_percent;
					}else if (pidQueryNo == 4){
						startOffsetNew = sV.start_offset;
					}
					newPIDquery = 1;
				}else{
					insidePIDLoop = 0;
					firstTime = 1;
				}
			}

			if (newPIDquery == 0){
				if ((noEntered <= maxLimit ) && ( noEntered >= minLimit)){
					if (pidQueryNo == 1){
						Kp_new = noEntered;
						printf("\r\n Kp = %5f",Kp_new);
						newPIDquery = 1;
					}else if (pidQueryNo == 2){
						Ki_new = noEntered;
						printf("\r\n Ki = %5f",Ki_new);
						newPIDquery = 1;
					}else if (pidQueryNo == 3){
						FF_factor_new = noEntered;
						printf("\r\n FF_factor = %02d",FF_factor_new);
						newPIDquery = 1;
					}else if (pidQueryNo == 4){
						startOffsetNew = noEntered;
						printf("\r\n startOffset = %03d",startOffsetNew);
						newPIDquery = 1;
					}else{}
					noEntered = 0;
				}else{
					printf("\r\n Outside Limits!Try again");
					noEntered = 0;
				}
			}

			if (newPIDquery){
				if(pidQueryNo == 1){
					printf("\r\n -----Ki-----");
					printf("\r\n Current Ki value :%5f. Enter new Value : (0-300)",sV.Ki);
					minLimit = 0;
					maxLimit = 300;
				}else if (pidQueryNo == 2){
					printf("\r\n -----FF_Percent-----");
					printf("\r\n Current FF_Percent value :%2d.Enter new value (0-80)",sV.ff_percent);
					minLimit = 0;
					maxLimit = 80;
				}else if (pidQueryNo == 3){
					printf("\r\n -----start Offset-----");
					printf("\r\n Current start Offset value %03d.Enter new value (0-200)",sV.start_offset);
					minLimit = 0;
					maxLimit = 200;
				}
				if (pidQueryNo == 4){
					printf("\r\n -----All values Entered!-----");
					printf("\r\n Kp:%5f, Ki:%5f, FF_percent:%02d, StartOffset:%03d",Kp_new,Ki_new,FF_factor_new,startOffsetNew);
					printf("\r\n Write into Eeprom and enable? (1 for yes,0 for no)");
					eepromWrite = 1;
				}
				pidQueryNo ++;
				newPIDquery = 0;
			}
		} // closes else

	} //finishes while
	return 1;
}


uint8_t configureSettings(uint8_t setting){
	int noEntered;
	char eepromWrite = 0;
	uint8_t firstTime = 1;
	uint16_t minLimit=0;
	uint16_t maxLimit=0;
	uint16_t noToSave =0;
	uint8_t eepromWriteSuccess =0;
	while(1){
		if (firstTime == 1){
			if (setting == CONSOLE_CANID){
				printf("\r\n***MOTOR CAN-ID MENU***");
				printf("\r\n Enter new Motor CAN ID (2-7)");
				minLimit = 2;
				maxLimit = 7;
			}else if (setting == CONSOLE_DIRECTION){
				printf("\r\n***MOTOR DEFAULT-DIRECTION MENU***");
				printf("\r\n Enter Motor Default Direction (CW-0,CCW-1)");
				minLimit = 0;
				maxLimit = 1;
			}else if (setting == CONSOLE_ENCODER_OFFSET){
				printf("\r\n***MOTOR ENCODER-OFFSET  MENU***");
				printf("\r\n Enter Encoder Offset (0-16384)");
				minLimit = 0;
				maxLimit = 16384;
			}else{}
			printf("\r\n Enter -1 to go back");
			firstTime = 0;
		}

		noEntered = waitForNoInput();

		if (eepromWrite == 0){
			if (noEntered == -1){
			  printf("\r\n");
			  break;
			}
			if ((noEntered <= maxLimit ) && ( noEntered >= minLimit)){
				noToSave = noEntered;
				printf("\r\n Write into Eeprom and enable? (1 for yes,0 for no)");
				eepromWrite = 1;
				noEntered = 0;
			}else{
				printf("\r\n Outside Limits!Try again");
				noEntered = 0;
			}
		}else{
			if (noEntered == 1){
				if (setting == CONSOLE_CANID){
					eepromWriteSuccess  = writeMotorSettingsToEEPROM_Manual(noToSave,-1,-1);
				}else if (setting == CONSOLE_DIRECTION){
					eepromWriteSuccess  = writeMotorSettingsToEEPROM_Manual(-1,-1,noToSave);
				}else if (setting == CONSOLE_ENCODER_OFFSET){
					eepromWriteSuccess  = writeMotorSettingsToEEPROM_Manual(-1,noToSave,-1);
				}
				if (eepromWriteSuccess){
					printf("\r\n Success! Written into Eeprom!Restart Motor for new values to take effect");
				}else{
					printf("\r\n Fail! Not Written into Eeprom!");
				}
				printf("\r\n Enter a new no to save or -1 to go back");
				eepromWrite = 0;
				noEntered = 0;
			}
			else if (noEntered == 0){
				printf("\r\n Exited without writing into Eeprom!");
				printf("\r\n Enter a new no to save or -1 to go back");
				eepromWrite = 0;
				noEntered = 0;
			}
			else {}
		}//finishes eepromWrite else

	} //finishes while
	return 1;
}

uint8_t printSettings(void){
	uint8_t firstTime = 1;
	int noEntered = 0;
	while(1){
		if (firstTime){
			printf("\r\n***MOTOR SETTINGS****");
			printf("\r\n MOTOR CAN ID = %d",sV.MOTID);
			printf("\r\n MOTOR Encoder Offset = %d",sV.AMS_offset_index);
			printf("\r\n MOTOR Default Dir = %d (CW=0,CCW=1)",sV.default_direction);
			printf("\r\n Kp:%5.2f, Ki:%5.2f, FF_percent:%02d, StartOffset:%03d",sV.Kp,sV.Ki,sV.ff_percent,sV.start_offset);
			printf("\r\n GearBox PWM Counts = %05d",GB.PWM_cnts);
			printf("\r\n GearBox Abs Position = %5.2f",GB.correctedPosition);
			printf("\r\n GearBox Homing Position = %05d",posPts.homingPositionCnts);
			printf("\r\n MotorEncoder Abs Position = %6.2f",encPos.absPosition);
			printf("\r\n GearBox LOB Limit Disabled? = %01d",GB.overRideBounds);
			printf("\r\n Enter -1 to exit");
			firstTime = 0;
		}

		noEntered = waitForNoInput();

		if (noEntered == -1){
			noEntered = 0;
			break;
		}else{
			noEntered = 0;
		}
	}

	return 1;
}

uint8_t runGearBoxHomingRoutine(void){
	uint8_t firstTime = 1;
	int noEntered = 0;
	while(1){
		if (firstTime){
			printf("\r\n***GEAR BOX Homing***");
			printf("\r\n Rotate the Gear Box Shaft till the PWM Count Duty cycle is 12.5");
			printf("\r\n Press any no key apart from 1 to refresh the value");
			printf("\r\n Press 1 to save the value into the Eeprom");
			printf("\r\n Enter -1 to exit");
			firstTime = 0;
		}

		noEntered = waitForNoInput();
		printf("\r\n GearBox PWM Duty Cycle = %5.2f",GB.PWM_dutyCycle);
		printf("\r\n GearBox PWM Cnts = %04d",GB.PWM_cnts);

		if (noEntered == -1){
			noEntered = 0;
			break;
		}else if (noEntered == 1){
			uint8_t eepromWriteSuccess  = writeHomingPositionToEeprom(GB.PWM_cnts);
			if (eepromWriteSuccess){
				printf("\r\n Success! Written into Eeprom!Restart Motor for new values to take effect");
			}else{
				printf("\r\n Fail! Not Written into Eeprom!");
			}
			noEntered = 0;
		}else{
			noEntered = 0;
		}
	}

	return 1;
}


uint8_t runGearBoxCalibrationRoutine(void){
	uint8_t firstTime = 1;
	int noEntered = 0;
	while(1){
		if (firstTime){
			printf("\r\n***GEAR BOX Calibration***");
			printf("\r\n Make sure the nut is removed from the lead screw!");
			printf("\r\n Prerequisites: The GearBox PWM count must be as close to 45000.");
			printf("\r\n Use the OL Run mode to bring it to this position");
			printf("\r\n Press 1 to start the run. Start saving terminal output Before pressing 1");
			printf("\r\n Press 2 to stop the run any time");
			printf("\r\n Press 3 to save bin errors to EEPROM");
			printf("\r\n Enter -1 to exit");
			firstTime = 0;
		}

		noEntered = waitForNoInput();

		if (noEntered == -1){
			noEntered = 0;
			break;
		}
		else if (noEntered == 1){
			if (C.runningOL == 0){
				printf("\r\n ---START!---\r\n");
				init_EG(&EGB);
				LRM.distance = 300;
				LRM.duty = 200;
				LRM.direction = 1; // (UP is 1, Down is 0)
				Console_LiftOL_Start();
				C.runningOL = 1;
				C.calibRun = 1;
				C.logging = 1;
				noEntered = 0;
			}
		}
		else if (noEntered == 2){
			if (C.runningOL == 1){
				printf("\r\n * Stopping *\r\n");
				Console_LiftOL_Stop();
				C.runningOL = 0;
				C.calibRun = 0;
				noEntered = 0;

				printf("\r\nFinal Average Error Per Bin:\r\n");
				printf("BIN\tMin_ENC\tMax_ENC\tAverage_Error\tCount\r\n");

				// Store average errors into global array
				for (int i = 0; i < MAX_BINS; i++) {
					if (EGB.bin_counts[i] > 0) {
						EGB.final_bin_errors[i] = EGB.bin_sums[i] / EGB.bin_counts[i];
						printf("%02d\t%5d\t%5d\t%7.3f\t\t%d\r\n",
							i,
							EGB.bin_min_enc[i],
							EGB.bin_max_enc[i],
							EGB.final_bin_errors[i],
							EGB.bin_counts[i]);
					} else {
						EGB.final_bin_errors[i] = 0.0f;
					}
				}

				firstTime = 1;
			}
		}

		else if (noEntered == 3){
			printf("\r\nWriting bin errors to EEPROM...\r\n");
			uint8_t eepromWriteSuccess = writeGBBinErrorsToEEPROM();
			if (eepromWriteSuccess)
				printf("EEPROM write success.\r\n");
			else
				printf("EEPROM write failed.\r\n");

			noEntered = 0;
		}
		else{
			noEntered = 0;
			C.calibRun = 0;
			C.logging = 0;
		}
	} // closes while

	return 1;
}

uint8_t runGearBoxBinManagement(void) {
    uint8_t firstTime = 1;
    int noEntered = 0;

    while (1) {
        if (firstTime) {
            printf("\r\n***GEAR BOX Bin Error Management***");
            printf("\r\n Press 1 to read bin errors from EEPROM and display");
            printf("\r\n Press 2 to zero all bin errors and write to EEPROM");
            printf("\r\n Enter -1 to return");
            firstTime = 0;
        }

        noEntered = waitForNoInput();

        if (noEntered == -1) {
            noEntered = 0;
            break;
        }
        else if (noEntered == 1) {
            printf("\r\nReading bin errors from EEPROM...\r\n");

            float readErrors[MAX_BINS];
            readGBBinMeanErrorsFromEEPROM(readErrors);

            printf("BIN\tError_from_EEPROM\r\n");
            for (int i = 0; i < MAX_BINS; i++) {
                printf("%02d\t%7.3f\r\n", i, readErrors[i]);
            }

            noEntered = 0;
        }
        else if (noEntered == 2) {
            printf("\r\nZeroing all bin errors...\r\n");

            float zeroErrors[MAX_BINS];
            for (int i = 0; i < MAX_BINS; i++) {
                zeroErrors[i] = 0.0f;
            }

            uint8_t eepromWriteSuccess = writeGBBinErrorsToEEPROM();

            if (eepromWriteSuccess) {
                printf("EEPROM zeroing success.\r\n");
            } else {
                printf("EEPROM zeroing failed.\r\n");
            }

            noEntered = 0;
        }
        else {
            noEntered = 0;
        }
    }

    return 1;
}




uint8_t ToggleLiftOutOfBoundsLimitSetting(void){
	uint8_t firstTime = 1;
	int noEntered = 0;
	while(1){
		if (firstTime){
			printf("\r\n***TOGGLE LIFT OUT OF BOUNDS LIMITS SETTING***");
			printf("\r\n ---DANGER! PROCEED CAUTIOUSLY---");
			printf("\r\n lift out of bounds (LOB) Disabled? :%01d",GB.overRideBounds);
			printf("\r\n Press 1 to toggle the LOB setting");
			printf("\r\n Enter -1 to exit");
			firstTime = 0;
		}

		noEntered = waitForNoInput();
		if (noEntered == -1){
			noEntered = 0;
			break;
		}else if (noEntered == 1){
			GB.overRideBounds = !(GB.overRideBounds);
			if (GB.overRideBounds){
			  E.liftOutOfBoundsError = 0;
			  E.errorFlag=0;
			  R.motorError = 0;
			  HAL_TIM_Base_Start_IT(&htim17);
			  S.motorState=IDLE_STATE;
			}
			printf("\r\n lift out of bounds (LOB) Disabled? :%01d",GB.overRideBounds);
			noEntered = 0;
		}else{
			noEntered = 0;
		}
	}

	return 1;
}

void MX_TIM1_Init_Copy(void){

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 4;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1499;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
	Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
	Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
	Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
	Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
	Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
	Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
	Error_Handler();
  }
  HAL_TIMEx_EnableDeadTimePreload(&htim1);
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 90;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
	Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
}

uint8_t runMotorCalibrationRoutine(void){
	int noEntered = 0;
	char skipScanf = 0;
	char firstTime = 1;
	uint8_t zeroed;
	while(1){

		if (firstTime == 1){
			printf("\r\n***MOTOR CALIBRATION ROUTINE****");
			printf("\r\n***DO NOT RUN WITH GEARBOX ASSEMBLED ****");
			printf("\r\n Start the Calibration Routine? Enter 1 for yes and -1 to go back");
			printf("\r\n You cannot stop the routine till its over");
			printf("\r\n Note down the index no and enter it through option 3 in the main Menu");
			skipScanf = 0;
			firstTime = 0;
		}

		if (skipScanf == 0){
			noEntered = waitForNoInput();
		}

		if (noEntered == 1){
			printf("\r\n running Calibration");
			zeroed = updateEncoderZeroPosition(0);
			if (zeroed){
				 MX_TIM1_Init_Copy();
				 RunCalibrationWithPrintf(); // BLOCKING
				 printf("\r\n Do you wish to run the routine again?(1 for yes, -1 for no and to go back)");
				 skipScanf = 0;
			 }else{
				 printf("\r\n Zeroing Encoder before Calibration Failed! Try Again!");
				 firstTime = 1;
			 }
			noEntered = 0;

		}

		if (noEntered == -1){
			noEntered = 0;
			printf("\r\n");
			break;
		}
	}
	return 1;
}


uint8_t runLiftMotorClosedLoop(void){
	int noEntered = 0;
	char firstTime = 1;

	float minVelocity = 1.5;
	float maxVelocity = 4.0;
	int minTime;
	int maxTime;
	while(1){
		if (firstTime == 1){
			printf("\r\n***LIFT MOTOR CLOSEDLOOP TESTS****");
			printf("\r\n ---- 1. Enter Target Distance (mm) (10-300)");
			printf("\r\n press '-1' to exit");
			firstTime = 0;
			C.CL_queryNo = 1;
			C.runningCL = 0;
		}

		noEntered = waitForNoInput();

		if (noEntered == -1){
			if (C.CL_queryNo == 1){
				break;
			}
			else{
				printf("\r\n Resetting Inputs!");
				C.CL_queryNo = 0;
				firstTime = 1;
			}
			noEntered = 0;
		}
		else{
			if (C.runningCL == 0){
				if (C.CL_queryNo == 1){
					if ((noEntered >= 10) && (noEntered <= 300)){
						LRM.distance = noEntered;
						minTime = (int)(noEntered/maxVelocity);
						if (minTime < 2){
							minTime = 2;
						}
						maxTime = (int)(noEntered/minVelocity);
						printf("\r\n ---- 2.Enter Time in which to move (%03d-%03d)",minTime,maxTime);
						printf("\r\n press -1 to go back");
						C.CL_queryNo = 2;
					}
				}else if (C.CL_queryNo == 2){
					if ((noEntered >= minTime) && (noEntered <= maxTime)){
						LRM.time = noEntered;
						printf("\r\n ---- 3.Enter Direction in which to move (1-UP,0-DOWN)");
						printf("\r\n press -1 to go back");
						C.CL_queryNo = 3;
					}
				}else if (C.CL_queryNo == 3){
					if ((noEntered == 0) || (noEntered == 1)){
						LRM.direction = noEntered;
						printf("\r\n---- 4. Setup Complete");
						printf("\r\n Distance = %5.2f,Time = %5.2f,Direction = %01d",LRM.distance,LRM.time,LRM.direction);
						printf("\r\n ---Controls---");
						printf("\r\n press 1 to start");
						printf("\r\n press 2 to stop (while running)");
						printf("\r\n press -1 to go back (while stopped)");
						printf("\r\n ---Data Format----");
						//Time.strokeVelocity,targetDist,EncActualDist,Error,GB_ActualDist,GBAbsPos,EncAbsPos,duty,current,liftRPM
						printf("\r\n Time(sec),liftVelocity(mm/s),targetStrokePos,EncActualStroke,PIDError,GBActualStroke,GB_AbsPos,Enc_AbsPos,Duty,Current(A),RPM\r\n");
						C.CL_queryNo = 4;
					}
				}else if (C.CL_queryNo == 4){
					if (noEntered == 1){
						printf("\r\n ---START!---\r\n");
						Console_LiftCL_Start();
						C.runningCL = 1;
						C.logging = 1;
					}
				}
				noEntered  = 0;
			} // closed C.CL_running == 0
			else{
				if (noEntered == 2){
					printf("\r\n *** Stopping ***\r\n");
					Console_LiftCL_Stop();
					noEntered = 0;
					C.runningCL = 0;
				}
			}
		}//closes if noEntered != -1

	} // closes while
	return 1;
}


uint8_t runLiftMotorOpenLoop(void){
	int noEntered = 0;
	char firstTime = 1;

	while(1){
		if (firstTime == 1){
			printf("\r\n***LIFT MOTOR OPENLOOP TESTS****");
			printf("\r\n ---- 1. Enter Target Distance (mm) (1-300)");
			printf("\r\n press '-1' to exit");
			firstTime = 0;
			C.OL_queryNo = 1;
			C.runningOL = 0;
		}

		noEntered = waitForNoInput();

		if (noEntered == -1){
			if (C.OL_queryNo == 1){
				break;
			}
			else{
				printf("\r\n Resetting Inputs!");
				C.OL_queryNo = 0;
				firstTime = 1;
			}
			noEntered = 0;
		}
		else{
			if (C.runningOL == 0){
				if (C.OL_queryNo == 1){
					if ((noEntered >= 1) && (noEntered <= 300)){
						LRM.distance = noEntered;
						printf("\r\n ---- 2.Enter Duty at which to move (50-1000)");
						printf("\r\n press -1 to go back");
						C.OL_queryNo = 2;
					}
				}else if (C.OL_queryNo == 2){
					if ((noEntered >= 50) && (noEntered <= 1000)){
						LRM.duty = noEntered;
						printf("\r\n ---- 3.Enter Direction in which to move (1-UP,0-DOWN)");
						printf("\r\n press -1 to go back");
						C.OL_queryNo = 3;
					}
				}else if (C.OL_queryNo == 3){
					if ((noEntered == 0) || (noEntered == 1)){
						LRM.direction = noEntered;
						printf("\r\n---- 4. Setup Complete");
						printf("\r\n Distance = %5.2f,Duty = %04d,Direction = %01d",LRM.distance,LRM.duty,LRM.direction);
						printf("\r\n ---Controls---");
						printf("\r\n press 1 to start");
						printf("\r\n press 2 to stop (while running)");
						printf("\r\n press -1 to go back (while stopped)");
						printf("\r\n ---Data Format----");
						//Time.strokeVelocity,targetDist,EncActualDist,Error,GB_ActualDist,GBAbsPos,EncAbsPos
						printf("\r\n Time(sec),liftVelocity(mm/s),targetStrokePos,EncActualStroke,PIDError,GBActualStroke,GB_AbsPos,Enc_AbsPos,Duty,Current(A),RPM\r\n");
						C.OL_queryNo = 4;
					}
				}else if (C.OL_queryNo == 4){
					if (noEntered == 1){
						printf("\r\n ---START!---\r\n");
						Console_LiftOL_Start();
						C.runningOL = 1;
						C.logging = 1;
					}
				}
				noEntered  = 0;
			} // closed C.CL_running == 0
			else{
				if (noEntered == 2){
					printf("\r\n *** Stopping ***\r\n");
					Console_LiftOL_Stop();
					noEntered = 0;
					C.runningOL = 0;
				}
			}
		}//closes if noEntered != -1

	} // closes while
	return 1;
}

uint8_t zeroEncAbsPosition(void){
	int noEntered = 0;
	char firstTime = 1;
	while(1){
		if (firstTime == 1){
			printf("\r\n***ZERO ENC ABS POSITION****");
			printf("\r\n Press 1 to set EncPos as 0");
			printf("\r\n Press 2 to set EncPos equal the GB Abs Pos");
			printf("\r\n press '-1' to exit");
			firstTime = 0;
		}

		noEntered = waitForNoInput();

		if (noEntered == -1){
			break;
		}
		else if(noEntered == 1){
			printf("\r\n Enc Abs Position set as 0");
			encPos.absPosition = 0;
			noEntered = 0;
		}else if (noEntered == 2){
			printf("\r\n Enc Abs Position set equal to GB Abs Position : %6.2f",GB.correctedPosition);
			encPos.absPosition = GB.correctedPosition;
			noEntered = 0;
		}
	}
	return 1;
}

void configurationFromTerminal(void) {
    char firstTime = 1;
    int noEntered;
    while (1) {
        if (firstTime == 1) {
            printf("\r\n***MOTOR CONFIGURATION MENU***");
            printf("\r\n 1. View Motor Settings");
            printf("\r\n 2. Change Motor CAN ID");
            printf("\r\n 3. Change Motor Encoder Offset");
            printf("\r\n 4. Change Default Direction");
            printf("\r\n 5. Change PID Settings");
            printf("\r\n 6. Run Encoder Calibration Routine");
            printf("\r\n 7. Run GearBox Homing Setup Routine");
            printf("\r\n 8. Run GearBox Calibration Routine");
            printf("\r\n 9. GearBox Bin Management (View/Zero Bin Errors)");
            printf("\r\n 10. Toggle Lift Out Of Bounds Setting");
            printf("\r\n 11. Run EncPositioning ClosedLoop");
            printf("\r\n 12. Run EncPositioning OpenLoop");
            printf("\r\n 13. Zero EncAbsPosition");
            printf("\r\n 14. Restart Motor Code");
            printf("\r\n Enter a number between 1-14 and press enter to select an option");
            printf("\r\n Press -1 and enter to exit this menu \r\n");
            firstTime = 0;
        }

        noEntered = waitForNoInput();

        if (noEntered == -1) {
            printf("\r\n Bye!");
            printf("\r\n ************");
            break;
        }

        if (noEntered == 1) {
            firstTime = printSettings();
            noEntered = 0;
        }
        if (noEntered == 2) {
            firstTime = configureSettings(CONSOLE_CANID);
            noEntered = 0;
        }
        if (noEntered == 3) {
            firstTime = configureSettings(CONSOLE_ENCODER_OFFSET);
            noEntered = 0;
        }
        if (noEntered == 4) {
            firstTime = configureSettings(CONSOLE_DIRECTION);
            noEntered = 0;
        }
        if (noEntered == 5) {
            firstTime = configurePIDSettings();
            noEntered = 0;
        }
        if (noEntered == 6) {
            firstTime = runMotorCalibrationRoutine();
            noEntered = 0;
        }
        if (noEntered == 7) {
            firstTime = runGearBoxHomingRoutine();
            noEntered = 0;
        }
        if (noEntered == 8) {
            firstTime = runGearBoxCalibrationRoutine();
            noEntered = 0;
        }
        if (noEntered == 9) {
            firstTime = runGearBoxBinManagement();
            noEntered = 0;
        }
        if (noEntered == 10) {
            firstTime = ToggleLiftOutOfBoundsLimitSetting();
            noEntered = 0;
        }
        if (noEntered == 11) {
            firstTime = runLiftMotorClosedLoop();
            noEntered = 0;
        }
        if (noEntered == 12) {
            firstTime = runLiftMotorOpenLoop();
            noEntered = 0;
        }
        if (noEntered == 13) {
            firstTime = zeroEncAbsPosition();
            noEntered = 0;
        }
        if (noEntered == 14) {
            printf("\r\n Resetting Motor Code! Exiting Console also!");
            printf("\r\n Bye!");
            printf("\r\n ************");
            NVIC_SystemReset();
            noEntered = 0;
        }
    }
}

