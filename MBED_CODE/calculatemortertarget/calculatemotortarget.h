#pragma once
#include "mbed.h"
#include "typedef.h"

extern motorDirection	sys_leftMotorDirection;
extern motorDirection	sys_rightMotorDirection;
extern uint16			sys_targetLeftMotorDuty;
extern uint16			sys_targetRightMotorDuty;
extern BOOL             sys_outInFlag;

extern void		CalculateMotorTargetInit(void);
extern BOOL		CalculateMotorTarget(void);

