#pragma once
#include "mbed.h"
#include "typedef.h"



extern uint16   sys_requestLeftMotorDuty;
extern uint16   sys_requestRightMotorDuty;
extern uint16   sys_requestServoPulseWidth;
extern uint16   sys_requestServoArmPulseWidth;

extern BOOL CalculateMotorRequest(void);
