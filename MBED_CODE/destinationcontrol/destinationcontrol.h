#pragma once
#include "mbed.h"
#include "typedef.h"


//-------------------------Destination control
extern void DistinationControlInit(void);
extern BOOL DistinationControl(void);

extern int16 sys_machineSpeed;
extern int16 sys_leftWheelSpeed;
extern int16 sys_rightWheelSpeed;
extern float64 sys_angularVelocity;
extern int16 sys_acceleration;
extern int32 sys_odoCount;
extern int32 sys_odoMeter;
extern int32 sys_tripCount;
extern int32 sys_tripmeter;
extern int32 sys_absoluteAxis_X;
extern int32 sys_absoluteAxis_Y;
extern int32 sys_relativeAxis_X;
extern int32 sys_relativeAxis_Y;
extern int32 sys_servoAngle;

extern uint16 param_encorderGauge;
extern int16  param_encCountErrorThreshold;
extern uint16 param_encorderWheelDiameter;
extern uint16 param_encorderResolution;;

extern uint16 param_errorLimitOfWheelSpeed;;
extern uint16 param_errorLimitOfAngularVelocity;
extern uint16 param_errorLimitOfAcceleration;


//---------------------------Input Data Adjust
extern BOOL InputDataAdjust(void);

