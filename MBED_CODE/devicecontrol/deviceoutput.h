#pragma once
#include "typedef.h"

/*
extern BOOL sys_onBordLED_1;
extern BOOL sys_onBordLED_2;
extern BOOL sys_onBordLED_3;
extern BOOL sys_onBordLED_4;
*/

extern int32 sys_servoAngle;       ///<サーボ角度 単位 : deg

extern void DeviceOutputInit();
extern BOOL DeviceOutput(void);
extern void DeviceOutputForceTerminate(void);
extern BOOL ArmOutput();

extern uint8 sys_USSCount;
extern uint32 sys_debugCount;


extern BOOL ServoOutput(void);