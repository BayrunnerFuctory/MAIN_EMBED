#pragma once
#include "mbed.h"
#include "typedef.h"



extern uint16				sys_leftEncorderCount;
extern uint16				sys_rightEncorderCount;

extern int32                sys_leftEncorderState; //20160531 追加
extern int32                sys_rightEncorderState; //20160531 追加

extern flont_line_senser		sys_frontLineSenser;
extern rear_line_senser		sys_rearLineSenser;

extern BOOL					sys_enableStartSwitch;
extern modeswitch			sys_modeSwitch;

extern float32				sys_batteryVoltage;

extern BOOL 				DeviceInputInit(void);
extern BOOL 				DeviceInput(void);

extern int32                sys_ussBackEchoWidth;

extern float32              sys_frontSenserLL;
extern float32              sys_frontSenserCL;
extern float32              sys_frontSenserCC;
extern float32              sys_frontSenserCR;
extern float32              sys_frontSenserRR;
extern BOOL                 GetPhotoSenserData_Analog(void);

extern BOOL                 IsArmLimit(int,int);