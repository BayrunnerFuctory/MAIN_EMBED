#pragma once
#include "mbed.h"
#include "typedef.h"
#include "communication.h"


#define _DEBUG

#ifdef _DEBUG
#define debug_printf(fmt , ...)       comm.printf(fmt, __VA_ARGS__)
#else
#define debug_printf(...)
#endif

BOOL DebugDeviceInputDataOverWrite(void);
BOOL DebugDestinationDataOverWrite(void);
BOOL DebugRequestOverWrite(void);
BOOL DebugTargetOverWrite(void);

//上書き制御
extern BOOL d_sys_InputOverWrite;
extern BOOL d_sys_DestinationDataOverWrite;
extern BOOL d_sys_RequestOverWrite;
extern BOOL d_sys_TargetOverWrite;
extern BOOL d_sys_OneTimeInputOverWrite;
extern BOOL d_sys_OneTimeDestinationDataOverWrite;
extern BOOL d_sys_OneTimeRequestOverWrite;
extern BOOL d_sys_OneTimeTargetOverWrite;


//モータ出力(6)
extern uint8           d_param_motorPeriod;                ///モーターのPWM周期：ms単位
extern motorDirection  d_sys_leftMotorDirection;           ///<左モーターの回転方向
extern motorDirection  d_sys_rightMotorDirection;          ///<右モーターの回転方向
extern uint16          d_sys_requestLeftMotorDuty;         ///<左モーター要求Duty比 0.0%　～ 100.0% 0.1%単位　-> 555 = 55.5%
extern uint16          d_sys_requestRightMotorDuty;        ///<右モーター要求Duty比 0.0%　～ 100.0% 0.1%単位　-> 443 = 44.3%


//モータ目標(4)
extern uint16 d_sys_targetLeftMotorSpeed;                     ///<左モーターの目標速度 単位mm/sec
extern uint16 d_sys_targetRightMotorSpeed;                    ///<右モーターの目標速度 単位mm/sec
extern uint16 d_sys_targetLeftMotorDuty;                      ///<モーターへ伝えるDuty比の目標値 0.0%　～ 100.0% 0.1%単位　-> 555 = 55.5%
extern uint16 d_sys_targetRightMotorDuty;                     ///<モーターへ伝えるDuty比の目標値 0.0%　～ 100.0% 0.1%単位　-> 443 = 44.3%

//ラインセンサ(4)
extern int32 d_sys_frontLineSenser;                        ///前ラインセンサ 4
extern int32 d_sys_rearLineSenser;                         ///後ラインセンサ 2

//エンコーダ(6)
extern int   d_sys_leftEncorderCount;
extern int   d_sys_rightEncorderCount;
extern int32 d_sys_odoCount;                              ///<累積左右合算エンコーダカウント
extern int32 d_sys_odoMeter;                              ///<累積走行距離 単位:mm
extern int32 d_sys_tripCount;                             ///<区間左右合算エンコーダカウント
extern int32 d_sys_tripmeter;                             ///<区間走行距離 単位:mm

//内部状態(24)
extern int16        d_sys_enableStartSwitch;                       ///スタートスイッチの状態
extern int16        d_sys_machineSpeed;                            ///<現在車速     v  単位:mm/sec
extern int16        d_sys_leftWheelSpeed;                          ///<左車輪速度 vL 単位:mm/sec
extern int16        d_sys_rightWheelSpeed;                         ///<右車輪速度 vR 単位:mm/sec
extern float64      d_sys_angularVelocity;                         ///<車体角速度 ω 単位:rad/sec
extern int16        d_sys_acceleration;                            ///<車体加速度 単位:mm/sec^2
extern int32        d_sys_absoluteAxis_X;                          ///<絶対座標X 単位：mm
extern int32        d_sys_absoluteAxis_Y;                          ///<絶対座標Y 単位：mm
extern int32        d_sys_relativeAxis_X;                          ///<相対座標X 単位：mm
extern int32        d_sys_relativeAxis_Y;                          ///<相対座標Y 単位：mm
extern route_state  d_sys_nowRouteState;                           ///< 現在の位置
extern route_state  d_sys_lastRouteState;                          ///< 前回の位置（ラインロスト時に利用）
extern route_state  d_sys_firstState;                              ///< スタートスイッチを押した後のステート
extern steer_order  d_sys_nowSteerOrder;                           ///< 現在の操舵要求
extern int16        d_sys_targetSpeed;                             ///<目標車速 単位 mm/sec
extern int32        d_param_distance1stVCPAxisX;                   ///<1stVCP
extern int32        d_param_distance2ndVCPAxisX;                   ///<2ndVCP
extern int32        d_param_distanceSlopCenterAxisX;               ///坂道の真ん中辺り
extern int32        d_param_distance6thVCPignoreRelativeAxisY;     ///6thVCPを判断するにあたり、センサー反応を無視するY軸方向相対距離
extern int32        d_param_distance7thVCPignoreRelativeAxisX;     ///7thVCPを判断するにあたり、センサー反応を無視するX軸方向相対距離
extern int32        d_param_distance7thVCPignoreRelativeAxisX;     ///7thVCPを判断するにあたり、センサー反応を無視するX軸方向相対距離
extern int32        d_sys_servoAngle;                              ///<サーボ角度 単位 : deg

/////////////////////////////////////////////////
extern BOOL sys_enableDeviceInputOverWrite;
extern BOOL sys_enablePWMOutOverWrite;
extern BOOL sys_enableStateOverWrite;
extern BOOL sys_enableTargetOverWrite;

//デバイスインプット関連
extern int32 ow_sys_lineSenser;
extern int32 ow_sys_motorBattVoltage;
extern int32 ow_sys_modeSwitch;

//モーター出力関連
extern int32 ow_leftMotorDirection;
extern int32 ow_rightMotorDirection;
extern int32 ow_requestLeftMotorDuty;
extern int32 ow_requestRightMotorDuty;

//ステート判断関連
extern int32 ow_sys_nowSteerOrder;
extern int32 ow_sys_nowRouteState;

//ターゲットスピード関連
extern int32 ow_sys_targetSpeed;
extern int32 ow_sys_targetLeftMotorDuty;
extern int32 ow_sys_targetRightMotorDuty;