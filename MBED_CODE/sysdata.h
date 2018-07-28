#pragma once
#include "typedef.h"


//main
extern uint32  sys_taskCount;                           ///< メインループカウンタ
extern BOOL    sys_onError;                             ///<システムエラーフラグ


//device input
///フォトセンサ確定データ
extern flont_line_senser sys_frontLineSenser;
extern rear_line_senser sys_rearLineSenser;

///エンコーダのカウント値
extern uint16 sys_leftEncorderCount;
extern uint16 sys_rightEncorderCount;

///エンコーダの現在状態 20160531 追加
extern int32 sys_leftEncorderState;
extern int32 sys_rightEncorderState;

///スタートスイッチのデータ
extern BOOL sys_enableStartSwitch;

extern BOOL sys_goalCheckBtnL;
extern BOOL sys_goalCheckBtnR;

//モーターバッテリー電圧
extern float32 sys_batteryVoltage;  //単位V



//destinationcontrol
extern int16 sys_machineSpeed;                          ///<現在車速     v  単位:mm/sec
extern int16 sys_leftWheelSpeed;                        ///<左車輪速度 vL 単位:mm/sec
extern int16 sys_rightWheelSpeed;                       ///<右車輪速度 vR 単位:mm/sec
extern float64 sys_angularVelocity;                     ///<車体角速度 ω 単位:rad/sec
extern float64 sys_angularRadian;                       ///<車体角度 単位:rad
extern float64 sys_angularDegree;                       ///<車体角度 単位:度 rad*57.2958
extern int16 sys_acceleration;                          ///<車体加速度 単位:mm/sec^2
extern int32 sys_odoCount;                              ///<累積左右合算エンコーダカウント
extern int32 sys_odoMeter;                              ///<累積走行距離 単位:mm
extern int32 sys_tripCount;                             ///<区間左右合算エンコーダカウント
extern int32 sys_tripmeter;                             ///<区間走行距離 単位:mm
extern int32 sys_absoluteAxis_X;                        ///<絶対座標X 単位：mm
extern int32 sys_absoluteAxis_Y;                        ///<絶対座標Y 単位：mm
extern int32 sys_relativeAxis_X;                        ///<相対座標X 単位：mm
extern int32 sys_relativeAxis_Y;                        ///<相対座標Y 単位：mm

//devicecontrol
extern int32 sys_servoAngle;                            ///<サーボ角度 単位:deg

//judgestate
extern route_state  sys_nowRouteState;                  ///< 現在の位置
extern route_state  sys_lastRouteState;                 ///< 前回の位置（ラインロスト時に利用）
extern route_state  sys_firstState;                     ///< スタートスイッチを押した後のステート
extern int32        sys_toGoalSubState;
extern int32        sys_loggingVCPAxisX[10];            ///< 絶対座標ログX
extern int32        sys_loggingVCPAxisY[10];            ///< 絶対座標ログY

//caluculatesteer
extern steer_order sys_steerOrderLog[5];                ///デバッグ用操舵変更要求ログ（変更した時のみログが残ります）
extern steer_order  sys_nowSteerOrder;                  ///< 現在の操舵要求

//CaluculateTargetSpeed
extern int16 sys_targetSpeed;                           ///<目標車速 単位 mm/sec


//calculatemotortarget
#define VOLTAGE_MAP_INDEX_MAX (5)
#define SPEED_MAP_INDEX_MAX   (6)
extern motorDirection sys_leftMotorDirection;               ///<左モーターの回転方向
extern motorDirection sys_rightMotorDirection;              ///<右モーターの回転方向
extern uint16 sys_targetLeftMotorSpeed;                     ///<左モーターの目標速度 単位mm/sec
extern uint16 sys_targetRightMotorSpeed;                    ///<右モーターの目標速度 単位mm/sec
extern uint16 sys_targetLeftMotorDuty;                      ///<モーターへ伝えるDuty比の目標値 0.0%　～ 100.0% 0.1%単位　-> 555 = 55.5%
extern uint16 sys_targetRightMotorDuty;                     ///<モーターへ伝えるDuty比の目標値 0.0%　～ 100.0% 0.1%単位　-> 443 = 44.3%
extern uint16 sys_speedVsVoltageMap[SPEED_MAP_INDEX_MAX];   ///<現在の電圧での速度対Duty比マップ
extern BOOL   sys_outInFlag;                                ///登坂終了時にラインの左側にいるかの判定用

//calculatemotorrequest
extern uint16 sys_requestLeftMotorDuty;                     ///<左モーター要求Duty比 0.0%　～ 100.0% 0.1%単位　-> 555 = 55.5%
extern uint16 sys_requestRightMotorDuty;                    ///<右モーター要求Duty比 0.0%　～ 100.0% 0.1%単位　-> 443 = 44.3%
extern uint16 sys_requestServoPulseWidth;                   ///<サーボ要求パルス幅 1000us～18000us
extern uint16 sys_requestServoArmPulseWidth; 
