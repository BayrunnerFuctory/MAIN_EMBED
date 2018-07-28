#include "mbed.h"
#include "typedef.h"
#include "communication.h"
#include "parameter.h"

#include "debugsupport.h"

#include "deviceinput.h"
#include "deviceoutput.h"
#include "sysdata.h"

static const uint16 DUTY_MAX  = 1001;
static const uint16 SPEED_MAX = 10000;
static const uint16 ANGLE_MAX = 181;

     
BOOL DebugDeviceInputDataOverWrite(void);
BOOL DebugDestinationDataOverWrite(void);
BOOL DebugRequestOverWrite(void);
BOOL DebugTargetOverWrite(void);
/////////////////////////////////////////////////
void DebugPrint(uint8* printfStr, ...);
void DebugMemdump_Hex(uint8* data, uint32 size);
void DebugMemdump_Dec(uint8* data, uint32 size);


//上書き制御
BOOL d_sys_InputOverWrite;
BOOL d_sys_DestinationDataOverWrite;
BOOL d_sys_RequestOverWrite;
BOOL d_sys_TargetOverWrite;
BOOL d_sys_OneTimeInputOverWrite;
BOOL d_sys_OneTimeDestinationDataOverWrite;
BOOL d_sys_OneTimeRequestOverWrite;
BOOL d_sys_OneTimeTargetOverWrite;


//モータ出力(6)
uint8           d_param_motorPeriod = 0;                        ///モーターのPWM周期：ms単位
motorDirection  d_sys_leftMotorDirection    = MD_UNNOWN;        ///<左モーターの回転方向
motorDirection  d_sys_rightMotorDirection   = MD_UNNOWN;        ///<右モーターの回転方向
uint16          d_sys_requestLeftMotorDuty  = DUTY_MAX;           ///<左モーター要求Duty比 0.0%　～ 100.0% 0.1%単位　-> 555 = 55.5%
uint16          d_sys_requestRightMotorDuty = DUTY_MAX;           ///<右モーター要求Duty比 0.0%　～ 100.0% 0.1%単位　-> 443 = 44.3%


//モータ目標(4)
uint16 d_sys_targetLeftMotorSpeed = 0;                     ///<左モーターの目標速度 単位mm/sec
uint16 d_sys_targetRightMotorSpeed = 0;                    ///<右モーターの目標速度 単位mm/sec
uint16 d_sys_targetLeftMotorDuty = DUTY_MAX;                      ///<モーターへ伝えるDuty比の目標値 0.0%　～ 100.0% 0.1%単位　-> 555 = 55.5%
uint16 d_sys_targetRightMotorDuty = DUTY_MAX;                     ///<モーターへ伝えるDuty比の目標値 0.0%　～ 100.0% 0.1%単位　-> 443 = 44.3%

//ラインセンサ(4)
int32 d_sys_frontLineSenser = INT32_MIN;                        ///前ラインセンサ 4
int32 d_sys_rearLineSenser = INT32_MIN;                         ///後ラインセンサ 2

//エンコーダ(6)
int   d_sys_leftEncorderCount = 0;
int   d_sys_rightEncorderCount = 0;
int32 d_sys_odoCount = INT32_MIN;                              ///<累積左右合算エンコーダカウント
int32 d_sys_odoMeter = INT32_MIN;                              ///<累積走行距離 単位:mm
int32 d_sys_tripCount = INT32_MIN;                             ///<区間左右合算エンコーダカウント
int32 d_sys_tripmeter = INT32_MIN;                             ///<区間走行距離 単位:mm

//内部状態(24)
int16        d_sys_enableStartSwitch = 0;                       ///スタートスイッチの状態
int16        d_sys_machineSpeed = SPEED_MAX;                    ///<現在車速     v  単位:mm/sec
int16        d_sys_leftWheelSpeed = SPEED_MAX;                  ///<左車輪速度 vL 単位:mm/sec
int16        d_sys_rightWheelSpeed = SPEED_MAX;                 ///<右車輪速度 vR 単位:mm/sec
float64      d_sys_angularVelocity = SPEED_MAX;                 ///<車体角速度 ω 単位:rad/sec
int16        d_sys_acceleration = 0;                            ///<車体加速度 単位:mm/sec^2
int32        d_sys_absoluteAxis_X = 0;                          ///<絶対座標X 単位：mm
int32        d_sys_absoluteAxis_Y = 0;                          ///<絶対座標Y 単位：mm
int32        d_sys_relativeAxis_X = 0;                          ///<相対座標X 単位：mm
int32        d_sys_relativeAxis_Y = 0;                          ///<相対座標Y 単位：mm
route_state  d_sys_nowRouteState = RS_UNKNOWN;                  ///< 現在の位置
route_state  d_sys_lastRouteState = RS_UNKNOWN;                 ///< 前回の位置（ラインロスト時に利用）
route_state  d_sys_firstState = RS_UNKNOWN;                     ///< スタートスイッチを押した後のステート
steer_order  d_sys_nowSteerOrder = SO_ST;                       ///< 現在の操舵要求
int16        d_sys_targetSpeed = SPEED_MAX;                     ///<目標車速 単位 mm/sec
int32        d_param_distance1stVCPAxisX = 0;                   ///<1stVCP
int32        d_param_distance2ndVCPAxisX = 0;                   ///<2ndVCP
int32        d_param_distanceSlopCenterAxisX = 0;               ///坂道の真ん中辺り
int32        d_param_distance6thVCPignoreRelativeAxisY = 0;     ///6thVCPを判断するにあたり、センサー反応を無視するY軸方向相対距離
int32        d_param_distance7thVCPignoreRelativeAxisX = 0;     ///7thVCPを判断するにあたり、センサー反応を無視するX軸方向相対距離
int32        d_sys_servoAngle = 0;                              ///サーボ角度 単位 : deg
float32      d_sys_batteryVoltage = 0;                              ///電圧 単位V
//uint16       d_sys_speedVsVoltageMap[SPEED_MAP_INDEX_MAX];  ///<現在の電圧での速度対Duty比マップ
//int32        d_sys_loggingVCPAxisX[10];                     ///< 絶対座標ログX
//int32        d_sys_loggingVCPAxisY[10];                     ///< 絶対座標ログY
//steer_order  d_sys_steerOrderLog[5];                        ///デバッグ用操舵変更要求ログ（変更した時のみログが残ります）


void DebugMemdump_Hex(uint8* data, uint32 size)
{
    
}

void DebugMemdump_Dec(uint8* data, uint32 size)
{
    
}
/**
@brief
判断情報上書き
上書き許可と上書き値が初期値でない場合のみ上書きする
@note
書き換え対象：現在車速、左車輪速度、右車輪速度、車体角速度、車体加速度、現在の操舵要求、目標車速
1回のみ書き換え対象:スタートスイッチ、絶対座標(X Y)、相対座標(X Y)、現在位置、前回位置、スタートスイッチ後の位置、
                    VCP1のX座標、VCP2のX座標、坂道のX座標、VCP6センサー無視判断のY座標、VCP7センサー無視判断のX座標
*/
BOOL DebugDestinationDataOverWrite()
{
    if(d_sys_OneTimeDestinationDataOverWrite == TRUE)
    {
        d_sys_OneTimeDestinationDataOverWrite = FALSE;
        
        if(d_sys_enableStartSwitch != 0)
        { sys_enableStartSwitch = d_sys_enableStartSwitch; }
        
//        if(d_sys_absoluteAxis_X != 0)
//        { sys_absoluteAxis_X = d_sys_absoluteAxis_X; }
//        
//        if(d_sys_absoluteAxis_Y != 0)
//        { sys_absoluteAxis_Y = d_sys_absoluteAxis_Y; }
        
        if(d_sys_relativeAxis_X != 0)
        { sys_relativeAxis_X = d_sys_relativeAxis_X; }
        
        if(d_sys_relativeAxis_Y != 0)
        { sys_relativeAxis_Y = d_sys_relativeAxis_Y; }
        
        if(d_sys_nowRouteState != RS_UNKNOWN)
        { sys_nowRouteState = d_sys_nowRouteState; }
        
        if(d_sys_lastRouteState != RS_UNKNOWN)
        {sys_lastRouteState = d_sys_lastRouteState; }
                
        if(d_sys_firstState != RS_UNKNOWN)
        {sys_firstState = d_sys_firstState; }
        /*
        if(d_param_distance1stVCPAxisX != 0)
        { param_distance1stVCPAxisX = d_param_distance1stVCPAxisX; }
        
        if(d_param_distance2ndVCPAxisX != 0)
        { param_distance2ndVCPAxisX = d_param_distance2ndVCPAxisX; }
        
        if(d_param_distanceSlopCenterAxisX != 0)
        { param_distanceSlopCenterAxisX = d_param_distanceSlopCenterAxisX; }
        
        if(d_param_distance6thVCPignoreRelativeAxisY != 0)
        { param_distance6thVCPignoreRelativeAxisY = d_param_distance6thVCPignoreRelativeAxisY; }
        
        if(d_param_distance7thVCPignoreRelativeAxisX != 0)
        { param_distance7thVCPignoreRelativeAxisX = d_param_distance7thVCPignoreRelativeAxisX; } 
*/
    }
    
    if(d_sys_DestinationDataOverWrite == TRUE)
    {
        if(d_sys_machineSpeed != SPEED_MAX )
        { sys_machineSpeed = d_sys_machineSpeed; }
        
        if(d_sys_leftWheelSpeed != SPEED_MAX )
        { sys_leftWheelSpeed = d_sys_leftWheelSpeed; }
        
        if( d_sys_rightWheelSpeed != SPEED_MAX)
        { sys_rightWheelSpeed = d_sys_rightWheelSpeed; }
        
        if(d_sys_angularVelocity != SPEED_MAX)
        { sys_angularVelocity = d_sys_angularVelocity; }
        
        if(d_sys_acceleration != 0)
        { sys_acceleration = d_sys_acceleration; }
        
        if(d_sys_nowSteerOrder != RS_UNKNOWN)
        { sys_nowSteerOrder = d_sys_nowSteerOrder; }
        
        if(d_sys_targetSpeed != SPEED_MAX)
        { sys_targetSpeed = d_sys_targetSpeed; }
        
        //if(d_sys_servoAngle != ANGLE_MAX)
        //{ sys_servoAngle = d_sys_servoAngle; }
        
    }
    
    return FALSE;
}

/**
@brief
モータ要求上書き
上書き許可と上書き値が初期値でない場合のみ上書きする
@note
書き換え対象：モータ回転方向(左右)、モータ要求duty比(左右)
1回のみ書き換え対象:PWM周期
*/
BOOL DebugRequestOverWrite()
{   
    if(d_sys_OneTimeRequestOverWrite == TRUE)
    {
        d_sys_OneTimeRequestOverWrite = FALSE;
        
//        if(d_param_motorPeriod != 0)
//        { param_motorPeriod = d_param_motorPeriod; }
    }
    
    if(d_sys_RequestOverWrite == TRUE)
    {
        if(d_sys_requestLeftMotorDuty < DUTY_MAX)
        { sys_requestLeftMotorDuty = d_sys_requestLeftMotorDuty; }
        
        if(d_sys_requestRightMotorDuty < DUTY_MAX)
        { sys_requestRightMotorDuty = d_sys_requestRightMotorDuty; }
    
        if(d_sys_leftMotorDirection != MD_UNNOWN)
        { sys_leftMotorDirection = d_sys_leftMotorDirection; }
        
        if(d_sys_rightMotorDirection != MD_UNNOWN)
        { sys_rightMotorDirection = d_sys_rightMotorDirection; }
    }
    return FALSE;
}
/**
@brief
モータ目標値上書き
上書き許可と上書き値が初期値でない場合のみ上書きする
@note
書き換え対象：モータ目標速度(左右)、モータ目標duty比(左右)
1回のみ書き換え対象:-
*/
BOOL DebugTargetOverWrite()
{
    
    //201810
    sys_absoluteAxis_X = d_sys_absoluteAxis_X;
    sys_absoluteAxis_Y = d_sys_absoluteAxis_Y;
    
    if(d_sys_OneTimeTargetOverWrite == TRUE)
    {
        d_sys_OneTimeTargetOverWrite = FALSE;
    }
    
    if(d_sys_TargetOverWrite == TRUE)
    {
        if(d_sys_targetLeftMotorSpeed != 0)
        { sys_targetLeftMotorSpeed = d_sys_targetLeftMotorSpeed; }
        
        if(d_sys_targetRightMotorSpeed != 0)
        { sys_targetRightMotorSpeed = d_sys_targetRightMotorSpeed; }
        
        if( d_sys_targetLeftMotorDuty < DUTY_MAX)
        { sys_targetLeftMotorDuty = d_sys_targetLeftMotorDuty; }
        
        if(d_sys_targetRightMotorDuty < DUTY_MAX)
        { sys_targetRightMotorDuty = d_sys_targetRightMotorDuty;}
        
        //201708
        if(d_sys_leftMotorDirection != 0)
        {sys_leftMotorDirection=d_sys_leftMotorDirection;}
        
        if(d_sys_rightMotorDirection != 0)
        {sys_rightMotorDirection = d_sys_rightMotorDirection;}
        
        //201709 サーボ操作（動画用）
        if(d_sys_servoAngle != 0)
        { sys_requestServoPulseWidth = d_sys_servoAngle;}
        else
        { sys_requestServoPulseWidth = 0;}
        
        if(d_param_distance7thVCPignoreRelativeAxisX != 0)
        { sys_requestServoArmPulseWidth = d_param_distance7thVCPignoreRelativeAxisX; }
        else
        { sys_requestServoArmPulseWidth = 0;}
        
        if(d_sys_odoCount != INT32_MIN)
        { sys_odoCount = d_sys_odoCount; }
        
        if(d_sys_odoMeter != INT32_MIN)
        { sys_odoMeter = d_sys_odoMeter; }
    }
    
    return FALSE;
}

/**
@brief
入力デバイス値上書き
上書き許可と上書き値が初期値でない場合のみ上書きする
@note
書き換え対象：前センサ、後センサ、左右エンコーダカウント
1回のみ書き換え対象：エンコーダ左右累積、左右走行合算
*/
BOOL DebugDeviceInputDataOverWrite()
{
    if(d_sys_OneTimeInputOverWrite == TRUE)
    {     
        d_sys_OneTimeInputOverWrite = FALSE;
        
        if(d_sys_odoCount != INT32_MIN)
        { sys_odoCount = d_sys_odoCount; }
        
        if(d_sys_odoMeter != INT32_MIN)
        { sys_odoMeter = d_sys_odoMeter; }
        
        if(d_sys_tripCount != INT32_MIN)
        { sys_tripCount = d_sys_tripCount; }
        
        if(d_sys_tripmeter != INT32_MIN)
        { sys_tripmeter = d_sys_tripmeter;}
    }
    
    if(d_sys_InputOverWrite == TRUE)
    {
        if(d_sys_frontLineSenser != INT32_MIN)
        { sys_frontLineSenser = (flont_line_senser)d_sys_frontLineSenser; }
        
        if(d_sys_rearLineSenser != INT32_MIN)
        { sys_rearLineSenser = (rear_line_senser)d_sys_rearLineSenser; }
        
        if(d_sys_leftEncorderCount != 0)
        { sys_leftEncorderCount = d_sys_leftEncorderCount; }
        
        if(d_sys_rightEncorderCount != 0)
        { sys_rightEncorderCount = d_sys_rightEncorderCount; }
    }
    
    return FALSE;
}


