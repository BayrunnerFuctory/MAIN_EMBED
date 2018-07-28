#include "mbed.h"
#include "typedef.h"
#include "sysdata.h"
#include "parameter.h"
#include "errorcontrol.h"
#include "calculatemotorrequest.h"

#include "calculatemotortarget.h"

uint16   sys_requestLeftMotorDuty;
uint16   sys_requestRightMotorDuty;
uint16   sys_requestServoPulseWidth;
uint16   sys_requestServoArmPulseWidth;
uint16   param_accelerate = 20;    ///<1ループでのモータの加減速値。この値を大きくすると目標値へ近づく時間が短くなります。(加速力、減速力が上がる)　単位0.1% 0.5%刻み -> 555 = 55.5%
int32   sys_servoAngle;

BOOL CalculateMotorRequest(void);


static BOOL LeftMotorRequestDutyCalculate(void);
static BOOL RightMotorRequestDutyCalculate(void);
static BOOL ServoRequestPulseCalculate(void);
/**

*/
BOOL CalculateMotorRequest()
{
    BOOL ret = FALSE;
    
    ret |= LeftMotorRequestDutyCalculate();
    ret |= RightMotorRequestDutyCalculate();
    if (sys_nowRouteState == RS_SYUMAI) {
        ret |= ServoRequestPulseCalculate();
    }
    
    return ret;
}



/**
@brief
左車輪の要求デューティー比を計算します。
この計算値がモーターへ出力されます。

@note
加減速比は固定。ホントはちゃんと計算した方がかっこいいけど
僕がめんどくさがりました。（コダマ）

@return F:エラー無し T:何らかのエラー有
*/
static BOOL LeftMotorRequestDutyCalculate()
{
    int32 delta;

    delta = (int16)sys_targetLeftMotorDuty - (int16)sys_requestLeftMotorDuty;
    if(delta > 0){          //加速要求
        if(delta > param_accelerate){
            sys_requestLeftMotorDuty += param_accelerate * 1.01; //20161008 調整
        }else{
            sys_requestLeftMotorDuty = sys_targetLeftMotorDuty;
        }
    }else if(delta < 0){    //減速要求
        if(delta < (param_accelerate * -1)){
            sys_requestLeftMotorDuty -= param_accelerate * 1.01;
        }else{
            sys_requestLeftMotorDuty = sys_targetLeftMotorDuty;
        }
    }else{                  //ターゲット到達
        //何もしない
    }

    return FALSE;
}



/**
@brief
右車輪の要求デューティー比を計算します。
この計算値がモーターへ出力されます。

@note
加減速比は固定。ホントはちゃんと計算した方がかっこいいけど
僕がめんどくさがりました。（コダマ）

@return F:エラー無し T:何らかのエラー有
*/
static BOOL RightMotorRequestDutyCalculate()
{
    //右だよ!! 間違えんなよ!!
    int16 delta;  

    delta = (int32)sys_targetRightMotorDuty - (int32)sys_requestRightMotorDuty;
    
    if(delta > 0){          //加速要求
        if(delta > param_accelerate){
            sys_requestRightMotorDuty += param_accelerate;
        }else{
            sys_requestRightMotorDuty = sys_targetRightMotorDuty;
        }
    }else if(delta < 0){    //減速要求
        if(delta < (param_accelerate*-1)){
            sys_requestRightMotorDuty -= param_accelerate; //2016 調整
        }else{
            sys_requestRightMotorDuty = sys_targetRightMotorDuty;
        }
     
    }else{                  //ターゲット到達
        //何もしない
    }

    return FALSE;
}

/**
@brief
サーボのパルス幅を計算します。
この計算値がサーボへ出力されます。

@note
暫定でパルス幅計算、デューティーへ変更も検討

@return F:エラー無し T:何らかのエラー有
*/
static BOOL ServoRequestPulseCalculate()
{
    //サーボです!!
    int16 unit_servoPulseWidth = 1000;

    sys_requestServoPulseWidth = sys_servoAngle * unit_servoPulseWidth;

    return FALSE;
}
