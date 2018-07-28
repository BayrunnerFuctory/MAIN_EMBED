#include "mbed.h"
#include "typedef.h"
#include "sysdata.h"
#include "parameter.h"
#include "errorcontrol.h"
#include "deviceoutput.h"
#include "bonusarea.h"


#include "deviceinput.h"
#include "calculatemotortarget.h"
#include "calculatemotorrequest.h"

uint32  sys_debugCount;
uint8   sys_USSCount;		///超音波センサカウンタ 60msごとに発信(12カウントごと)

uint8 param_servoMotorPeriod = 20;	//サーボモーターのPWM周期：ms単位
uint8 param_motorPeriod = 1;	//モーターのPWM周期：ms単位
uint8 param_Counter = 0;

//201710
static DigitalOut ussLeftTriger(p9);		///超音波センサLeftトリガ発信
static DigitalOut ussBackTriger(p10);		///超音波センサBackトリガ発信
static DigitalOut ussRightTriger(p11);		///超音波センサRightトリガ発信

static DigitalOut armLeft(p5);
static DigitalOut armRight(p6);
static DigitalOut armGetLeft(p7);
static DigitalOut armGetRight(p8);

static PwmOut pwm_L1(p22);
static PwmOut pwm_L2(p21);
static PwmOut pwm_R1(p24);
static PwmOut pwm_R2(p23);
static PwmOut pwm_servo_arm(p25);
static PwmOut pwm_servo(p26);
//static DigitalOut motorShutdown(p29);

void DeviceInit();
BOOL DeviceOutput();

static BOOL PWMOutput();
BOOL ServoOutput();
static BOOL ArmOutput();

/**
@brief
デバイス出力の初期化を行う

主にモーター出力のパルス設定と、モータードライバ
シャットダウン信号の設定

*/
void DeviceOutputInit()
{
	pwm_L1.period_ms(param_motorPeriod);
	pwm_L1.pulsewidth_us(0);
	pwm_L2.period_ms(param_motorPeriod);
	pwm_L2.pulsewidth_us(0);

	pwm_R1.period_ms(param_motorPeriod);
	pwm_R1.pulsewidth_us(0);
	pwm_R2.period_ms(param_motorPeriod);
	pwm_R2.pulsewidth_us(0);

    //pwm_servo.period_ms(16);
	//pwm_servo.pulsewidth_us(0);
	pwm_servo.pulsewidth_us(1000);
	//pwm_servo_arm.pulsewidth_us(500);

	//motorShutdown = 1;
	armLeft=0;
	armRight=0;
	armGetLeft=1;
	armGetRight=0;
	
	sys_USSCount=0;
	sys_debugCount=0;
}


/**
@brief
デバイスへの出力処理のメイン関数です。
マイコンボードに接続されている各種デバイスへ信号を出力します。
(PCへのシリアル通信は別モジュールです。)
@return FALSE:エラー無し TRUE:何らかのエラー
@note	ペットボトルを置く動作の場合にのみサーボを動かします。
*/
BOOL DeviceOutput()
{
	BOOL onErr = FALSE;
	
	if(sys_USSCount % 20 == 0)
	{
		ussBackTriger = 0;
		wait_us(5);
		ussBackTriger = 1;
		wait_us(10);
		ussBackTriger = 0;
	}
	if(sys_USSCount % 25 == 1)
	{
		//ussLeftTriger = 0;
		//wait_us(5);
		//ussLeftTriger = 1;
		//wait_us(10);
		//ussLeftTriger = 0;
	}
	if(sys_USSCount % 25 == 2)
	{
		//ussRightTriger = 0;
		//wait_us(5);
		//ussRightTriger = 1;
		//wait_us(10);
		//ussRightTriger = 0;
	}
	
	//onErr |= ServoOutput();
	onErr |= PWMOutput();
	onErr |= ArmOutput();
	
	if(sys_requestServoPulseWidth != 0)
	{
		//onErr |= ServoOutput();
	}
	
	if(b_HavingBottle_flag == TRUE)
	{
		//onErr |= PWMOutput();
		//onErr |= ServoOutput();
	}
	else
	{
		//onErr |= ServoOutput();
	}
	return onErr;
}


/**
@brief
モータの回転方向を決める信号を出力します。
@return F:成功 T:失敗
@note
L1 : H  
L2 : L 
で前進

R1 : H
R2 : L 
で前進
です。

L1: L L2:L でイナーシャー（慣性）
L1: H L2:H で電気ブレーキ

100%電気ブレーキはエレキ側から禁止されてます。

*/
static BOOL PWMOutput()
{
	BOOL err = FALSE;
	int32 ld, rd;

	//周波数に対して千分率の値でHiの割合を求める (ex 20ms *  0.2(20% = 200) = 4msec = 4000usec
	//
	ld = (int32)((float32)param_motorPeriod * ((float32)sys_requestLeftMotorDuty / (float32)1000.0) * (float32)1000.0);
	rd = (int32)((float32)param_motorPeriod * ((float32)sys_requestRightMotorDuty / (float32)1000.0) * (float32)1000.0); //左モーターとの出力調整
	
	//20160917 debug 100 um * 200 / 1000 
	//ld = (int32)(((float32)sys_requestLeftMotorDuty / (float32)1000.0) * (float32)1000.0);
	//rd = (int32)(((float32)sys_requestRightMotorDuty / (float32)1000.0) * (float32)1000.0);


	//左側出力
	//前進
	if (sys_leftMotorDirection == MD_FORWORD)
	{
		pwm_L1.pulsewidth_us(ld);
		pwm_L2.pulsewidth_us(0);

	}//後進
	else if (sys_leftMotorDirection == MD_BACK)
	{
		pwm_L1.pulsewidth_us(0);
		pwm_L2.pulsewidth_us(ld);

	}//ブレーキ
	else if (sys_leftMotorDirection == MD_BRAKE)
	{
		//pwm_L1.pulsewidth_us(100);
		//pwm_L2.pulsewidth_us(100);

	}//惰性
	else if (sys_leftMotorDirection == MD_INERTIA)
	{
		//pwm_L1.pulsewidth_us(0);
		//pwm_L2.pulsewidth_us(0);

	}//エラー
	else
	{
		//err = TRUE;
		/* エラー時におけるデバイスの処理は、エラー処理専用モジュールで行うので、
		 ここでは信号処理を行わなくても大丈夫です。 */
		//ERR_WriteID(EID_UNKNOWN_TYPE, __func__, __LINE__);
	}

	//右側出力
	//前進
	if (sys_rightMotorDirection == MD_FORWORD)
	{
		pwm_R1.pulsewidth_us(rd);
		pwm_R2.pulsewidth_us(0);

	}//後進
	else if (sys_rightMotorDirection == MD_BACK)
	{
		pwm_R1.pulsewidth_us(0);
		pwm_R2.pulsewidth_us(rd);

	}//ブレーキ
	else if (sys_rightMotorDirection == MD_BRAKE)
	{
		//pwm_R1.pulsewidth_us(100);
		//pwm_R2.pulsewidth_us(100);

	}//惰性
	else if (sys_rightMotorDirection == MD_INERTIA)
	{
		//pwm_R1.pulsewidth_us(0);
		//pwm_R2.pulsewidth_us(0);

	}//エラー
	else
	{
		//err = TRUE;

		/* エラー時におけるデバイスの処理は、エラー処理専用モジュールで行うので、
		 ここでは信号処理を行わなくても大丈夫です。 */
		//ERR_WriteID(EID_UNKNOWN_TYPE, __func__, __LINE__);
	}

	return err;
}

/**
@brief
サーボの角度を決める信号を出力します。

@note
パルス幅を調節することにより、角度を変更します。
*/
BOOL ServoOutput()
{
	int16 spw;
	//param_Counter++;
	
	//if(param_Counter == 4)
	//{
	//	param_Counter = 0;
	//}
	
	//sys_requestServoPulseWidth
	
	//pwm_servo.pulsewidth_us(1000);
	
	if(sys_taskCount % 4  == 0)
	{
		//pwm_servo.write(0.01);
		if(sys_requestServoPulseWidth > 1000)
		{
			pwm_servo.pulsewidth_us(1000);
			wait_us(sys_requestServoPulseWidth);
			pwm_servo.pulsewidth_us(0);
		}
		else
		{
			pwm_servo.pulsewidth_us(sys_requestServoPulseWidth);
			wait_us(sys_requestServoPulseWidth);
			pwm_servo.pulsewidth_us(0);			
		}
		//pwm_servo.pulsewidth_ms(1000);
		//pwm_servo_arm.pulsewidth_us(0);
		//pwm_servo.pulsewidth_us(1000);
	}
	else if(sys_taskCount % 4  == 1)
	{
		//pwm_servo.write(0.01);
		//pwm_servo.plusewidth_ms(sys_requestServoPulseWidth / 1000)
		//pwm_servo.pulsewidth_us(sys_requestServoPulseWidth);
		//wait_ms(sys_requestServoPulseWidth / 1000);
		//pwm_servo.pulsewidth_us(0);
		//pwm_servo.pulsewidth_ms(1000);
		//pwm_servo_arm.pulsewidth_us(0);
		//pwm_servo.pulsewidth_us(1000);
	}
	else
	{
		//pwm_servo.write(0);
		pwm_servo.pulsewidth_us(0);
		//pwm_servo.pulsewidth_us(0);
		//pwm_servo_arm.pulsewidth_us(0);
		//pwm_servo.pulsewidth_us(0);
	}
	
	//pwm_servo.period_ms(1);
	//pwm_servo.pulsewidth_us(spw);
	//pwm_servo.period_ms(param_motorPeriod);
	
	//b_HavingBottle_flag = TRUE;
	
	////return TRUE; //20161111動作チェック
	return FALSE;
}

BOOL ArmOutput()
{
	armLeft=sys_absoluteAxis_X;
	armRight=sys_absoluteAxis_Y;
	
	armGetLeft=sys_odoCount;
	armGetRight=sys_odoMeter;
	
	if(IsArmLimit(armLeft, armRight) == TRUE)
	{
		armLeft=0;
		armRight=0;
	}

	return FALSE;
}

/**
@brief
モーターへの出力を停止します。

@note
通信モジュール、インジケータLEDは停止しません。
*/
void DeviceOutputForceTerminate()
{
	pwm_L1.pulsewidth_ms(0);
	pwm_L2.pulsewidth_ms(0);

	pwm_R1.pulsewidth_ms(0);
	pwm_R2.pulsewidth_ms(0);

	//motorShutdown = 0;
}