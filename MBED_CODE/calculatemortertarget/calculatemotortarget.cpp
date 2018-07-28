#include "mbed.h"
#include "typedef.h"
#include "sysdata.h"
#include "parameter.h"
#include "errorcontrol.h"
#include "calculatemotortarget.h"

#include "caluculatetargetspeed.h"
#include "caluculatesteer.h"
#include "deviceinput.h"

motorDirection	sys_leftMotorDirection;
motorDirection	sys_rightMotorDirection;
uint16			sys_targetLeftMotorSpeed;			///<左モーターの目標速度 単位mm/sec
uint16			sys_targetRightMotorSpeed;			///<右モーターの目標速度 単位mm/sec
uint16			sys_targetLeftMotorDuty;			///<モーターへ伝えるDuty比の目標値 0.0%　～ 100.0% 0.5%刻み 0.1%単位　-> 555 = 55.5%
uint16			sys_targetRightMotorDuty;			///<モーターへ伝えるDuty比の目標値 0.0%　～ 100.0% 0.5%刻み 0.1%単位　-> 44.5 = 44.5%
BOOL            sys_outInFlag;

uint8 param_7gain = 40;	//操舵のゲイン マイナスするパーセンテージ 単位%
uint8 param_6gain = 30;	//操舵のゲイン 単位%
uint8 param_5gain = 20;	//操舵のゲイン 単位%
uint8 param_4gain = 15;	//操舵のゲイン 単位%
uint8 param_3gain = 10;	//操舵のゲイン 単位%
uint8 param_2gain = 5;	//操舵のゲイン 単位%
uint8 param_1gain = 3;	//操舵のゲイン 単位%

float32 param_vMap0 = 8.4;
float32 param_vMap1 = 8.0;
float32 param_vMap2 = 7.6;
float32 param_vMap3 = 7.2;
float32 param_vMap4 = 6.8;

//第1第2セクタ間2段階曲がり
uint16 param_firstTurnLeft = 190;//170 600, 200 400
uint16 param_firstTurnRight = 650;
uint16 param_secondTurnLeft = 175; //check
uint16 param_secondTurnRight = 375; //check
//第2第3セクタ間2段階曲がり
uint16 param_thirdTurnLeft = 570;
uint16 param_thirdTurnRight = 240;
uint16 param_fourthTurnLeft = 400;
uint16 param_fourthTurnRight = 240;
uint16 param_fourthTurnRight_OutIn = 240;
/* k_sato */
//第3セクタ直角後の右曲がり
uint16 param_fifthTurnLeft = 160;
uint16 param_fifthTurnRight = 100;
//第3セクタゴールへまっすぐ向ける
uint16 param_sixthTurnLeft = 100;
uint16 param_sixthTurnRight = 0;
/* k_sato End */


/**
 肝です。ちゃんと調査するように。
*/
#define VOLTAGE_MAP_INDEX_MAX (5)
#define SPEED_MAP_INDEX_MAX   (6)
uint16 param_speedVsVoltageMap[VOLTAGE_MAP_INDEX_MAX][SPEED_MAP_INDEX_MAX] =
{
	//0%,10%,20%,30%,50%,100%
	{0,400,800,1200,2000,4000},//8.4V
	{0,400,800,1200,2000,4000},//8.0V
	{0,400,800,1200,2000,4000},//7.6V
	{0,400,800,1200,2000,4000},//7.2V
	{0,400,800,1200,2000,4000}//6.8V
};
uint16 sys_speedVsVoltageMap[SPEED_MAP_INDEX_MAX];
uint16 sys_speedVsDutyMap[SPEED_MAP_INDEX_MAX] = {0,100,200,300,500,1000};


void InitializeCalculateMotorOutput(void);
BOOL CalculateMotorOutput(void);


static void CreateSpeedVsVoltageMap(void);
static void CalculateMotorTargetTo1stVCP();

static void CalculateMotorTargetTo2ndVCP();
static void CalculateMotorTargetTo3rdVCP();
static void CalculateMotorTargetTo4thVCP();
static void CalculateMotorTargetTo5thVCP();
static void CalculateMotorTargetTo6thVCP();
static void CalculateMotorTargetTo7thVCP();
static void CalculateMotorTargetTo8thVCP();
static void CalculateMotorTargetTo9thVCP();
/* k_sato */
static void CalculateMotorTargetTo10thVCP();
static void CalculateMotorTargetTo11thVCP();
/* k_sato End */
static void CalculateMotorTargetToGoal();
static void CalculateMotorTargetInGoal();
static void CalculateMotorTargetSyumai();
static void CalculateMotorTargetCaribrate();
static void CalculateMotorTargetBetween4thTo5thVCP();

/**
@brief
モーター出力計算の初期化処理
@note
この関数は基本的に呼び出すだけにしてください。

@return F:エラー無し T:何らかのエラー有
*/
void CalculateMotorTargetInit()
{
	//電圧に対しての速度とDuty比のマップを作成します。
	CreateSpeedVsVoltageMap();

}

static void CreateSpeedVsVoltageMap()
{

return;

}


/**
@brief
操舵指示、速度指示から
実際にモータに出力するパルスのデューティー比を決定します。

@note
この関数は基本的に呼び出すだけにしてください。

@return F:エラー無し T:何らかのエラー有
*/
BOOL CalculateMotorTarget()
{
	if(sys_nowRouteState == RS_READY){
		sys_rightMotorDirection = MD_FORWORD;
		sys_leftMotorDirection = MD_FORWORD;
		sys_targetLeftMotorDuty = 0;
		sys_targetRightMotorDuty = 0;
		return FALSE;
	}

	if(sys_nowRouteState == RS_TO_1ST_VCP){
		CalculateMotorTargetTo1stVCP();
	}

	if(sys_nowRouteState == RS_TO_2ND_VCP){
		CalculateMotorTargetTo2ndVCP();
	}

	if(sys_nowRouteState == RS_TO_3RD_VCP){
		CalculateMotorTargetTo3rdVCP();
	}

	if(sys_nowRouteState == RS_TO_4TH_VCP){
		CalculateMotorTargetTo4thVCP();
	}
	
	if(sys_nowRouteState == RS_BW_4TH_5TH_VCP){
		CalculateMotorTargetBetween4thTo5thVCP();
	}
	
	if(sys_nowRouteState == RS_TO_5TH_VCP){
		CalculateMotorTargetTo5thVCP();
	}

	if(sys_nowRouteState == RS_TO_6TH_VCP){
		CalculateMotorTargetTo6thVCP();
	}


	if(sys_nowRouteState == RS_TO_7TH_VCP){
		CalculateMotorTargetTo7thVCP();
	}

	if(sys_nowRouteState == RS_TO_8TH_VCP){
		CalculateMotorTargetTo8thVCP();
	}

	if(sys_nowRouteState == RS_TO_9TH_VCP){
		CalculateMotorTargetTo9thVCP();
	}

/* k_sato */
	if(sys_nowRouteState == RS_TO_10TH_VCP){
		CalculateMotorTargetTo10thVCP();
	}

	if(sys_nowRouteState == RS_TO_11TH_VCP){
		CalculateMotorTargetTo11thVCP();
	}
/* k_sato End */

	if(sys_nowRouteState == RS_TO_GOAL){
		CalculateMotorTargetToGoal();
	}
	
	if(sys_nowRouteState == RS_IN_GOAL){
		CalculateMotorTargetInGoal();
	}
	
	if(sys_nowRouteState == RS_SYUMAI){
		CalculateMotorTargetSyumai();
	}
	
	if(sys_nowRouteState == RS_CALIBRATION){
		CalculateMotorTargetCaribrate();
	}
	
		
	//sys_leftMotorDirection = MD_FORWORD;
	//sys_rightMotorDirection  = MD_FORWORD;
	
	return FALSE;

}

static void CalculateMotorTargetTo1stVCP()
{
	uint16 maxDuty = 400; //800

	switch(sys_nowSteerOrder)
	{
	case SO_L7:
			sys_targetLeftMotorDuty = maxDuty-70;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L6:
			sys_targetLeftMotorDuty = maxDuty-50;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L5:
			sys_targetLeftMotorDuty = maxDuty-30;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L4:
			sys_targetLeftMotorDuty = maxDuty-20;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L3:
			sys_targetLeftMotorDuty = maxDuty-10;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L2:
			sys_targetLeftMotorDuty = maxDuty-5;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L1:
			sys_targetLeftMotorDuty = maxDuty-3;
			sys_targetRightMotorDuty = maxDuty;
		break;

	case SO_ST:
		sys_targetLeftMotorDuty = maxDuty;
		sys_targetRightMotorDuty = maxDuty;
		break;

	case SO_R1:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-4;
		break;

	case SO_R2:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-8;
		break;
		
	case SO_R3:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-12;
		break;
	case SO_R4:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-20;
		break;
	case SO_R5:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-30;
		break;
	case SO_R6:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-50;
		break;
	case SO_R7:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-70;
		break;
	default:
		break;
	}
}


static void CalculateMotorTargetTo2ndVCP()
{
	uint16 maxDuty = 200;
	switch(sys_nowSteerOrder)
	{
	case SO_L7:
			sys_targetLeftMotorDuty = maxDuty-100;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L6:
			sys_targetLeftMotorDuty = maxDuty-80;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L5:
			sys_targetLeftMotorDuty = maxDuty-60;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L4:
			sys_targetLeftMotorDuty = maxDuty-40;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L3:
			sys_targetLeftMotorDuty = maxDuty-20;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L2:
			sys_targetLeftMotorDuty = maxDuty-10;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L1:
			sys_targetLeftMotorDuty = maxDuty-3;
			sys_targetRightMotorDuty = maxDuty;
		break;

	case SO_ST:
		sys_targetLeftMotorDuty = maxDuty;
		sys_targetRightMotorDuty = maxDuty;
		break;

	case SO_R1:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-3;
		break;

	case SO_R2:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-10;
		break;
		
	case SO_R3:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-20;
		break;
	case SO_R4:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-40;
		break;
	case SO_R5:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-60;
		break;
	case SO_R6:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-80;
		break;
	case SO_R7:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-100;
		break;
	default:
		break;
	}
}


static void CalculateMotorTargetTo3rdVCP()
{
	sys_targetLeftMotorDuty = param_firstTurnLeft;
	sys_targetRightMotorDuty = param_firstTurnRight;
}

static void CalculateMotorTargetTo4thVCP()
{
	sys_targetLeftMotorDuty = param_secondTurnLeft;
	sys_targetRightMotorDuty = param_secondTurnRight;
}

static void CalculateMotorTargetBetween4thTo5thVCP()
{
	
	uint16 maxDuty = 300;
	uint16 lebel1 = 5;
	uint16 lebel2 = 10;
	uint16 lebel3 = 20;
	uint16 lebel4 = 40;
	uint16 lebel5 = 80;
	uint16 lebel6 = 160;
	uint16 lebel7 = 300;
	switch(sys_nowSteerOrder)
	{
	case SO_L7:
			sys_targetLeftMotorDuty = maxDuty-lebel7;//-76;
			sys_targetRightMotorDuty = maxDuty;//+170;
		break;
		
	case SO_L6:
			sys_targetLeftMotorDuty = maxDuty-lebel6;;//-76;
			sys_targetRightMotorDuty = maxDuty;//+146;
		break;
		
	case SO_L5:
			sys_targetLeftMotorDuty = maxDuty-lebel5;//-76;
			sys_targetRightMotorDuty = maxDuty;//+122;
		break;
		
	case SO_L4:
			sys_targetLeftMotorDuty = maxDuty-lebel4;//-76;
			sys_targetRightMotorDuty = maxDuty;//+98;
		break;
		
	case SO_L3:
			sys_targetLeftMotorDuty = maxDuty-lebel3;//-76;
			sys_targetRightMotorDuty = maxDuty;//+76;
		break;
		
	case SO_L2:
			sys_targetLeftMotorDuty = maxDuty-lebel2;;//-50;
			sys_targetRightMotorDuty = maxDuty;//+50;
		break;
		
	case SO_L1:
			sys_targetLeftMotorDuty = maxDuty-lebel1;//-26;
			sys_targetRightMotorDuty = maxDuty;//+26;
		break;

	case SO_ST:
		sys_targetLeftMotorDuty = maxDuty;
		sys_targetRightMotorDuty = maxDuty;
		break;

	case SO_R1:
			sys_targetLeftMotorDuty = maxDuty;//+26;
			sys_targetRightMotorDuty = maxDuty-lebel1;//-26;
		break;

	case SO_R2:
			sys_targetLeftMotorDuty = maxDuty;//+50;
			sys_targetRightMotorDuty = maxDuty-lebel2;//-50;
		break;
		
	case SO_R3:
			sys_targetLeftMotorDuty = maxDuty;//+74;
			sys_targetRightMotorDuty = maxDuty-lebel3;//-74;
		break;
	case SO_R4:
			sys_targetLeftMotorDuty = maxDuty;//+98;
			sys_targetRightMotorDuty = maxDuty-lebel4;//-74;
		break;
	case SO_R5:
			sys_targetLeftMotorDuty = maxDuty;//+122;
			sys_targetRightMotorDuty = maxDuty-lebel5;//-74;
		break;
	case SO_R6:
			sys_targetLeftMotorDuty = maxDuty;//+146;
			sys_targetRightMotorDuty = maxDuty-lebel6;//-74;
		break;
	case SO_R7:
			sys_targetLeftMotorDuty = maxDuty;//+170;
			sys_targetRightMotorDuty = maxDuty-lebel7;//-76;
		break;
	default:
		break;
	}
	
			

}

static void CalculateMotorTargetTo5thVCP()
{
	/*
	uint16 maxDuty = 350;
	switch(sys_nowSteerOrder)
	{
	case SO_L7:
			sys_targetLeftMotorDuty = maxDuty-38;
			sys_targetRightMotorDuty = maxDuty+80;
		break;
		
	case SO_L6:
			sys_targetLeftMotorDuty = maxDuty-38;
			sys_targetRightMotorDuty = maxDuty+73;
		break;
		
	case SO_L5:
			sys_targetLeftMotorDuty = maxDuty-38;
			sys_targetRightMotorDuty = maxDuty+61;
		break;
		
	case SO_L4:
			sys_targetLeftMotorDuty = maxDuty-38;
			sys_targetRightMotorDuty = maxDuty+49;
		break;
		
	case SO_L3:
			sys_targetLeftMotorDuty = maxDuty-38;
			sys_targetRightMotorDuty = maxDuty+38;
		break;
		
	case SO_L2:
			sys_targetLeftMotorDuty = maxDuty-25;
			sys_targetRightMotorDuty = maxDuty+25;
		break;
		
	case SO_L1:
			sys_targetLeftMotorDuty = maxDuty-13;
			sys_targetRightMotorDuty = maxDuty+13;
		break;

	case SO_ST:
		sys_targetLeftMotorDuty = maxDuty;
		sys_targetRightMotorDuty = maxDuty;
		break;

	case SO_R1:
			sys_targetLeftMotorDuty = maxDuty+13;
			sys_targetRightMotorDuty = maxDuty-13;
		break;

	case SO_R2:
			sys_targetLeftMotorDuty = maxDuty+25;
			sys_targetRightMotorDuty = maxDuty-25;
		break;
		
	case SO_R3:
			sys_targetLeftMotorDuty = maxDuty+38;
			sys_targetRightMotorDuty = maxDuty-38;
		break;
	case SO_R4:
			sys_targetLeftMotorDuty = maxDuty+49;
			sys_targetRightMotorDuty = maxDuty-38;
		break;
	case SO_R5:
			sys_targetLeftMotorDuty = maxDuty+61;
			sys_targetRightMotorDuty = maxDuty-38;
		break;
	case SO_R6:
			sys_targetLeftMotorDuty = maxDuty+73;
			sys_targetRightMotorDuty = maxDuty-38;
		break;
	case SO_R7:
			sys_targetLeftMotorDuty = maxDuty+85;
			sys_targetRightMotorDuty = maxDuty-38;
		break;
	default:
		break;
	}		
	*/
	uint16 maxDuty = 300;
	
	uint16 lebel1 = 60;//5;
	uint16 lebel2 = 90;//10;
	uint16 lebel3 = 120;//20;
	uint16 lebel4 = 150;//40;
	uint16 lebel5 = 180;//80;
	uint16 lebel6 = 200;//160;
	uint16 lebel7 = 220;
	switch(sys_nowSteerOrder)
	{
	case SO_L7:
			sys_targetLeftMotorDuty = maxDuty-lebel7;//-76;
			sys_targetRightMotorDuty = maxDuty;//+170;
		break;
		
	case SO_L6:
			sys_targetLeftMotorDuty = maxDuty-lebel6;;//-76;
			sys_targetRightMotorDuty = maxDuty;//+146;
		break;
		
	case SO_L5:
			sys_targetLeftMotorDuty = maxDuty-lebel5;//-76;
			sys_targetRightMotorDuty = maxDuty;//+122;
		break;
		
	case SO_L4:
			sys_targetLeftMotorDuty = maxDuty-lebel4;//-76;
			sys_targetRightMotorDuty = maxDuty;//+98;
		break;
		
	case SO_L3:
			sys_targetLeftMotorDuty = maxDuty-lebel3;//-76;
			sys_targetRightMotorDuty = maxDuty;//+76;
		break;
		
	case SO_L2:
			sys_targetLeftMotorDuty = maxDuty-lebel2;;//-50;
			sys_targetRightMotorDuty = maxDuty;//+50;
		break;
		
	case SO_L1:
			sys_targetLeftMotorDuty = maxDuty-lebel1;//-26;
			sys_targetRightMotorDuty = maxDuty;//+26;
		break;

	case SO_ST:
		sys_targetLeftMotorDuty = maxDuty;
		sys_targetRightMotorDuty = maxDuty;
		break;

	case SO_R1:
			sys_targetLeftMotorDuty = maxDuty;//+26;
			sys_targetRightMotorDuty = maxDuty-lebel1;//-26;
		break;

	case SO_R2:
			sys_targetLeftMotorDuty = maxDuty;//+50;
			sys_targetRightMotorDuty = maxDuty-lebel2;//-50;
		break;
		
	case SO_R3:
			sys_targetLeftMotorDuty = maxDuty;//+74;
			sys_targetRightMotorDuty = maxDuty-lebel3;//-74;
		break;
	case SO_R4:
			sys_targetLeftMotorDuty = maxDuty;//+98;
			sys_targetRightMotorDuty = maxDuty-lebel4;//-74;
		break;
	case SO_R5:
			sys_targetLeftMotorDuty = maxDuty;//+122;
			sys_targetRightMotorDuty = maxDuty-lebel5;//-74;
		break;
	case SO_R6:
			sys_targetLeftMotorDuty = maxDuty;//+146;
			sys_targetRightMotorDuty = maxDuty-lebel6;//-74;
		break;
	case SO_R7:
			sys_targetLeftMotorDuty = maxDuty;//+170;
			sys_targetRightMotorDuty = maxDuty-lebel7;//-76;
		break;
	default:
		break;
	}
	/**/		
}

static void CalculateMotorTargetTo6thVCP()
{
	uint16 maxDuty = 300;
	
	uint16 lebel1 = 10;//5;
	uint16 lebel2 = 20;//10;
	uint16 lebel3 = 30;//20;
	uint16 lebel4 = 40;
	uint16 lebel5 = 50;//80;
	uint16 lebel6 = 70;//160;
	uint16 lebel7 = 90;//300;
	switch(sys_nowSteerOrder)
	{
	case SO_L7:
			sys_targetLeftMotorDuty = maxDuty-lebel7;//-76;
			sys_targetRightMotorDuty = maxDuty;//+170;
		break;
		
	case SO_L6:
			sys_targetLeftMotorDuty = maxDuty-lebel6;;//-76;
			sys_targetRightMotorDuty = maxDuty;//+146;
		break;
		
	case SO_L5:
			sys_targetLeftMotorDuty = maxDuty-lebel5;//-76;
			sys_targetRightMotorDuty = maxDuty;//+122;
		break;
		
	case SO_L4:
			sys_targetLeftMotorDuty = maxDuty-lebel4;//-76;
			sys_targetRightMotorDuty = maxDuty;//+98;
		break;
		
	case SO_L3:
			sys_targetLeftMotorDuty = maxDuty-lebel3;//-76;
			sys_targetRightMotorDuty = maxDuty;//+76;
		break;
		
	case SO_L2:
			sys_targetLeftMotorDuty = maxDuty-lebel2;;//-50;
			sys_targetRightMotorDuty = maxDuty;//+50;
		break;
		
	case SO_L1:
			sys_targetLeftMotorDuty = maxDuty-lebel1;//-26;
			sys_targetRightMotorDuty = maxDuty;//+26;
		break;

	case SO_ST:
		sys_targetLeftMotorDuty = maxDuty;
		sys_targetRightMotorDuty = maxDuty;
		break;

	case SO_R1:
			sys_targetLeftMotorDuty = maxDuty;//+26;
			sys_targetRightMotorDuty = maxDuty-lebel1;//-26;
		break;

	case SO_R2:
			sys_targetLeftMotorDuty = maxDuty;//+50;
			sys_targetRightMotorDuty = maxDuty-lebel2;//-50;
		break;
		
	case SO_R3:
			sys_targetLeftMotorDuty = maxDuty;//+74;
			sys_targetRightMotorDuty = maxDuty-lebel3;//-74;
		break;
	case SO_R4:
			sys_targetLeftMotorDuty = maxDuty;//+98;
			sys_targetRightMotorDuty = maxDuty-lebel4;//-74;
		break;
	case SO_R5:
			sys_targetLeftMotorDuty = maxDuty;//+122;
			sys_targetRightMotorDuty = maxDuty-lebel5;//-74;
		break;
	case SO_R6:
			sys_targetLeftMotorDuty = maxDuty;//+146;
			sys_targetRightMotorDuty = maxDuty-lebel6;//-74;
		break;
	case SO_R7:
			sys_targetLeftMotorDuty = maxDuty;//+170;
			sys_targetRightMotorDuty = maxDuty-lebel7;//-76;
		break;
	default:
		break;
	}		
}

static void CalculateMotorTargetTo7thVCP()	//上の大カーブ
{		
//	if(sys_frontLineSenser == FLS_WBWW){	//内側にいるけど左に曲がろうとしていたので強く
		sys_targetLeftMotorDuty = param_thirdTurnLeft;
		sys_targetRightMotorDuty = param_thirdTurnRight;
//	}else if(sys_frontLineSenser == FLS_WWBW) //外側にいるけど右に曲がろうとしているので弱く
//	{
//		sys_targetLeftMotorDuty = 210;
//		sys_targetRightMotorDuty = 80;
	
//	}	
}

static void CalculateMotorTargetTo8thVCP()	//カーブ後の90向かうとこ
{
	if(sys_outInFlag)
	{
		//登坂終了時にラインより左側にいる場合、回転半径を大きくする
		sys_targetLeftMotorDuty = param_fourthTurnLeft;
		sys_targetRightMotorDuty = param_fourthTurnRight_OutIn;
	}
	else
	{
		sys_targetLeftMotorDuty = param_fourthTurnLeft;
		sys_targetRightMotorDuty = param_fourthTurnRight;
	}
}
static void CalculateMotorTargetTo9thVCP()	//
{
//	sys_targetLeftMotorDuty = 400;
//	sys_targetRightMotorDuty = 100;
	uint16 maxDuty = 250;
	
	uint16 lebel1 = 50;
	uint16 lebel2 = 80;
	uint16 lebel3 = 120;
	uint16 lebel4 = 120;
	uint16 lebel5 = 160;
	uint16 lebel6 = 200;
	uint16 lebel7 = 250;
	switch(sys_nowSteerOrder)
	{
	case SO_L7:
			sys_targetLeftMotorDuty = maxDuty-lebel7;//-76;
			sys_targetRightMotorDuty = maxDuty;//+170;
		break;
		
	case SO_L6:
			sys_targetLeftMotorDuty = maxDuty-lebel6;;//-76;
			sys_targetRightMotorDuty = maxDuty;//+146;
		break;
		
	case SO_L5:
			sys_targetLeftMotorDuty = maxDuty-lebel5;//-76;
			sys_targetRightMotorDuty = maxDuty;//+122;
		break;
		
	case SO_L4:
			sys_targetLeftMotorDuty = maxDuty-lebel4;//-76;
			sys_targetRightMotorDuty = maxDuty;//+98;
		break;
		
	case SO_L3:
			sys_targetLeftMotorDuty = maxDuty-lebel3;//-76;
			sys_targetRightMotorDuty = maxDuty;//+76;
		break;
		
	case SO_L2:
			sys_targetLeftMotorDuty = maxDuty-lebel2;;//-50;
			sys_targetRightMotorDuty = maxDuty;//+50;
		break;
		
	case SO_L1:
			sys_targetLeftMotorDuty = maxDuty-lebel1;//-26;
			sys_targetRightMotorDuty = maxDuty;//+26;
		break;

	case SO_ST:
		sys_targetLeftMotorDuty = maxDuty;
		sys_targetRightMotorDuty = maxDuty;
		break;

	case SO_R1:
			sys_targetLeftMotorDuty = maxDuty;//+26;
			sys_targetRightMotorDuty = maxDuty-lebel1;//-26;
		break;

	case SO_R2:
			sys_targetLeftMotorDuty = maxDuty;//+50;
			sys_targetRightMotorDuty = maxDuty-lebel2;//-50;
		break;
		
	case SO_R3:
			sys_targetLeftMotorDuty = maxDuty;//+74;
			sys_targetRightMotorDuty = maxDuty-lebel3;//-74;
		break;
	case SO_R4:
			sys_targetLeftMotorDuty = maxDuty;//+98;
			sys_targetRightMotorDuty = maxDuty-lebel4;//-74;
		break;
	case SO_R5:
			sys_targetLeftMotorDuty = maxDuty;//+122;
			sys_targetRightMotorDuty = maxDuty-lebel5;//-74;
		break;
	case SO_R6:
			sys_targetLeftMotorDuty = maxDuty;//+146;
			sys_targetRightMotorDuty = maxDuty-lebel6;//-74;
		break;
	case SO_R7:
			sys_targetLeftMotorDuty = maxDuty;//+170;
			sys_targetRightMotorDuty = maxDuty-lebel7;//-76;
		break;
	default:
		break;
	}		

}

/* k_sato */
/**
@brief
右曲がり中。右がついたら次へ状態遷移。
ゴールへまっすぐ向かう為の準備期間
**/
static void CalculateMotorTargetTo10thVCP()	//
{
	uint16 maxDuty = 200;
	switch(sys_nowSteerOrder)
	{
	case SO_L7:
			sys_targetLeftMotorDuty = maxDuty-70;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L6:
			sys_targetLeftMotorDuty = maxDuty-50;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L5:
			sys_targetLeftMotorDuty = maxDuty-30;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L4:
			sys_targetLeftMotorDuty = maxDuty-20;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L3:
			sys_targetLeftMotorDuty = maxDuty-50;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L2:
			sys_targetLeftMotorDuty = maxDuty-70;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L1:
			sys_targetLeftMotorDuty = maxDuty-50;
			sys_targetRightMotorDuty = maxDuty;
		break;

	case SO_ST:
		sys_targetLeftMotorDuty = maxDuty;
		sys_targetRightMotorDuty = maxDuty;
		break;

	case SO_R1:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-50;
		break;

	case SO_R2:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-70;
		break;
		
	case SO_R3:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-60;
		break;
	case SO_R4:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-100;
		break;
	case SO_R5:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-30;
		break;
	case SO_R6:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-50;
		break;
	case SO_R7:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-70;
		break;
	default:
		break;
	}
}

/**
@brief
信地旋回(左タイヤをバックに回す)し、
後ろのセンサーがWWWBになったら次の状態へ。
これで大体ゴールへまっすぐ向いているはず。
**/
static void CalculateMotorTargetTo11thVCP()	//
{
	uint16 maxDuty = 200;
	switch(sys_nowSteerOrder)
	{
	case SO_L7:
			sys_targetLeftMotorDuty = maxDuty-70;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L6:
			sys_targetLeftMotorDuty = maxDuty-50;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L5:
			sys_targetLeftMotorDuty = maxDuty-30;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L4:
			sys_targetLeftMotorDuty = maxDuty-100;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L3:
			sys_targetLeftMotorDuty = maxDuty-50;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L2:
			sys_targetLeftMotorDuty = maxDuty-50;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L1:
			sys_targetLeftMotorDuty = maxDuty-30;
			sys_targetRightMotorDuty = maxDuty;
		break;

	case SO_ST:
		sys_targetLeftMotorDuty = maxDuty;
		sys_targetRightMotorDuty = maxDuty;
		break;

	case SO_R1:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-30;
		break;

	case SO_R2:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-50;
		break;
		
	case SO_R3:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-60;
		break;
	case SO_R4:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-100;
		break;
	case SO_R5:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-30;
		break;
	case SO_R6:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-50;
		break;
	case SO_R7:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-70;
		break;
	default:
		break;
	}
}
/* k_sato End */

static void CalculateMotorTargetToGoal()
{
	uint16 maxDuty = 300;
		
	switch(sys_nowSteerOrder)
	{
	case SO_L7:
			sys_targetLeftMotorDuty = maxDuty-170;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L6:
			sys_targetLeftMotorDuty = maxDuty-146;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L5:
			sys_targetLeftMotorDuty = maxDuty-122;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L4:
			sys_targetLeftMotorDuty = maxDuty-98;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L3:
			sys_targetLeftMotorDuty = maxDuty-76;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L2:
			sys_targetLeftMotorDuty = maxDuty-50;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L1:
			sys_targetLeftMotorDuty = maxDuty-26;
			sys_targetRightMotorDuty = maxDuty;
		break;

	case SO_ST:
		sys_targetLeftMotorDuty = maxDuty;
		sys_targetRightMotorDuty = maxDuty;
		break;

	case SO_R1:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-26;
		break;

	case SO_R2:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-50;
		break;
		
	case SO_R3:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-74;
		break;
	case SO_R4:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-98;
		break;
	case SO_R5:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-122;
		break;
	case SO_R6:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-146;
		break;
	case SO_R7:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-170;
		break;
	default:
		break;
	}
}

static void CalculateMotorTargetInGoal()
{
	//sys_targetLeftMotorDuty = 30;
	//sys_targetRightMotorDuty = 30;
	//sys_leftMotorDirection = MD_BACK;
	//sys_rightMotorDirection = MD_BACK;
	uint16 maxDuty = 300;
		
	switch(sys_nowSteerOrder)
	{
	case SO_L7:
			sys_targetLeftMotorDuty = maxDuty-170;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L6:
			sys_targetLeftMotorDuty = maxDuty-146;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L5:
			sys_targetLeftMotorDuty = maxDuty-122;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L4:
			sys_targetLeftMotorDuty = maxDuty-98;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L3:
			sys_targetLeftMotorDuty = maxDuty-76;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L2:
			sys_targetLeftMotorDuty = maxDuty-50;
			sys_targetRightMotorDuty = maxDuty;
		break;
		
	case SO_L1:
			sys_targetLeftMotorDuty = maxDuty-26;
			sys_targetRightMotorDuty = maxDuty;
		break;

	case SO_ST:
		sys_targetLeftMotorDuty = maxDuty;
		sys_targetRightMotorDuty = maxDuty;
		break;

	case SO_R1:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-26;
		break;

	case SO_R2:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-50;
		break;
		
	case SO_R3:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-74;
		break;
	case SO_R4:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-98;
		break;
	case SO_R5:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-122;
		break;
	case SO_R6:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-146;
		break;
	case SO_R7:
			sys_targetLeftMotorDuty = maxDuty;
			sys_targetRightMotorDuty = maxDuty-170;
		break;
	default:
		break;
	}
}

static void CalculateMotorTargetSyumai()
{
	sys_targetLeftMotorDuty = 0;
	sys_targetRightMotorDuty = 0;
}

static void CalculateMotorTargetCaribrate()
{
	//sys_targetLeftMotorDuty = 150;
	//sys_targetRightMotorDuty = 150;
	//uint16 maxDuty = 150;
	sys_requestServoPulseWidth = 1500;
}

#if 0
//テストの為計算しません--------------------------------------------
/*
	//モーターに対する速度を確定します
	bret |= SetMotorTargetSpeed();

	//左モーターの回転方向決める
	bret |= LeftMotorDirectionCalculate();
	//左モーターの目標速度を決める
	bret |= LeftMotorTargetDutyCalculate();

	//右モーターの回転方向を決める
	bret |= RightMotorDirectionCalculate();
	//右モーターの目標速度決める
	bret |= RightMotorTargetDutyCalculate();

    return bret;
    */
}






static BOOL SetMotorTargetSpeed()
{
	BOOL ret = FALSE;
[
	switch(sys_nowSteerOrder)
	{
	case SO_L7:
		sys_targetLeftMotorSpeed = sys_targetSpeed;
		sys_targetRightMotorSpeed = (uint16)((float32)sys_targetSpeed * ((float32)(100 - param_7gain) / (float32)(100)));
		break;
	case SO_L6:
		sys_targetLeftMotorSpeed = sys_targetSpeed;
		sys_targetRightMotorSpeed = (uint16)((float32)sys_targetSpeed * ((float32)(100 - param_6gain) / (float32)(100)));
		break;
	case SO_L5:
		sys_targetLeftMotorSpeed = sys_targetSpeed;
		sys_targetRightMotorSpeed = (uint16)((float32)sys_targetSpeed * ((float32)(100 - param_5gain) / (float32)(100)));
		break;
	case SO_L4:
		sys_targetLeftMotorSpeed = sys_targetSpeed;
		sys_targetRightMotorSpeed = (uint16)((float32)sys_targetSpeed * ((float32)(100 - param_4gain) / (float32)(100)));
		break;
	case SO_L3:
		sys_targetLeftMotorSpeed = sys_targetSpeed;
		sys_targetRightMotorSpeed = (uint16)((float32)sys_targetSpeed * ((float32)(100 - param_3gain) / (float32)(100)));
		break;
	case SO_L2:
		sys_targetLeftMotorSpeed = sys_targetSpeed;
		sys_targetRightMotorSpeed = (uint16)((float32)sys_targetSpeed * ((float32)(100 - param_2gain) / (float32)(100)));
		break;
	case SO_L1:
		sys_targetLeftMotorSpeed = sys_targetSpeed;
		sys_targetRightMotorSpeed = (uint16)((float32)sys_targetSpeed * ((float32)(100 - param_1gain) / (float32)(100)));
		break;
	case SO_ST:
		sys_targetLeftMotorSpeed = sys_targetSpeed;
		sys_targetRightMotorSpeed = sys_targetSpeed;
		break;
	case SO_R1:
		sys_targetLeftMotorSpeed = (uint16)((float32)sys_targetSpeed * ((float32)(100 - param_1gain) / (float32)(100)));
		sys_targetRightMotorSpeed = sys_targetSpeed;
		break;
	case SO_R2:
		sys_targetLeftMotorSpeed = (uint16)((float32)sys_targetSpeed * ((float32)(100 - param_2gain) / (float32)(100)));
		sys_targetRightMotorSpeed = sys_targetSpeed;
		break;
	case SO_R3:
		sys_targetLeftMotorSpeed = (uint16)((float32)sys_targetSpeed * ((float32)(100 - param_3gain) / (float32)(100)));
		sys_targetRightMotorSpeed = sys_targetSpeed;
		break;
	case SO_R4:
		sys_targetLeftMotorSpeed = (uint16)((float32)sys_targetSpeed * ((float32)(100 - param_4gain) / (float32)(100)));
		sys_targetRightMotorSpeed = sys_targetSpeed;
		break;
	case SO_R5:
		sys_targetLeftMotorSpeed = (uint16)((float32)sys_targetSpeed * ((float32)(100 - param_5gain) / (float32)(100)));
		sys_targetRightMotorSpeed = sys_targetSpeed;
		break;
	case SO_R6:
		sys_targetLeftMotorSpeed = (uint16)((float32)sys_targetSpeed * ((float32)(100 - param_6gain) / (float32)(100)));
		sys_targetRightMotorSpeed = sys_targetSpeed;
		break;
	case SO_R7:
		sys_targetLeftMotorSpeed = (uint16)((float32)sys_targetSpeed * ((float32)(100 - param_7gain) / (float32)(100)));
		sys_targetRightMotorSpeed = sys_targetSpeed;
		break;
	default:
		ERR_WriteID(EID_RANGE_OVER, __func__, __LINE__);
		sys_targetLeftMotorSpeed = 0;
		sys_targetRightMotorSpeed = 0;
		ret = TRUE;
		break;
	}
	return ret;

}



/**
@brief
左車輪の回転方向を確定します。
ここでの決定した値をもとにモーターへのパルスが確定します。

@note
2015ARC横浜の制御の仕様では後退はリモコン操作の時のみ
基本的に制御側で後退処理は入りません

@return F:エラー無し T:何らかのエラー有
*/
static BOOL LeftMotorDirectionCalculate()
{
	sys_leftMotorDirection = MD_FORWORD;

    return FALSE;
}

/**
@brief
左車輪の目標速度に対応したDuty比を計算します

計算方法）

1.起動時に取得したバッテリーの電圧からどのマップとマップのパラメータを使うか選定します
（電圧上位マップ"hVolMap"と電圧下位マップ"lVolMap"）
2.現在電圧に対応した新たなマップを作成します →ここまでinit時の処理
3.作製した新規マップから目標のDuty比を算出します

例）
ターゲットスピードが2000mm/sec、
電圧が7.5vとした場合

マップが
         10%  20%  30%  40%  50%
8V =   |1000|1500|2000|2500|3000|
7V =   |800 |1000|1200|1400|1600|
だとすると
7.5V = |900 |1250|1600|1950|2300|
になる
したがって 求める2000の時のDuty比は
40+{(50-40)*(2000-1950)/(2300-1950)} = 41.428 ≒ 41.5%
となります

@attention
目標値の値をそのままモーターに伝えると電気回路負荷が大きくなり破損の恐れがあります。
その為、目標値に対して現在値が少しずつ近づくように加速、減速処理を入れます。
この、加減速計算が完了した値をドライバに伝える為、ここでの計算値を直接モーターに命令しないようにしてください。
加速減速処理計算は別関数で行います。

@note
この方式だと、設定した目標速度に永久に到達しません（設計未達ですごめんなさい,曲がっているときの速度制御も未達です、ホントごめんなさい）
が、めんどくさいので後々ロジックを変更できればと思います。
でも、位置決め制御モノじゃないからいいよね？ね？能力不足ごめんなさいm（_ _）m

@return F:エラー無し T:何らかのエラー有
*/
static BOOL LeftMotorTargetDutyCalculate()
{

	uint16 idx = 0;
	uint16 i;

	for(i = 0; i < SPEED_MAP_INDEX_MAX ; i++)
	{
		if(sys_speedVsVoltageMap[i] < sys_targetLeftMotorSpeed)
		{
			idx = i;
			break;
		}
	}
	if((idx + 1) >= SPEED_MAP_INDEX_MAX)
	{
		ERR_WriteID(EID_RANGE_OVER, __func__, __LINE__);
		return TRUE;
	}

	sys_targetLeftMotorDuty = 	sys_speedVsDutyMap[idx] +
								sys_speedVsDutyMap[idx+1] - sys_speedVsDutyMap[idx] *
							   (sys_targetLeftMotorSpeed - sys_speedVsVoltageMap[idx]) /
							   (sys_speedVsDutyMap[idx+1] - sys_speedVsDutyMap[idx]);


    return FALSE;
}




/**
@brief
右車輪の回転方向を確定します。
ここでの決定した値をもとにモーターへのパルスが確定します。

@note
2015ARC横浜の制御の仕様では後退はリモコン操作の時のみ
基本的に制御側で後退処理は入りません

@return F:エラー無し T:何らかのエラー有
*/
static BOOL RightMotorDirectionCalculate()
{

	sys_rightMotorDirection = MD_FORWORD;

	return FALSE;
}


/**
@brief
左モノと同じ。

*/
static BOOL RightMotorTargetDutyCalculate()
{
	uint16 idx = 0;
	uint16 i;

    
	for(i = 0; i < SPEED_MAP_INDEX_MAX ; i++)
	{
		if(sys_speedVsVoltageMap[i] < sys_targetRightMotorSpeed)
		{
			idx = i;
			break;
		}
	}
	if((idx + 1) >= SPEED_MAP_INDEX_MAX)
	{
		ERR_WriteID(EID_RANGE_OVER, __func__, __LINE__);
		return TRUE;
	}

	sys_targetLeftMotorDuty = 	sys_speedVsDutyMap[idx] +
								sys_speedVsDutyMap[idx+1] - sys_speedVsDutyMap[idx] *
							   (sys_targetRightMotorSpeed - sys_speedVsVoltageMap[idx]) /
							   (sys_speedVsDutyMap[idx+1] - sys_speedVsDutyMap[idx]);
	return FALSE;
}

#endif
