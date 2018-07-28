#include "mbed.h"
#include "typedef.h"
#include "sysdata.h"
#include "parameter.h"
#include "errorcontrol.h"
#include "caluculatetargetspeed.h"

#include "judgestate.h"


BOOL CaluculateTargetSpeed(void);

int16 sys_targetSpeed				= 0;		///<目標車速 単位 mm/sec

int16 param_targetSpeedReady		= 0;		///<ready状態の時の目標速度
int16 param_targetSpeedTo1stVCP		= 1000;		///<VCP1へ向かう時の目標速度
int16 param_targetSpeedTo2ndVCP		= 0;		///<VCP2へ向かう時の目標速度
int16 param_targetSpeedTo3rdVCP		= 0;		///<VCP3へ向かう時の目標速度
int16 param_targetSpeedTo4thVCP		= 0;		///<VCP4へ向かう時の目標速度
int16 param_targetSpeedTo5thVCP		= 0;		///<VCP5へ向かう時の目標速度
int16 param_targetSpeedTo6thVCP		= 0;		///<VCP6へ向かう時の目標速度
int16 param_targetSpeedTo7thVCP		= 0;		///<VCP7へ向かう時の目標速度
int16 param_targetSpeedTo8thVCP		= 0;		///<VCP8へ向かう時の目標速度
int16 param_targetSpeedToGoal		= 0;		///<goalへ向かう時の目標速度
int16 param_targetSpeedCalibration	= 0;		///<キャリブレーションの時の目標速度

int16 param_targetSpeedLimit		= 1000;		///<目標速度ソフトウェアリミット


static BOOL CaluculateTargetSpeedReady(void);
static BOOL CaluculateTargetSpeedTo1stVCP(void);
static BOOL CaluculateTargetSpeedTo2ndVCP(void);
static BOOL CaluculateTargetSpeedTo3rdVCP(void);
static BOOL CaluculateTargetSpeedTo4thVCP(void);
static BOOL CaluculateTargetSpeedTo5thVCP(void);
static BOOL CaluculateTargetSpeedTo6thVCP(void);
static BOOL CaluculateTargetSpeedTo7thVCP(void);
static BOOL CaluculateTargetSpeedTo8thVCP(void);
static BOOL CaluculateTargetSpeedToGoal(void);
static BOOL CaluculateTargetSpeedCalibration(void);
static BOOL SetNowTargetSpeed(int16 order,uint8 gain);

static const uint8 MAX_GAIN	=	100;



BOOL CaluculateTargetSpeed()
{
	BOOL err = FALSE;

	switch (sys_nowRouteState)
	{
	case RS_READY:					///<スタート待ち
		err |= CaluculateTargetSpeedReady();
		break;
	case RS_TO_1ST_VCP:				///<第一仮想チェックポイントまで
		err |= CaluculateTargetSpeedTo1stVCP();
		break;
	case RS_TO_2ND_VCP:
		err |= CaluculateTargetSpeedTo2ndVCP();
		break;
	case RS_TO_3RD_VCP:
		err |= CaluculateTargetSpeedTo3rdVCP();
		break;
	case RS_TO_4TH_VCP:
		err |= CaluculateTargetSpeedTo4thVCP();
		break;
	case RS_TO_5TH_VCP:
		err |= CaluculateTargetSpeedTo5thVCP();
		break;
	case RS_TO_6TH_VCP:
		err |= CaluculateTargetSpeedTo6thVCP();
		break;
	case RS_TO_7TH_VCP:
		err |= CaluculateTargetSpeedTo7thVCP();
		break;
	case RS_TO_8TH_VCP:
		err |= CaluculateTargetSpeedTo8thVCP();
		break;
	case RS_TO_GOAL:
		err |= CaluculateTargetSpeedToGoal();
		break;
	case RS_IN_GOAL:
		//なにもしない
		break;
	case RS_CALIBRATION:				///直線補正計算用
		err |= CaluculateTargetSpeedCalibration();
		break;
	case RS_DEBUG_TYPE1:
	case RS_DEBUG_TYPE2:
	default:
		err |= CaluculateTargetSpeedTo1stVCP();
		break;
	}

	return err;

}


static BOOL CaluculateTargetSpeedReady()
{
	BOOL ret;

	ret = SetNowTargetSpeed(param_targetSpeedReady,0);

	return ret;
}

static BOOL CaluculateTargetSpeedTo1stVCP(void)
{
 //   BOOL ret;
//    ret = SetNowTargetSpeed(param_targetSpeedTo1stVCP,0);
	return FALSE;
}

static BOOL CaluculateTargetSpeedTo2ndVCP(void)
{

	return FALSE;
}

static BOOL CaluculateTargetSpeedTo3rdVCP(void)
{

	return FALSE;
}

static BOOL CaluculateTargetSpeedTo4thVCP(void)
{

	return FALSE;
}

static BOOL CaluculateTargetSpeedTo5thVCP(void)
{

	return FALSE;
}

static BOOL CaluculateTargetSpeedTo6thVCP(void)
{

	return FALSE;
}

static BOOL CaluculateTargetSpeedTo7thVCP(void)
{

	return FALSE;
}

static BOOL CaluculateTargetSpeedTo8thVCP(void)
{

	return FALSE;
}

static BOOL CaluculateTargetSpeedToGoal(void)
{

	return FALSE;
}

static BOOL CaluculateTargetSpeedCalibration(void)
{

	return FALSE;
}

static BOOL SetNowTargetSpeed(int16 order,uint8 gain)
{
	int16 tmpSp;
	uint8 tmpGain;

	if(gain > MAX_GAIN)
	{
		ERR_WriteID(EID_RANGE_OVER, __func__, __LINE__);
		return TRUE;
	}

	tmpGain = gain + 100;
	tmpSp = (int16)((float32)order * (float32)(tmpGain / 100));

	//設定速度がリミットを越えていないかチェック＆上書き
	if(tmpSp > param_targetSpeedLimit)
	{
		sys_targetSpeed = param_targetSpeedLimit;
	}
	else
	{
		sys_targetSpeed = tmpSp;
	}

	return FALSE;
}

