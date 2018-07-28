#include "mbed.h"
#include "typedef.h"
#include "sysdata.h"
#include "parameter.h"
#include "errorcontrol.h"
#include "caluculatesteer.h"

#include "destinationcontrol.h"
#include "judgestate.h"
#include <math.h>

steer_order sys_steerOrderLog[5]			= {SO_ST,SO_ST,SO_ST,SO_ST,SO_ST};	//デバッグ用操舵変更要求ログ（変更した時のみログが残ります）
steer_order	sys_nowSteerOrder				= SO_ST;							///< 現在の操舵要求

BOOL param_ignorOverSteerError				= FALSE;							///操舵量が大きすぎても、それをエラーとしない。 T:エラーとしない F:エラーとする

int param_d_state                           = 0;


/* k_sato */
static BOOL LineTrace_k_sato(void);
/* k_sato End */

int32 param_target1stVCPAxis_X = 2500000;
int32 param_target1stVCPAxis_Y = 1080000;
int32 param_target2ndVCPAxis_X = 2760000;
int32 param_target2ndVCPAxis_Y = 1080000;
int32 param_target3rdVCPAxis_X = 3030000;
int32 param_target3rdVCPAxis_Y = 1390000;
float64 param_target3rdVCPRadian = 0.83798122500839;	//48.01278度

int32 param_target4thVCPAxis_X = 2870000;
int32 param_target4thVCPAxis_Y = 1580000;
int32 param_target5thVCPAxis_X = 640000;
int32 param_target5thVCPAxis_Y = 1580000;
int32 param_target6thVCPAxis_X = 250000;
int32 param_target6thVCPAxis_Y = 1980000;
int32 param_target7thVCPAxis_X = 460000;
int32 param_target7thVCPAxis_Y = 2210000;
int32 param_target8thVCPAxis_X = 1020000;
int32 param_target8thVCPAxis_Y = 2210000;

int32 param_target10thVCPAxis_X = 1600000;
int32 param_target10thVCPAxis_Y = 0;
int32 param_target11thVCPAxis_Y = 0;
int32 param_targetGoalAxis_X;
/* k_sato */
//超絶適当。基準値なんて0でも差分抽出できればいいからいける！
//int32 param_targetGoalAxis_Y;
int32 param_targetGoalAxis_Y   = 0;
/* k_sato End */

//static uint16		integralRightSteerCount			= 0;								///<累積ステアカウント:このカウンタが一定以上になると操舵を強めます。
///static uint16		integralLeftSteerCount			= 0;								///<累積ステアカウント

//static const float64 deg1toRadian					= 0.017453292519943295;				///1度をラジアンに変換した値


static BOOL CaluculateSteerReady(void);
static BOOL CaluculateSteerTo1stVCP(void);
static BOOL CaluculateSteerTo2ndVCP(void);
static BOOL CaluculateSteerTo3rdVCP(void);
static BOOL CaluculateSteerTo4thVCP(void);
static BOOL CaluculateSteerTo5thVCP(void);
static BOOL CaluculateSteerTo6thVCP(void);
static BOOL CaluculateSteerTo7thVCP(void);
static BOOL CaluculateSteerTo8thVCP(void);
static BOOL CaluculateSteerTo9thVCP(void);
/* k_sato */
static BOOL CaluculateSteerTo10thVCP(void);
static BOOL CaluculateSteerTo11thVCP(void);
/* k_sato End */
static BOOL CaluculateSteerToGoal(void);
static BOOL CaluculateSteerCalibration(void);
static BOOL SetNowSteerOrder(steer_order newOrder);
static BOOL CalculateSteerBetween4thTo5th(void);

static BOOL AxisYTrace(int32); //201611
static BOOL LineTrace_old(void); //201611
static BOOL LineTrace(void); // 201611

/* k_sato */
/**
 
**/
BOOL LineTrace_k_sato()
{
    steer_order state = SO_ST;
 
    switch(sys_frontLineSenser)
    {
        case FLS_WWWW:
            switch(sys_rearLineSenser)
            {
                case RLS_WWWW:
                    state = SO_ST;
                    break;
                case RLS_WWWB:
                    if(sys_nowSteerOrder >= SO_ST){
                        state = SO_R3;
                    }else{
                        state = SO_L2;
                    }
                    break;
                case RLS_WWBW:
                    state = SO_L1;
                    break;
                case RLS_WBWW:
                    state = SO_R1;
                    break;
                case RLS_BWWW:
                    if(sys_nowSteerOrder <= SO_ST){
                        state = SO_L3;
                    }else{
                        state = SO_R2;
                    }
                    break;
                default:
                    break;
            }
            break;
        case FLS_WWWB:
            switch(sys_rearLineSenser)
            {
                case RLS_WWWW:
                    state = SO_R2;
                    break;
                case RLS_WWWB:
                    state = SO_R1;
                    break;
                case RLS_WWBW:
                    state = SO_R2;
                    break;
                case RLS_WBWW:
                    state = SO_R3;
                    break;
                case RLS_BWWW:
                    state = SO_R4;
                    break;
                default:
                    break;
            }
            break;
        case FLS_WWBW:
            switch(sys_rearLineSenser)
            {
                case RLS_WWWW:
                    state = SO_R1;
                    break;
                case RLS_WWWB:
                    state = SO_L2;
                    break;
                case RLS_WWBW:
                    state = SO_ST;
                    break;
                case RLS_WBWW:
                    state = SO_R2;
                    break;
                case RLS_BWWW:
                    state = SO_R3;
                    break;
                default:
                    break;
            }
            break;
        case FLS_WBWW:
            switch(sys_rearLineSenser)
            {
                case RLS_WWWW:
                    state = SO_L1;
                    break;
                case RLS_WWWB:
                    state = SO_L3;
                    break;
                case RLS_WWBW:
                    state = SO_L2;
                    break;
                case RLS_WBWW:
                    state = SO_ST;
                    break;
                case RLS_BWWW:
                    state = SO_R2;
                    break;
                default:
                    break;
            }
            break;
        case FLS_BWWW:
            switch(sys_rearLineSenser)
            {
                case RLS_WWWW:
                    state = SO_L2;
                    break;
                case RLS_WWWB:
                    state = SO_L4;
                    break;
                case RLS_WWBW:
                    state = SO_L3;
                    break;
                case RLS_WBWW:
                    state = SO_L2;
                    break;
                case RLS_BWWW:
                    state = SO_L1;
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
 
    SetNowSteerOrder(state);
    return FALSE;
}
/* k_sato End */

/**
**/
BOOL AxisYTrace(int32 targetAxisY)
{
	int32 deltaY;
	BOOL ret = FALSE;

	deltaY = sys_absoluteAxis_Y - targetAxisY;		//mm単位 int値

	if(abs(deltaY) <= 500)
	{
		SetNowSteerOrder(SO_ST);
		return ret;
	}

	/* deltaYは目的地に対し、自車が右方向（-Y）にずれていたら値がマイナスに、左方向（＋Y）にずれていれば値はプラスになります */
	/* その為、deltaYがマイナスであれば左に曲がる処理を行います */
	if(deltaY < 0)
	{
		if(abs(deltaY) > 1000)
		{
			SetNowSteerOrder(SO_L2);
		}
		else
		{
			SetNowSteerOrder(SO_L1);
		}
	}
	else
	{
		if(abs(deltaY) > 1000)
		{
			SetNowSteerOrder(SO_R2);
		}
		else
		{
			SetNowSteerOrder(SO_R1);
		}
	}

	return ret;
}
/**

**/
BOOL LineTrace_old()
{
	/*
	int state = 0;
	switch(sys_frontLineSenser)
	{
		case FLS_WWWB:
			state = 2;
			break;
		case FLS_WWBW:
			state = 1;
			break;
		case FLS_WBWW:
			state = -1;
			break;
		case FLS_BWWW:
			state = -2;
			break;
	}
	
	switch(sys_rearLineSenser)
	{
		case RLS_WWWB:
			state -= 2;
			break;
		case RLS_WWBW:
			state -= 1;
			break;
		case RLS_WBWW:
			state += 1;
			break;
		case RLS_BWWW:
			state += 2;
			break;
		default:
			break;
	}
	
	if(sys_frontLineSenser == FLS_WWWB && sys_rearLineSenser == RLS_WWWB)
	{
		state = 1;
	}
	else if(sys_frontLineSenser == FLS_BWWW && sys_rearLineSenser == RLS_BWWW)
	{
		state = -1;
	}
	
	/* k_sato 
    if(sys_frontLineSenser == FLS_WWWW && sys_rearLineSenser == RLS_BWWW){
        if(sys_nowSteerOrder <= 0){  
            state = -2;
        }
    }
 
    if(sys_frontLineSenser == FLS_WWWW && sys_rearLineSenser == RLS_WWWB){
        if(sys_nowSteerOrder >= 0){  
            state = 2;
        }
        //逆にした
    }
/* k_sato End 
	
	switch(state)
	{
		case 4:
			SetNowSteerOrder(SO_R4);
			break;
		case 3:
			SetNowSteerOrder(SO_R3);
			break;
		case 2:
			SetNowSteerOrder(SO_R2);
			break;
		case 1:
			SetNowSteerOrder(SO_R1);
			break;
		case 0:
			SetNowSteerOrder(SO_ST);
			break;
		case -1:
			SetNowSteerOrder(SO_L1);
			break;
		case -2:
			SetNowSteerOrder(SO_L2);
			break;
		case -3:
			SetNowSteerOrder(SO_L3);
			break;
		case -4:
			SetNowSteerOrder(SO_L4);
			break;
	}
	*/
	static BOOL haziL = FALSE;	
	static BOOL haziR = FALSE;	


	switch(sys_frontLineSenser)
	{
		case FLS_WWWB:
			SetNowSteerOrder(SO_R5);
			haziR = TRUE;
			break;
		case FLS_WWBB:
			SetNowSteerOrder(SO_R4);
			break;
		case FLS_WWBW:
			if(haziR == TRUE){
				SetNowSteerOrder(SO_L2);
			}else{
				SetNowSteerOrder(SO_R3);
			}
			haziL = FALSE;
			break;
		
		case FLS_WBWW:
			if(haziL == TRUE){
				SetNowSteerOrder(SO_R2);
			}else{
				SetNowSteerOrder(SO_L3);
			}
			haziR = FALSE;
			break;
		case FLS_BBWW:
			SetNowSteerOrder(SO_L4);
			break;
		case FLS_BWWW:
			SetNowSteerOrder(SO_L5);
			haziL = TRUE;
			break;
		}
	return TRUE;
}

/**

**/
BOOL LineTrace()
{
	uint16 minDelta = 2;
	
	if(sys_frontLineSenser == FLS_WWWW && sys_rearLineSenser == RLS_WWWW)
	{
		if(param_d_state > 6 )
		{
			param_d_state = 6;
		}
		else if(param_d_state > 0 ) 
		{
			param_d_state -= 1;
		}
		else if(param_d_state < 0 ) 
		{
			param_d_state += 1;
		}
		else if(param_d_state < -6 ) 
		{
			param_d_state = -6;
		}
	}
	else
	{
		param_d_state = 0;
	}
	
	switch(sys_frontLineSenser)
	{
		case FLS_WWWB:
			param_d_state = -1 * (minDelta + 1);
			break;
		case FLS_WWBW:
			param_d_state = -1 * minDelta;
			break;
		case FLS_WBWW:
			param_d_state = minDelta;
			break;
		case FLS_BWWW:
			param_d_state = minDelta + 1;
			break;
	}
	/*
	switch(sys_rearLineSenser)
	{
		case RLS_WWWB:
			param_d_state += minDelta + 1;
			break;
		case RLS_WWBW:
			param_d_state += minDelta;
			break;
		case RLS_WBWW:
			param_d_state -= minDelta;
			break;
		case RLS_BWWW:
			param_d_state -= minDelta + 1;
			break;
		default:
			break;
	}
	
	if(sys_frontLineSenser == FLS_WWWB && sys_rearLineSenser == RLS_WWWB)
	{
		param_d_state = -1 * minDelta;
	}
	else if(sys_frontLineSenser == FLS_BWWW && sys_rearLineSenser == RLS_BWWW)
	{
		param_d_state = minDelta;
	}
	*/
/* k_sato */
    if(sys_frontLineSenser == FLS_WWWW && sys_rearLineSenser == RLS_BWWW){
        if(sys_nowSteerOrder <= 0){  
            param_d_state = 2;
        }
    }
 
    if(sys_frontLineSenser == FLS_WWWW && sys_rearLineSenser == RLS_WWWB){
        if(sys_nowSteerOrder >= 0){  
            param_d_state = -2;
        }
    }
/* k_sato End */	
	
	if(param_d_state > 7)
	{
		param_d_state = 7;
	}
	else if(param_d_state < -7)
	{
		param_d_state = -7;
	}
	
	switch(param_d_state)
	{
		case 7:
			SetNowSteerOrder(SO_L7);
			break;
		case 6:
			SetNowSteerOrder(SO_L6);
			break;
		case 5:
			SetNowSteerOrder(SO_L5);
			break;
		case 4:
			SetNowSteerOrder(SO_L4);
			break;
		case 3:
			SetNowSteerOrder(SO_L3);
			break;
		case 2:
			SetNowSteerOrder(SO_L2);
			break;
		case 1:
			SetNowSteerOrder(SO_L1);
			break;
		case 0:
			SetNowSteerOrder(SO_ST);
			break;
		case -1:
			SetNowSteerOrder(SO_R1);
			break;
		case -2:
			SetNowSteerOrder(SO_R2);
			break;
		case -3:
			SetNowSteerOrder(SO_R3);
			break;
		case -4:
			SetNowSteerOrder(SO_R4);
			break;
		case -5:
			SetNowSteerOrder(SO_R5);
			break;
		case -6:
			SetNowSteerOrder(SO_R6);
			break;
		case -7:
			SetNowSteerOrder(SO_R7);
			break;
	}
	return TRUE;
}

/**
@brief
現在のステートに合わせ、目標座標と現在の座標から操舵量を算出します。
詳細な処理は、各関数に遷ってから
*/
BOOL CaluculateSteer()
{
	BOOL err = FALSE;

	switch (sys_nowRouteState)
	{
	case RS_READY:					///<スタート待ち
		err |= CaluculateSteerReady();
		break;
	case RS_TO_1ST_VCP:				///<第一仮想チェックポイントまで
		err |= CaluculateSteerTo1stVCP();
		break;
	case RS_TO_2ND_VCP:
		err |= CaluculateSteerTo2ndVCP();
		break;
	case RS_TO_3RD_VCP:
		err |= CaluculateSteerTo3rdVCP();
		break;
	case RS_TO_4TH_VCP:
		err |= CaluculateSteerTo4thVCP();
		break;
	case RS_BW_4TH_5TH_VCP:
		err |= CalculateSteerBetween4thTo5th();
		break;
	case RS_TO_5TH_VCP:
		err |= CaluculateSteerTo5thVCP();
		break;
	case RS_TO_6TH_VCP:
		err |= CaluculateSteerTo6thVCP();
		break;
	case RS_TO_7TH_VCP:
		err |= CaluculateSteerTo7thVCP();
		break;
	case RS_TO_8TH_VCP:
		err |= CaluculateSteerTo8thVCP();
		break;
	case RS_TO_9TH_VCP:
		err |= CaluculateSteerTo9thVCP();
		break;
/* k_sato */
	case RS_TO_10TH_VCP:
		err |= CaluculateSteerTo10thVCP();
		break;
	case RS_TO_11TH_VCP:
		err |= CaluculateSteerTo11thVCP();
		break;
/* k_sato End */
	case RS_TO_GOAL:
		err |= CaluculateSteerToGoal();
		break;
	case RS_IN_GOAL:
		//なにもしない
		break;
	case RS_SYUMAI:
		
	case RS_CALIBRATION:				///直線補正計算用
		err |= CaluculateSteerCalibration();
		break;
	case RS_DEBUG_TYPE1:
	case RS_DEBUG_TYPE2:
	default:
		break;
	}

	return err;
}


/**
@brief
Ready状態の時の操舵を決めます。
と、言って停止中なはずなので、直進にします。

*/
static BOOL CaluculateSteerReady()
{
	SetNowSteerOrder(SO_ST);
	return FALSE;
}


/**
@brief
１ｓｔVCPに向かう時の操舵です。
基本的には直進ですが、メカの誤差によりブレが出た場合操舵を行います。
累積補正はかけません。現在座標と目的地差分から操舵を決める処理のみとします。

@note
X軸でのブレは速度ブレなので、ここでは操舵の計算に入れません。
Y軸方向にどれだけずれているかと現在のωを検討し補正操舵を入れます。

ズレ幅（絶対値）
Y  <= 2			操舵0
3  <= Y <  5	操舵１
5  <= Y <  10	操舵2
10 <  Y         操舵3 エラー
*/
static BOOL CaluculateSteerTo1stVCP()
{
	int32 deltaY;
	BOOL ret = FALSE;

	deltaY = sys_absoluteAxis_Y - param_target1stVCPAxis_Y;		//mm単位 int値

	if(abs(deltaY) <= 1000)
	{
		SetNowSteerOrder(SO_ST);
		return ret;
	}

	/* deltaYは目的地に対し、自車が右方向（-Y）にずれていたら値がマイナスに、左方向（＋Y）にずれていれば値はプラスになります */
	/* その為、deltaYがマイナスであれば左に曲がる処理を行います */
	if(deltaY < 0)
	{
		SetNowSteerOrder(SO_L1);
		/*
		//左曲り検討(-Yにずれてる)
		deltaY = abs(deltaY);
		
		if((1000 <= deltaY ) && (deltaY < 1500))
		{
			SetNowSteerOrder(SO_L1);
		}
		else if((1500 <= deltaY ) && (deltaY < 3500))
		{
			SetNowSteerOrder(SO_L2);
		}
		else
		{
			SetNowSteerOrder(SO_L3);
			if(param_ignorOverSteerError)
			{
				ERR_WriteID(EID_OVER_STREE, __func__, __LINE__);
				ret = TRUE;
			}
		}
		*/
	}
	else
	{
		SetNowSteerOrder(SO_R1);
		/*
		//右曲り検討(＋Yにずれてる)
		if((1000 <= deltaY ) && (deltaY < 1500))
		{
			SetNowSteerOrder(SO_R1);
		}
		else if((1500 <= deltaY ) && (deltaY < 3500))
		{
			SetNowSteerOrder(SO_R2);
		}
		else
		{
			SetNowSteerOrder(SO_R3);
			if(param_ignorOverSteerError)
			{
				ERR_WriteID(EID_OVER_STREE, __func__, __LINE__);
				ret = TRUE;
			}
		}
		*/
	}

	return ret;
}


/**
@brief
2ndVCPに向かう時の操舵です。
基本的には直進ですが、メカの誤差によりブレが出た場合操舵を行います。
累積補正はかけません。現在座標と目的地差分から操舵を決める処理のみとします。
1stVCPと違い速度を落としているので、目的地への移動精度を高めるようにします

@note

ズレ幅（絶対値）
Y  <= 1			操舵0
2  <= Y <  4	操舵１
4  <= Y <  6	操舵2
6 <  Y         操舵3 エラー
*/
static BOOL CaluculateSteerTo2ndVCP()
{
	int deltaY;
	BOOL ret = FALSE;

	deltaY = sys_absoluteAxis_Y - param_target2ndVCPAxis_Y;		//mm単位 int値

	if(abs(deltaY) <= 1000)
	{
		SetNowSteerOrder(SO_ST);
		return ret;
	}

	/* deltaYは目的地に対し、自車が右方向（-Y）にずれていたら値がマイナスに、左方向（＋Y）にずれていれば値はプラスになります */
	/* その為、deltaYがマイナスであれば左に曲がる処理を行います */
	if(deltaY < 0)
	{
		//左曲り検討(-Yにずれてる)
		SetNowSteerOrder(SO_L1);
		/*
		deltaY = abs(deltaY);

		if((2000 <= deltaY ) && (deltaY < 4000))
		{
			SetNowSteerOrder(SO_L1);
		}
		else if((4000 <= deltaY ) && (deltaY < 6000))
		{
			SetNowSteerOrder(SO_L2);
		}
		else
		{
			SetNowSteerOrder(SO_L3);
			if(param_ignorOverSteerError)
			{
				ERR_WriteID(EID_OVER_STREE, __func__, __LINE__);
				ret = TRUE;
			}
		}
		*/
	}
	else
	{
		//右曲り検討(＋Yにずれてる)
		SetNowSteerOrder(SO_R1);
		/*
		if((2000 <= deltaY ) && (deltaY < 4000))
		{
			SetNowSteerOrder(SO_R1);
		}
		else if((4000 <= deltaY ) && (deltaY < 6000))
		{
			SetNowSteerOrder(SO_R2);
		}
		else
		{
			SetNowSteerOrder(SO_R3);
			if(param_ignorOverSteerError)
			{
				ERR_WriteID(EID_OVER_STREE, __func__, __LINE__);
				ret = TRUE;
			}
		}
		*/
	}

	return ret;
}
/**
@brief
3rdVCPに向かう時の操舵です。

@note
*/
static BOOL CaluculateSteerTo3rdVCP()
{

	//直接targetmotorで決め打ちします。
	return FALSE;
}

/**
@brief
4thVCPに向かう時の操舵です。

@note
*/

static BOOL CaluculateSteerTo4thVCP()
{
	//処理しません
	SetNowSteerOrder(SO_L1);
	return FALSE;
}

/**
@brief
坂道直線

*/

static BOOL CalculateSteerBetween4thTo5th()
{
	SetNowSteerOrder(SO_ST);	
	param_d_state = 0;
	param_target5thVCPAxis_Y = sys_absoluteAxis_Y; //20161111
	return FALSE;
}

static BOOL CaluculateSteerTo5thVCP()
{
	AxisYTrace(param_target5thVCPAxis_Y); //20161111
	//LineTrace_k_sato();
	
	//LineTrace_old();
	/*
	static BOOL haziL = FALSE;	
	static BOOL haziR = FALSE;	


	switch(sys_frontLineSenser)
	{
		case FLS_WWWB:
			SetNowSteerOrder(SO_R3);
			haziR = TRUE;
			break;
		case FLS_WWBB:
			SetNowSteerOrder(SO_R4);
			break;
		case FLS_WWBW:
			if(haziR == TRUE){
				SetNowSteerOrder(SO_L2);
			}else{
				SetNowSteerOrder(SO_R3);
			}
			haziL = FALSE;
			break;
		
		case FLS_WBWW:
			if(haziL == TRUE){
				SetNowSteerOrder(SO_R2);
			}else{
				SetNowSteerOrder(SO_L3);
			}
			haziR = FALSE;
			break;
		case FLS_BBWW:
			SetNowSteerOrder(SO_L4);
			break;
		case FLS_BWWW:
			SetNowSteerOrder(SO_L3);
			haziL = TRUE;
			break;
		}	

*/
	return FALSE;
}
/**
@brief
坂道後、ちょっとだけ直進

*/
static BOOL CaluculateSteerTo6thVCP()
{
	static BOOL haziL = FALSE;	
	static BOOL haziR = FALSE;	


	switch(sys_frontLineSenser)
	{
		case FLS_WWWB:
			SetNowSteerOrder(SO_R5);
			haziR = TRUE;
			break;
		case FLS_WWBB:
			SetNowSteerOrder(SO_R4);
			break;
		case FLS_WWBW:
			if(haziR == TRUE){
				SetNowSteerOrder(SO_L1);
			}else{
				SetNowSteerOrder(SO_R3);
			}
			haziL = FALSE;
			break;
		
		case FLS_WBWW:
			if(haziL == TRUE){
				SetNowSteerOrder(SO_R1);
			}else{
				SetNowSteerOrder(SO_L3);
			}
			haziR = FALSE;
			break;
		case FLS_BBWW:
			SetNowSteerOrder(SO_L4);
			break;
		case FLS_BWWW:
			SetNowSteerOrder(SO_L5);
			haziL = TRUE;
			break;
		}	

	return FALSE;
}
/**
@brief

大きく右にターン
*/
static BOOL CaluculateSteerTo7thVCP()
{
	return FALSE;
}
/**
@brief
右にターン後の直進。
鍵字にぶつかるまで

*/
static BOOL CaluculateSteerTo8thVCP()
{
	static BOOL haziL = FALSE;	
	static BOOL haziR = FALSE;	


	switch(sys_frontLineSenser)
	{
		case FLS_WWWB:
			SetNowSteerOrder(SO_R5);
			haziR = TRUE;
			break;
		case FLS_WWBB:
			SetNowSteerOrder(SO_R4);
			break;
		case FLS_WWBW:
			if(haziR == TRUE){
				SetNowSteerOrder(SO_L1);
			}else{
				SetNowSteerOrder(SO_R3);
			}
			haziL = FALSE;
			break;
		
		case FLS_WBWW:
			if(haziL == TRUE){
				SetNowSteerOrder(SO_R1);
			}else{
				SetNowSteerOrder(SO_L3);
			}
			haziR = FALSE;
			break;
		case FLS_BBWW:
			SetNowSteerOrder(SO_L4);
			break;
		case FLS_BWWW:
			SetNowSteerOrder(SO_L5);
			haziL = TRUE;
			break;
		}	

	return FALSE;

}

/**
@brief
鍵字から障害物よける為右にターン

*/
static BOOL CaluculateSteerTo9thVCP()
{
	//LineTrace_old();
	
	//LineTrace();
	AxisYTrace(sys_absoluteAxis_Y);
	param_target10thVCPAxis_Y = sys_absoluteAxis_Y - 120000;
	return FALSE;
}

/* k_sato */
/**
@brief
右曲がり中。右がついたら次へ状態遷移。
ゴールへまっすぐ向かう為の準備期間
calculatemotortargetで設定する為、こちらでは何もしない
**/
static BOOL CaluculateSteerTo10thVCP()
{
	AxisYTrace(param_target10thVCPAxis_Y);
	param_target11thVCPAxis_Y = sys_absoluteAxis_Y + 30000;
	return FALSE;
}

/**
@brief
信地旋回(左タイヤをバックに回す)し、
後ろのセンサーがWWWBになったら次の状態へ。
これで大体ゴールへまっすぐ向いているはず。
calculatemotortargetで設定する為、こちらでは何もしない
**/
static BOOL CaluculateSteerTo11thVCP()
{
	AxisYTrace(param_target11thVCPAxis_Y);
	return FALSE;
}
/* k_sato End */

/**
@brief
右よけからのゴールめざし
AXISはum単位
*/
static BOOL CaluculateSteerToGoal()
{
/* k_sato */
#if 0
/*
	if(sys_absoluteAxis_X < 680000){
		if (sys_absoluteAxis_Y < -100000){
			SetNowSteerOrder(SO_L5);
		}else if(( sys_absoluteAxis_Y < -50000) && ( sys_absoluteAxis_Y >= -1000000)){
			SetNowSteerOrder(SO_L5);
		}else if((sys_absoluteAxis_Y < 0) && (sys_absoluteAxis_Y >= -50000)){
			SetNowSteerOrder(SO_L3);
		}else if((sys_absoluteAxis_Y > 0) && (sys_absoluteAxis_Y <= 50000)){
			SetNowSteerOrder(SO_R3);
		}else if((sys_absoluteAxis_Y > 50000) && (sys_absoluteAxis_Y <= 100000)){
			SetNowSteerOrder(SO_R4);
		}else if((sys_absoluteAxis_Y > 100000)){
			SetNowSteerOrder(SO_R5);
		}
	}

	//障害物越えた先からゴールまでの間
	if(sys_absoluteAxis_X >= 680000)
	{
		if (sys_absoluteAxis_Y < -100000){
			SetNowSteerOrder(SO_L5);
		}else if(( sys_absoluteAxis_Y < -50000) && ( sys_absoluteAxis_Y >= -1000000)){
			SetNowSteerOrder(SO_L3);
		}else if((sys_absoluteAxis_Y < 0) && (sys_absoluteAxis_Y >= -50000)){
			SetNowSteerOrder(SO_L2);
		}else if((sys_absoluteAxis_Y > 0) && (sys_absoluteAxis_Y <= 50000)){
			SetNowSteerOrder(SO_R2);
		}else if((sys_absoluteAxis_Y > 50000) && (sys_absoluteAxis_Y <= 100000)){
			SetNowSteerOrder(SO_R3);
		}else if((sys_absoluteAxis_Y > 100000)){
			SetNowSteerOrder(SO_R5);
		}
	}
*/
	int state = 0;
	switch(sys_frontLineSenser)
	{
		case FLS_WWWB:
			state = 2;
			break;
		case FLS_WWBW:
			state = 1;
			break;
		case FLS_WBWW:
			state = -1;
			break;
		case FLS_BWWW:
			state = -2;
			break;
	}
	
	switch(sys_rearLineSenser)
	{
		case RLS_WWWB:
			state -= 2;
			break;
		case RLS_WWBW:
			state -= 1;
			break;
		case RLS_WBWW:
			state += 1;
			break;
		case RLS_BWWW:
			state += 2;
			break;
		default:
			break;
	}
	
	if(sys_frontLineSenser == FLS_WWWB && sys_rearLineSenser == RLS_WWWB)
	{
		state = 1;
	}
	else if(sys_frontLineSenser == FLS_BWWW && sys_rearLineSenser == RLS_BWWW)
	{
		state = -1;
	}
	
	
	switch(state)
	{
		case 4:
			SetNowSteerOrder(SO_R4);
			break;
		case 3:
			SetNowSteerOrder(SO_R3);
			break;
		case 2:
			SetNowSteerOrder(SO_R2);
			break;
		case 1:
			SetNowSteerOrder(SO_R1);
			break;
		case 0:
			SetNowSteerOrder(SO_ST);
			break;
		case -1:
			SetNowSteerOrder(SO_L1);
			break;
		case -2:
			SetNowSteerOrder(SO_L2);
			break;
		case -3:
			SetNowSteerOrder(SO_L3);
			break;
		case -4:
			SetNowSteerOrder(SO_L4);
			break;
	}
#else
	int32 deltaY;

	//エンコーダで処理
	//sys_absoluteAxis_Yはjudgestateで初期済み
	deltaY = sys_absoluteAxis_Y - param_targetGoalAxis_Y;		//mm単位 int値

	if(abs(deltaY) <= 1000)
	{
		SetNowSteerOrder(SO_ST);
		return FALSE;
	}

	/* deltaYは目的地に対し、自車が右方向（-Y）にずれていたら値がマイナスに、左方向（＋Y）にずれていれば値はプラスになります */
	/* その為、deltaYがマイナスであれば左に曲がる処理を行います */
	if(deltaY < 0)
	{
		SetNowSteerOrder(SO_L1);
	}
	else
	{
		SetNowSteerOrder(SO_R1);
	}
#endif
/* k_sato End */

	return FALSE;
}
/**
@brief


*/
static BOOL CaluculateSteerCalibration()
{
	if(sys_frontLineSenser == FLS_WWWW && sys_rearLineSenser==RLS_WWWW)
	{
		return FALSE;
	}
	
	int state = 0;
	switch(sys_frontLineSenser)
	{
		case FLS_WWWB:
			state = 2;
			break;
		case FLS_WWBW:
			state = 1;
			break;
		case FLS_WBWW:
			state = -1;
			break;
		case FLS_BWWW:
			state = -2;
			break;
	}
	
	switch(sys_rearLineSenser)
	{
		case RLS_WWWB:
			state -= 2;
			break;
		case RLS_WWBW:
			state -= 1;
			break;
		case RLS_WBWW:
			state += 1;
			break;
		case RLS_BWWW:
			state += 2;
			break;
		default:
			break;
	}
	
	if(sys_frontLineSenser == FLS_WWWB && sys_rearLineSenser == RLS_WWWB)
	{
		state = 1;
	}
	else if(sys_frontLineSenser == FLS_BWWW && sys_rearLineSenser == RLS_BWWW)
	{
		state = -1;
	}
	
/* k_sato */
    if(sys_frontLineSenser == FLS_WWWW && sys_rearLineSenser == RLS_BWWW){
        if(sys_nowSteerOrder <= 0){  
            state = -2;
        }
    }
 
    if(sys_frontLineSenser == FLS_WWWW && sys_rearLineSenser == RLS_WWWB){
        if(sys_nowSteerOrder >= 0){  
            state = 2;
        }
    }
/* k_sato End */	
	/*
	switch(state)
	{
		case 4:
			SetNowSteerOrder(SO_R4);
			break;
		case 3:
			SetNowSteerOrder(SO_R3);
			break;
		case 2:
			SetNowSteerOrder(SO_R2);
			break;
		case 1:
			SetNowSteerOrder(SO_R1);
			break;
		case 0:
			SetNowSteerOrder(SO_ST);
			break;
		case -1:
			SetNowSteerOrder(SO_L1);
			break;
		case -2:
			SetNowSteerOrder(SO_L2);
			break;
		case -3:
			SetNowSteerOrder(SO_L3);
			break;
		case -4:
			SetNowSteerOrder(SO_L4);
			break;
	}
	*/
	return FALSE;
}

static BOOL SetNowSteerOrder(steer_order newOrder)
{

	//想定外の入力要求チェック
	if(abs((int32)newOrder) >= 100)
	{
		ERR_WriteID(EID_RANGE_OVER, __func__, __LINE__);
		return TRUE;
	}

	//オーダーに変更があった場合にのみ、ステアオーダーのログ更新処理を行う
	if(newOrder != sys_nowSteerOrder)
	{
		sys_steerOrderLog[4]		= sys_steerOrderLog[3];
		sys_steerOrderLog[3]		= sys_steerOrderLog[2];
		sys_steerOrderLog[2]		= sys_steerOrderLog[1];
		sys_steerOrderLog[1]		= sys_steerOrderLog[0];
		sys_steerOrderLog[0]		= sys_nowSteerOrder;
		sys_nowSteerOrder			= newOrder;

	}

	return FALSE;
}


