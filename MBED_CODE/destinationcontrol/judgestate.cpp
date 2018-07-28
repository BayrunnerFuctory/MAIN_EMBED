#include "mbed.h"
#include "typedef.h"
#include "sysdata.h"
#include "parameter.h"
#include "errorcontrol.h"
#include "judgestate.h"
#include "communication.h"

#include "destinationcontrol.h"
#include "deviceinput.h"

route_state	sys_nowRouteState				= RS_UNKNOWN;					///< 現在の位置
route_state	sys_lastRouteState				= RS_UNKNOWN;					///< 前回の位置（ラインロスト時に利用）
route_state	sys_firstState					= RS_TO_1ST_VCP; //RS_TO_8TH_VCP; //RS_TO_1ST_VCP; //RS_TO_5TH_VCP; //RS_TO_1ST_VCP;				///< スタートスイッチを押した後のステート

int32		param_distance1stVCPAxisX = 2530000;//2780000;								///<1stVCP 単位:um
int32		param_distance2ndVCPAxisX = 2680000;//2930000;//test
int32		param_distanceSlopCenterAxisX = 3030000;										///坂道の真ん中辺り
int32		param_distance6thVCPignoreRelativeAxisY;							///6thVCPを判断するにあたり、センサー反応を無視するY軸方向相対距離
int32		param_distance7thVCPignoreRelativeAxisX;							///7thVCPを判断するにあたり、センサー反応を無視するX軸方向相対距離

int32       param_distance8thInBorderAxisY = 1400000;
int32       param_distance10thVCPAxisX = 700000;

int32		sys_loggingVCPAxisX[10];
int32		sys_loggingVCPAxisY[10];

int32       sys_toGoalSubState = 0;

BOOL firstTurnLineSenser = false;
BOOL firstTurnBodyRadian = false;
float64 VCP6th_angularRadian = 0;

static BOOL JudgeReady(void);
static BOOL JudgeTo1stVCP(void);
static BOOL JudgeTo2ndVCP(void);
static BOOL JudgeTo3rdVCP(void);
static BOOL JudgeTo4thVCP(void);
static BOOL JudgeBetween4thTo5thVCP(void);
static BOOL JudgeTo5thVCP(void);
static BOOL JudgeTo6thVCP(void);
static BOOL JudgeTo7thVCP(void);
static BOOL JudgeTo8thVCP(void);
static BOOL JudgeTo9thVCP(void);
/* k_sato */
static BOOL JudgeTo10thVCP(void);
static BOOL JudgeTo11thVCP(void);
/* k_sato End */
static BOOL JudgeToGoal(void);
static BOOL JudgeInGoal(void);
static BOOL JudgeSyumai(void);
static BOOL JudgeCaribration(void);
static void SetNowRouteState(route_state now);
static void LoggingVCPAxis(route_state now);

/**
 *   sys_absoluteAxis_Y
 *   |                                                                                       |G
 *   |           VCP8                                                                        |O
 *   |                                     VCP9（障害物よけて、ラインにさわった時                 |A
 *   |                       VCP8(最後の90度カーブ、ゴールと平行線）                              |L
 *   |   VCP7(車体90°向いたらゆるく旋回)                                                                                    |
 *   |              ------------------------------------------------------------------------
 *   |
 *   |    
 *   |    VCP6(坂上縦ライン踏んだら急旋回)
 *   |             VCP5(坂の頂点 黒線上)                                VCP4-2（坂入る前黒線上で姿勢整える）
 *   |                                                                                     VCP4(真ん中から緩め旋回で姿勢整え)
 *   |
 *   |                                                                                     VCP3(コの字カーブの真ん中辺りで緩めの旋回へ)
 *   |
 *   |
 *   |------------------------------------------------------------------
 *   |                                                       VCP1(コーナー前減速ポイント)  VCP2（ライン踏んだら急旋回開始）
 *   |     /           |---------|
 *   |    /            |         |  -->
 *   |   /             |         |  -->
 *   |  /              |---------|
 *   | /
 *   |/ ) rad  sys_angularVelocity
 *  0|------------------------------------------------------------------------------------------  sys_absoluteAxis_X
 *   0
 *
 */




void JudgeStateInit()
{

	sys_nowRouteState = RS_READY;
	sys_lastRouteState = RS_READY;
}


/*
@brief
現在のステートを判断します


*/
BOOL JudgeState()
{
	BOOL err = FALSE;

	switch (sys_nowRouteState)
	{
	case RS_READY:					///<スタート待ち
		err |= JudgeReady();
		break;
	case RS_TO_1ST_VCP:				///<第一仮想チェックポイントまで
		err |= JudgeTo1stVCP();
		break;
	case RS_TO_2ND_VCP:
		err |= JudgeTo2ndVCP();
		break;
	case RS_TO_3RD_VCP:
		err |= JudgeTo3rdVCP();
		break;
	case RS_TO_4TH_VCP:
		err |= JudgeTo4thVCP();
		break;
	case RS_BW_4TH_5TH_VCP:
		err |= JudgeBetween4thTo5thVCP();
		break;
	case RS_TO_5TH_VCP:
		err |= JudgeTo5thVCP();
		break;
	case RS_TO_6TH_VCP:
		err |= JudgeTo6thVCP();
		break;
	case RS_TO_7TH_VCP:
		err |= JudgeTo7thVCP();
		break;
	case RS_TO_8TH_VCP:
		err |= JudgeTo8thVCP();
		break;
	case RS_TO_9TH_VCP:
		err |= JudgeTo9thVCP();
		break;
/* k_sato */
	case RS_TO_10TH_VCP:
		err |= JudgeTo10thVCP();
		break;
	case RS_TO_11TH_VCP:
		err |= JudgeTo11thVCP();
		break;
/* k_sato End */
	case RS_TO_GOAL:
		err |= JudgeToGoal();
		break;
	case RS_IN_GOAL:
		//なにもしない
		err |= JudgeInGoal();
		break;
	case RS_SYUMAI:
		err |= JudgeSyumai();
		break;
	case RS_CALIBRATION:				///直線補正計算用
		err |= JudgeCaribration();
	case RS_DEBUG_TYPE1:
	case RS_DEBUG_TYPE2:
	default:
		break;
	}

	return err;
}

/**
@brief
Ready状態中の処理です

@note
スタートスイッチ判断で次のステートへ移ります。
デバッグ用にわざと次のステートを変更可能にしています。
その為基本的にはsys_firstState == RS_TO_1ST_VCP
になります。
sys_firstStateをデバッグツールで書き換えて最初の行動を変更し
テストしてください。
*/
static BOOL JudgeReady()
{
	if ((sys_enableStartSwitch == TRUE) && (sys_nowRouteState == RS_READY))
	{
		SetNowRouteState (sys_firstState);
	}

	return FALSE;
}

/**
@brief
スタートからVCP1到着判断です
一定以上の進んだX座標のみで判断します。
スタートからVCP1までは直線の為Y座標判断は入れません。
@note
最初の左曲りコーナー前のポイントになります
ここにポイントを置いたのは今回のロボットの仕様上コーナー前で減速しないとコーナリング時に慣性に吹き飛ばされると
考えたためです。
スタートから直進、最高速を目指して加速、VCP1でコーナリングの為の減速→VCP2を越えてコーナーリング処理
@attention
座標系と閾値の単位に気を付けてください。
単位は必ず合わせるように!!
*/
static BOOL JudgeTo1stVCP()
{
	if(sys_absoluteAxis_X >= param_distance1stVCPAxisX)	//単位はmm同士 VCP1を越えたと判断
	{
		SetNowRouteState(RS_TO_2ND_VCP);
	}
	return FALSE;
}



/**
@brief
VCP2到着判断
一定以上の進んだX座標のみで判断します。
スタートからVCP1までは直線の為Y座標判断は入れません。
@note
ここからコーナリング処理を始めます。
スピード、操舵に関してはこの後に続く関数群である
CaluculateSteer、CaluculateTargetSpeedで確定します。

@attention
座標系と閾値の単位に気を付けてください。
単位は必ず合わせるように!!
*/
static BOOL JudgeTo2ndVCP()
{
	//if(sys_absoluteAxis_X >= param_distance2ndVCPAxisX)	//単位はmm同士 VCP1を越えたと判断
	//if((sys_frontLineSenser == FLS_WBBW) ||	(sys_frontLineSenser == FLS_BBBB) || (sys_frontLineSenser == FLS_BBBW) || (sys_frontLineSenser == FLS_WBBB))
	if(sys_frontLineSenser != FLS_WWWW)
	{
		SetNowRouteState(RS_TO_3RD_VCP);
	}
	else if(sys_absoluteAxis_X >= param_distance2ndVCPAxisX)
	{
		//SetNowRouteState(RS_TO_3RD_VCP);
	}

	return FALSE;
}


/**
@brief
VCP3到着判断
車体、前右端センサーが黒色を判断したらVCP3とします
他のセンサーは判断に使いません。
他のフロントセンサーが反応するような向きで入ったらそれはエラーシステムテストで潰すこと。
@note
@attention
*/
static BOOL JudgeTo3rdVCP()
{
	BOOL ret = FALSE;
	if(sys_angularRadian > 1.4) //2.5 
	{
		SetNowRouteState(RS_TO_4TH_VCP);
		
		if(sys_frontLineSenser == FLS_BWWW || sys_frontLineSenser == FLS_BBWW || sys_frontLineSenser == FLS_BBBW || sys_frontLineSenser == FLS_BBBB) 
		{
//			SetNowRouteState(RS_TO_5TH_VCP);
		}
	}
	
	return ret;
}



/**
@brief
VCP4到着判断
車体、前中右センサーが黒色を判断したらVCPとします
場合により、前中左センサー、右端センサーも反応する可能性があるので、ORで取ります
但し、前中左センサーだけ反応することは有りえないので、それはエラーとします。

OK条件
WWBW
WBBW

NGとしたいけど、テストで潰せなかったらOKにする
WWBB #if 0で処理。念のため取っておく

他のフロントセンサーが反応するような向きで入ったらそれはエラー。システムテストで潰すこと。
@note
@attention
*/
static BOOL JudgeTo4thVCP()
{/*
	BOOL ret = FALSE;

	if((sys_frontLineSenser == FLS_WWBW) || (sys_frontLineSenser == FLS_WBBW))
	{
		sys_tripCount = 0;
		SetNowRouteState(RS_TO_5TH_VCP);
	}
	else if(sys_frontLineSenser == FLS_WWBB)
	{
#if FALSE
		SetNowRouteState(RS_TO_5TH_VCP);
#else
		ERR_WriteID(EID_FAIL_TO_ARRIVE_VCP4, __func__, __LINE__);
		ret = TRUE;
#endif
	}
	else
	{
		//ERR_WriteID(EID_FAIL_TO_ARRIVE_VCP4, __func__, __LINE__);
		//ret = TRUE;
	}

	return ret;
	*/
	
	BOOL ret = false;
	
	if(sys_frontLineSenser != FLS_BWWW && sys_frontLineSenser != FLS_WWWW )
	{
		//sys_tripCount = 0;
		SetNowRouteState(RS_BW_4TH_5TH_VCP);
		//return TRUE;
	}
	
	return ret;
}

static BOOL JudgeBetween4thTo5thVCP()
{
	BOOL ret = false;
	
	
	if(sys_rearLineSenser != RLS_BWWW && sys_rearLineSenser != RLS_WWWW )
	{
		firstTurnLineSenser = TRUE;
		//return TRUE;
		//SetNowRouteState(RS_TO_5TH_VCP);
	}
	
	if(sys_angularRadian > 3.1) //3.31
	{
		//sys_angularRadian = 3.1;
		firstTurnBodyRadian = TRUE;
	}
	
	if(firstTurnLineSenser && firstTurnBodyRadian)
	{
		firstTurnLineSenser = FALSE;
		sys_angularRadian = 3.14;//20161111 車体角度を180°に強制変更 [未チェック]
		
		SetNowRouteState(RS_TO_5TH_VCP);
	}
	
	return ret;
}

/**
@brief
VCP5到着判断
坂道の終端で、メカ前方が床から離れる。
この時前センサーすべてが一時的に黒になる（ハードウェア的にならざるを得ない）ので、そのタイミングを判定とする。
念のため、絶対X座標が1600未満（坂道の真ん中ぐらいを越えている）の&&も加える

@note
@attention
*/
static BOOL JudgeTo5thVCP()
{
	if(sys_tripCount > 500)
	{
		if((sys_frontLineSenser == FLS_BBBW || sys_frontLineSenser == FLS_BBBB || sys_frontLineSenser == FLS_WBBB))// && sys_absoluteAxis_X < 1600000)
		{
			SetNowRouteState(RS_TO_6TH_VCP);
			comm.printf("----noborikiri------\r\n");
			sys_tripCount = 0;
		}
	}
	return FALSE;
}


/**
@brief
VCP6到着判断
坂道から上がり、ある一定以上Y軸方向に行った後、前左端センサーが反応した箇所をVCP6と判断する。

@note
坂道から上がるときは中央センサーでライン取りしてる為、X軸方向黒線に左端センサーが反応してしまう。
その為、ある一定以上Y軸方向に移動するまで左端センサーの反応を無視するようにします。

*/
static BOOL JudgeTo6thVCP()
{
	/*
	if(sys_tripCount > 80) 
	{
		sys_absoluteAxis_X = 0;
		sys_absoluteAxis_Y = 0;
		sys_angularRadian = 0;
		SetNowRouteState(RS_TO_7TH_VCP);
	}
*/
	//if(sys_frontLineSenser == FLS_WWWB || sys_frontLineSenser == FLS_WWBB || sys_frontLineSenser == FLS_WBBB || sys_frontLineSenser == FLS_BBBB  )
	if(sys_rearLineSenser == RLS_WBBB || sys_rearLineSenser == RLS_BBBW || sys_rearLineSenser == RLS_BBBB || sys_rearLineSenser == RLS_WWBB|| sys_rearLineSenser == RLS_BBWW || sys_rearLineSenser == RLS_WBBW )
	{
		//sys_absoluteAxis_X = 0;
		//sys_absoluteAxis_Y = 0;
		VCP6th_angularRadian = sys_angularRadian;
		if(sys_absoluteAxis_Y < param_distance8thInBorderAxisY)
		{
			sys_outInFlag = TRUE;
		}
		else
		{
			sys_outInFlag = FALSE;
		}
		SetNowRouteState(RS_TO_7TH_VCP);
	}
	return FALSE;
}

/**
@brief
VCP7到着判断
VCP6からある一定以上X軸方向に行った後、前左端センサーが反応した箇所をVCP7と判断する。

@note
*/
static BOOL JudgeTo7thVCP()
{

	if(sys_angularRadian - VCP6th_angularRadian < -1.4)
	{
		SetNowRouteState(RS_TO_8TH_VCP);
	}
	/*
	if(sys_angularRadian < -2.7)
	{
		if((sys_frontLineSenser == FLS_WWBW) ||
		   (sys_frontLineSenser == FLS_WBWW) ||
		   (sys_frontLineSenser == FLS_BWWW))
		{
			SetNowRouteState(RS_TO_8TH_VCP);
		}
	}
*/
	return FALSE;
}


/**
@brief
VCP8到着判断

@note
*/
static BOOL JudgeTo8thVCP()
{
/*
	if((sys_frontLineSenser == FLS_BBBB) ||
	   (sys_frontLineSenser == FLS_WBBB) ||
	   (sys_frontLineSenser == FLS_WWBB))
	{
		//ここでカウンターリセットする
		sys_absoluteAxis_X = 0;
		sys_absoluteAxis_Y = 0;
		sys_angularRadian = 0;
		
		SetNowRouteState(RS_TO_9TH_VCP);
		
	}
	*/
	
	if((sys_frontLineSenser == FLS_BWWW || sys_frontLineSenser == FLS_BBWW || sys_frontLineSenser == FLS_WBWW) && (sys_angularRadian < 1.2))
	{
		SetNowRouteState(RS_TO_9TH_VCP);
	}
	return FALSE;
}

static BOOL JudgeTo9thVCP()
{
	//直角ライン判定
	if(sys_frontLineSenser == FLS_WWBB || sys_frontLineSenser == FLS_WBBB){
/* k_sato */
//		SetNowRouteState(RS_TO_GOAL); //動作確認用停止コマンド
		sys_angularRadian = 0;//20161111 直角ラインを見つけたら車体角度を0°に強制変更 [未チェック]
		SetNowRouteState(RS_TO_10TH_VCP);
/* k_sato End */
	}
	
	return FALSE;
}

/* k_sato */
/**
@brief
右曲がり中。右がついたら次へ状態遷移。
ゴールへまっすぐ向かう為の準備期間
**/
static BOOL JudgeTo10thVCP()
{
	/*
	if(firstTurnLineSenser)
	{
		SetNowRouteState(RS_TO_11TH_VCP);
	}
	
	//ライン上でここに状態遷移した場合、間違って↓判定してしまうかも。
	//直角後のライン到達判定
	if(sys_frontLineSenser == FLS_WWWB || sys_frontLineSenser == FLS_WWBB || sys_frontLineSenser == FLS_WBBB || sys_frontLineSenser == FLS_BBBB)
	{	
		firstTurnLineSenser = TRUE;
	}
	*/
	if(sys_absoluteAxis_X > param_distance10thVCPAxisX)
	{
		//return TRUE;
//		SetNowRouteState(RS_TO_11TH_VCP);
		SetNowRouteState(RS_TO_GOAL); //20161111 11thは不要になったので、チェックせずにゴールへ
	}
	
	return FALSE;
}

/**
@brief
信地旋回(左タイヤをバックに回す)し、
後ろのセンサーがWWWBになったら次の状態へ。
これで大体ゴールへまっすぐ向いているはず。
**/
static BOOL JudgeTo11thVCP()
{
	//ゴールへ車体を向ける。
	if(sys_rearLineSenser == RLS_WBBB || sys_rearLineSenser == RLS_BBBB){
		//ここでカウンターリセットする

		return TRUE;
		SetNowRouteState(RS_TO_GOAL);
	}
	
	return FALSE;
}
/* k_sato End */


/**
@brief
ゴール到着判断
後ろのセンサーが全黒になったらゴール

@note
*/
static BOOL JudgeToGoal()
{
/* k_sato */
#if 0
//	if (sys_absoluteAxis_X > 680000 + 180000 + 820000 + 100000 /*ゴールにうまる為*/){
/*	if((sys_goalCheckBtnL == TRUE) || (sys_goalCheckBtnR == TRUE))
	{
		sys_tripCount = 0;
		SetNowRouteState(RS_IN_GOAL);
	}
	*/

	//直角ライン回避
	if (sys_frontLineSenser == FLS_WBWW)
	{
		return TRUE;//動作確認用停止コマンド
		SetNowRouteState(RS_IN_GOAL);
	}
#else
	//if(sys_rearLineSenser == RLS_BBBB){
	if(sys_rearLineSenser == RLS_WBBB || sys_rearLineSenser == RLS_BBBB){
		SetNowRouteState(RS_IN_GOAL); //全黒から全白になったらゴール判定
	}
#endif
/* k_sato End */

	return FALSE;
}

/**
@brief
後ろのセンサーが全白になったら車体が全てゴール内に入った。
ペットボトル操作へ遷移
**/
static BOOL JudgeInGoal()
{
/* k_sato */
#if 0
	//ゴールまでライントレース
	if (sys_frontLineSenser == FLS_BBWW || sys_frontLineSenser == FLS_WBBW || sys_frontLineSenser == FLS_WWBB 
	|| sys_frontLineSenser == FLS_WBBB || sys_frontLineSenser == FLS_BBBW || sys_frontLineSenser == FLS_BBBB)
	{
		return TRUE;//動作確認用停止コマンド
		SetNowRouteState(RS_SYUMAI); 　　
	}
#else
	if(sys_rearLineSenser == RLS_WWWW){//|| sys_rearLineSenser == RLS_BWWW || sys_rearLineSenser == RLS_WWWB){
		SetNowRouteState(RS_SYUMAI); //全黒から全白になったらゴール判定
	}
#endif
/* k_sato End */

	return FALSE;
}

/**

**/
static BOOL JudgeSyumai()
{
	return FALSE;
}

/**
@brief
ルートステートの変更処理

ステートが変更＝チェックポイント通過 -> トリップメータのリセット&次ステート不可設定
なので、同時にトリップメータと次ステートフラグもリセットします。
トリップメータのリセット、次ステートフラグ処理が必要ないステート変更は行わないように注意してください。
*/
static void SetNowRouteState(route_state now)
{
	sys_lastRouteState		= sys_nowRouteState;
	sys_nowRouteState		= now;
	sys_tripCount			= 0;
	LoggingVCPAxis(now);
}

/**
@brief
デバッグ、確認用にVCPを通過したと判断したタイミングの絶対座標を取得する。

*/
static void LoggingVCPAxis(route_state now)
{

	switch (now)
	{
	case RS_TO_1ST_VCP:
		sys_loggingVCPAxisX[0]	= sys_absoluteAxis_X;
		sys_loggingVCPAxisY[0]	= sys_absoluteAxis_Y;
		break;
	case RS_TO_2ND_VCP:
		sys_loggingVCPAxisX[1]	= sys_absoluteAxis_X;
		sys_loggingVCPAxisY[1]	= sys_absoluteAxis_Y;
		break;
	case RS_TO_3RD_VCP:
		sys_loggingVCPAxisX[2]	= sys_absoluteAxis_X;
		sys_loggingVCPAxisY[2]	= sys_absoluteAxis_Y;
		break;
	case RS_TO_4TH_VCP:
		sys_loggingVCPAxisX[3]	= sys_absoluteAxis_X;
		sys_loggingVCPAxisY[3]	= sys_absoluteAxis_Y;
		break;
	case RS_TO_5TH_VCP:
		sys_loggingVCPAxisX[4]	= sys_absoluteAxis_X;
		sys_loggingVCPAxisY[4]	= sys_absoluteAxis_Y;
		break;
	case RS_TO_6TH_VCP:
		sys_loggingVCPAxisX[5]	= sys_absoluteAxis_X;
		sys_loggingVCPAxisY[5]	= sys_absoluteAxis_Y;
		break;
	case RS_TO_7TH_VCP:
		sys_loggingVCPAxisX[6]	= sys_absoluteAxis_X;
		sys_loggingVCPAxisY[6]	= sys_absoluteAxis_Y;
		break;
	case RS_TO_8TH_VCP:
		sys_loggingVCPAxisX[7]	= sys_absoluteAxis_X;
		sys_loggingVCPAxisY[7]	= sys_absoluteAxis_Y;
		break;
	case RS_TO_GOAL:
		sys_loggingVCPAxisX[8]	= sys_absoluteAxis_X;
		sys_loggingVCPAxisY[8]	= sys_absoluteAxis_Y;
		break;
	case RS_IN_GOAL:
		sys_loggingVCPAxisX[9]	= sys_absoluteAxis_X;
		sys_loggingVCPAxisY[9]	= sys_absoluteAxis_Y;
		break;
	case RS_CALIBRATION:
	case RS_DEBUG_TYPE1:
	case RS_DEBUG_TYPE2:
	default:
		break;
	}



}


static BOOL JudgeCaribration()
{
	if(sys_odoMeter > 100000)
	{
		//return TRUE;
	}
	
	return FALSE;
}
