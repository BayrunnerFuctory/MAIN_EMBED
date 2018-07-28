#pragma once


typedef unsigned char       BOOL;
typedef char       			uint8;      //この処理系ではcharは unsigned扱いです ->see handbook http://mbed.org/handbook/C-Data-Types
typedef unsigned short      uint16;
typedef unsigned long       uint32;
typedef unsigned long long  uint64;

typedef signed char         int8; 
typedef signed short        int16;
typedef signed long         int32;
typedef signed long long    int64;

typedef float               float32;
typedef double              float64;

static const BOOL FALSE = 0;
static const BOOL TRUE = 1;

static const float32 PI = 3.1415;

#define UINT8_MAX           (255)
#define UINT16_MAX          (65535)
#define UINT32_MAX          (4294967295)
#define INT32_MAX           (2147483647)
#define INT32_MIN           (-2147483648)


/**
現在位置、現在状況
*/
enum route_state
{
	RS_UNKNOWN,					///<未初期化、エラー等                ID = 0
	RS_READY,					///<スタート待ち
	RS_TO_1ST_VCP,				///<第一仮想チェックポイントまで
	RS_TO_2ND_VCP,				///<第二仮想チェックポイントまで
	RS_TO_3RD_VCP,
	RS_TO_4TH_VCP,
	RS_TO_5TH_VCP,
	RS_TO_6TH_VCP,
	RS_TO_7TH_VCP,
	RS_TO_8TH_VCP,
	RS_TO_9TH_VCP,
	RS_TO_10TH_VCP, //k_sato
	RS_TO_11TH_VCP, //k_sato
	RS_TO_GOAL,
	RS_IN_GOAL,
	RS_SYUMAI,
	RS_CALIBRATION,				// ID=16 ///直線補正計算用
	RS_DEBUG_TYPE1,
	RS_DEBUG_TYPE2,
	RS_BW_4TH_5TH_VCP			//ID=19
	};


/***
@brief
車体前面にあるセンサーの状態を表したenumです
@note
*/
enum flont_line_senser
{
	FLS_UNKNOWN	= 0xFF,
	FLS_WWWW = 0x0F,		//白白白白
	FLS_WWWB = 0x0E,
	FLS_WWBW = 0x0D,
	FLS_WWBB = 0x0C,
	FLS_WBWW = 0x0B,
	FLS_WBWB = 0x0A,
	FLS_WBBW = 0x09,
	FLS_WBBB = 0x08,
	FLS_BWWW = 0x07,
	FLS_BWWB = 0x06,
	FLS_BWBW = 0x05,
	FLS_BWBB = 0x04,
	FLS_BBWW = 0x03,
	FLS_BBWB = 0x02,
	FLS_BBBW = 0x01,
	FLS_BBBB = 0x00		//黒黒黒黒
};

enum rear_line_senser
{
	RLS_UNKNOWN	= 0xFF,
	RLS_WWWW = 0x0F,		//白白白白
	RLS_WWWB = 0x0E,
	RLS_WWBW = 0x0D,
	RLS_WWBB = 0x0C,
	RLS_WBWW = 0x0B,
	RLS_WBWB = 0x0A,
	RLS_WBBW = 0x09,
	RLS_WBBB = 0x08,
	RLS_BWWW = 0x07,
	RLS_BWWB = 0x06,
	RLS_BWBW = 0x05,
	RLS_BWBB = 0x04,
	RLS_BBWW = 0x03,
	RLS_BBWB = 0x02,
	RLS_BBBW = 0x01,
	RLS_BBBB = 0x00		//黒黒黒黒
	
//	RLS_WW = 0x03,		//白白
//	RLS_WB = 0x02,
//	RLS_BW = 0x01,
//	RLS_BB = 0x00		//黒黒
};





enum modeswitch{
    MS_0,
	MS_1,
	MS_2,
	MS_3
};

/**
@note
引き継ぎの方へ
曲がる力が強ければ強いほど絶対値を大きくするようにしてください。
if文で一定以上の曲りの強さならば○○するというようにプログラムが
書けるよう設計しています。
ex)  if(SO_R1 > hoge) {// 右に少しでも曲がっているならｘｘする   }
ex2) if(SO_L3 >= hoge2){ //左に強く曲がっているのならｘｘする }
*/
enum steer_order
{
	SO_COUNTER_ROTATION_COUNTER_CLOCK	= 99,		//超信地旋回 時計回り(デバッグ命令の時のみ有効) 左右タイヤ反転
	SO_PIVOT_TURN_COUNTER_CLOCK			= 90,		//信地旋回 時計回り(デバッグ命令の時のみ有効)	左タイヤのみ回す
	SO_L7								= 7,		//最小旋回
	SO_L6								= 6,
	SO_L5								= 5,		//強いカーブ
	SO_L4								= 4,
	SO_L3								= 3,
	SO_L2								= 2,		//曲りが分かるレベル
	SO_L1								= 1,		//弱い左カーブ 微調整レベル
	SO_ST								= 0,		//直進
	SO_R1								= -1,		//弱い右カーブ 微調整レベル
	SO_R2								= -2,
	SO_R3								= -3,
	SO_R4								= -4,
	SO_R5								= -5,     	//強い右カーブ
	SO_R6								= -6,
	SO_R7								= -7,		//最小旋回
	SO_PIVOT_TURN_CLOCK					= -90,		//信地旋回 時計回り(デバッグ命令の時のみ有効)右タイヤのみ回す
	SO_COUNTER_ROTATION_CLOCK			= -99,		//超信地旋回 時計回り(デバッグ命令の時のみ有効) 左右タイヤ反転

};



//速度ゲイン
enum speedgain{
    SG_P20,         //+20%
    SG_P10,
    SG_P5,
    SG_NO_GAIN,
    SG_N5,    
    SG_N10,
    SG_N20          //-20%
};

enum targetspeed{
	TS_READY,				//スタート地点
	TS_TO_1ST_VCP,			//1stVCPまでの直線
	TS_NEAR_1ST_VCP			//1stVCP付近
};
/* switch(now_ts)
	case a
	sys_leftWheelSpeed = param_WheelSpeed_TS_TO_1ST_VCP
*/

#if 0
enum targetspeed{
    TS_READY,
    TS_NOMAL_STRAIGHT,  //通常ストレート時の速度
    TS_LONG_STRAIGHT,   //長い直線の時の速度
    TS_SHORT_STRAIGHT,  //短い直線の時の速度
    TS_CORNER_STOP,     //コーナー時の停止
    TS_TURN,            //旋回時の速度
    TS_TURN_STOP,       //旋回時停止
    TS_LANEOUT_STOP,    //レーンアウト時の停止
    TS_RETURN_LANE,     //レーン復帰の後進速度
    TS_NEAR_CORNER,     //コーナー近く徐行
    TS_BEFORE_TURN      //コーナーを抜けた後のターンの為の調整徐行
    
};
#endif


/**
@brief
モーターの向きを規定するenumです
MD_UNNOWN,    ：未定義
MD_FORWORD,    ：前転
MD_BACK,       ：後転
MD_BRAKE,      ：電気ブレーキ
MD_INERTIA     ：惰性
初期値はMD_UNNOWNにすること。

@note
TBD モータードライバ次第で変更有りえます

*/
enum motorDirection
{
    MD_UNNOWN,
    MD_FORWORD,
    MD_BACK,
    MD_BRAKE,
    MD_INERTIA
};

