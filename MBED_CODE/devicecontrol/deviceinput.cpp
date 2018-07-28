#include "mbed.h"
#include "typedef.h"
#include "sysdata.h"
#include "parameter.h"
#include "errorcontrol.h"
#include "deviceinput.h"
#include "deviceoutput.h"

#include "communication.h"

#include "main.h"
#include "errorcontrol.h"
#include <AnalogIn.h>


BOOL	DeviceInput(void);
BOOL	DeviceInputInit(void);
BOOL    IsArmLimit(int, int);

static BOOL GetPhotoSenserData(void);
//static void InterruptLeftEncorderCount(void);
//static void InterruptRightEncorderCount(void);
static BOOL GetSwitchData(void);
static BOOL GetModeSwitchData(void);
static void ChangeLeftEncorderCount(int32, int32); //20160531 追加
static void ChangeRightEncorderCount(int32, int32);//20160531 追加
//static void InterruptLeftEncorderRise(); //20160531 追加
//static void InterruptLeftEncorderFall(); //20160531 追加
//static void InterruptRightEncorderRise(); //20160531 追加
//static void InterruptRightEncorderFall(); //20160531 追加

//static void InterruptLeftBEncorderRise(); //20160531 追加
//static void InterruptLeftBEncorderFall(); //20160531 追加
//static void InterruptRightBEncorderRise(); //20160531 追加
//static void InterruptRightBEncorderFall(); //20160531 追加


///センサ処理するクラス

//static DigitalIn potosensInput_fl(p5);		///<前左 //2017
//static DigitalIn potosensInput_fml(p6);		///<前中左


//static DigitalIn potosensInput_fmr(p7);		///<前中右
//static DigitalIn potosensInput_fr(p8);		///<前右
//static DigitalIn potosensInput_rl(p9);		///<後右
//static DigitalIn potosensInput_rml(p10);	///<後中右
//static DigitalIn potosensInput_rmr(p11);	///<後中左
//static DigitalIn potosensInput_rr(p12);		///<後左

InterruptIn ussLeftEcho(p12);			///超音波センサLeftエコー受信
InterruptIn ussBackEcho(p13);;			///超音波センサBackエコー受信
InterruptIn ussRightEcho(p14);			///超音波センサRightエコー受信
/*
static DigitalIn potosensInput_fl(p8);		///<前左
static DigitalIn potosensInput_fml(p7);	///<前中左
static DigitalIn potosensInput_fmr(p6);	///<前中右
static DigitalIn potosensInput_fr(p5);		///<前右
static DigitalIn potosensInput_rl(p10);		///<後左
static DigitalIn potosensInput_rr(p9);		///<後右
*/
///フォトセンサのデータ。
static BOOL potoSenserData_fl[3];	///<前左
static BOOL potoSenserData_fml[3];	///<前中左
static BOOL potoSenserData_fmr[3];	///<前中右
static BOOL potoSenserData_fr[3];	///<前右
static BOOL potoSenserData_rl[3];	///<後左
static BOOL potoSenserData_rml[3];	///<後中左
static BOOL potoSenserData_rmr[3];	///<後中右
static BOOL potoSenserData_rr[3];	///<後右

static AnalogIn frontSenserLL(p15);
static AnalogIn frontSenserCL(p16);
static AnalogIn frontSenserCC(p17);
static AnalogIn frontSenserCR(p18);
static AnalogIn frontSenserRR(p19);


//ADコンバータ
static AnalogIn   batteryAnalogIn(p20);

///スタートプッシュスイッチとモードスイッチクラス
//static DigitalIn startPushBtn(p30);

//static DigitalIn goalCheckBtnL(p12);
//static DigitalIn goalCheckBtnR(p11);


///エンコーダ処理するクラス 割り込みだからstatic付けちゃダメよ
//InterruptIn		leftEncorderChannelA(p18);
//DigitalIn 		leftEncorderChannelB(p18);
//InterruptIn     leftEncorderChannelB(p17);
//InterruptIn		rightEncorderChannelA(p16);
//DigitalIn 		rightEncorderChannelB(p16);
//InterruptIn     rightEncorderChannelB(p15);

///フォトセンサ確定データ
flont_line_senser		sys_frontLineSenser;
rear_line_senser		sys_rearLineSenser;

///エンコーダのカウント値
uint16				sys_leftEncorderCount;
uint16				sys_rightEncorderCount;

///エンコーダの現在状態
int32               sys_leftEncorderState;
int32               sys_rightEncorderState;

///スタートスイッチのデータ
BOOL				sys_enableStartSwitch;
modeswitch			sys_modeSwitch;

BOOL 				sys_goalCheckBtnL;
BOOL                sys_goalCheckBtnR;

//Arm limit switch
static DigitalIn limitOpenSide(p29);
static DigitalIn limitCloseSide(p30);

//モーターバッテリー電圧
float32 sys_batteryVoltage;	//単位V

//超音波センサの受信パルス幅
int32 sys_ussBackEchoWidth = 0;
int32 sys_ussLeftEchoWidth = 0;
int32 sys_ussRightEchoWidth = 0;

//超音波センサのタイマー
Timer ussBackTimer;
Timer ussLeftTimer;
Timer ussRightTimer;
static void InterruptUSSBackRise();
static void InterruptUSSBackFall();
static void InterruptUSSLeftRise();
static void InterruptUSSLeftFall();
static void InterruptUSSRightRise();
static void InterruptUSSRightFall();

float32 sys_frontSenserLL;
float32 sys_frontSenserCL;
float32 sys_frontSenserCC;
float32 sys_frontSenserCR;
float32 sys_frontSenserRR;
BOOL GetPhotoSenerData_Analog(void);



/**
@brief
インプットポート各種の初期設定
デジタルインプットポートのプルアップ設定などを行います。

@return FALSE:エラー無し TRUE:何らかのエラー
@note
NXP LPC176x User Manualを参照しましたが
全てのDIポートがリセット時に内部プルアップされるようです。
プルアップが必要ないポートについては逆に明示的にプルアップを止める必要が
あるみたいですね。 

*/

BOOL GetUltraSonicInputData(void); //UltraSonic

BOOL DeviceInputInit()
{
	uint16 battery;

	//potosensInput_fl.mode(PullUp);  //2017
	//potosensInput_fml.mode(PullUp);
//	potosensInput_fmr.mode(PullUp);
//	potosensInput_fr.mode(PullUp);
//	potosensInput_rl.mode(PullUp);
//	potosensInput_rr.mode(PullUp);
//	startPushBtn.mode(PullUp);


	//エンコーダA相の割り込み関数の登録 20160531 変更
//	leftEncorderChannelA.rise(*InterruptLeftEncorderCount);
//	rightEncorderChannelA.rise(*InterruptRightEncorderCount);

//	leftEncorderChannelA.rise(*InterruptLeftEncorderRise);
//	leftEncorderChannelA.fall(*InterruptLeftEncorderFall);
//	rightEncorderChannelA.rise(*InterruptRightEncorderRise);
//	rightEncorderChannelA.fall(*InterruptRightEncorderFall);
//	leftEncorderChannelB.rise(*InterruptLeftBEncorderRise);
//	leftEncorderChannelB.fall(*InterruptLeftBEncorderFall);
//	rightEncorderChannelB.rise(*InterruptRightBEncorderRise);
//	rightEncorderChannelB.fall(*InterruptRightBEncorderFall);

	sys_frontLineSenser		= FLS_UNKNOWN;
	sys_rearLineSenser		= RLS_UNKNOWN;
	sys_leftEncorderCount	= 0;
	sys_rightEncorderCount	= 0;
	sys_enableStartSwitch	= 0;
	sys_modeSwitch			= MS_0;

	//バッテリー電圧の取得（ハード的に３分圧してるので３倍する）
	battery = batteryAnalogIn.read_u16() >> 4;
	sys_batteryVoltage = (battery * 3300 / 4095) * 3;
	
	
	ussBackEcho.rise(*InterruptUSSBackRise);
	ussBackEcho.fall(*InterruptUSSBackFall);
	ussBackEcho.mode(PullDown);
	//ussLeftEcho.rise(*InterruptUSSLeftRise);
	//ussLeftEcho.fall(*InterruptUSSLeftFall);
	//ussLeftEcho.mode(PullDown);
	//ussRightEcho.rise(*InterruptUSSRightRise);
	//ussRightEcho.fall(*InterruptUSSRightFall);
	//ussRightEcho.mode(PullDown);
	
	sys_frontSenserLL = 0;
	sys_frontSenserCL = 0;
	sys_frontSenserCC = 0;
	sys_frontSenserCR = 0;
	sys_frontSenserRR = 0;
	
	limitCloseSide.mode(PullDown);
	limitOpenSide.mode(PullDown);
	
	return FALSE;
}


/**
@brief
デバイス情報取得処理のメイン関数です。
マイコンボードに接続されている各種デバイスから情報を取得します。
(シリアル通信は別モジュールです。)

@return FALSE:エラー無し TRUE:何らかのエラー
@note
DeviceInputから呼ばれて、取得するデータは必ずdeviceinput.hのexternで参照できるようにしてください。
ここではデータの取得をメインに処理してください（やっても整列、平滑化程度）
計算や単位変換等の処理はdestinationControlのモジュールで行ってください
理由：destinationControlの前にデバッグ用オーバーライドの処理があるため、計算値ではオーバーライド処理が大変になる。
*/
BOOL DeviceInput()
{
	BOOL onErr = FALSE;

	//フォトセンサの情報を取得
	onErr |= GetPhotoSenerData_Analog();
	//スイッチ情報の取得
	onErr |= GetSwitchData();
	//モードスイッチの情報を取得
	onErr |= GetModeSwitchData();
	//GetUltraSonicInput
	onErr |= GetUltraSonicInputData();
	
	return onErr;
}


static BOOL GetPhotoSenerData_Analog()
{
	sys_frontSenserLL = frontSenserLL.read();
	sys_frontSenserCL = frontSenserCL.read();
	sys_frontSenserCC = frontSenserCC.read();
	sys_frontSenserCR = frontSenserCR.read();
	sys_frontSenserRR = frontSenserRR.read();
	return FALSE;
}

/**
@brief
ラインの白黒を判断するフォトセンサのデータを取得します。

@note
センサーの値は、3回取って多い方を信じるようにします

@return FALSE:エラー無し TRUE:何らかのエラー
*/
static BOOL GetPhotoSenserData()
{
	uint8 fsens = 0;
	uint8 rsens = 0;
	BOOL ret = FALSE;
	uint8 cnt;
	uint8 chk;

	//タスクカウンタを3で割った余りを求める
	cnt = sys_taskCount & 0x03;

	if(cnt == 0)
	{
		//potoSenserData_fl[0]	= potosensInput_fl; //2017
		//potoSenserData_fml[0]	= potosensInput_fml;
//		potoSenserData_fmr[0]	= potosensInput_fmr;
//		potoSenserData_fr[0]	= potosensInput_fr;
//		potoSenserData_rl[0]	= potosensInput_rl;
//		potoSenserData_rml[0]	= potosensInput_rml;
//		potoSenserData_rmr[0]	= potosensInput_rmr;
//		potoSenserData_rr[0]	= potosensInput_rr;
    }
	else if(cnt == 1)
	{
		//potoSenserData_fl[1]	= potosensInput_fl; //2017
		//potoSenserData_fml[1]	= potosensInput_fml;
//		potoSenserData_fmr[1]	= potosensInput_fmr;
//		potoSenserData_fr[1]	= potosensInput_fr;
//		potoSenserData_rl[1]	= potosensInput_rl;
//		potoSenserData_rml[1]	= potosensInput_rml;
//		potoSenserData_rmr[1]	= potosensInput_rmr;
//		potoSenserData_rr[1]	= potosensInput_rr;
    }
	else if(cnt == 2)
	{
		//potoSenserData_fl[2]	= potosensInput_fl; //2017
		//potoSenserData_fml[2]	= potosensInput_fml;
//		potoSenserData_fmr[2]	= potosensInput_fmr;
//		potoSenserData_fr[2]	= potosensInput_fr;
//		potoSenserData_rl[2]	= potosensInput_rl;
//		potoSenserData_rml[2]	= potosensInput_rml;
//		potoSenserData_rmr[2]	= potosensInput_rmr;
//		potoSenserData_rr[2]	= potosensInput_rr;
	}

	//フロントセンサー判断
	chk = potoSenserData_fl[0] + potoSenserData_fl[1] + potoSenserData_fl[2];
	if (chk >= 2)
	{
		fsens += 0x08;
	}

	chk = potoSenserData_fml[0] + potoSenserData_fml[1] + potoSenserData_fml[2];
	if (chk >= 2)
	{
		fsens += 0x04;
	}

	chk = potoSenserData_fmr[0] + potoSenserData_fmr[1] + potoSenserData_fmr[2];
	if (chk >= 2)
	{
		fsens += 0x02;
	}

	chk = potoSenserData_fr[0] + potoSenserData_fr[1] + potoSenserData_fr[2];
	if (chk >= 2)
	{
		fsens += 0x01;
	}

	//リアセンサー判断
	chk = potoSenserData_rl[0] + potoSenserData_rl[1] + potoSenserData_rl[2];
	if (chk >= 2)
	{
		rsens += 0x08;
	}

	chk = potoSenserData_rml[0] + potoSenserData_rml[1] + potoSenserData_rml[2];
	if (chk >= 2)
	{
		rsens += 0x04;
	}
	chk = potoSenserData_rmr[0] + potoSenserData_rmr[1] + potoSenserData_rmr[2];
	if (chk >= 2)
	{
		rsens += 0x02;
	}

	chk = potoSenserData_rr[0] + potoSenserData_rr[1] + potoSenserData_rr[2];
	if (chk >= 2)
	{
		rsens += 0x01;
	}

	//車体前センサー値確定
	sys_frontLineSenser = (flont_line_senser)fsens;
	if(sys_frontLineSenser > 0x0F)
	{
		ERR_WriteID(EID_RANGE_OVER, __func__, __LINE__);
		ret = TRUE;
	}

	//車体後センサー値確定
	sys_rearLineSenser = (rear_line_senser)rsens;
	if(sys_rearLineSenser > 0x0F)
	{
		ERR_WriteID(EID_RANGE_OVER, __func__, __LINE__);
		ret = TRUE;
	}

	return ret;
}

/**
@brief
メカ動作を開始するスタートスイッチのデータを取得します。
@return FALSE:エラー無し TRUE:何らかのエラー
*/
static BOOL GetSwitchData()
{
	//スイッチ押すと、グランドに落ちるエレキ仕様なので、０でスイッチオン判断
//	if (startPushBtn.read() == 0)
//	{
//		sys_enableStartSwitch = TRUE;
//	}
//	else
//	{
//		sys_enableStartSwitch = FALSE;
//	}
	
//	if(goalCheckBtnL.read() == 0)	//押されて無いとH
//	{
//		sys_goalCheckBtnL = 1;
//	}else{
//		sys_goalCheckBtnL = 0;
//	}
	
//	if(goalCheckBtnR.read() == 0)	//押されて無いとH
//	{
//		sys_goalCheckBtnR = 1;
//	}else{
//		sys_goalCheckBtnR = 0;
//	}


	//.printf("sw = %d \n",startPushBtn.read());
	return FALSE;
}

static BOOL GetUltraSonicInputData()
{}

/**
@brief
	動作モードを決定するモードスイッチの値を取得します
@return FALSE:エラー無し TRUE:何らかのエラー
*/
static BOOL GetModeSwitchData()
{
	uint8 sw = 0;
	BOOL ret = FALSE;

	switch (sw)
	{
		case 0:
			sys_modeSwitch = MS_0;
			break;
		case 1:
			sys_modeSwitch = MS_1;
			break;
		case 2:
			sys_modeSwitch = MS_2;
			break;
		case 3:
			sys_modeSwitch = MS_3;
			break;
		default:
			ERR_WriteID(EID_UNKNOWN_TYPE, __func__, __LINE__);
			ret = TRUE;
			break;
	}

	return ret;
}

/**
@brief
左エンコーダカウントデータの取得 （割り込み関数）
@note
エンコーダのA相がHになった場合、この関数がコールされます。
B相がHの場合前進、Lの場合後退となります。
割り込みでカウントアップ・ダウンする為、DeviceInput()でコール不要です。
*/
/*
static void InterruptLeftEncorderCount()
{
	int32 cb = 0;

	cb = leftEncorderChannelB.read();

    if(cb == 1){
        sys_leftEncorderCount++;
    }
    else{
    	sys_leftEncorderCount--;
    }
}
*/

/**
@brief
右エンコーダカウントデータの取得 （割り込み関数）
@return
@note
エンコーダのA相がHになった場合、この関数がコールされます。
B相がHの場合前進、Lの場合後退となります。
割り込みでカウントアップ・ダウンする為、DeviceInput()でコール不要です。
*/
/*
static void InterruptRightEncorderCount()
{
	int32 cb = 0;

	cb = rightEncorderChannelB.read();

// k_sato 20151017 ハード問題で暫定で+と-を入れ替えました 
    if(cb == 1){
        sys_rightEncorderCount--;
    }
    else{
    	sys_rightEncorderCount++;
    }
}
*/
/* 20160531 追記 */

/**
@brief
左エンコーダ立ち上がり割り込み
@return
@note
左エンコーダのA相がHになった時にこの関数をコールする
過去状態に現在状態を代入して
B相の状態に応じて現在状態を更新する
A相 B相 状態
 H  L   1
 H  H   2
 現在状態と過去状態をカウント変更関数に渡して、カウント変更を行う
*/
static void InterruptLeftEncorderRise()
{
	int32 nowState = 0;
	int32 cb = 0;//leftEncorderChannelB.read();
	
	if (cb == 0)
	{
		nowState = 1;
	}
	else
	{
		nowState = 2;
	}
	
	ChangeLeftEncorderCount(sys_leftEncorderState, nowState);
	
	sys_leftEncorderState = nowState;
}

/**
@brief
左エンコーダ立ち下がり割り込み
@return
@note
左エンコーダのA相がLになった時にこの関数をコールする
過去状態に現在状態を代入して
B相の状態に応じて現在状態を更新する
A相 B相 状態
 L  L   0
 L  H   3
*/
static void InterruptLeftEncorderFall()
{
	int32 nowState = 0;
	int32 cb = 0;//leftEncorderChannelB.read();
	
	if (cb == 0)
	{
		nowState = 0;
	}
	else
	{
		nowState = 3;
	}
	
	ChangeLeftEncorderCount(sys_leftEncorderState, nowState);
	sys_leftEncorderState = nowState;
}
/**
@brief
右エンコーダ立ち上がり割り込み
@return
@note
右エンコーダのA相がHになった時にこの関数をコールする
過去状態に現在状態を代入して
B相の状態に応じて現在状態を更新する
A相 B相 状態
 H  L   1
 H  H   2
 現在状態と過去状態をカウント変更関数に渡して、カウント変更を行う
*/
static void InterruptRightEncorderRise()
{
	int32 nowState = 0;
	int32 cb = 0;//rightEncorderChannelB.read();
	
	if (cb == 0)
	{
		nowState = 1;
	}
	else
	{
		nowState = 2;
	}
	
	ChangeRightEncorderCount(sys_rightEncorderState, nowState);
	
	sys_rightEncorderState = nowState;
}

/**
@brief
右エンコーダ立ち下がり割り込み
@return
@note
右エンコーダのA相がLになった時にこの関数をコールする
過去状態に現在状態を代入して
B相の状態に応じて現在状態を更新する
A相 B相 状態
 L  L   0
 L  H   3
*/
static void InterruptRightEncorderFall()
{
	int32 nowState = 0;
	int32 cb = 0;//rightEncorderChannelB.read();
	
	if (cb == 0)
	{
		nowState = 0;
	}
	else
	{
		nowState = 3;
	}
	
	ChangeRightEncorderCount(sys_rightEncorderState, nowState);
	sys_rightEncorderState = nowState;
}

/**
@brief
左エンコーダのカウント変更関数
@return
@note
過去状態と現在状態から左エンコーダのカウントを変更する
A相 B相 状態
 L  L   0
 H  L   1  ↑ 逆転
 H  H   2  ↓ 正転
 L  H   3
加算）現在状態 - 過去状態 = -1 or  3
減算）現在状態 - 過去状態 =  1 or -3

TODO エンコーダの向きが右と逆なのに同じ理論なので、加算減算を逆にする必要あり
->20160716 右エンコーダのカウントを逆転
*/
static void ChangeLeftEncorderCount(int32 beforeState, int32 nowState)
{
	//int32 diff = nowState - beforeState;
	if(beforeState == 0 && nowState == 1)
	{
		sys_leftEncorderCount--;
	}
	else if(beforeState == 3 && nowState == 2)
	{
		sys_leftEncorderCount++;
	}
}

/**
@brief
右エンコーダのカウントアップ/ダウン関数
@return
@note
過去状態と現在状態から右エンコーダのカウントを変更する
A相 B相 状態
 L  L   0
 H  L   1 ↑ 逆転
 H  H   2 ↓ 正転
 L  H   3
加算）現在状態 - 過去状態 = -1 or  3
減算）現在状態 - 過去状態 =  1 or -3

TODO エンコーダの向きが左と逆なのに同じ理論なので、加算減算を逆にする必要あり
-> 20160716 逆転させた。動作確認でむきに問題ないかを確認すること
*/
static void ChangeRightEncorderCount(int32 beforeState, int32 nowState)
{
	if(beforeState == 0 && nowState == 1)
	{
		sys_rightEncorderCount++;
	}
	else if(beforeState == 3 && nowState == 2)
	{
		sys_rightEncorderCount--;
	}
}

/* 20160531 追記 終*/

/* 20160716 追記 */
/**
@brief
左エンコーダ立ち上がり割り込み B
@return
@note
左エンコーダのB相がHになった時にこの関数をコールする
過去状態に現在状態を代入して
A相の状態に応じて現在状態を更新する
A相 B相 状態
 L  H   3
 H  H   2
 現在状態と過去状態をカウント変更関数に渡して、カウント変更を行う
*/
static void InterruptLeftBEncorderRise()
{
	int32 nowState = 0;
	int32 ca = 0;//leftEncorderChannelA.read();
	
	if (ca == 0)
	{
		nowState = 3;
	}
	else
	{
		nowState = 2;
	}
	
	ChangeLeftEncorderCount(sys_leftEncorderState, nowState);
	
	sys_leftEncorderState = nowState;
}

/**
@brief
左エンコーダ立ち下がり割り込み B
@return
@note
左エンコーダのB相がLになった時にこの関数をコールする
過去状態に現在状態を代入して
A相の状態に応じて現在状態を更新する
A相 B相 状態
 L  L   0
 H  L   1
*/
static void InterruptLeftBEncorderFall()
{
	int32 nowState = 0;
	int32 ca = 0;//leftEncorderChannelA.read();
	
	if (ca == 0)
	{
		nowState = 0;
	}
	else
	{
		nowState = 1;
	}
	
	ChangeLeftEncorderCount(sys_leftEncorderState, nowState);
	sys_leftEncorderState = nowState;
}
/**
@brief
右エンコーダ立ち上がり割り込み B
@return
@note
右エンコーダのB相がHになった時にこの関数をコールする
過去状態に現在状態を代入して
A相の状態に応じて現在状態を更新する
A相 B相 状態
 L  H   3
 H  H   2
 現在状態と過去状態をカウント変更関数に渡して、カウント変更を行う
*/
static void InterruptRightBEncorderRise()
{
	int32 nowState = 0;
	int32 cb = 0;//rightEncorderChannelA.read();
	
	if (cb == 0)
	{
		nowState = 3;
	}
	else
	{
		nowState = 2;
	}
	
	ChangeRightEncorderCount(sys_rightEncorderState, nowState);
	
	sys_rightEncorderState = nowState;
}

/**
@brief
右エンコーダ立ち下がり割り込み B
@return
@note
右エンコーダのB相がLになった時にこの関数をコールする
過去状態に現在状態を代入して
A相の状態に応じて現在状態を更新する
A相 B相 状態
 L  L   0
 H  L   1
*/
static void InterruptRightBEncorderFall()
{
	int32 nowState = 0;
	int32 cb = 0;//rightEncorderChannelA.read();
	
	if (cb == 0)
	{
		nowState = 0;
	}
	else
	{
		nowState = 1;
	}
	
	ChangeRightEncorderCount(sys_rightEncorderState, nowState);
	sys_rightEncorderState = nowState;
}

/**
@brief
後ろ超音波センサ立ち上がり割り込み
@return
@note
超音波センサ立上がりから立下りまでのパルス幅取得
*/
static void InterruptUSSBackRise()
{
	//sys_ussBackEchoWidth = 0;
		sys_debugCount++;
	ussBackTimer.start();	
}

/**
@brief
後ろ超音波センサ立ち下がり割り込み
@return
@note
超音波センサ立上がりから立下りまでのパルス幅取得
受信パルス幅[us] / 58 = 距離[cm]
*/
static void InterruptUSSBackFall()
{
	ussBackTimer.stop();
	sys_ussBackEchoWidth = ussBackTimer.read_us();
	ussBackTimer.reset();
}

/**
@brief
左超音波センサ立ち上がり割り込み
@return
@note
超音波センサ立上がりから立下りまでのパルス幅取得
*/
static void InterruptUSSLeftRise()
{
	//sys_ussBackEchoWidth = 0;
		sys_debugCount++;
	ussLeftTimer.start();	
}

/**
@brief
左超音波センサ立ち下がり割り込み
@return
@note
超音波センサ立上がりから立下りまでのパルス幅取得
受信パルス幅[us] / 58 = 距離[cm]
*/
static void InterruptUSSLeftFall()
{
	ussLeftTimer.stop();
	sys_ussLeftEchoWidth = ussLeftTimer.read_us();
	ussLeftTimer.reset();
}
/**
@brief
右超音波センサ立ち上がり割り込み
@return
@note
超音波センサ立上がりから立下りまでのパルス幅取得
*/
static void InterruptUSSRightRise()
{
	//sys_ussBackEchoWidth = 0;
	sys_debugCount++;
	ussRightTimer.start();	
}

/**
@brief
右超音波センサ立ち下がり割り込み
@return
@note
超音波センサ立上がりから立下りまでのパルス幅取得
受信パルス幅[us] / 58 = 距離[cm]
*/
static void InterruptUSSRightFall()
{
	ussRightTimer.stop();
	sys_ussRightEchoWidth = ussRightTimer.read_us();
	ussRightTimer.reset();
}
/*  */

BOOL IsArmLimit(int leftSig, int rightSig)
{
	BOOL isCloseArm = FALSE;
	BOOL isOpenArm = FALSE;
	
	if(leftSig == 1 && rightSig == 0)
	{
		isCloseArm = TRUE;
	}
	
	if(leftSig == 0 && rightSig == 1)
	{
		isOpenArm = TRUE;
	}
	
	if(isCloseArm && limitCloseSide == 0)
	{
		return TRUE;
	}
	
	if(isOpenArm &&  limitOpenSide == 0)
	{
		return TRUE;
	}
	
	return FALSE;
}