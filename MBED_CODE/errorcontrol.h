#pragma once
#include "typedef.h"



enum errorID
{
	EID_NO_ERROR 							= 0x00000000,	//エラー無

	//プログラム処理エラー系
	EID_UNKNOWN_TYPE 						= 0x00000001,	//想定していないelseとかswitchとか
	EID_NON_PARAMETER 						= 0x00000002,	//パラメータが入力されていない
	EID_RANGE_OVER 							= 0x00000004,	//想定していない入力値

	//ハードウェアエラー系
	EID_ENC_COUNT_IMPROBABLE				= 0x00000010,	//エンコーダのカウント値がおかしい

	//制御系エラー
	EID_LINE_SENS_IMPROBABLE_ON_LEFT_LANE	= 0x00000100,	//ライン取りで想定外の事が起こった。（左車線）
	EID_LINE_SENS_IMPROBABLE_ON_RIGHT_LANE	= 0x00000200,	//ライン取りで想定外の事が起こった。（右車線）
	EID_LINE_SENS_IMPROBABLE_ON_LEFT_TURN	= 0x00000300,	//ターン中に想定外の事が起こった。
	EID_LINE_SENS_IMPROBABLE_ON_RIGHT_TURN	= 0x00000400,	//ターン中に想定外の事が起こった。
	EID_LINE_AJUST_TIMEOVER					= 0x00000500,	//ラインの白黒が切り替わるまでに時間がかかりすぎた。
	EID_LINE_SENS_LINELOST					= 0x00000600,	//ラインを見失った（デバッグ用、本番ではラインロストでも動作します）
	EID_IMPROBABLE_LR_ADJUST_COUNT			= 0x00000700,	//左右アジャスト処理数値が異常
	EID_OVER_STREE							= 0x00000800,	//必要操舵量が想定より大きすぎる
	EID_RETURN_ROUTE_OVER_TIME				= 0x00000900,	//時間内に想定した座標に戻れなかった
	EID_LINE_SENS_IMPROBABLE                = 0x00000a00,	//ライン取りで想定外の事が起こった

	//データ計算計エラー
	EID_CALCULATE_WHEEL_SPEED				= 0x00001000,	//車輪速度が異常数値
	EID_CALCULATE_MACHINE_SPEED				= 0x00002000,	//車体速度が異常数値

	//コース取り失敗エラー
	EID_FAIL_TO_ARRIVE_VCP1					= 0x00010000,	//VCP1に到着失敗
	EID_FAIL_TO_ARRIVE_VCP2					= 0x00020000,
	EID_FAIL_TO_ARRIVE_VCP3					= 0x00030000,
	EID_FAIL_TO_ARRIVE_VCP4					= 0x00040000,
	EID_FAIL_TO_ARRIVE_VCP5					= 0x00050000,
	EID_FAIL_TO_ARRIVE_VCP6					= 0x00060000,
	EID_FAIL_TO_ARRIVE_VCP7					= 0x00070000,
	EID_FAIL_TO_ARRIVE_VCP8					= 0x00080000,
	EID_FAIL_TO_ARRIVE_VCP9					= 0x00090000
};

extern BOOL errorIndicator[4];
extern void ERR_WriteID(errorID eid, const uint8* functionName,int32 line);

