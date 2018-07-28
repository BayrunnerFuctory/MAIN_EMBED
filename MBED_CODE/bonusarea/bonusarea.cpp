#include "typedef.h"
#include "sysdata.h"
#include "parameter.h"
#include "bonusarea.h"

// ローカルでのみ使用
BOOL b_first_BonusAreaFind = FALSE; // 初めてボーナスエリアを見つけに行ったかのフラグ
BOOL b_first_PutBottle_flag = TRUE; // ペットボトルを初めて置く動作をしたかのフラグ
// extern用実体定義
BOOL b_HavingBottle_flag = TRUE; // ペットボトルを持っているフラグ

/**
@brief ボーナスエリア関数です。
@param 無し
@return FALSE:エラー無し
@note メイン関数から呼び出されます。
*/
BOOL BonusArea()
{
    // ボーナスエリアを探す
    //BonusArea_Find();

    // フロントのセンサが全て反応したら
    if(sys_frontLineSenser == FLS_BBBB){
        // ペットボトルを置く
        BonusArea_PutBottle();
    }
    // フロントのセンサが全て反応していなかったら
    else{
        // ボーナスエリアを再び探す
        BonusArea_Find();
    }

    return FALSE;
}

/**
@brief ボーナスエリアを探す関数です。
@param 無し
@return FALSE:エラー無し
@note ローカルでのみ使用。
*/
static BOOL BonusArea_Find()
{
    // 初めてボーナスエリアを探す場合
    //if(b_first_BonusAreaFind == TRUE){
        // 右に旋回する
        //BonusArea_Find_TurnRight();
        // 前進する
        //BonusArea_Find_GoForward();
        // 初回フラグをオフにする
        //b_first_BonusAreaFind = FALSE;
    //}
    // 2回目以降ボーナスエリアを探す場合
    //else{
        // ボーナスエリアを再び探す
        //BonusArea_ReFind();
    //}

    BonusArea_Find_TurnRight();
    return FALSE;
}

/**
@brief  右に旋回する関数です。
@param  無し
@return FALSE:エラー無し
@note   ローカルでのみ使用。
    ボーナスエリアを探す関数で呼び出されます。
*/
static BOOL BonusArea_Find_TurnRight()
{
    // 旋回角度を指定し要求値を指定する
    BonusArea_Find_TurnRight_Caluculate_Request();

    return FALSE;
}

/**
@brief  前進する関数です。
@param  無し
@return FALSE:エラー無し
@note   ローカルでのみ使用。
    ボーナスエリアを探す関数で呼び出されます。
*/
static BOOL BonusArea_Find_GoForward()
{
    // 前進距離を指定し要求値を指定する
    BonusArea_Find_GoForward_Caluculate_Request();

    return FALSE;
}

/**
@brief  旋回角度を指定し要求値を指定する関数です。
@param  無し
@return FALSE:エラー無し
@note   ローカルでのみ使用。
    右に旋回する関数で呼び出されます。
*/
static BOOL BonusArea_Find_TurnRight_Caluculate_Request()
{
    // 車体角度が16.7°になるまで旋回する
    // 16.7 = (vR - vL) / 2*200 となるように設定する
    //if(sys_angularDegree <= 16.7){
    //    sys_requestLeftMotorDuty = 200;
    //    sys_requestRightMotorDuty = 100;
    //}
    //else{
    //    sys_requestLeftMotorDuty = 0;
    //    sys_requestRightMotorDuty = 0;
    //}
    sys_requestLeftMotorDuty = 200;
    sys_requestRightMotorDuty = 0;
    return FALSE;
}

/**
@brief  前進距離を指定し要求値を指定する関数です。
@param  無し
@return FALSE:エラー無し
@note   ローカルでのみ使用。
    前進する関数で呼び出されます。
    
    回転した円弧分の直進距離(円弧と同等と近似)を、
    交差点からボーナスエリアの中心までの距離
    から引いた分直進させます。
*/
static BOOL BonusArea_Find_GoForward_Caluculate_Request()
{
    // 前進距離が313 - ρ*16.7になるまで前進する
    if(sys_tripmeter <= 300){
        sys_requestLeftMotorDuty = 150;
        sys_requestRightMotorDuty = 150;
    }
    else{
        sys_requestLeftMotorDuty = 0;
        sys_requestRightMotorDuty = 0;
    }

    return FALSE;
}

/**
@brief  ボーナスエリアを再び探す関数です。
@param  無し
@return FALSE:エラー無し
@note   ローカルでのみ使用。
    ボーナスエリアを探す関数で呼び出されます。
    実際のデバッグ時に必要に応じて実装。
*/
static BOOL BonusArea_ReFind()
{

    return FALSE;
}

/**
@brief  ペットボトルを置く関数です。
@param  無し
@return FALSE:エラー無し
@note   ローカルでのみ使用。
    ボーナスエリア関数で呼び出されます。
*/
static BOOL BonusArea_PutBottle()
{
    //// 初めてペットボトルを置く場合
    //if(b_first_PutBottle_flag == TRUE)
    //{
        //// ペットボトルを持っているフラグをOFF
    //    b_HavingBottle_flag = FALSE;
    //}
    //// 既にペットボトルを置いている場合
    //else
    //{
        //// 処理無し
    //}
    b_HavingBottle_flag = FALSE;
    sys_requestServoPulseWidth = 50;
    return FALSE;
}