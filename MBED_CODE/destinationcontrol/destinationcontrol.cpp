#include "mbed.h"
#include "typedef.h"
#include "sysdata.h"
#include "parameter.h"
#include "errorcontrol.h"
#include "destinationcontrol.h"
#include "debugsupport.h"

#include "judgestate.h"
#include "deviceinput.h"
#include "caluculatesteer.h"
#include "caluculatetargetspeed.h"
#include "stdlib.h"
#include "math.h"

//debug
#include "communication.h"

void DistinationControlInit(void);
BOOL DistinationControl(void);

static BOOL UpdateMachineData(void);
static BOOL UpdateEncorderLogData(void);
static BOOL CaluclateMashineLRSpeed(void);

static BOOL CaluclateDistance(void);
static BOOL CaluculateDeltaCount(uint16 nowCount, uint16 lastCount, int32* retDeltaCount);
static BOOL CaluclateMashineSpeed(void);
static BOOL CaluclateMashineAngularvelocity(void);
static BOOL CaluclateMashineAcceraretion(void);
static BOOL CaluclateAbsoluteXYAxis(void);
static BOOL CaluclateRelativeXYAxis(void);



int16 sys_machineSpeed 						= 0;	///<現在車速     v  単位:mm/sec
int16 sys_leftWheelSpeed 					= 0;	///<左車輪速度 vL 単位:mm/sec
int16 sys_rightWheelSpeed 					= 0;	///<右車輪速度 vR 単位:mm/sec
float64 sys_angularVelocity					= 0;	///<車体角速度 ω 単位:rad/sec
float64 sys_angularRadian					= 0;	///<車体角度 単位:rad
float64 sys_angularDegree					= 0;	///<車体角度 単位:度 rad*57.2958
int16 sys_acceleration						= 0;	///<車体加速度 単位:mm/sec^2
int32 sys_odoCount							= 0;	///<累積左右合算エンコーダカウント
int32 sys_odoMeter							= 0;	///<累積走行距離 単位:mm
int32 sys_tripCount							= 0;	///<区間左右合算エンコーダカウント
int32 sys_tripmeter							= 0;	///<区間走行距離 単位:mm
int32 sys_absoluteAxis_X					= 0;	///<絶対座標X 単位：um
int32 sys_absoluteAxis_Y					= 0;	///<絶対座標Y 単位：um
int32 sys_relativeAxis_X					= 0;	///<相対座標X 単位：um
int32 sys_relativeAxis_Y					= 0;	///<相対座標Y 単位：um

//uint16 param_encorderGauge					= 200;	///<エンコーダとエンコーダの間隔 単位：mm
uint16 param_encorderGauge					= 190;	///<エンコーダとエンコーダの間隔 単位：mm
int16  param_encCountErrorThreshold 		= 3000;	///<エンコーダの差分値がこの値より大きい場合は有りえない値なのでエラーとします。 単位：count
uint16 param_encorderWheelDiameter			= 40;	///<エンコーダに付いているホイールの直径 単位:mm
uint16 param_encorderResolution				= 100;	///<エンコーダの分解能
int16  param_leftEncorderAdjust				= 0;	///<エンコーダーの直進補正値_左
int16  param_rightEncorderAdjust			= 0;	///<エンコーダーの直進補正値_右


//速度系エラーリミット
uint16 param_errorLimitOfWheelSpeed			= 5000; ///<この値以上の速度が出ていたらエラーとする ex) 2000 = 2m/sec以上の速度は異常
uint16 param_errorLimitOfAngularVelocity	=
(param_errorLimitOfWheelSpeed/param_encorderGauge); ///<この値以上の車体角速度が出ていたらエラーとする 単位ラジアン/Sec
uint16 param_errorLimitOfAcceleration		= 1000;	///<この値以上の速度が出ていたらエラーとする ex) 1000 = 1m/sec^2以上の速度は異常 。一秒間で1m/s加速とか化けもんだろ

static uint32 encorder1CountDistination_um;			///<エンコーダ1カウントあたりの距離 単位:um
static uint16 logLeftEncorderCount[21];				///<左エンコーダカウント過去20回分（100ms）
static uint16 logRightEncorderCount[21];			///<右エンコーダカウント過去20回分（100ms）
static int16 machineSpeed[21];						///<車体中心部速度過去20回分（100ms）


void DistinationControlInit()
{
	uint8 i;
	
	sys_machineSpeed = 0;
	sys_leftWheelSpeed = 0;
	sys_rightWheelSpeed = 0;
	encorder1CountDistination_um = (uint32)((float64)PI * (float64)param_encorderWheelDiameter / (float64)param_encorderResolution * (float64)1000);

	for(i = 0; i < 20 ; i++){
    	logLeftEncorderCount[i] = 0;
    	logRightEncorderCount[i] = 0;
    	machineSpeed[i] = 0;
    }
    
    //sys_absoluteAxis_X = 500000; 
    //sys_absoluteAxis_Y = 1080000;//1090000;
    
	JudgeStateInit();
}


/**
@brief
メカ状態をアップデートし
目的地に合わせ目標速度、目標操舵を確定します。

@note
この関数での目標速度、目標操舵では
実際の値を確定しません。
「こんぐらいの値にしたいな～」までです。
ex）チェックポイントⅠに向かってるので、チェックポイント１の速度をセット
  チェックポイント1への直線から少し左にずれているので、ちょっとだけ右に旋回をセット
までです。

直接の値は次の calculate motor targetで確定します。
*/
BOOL DistinationControl()
{
	BOOL err = FALSE;

	//メカ情報を取得、計算を行う
	err |= UpdateMachineData();
	//ステートを判断、決定する
	err |= JudgeState();
	//現在の座標から操舵量を決める
	err |= CaluculateSteer();
	//現在のステートと操舵量から目標速度を決める
	err |= CaluculateTargetSpeed();

	return err;
}


/**
@brief
メカの状態を算出します。
速度
加速度、減速度
スタートからの移動距離、
車体向き（ヨー）
回転半径
座標
仮想チェックポイントからの移動距離

@note
公式
左右車輪角速度をLΦ、RΦとした場合車輪半径をｒ左右速度vL,vRとした場合
vL=rΦL
vR=rΦR
車体の旋回角速度をω車体エンコーダ間の中心の速度をｖ旋回半径をρとすると
v=ρω
左右エンコーダ間の距離を2dとすると、車体の旋回半径はｄだけ増減するので
vL=(ρ-d)ω
vR=(ρ+d)ω
この式を解いて
ω=(vR-vL)/2d
v=(vR+vL)/2
ρ=d(vR+vL)/(vR-vL)
このデータをもとに車体を座標系で見たとき
X = vCOSθ
Y = vSINθ
θ=ω

@return F:エラー無し T:何らかのエラー有
*/
static BOOL UpdateMachineData()
{
	BOOL err = FALSE;

	//エンコーダカウンタデータログをアップデート
	err |= UpdateEncorderLogData();
	//左右速度のｖL,vR算出
	err |= CaluclateMashineLRSpeed();
	//車体中心部の速度ｖの算出
	err |= CaluclateMashineSpeed();
	//車体角速度ωの算出
	err |= CaluclateMashineAngularvelocity();
	//加速度算出
	err |= CaluclateMashineAcceraretion();
	//移動距離算出(odo,trip)
	err |= CaluclateDistance();
	//現在のスタート地点からのXY座標
	err |= CaluclateAbsoluteXYAxis();
	//現在のチェックポイントからの相対XY座標
	err |= CaluclateRelativeXYAxis();

	return err;
}



/**
@brief
エンコーダカウンタのデータログをアップデートします。

@note
エンコーダのデータは速度平均、加速度平均、旋回速度等の計算に
利用するため、一定回数以上のログとして残します。
最新をsys_変数として公開し、ログ自体は計算値として使うためsys
としては利用しません。
モーターへの出力が20ms単位の為、最低で4回はログが欲しい。

@attention
エンコーダ補正計算をここに入れます。

*/
static BOOL UpdateEncorderLogData()
{
	uint8 i;
	
	//キャリブレーションモードの時は補正しません。
	if(sys_nowRouteState != RS_CALIBRATION)
	{
		sys_leftEncorderCount	+= param_leftEncorderAdjust;
		sys_rightEncorderCount	+= param_rightEncorderAdjust;
	}

    for(i = 20 ; i > 0; i--)
    {
    	logLeftEncorderCount[i] = logLeftEncorderCount[i-1];
    	logRightEncorderCount[i] = logRightEncorderCount[i-1];
    }
	logLeftEncorderCount[0]		= sys_leftEncorderCount;
	logRightEncorderCount[0]	= sys_rightEncorderCount;
	
    return FALSE;
}


/**
@brief
現在の車体左右の速度を算出します
単位はmm/s




@note
車速は20ms（4ループ）の差分の計算と
車速は100ms（20ループ）の差分の計算と
2つの時間で車速のを取り、その2つの平均で判定をします。

コレは低速時の場合、4ループでの差分ではエンコードカウンタが上がらず速度を計算できなくなるのを
防ぐためです。
しかし、評価時間を大きくすると、逆に停止時からの急加速時に速度が計算できなくなります。
その為、2つの評価速度の平均を取り、現在速度判断とします。


@return F:エラー無し T:計算エラー
*/
static BOOL CaluclateMashineLRSpeed()
{

	BOOL ret = FALSE;
	int32 deltaL = 0, deltaR = 0;
	int32 deltaL100ms = 0, deltaR100ms = 0;
	int32 umL,umR;
	int32 umL100ms,umR100ms;
	int32 tmpSpL,tmpSpR,tmpSp100L,tmpSp100R;

	//左右エンコーダが20msでどれだけ進んだかを算出  --> 0:現在 1:5ms前 ・・・ 4:20ms前
	ret |= CaluculateDeltaCount(logLeftEncorderCount[0],  logLeftEncorderCount[4],  &deltaL);
	ret |= CaluculateDeltaCount(logRightEncorderCount[0], logRightEncorderCount[4], &deltaR);
    
    //左右エンコーダが100msでどれだけ進んだかを算出
	ret |= CaluculateDeltaCount(logLeftEncorderCount[0],  logLeftEncorderCount[20],  &deltaL100ms);
	ret |= CaluculateDeltaCount(logRightEncorderCount[0], logRightEncorderCount[20], &deltaR100ms);
    
	//エンコードカウント x エンコーダ1カウントあたりの距離
	umL = deltaL * encorder1CountDistination_um;
	umR = deltaR * encorder1CountDistination_um;

	umL100ms = deltaL100ms * encorder1CountDistination_um;
	umR100ms = deltaR100ms * encorder1CountDistination_um;

	//一秒あたり何mm進んだかの計算 um/1000 / (0.02sec) --> (um / 1000) * 50 -->  um / 20
	tmpSpL = (int16)(umL / 20);
	tmpSpR = (int16)(umR / 20);
	
	//一秒あたり何mm進んだかの計算 um/1000 / (0.1sec) --> (um / 1000) * 10 -->  um / 100
	tmpSp100L = (int16)(umL100ms / 100);
	tmpSp100R = (int16)(umR100ms / 100);


	sys_leftWheelSpeed = (tmpSpL + tmpSp100L) / 2;
	sys_rightWheelSpeed = (tmpSpR + tmpSp100R) / 2;

#ifndef _DEBUG
	//計算エラーチェック
	if(   (sys_leftWheelSpeed  > param_errorLimitOfWheelSpeed)
	   || (sys_rightWheelSpeed > param_errorLimitOfWheelSpeed))
	{
		ERR_WriteID(EID_CALCULATE_WHEEL_SPEED, __func__, __LINE__);
		ret = TRUE;
	}
#endif

	return ret;
}

/**
@brief 車体中心（エンコーダとエンコーダの間）の速度の算出
@return F:エラー無し T:計算エラー
*/
static BOOL CaluclateMashineSpeed()
{
	BOOL ret = FALSE;
	uint8 i;


	sys_machineSpeed = (sys_leftWheelSpeed + sys_rightWheelSpeed) / 2;

	//計算エラーチェック
	if(sys_machineSpeed  > param_errorLimitOfWheelSpeed)
	{
		ERR_WriteID(EID_CALCULATE_MACHINE_SPEED, __func__, __LINE__);
		comm.printf("speed L:R %d:%d \n",sys_leftWheelSpeed,sys_rightWheelSpeed);
		for(i = 0 ; i < 5; i++){
			comm.printf("enc log c=%d L:R %d:%d  \n",i,logLeftEncorderCount[i],logRightEncorderCount[i]);
		}
		ret = TRUE;
	}

    for(i = 20 ; i > 0; i--)
    {
    	machineSpeed[i] = machineSpeed[i-1];
    }
	machineSpeed[0]		= sys_machineSpeed;

	return ret;
}

/**
@brief 車体角速度の算出
@note 左方向に回転でプラスとする
@return F:エラー無し T:計算エラー
*/
static BOOL CaluclateMashineAngularvelocity()
{
	int32 lEnc,rEnc;
	int32 umL,umR;
	BOOL ret;
	
	ret |= CaluculateDeltaCount(logLeftEncorderCount[0],  logLeftEncorderCount[1],  &lEnc);
	ret |= CaluculateDeltaCount(logRightEncorderCount[0],  logRightEncorderCount[1],  &rEnc);
	
	umL = lEnc * encorder1CountDistination_um;
	umR = rEnc * encorder1CountDistination_um;
	
	sys_angularVelocity = (float64)(umR - umL) / float64(param_encorderGauge * 1000);


//    comm.printf("encDelta LR = %d:%d  umLR %d:%d vero %lf \n",lEnc,rEnc,umL,umR,sys_angularVelocity);
 
    sys_angularRadian += sys_angularVelocity;
    sys_angularDegree = sys_angularRadian * 57.2958;
 	
 	
	return FALSE;
}

/**
@brief 車体加速度、減速度の計算
@return F:エラー無し T:計算エラー
*/
static BOOL CaluclateMashineAcceraretion()
{
	BOOL ret = FALSE;
	uint8 i;
	int32 tmp = 0;
	
	for(i = 0; i < 20; i++){
		tmp += (machineSpeed[i] - machineSpeed[i+1]);
	}    
    sys_acceleration = (tmp / 20) * 50;

	//エラーチェック
#ifndef _DEBUG
	if(abs(sys_acceleration)  > param_errorLimitOfAcceleration)
	{
		ERR_WriteID(EID_CALCULATE_MACHINE_SPEED, __func__, __LINE__);
		ret = TRUE;
	}
#endif
	return ret;
}

/**
@brief 移動距離の計算(累積、区間 両方算出してます)
@return F:エラー無し T:計算エラー
*/
static BOOL CaluclateDistance()
{
	BOOL ret = FALSE;
	int32 deltaL = 0, deltaR = 0;
	int32 tmpcnt;

	//左右エンコーダが5msでどれだけ進んだかを算出  --> 0:現在 1:前回
	ret |= CaluculateDeltaCount(logLeftEncorderCount[0],  logLeftEncorderCount[1],  &deltaL);
	ret |= CaluculateDeltaCount(logRightEncorderCount[0], logRightEncorderCount[1], &deltaR);

	tmpcnt = (deltaL + deltaR) / 2;
	
	sys_odoCount += tmpcnt;
	sys_tripCount += tmpcnt;
	sys_odoMeter = (sys_odoCount * encorder1CountDistination_um) / 1000;
	sys_tripmeter = (sys_tripCount * encorder1CountDistination_um) / 1000;

	return FALSE;
}

/**
@brief 開始地点からの絶対XY座標算出
@return F:エラー無し T:計算エラー
*/
static BOOL CaluclateAbsoluteXYAxis()
{

	int32 lEnc,rEnc;
	int32 umL,umR;
	float64 tmpSpAbs;
	BOOL ret;
	
	ret |= CaluculateDeltaCount(logLeftEncorderCount[0],  logLeftEncorderCount[1],  &lEnc);
	ret |= CaluculateDeltaCount(logRightEncorderCount[0],  logRightEncorderCount[1],  &rEnc);
	
	umL = lEnc * encorder1CountDistination_um;
	umR = rEnc * encorder1CountDistination_um;
	
	tmpSpAbs = (umL+umR) / 2;

	sys_absoluteAxis_X += tmpSpAbs * cos(sys_angularRadian);
	sys_absoluteAxis_Y += tmpSpAbs * sin(sys_angularRadian);

	return FALSE;
}

/**
@brief チェックポイントからの相対XY座標算出
@return F:エラー無し T:計算エラー
*/
static BOOL CaluclateRelativeXYAxis()
{
	
	int32 lEnc,rEnc;
	int32 umL,umR;
	float64 tmpSpAbs;
	BOOL ret;
	
	ret |= CaluculateDeltaCount(logLeftEncorderCount[0],  logLeftEncorderCount[1],  &lEnc);
	ret |= CaluculateDeltaCount(logRightEncorderCount[0],  logRightEncorderCount[1],  &rEnc);
	
	umL = lEnc * encorder1CountDistination_um;
	umR = rEnc * encorder1CountDistination_um;
	
	tmpSpAbs = (umL+umR) / 2;

	sys_relativeAxis_X += tmpSpAbs * cos(sys_angularRadian);
	sys_relativeAxis_Y += tmpSpAbs * sin(sys_angularRadian);
	
	return FALSE;
}

/**
@brief
エンコーダカウンタの差分を計算します。

@param[in]	nowCount		現在のエンコーダカウント
@param[in]	lastCount		差分をとりたい前回のエンコーダカウント
@param[out]	retDeltaCount	カウントの差分
@return F:エラー無し T:何らかのエラー有

@note
エンコーダカウントの最大値に注意してください。
この関数のロジックではエンコーダカウントの最大最小がuint16以外では正しく計算できません
*/
BOOL CaluculateDeltaCount(uint16 nowCount, uint16 lastCount, int32* retDeltaCount)
{
	int32 chkDelta;
	BOOL bret;

	/*
	 65530 -> 10 =  -65520 :+15count
	 10 -> 65530 =   65520 :-15count
	 */
	chkDelta = (int32) nowCount - (int32) lastCount;
	if (chkDelta < -63535)//+方向へオーバーフロー OK
	{
		*retDeltaCount = 65535 + chkDelta;
		bret = FALSE;
	}
	else if (chkDelta > 63535)//-方向へオーバーフロー OK
	{
		*retDeltaCount = chkDelta - 65535;
		bret = FALSE;
	}
	else if (abs(chkDelta) > param_encCountErrorThreshold) //プラスマイナス方向に大きすぎる NG
	{
		ERR_WriteID(EID_ENC_COUNT_IMPROBABLE, __func__, __LINE__);
		bret = TRUE;
	}
	else
	{
		*retDeltaCount = chkDelta;
		bret = FALSE;
	}

	return bret;
}




