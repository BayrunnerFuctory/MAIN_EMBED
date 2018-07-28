#include "mbed.h"
#include "typedef.h"
#include "main.h"

#include "deviceinput.h"
#include "deviceoutput.h"
#include "destinationcontrol.h"
#include "calculatemotortarget.h"
#include "calculatemotorrequest.h"
#include "debugsupport.h"
#include "communication.h"
#include "sysdata.h"
#include "parameter.h"
#include "bonusarea.h"

/**
コメントはDoxygenに合わせてね
あとjavadocルールな。よろしく。
**/

DigitalOut myLED1(LED1);
DigitalOut myLED2(LED2);
DigitalOut myLED3(LED3);
DigitalOut myLED4(LED4);

uint32  sys_taskCount   = 0;		///////< メインループカウンタ
BOOL    sys_onError     = FALSE;
Ticker  eventTimer;					///イベントタイマ
BOOL    mainTaskTimer;


uint8 c;
uint8 data[32];


void NotifyEventTimer(void);
void NotifyServoEventTimer(void);

static void SysInit(void);
static void DebugCall(void);

void DebugCall()
{
    debug_printf("hoge %d\r\n",30);
}


uint32 debugCount = 0;

int main()
{

    BOOL onError = FALSE;
    mainTaskTimer = FALSE;

    //各種初期化
    SysInit();

    DebugCall();

    //各種パラメータ値の変更
//	onError |= AdjustParameter();
//	if (onError == TRUE)
//	{
//		break;
//	}


    // イベントタイマー開始  @note NotifyEventTimer関数を5ms間隔で呼ぶ
    eventTimer.attach(&NotifyEventTimer, 0.005);
    
	//201708 debug
	sys_leftMotorDirection=MD_FORWORD;
	sys_rightMotorDirection = MD_FORWORD;


    //メインループ
    while(!onError) {
        
        if (mainTaskTimer == TRUE) {


            mainTaskTimer = FALSE;
            sys_taskCount++;
			
			
	        if(sys_USSCount == 20){
            	sys_USSCount = 0;
            }
            sys_USSCount++;

            //デバイス情報の入手
            onError |= DeviceInput();
            //テスト用デバイス情報強制上書き
            //onError |= DebugDeviceInputDataOverWrite();
            if (onError == TRUE) {
                break;
            }

            ////目的地、状況判断
            //onError |= DistinationControl();
            ////テスト用判断情報強制上書き
            //onError |= DebugDestinationDataOverWrite();
            //if (onError == TRUE) {
            //    break;
            //}

            ////モーター目標値計算
            //onError |= CalculateMotorTarget();
            //テスト用目標値強制上書き
            onError |= DebugTargetOverWrite();
            if (onError == TRUE) {
                break;
            }

            if((sys_taskCount & 0x03) == 0) {
                //モーター要求値計算
                onError |= CalculateMotorRequest();
                //onError |= DebugRequestOverWrite();
                if (onError == TRUE) {
                    break;
                }

				//if(sys_nowRouteState == RS_SYUMAI) {
	            //    //ボーナスエリア
	            //    myLED1 = 1;
	            //    myLED2 = 1;
	            //    myLED3 = 1;
	            //    myLED4 = 1;
	
	            //   onError |= BonusArea();
	            //    if (onError == TRUE) {
	            //        break;
	            //    }
	            //}

                //デバイス出力命令
                onError |= DeviceOutput();
                if (onError == TRUE) {
                    break;
                }
                
                
            }
            if((sys_taskCount % 5) == 0)
            {
            }
            
            ServoOutput();
			

/*
            //sys_nowRouteState = RS_SYUMAI;

            if(sys_nowRouteState == RS_SYUMAI) {
                //pwm_servo.pulsewidth_us(100);
                //ボーナスエリア
                myLED1 = 1;
                myLED2 = 1;
                myLED3 = 1;
                myLED4 = 1;

                //onError |= BonusArea();
                if (onError == TRUE) {
                    break;
                }
            }
*/
            //メカ,状況モニター用にPCへデータの出力
            if ((sys_taskCount % 40) == 0) {
                //CommunicationOutputToPC();
//                comm.printf(" Left PWM period:%f %f \r\n", pwm_L1.read(), pwm_L2.read());
//                comm.printf(" Right PWM period:%f %f", pwm_R1.read(), pwm_R2.read());
//                comm.printf(" \r\n");

                comm.printf(" task=%d |",sys_taskCount);
                //comm.printf(" state %d substate %d |",sys_nowRouteState, sys_toGoalSubState);
                //comm.printf(" Enc L:R=%d %d |",sys_leftEncorderCount,sys_rightEncorderCount);
                //comm.printf(" speedL:R:M=%d:%d:%d |",sys_leftWheelSpeed,sys_rightWheelSpeed,sys_machineSpeed);
                //comm.printf(" rad=%lf |",sys_angularVelocity);
                //comm.printf(" \r\n");
                //comm.printf(" vatt=%lf |",sys_batteryVoltage);
                comm.printf(" \r\n");
                comm.printf(" targetDuty L:R=%d:%d |",sys_targetLeftMotorDuty,sys_targetRightMotorDuty);
                comm.printf(" \r\n");
                ////comm.printf(" reqDuty L:R=%d:%d |",sys_requestLeftMotorDuty,sys_requestRightMotorDuty);
                ////comm.printf(" \r\n");
                //comm.printf(" steer order=%d |",(int32)sys_nowSteerOrder);
                //comm.printf(" Line Sensor=%d %d|",(int32)sys_frontLineSenser,(int32)sys_rearLineSenser);
                //comm.printf(" Angle Deg=%lf Rad=%lf |",sys_angularDegree,sys_angularRadian);
                comm.printf(" AXIS(X, Y) = %d,%d |",sys_absoluteAxis_X,sys_absoluteAxis_Y);
                comm.printf(" \r\n");
                //comm.printf(" tripmater =%d |",sys_tripmeter);
                //debugCount += sys_ussBackEchoWidth;
                comm.printf("PulsWidth Left:Back:Light = %d %d %d|", sys_ussBackEchoWidth);
                comm.printf(" \r\n");
                comm.printf(" LineSenser %f, %f, %f |",sys_frontSenserLL,sys_frontSenserCL,sys_frontSenserCC);
                comm.printf(" LineSenser %f, %f |",sys_frontSenserCR,sys_frontSenserRR);
                comm.printf(" \r\n");
            }
        }

        //PCからデータを受信
        onError = CommunicationInputFromPC();
        if (onError == TRUE) {
            break;
        }

    }//------------main loop enda

    //---ここからエラー処理
    DeviceOutputForceTerminate();

    comm.printf("On ERROR \r\n");


    comm.printf("system:%d \r\n", sys_leftMotorDirection);

    comm.printf(" task=%d |",sys_taskCount);
    comm.printf(" state %d |",sys_nowRouteState);
    comm.printf(" Enc L:R=%d %d |",sys_leftEncorderCount,sys_rightEncorderCount);
    comm.printf(" speedL:R:M=%d:%d:%d |",sys_leftWheelSpeed,sys_rightWheelSpeed,sys_machineSpeed);
    comm.printf(" rad=%lf |",sys_angularVelocity);
    comm.printf(" \r\n");
    comm.printf(" vatt=%lf |",sys_batteryVoltage);
    comm.printf(" reqDuty L:R=%d:%d |",sys_requestLeftMotorDuty,sys_requestRightMotorDuty);
    comm.printf(" steer order=%d |",(int32)sys_nowSteerOrder);
    comm.printf(" Angle Deg=%lf Rad=%lf |",sys_angularDegree,sys_angularRadian);
    comm.printf(" AXIS X=%d Y=%d |",sys_absoluteAxis_X,sys_absoluteAxis_Y);

    comm.printf(" \r\n");

	//念のためフリーに設定
	sys_requestLeftMotorDuty = 0;
	sys_requestRightMotorDuty = 0;
	DeviceOutput();
	    
    //エラー処理
    while(onError) {

        //LEDつけよう
	    CommunicationInputFromPC();
        sys_onError = TRUE;
        myLED1 = 1;
        myLED2 = 1;
        myLED3 = 1;
        myLED4 = 1;
        wait(1.00);
	    //CommunicationInputFromPC(); //1秒Waitがあるので1秒ごとにコマンドチェック
        myLED1 = 0;
        myLED2 = 0;
        myLED3 = 0;
        myLED4 = 0;
        wait(1.00);
        
    }
    
    
    
    while(TRUE); //mainから抜けない為のループ
}


/**
＠brief
システム初期化処理

@note
ポート設定やペリフェラルのセッティング
タイマーのリセット
グローバル値の初期化を行います。
*/
static void SysInit(void)
{
    DeviceInputInit();
    DeviceOutputInit();
    CommunicationInit();
    DistinationControlInit();
    CalculateMotorTargetInit();

}


/**
＠brief
イベントタイマー呼び出し

@note
基本的にフラグ処理だけにしてください。
この中に処理を書くと他の割り込み（エンコーダとか）
が遅れる・・・と思う。
*/
void NotifyEventTimer()
{
    mainTaskTimer = TRUE;
}
