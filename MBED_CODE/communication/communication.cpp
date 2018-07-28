#include "mbed.h"
#include "typedef.h"
#include "communication.h"
#include "debugsupport.h"
#include "deviceinput.h"
#include "deviceoutput.h"
#include <string>

#include "communication.h"

#include "adjustparameter.h"

#include "sysdata.h"

//シリアル処理するクラス  
//The Serial Interface defaults to a 9600 baud standard serial connection (8 bits, 1 stop bit, no parity)
//--> see http://developer.mbed.org/handbook/SerialPC

#define OUTPUT_STR_BUFFER 1024

//Serial comm(USBTX, USBRX);
Serial comm(p28, p27);
//DigitalOut terminate(p29);
DigitalOut mLED1(LED1);
DigitalOut mLED2(LED2);
DigitalOut mLED3(LED3);
DigitalOut mLED4(LED4);

void CommunicationInit(void);
BOOL CommunicationOutputToPC(void);
BOOL CommunicationInputFromPC(void);
BOOL RxSerial(uint8*);
BOOL SetCommandValue(uint8*, uint8*);
BOOL SetSpecialCommand(uint8*);
BOOL CheckCommand(uint8*);
void CommunicationInit()
{
	comm.baud(115200);
}

/**
@brief
PCへのメッセージ送信処理
@return FALSE：エラー無し TURE：何らかのエラー
@note
現状、シリアル通信のみ。デバッグコマンドとパラメータ変更コマンド
*/
BOOL CommunicationOutputToPC()
{
	//output system param
	comm.printf("%d ", sys_taskCount);
	//comm.printf("%d ", sys_enableStartSwitch);
	comm.printf("%d ", d_param_motorPeriod);
	comm.printf("%d ", d_sys_leftMotorDirection);
	
	comm.printf("%d ", d_sys_rightMotorDirection);
	comm.printf("%d ", d_sys_requestLeftMotorDuty);
	comm.printf("%d ", d_sys_requestRightMotorDuty);
	comm.printf("%d ", d_sys_targetLeftMotorSpeed);
	comm.printf("%d ", d_sys_targetRightMotorSpeed);
	comm.printf("%d ", d_sys_targetLeftMotorDuty);
	comm.printf("%d ", d_sys_targetRightMotorDuty);
	comm.printf("%d ", d_sys_frontLineSenser);
	comm.printf("%d ", d_sys_rearLineSenser);
	comm.printf("%d ", d_sys_leftEncorderCount);
	comm.printf("%d ", d_sys_rightEncorderCount);
	comm.printf("%d ", d_sys_odoCount);
	comm.printf("%d ", d_sys_odoMeter);
	comm.printf("%d ", d_sys_tripCount);
	comm.printf("%d ", d_sys_tripmeter);
	comm.printf("%d ", d_sys_enableStartSwitch);
	comm.printf("%d ", d_sys_machineSpeed);
	comm.printf("%d ", d_sys_leftWheelSpeed);
	comm.printf("%d ", d_sys_rightWheelSpeed);
	comm.printf("%d ", d_sys_acceleration);
	comm.printf("%d ", d_sys_absoluteAxis_X);
	comm.printf("%d ", d_sys_absoluteAxis_Y);
	comm.printf("%d ", d_sys_relativeAxis_X);
	comm.printf("%d ", d_sys_relativeAxis_Y);
	comm.printf("%d ", d_sys_nowRouteState);
	comm.printf("%d ", d_sys_lastRouteState);
	comm.printf("%d ", d_sys_firstState);
	comm.printf("%d ", d_sys_nowSteerOrder);
	comm.printf("%d ", d_sys_targetSpeed);
	comm.printf("%d ", d_param_distance1stVCPAxisX);
	comm.printf("%d ", d_param_distance2ndVCPAxisX);
	comm.printf("%d ", d_param_distanceSlopCenterAxisX);
	comm.printf("%d ", d_param_distance6thVCPignoreRelativeAxisY);
	comm.printf("%d ", d_param_distance7thVCPignoreRelativeAxisX);
	comm.printf("%d ", d_sys_servoAngle);
	comm.printf("%f ", d_sys_angularVelocity);
	
	comm.printf("\r\n");
	//output debug params
	//comm.printf("%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %f \n\n", );
	return FALSE;
}


/**
@brief
PCからのメッセージ受信処理
@return FALSE：エラー無し TURE：緊急停止
@note
現状、シリアル通信のみ。デバッグコマンドとパラメータ変更コマンド
*/
BOOL CommunicationInputFromPC()
{
	uint8 input[128];
    uint8 rtn = 0;   //戻り値
    
    //シリアルメッセージ受信
    rtn = RxSerial(input);

	//未受信の場合、処理終了
    if(rtn == FALSE)
    {
        return FALSE;
    }
    
    //コマンド受信処理
    //mLED1=1;
    rtn = CheckCommand(input);
    //mLED1=0;
    if(rtn == TRUE)
    {
    	return TRUE;	
    }
    
    /*
    //デバックコマンド受信処理
    if(input[0] == 'd'){
    	if(input[1] == 's'){
    		return(TRUE);
    	}
    }

    if(input[0] == 'd'){
    	rtn = RxDebugCommand(input + 1);
		comm.printf("rtn:%d\n", rtn);
    	if(rtn == 1)
    	{
    		return (TRUE);
    	}
    }
    else if(input[0] == 'a'){
    	rtn = RxAdjustParameter(input + 1);
    }
    else if(input[0] == 'o'){
    	rtn = RxOverWriteData(input + 1);
    }
    else{
    	//
    }
    */
	return FALSE;
}
/**
@brief
PCからの受信データチェック
@return FALSE：エラー無し TURE：何らかのエラー
@note
*/
static BOOL RxSerial(uint8* input)
{
	int8 rtn = 0;	//戻り値
	
	//受信データチェック
    if(comm.readable())
    {
        memset(input, 0, sizeof(input));
        //シリアル通信受信処理
        rtn = comm.scanf("%s", input);
        //戻り値チェック
        if(rtn == -1)
        {
            comm.printf("scanf err!\n", rtn);
            return(FALSE);
        }
        
		//20151010変更箇所
		//ZigBeeがIDの様なものを付与してくるのでそれを除外する。
		//ZigBeeが付与する文字列の最後にスペースがある為、二通に分かれて受信する。
		//1通目を無視し、2通目を処理すればOK。1通目のフォーマットは以下
		//[xxxxxxxx:y] x:ID y:メッセージ番号
		if(input[0] == '['){
			return(FALSE);
		}
        return(TRUE);
    }

    return (FALSE);
}

/**
@brief
受信データを解析し、コマンド命令を実行する
@return FALSE：エラー無し TURE：Stopコマンドによる緊急停止
@note
*/
BOOL CheckCommand(uint8* input)
{
	uint8 command[] = "aaaa";
    uint8 value[] = "aaaa";
	int commandIndex = 0;
	int valueIndex = 0;
    BOOL commandFlag = TRUE;
    BOOL specialCommandFlag = TRUE;
    
    memset(command, 0, sizeof(command));
    memset(value, 0, sizeof(value));
    for(int i = 0; i < 96; i++)
    {
    	//mLED2=1;
    	
        if(input[i] == '_')
        { 
            //”_”以前の文字列をコマンドとして取得する
            commandFlag = FALSE;
            commandIndex = 0;
        }
        else if(input[i] == '|')
        {
            //”｜”以降はコマンドとして保持
            if(specialCommandFlag == TRUE)
            {
                //最初のコマンドは特殊命令が入るので、SetSpecialCommandで処理する
                //startSwitchだけここで直接書き換える
            	//comm.printf("%s %s\r\n", command, value);
                if(SetSpecialCommand(command) == TRUE)
                { 
                    return TRUE;
                }
                specialCommandFlag = FALSE;
	            commandIndex = 0;
            }
            else
            {
    			mLED3=1;
	            //この時点で、コマンドと値を保持できているので、一時保存変数に値を入れる
	            //書き換えは各OverWrite関数で行う
	            //”｜”以前の文字列をvalueとして取得する
	            SetCommandValue(command, value);
	            
            	//comm.printf("%s %s\r\n", command, value);
	            commandFlag = TRUE;
	            valueIndex = 0;
            }
        }
        else if(input[i] == '#')
        {
        	//#は終わり文字なので、ここで終了
        	return FALSE;
        }
        else
        {
            //区切り文字以外はコマンドor値として保持
            if(input[i] != ' ')
            {
	            if(commandFlag)
	            {
	                command[commandIndex] = input[i];
	                commandIndex++;
	            }
	            else
	            {
	                value[valueIndex] = input[i];
	                valueIndex++;
	            }
            }
        }
        
        
    	if(input[i] == '\0')
    	{
    		break;
    	}
    }
    
    //mLED2=0;
    //mLED3=0;
    return FALSE;
}


/**
@brief
受信文字の最初の4文字に対する処理
特殊コマンド
■stop -> Stop 		緊急停止
■strt -> Start 		スタートボタン sys_enableStartSwitchをTRUEにする
状態書き換えコマンド
■comd -> Command 	常時書き換えフラグをONにする
■onet -> OneTime 	１回のみ書き換えフラグをONにする
上記以外のコマンド
書き換えフラグをOFFにしてOverWriteさせないようにする

@return FALSE:エラー無 TRUE:Stopコマンドが来たので緊急停止
@note
*/
BOOL SetSpecialCommand(uint8* command)
{   
    if(strcmp(command, "stop") == 0)   //緊急停止
    {
    	comm.printf("Stop\r\n"); 
    	return TRUE;
    }
    else if(strcmp(command, "strt") == 0)  //スタート
    {
    	sys_enableStartSwitch = TRUE;
    	comm.printf("Start\r\n");
    }
    else if(strcmp(command, "comd") == 0)	//常時書き換え
    {
    	d_sys_InputOverWrite = TRUE;
    	d_sys_DestinationDataOverWrite = TRUE;
    	d_sys_RequestOverWrite = TRUE;
    	d_sys_TargetOverWrite = TRUE;
    	comm.printf("Always over write\r\n");
    }
    else if(strcmp(command, "onet") == 0)	//1回のみ書き換え
    {
    	d_sys_OneTimeInputOverWrite = TRUE;
    	d_sys_OneTimeDestinationDataOverWrite = TRUE;
    	d_sys_OneTimeRequestOverWrite = TRUE;
    	d_sys_OneTimeTargetOverWrite = TRUE;
    	comm.printf("One time over write\r\n");
    }
    else
    {
    	/*
    	d_sys_InputOverWrite = FALSE;
    	d_sys_DestinationDataOverWrite = FALSE;
    	d_sys_RequestOverWrite = FALSE;
    	d_sys_TargetOverWrite = FALSE;
    	*/
    	d_sys_OneTimeInputOverWrite = FALSE;
    	d_sys_OneTimeDestinationDataOverWrite = FALSE;
    	d_sys_OneTimeRequestOverWrite = FALSE;
    	d_sys_OneTimeTargetOverWrite = FALSE;
    	
    	//comm.printf("Unknown command\r\n");
    }
    //comm.printf("SpecialCommand: %s a\r\n", command);
	return FALSE;
}

/**
@brief
コマンドごとに設定する値の型をStringからそれぞれに対応した型にキャストし保存する
@return FALSE：エラー無し TURE：処理なし
@note
*/
BOOL SetCommandValue(uint8* command, uint8* value)
{
    if(strcmp(command, "0000") == 0 )
    { 
    	d_param_motorPeriod = atoi(value);
    	//comm.printf("d_param_motorPeriod:%s %s\r\n", command, value);
    }
    else if(strcmp(command,  "0001") == 0 )
    {
    	d_sys_leftMotorDirection = (motorDirection)atoi(value);
    }
    else if(strcmp(command,  "0002") == 0 )
    {
    	d_sys_rightMotorDirection = (motorDirection)atoi(value); 
    } 
    else if(strcmp(command,  "0003") == 0 )
    {
    	d_sys_requestLeftMotorDuty = atoi(value); 
    }
    else if(strcmp(command,  "0004") == 0 ) 
    {
    	d_sys_requestRightMotorDuty = atoi(value); 
    } 
    else if(strcmp(command,  "0005") == 0 )
    {
    	d_sys_targetLeftMotorSpeed = atoi(value); 
    } 
    else if(strcmp(command,  "0006") == 0 )
    {
    	d_sys_targetRightMotorSpeed = atoi(value);
    } 
    else if(strcmp(command,  "0007") == 0 )
    {
    	d_sys_targetLeftMotorDuty = atoi(value);
    } 
    else if(strcmp(command,  "0008") == 0 )
    {
    	d_sys_targetRightMotorDuty = atoi(value);
    } 
    else if(strcmp(command,  "0009") == 0 )
    {
    	d_sys_frontLineSenser = atoi(value);
    } 
    else if(strcmp(command,  "0010") == 0 )
    {
    	d_sys_rearLineSenser = atoi(value);
    } 
    else if(strcmp(command,  "0011") == 0 )
    {
    	d_sys_leftEncorderCount = atoi(value); 
    } 
    else if(strcmp(command,  "0012") == 0 )
    {
    	d_sys_rightEncorderCount = atoi(value); 
    } 
    else if(strcmp(command,  "0013") == 0 )
    {
    	d_sys_odoCount = atoi(value); 
    } 
    else if(strcmp(command,  "0014") == 0 )
    {
    	d_sys_odoMeter = atoi(value); 
    } 
    else if(strcmp(command,  "0015") == 0 )
    {
    	d_sys_tripCount = atoi(value); 
    } 
    else if(strcmp(command,  "0016") == 0 )
    {
    	d_sys_tripmeter = atoi(value);
    } 
    else if(strcmp(command,  "0017") == 0 )
    {
    	d_sys_enableStartSwitch = atoi(value);
    } 
    else if(strcmp(command,  "0018") == 0 )
    {
    	d_sys_machineSpeed = atoi(value); 
    } 
    else if(strcmp(command,  "0019") == 0 )
    {
    	d_sys_leftWheelSpeed = atoi(value); 
    } 
    else if(strcmp(command,  "0020") == 0 )
    { 
    	d_sys_rightWheelSpeed = atoi(value); 
    } 
    else if(strcmp(command,  "0021") == 0 )
    { 
    	d_sys_angularVelocity = atof(value); //float
    } 
    else if(strcmp(command,  "0022") == 0 )
    { 
    	d_sys_acceleration = atoi(value); 
    }
    else if(strcmp(command,  "0023") == 0 )
    { 
    	d_sys_absoluteAxis_X = atoi(value);// * 1000;
    }
    else if(strcmp(command,  "0024") == 0 )
    { 
    	d_sys_absoluteAxis_Y = atoi(value);// * 1000;
    }
    else if(strcmp(command,  "0025") == 0 )
    {
    	d_sys_relativeAxis_X = atoi(value); 
    }
    else if(strcmp(command,  "0026") == 0 )
    {
    	d_sys_relativeAxis_Y = atoi(value);
    }
    else if(strcmp(command,  "0027") == 0 )
    {
    	d_sys_nowRouteState = (route_state)atoi(value);
    }
    else if(strcmp(command,  "0028") == 0 )
    {
    	d_sys_lastRouteState = (route_state)atoi(value);
    }
    else if(strcmp(command,  "0029") == 0 )
    {
    	d_sys_firstState = (route_state)atoi(value);
    }
    else if(strcmp(command,  "0030") == 0 )
    {
    	d_sys_nowSteerOrder = (steer_order)atoi(value);
    }
    else if(strcmp(command,  "0031") == 0 )
    {
    	d_sys_targetSpeed = atoi(value);
    }
    else if(strcmp(command,  "0032") == 0 )
    { 
    	d_param_distance1stVCPAxisX = atoi(value); 
    }
    else if(strcmp(command,  "0033") == 0 )
    {
    	d_param_distance2ndVCPAxisX = atoi(value); 
    }
    else if(strcmp(command,  "0034") == 0 )
    {
    	d_param_distanceSlopCenterAxisX = atoi(value);
    }
    else if(strcmp(command,  "0035") == 0 )
    {
    	d_param_distance6thVCPignoreRelativeAxisY = atoi(value); 
    }
    else if(strcmp(command,  "0036") == 0 )
    {
        d_param_distance7thVCPignoreRelativeAxisX = atoi(value); 
    }
    else if(strcmp(command,  "0037") == 0 )
    {
        d_sys_servoAngle = atoi(value); 
    }
        
    return FALSE;
}

