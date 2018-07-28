#include "mbed.h"
#include "typedef.h"
#include "errorcontrol.h"
#include "communication.h"

BOOL errorIndicator[4];

/**
@brief エラー原因を記入します
IDだけではわかりにくかったり、複合的な理由があった場合に
文字列としてエラーログを残したい場合に使ってください

文字コードは分かりません（UTF-8だと思う）。受信側（PC側）で上手い事フォローしてください。
1メッセージに付き128”バイト”まで、

*/
void ERR_WriteCause()
{
    
}

void ERR_WriteLog()
{
    
}

void ERR_WriteID(errorID eid, const uint8* functionName,int32 line)
{
	comm.printf("EID= %d FuncName= %s Line =%d \n",eid,functionName,line);
}

void ERR_GetLastError()
{
    
}

