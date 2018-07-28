#include "mbed.h"
#include "typedef.h"
#include "adjustparameter.h"

#include "communication.h"
#include "parameter.h"

BOOL AdjustParameter(void);

/**
@brief
各種パラメータ値を上書きします。

@note
このパラメータ変更によりマシン設定が変わるので、チューニング用プログラムからの
通信のみで呼び出されるようにしてください。

@return F:エラー無し T:何らかのエラー有
*/
BOOL AdjustParameter()
{
    return FALSE;
}

