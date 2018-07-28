#pragma once
#include "mbed.h"
#include "typedef.h"

extern void JudgeStateInit(void);
extern BOOL JudgeState(void);


extern route_state  sys_nowRouteState;          ///< 現在の位置
extern route_state  sys_lastRouteState;         ///< 前回の位置（ラインロスト時に利用）



