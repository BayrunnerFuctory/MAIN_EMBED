#pragma once
#include "typedef.h"

extern BOOL b_HavingBottle_flag;   /// ペットボトルを持っているフラグ

BOOL BonusArea();
static BOOL BonusArea_Find();
static BOOL BonusArea_Find_TurnRight();
static BOOL BonusArea_Find_GoForward();
static BOOL BonusArea_Find_TurnRight_Caluculate_Request();
static BOOL BonusArea_Find_GoForward_Caluculate_Request();
static BOOL BonusArea_ReFind();
static BOOL BonusArea_PutBottle();
