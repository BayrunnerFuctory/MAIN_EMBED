#pragma once
#include "typedef.h"

extern void CommunicationInit(void);
extern BOOL CommunicationOutputToPC(void);
extern BOOL CommunicationInputFromPC(void);
extern Serial comm;
