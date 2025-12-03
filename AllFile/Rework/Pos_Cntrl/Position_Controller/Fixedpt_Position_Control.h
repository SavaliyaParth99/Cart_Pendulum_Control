#ifndef Fixedpt_Position_Control_H_
#define Fixedpt_Position_Control_H_
#include "xil_types.h"

short Fixedpt_Position_Control(long Ref_Pos,short Cart_Pos,long Pend_Ang);

#define PG_f 185
#define IG_f 930
typedef struct {
	s32 kp;
	s32 ki;
	s32 xi;
    s32 Pend_Ang_Prev;
    s32 Cart_Position_Prev;
} PiObj_struct;

void UPiCtrlInit(s32 pgain, s32 igain);
s16 Fixedpt_Position_Control(s32 Ref_Pos,s16 Pend_Ang,s32 Cart_Pos);
s16 spctrl(s16 reset, s32 RP, s16 PA, s32 CP);
#endif
