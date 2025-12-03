#ifndef Fixedpt_Position_Control_H_
#define Fixedpt_Position_Control_H_
#include "xil_types.h"

short Fixedpt_Position_Control(short Ref_Pos,long Cart_Pos,short Pend_Ang);

#define SP_f 57147
#define PG_f 185
#define IG_f 930
typedef struct {
	s32 kp;
	s32 ki;
	s32 xi;
    s32 P_s;
    s16 Pend_Ang_Prev;
    s32 Cart_Position_Prev;
} PiObj_struct;

void UPiCtrlInit(s32 Ps, s32 pgain, s32 igain);
s16 Fixedpt_Position_Control(s16 Ref_Pos,s32 Cart_Pos,s16 Pend_Ang);
s16 spctrl(s16 reset, s16 RP, s32 CP, s16 PA);
#endif
