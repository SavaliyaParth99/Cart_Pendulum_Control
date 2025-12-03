#ifndef Fixedpt_Speed_Control_H_
#define Fixedpt_Speed_Control_H_


#include "xil_types.h"

short Fixedpt_Speed_Control(long Ref_Velo, long Cart_Pos);
#define PG_f_Speed 18536
#define IG_f_Speed 198
#define IG_Limit 67108864
#define Out_Limit 134217728
typedef struct {
	s32 kp;
	s32 ki;
	s32 xi;
    s32 Cart_Position_Prev;
} PiObj_struct;

void UPiCtrlInit(s32 pgain, s32 igain);
s16 Fixedpt_Position_Control(s16 Ref_Pos,s32 Cart_Pos,s16 Pend_Ang);
s16 spctrl_Speed(s16 reset, s32 RV, s32 CP);

#endif