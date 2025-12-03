#ifndef Fixedpt_Speed_Control_H_
#define Fixedpt_Speed_Control_H_


#include "xil_types.h"

short Fixedpt_Speed_Control(long Ref_Velo, long Cart_Pos);
#define SV_f 18287
#define PG_f_Speed 4634
#define IG_f_Speed 198
typedef struct {
	s32 kp;
	s32 ki;
	s32 xi;
    s32 P_s;
    s32 Cart_Position_Prev;
} PiObj_struct;

void UPiCtrlInit(s32 Ps, s32 pgain, s32 igain);
s16 Fixedpt_Position_Control(s32 Ref_Velo,s32 Cart_Pos);
s16 spctrl(s16 reset, s32 RV, s32 CP);

#endif