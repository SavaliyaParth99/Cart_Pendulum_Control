#ifndef Fixedpt_Speed_Control_H_
#define Fixedpt_Speed_Control_H_


#include "xil_types.h"

short Fixedpt_Speed_Control(long Ref_Velo, long Cart_Pos);
#define SV_f 18287
#define PG_f_Speed 18536
#define IG_f_Speed 198
#define IG_Limit 67076096
#define Out_Limit 16769024


// typedef struct {
// 	s32 kp;
// 	s32 ki;
// 	s32 xi;
//     s32 P_s;
//     s32 Cart_Position_Prev;
// } PiObj_struct;

// void UPiCtrlInit_Speed(s32 Ps, s32 pgain, s32 igain);
// s16 Fixedpt_Speed_Control(s32 Ref_Velo, s32 Cart_Pos);
// s16 spctrl_Speed(s16 reset, s32 RV, s32 CP);

#endif
