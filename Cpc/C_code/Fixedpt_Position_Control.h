#ifndef Fixedpt_Position_Control_H_
#define Fixedpt_Position_Control_H_
#include "xil_types.h"

short Fixedpt_Position_Control(long Ref_Pos,short Pend_Ang,long Cart_Pos);

#define SP_f 57147
#define PG_f 185
#define IG_f 930
// typedef struct {
// 	s32 kp;
// 	s32 ki;
// 	s32 xi;
//     s32 P_s;
//     s16 Pend_Ang_Prev;
//     s32 Cart_Position_Prev;
// } PiObj_struct_Pos;

// void UPiCtrlInit(s32 Ps, s32 pgain, s32 igain);
// s16 Fixedpt_Position_Control(s32 Ref_Pos,s16 Pend_Ang,s32 Cart_Pos);
// s16 spctrl_Pos(s16 reset, s32 RP, s16 PA, s32 CP);
#endif
