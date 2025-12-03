/*
 ============================================================================
 Name        : spctrl.c
 Author      : Kai Mueller
 Version     : 0.0a as of 18-NOV-2012
 Copyright   : (c) 2011 Univ. Bremerhaven
 Description : PI controller in fixed point
 ============================================================================
 */

#include "xil_types.h"
#include "Fixedpt_Speed_Control.h"
// #include "pi_control.h"


// speed controller (discrete state space)
s16 spctrl_Speed(s16 reset, s32 RV, s32 CP) {
    s16  uu=0;

    if (reset > 0) {
        UPiCtrlInit(PG_f_Speed,IG_f_Speed);
        uu = 0;
    } 
    else{
         uu = Fixedpt_Speed_Control(RV,CP);
    } 
    return uu;
}


