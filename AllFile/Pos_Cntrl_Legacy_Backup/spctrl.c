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
#include "Fixedpt_Position_Control.h"
// #include "spctrl.h"

// speed controller (discrete state space)
s16 spctrl(s16 reset, s16 RP, s32 CP, s16 PA) {
    s16  uu=0;

    if (reset > 0) {
        UPiCtrlInit(SP_f,PG_f,IG_f);
        uu = 0;
    } 
    else{
         uu = Fixedpt_Position_Control(RP,CP,PA);
    } 
    return uu;
}

