/**
 * Pos_Cntrl_FixedPt.h
 *
 *    Abstract:
 *      The purpose of this .h file is to provide function definitions for 
 *      functions in  Pos_Cntrl_FixedPt.c:
 *
 *    
 *    File version: 2.0
 *
 **/


#ifndef POS_CNTRL_FIXEDPT_H_
#define POS_CNTRL_FIXEDPT_H_

short Pos_Cntrl_FixedPt(short Ref_Pos, long Pend_Angle, long Cart_Position);

#define P_si 57147
#define Gp_i 185
#define Gi_i 930
#define upLI_f 1.8119e+09
#define dnLI_f -1.8119e+09
#define upLO_f 2.1454e+09
#define dnLO_f -2.1454e+09
#endif

