/*------------------------------------------------------------------------------------------
* Speed_Cntrl_FixedPt.h
*
* Description: Header file for cart speed control
* Version : 10.0
* Subject : Embedded Systems Project
* Course : Embedded Systems Design, Hochschule Bremerhaven
-------------------------------------------------------------------------------------------*/
#ifndef Speed_Cntrl_FixedPt_H_
#define Speed_Cntrl_FixedPt_H_
#endif
#ifndef XIL_TYPES_H_INCLUDED
#define XIL_TYPES_H_INCLUDED
#endif
short Speed_Ctrl_FixedPt(short Ref_Vel, long Cart_Position, short Pend_Angle);
//Velocity to counts conversion factor in fixed-point
#define V_si 18286
//Prefilter gain in fixed-point
#define Gp_i_speed 70500
//Integral gain in fixed-point
#define Gi_i_speed 550
//Upper limit for the integral channel
#define upLI_f_speed 3932160
//Lower limit for the integral channel
#define dnLI_f_speed -3932160
//Upper limit for the controller output 
#define upLO_f_speed 91750400
//Lower limit for the controller output
#define dnLO_f_speed -91750400

