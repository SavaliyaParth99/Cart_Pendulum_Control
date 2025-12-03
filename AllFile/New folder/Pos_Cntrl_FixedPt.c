/**
 * Pos_Cntrl_FixedPt.c
 *
 *    Abstract:
 *      The purpose of this .c file is to generate the required force value 
 *      by means of PI control logic:
 *
 *    
 *    File version: 2.0
 *
 **/

#include "Pos_Cntrl_FixedPt.h"
#include "xil_types.h"

s16 Pos_Cntrl_FixedPt(s16 Ref_Pos, s32 Cart_Position, s16 Pend_Angle)
{
    static s32 Cur_i_state = 0;
    static s16 Prev_Pend_Ang = 0;
    static s32 Prev_Cart_Position = 0;
    u8 States = 4;
    s32 Feedback[States];
    s32 Feedback_Sum = 0;
    s16 Force;
    s32 Pend_Ang_Velocity, Cart_Velocity, PI_controller, Feedback_mul, Control_Err, Gp_mul,Gi_mul;
    s16 Kst_f[]={-26228,-21418,14344,340};
    
    /*PI Controller logic*/
    Control_Err = ((Ref_Pos)*(P_si)) - (Cart_Position);
    Control_Err=Control_Err;
    Gi_mul = Control_Err*Gi_i;
   
    Cur_i_state = Cur_i_state + Gi_mul;
    Gp_mul = (Control_Err*Gp_i)<<6;
    
    /* Derivating Pendulum Angle and  Cart Position to get 
    Pendulum Angular Velocity and Cart Velocity*/
    Pend_Ang_Velocity = ((Pend_Angle-Prev_Pend_Ang)<<3)*Kst_f[0];
    Prev_Pend_Ang = Pend_Angle;
    Cart_Velocity = (Cart_Position-Prev_Cart_Position)*Kst_f[2];
    Prev_Cart_Position = Cart_Position;
    Feedback[0] = Cart_Velocity;
    Feedback[1] = Cart_Position*Kst_f[3];
    Feedback[2] = Pend_Ang_Velocity;
    Feedback[3] = Pend_Angle*Kst_f[1];
    
    /*State feedback gain summation*/
    for (int k = 0; k < States; k++)
    {
        Feedback_Sum += Feedback[k];
    }
    Feedback_Sum = Feedback_Sum<<6;
    PI_controller = Gp_mul + Cur_i_state - Feedback_Sum;
    if (PI_controller > upLO_f){
       PI_controller = upLO_f;
    }
    else if(PI_controller < dnLO_f){
        PI_controller = dnLO_f;
    }
    PI_controller=PI_controller >>21;
    
    /*Force calculation*/
	Force = PI_controller; //main
	return (Force);
}
