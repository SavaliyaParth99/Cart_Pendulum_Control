/*------------------------------------------------------------------------------------------
* Speed_Ctrl_FixedPt.c
*
* Description: Generate force for cart speed control
* Version : 10.0
* Subject : Embedded Systems Project
* Course : Embedded Systems Design, Hochschule Bremerhaven
-------------------------------------------------------------------------------------------*/
#include "Speed_Ctrl_FixedPt.h"
#include "xil_types.h"
s16 Speed_Ctrl_FixedPt(s16 Ref_Vel, s32 Cart_Position, s16 Pend_Angle) {
 static s32 Cur_i_state = 0;
 static s32 Prev_Cart_Position = 0;
 s16 Force; 
 s32 PI_controller,Gp_mul,Gi_mul,Control_Err,Cart_Velocity;
 //Derivating Cart Position to get Cart Velocity
 Cart_Velocity = (Cart_Position-Prev_Cart_Position)<<19;
 Prev_Cart_Position = Cart_Position;
 //Control error is calculated
 Control_Err = ((Ref_Vel * V_si)-Cart_Velocity)>> 16;
 //Multiply with proportional gain
 Gp_mul = (Control_Err*Gp_i_speed)<<2;
 //Multiply with integral gain
 Gi_mul = Control_Err*Gi_i_speed;   
 Cur_i_state = Cur_i_state + Gi_mul;
 if (Cur_i_state > upLI_f_speed){
 Cur_i_state = upLI_f_speed;
 }
 else if(Cur_i_state< dnLI_f_speed){
 Cur_i_state = dnLI_f_speed;
 }
 //Main PI logic
 PI_controller = Gp_mul + Cur_i_state;
 //Applying upper and lower limits
 if (PI_controller > upLO_f_speed){
 PI_controller = upLO_f_speed;
 }
 else if(PI_controller < dnLO_f_speed){
 PI_controller = dnLO_f_speed;
 }
 //Right shift by 18 to convert back to int16 format
 PI_controller=PI_controller    ;
  //Force calculation
  Force = ( PI_controller);
  return (Force);
 }
