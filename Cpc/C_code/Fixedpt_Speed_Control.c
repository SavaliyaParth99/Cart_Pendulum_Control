#include "xil_types.h"
#include "Fixedpt_Speed_Control.h"

// static PiObj_struct PiObj;

// void UPiCtrlInit_Speed(s32 Ps, s32 pgain, s32 igain) {
// 	PiObj.P_s = Ps;
// 	PiObj.kp = pgain;
// 	PiObj.ki = igain;
// 	PiObj.xi = 0;
//     PiObj.Cart_Position_Prev=0;
// }
s16 Fixedpt_Speed_Control(s32 Ref_Velo, s32 Cart_Pos) {
 static s32 Cur_Igain = 0;
 static s32 Cart_Position_Prev = 0;
 s16 Force=0; 
 
 s32 KP_mul,KI_mul,Cntrl_Err,PI_controller,Cart_Velocity;

 Cart_Velocity = (Cart_Pos-Cart_Position_Prev);
 Cart_Position_Prev = Cart_Pos; 

 Cntrl_Err = Ref_Velo-Cart_Velocity;

 KP_mul = (Cntrl_Err*PG_f_Speed)<<2;
 KI_mul = Cntrl_Err*IG_f_Speed;
 Cur_Igain = (Cur_Igain) + KI_mul;
 if (Cur_Igain>IG_Limit){
	 Cur_Igain=IG_Limit;
 }
 else if(Cur_Igain < -IG_Limit){
	 Cur_Igain = -IG_Limit;
 }

  PI_controller = KP_mul + Cur_Igain;
  if (PI_controller>Out_Limit){
	  PI_controller=Out_Limit;
  }
  else if(PI_controller < -Out_Limit){
	  PI_controller = -Out_Limit;
  }
  PI_controller = PI_controller>>17;
  Force= PI_controller;
  return (Force);
 }
