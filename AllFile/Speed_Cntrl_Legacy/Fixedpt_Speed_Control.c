#include "xil_types.h"
#include "Fixedpt_Speed_Control.h"

static PiObj_struct PiObj;

void UPiCtrlInit(s32 Ps, s32 pgain, s32 igain) {
	PiObj.P_s = Ps;
	PiObj.kp = pgain;
	PiObj.ki = igain;
	PiObj.xi = 0;
    PiObj.Cart_Position_Prev=0;
}
s16 Fixedpt_Speed_Control(s32 Ref_Velo, s32 Cart_Pos) {
//  static s32 Cur_I_state = 0;
//  static s32 Cart_Position_Prev = 0;
 s16 Force=0; 
 
 s32 KP_mul,KI_mul,Cntrl_Err,PI_controller,Cart_Velocity;

 Cart_Velocity = (Cart_Pos-PiObj.Cart_Position_Prev)<<14;
 PiObj.Cart_Position_Prev = Cart_Pos; 

 Cntrl_Err = ((Ref_Vel<<32)-Cart_Velocity);

 KP_mul = (Cntrl_Err*PG_f_Speed)<<2;
 KI_mul = Cntrl_Err*IG_f_Speed;   
 PiObj.xi = PiObj.xi + KI_mul;


  PI_controller = KP_mul + PiObj.xi;

  Force = PI_controller>>20;
  
  return (Force);
 }
