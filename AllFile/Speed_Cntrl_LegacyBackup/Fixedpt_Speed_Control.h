#include "xil_types.h"
#include "Fixedpt_Speed_Control.h"
static PiObj_struct PiObj;

// Reset the C code
void UPiCtrlInit(s32 pgain, s32 igain) {
PiObj.kp = pgain;
PiObj.ki = igain;
PiObj.xi = 0;
PiObj.Cart_Position_Prev=0;
}
s16 Fixedpt_Speed_Control(s32 Ref_Velo, s32 Cart_Pos)
{
s16 Force=0;
s32 KP_mul,KI_mul,Cntrl_Err,PI_controller,Cart_Velocity;

//Getting cart velocity by derivating cart position
Cart_Velocity = (Cart_Pos-PiObj.Cart_Position_Prev);
PiObj.Cart_Position_Prev = Cart_Pos;
//Calculated control error
Cntrl_Err = Ref_Velo-Cart_Velocity;
//Multiplication with proportional gain
KP_mul = (Cntrl_Err*PG_f_Speed);
//Multiplication with integral gain
KI_mul = Cntrl_Err*IG_f_Speed;
//Integral gain addition
PiObj.xi = (PiObj.xi) + KI_mul;
//Applying upper and lower limits
if (PiObj.xi>IG_Limit){
PiObj.xi=IG_Limit;
}
elseif(PiObj.xi < -IG_Limit){
PiObj.xi = -IG_Limit;
}
//Main PI controller logic
PI_controller = KP_mul + PiObj.xi;
//Applying upper and lower limits
if (PI_controller>Out_Limit){
PI_controller=Out_Limit;
}
elseif(PI_controller < -Out_Limit){
PI_controller = -Out_Limit;
}
Force = PI_controller>>15;
return (Force);
}

