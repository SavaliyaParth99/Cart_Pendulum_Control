#include "xil_types.h"
#include "Fixedpt_Position_Control.h"
static PiObj_struct PiObj;

void UPiCtrlInit(s32 pgain, s32 igain) {
	PiObj.kp = pgain;
	PiObj.ki = igain;
	PiObj.xi = 0;
    PiObj.Pend_Ang_Prev = 0;
    PiObj.Cart_Position_Prev = 0;
}

s16 Fixedpt_Position_Control(s32 Ref_Pos,s16 Pend_Ang,s32 Cart_Pos)
{
    s32 Feedback_Sum=0;
    s16 Force;
    s32 Pend_Ang_Velo,Cart_Velo,PI_controller,Cntrl_Err, KI_mul, KP_mul;
    //State beedbak gain  
    s16 KTD_f[] = {-19343,-25055,17564,363};
    
    //Calculated control error
    Cntrl_Err = Ref_Pos - Cart_Pos; 
//Multiplication with integral gain
    KI_mul = Cntrl_Err * IG_f;    
//Integral gain addition
    PiObj.xi = PiObj.xi + KI_mul;
//Multiplication with proportional gain and 
//left shifted by 7-bits to match integral gain fixed point format
KP_mul = (Cntrl_Err * PG_f)<<7;
//Getting pendulum angular velocity by derivating pendulum angle
    Pend_Ang_Velo = ((Pend_Ang - PiObj.Pend_Ang_Prev)<<3) * KTD_f[0];
//Getting cart velocity by derivating cart position
    Cart_Velo = (Cart_Pos -  PiObj.Cart_Position_Prev) * KTD_f[2];
    PiObj.Pend_Ang_Prev = Pend_Ang;
    PiObj.Cart_Position_Prev = Cart_Pos;
//Addition of state feedback gain
    Feedback_Sum = Pend_Ang_Velo+Cart_Velo+Pend_Ang*KTD_f[1]+ PiObj.Cart_Position_Prev*KTD_f[3];
//left shifted by 8-bits to match integral gain fixed point format
    Feedback_Sum = Feedback_Sum<<8;
//Main PI controller logic
    PI_controller = KP_mul + PiObj.xi - Feedback_Sum;    
//Right shift by 22 to convert back to int_16 format
    PI_controller=PI_controller>>22;
    Force = (PI_controller);
return (Force);
}

