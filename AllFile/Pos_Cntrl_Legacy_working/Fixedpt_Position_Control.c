#include "xil_types.h"
#include "Fixedpt_Position_Control.h"
static PiObj_struct PiObj;

void UPiCtrlInit(s32 Ps, s32 pgain, s32 igain) {
	PiObj.P_s = Ps;
	PiObj.kp = pgain;
	PiObj.ki = igain;
	PiObj.xi = 0;
}

s16 Fixedpt_Position_Control(s16 Ref_Pos,s32 Cart_Pos,s16 Pend_Ang)
{
    // static s32 Cur_I_state=0;
    static s16 Pend_Ang_Prev=0;
    static s32 Cart_Position_Prev=0;
    s32 Feedback_Sum=0;
    s16 Force;
    s32 Pend_Ang_Velo,Cart_Velo,PI_controller,Cntrl_Err, KI_mul, KP_mul;
   
    s16 KTD_f[] = {-19348,-25061,17564,363};
    
    Cntrl_Err = ((Ref_Pos)*(SP_f)) - (Cart_Pos);
    Cntrl_Err=Cntrl_Err;
    KI_mul = Cntrl_Err * IG_f;
     
    PiObj.xi = PiObj.xi + KI_mul; 
    KP_mul = (Cntrl_Err * PG_f)<<7;

    Pend_Ang_Velo = ((Pend_Ang - Pend_Ang_Prev)<<3) * KTD_f[0];
    Cart_Velo = (Cart_Pos - Cart_Position_Prev) * KTD_f[2];
    Pend_Ang_Prev = Pend_Ang;
    Cart_Position_Prev = Cart_Pos;

    Feedback_Sum = Pend_Ang_Velo+Cart_Velo+Pend_Ang*KTD_f[1]+Cart_Position_Prev*KTD_f[3];

    Feedback_Sum = Feedback_Sum<<8;

    PI_controller = KP_mul + PiObj.xi - Feedback_Sum;
    
    PI_controller=PI_controller>>22;
    
    Force = (PI_controller);

    return (Force);
}
