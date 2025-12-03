#include "xil_types.h"
#include "Fixedpt_Position_Control.h"
// static PiObj_struct_Pos PiObj_Pos;

// void UPiCtrlInit(s32 Ps, s32 pgain, s32 igain) {
// 	PiObj_Pos.P_s = Ps;
// 	PiObj_Pos.kp = pgain;
// 	PiObj_Pos.ki = igain;
// 	Cur_Igain = 0;
//     Pend_Ang_Prev = 0;
//     Cart_Position_Prev = 0;
// }

s16 Fixedpt_Position_Control(s32 Ref_Pos,s16 Pend_Ang,s32 Cart_Pos)
{
    static s32 Cur_Igain=0;
    static s16 Pend_Ang_Prev=0;
    static s32 Cart_Position_Prev=0;
    s32 Feedback_Sum=0;
    s16 Force;
    s32 Pend_Ang_Velo,Cart_Velo,PI_controller,Cntrl_Err, KI_mul, KP_mul;
   
    s16 KTD_f[] = {-19348,-25061,17564,363};
    
    Cntrl_Err = Ref_Pos - Cart_Pos;  //int32
    KI_mul = Cntrl_Err * IG_f; //int32.22
     
    Cur_Igain = Cur_Igain + KI_mul;  //int32.22
    KP_mul = (Cntrl_Err * PG_f)<<7;

    Pend_Ang_Velo = ((Pend_Ang - Pend_Ang_Prev)<<3) * KTD_f[0]; //int
    Cart_Velo = (Cart_Pos -  Cart_Position_Prev) * KTD_f[2];
    Pend_Ang_Prev = Pend_Ang;
     Cart_Position_Prev = Cart_Pos;

    Feedback_Sum = Pend_Ang_Velo+Cart_Velo+Pend_Ang*KTD_f[1]+ Cart_Position_Prev*KTD_f[3];

    Feedback_Sum = Feedback_Sum<<8;

    PI_controller = KP_mul + Cur_Igain - Feedback_Sum;
    
    PI_controller=PI_controller>>22;
    
    Force = (PI_controller);

    return (Force);
}
