#include "Fixedpoint_Pos_Control.h"
#include "xil_types.h"

#define STATES 4

typedef struct
{
    s32 Cart_Velocity;
    s32 Cart_Position;
    s32 Pendulum_Angular_Velocity;
    s32 Pendulum_Angle;
} FeedbackStruct;

s16 Fixedpt_Pos_Control(s16 Ref_Pos, s16 Pend_Ang, s32 Cart_Pos)
{
    static FeedbackStruct Prev;
    static s32 Cur_I_state = 0;
    s32 Feedback_Sum = 0;
    s16 Force;
    s32 Cntrl_Err, KI_mul, KP_mul, Feedback_mul;
    s16 KTD_f[] = {-19348, -3133, 9195, 45};

    Cntrl_Err = (Ref_Pos * SP_f) - (Cart_Pos << 14);
    Cntrl_Err = Cntrl_Err >> 14;
    KI_mul = Cntrl_Err * IG_f;
    Cur_I_state = Cur_I_state + KI_mul;
    KP_mul = (Cntrl_Err * PG_f) << 6;

    s32 Pend_Ang_Velocity = ((Pend_Ang - Prev.Pendulum_Angle) << 3) * KTD_f[0];
    Prev.Pendulum_Angle = Pend_Ang;
    s32 Cart_Velocity = (Cart_Pos - Prev.Cart_Position) * KTD_f[2];
    Prev.Cart_Position = Cart_Pos;

    FeedbackStruct Feedback = {Pend_Ang_Velocity, Pend_Ang * KTD_f[1], , Cart_Velocity, Cart_Pos * KTD_f[3]};

    for (int k = 0; k < STATES; k++)
    {
        Feedback_Sum += Feedback[k];
    }
    Feedback_Sum = Feedback_Sum << 6;

    s32 PI_controller = KP_mul + Cur_I_state - Feedback_Sum;

    PI_controller = PI_controller >> 21;

    Force = PI_controller;

    return Force;
}
