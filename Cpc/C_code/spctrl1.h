/*
 * spctrl.h
 *
 *  Created on: 13-DEZ-2011
 *      Author: mueller
 */

#ifndef SPCTRL_H_
#define SPCTRL_H_

#include "xil_types.h"

/* ---- from MATLAB: ------- */
#define KP_D  358
#define KI_D  229
/* ------------------------- */


s16 spctrl(s16 reset, s16 state1, s32 state2, s16 state3);

#endif /* SPCTRL_H_ */

