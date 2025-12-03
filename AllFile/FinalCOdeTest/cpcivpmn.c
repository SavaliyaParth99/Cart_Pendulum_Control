/*------------------------------------------------------------------------------------------
 * cpcivpmn.c
 *
 * Description: Main file for controller
 * Version : 8.0
 * Date : 24-01-2022
 * Subject : Embedded Systems Project
 * Course : Embedded Systems Design, Hochschule Bremerhaven
 * ------------------------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "platform.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "xscugic.h"
#include "xil_exception.h"
#include "xscutimer.h"
#include "Pos_Cntrl_FixedPt.h"
#include "Speed_Ctrl_FixedPt.h"
#include "math.h"

#define AD5624RINTREF 0

#define AONB_REG (*(volatile unsigned int *)XPAR_APRIOIFN_0_S00_AXI_BASEADDR)
#define DIGIO_REG (*(volatile unsigned int *)(XPAR_APRIOIFN_0_S00_AXI_BASEADDR + 4))
#define ADCSTAT_REG (*(volatile unsigned int *)(XPAR_APRIOIFN_0_S00_AXI_BASEADDR + 8))
#define ADCDATA_REG (*(volatile unsigned int *)(XPAR_APRIOIFN_0_S00_AXI_BASEADDR + 12))
#define DACF_REG (*(volatile unsigned int *)(XPAR_APRIOIFN_0_S00_AXI_BASEADDR + 16))
#define IENC0_REG (*(volatile unsigned int *)(XPAR_APRIOIFN_0_S00_AXI_BASEADDR + 20))
#define IENC1_REG (*(volatile unsigned int *)(XPAR_APRIOIFN_0_S00_AXI_BASEADDR + 24))
#define IENCZZ_REG (*(volatile unsigned int *)(XPAR_APRIOIFN_0_S00_AXI_BASEADDR + 28))
// ---- Interrupt Controller -----
// interrupt controller instance
static XScuGic Intc;
// configuration instance
static XScuGic_Config *IntcConfig;
// ---- Scu Timer -----
// Private Timer Instance
static XScuTimer pTimer;
// Configuration Instance
static XScuTimer_Config *pTimerConfig;
// 100 Hz => 3333333
// 2 Hz => 166666500
#define TIMER_LOAD_VALUE 3333333
// ---- Led Display -----
static volatile unsigned int Led_Output;
static volatile unsigned int Led_Status;
static volatile unsigned int ISR_Count;
static volatile unsigned int Do_Display;
#define CBUF_LEN 64
// ---- Digital IO -----
#define Inverter_OK (DIGIO_REG & 0x00000001)
#define Inverter_Enable (DIGIO_REG & 0x00000002) >> 1
#define Left_Switch (DIGIO_REG & 0x00000005) >> 2
#define Right_Switch (DIGIO_REG & 0x00000009) >> 3
// ---- Referencing Phase Variables-----
static volatile int Left_End_Position;
static volatile int Right_End_Position;
static volatile unsigned int Cart_Ref_Speed;
static volatile unsigned int Done_Referencing;
static volatile unsigned int Start_Referencing;
static volatile int Mid_Position;
static volatile int Ref_State;
static volatile int Cart_pos_check;
static volatile int Right_End_Position_i;
static volatile int Cart_pos_check_i;
static volatile unsigned int Angle_offset;
static volatile short int Angle_offset_i;
// ---- Referencing Phase Constants ----1255555
#define RP_Idle 0
#define RP_Move_Left 1
#define RP_Pause1 2
#define RP_Move_Right 3
#define RP_Pause2 4
#define RP_Move_Center 5
#define RP_Done 6
#define Ref_Speed_Left -4915
#define Ref_Speed_Right 4915
#define Channel0 0
#define Set 1
#define Reset 0
// ---- Control Phase Variables -----
static volatile short int Cart_Ref_Position;
static volatile unsigned int Cart_Position;
static volatile unsigned int Pend_Angle;
static volatile unsigned int Start_Control;
static volatile int Control_State;
static volatile int Cart_Position_i;
static volatile short int Pend_Angle_i;
static volatile int Force;
// ---- Control Phase Constants ----
#define CP_Idle 0
#define CP_Start 1
#define CP_Stop 2
#define Zero_Speed 0
// ---- Record ----
static volatile unsigned int Record;
// ---- GUI ----
static volatile int Force_Gui;
static volatile int Cart_Position_Gui;
static volatile int Cart_Position_Prev_Gui;
static volatile int Cart_Velocity_Gui;
static volatile short int Pend_Angle_Gui;
static volatile short int Pend_Angle_Prev_Gui;
static volatile short int Pend_AngularVelocity_Gui;
static volatile short int Cart_Ref_Position_Gui;
/*
 * --------------------------------------------
 * Send data to DAC (requires 24 bit value)
 * --------------------------------------------
 */
static void DACWrite(unsigned int dval)
{
	int k;
	unsigned int retc;
	k = 0;
	DACF_REG = dval;
	do
	{
		retc = DACF_REG;
		k++;
	} while (((retc & 0x01) == 0) && (k < 500));
	// printf(" o %x written to DAC (%d retries)\n\r", dval, k);
}
static void DacOff()
{
	printf("1/2. DACoff Reset\n\r");
	DACWrite(0x0280001);
	printf("2/2. DACoff Power down\n\r");
	DACWrite(0x020001f);
}
static void DacOn()
{
	printf("1/4. DACon Reset\n\r");
	DACWrite(0x0280001);
	printf("2/4. DACon Power up\n\r");
	DACWrite(0x020000f);
#if AD5624RINTREF == 1
	printf("3/4. DACon Vref on (AD56624R)\n\r");
	DACWrite(0x0380001);
#else
	printf("3/4. *ignore* DACon Vref on command (AD56624)\n\r");
#endif
	printf("4/4. DACon LDAC setup\n\r");
	DACWrite(0x0300000);
}
/*
 * --------------------------------------------
 * Send data to DAC on specified channel
 * --------------------------------------------
 */
static void DacSet(int chan, int daval)
{
	int da_offs;
	unsigned int xval, dac_cmd;
	da_offs = 2048 + daval;
	if (da_offs > 4095)
	{
		da_offs = 4095;
	}
	else if (da_offs < 0)
	{
		da_offs = 0;
	}
	xval = da_offs & 0x0fff;
	// printf("DAC[%d]: %d (%x)\n\r", chan, xval, xval);
	// chan==4 => all dac channels
	dac_cmd = (xval & 0x0fff) << 4;
	if (chan == 0)
	{
		dac_cmd |= 0x0180000;
	}
	else if (chan == 1)
	{
		dac_cmd |= 0x0190000;
	}
	else if (chan == 2)
	{
		dac_cmd |= 0x01a0000;
	}
	else if (chan == 3)
	{
		dac_cmd |= 0x01b0000;
	}
	else if (chan == 4)
	{
		dac_cmd |= 0x01f0000;
	}
	else
	{
		printf(" *** invalid dac channel: %d\n\r", chan);
		printf(" *** data will not be send to DAC.\n\r");
		dac_cmd = 0;
	}
	if (dac_cmd != 0)
	{
		DACWrite(dac_cmd);
	}
}
/*
 * --------------------------------------------
 * Receive data from ADC (2 x 16 Bit)
 * --------------------------------------------
 */
static void ADCRead()
{
	int k;
	unsigned int retc, adcdat, adcA, adcB;
	int ai_0, ai_1;
	k = 0;
	ADCSTAT_REG = 0;
	do
	{
		retc = ADCSTAT_REG;
		k++;
	} while (((retc & 0x01) == 0) && (k < 500));
	adcdat = ADCDATA_REG;
	adcA = adcdat & 0x0FFFF;
	adcB = adcdat >> 16;
	printf(" o ADC_B: %x ADC_A: %x (%8x) [%d retries]\n\r", adcA, adcB, adcdat, k);
	ai_0 = adcA & 0x0FFF;
	if ((adcA & 0x0800) != 0)
	{
		ai_0 |= 0xFFFFF000;
	}
	ai_0 = -ai_0;
	ai_1 = adcB & 0x0FFF;
	if ((adcB & 0x0800) != 0)
	{
		ai_1 |= 0xFFFFF000;
	}
	ai_1 = -ai_1;
	printf(" o Ai_0: %d Ai_1: %d\n\r", ai_0, ai_1);
}
/*
 * --------------------------------------------
 * Set inverter enable pin in DIGIO_REG to 1
 * --------------------------------------------
 */
void Enable_Inverter()
{
	DIGIO_REG |= 0x00000002;
}
/*
 * --------------------------------------------
 * Set inverter enable pin in DIGIO_REG to 0
 * --------------------------------------------
 */
void Disable_Inverter()
{
	DIGIO_REG &= 0xFFFFFFFD;
}

/*
   * -----------------------------------------------------------------------------------------------
   * Sends Force, Cart_Position, Cart_Velocity, Pend_Angle, Pend_AngularVelocity to GUI for
   display
   * -----------------------------------------------------------------------------------------------
   */
static void SendTo_GUI()
{
	Cart_Position_Prev_Gui = Cart_Position_Gui;
	Pend_Angle_Prev_Gui = Pend_Angle_Gui;
	// IENC1_REG write => latch counters
	IENC1_REG = 0;
	Cart_Position = IENC0_REG;
	// Check if negative, 19th bit is signed
	if ((Cart_Position & 0x080000) != 0)
	{
		Cart_Position |= 0xFFF00000;
	}
	// Convert to IEEE standard
	Cart_Position_Gui = *(int *)&(Cart_Position);
	Pend_Angle = IENC1_REG;
	// Check if negative, 19th bit is signed
	if ((Pend_Angle & 0x080000) != 0)
	{
		Pend_Angle |= 0xFFF00000;
	}
	// Convert to IEEE standard
	Pend_Angle_i = *(short *)&(Pend_Angle);
	// Check if negative, 19th bit is signed
	if ((Angle_offset & 0x080000) != 0)
	{
		Angle_offset |= 0xFFF00000;
	}
	// Convert to IEEE standard
	Angle_offset_i = *(short *)&(Angle_offset);
	// Subtract angle offset
	if (Pend_Angle_i > 0)
	{
		Pend_Angle_i = Pend_Angle_i - Angle_offset_i;
	}
	else
	{
		Pend_Angle_i = Pend_Angle_i + Angle_offset_i;
	}
	Pend_Angle_Gui = (-1) * Pend_Angle_i;
	Cart_Velocity_Gui = (Cart_Position_Gui - Cart_Position_Prev_Gui);
	Pend_AngularVelocity_Gui = (Pend_Angle_Gui - Pend_Angle_Prev_Gui);
	Force_Gui = Force;
	Cart_Ref_Position_Gui = (Cart_Ref_Position);
	if (Record)
	{
		// For logging data start string with "~"
		printf("~ %d %d %d %d %d %d\n\r", Cart_Ref_Position_Gui,
			   Cart_Position_Gui, Cart_Velocity_Gui, Pend_Angle_Gui, Pend_AngularVelocity_Gui,
			   Force_Gui);
	}
	// For displaying states on GUI string with "@"
	printf("@ %d %d %d %d %d %d\n\r", Cart_Ref_Position_Gui,
		   Cart_Position_Gui, Cart_Velocity_Gui, Pend_Angle_Gui, Pend_AngularVelocity_Gui,
		   Force_Gui);
}

/*
 * ------------------------------------------------------------
 * Interrupt handler (ZYNQ private timer)
 * ------------------------------------------------------------
 */
static void TimerIntrHandler(void *CallBackRef)
{
	XScuTimer *TimerInstance = (XScuTimer *)CallBackRef;
	XScuTimer_ClearInterruptStatus(TimerInstance);
	if (Start_Referencing && !Start_Control)
	{
		// State machine for referencing phase
		switch (Ref_State)
		{
		case RP_Idle:
			Left_End_Position = 0;
			Right_End_Position = 0;
			Right_End_Position_i = 0;
			Cart_pos_check = 0;
			Cart_pos_check_i = 0;
			Done_Referencing = 0;
			Mid_Position = 0;
			if ((1 == Start_Referencing) && (1 == Inverter_OK))
			{
				// Check if left end switch is active
				if (Left_Switch)
				{
					Ref_State = RP_Pause1;
				}
				else
				{
					Ref_State = RP_Move_Left;
				}
			}
			else
			{
				printf("inverter not OK\n\r");
			}
			break;
		case RP_Move_Left:
			// IENC1_REG write => latch counters
			IENC1_REG = 0;
			Cart_Position = IENC0_REG;
			// Check if negative, 19th bit is signed
			if ((Cart_Position & 0x080000) != 0)
			{
				Cart_Position |= 0xFFF00000;
			}
			// Convert to IEEE standard
			Cart_Position_i = *(int *)&(Cart_Position);
			// Get force value from speed controller for left side movement
			Force = Speed_Ctrl_FixedPt(Ref_Speed_Left, Pend_Angle_i, Cart_Position_i);
			// Set DAC with force for controlling cart speed
			// printf("value of force: %d \n",Force);
			// DacSet(0, -149);
			// Check if left end switch is active
			if (Left_Switch)
			{
				// Set DAC for 0 speed
				DacSet(0, 0);
				Ref_State = RP_Pause1;
			}
			break;
		case RP_Pause1:
			// Reset encoder values
			IENC0_REG = 0;
			// IENC1_REG write => latch counters
			IENC1_REG = 0;
			Ref_State = RP_Move_Right;
			break;
		case RP_Move_Right:
			// IENC1_REG write => latch counters
			IENC1_REG = 0;
			Cart_Position = IENC0_REG;
			// Check if negative, 19th bit is signed
			if ((Cart_Position & 0x080000) != 0)
			{
				Cart_Position |= 0xFFF00000;
			}
			// Convert to IEEE standard
			Cart_Position_i = *(int *)&(Cart_Position);
			// Get force value from speed controller for right side movement
			Force = Speed_Ctrl_FixedPt(Ref_Speed_Right, Pend_Angle_i, Cart_Position_i);
			// Set DAC with force for controlling cart speed
			// DacSet(0, 149);
			// IENC1_REG write => latch counters
			IENC1_REG = 0;
			Right_End_Position = IENC0_REG;
			// Check if right end switch is active
			if (Right_Switch)
			{
				// Set DAC for 0 speed
				DacSet(0, 0);
				Ref_State = RP_Pause2;
			}
			break;
		case RP_Pause2:
			// IENC1_REG write => latch counters
			IENC1_REG = 0;
			// Check if negative, 19th bit is signed
			if ((Right_End_Position & 0x080000) != 0)
			{
				Right_End_Position |= 0xFFF00000;
			}
			// Convert to IEEE format
			Right_End_Position_i = *(int *)&(Right_End_Position);
			// Calculate mid position of the shaft
			Mid_Position = Right_End_Position_i / 2;
			Ref_State = RP_Move_Center;
			break;
		case RP_Move_Center:
			// IENC1_REG write => latch counters
			IENC1_REG = 0;
			Cart_Position = IENC0_REG;
			// Check if negative, 19th is signed bit
			if ((Cart_Position & 0x080000) != 0)
			{
				Cart_Position |= 0xFFF00000;
			}
			// Convert to IEEE standard
			Cart_Position_i = *(int *)&(Cart_Position);
			// Get force value from speed controller for left side movement
			Force = Speed_Ctrl_FixedPt(Ref_Speed_Left,Pend_Angle_i, Cart_Position_i);
			// Set DAC with force for controlling cart speed
			// DacSet(0, -149);
			// IENC1_REG write => latch counters
			IENC1_REG = 0;
			Cart_pos_check = IENC0_REG;
			// Check if negative, 19th is signed bit
			if ((Cart_pos_check & 0x080000) != 0)
			{
				Cart_pos_check |= 0xFFF00000;
			}
			// Convert to IEEE standard
			Cart_pos_check_i = *(int *)&(Cart_pos_check);
			// Check if cart has reached mid position
			if (Cart_pos_check_i < (Mid_Position))
			{
				Ref_State = RP_Done;
			}
			break;
		case RP_Done:
			// Set DAC for 0 cart speed
			DacSet(0, 0);
			// Update status of Done_Referencing
			Done_Referencing = 1;
			// Reset all variables
			Left_End_Position = 0;
			Right_End_Position = 0;
			Right_End_Position_i = 0;
			Cart_pos_check = 0;
			Cart_pos_check_i = 0;
			Mid_Position = 0;
			Start_Referencing = 0;
			Cart_Ref_Position = 1;
			// Get angle offset before encoder reset
			Angle_offset = IENC1_REG;
			// Reset encoder values
			IENC0_REG = 0;
			Ref_State = RP_Idle;
			printf("cart_ref_position: %d", Cart_Ref_Position);
			printf("Referencing phase complete \n\r");

			break;
		default:
			Ref_State = RP_Idle;
			break;
		}
	}
	// State machine for position control
	else if (Start_Control)
	{
		// Check if left end or right end switch is active
		if ((Left_Switch) || (Right_Switch))
		{
			// Set DAC for 0 speed
			DacSet(0, 0);
			Disable_Inverter();
			printf("left / right switch active, cannot start control phase\n\r");
		}
		else if (!Inverter_OK)
		{
			printf("inverter not OK\n\r");
			// Set DAC for 0 speed
			DacSet(0, 0);
			Disable_Inverter();
		} // Check if referencing is already complete
		else if (Done_Referencing)
		{
			// State machine for cart position control
			switch (Control_State)
			{
			case CP_Idle:
				// Reset variables
				Cart_Position = 0;
				Cart_Position_i = 0;
				Pend_Angle = 0;
				Pend_Angle_i = 0;
				Control_State = CP_Start;
				break;
			case CP_Start:
				if (!Start_Control)
				{
					// Go to CP_Stop stop controller requested
					Control_State = CP_Stop;
					break;
				}
				// IENC1_REG write => latch counters
				IENC1_REG = 0;
				Cart_Position = IENC0_REG;
				// Check if negative, 19th bit is signed
				if ((Cart_Position & 0x080000) != 0)
				{
					Cart_Position |= 0xFFF00000;
				}
				// Convert to IEEE standard
				Cart_Position_i = *(int *)&(Cart_Position);
				Pend_Angle = IENC1_REG;
				// Check if negative. 19th bit is signed
				if ((Pend_Angle & 0x080000) != 0)
				{
					Pend_Angle |= 0xFFF00000;
				}
				// Convert to IEEE standard
				Pend_Angle_i = (*(short *)&(Pend_Angle)) * (-1);
				// printf("pend_ang_i %d %d \n\r",Pend_Angle,Pend_Angle_i);
				// Get force value from cart position controller
				Force = Pos_Cntrl_FixedPt(Cart_Ref_Position, Pend_Angle_i,Cart_Position_i);
				// Set DAC with force for controlling cart position
				// printf("value of : %d \n",Force);
				// DacSet(0, Force);
				break;
			case CP_Stop:
				DacSet(0, 0);
				Disable_Inverter();
				Start_Control = 0;
				Control_State = CP_Idle;
				break;
			default:
				Control_State = CP_Idle;
				break;
			}
		}
	}
	Do_Display = 1;
	ISR_Count++;
}
/*
 * -----------------------------
 * Main function
 * -----------------------------
 */
int main()
{
	int terminate, isr_run, regn, cnt, repeatc, dacchan, idata;
	unsigned int xdata, pos_0, pos_1, pos_z;
	char cbuf[CBUF_LEN], *chp;
	int pos0i, pos0i_last, pos1i, pos1i_last;
	double speed0, speed1;
	init_platform();
	printf("\n\n--- Cart Pendulum Control System ---\n\r");
	terminate = 0;
	ISR_Count = 0;
	Do_Display = 0;
	isr_run = 0;
	Led_Output = 0;
	Led_Status = 0;
	pos0i = 0;
	pos0i_last = 0;
	pos1i = 0;
	pos1i_last = 0;
	DacOn();
	DIGIO_REG = 0;
	DacSet(0, 0);
	DacSet(1, 0);
	DacSet(2, 0);
	DacSet(3, 0);
	Left_End_Position = 0;
	Right_End_Position = 0;
	Done_Referencing = 0;
	Start_Referencing = 0;
	Mid_Position = 0;
	Ref_State = RP_Idle;
	Cart_Ref_Position = 0;
	Cart_Position = 0;
	Cart_Position_i = 0;
	Pend_Angle = 0;
	Pend_Angle_i = 0;
	Start_Control = 0;
	Control_State = CP_Idle;
	Record = 0;
	Force_Gui = 0;
	Cart_Position_Gui = 0;
	Cart_Position_Prev_Gui = 0;
	Cart_Velocity_Gui = 0;
	Pend_Angle_Gui = 0;
	Pend_Angle_Prev_Gui = 0;
	Pend_AngularVelocity_Gui = 0;
	printf(" * initialize exceptions...\n\r");
	Xil_ExceptionInit();
	printf(" * lookup config GIC...\n\r");
	IntcConfig = XScuGic_LookupConfig(XPAR_SCUGIC_0_DEVICE_ID);
	printf(" * initialize GIC...\n\r");
	XScuGic_CfgInitialize(&Intc, IntcConfig, IntcConfig->CpuBaseAddress);
	// Connect the interrupt controller interrupt handler to the hardware
	printf(" * connect interrupt controller handler...\n\r");
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT,
								 (Xil_ExceptionHandler)XScuGic_InterruptHandler, &Intc);
	printf(" * lookup config scu timer...\n\r");
	pTimerConfig = XScuTimer_LookupConfig(XPAR_XSCUTIMER_0_DEVICE_ID);
	printf(" * initialize scu timer...\n\r");
	XScuTimer_CfgInitialize(&pTimer, pTimerConfig, pTimerConfig->BaseAddr);
	printf(" * Enable Auto reload mode...\n\r");
	XScuTimer_EnableAutoReload(&pTimer);
	printf(" * load scu timer...\n\r");
	XScuTimer_LoadTimer(&pTimer, TIMER_LOAD_VALUE);
	printf(" * set up timer interrupt...\n\r");
	XScuGic_Connect(&Intc, XPAR_SCUTIMER_INTR, (Xil_ExceptionHandler)TimerIntrHandler,
					(void *)&pTimer);
	printf(" * enable interrupt for timer at GIC...\n\r");
	XScuGic_Enable(&Intc, XPAR_SCUTIMER_INTR);
	printf(" * enable interrupt on timer...\n\r");
	XScuTimer_EnableInterrupt(&pTimer);
	// Enable interrupts in the Processor.
	printf(" * enable processor interrupts...\n\r");
	Xil_ExceptionEnable();
	do
	{
		print(">> ");
		fflush(stdout);
		fgets(cbuf, CBUF_LEN, stdin);
		cbuf[CBUF_LEN - 1] = '\0';
		chp = cbuf;
		do
		{
			if (*chp == '\n' || *chp == '\a')
			{
				*chp = '\0';
			}
		} while (*chp++ != '\0');
		printf("\r");
		fflush(stdout);
		if (!strncmp(cbuf, "exit", 4))
		{
			terminate = 1;
		}
		else if (!strncmp(cbuf, "isr", 3))
		{
			if (isr_run == 0)
			{
				// start scu timer
				printf(" * start timer...\n\r");
				XScuTimer_Start(&pTimer);
				isr_run = 1;
			}
			else
			{
				// stop scu timer
				printf(" * stop timer...\n\r");
				XScuTimer_Stop(&pTimer);
				isr_run = 0;
			}
			printf("ISR count: %d\n\r", ISR_Count);
		}
		else if (cbuf[0] == 'x')
		{
			printf("Emergency shutdown!\n\r");
			DacSet(0, 0);
			DIGIO_REG = 0;
			printf("...done.\n\r");
		}
		else if (!strncmp(cbuf, "dacoff", 6))
		{
			printf("Shutdown DAC...\n\r");
			DacOff();
			printf("...done.\n\r");
		}
		else if (!strncmp(cbuf, "dacon", 5))
		{
			printf("Enable DAC...\n\r");
			DacOn();
			printf("...done.\n\r");
		}
		else if (!strncmp(cbuf, "dac", 3))
		{
			if (sscanf(&cbuf[3], "%d %d", &dacchan, &idata) != 2)
			{
				printf(" *** dac channel/value conversion error\n\r");
			}
			else
			{
				DacSet(dacchan, idata);
			}
		}
		else if (!strncmp(cbuf, "adc", 3))
		{
			ADCRead();
		}
		else if (!strncmp(cbuf, "pclr", 4))
		{
			// IENC0_REG write => clear all position counters
			IENC0_REG = 0;
			printf(" Position registers cleared.\n\r");
		}
		else if (!strncmp(cbuf, "pos", 3))
		{
			if (sscanf(&cbuf[3], "%d", &repeatc) != 1)
			{
				printf(" *** repeat counter conversion error\n\r");
			}
			else
			{
				cnt = 0;
				do
				{
					Do_Display = 0;
					// IENC1_REG write => latch counters
					IENC1_REG = 0;
					pos_0 = IENC0_REG;
					pos_1 = IENC1_REG;
					pos_z = IENCZZ_REG;
					printf(" pos_0: %8x pos_1: %8x (zz: %x)\n", pos_0, pos_1, pos_z);
					if ((pos_0 & 0x080000) != 0)
					{
						pos_0 |= 0xFFF00000;
					}
					pos0i_last = pos0i;
					pos0i = *(int *)&pos_0;
					speed0 = 0.05859375 * (pos0i - pos0i_last);
					if ((pos_1 & 0x080000) != 0)
					{
						pos_1 |= 0xFFF00000;
					}
					pos1i_last = pos1i;
					pos1i = *(int *)&pos_1;
					speed1 = 0.05859375 * (pos1i - pos1i_last);
					printf("pos0: %7d (%7d) speed0: %12.4f rpm\n", pos0i, pos0i_last, speed0);
					printf("pos1: %7d (%7d) speed1: %12.4f rpm\n", pos1i, pos1i_last, speed1);
					cnt++;
					if (isr_run == 1)
					{
						while (Do_Display == 0)
							;
					}
				} while ((isr_run == 1) && (cnt < repeatc));
			}
			printf("...pos done.\n");
		}
		else if (cbuf[0] == 'r')
		{
			xdata = AONB_REG;
			printf(" R0 (OBP): %8x\n\r", xdata);
			xdata = DIGIO_REG;
			printf(" R1 (DIGIO): %8x\n\r", xdata);
			xdata = ADCSTAT_REG;
			printf(" R2 (ADCSTAT): %8x\n\r", xdata);
			xdata = ADCDATA_REG;
			printf(" R3 (ADCDATA): %8x\n\r", xdata);
			xdata = DACF_REG;
			printf(" R4 (DACF): %8x\n\r", xdata);
			xdata = IENC0_REG;
			printf(" R5 (IENC0): %8x\n\r", xdata);
			xdata = IENC1_REG;
			printf(" R6 (IENC1): %8x\n\r", xdata);
			xdata = IENCZZ_REG;
			printf(" R7 (IENCZZ): %8x\n\r", xdata);
		}
		else if (cbuf[0] == 'w')
		{
			if (sscanf(&cbuf[1], "%d %x", &regn, &xdata) != 2)
			{
				printf(" *** conversion error\n\r");
			}
			else
			{
				switch (regn)
				{
				case 0:
					AONB_REG = xdata;
					printf(" %8x -> R0 (OBP)\n\r", xdata);
					break;
				case 1:
					DIGIO_REG = xdata;
					printf(" %8x -> R1 (DIGIO)\n\r", xdata);
					break;
				case 2:
					ADCSTAT_REG = xdata;
					printf(" %8x -> R2 (ADCSTAT)\n\r", xdata);
					break;
				case 3:
					ADCDATA_REG = xdata;
					printf(" %8x -> R3 (ADCDATA)\n\r", xdata);
					break;
				case 4:
					DACF_REG = xdata;
					printf(" %8x -> R4 (DACF)\n\r", xdata);
					break;
				case 5:
					IENC0_REG = xdata;
					printf(" %8x -> R5 (IENC0)\n\r", xdata);
					break;
				case 6:
					IENC1_REG = xdata;
					printf(" %8x -> R6 (IENC1)\n\r", xdata);
					break;
				case 7:
					IENCZZ_REG = xdata;
					printf(" %8x -> R7 (IENCZZ)\n\r", xdata);
					break;
				default:
					printf("*** illegal regnum\n\r");
				}
			}
		} // Check if command to start referencing is received
		else if (!strncmp(cbuf, "Ref", 3))
		{
			if (Inverter_OK)
			{
				Left_End_Position = 0;
				Right_End_Position = 0;
				Mid_Position = 0;
				if (!Start_Control)
				{
					Done_Referencing = 0;
					printf(" Starting referencing phase \n\r");
					Start_Referencing = 1;
					Enable_Inverter();
				}
				if (!isr_run)
				{
					XScuTimer_Start(&pTimer);
					isr_run = 1;
				}
			}
		}
		// Check if command to start cart position control is received
		if (!strncmp(cbuf, "ctrl", 4))
		{

			printf("cart_ref_position: (main): %d", Cart_Ref_Position);
			if (Cart_Ref_Position != 1)
			{
				printf("cart_ref_position: (loop): %d", Cart_Ref_Position);
				printf(" reference cart position input error\n\r");
			}
			else
			{
				if (!Inverter_OK)
				{
					printf(" inverter not OK\n\r");
				}
				else if (!Done_Referencing)
				{
					printf("referencing not done\n\r");
				}
				else
				{
					printf("Starting cart position control \n\r");
					Enable_Inverter();
					// To start position controller in interrupt handler
					Start_Control = 1;
					// To send data to GUI
					SendTo_GUI();
				}
			}
		} // Check if command to stop cart position control is received
		if (!strncmp(cbuf, "stop", 4))
		{
			DacSet(0, 0);
			Disable_Inverter();
			Start_Control = 0;
			// Stop logging data if already on
			printf("###\n\r");
		} // Check if command to start data logging is received
		if (!strncmp(cbuf, "Recordon", 8))
		{
			// To start sending data to GUI for logging
			Record = 1;
			printf("Data logging started");
		} // Check if command to stop data logging is received
		else if (!strncmp(cbuf, "Recordoff", 9))
		{
			Record = 0;
			// Stop logging data
			printf("###\n\r");
		}
		else if (cbuf[0] == 'p')
		{
			SendTo_GUI();
		}
	} while (terminate == 0);
	printf("shutting down...\n\r");
	XScuTimer_Stop(&pTimer);
	Xil_ExceptionDisable();
	XScuTimer_DisableInterrupt(&pTimer);
	XScuGic_Disable(&Intc, XPAR_SCUTIMER_INTR);
	DacSet(0, 0);
	DIGIO_REG = 0;
	AONB_REG = 0x0;
	printf("Thank you for using PrIO Full Tests.\n\r");
	cleanup_platform();
	return 0;
}
