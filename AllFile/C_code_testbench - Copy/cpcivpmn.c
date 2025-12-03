/*
 * cpcivpmn.c: Process IO Full Tests
 */

#include <stdio.h>
#include <string.h>
#include "platform.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "xscugic.h"
#include "xil_exception.h"
#include "xscutimer.h"

#define AD5624RINTREF 0

#define AONB_REG (*(volatile unsigned int *)XPAR_APRIOIFN_0_S00_AXI_BASEADDR)
#define DIGIO_REG (*(volatile unsigned int *)(XPAR_APRIOIFN_0_S00_AXI_BASEADDR + 4))
#define ADCSTAT_REG (*(volatile unsigned int *)(XPAR_APRIOIFN_0_S00_AXI_BASEADDR + 8))
#define ADCDATA_REG (*(volatile unsigned int *)(XPAR_APRIOIFN_0_S00_AXI_BASEADDR + 12))
#define DACF_REG (*(volatile unsigned int *)(XPAR_APRIOIFN_0_S00_AXI_BASEADDR + 16))
#define IENC0_REG (*(volatile unsigned int *)(XPAR_APRIOIFN_0_S00_AXI_BASEADDR + 20))
#define IENC1_REG (*(volatile unsigned int *)(XPAR_APRIOIFN_0_S00_AXI_BASEADDR + 24))
#define IENCZZ_REG (*(volatile unsigned int *)(XPAR_APRIOIFN_0_S00_AXI_BASEADDR + 28))

// ---- interrupt controller -----
static XScuGic Intc;               // interrupt controller instance
static XScuGic_Config *IntcConfig; // configuration instance

// ---- scu timer -----
static XScuTimer pTimer;               // private Timer instance
static XScuTimer_Config *pTimerConfig; // configuration instance
// 100Hz => 3333333
#define TIMER_LOAD_VALUE 3333333 // should be 2 Hz(***changed***)

static volatile unsigned int Led_Output;
static volatile unsigned int Led_Status;
static volatile unsigned int ISR_Count;
static volatile unsigned int Do_Display;

// DIN bits
#define AMPL_OK (DIGIO_REG & 0x01)
#define LEFT_SWITCH (DIGIO_REG & 0x05) >> 2
#define RIGHT_SWITCH (DIGIO_REG & 0x09) >> 3

// DOUT bits
#define AMPL_ENABLE (DIGIO_REG & 0x03) >> 1

#define CBUF_LEN 64

// ------ Referencing Mode -------
static volatile unsigned int Start_Ref;
static volatile int L_End_Pos;
static volatile int R_End_Pos;
static volatile int R_End_Pos_i;
static volatile int Mid_Pos;
static volatile int Ref_Mode;
static volatile int Check_Cart_Pos;
static volatile int Check_Cart_Pos_e;
static volatile unsigned int Done_Ref;
static volatile unsigned int Start_Control;

#define Ref_Init 0
#define Ref_Move_L 1
#define Ref_Pause_L 2
#define Ref_Move_R 3
#define Ref_Pause_R 4
#define Ref_Move_C 5
#define Ref_Done 6
#define Check_Ampl 3

/*
 * Send data to DAC (requires 24 bit value)
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

    //printf("DAC[%d]: %d (%x)\n\r", chan, xval, xval);
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
 * Receive data from ADC (2 x 16 Bit)
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
    printf(" o ADC_B: %x  ADC_A: %x  (%8x) [%d retries]\n\r", adcA, adcB, adcdat, k);
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
    printf(" o Ai_0: %d  Ai_1: %d\n\r", ai_0, ai_1);
}

/*
 * Set inverter enable pin in DIGIO_REG to 1
 */
void Inverter_On()
{
    DIGIO_REG |= 0x02;
//    printf("digio: %d\n\r",DIGIO_REG);

}

/*
 * Set inverter enable pin in DIGIO_REG to 0
 */
void Inverter_Off()
{
    DIGIO_REG &= 0xFFFFFFFD;
    //printf("digio: %d\n\r",DIGIO_REG);
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

    /*
    Led_Output &= 0x03F0;
    switch (Led_Status) {
    case 0:
        Led_Output |= 0x01;
        Led_Status++;
        break;
    case 1:
        Led_Output |= 0x02;
        Led_Status++;
        break;
    case 2:
        Led_Output |= 0x04;
        Led_Status++;
        break;
    default:
        Led_Output |= 0x08;
        Led_Status = 0;
    }
    AONB_REG = Led_Output;*/

    if (Start_Ref)
    {
        switch (Ref_Mode)
        {
        case Ref_Init:
            L_End_Pos = 0;
            R_End_Pos = 0;
            R_End_Pos_i = 0;
            Check_Cart_Pos = 0;
            Check_Cart_Pos_e = 0;
            Done_Ref = 0;
            Mid_Pos = 0;
            //printf("left and right switch: %d %d\n\r",LEFT_SWITCH,RIGHT_SWITCH);
            if ((Start_Ref) && (AMPL_OK))
            {
                if (LEFT_SWITCH)
                {
                    Ref_Mode = Ref_Pause_L;
                }
                else
                {
                    Ref_Mode = Ref_Move_L;
                }
            }
            else
            {
                printf(" *** inverter not OK\n\r");
            }

        case Ref_Move_L:

            DacSet(0, -120);
            IENC1_REG = 0;   // IENC1_REG write => latch counters
            IENC0_REG = 0;
            //printf(" ***IENC0_REG, MoveLeft : %7d \n\r", IENC0_REG);
            if (LEFT_SWITCH)
            {
                DacSet(0, 0);
                Ref_Mode = Ref_Pause_L;
            }
            //printf("left and right switch: %d %d\n\r",LEFT_SWITCH,RIGHT_SWITCH);
            break;

        case Ref_Pause_L:
            IENC0_REG = 0;
            IENC1_REG = 0; // IENC1_REG write => latch counters
            // printf(" ***IENC0_REG, PauseLeft : %7d \n\r",IENC0_REG);
            Ref_Mode = Ref_Move_R;
            break;

        case Ref_Move_R:
            DacSet(0, 120);
            IENC1_REG = 0;  // IENC1_REG write => latch counters
            R_End_Pos = IENC0_REG;
            // printf(" ***IENC0_REG MoveRight: %7d \n\r",R_End_Pos);
            if (RIGHT_SWITCH)
            {
                DacSet(0, 0);
                Ref_Mode = Ref_Pause_R;
            }
            break;

        case Ref_Pause_R:
            IENC1_REG = 0; // IENC1_REG write => latch counters
            // R_End_Pos = IENC0_REG;
            // printf(" ***IENC0_REG PauseRight: %7d \n\r",R_End_Pos);
            if ((R_End_Pos & 0x080000) != 0)
            {
                R_End_Pos |= 0xFFF00000;
            }
            R_End_Pos_i = *(int *)&(R_End_Pos);
            printf(" ***IENC0_REG PauseRightAftercalc: %7d \n\r", R_End_Pos_i);
            Mid_Pos = R_End_Pos_i / 2;
            //printf(" ***Mid_Pos : %7d \n\r",Mid_Pos);
            //IENC0_REG = 0;
            Ref_Mode = Ref_Move_C;
            break;

        case Ref_Move_C:
            DacSet(0, -120);
            IENC1_REG = 0; // IENC1_REG write => latch counters
            Check_Cart_Pos = IENC0_REG;
            if ((Check_Cart_Pos & 0x080000) != 0)
            {
                Check_Cart_Pos |= 0xFFF00000;
            }
            Check_Cart_Pos_e = *(int *)&(Check_Cart_Pos);

            //printf(" ** Check_Cart_Pos_e MoveCenter: %7d \n\r",Check_Cart_Pos_e);
            if (Check_Cart_Pos_e < (Mid_Pos))
            //if (Check_Cart_Pos_e == Mid_Pos)
            {
                // printf(" **Mid_Pos MoveCenter: %7d \n\r",Mid_Pos);

                Ref_Mode = Ref_Done;
            }

            break;

        case Ref_Done:
            DacSet(0, 0);
            Done_Ref = 1;
            L_End_Pos = 0;
            R_End_Pos = 0;
            R_End_Pos_i = 0;
            Check_Cart_Pos = 0;
            Check_Cart_Pos_e = 0;
            Mid_Pos = 0;
            Start_Ref = 0;
            IENC0_REG = 0;
            Ref_Mode = Ref_Init;
            printf(" ***Initial phase completed\n\r");
            break;

        default:
            Ref_Mode = Ref_Init;
            break;
        }
    }

    Do_Display = 1;
    ISR_Count++;
}

int main()
{
    int terminate, isr_run, regn, cnt, repeatc, dacchan, idata;
    // unsigned int xdata, pos_0, pos_1, pos_z;
    unsigned int xdata, pos_0, pos_1;
    char cbuf[CBUF_LEN], *chp;
    int pos0i, pos0i_last, pos1i, pos1i_last;
    double speed0, speed1;

    init_platform();
    printf("\n\n--- PrIO Full Tests V0.1a W21/22 ---\n\r");
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

    L_End_Pos = 0;
    R_End_Pos = 0;
    Done_Ref = 0;
    Start_Ref = 0;
    Mid_Pos = 0;
    Ref_Mode = Ref_Init;
    Start_Control = 0;

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
        print(">> "); fflush(stdout);
        fgets(cbuf, CBUF_LEN, stdin);
        cbuf[CBUF_LEN - 1] = '\0';
        chp = cbuf;
        do
        {
            if (*chp == '\n' || *chp == '\a')
            {
                *chp = '\0';
            }
        }
        while (*chp++ != '\0');
        printf("\r"); fflush(stdout);
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
            IENC0_REG = 0; // IENC0_REG write => clear all position counters
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
                    IENC1_REG = 0; // IENC1_REG write => latch counters
                    pos_0 = IENC0_REG;
                    pos_1 = IENC1_REG;
                    //printf("pos_0: %7d\n\r",pos_0);
                    //pos_z = IENCZZ_REG;
                    //printf(" pos_0: %8x  pos_1: %8x  (zz: %x)\n", pos_0, pos_1, pos_z);
                    if ((pos_0 & 0x080000) != 0)
                    {
                        pos_0 |= 0xFFF00000;
                    }
                    //printf("pos_0: %7d\n\r",pos_0);
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
                    printf("pos0: %7d (%7d)  speed0: %12.4f rpm\n", pos0i, pos0i_last, speed0);
                    printf("pos1: %7d (%7d)  speed1: %12.4f rpm\n", pos1i, pos1i_last, speed1);

                    // printf("~%d/%d@%f<%d:%f#%d\n",0,pos0i,speed0,pos1i,speed1,0);
                    cnt++;
                    if (isr_run == 1)
                    {
                        while (Do_Display == 0);
                    }
                } while ((isr_run == 1) && (cnt < repeatc));
            }
            printf("...pos done.\n");
        }
        else if (cbuf[0] == 'r')
        {
            xdata = AONB_REG;
            printf(" R0     (OBP): %8x\n\r", xdata);
            xdata = DIGIO_REG;
            printf(" R1   (DIGIO): %8x\n\r", xdata);
            xdata = ADCSTAT_REG;
            printf(" R2 (ADCSTAT): %8x\n\r", xdata);
            xdata = ADCDATA_REG;
            printf(" R3 (ADCDATA): %8x\n\r", xdata);
            xdata = DACF_REG;
            printf(" R4    (DACF): %8x\n\r", xdata);
            xdata = IENC0_REG;
            printf(" R5   (IENC0): %8x\n\r", xdata);
            xdata = IENC1_REG;
            printf(" R6   (IENC1): %8x\n\r", xdata);
            xdata = IENCZZ_REG;
            printf(" R7  (IENCZZ): %8x\n\r", xdata);
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
        }
        //------------Added--------------
        else if (!strncmp(cbuf, "Ref", 3))
        {
            if (AMPL_OK)
            {
                L_End_Pos = 0;
                R_End_Pos = 0;
                Mid_Pos = 0;
                if (!Start_Control)
                {
                    Done_Ref = 0;
                    printf(" ***Starting initial phase... \n\r");
                    Start_Ref = 1;
                    Inverter_On();
                    //printf("digio: %d\n\r",DIGIO_REG);
                }

                if (AMPL_ENABLE)
                {
                    printf(" ***Turn on the Amplifire! \n\r");
                    Start_Ref = 0;

                }
                if (isr_run == 0)
                {
                    // start scu timer
                    XScuTimer_Start(&pTimer);
                    isr_run = 1;
                }
            }
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
