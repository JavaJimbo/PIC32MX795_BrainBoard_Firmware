/**********************************************************************************
 * main.c
 *
 * Compiled for PIC32MX795 XC32 compiler version 1.30
 * Project: Robotnik Brain Board
 * Created from USB_Robotnik - USB routines removed
 * 
 * 7-20-19 Tested: USB, SPI_CS, PWM, DIR, DISABLE, AD, SD Card, Encoder counter and direction inputs.
 * 
 ***********************************************************************************/
#include <xc.h>
#include "HardwareProfile.h"
#include "uart2.h"
#include "Delay.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** CONFIGURATION **************************************************/
#pragma config UPLLEN   = ON        // USB PLL Enabled
#pragma config FPLLMUL  = MUL_20        // PLL Multiplier
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = HS            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx2      // ICE/ICD Comm Channel Select

/** I N C L U D E S **********************************************************/

#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "HardwareProfile.h"
#include <ctype.h>

#include "FSIO.h"
#include "Delay.h"
#include "Defs.h"

#define EncoderOne TMR1
#define EncoderTwo TMR4
#define EncoderThree TMR3
#define EncoderFour TMR5

#define EncoderOneDir PORTEbits.RE9
#define EncoderTwoDir PORTEbits.RE5
#define EncoderThreeDir PORTEbits.RE7
#define EncoderFourDir PORTEbits.RE8

#define DIR1 LATGbits.LATG12
#define DIR2 LATDbits.LATD13
#define DIR3 LATDbits.LATD11
#define DIR4 LATAbits.LATA5

#define PWM_DISABLE LATFbits.LATF1
#define PWM_SPI_CS LATDbits.LATD7

#define TEST_OUT LATEbits.LATE3
#define LED_OUT LATEbits.LATE6

// UART FOR PC SERIAL PORT
#define HOSTuart UART2
#define HOSTbits U2STAbits
#define HOST_VECTOR _UART_2_VECTOR

#define MIDIuart UART1
#define MIDIbits U1STAbits
#define MIDI_VECTOR _UART_1_VECTOR


#define CR 13
#define LF 10
#define BACKSPACE 8
#define SPACE 32
#define ESCAPE 27
#define RIGHT_ARROW 67
#define LEFT_ARROW 68
#define UP_ARROW 65
#define DOWN_ARROW 66

#define false FALSE
#define true TRUE
/** V A R I A B L E S ********************************************************/
// #define MAXBUFFER CDC_DATA_IN_EP_SIZE
#define MAXBUFFER 64
unsigned char HOSTRxBuffer[MAXBUFFER+1];
unsigned char HOSTRxBufferFull = false;
//unsigned char MemoryBufferFull = false;
unsigned char HOSTTxBuffer[MAXBUFFER+1];
unsigned char HOSTTxBufferFull = false;
//unsigned char tempBuffer[MAXBUFFER+1];

unsigned char outMessage[] = "\rJust putzing around";

// short MIDItimeout = 0;


/** P R I V A T E  P R O T O T Y P E S ***************************************/
static void InitializeSystem(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void UserInit(void);
void InitializeUSART(void);
void putcUSART(char c);
unsigned char getcUSART ();

//unsigned char command = 0;
//unsigned char mode = 0;
//unsigned short PortBRead = 0x0000;
//unsigned char PortBflag = false;
unsigned char SendFlag = false;
unsigned char EnableAD = false;

void ConfigAd(void);

enum
{
    STANDBY = 0,
    RECORD,
    PLAY,
    PAUSE_RECORD,
    PAUSE_PLAY,
    HALT
};

short time, seconds = 0, minutes = 0, hundredths = 0;

unsigned char TestMode = false;

#define MAXPOTS 4
unsigned short ADresult[MAXPOTS];

int main(void)
{   
unsigned short i = 0, j = 0, numBytes;
FSFILE *filePtr;
char filename[] = "TestFile.txt";
char MessageOut[] = "\rLife is a beach!";
unsigned char ch;    
short length = 0;
short counter = 0;


    InitializeSystem();
    
    DelayMs(100);    
    printf("\rTesting SD card - wait for Media Detect...");
    while (!MDD_MediaDetect());     // Wait for SD detect to go low    
    printf("\rInitializing SD card...");
    while (!FSInit());
    printf("\rOpening test file to read...");
    filePtr = FSfopen(filename, FS_READ);
    if (filePtr==NULL) printf("Error: could not open %s", filename);
    else
    {
        printf("\rSuccess! Opened %s. Reading data\r\r", filename);    
        do {                      
            numBytes = FSfread(&ch, 1, 1, filePtr);
            putchar(ch); 
        } while (!FSfeof(filePtr)); 
        printf("\rEND OF FILE");
        printf("\rClosing file");
        FSfclose(filePtr); 
    }   
    DelayMs(10);        // Initialize SD card       

    printf("\rOpening test file to write:");
    filePtr = FSfopen(filename, FS_APPEND);    
    
    if (filePtr==NULL) printf("Error: could not open %s", filename);
    else
    {
        printf("\rSuccess! Opened %s. writing data\r", filename);    
        length = strlen(MessageOut);
        numBytes = FSfwrite(MessageOut, 1, length, filePtr);
        printf("\rLength of message: %d, bytes written: %d", length, numBytes);
        printf("\rClosing file");
        FSfclose(filePtr); 
    }   
    
    while(1);
    
    DelayMs(100);
    printf("\rTesting encoders\r");
    while(1)    
    {
        if (EnableAD)
        {
            mAD1IntEnable(INT_ENABLED);
            EnableAD = false;
            // printf("\rAD0 = %d, AD1 = %d, AD3 = %d AD4 = %d", ADresult[0], ADresult[1], ADresult[2], ADresult[3]);            
            T1CONbits.TON = 0; 
            DelayMs(10);            
            
            if (EncoderOneDir) printf("\r#%d: Enc Dir1: HIGH, ", counter++);
            else printf("\r#%d: Enc Dir1: LOW, ", counter++);
            
            if (EncoderTwoDir) printf("Enc Dir2: HIGH, ");
            else printf("Enc Dir2: LOW, ");
            
            if (EncoderThreeDir) printf("Enc Dir3: HIGH, ");
            else printf("Enc Dir3: LOW, ");
            
            if (EncoderFourDir) printf("Enc Dir4: HIGH, ");
            else printf("Enc Dir4: LOW, ");                        
            
            T1CONbits.TON = 1; // Let her rip
            DelayMs(10);
        }
        
        DelayMs(2);
        if (HOSTRxBufferFull)
        {
            HOSTRxBufferFull = false;
            printf("\rReceived: %s", HOSTRxBuffer);            
        }
    }

    DelayMs(200);
}//end main


void InitializeSystem(void) 
{
	unsigned char i; 
    
    SYSTEMConfigPerformance(80000000);
    
    // Turn off JTAG so we get the pins back
    mJTAGPortEnable(false);
    
    ConfigAd();
    
    // Set up timers as counters
    T1CON = 0x00;
    T1CONbits.TCS = 1; // Use external counter as input source (motor encoder)
    T1CONbits.TCKPS1 = 0; // 1:1 Prescaler
    T1CONbits.TCKPS0 = 0;
    T1CONbits.TSYNC = 1;
    PR1 = 0xFFFF;
    T1CONbits.TON = 1; // Let her rip 

    T3CON = 0x00;
    T3CONbits.TCS = 1; // Use external counter as input source (motor encoder)
    T3CONbits.TCKPS2 = 0; // 1:1 Prescaler
    T3CONbits.TCKPS1 = 0;
    T3CONbits.TCKPS0 = 0;
    PR3 = 0xFFFF;
    T3CONbits.TON = 1; // Let her rip 

    T4CON = 0x00;
    T4CONbits.TCS = 1; // Use external counter as input source (motor encoder)
    T4CONbits.TCKPS2 = 0; // 1:1 Prescaler
    T4CONbits.TCKPS1 = 0;
    T4CONbits.TCKPS0 = 0;
    T4CONbits.T32 = 0; // TMRx and TMRy form separate 16-bit timers
    PR4 = 0xFFFF;
    T4CONbits.TON = 1; // Let her rip 

    T5CON = 0x00;
    T5CONbits.TCS = 1; // Use external counter as input source (motor encoder)
    T5CONbits.TCKPS2 = 0; // 1:1 Prescaler
    T5CONbits.TCKPS1 = 0;
    T5CONbits.TCKPS0 = 0;
    PR5 = 0xFFFF;
    T5CONbits.TON = 1; // Let her rip     

    // Set up PWM OC2
    OC1CON = 0x00;
    OC1CONbits.OC32 = 0; // 16 bit PWM
    OC1CONbits.ON = 1; // Turn on PWM
    OC1CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC1CONbits.OCM2 = 1; // PWM runMode enabled, no fault pin
    OC1CONbits.OCM1 = 1;
    OC1CONbits.OCM0 = 0;
    OC1RS = 3500;

    // Set up PWM OC2
    OC2CON = 0x00;
    OC2CONbits.OC32 = 0; // 16 bit PWM
    OC2CONbits.ON = 1; // Turn on PWM
    OC2CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC2CONbits.OCM2 = 1; // PWM runMode enabled, no fault pin
    OC2CONbits.OCM1 = 1;
    OC2CONbits.OCM0 = 0;
    OC2RS = 3000;

    // Set up PWM OC3
    OC3CON = 0x00;
    OC3CONbits.OC32 = 0; // 16 bit PWM
    OC3CONbits.ON = 1; // Turn on PWM
    OC3CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC3CONbits.OCM2 = 1; // PWM runMode enabled, no fault pin
    OC3CONbits.OCM1 = 1;
    OC3CONbits.OCM0 = 0;
    OC3RS = 2000;

    // Set up PWM OC4 on D6 on the Olimex 220 board:
    OC4CON = 0x00;
    OC4CONbits.OC32 = 0; // 16 bit PWM
    OC4CONbits.ON = 1; // Turn on PWM
    OC4CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC4CONbits.OCM2 = 1; // PWM runMode enabled, no fault pin
    OC4CONbits.OCM1 = 1;
    OC4CONbits.OCM0 = 0;
    OC4RS = 1000;
    
    // Set up PWM OC5
    OC5CON = 0x00;
    OC5CONbits.OC32 = 0; // 16 bit PWM
    OC5CONbits.ON = 1; // Turn on PWM
    OC5CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC5CONbits.OCM2 = 1; // PWM runMode enabled, no fault pin
    OC5CONbits.OCM1 = 1;
    OC5CONbits.OCM0 = 0;
    OC5RS = 700;
    
    
    #define SYS_FREQ 80000000    
    
    // Set up HOST UART    
    UARTConfigure(HOSTuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(HOSTuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(HOSTuart, SYS_FREQ, 921600);
    UARTEnable(HOSTuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure HOST UART Interrupts
    INTEnable(INT_SOURCE_UART_TX(HOSTuart), INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(HOSTuart), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(HOSTuart), INT_SUB_PRIORITY_LEVEL_0);

    //PORTClearBits(IOPORT_B, BIT_3 | BIT_12 | BIT_13 | BIT_14 | BIT_15);
    //mPORTBSetPinsDigitalOut(BIT_0 | BIT_3 | BIT_12 | BIT_13 | BIT_14 | BIT_15);

    // PORTSetPinsDigitalIn(IOPORT_B, BIT_1 | BIT_2);
    // PORTSetPinsDigitalOut(IOPORT_B, BIT_0);
    
    PORTSetPinsDigitalOut(IOPORT_A, BIT_5);  // RA5 = DIR4
    //PORTClearBits(IOPORT_A, BIT_5);
    
    PORTSetPinsDigitalOut(IOPORT_D, BIT_7 | BIT_11 | BIT_13);  // RD7 = PWM_SPI_CS, RD11 = DIR3, RD13 = DIR2
    PWM_SPI_CS = 0;
    //PORTClearBits(IOPORT_D, BIT_11 | BIT_13);
    // PORTSetBits(IOPORT_D, BIT_11|BIT_13);    
    
    PORTSetPinsDigitalIn(IOPORT_E, BIT_5 | BIT_7 | BIT_8 | BIT_9);    // RE5 = ENC2 DIR, RE7 = ENC3 DIR, RE8 = ENC4 DIR, RE9 = ENC1 DIR
    PORTSetPinsDigitalOut(IOPORT_E, BIT_3 | BIT_6);    // RE3 = TEST_OUT, RE6 = LED_OUT
    
    TEST_OUT = 0;
    LED_OUT = 0;
    
    PORTSetPinsDigitalOut(IOPORT_F, BIT_1);             //RF1 = PWM_DISABLE
    PWM_DISABLE = 0;
    
    PORTSetPinsDigitalOut(IOPORT_G, BIT_12);   // RG12 = DIR1 
    //PORTClearBits(IOPORT_G, BIT_12);
        
    DIR1 = DIR2 = DIR3 = DIR4 = 0;            
    
    //PORTSetPinsDigitalIn(IOPORT_B, BIT_1 | BIT_4 | BIT_2);
    //mCNOpen(CN_ON, CN6_ENABLE | CN4_ENABLE, CN6_PULLUP_ENABLE | CN4_PULLUP_ENABLE);
    //ConfigIntCN(CHANGE_INT_ON | CHANGE_INT_PRI_2);
    
    // Set up Timer 2 for PWM time base    
    T2CON = 0x00;
    T2CONbits.TCKPS2 = 0; // 1:1 Prescaler
    T2CONbits.TCKPS1 = 0;
    T2CONbits.TCKPS0 = 0;    
    PR2 = 4000; // Use 50 microsecond rollover for 20 khz
    T2CONbits.TON = 1; // Let her rip   
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);        
    
    
    // Turn on the interrupts
    INTEnableSystemMultiVectoredInt();

    
	mInitAllLEDs();    
}//end UserInit

// HOST UART interrupt handler it is set at priority level 2
void __ISR(HOST_VECTOR, ipl2) IntHostUartHandler(void) {
    unsigned char ch;
    static unsigned short HOSTRxIndex = 0;
    static unsigned char arrowIndex = 0;
    static unsigned char arrArrow[3];
    int i;

    if (HOSTbits.OERR || HOSTbits.FERR) {
        if (UARTReceivedDataIsAvailable(HOSTuart))
            ch = UARTGetDataByte(HOSTuart);
        HOSTbits.OERR = 0;
        HOSTRxIndex = 0;
    } else if (INTGetFlag(INT_SOURCE_UART_RX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_RX(HOSTuart));
        if (UARTReceivedDataIsAvailable(HOSTuart)) {
            // ch = toupper(UARTGetDataByte(HOSTuart));
            ch = UARTGetDataByte(HOSTuart);
            /*
            if (mode == STANDBY)
            {
                if (ch == 27 && arrowIndex == 0) 
                    arrowIndex++;
                else if (ch == 91 && arrowIndex == 1)
                    arrowIndex++;
                else if ((ch >= 65 && ch <=68) && arrowIndex == 2)                
                {
                    command = ch;
                    arrowIndex = 0;
                }
            }
            
            if (ch == SPACE)
            {
                if (mode != STANDBY) command = SPACE;
            }
            else if (ch < 27 && ch != CR) 
            {
                command = ch;
            }
            else
            */
            {
                if (ch == LF || ch == 0);
                else if (ch == BACKSPACE) 
                {
                    while (!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte(HOSTuart, ' ');
                    while (!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte(HOSTuart, BACKSPACE);
                    if (HOSTRxIndex > 0) HOSTRxIndex--;
                } 
                else if (ch == CR) 
                {
                    if (HOSTRxIndex < (MAXBUFFER-1)) 
                    {
                        HOSTRxBuffer[HOSTRxIndex] = CR;
                        HOSTRxBuffer[HOSTRxIndex + 1] = '\0'; 
                        HOSTRxBufferFull = true;
                    }
                    HOSTRxIndex = 0;
                }                
                else 
                {
                    if (HOSTRxIndex < (MAXBUFFER-1))
                        HOSTRxBuffer[HOSTRxIndex++] = ch;                    
                }
            }
        }
    }
    if (INTGetFlag(INT_SOURCE_UART_TX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_TX(HOSTuart));
    }
}


void __ISR(_TIMER_2_VECTOR, ipl5) Timer2Handler(void) 
{
    static short PIDupdateCounter = 0;
    static short milliSecondCounter = 0;
    mT2ClearIntFlag(); // clear the interrupt flag
    
    if (TEST_OUT) TEST_OUT = 0;
    else TEST_OUT = 1;    
    
    PIDupdateCounter++;
    if (PIDupdateCounter >= 30)
    {
        PIDupdateCounter = 0;
        if (TestMode)
        {
            SendFlag = true;
            //while (!UARTTransmitterIsReady(HOSTuart));            
            //UARTSendDataByte(HOSTuart, '>');
        }
        //HOSTRxBuffer[0] = '>';
        //HOSTRxBuffer[1] = '\0';
        //HOSTRxBufferFull = true;        
    }
    
    
    milliSecondCounter++;
    if (milliSecondCounter >= 2000)
    {
        milliSecondCounter = 0;
        if (LED_OUT) 
        {
            LED_OUT = 0;            
            DIR1 = 0;
            DIR2 = 0;
            DIR3 = 0;
            DIR4 = 0;      
            PWM_DISABLE = 1;
            PWM_SPI_CS = 0;
        }
        else
        {
            LED_OUT = 1;            
            DIR1 = 1;
            DIR2 = 1;
            DIR3 = 1;
            DIR4 = 1;         
            PWM_DISABLE = 0;
            PWM_SPI_CS = 1;
            
            EnableAD = true;
        }
    }
    
    
}

void ConfigAd(void) 
{
    mPORTBSetPinsAnalogIn(BIT_12 | BIT_13 | BIT_14 | BIT_15); // $$$$

    // ---- configure and enable the ADC ----

    // ensure the ADC is off before setting the configuration
    CloseADC10();

    // define setup parameters for OpenADC10
    //                 Turn module on | ouput in integer | trigger mode auto | enable autosample
#define PARAM1  ADC_MODULE_ON | ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON

    // ADC ref external    | disable offset test    | enable scan mode | perform  samples | use dual buffers | use only mux A
#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_4 | ADC_ALT_BUF_ON | ADC_ALT_INPUT_OFF

    //                   use ADC internal clock | set sample time
    // #define PARAM3  ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_31
#define PARAM3  ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_31 | ADC_CONV_CLK_32Tcy

#define PARAM4    ENABLE_AN12_ANA | ENABLE_AN13_ANA | ENABLE_AN14_ANA | ENABLE_AN15_ANA

// USE FOUR POTS ON RB3, RB8, RB9, RB10: AN3, AN8, AN9, AN10, SKIP ALL OTHERS:
#define PARAM5 SKIP_SCAN_AN0 | SKIP_SCAN_AN1 | SKIP_SCAN_AN2 | SKIP_SCAN_AN3 |\
SKIP_SCAN_AN4 | SKIP_SCAN_AN5 | SKIP_SCAN_AN6 | SKIP_SCAN_AN7 |\
SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 | SKIP_SCAN_AN11

    // set negative reference to Vref for Mux A
    SetChanADC10(ADC_CH0_NEG_SAMPLEA_NVREF);

    // open the ADC
    OpenADC10(PARAM1, PARAM2, PARAM3, PARAM4, PARAM5);

    ConfigIntADC10(ADC_INT_PRI_2 | ADC_INT_SUB_PRI_2 | ADC_INT_ON);

    // clear the interrupt flag
    mAD1ClearIntFlag();

    // Enable the ADC
    EnableADC10();
}

void __ISR(_ADC_VECTOR, ipl6) ADHandler(void) 
{
    unsigned short offSet;
    unsigned char i;

    mAD1IntEnable(INT_DISABLED);
    mAD1ClearIntFlag();

    // Determine which buffer is idle and create an offset
    offSet = 8 * ((~ReadActiveBufferADC10() & 0x01));

    for (i = 0; i < MAXPOTS; i++)
        ADresult[i] = (unsigned short) ReadADC10(offSet + i); // read the result of channel 0 conversion from the idle buffer
}



/** EOF main.c *************************************************/
