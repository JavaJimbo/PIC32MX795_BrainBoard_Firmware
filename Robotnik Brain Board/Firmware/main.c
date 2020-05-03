/**********************************************************************************
 * PROJECT: ROBOTNIK BRAIN BOARD - NON-USB VERSION
  * main.c
 *
 * Compiled for PIC32MX795 XC32 compiler version 1.30
 * Utilizes Infineon IFX9201 BRIDGE DRIVER
 
 * Created from USB_Robotnik - USB routines removed
 * 
 * 7-20-19 Tested: USB, SPI_CS, PWM, DIR, DISABLE, AD, SD Card, Encoder counter and direction inputs.
 * 7-25-19 Tested: RS485 on UART5 Rx and Tx both working; PUSHBUTTON input on RG15, interrupt on change for RB1-RB4
 * 8-05-19 SD_CS is RE4
 * 8-9-19: Testing PWM with ADC inputs. Everything works!
 * 8-11-19: Got I2C EEprom working.
 * 8-13-19 Substituted C:\Program Files (x86)\Microchip\xc32\v1.30\pic32-libs\include\lega-c\peripheral
 * Adapted I2C routines from Super Copter on Electro Tech Online: 
 *  https://www.electro-tech-online.com/threads/pic32-c32-cant-read-i2c-bus.145272/
 * 8-15-19: Recompiled. Added files for Adafruit PCA9685 servo controller. Works great with pot input using PIC I2C3 port.
 * 8-23-19: DMA for RX: got rid of bugs - works great at 921600 baud
 * 8-26-19: Works receiving 100 servos @ 921600 baud.
 * 8-31-19: Servo loop works great, also added SPI with SD card and H Bridge.
 * 9-1-19:  Works with USB_Robotnik and VC++ Robotnik Controller recording/playing four servo motors.
 *          Cleaned up flashing LED indicator.
 * 9-22-19: Recompiled 
 * 12-8-19:  Recompiled.  Default mode = REMOTE
 * 5-3-20: Modified and optimized PID for ServoCity motors 53:1 ratio. 
 *         Checked SD card reads & writes; Also SPI diagnostics from IFX9201 H bridge.
 ***********************************************************************************/
#include <xc.h>
#include "HardwareProfile.h"
#include "uart2.h"
#include "Delay.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "HardwareProfile.h"
#include "I2C_4BUS_EEPROM_PIC32.h"

#include "FSIO.h"
#include "Delay.h"
#include "Defs.h"
#include "SD-SPI.h"
#define _SUPPRESS_PLIB_WARNING

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

#define false FALSE
#define true TRUE

#define HALTED 0
#define LOCAL 1
#define JOG 2
#define REMOTE 3

#define PUSHBUTTON_IN LATGbits.LATG15

#define EncoderOne TMR1
#define EncoderTwo TMR5
#define EncoderThree TMR3
#define EncoderFour TMR4

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
#define LED LATEbits.LATE6

#define RS485_Control LATBbits.LATB0

// UART FOR PC SERIAL PORT
#define HOSTuart UART2
#define HOSTbits U2STAbits
#define HOST_VECTOR _UART_2_VECTOR

#define RS485uart UART5
#define RS485bits U5STAbits
#define RS485_VECTOR _UART_5_VECTOR


#define CR 13
#define LF 10
#define BACKSPACE 8
#define SPACE 32
#define ESCAPE 27
#define RIGHT_ARROW 67
#define LEFT_ARROW 68
#define UP_ARROW 65
#define DOWN_ARROW 66
#define MAXNUM 16
#define MAXBUFFER 255

/** V A R I A B L E S ********************************************************/
unsigned char NUMbuffer[MAXNUM + 1];
unsigned char HOSTRxBuffer[MAXBUFFER+1];
unsigned char HOSTRxBufferFull = false;
unsigned char HOSTTxBuffer[MAXBUFFER+1]; 
unsigned char MEMORYBuffer[MAXBUFFER+1]; 
unsigned char displayFlag = false;

unsigned char RS485RxBufferFull = false;
unsigned char RS485TxBufferFull = false;
unsigned char RS485RxBuffer[MAXBUFFER+1];
unsigned char RS485RxBufferCopy[MAXBUFFER+1];
unsigned char ServoData[MAXBUFFER+1];
short servoPositions[MAXSERVOS];
long ActualRS485BaudRate = 0;
int timeout = 0;

/** P R I V A T E  P R O T O T Y P E S ***************************************/

static void InitializeSystem(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void UserInit(void);
void InitializeUSART(void);
void putcUSART(char c);
unsigned char getcUSART();
extern unsigned char CheckCRC (unsigned char *ptrRxModbus, short RxModbusLength);
unsigned short decodePacket(unsigned char *ptrInPacket, unsigned char *ptrData);
void PrintServoData(short numServos, short *ptrServoPositions, unsigned char command, unsigned char subCommand);
unsigned char processPacketData(short packetDataLength, unsigned char *ptrPacketData, short *numServos, short *ptrServoPositions, unsigned char *command, unsigned char *subCommand);
unsigned char SendReceiveSPI(unsigned char dataOut);
void ResetPID();
long PIDcontrol(long servoID, struct PIDtype *PID);
unsigned char ReadHBridgeData(unsigned char *ptrData);
void initHBridgeSPI(void);
unsigned char ADready = false;
unsigned char EnableAD = false;
unsigned char DATABufferFull = false;
void ConfigAd(void);
void ClearCopyBuffer();
void makeFloatString(float InValue, int numDecimalPlaces, unsigned char *arrAscii);


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
#define MAXPOTS 8
unsigned short ADresult[MAXPOTS];
unsigned short PortBRead = 0;
unsigned char PortBflag = false;
unsigned char intFlag = false;
unsigned char memoryFlag = false;



int main(void) 
{
    short i = 0, j = 0, p = 0, q = 0;        
    long PWMvalue = 0;    
    unsigned char command, subCommand, ch;
    int JogPWM = 0;
    float floValue;            
    unsigned char HBridgeData[4];
    short LEDcounter = 0;
    FSFILE *filePtr;
    char filename[] = "NextFile.txt";
    short length = 0;
    unsigned short numBytes;
    short dataLength = 0;
    unsigned char PacketData[MAXBUFFER];
    short numServos;
    short ServoPositions[MAXSERVOS] = {512,512,512,512};
    unsigned char MessageOut[] = "You can eat the driver and his gloves\r\n ";
    unsigned ADdisplay = true;
    short SPIcounter = 0;
    
    unsigned char runMode = LOCAL;
    unsigned char previousRunMode = LOCAL;
        
    PWM1 = PWM2 = PWM3 = PWM4 = 0;
    ResetPID();
        
    DelayMs(10);
    InitializeSystem();  
    DelayMs(10);
    
    LED = 1;
    SD_CS = 1;
    SD_WE = 1; // ENable writes to SD card
    PWM_SPI_CS = 1;   
        
    DelayMs(10);    
    
    if (runMode) printf("\r\rBrain Board - PID for Servo City motors. MOTORS ON: ");
    else printf("\r\rBrain Board - PID for Servo City motors. MOTORS OFF: ");
    
    if (previousRunMode == LOCAL) printf("LOCAL MODE");
    else if (previousRunMode == REMOTE) printf("REMOTE MODE");
    else if (previousRunMode == JOG) printf("JOG MODE");
    else printf("MODE ERROR");    
    
    if (MDD_MediaDetect())
    {
        printf("\rInitializing SD card...");
        while (!FSInit());               
        printf("\rOpening test file %s to write...", filename);
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
        DelayMs(10);       
    }
    else printf("\rNo SD card found");    
    
    while(1) 
    {          
        if (RS485RxBufferFull)
        {
            RS485RxBufferFull = false;
            dataLength =  decodePacket(RS485RxBufferCopy, PacketData);            
            if (!processPacketData(dataLength, PacketData, &numServos, ServoPositions, &command, &subCommand)) 
                printf("\rCRC ERROR");
            else PrintServoData(numServos, ServoPositions, command, subCommand);                                    
            ClearCopyBuffer();
            
            LEDcounter++;
            if (LEDcounter > 4)
            {
                LEDcounter = 0;
                if (LED) LED = 0;
                else LED = 1;
            }            
        }
        
        if (intFlag)
        {
            intFlag = false;
            if (TEST_OUT) TEST_OUT = 0;
            else TEST_OUT = 1;

            if (runMode)
            {               
                /*
                HBridgeSPIenable++;
                if (HBridgeSPIenable >= 1000)
                {
                    HBridgeSPIenable = 0;
                    initHBridgeSPI();
                    ReadHBridgeData(HBridgeData);
                    printf("\r#%d: HBridge: %02X, %02X, %02X, %02X", SPIcounter++, HBridgeData[0], HBridgeData[1], HBridgeData[2], HBridgeData[3]);
                    SpiChnClose(SPI_CHANNEL);                
                }            
                */
                mAD1IntEnable(INT_ENABLED);
                for (i = 0; i < NUMMOTORS; i++)
                {                            
                    if (runMode == REMOTE) PID[i].ADCommand = (long)(((ServoPositions[i] * 3) / 5) + 300);
                    else PID[i].ADCommand = (long)(((ADresult[i] * 3) / 5) + 300);                    
                    PID[i].ADActual = (long) (1023 - ADresult[i+4]);                    
                    PIDcontrol(i, PID);                
                    if (runMode == JOG) PWMvalue = JogPWM;
                    else PWMvalue = PID[i].PWMvalue;                
                    if (i == 0) 
                    {
                        if (PWMvalue < 0)
                        {            
                            DIR1 = REVERSE;                                      
                            PWMvalue = 0 - PWMvalue;
                        }
                        else DIR1 = FORWARD;                                    
                        PWM1 = PWMvalue;                            
                    }                        
                    else if (i == 1)
                    {
                        if (PWMvalue < 0)
                        {            
                            DIR2 = REVERSE;                    
                            PWMvalue = 0 - PWMvalue;
                        }
                        else DIR2 = FORWARD;
                        PWM2 = PWMvalue;                            
                    }                                                
                    else if (i == 2) 
                    {
                        if (PWMvalue < 0)
                        {            
                            DIR3 = REVERSE;                    
                            PWMvalue = 0 - PWMvalue;
                        }
                        else DIR3 = FORWARD;                            
                        PWM3 = PWMvalue;
                    }
                    else
                    {
                        if (PWMvalue < 0)
                        {            
                            DIR4 = REVERSE;                    
                            PWMvalue = 0 - PWMvalue;
                        }
                        else DIR4 = FORWARD;                            
                        PWM4 = PWMvalue;                            
                    }                        
                }
            }
            else
            {
                PWM1 = PWM2 = PWM3 = PWM4 = 0;
                LED = 0;
            }
        }    
        
        
        if (HOSTRxBufferFull)
        {
            HOSTRxBufferFull = false; 
            printf("\rReceived: ");
            q = 0;
            command = 0;
            for (p = 0; p < MAXBUFFER; p++) 
            {
                ch = HOSTRxBuffer[p];
                if (isalpha(ch) || ch == ' ') command = ch;                
                putchar(ch);
                if (ch == '\r' || ch == ' ')break;
                if ( (isdigit(ch) || ch == '.' || ch == '-') && q < MAXNUM) NUMbuffer[q++] = ch;
            }
            if (q) 
            {
                NUMbuffer[q] = '\0';
                floValue = atof(NUMbuffer);
            }
            if (command) 
            {
                switch (command) 
                {
                    case 'P':
                        if (q) PID[0].kP = PID[1].kP = PID[2].kP = PID[3].kP = floValue;
                        break;
                    case 'I':
                        if (q) PID[0].kI = PID[1].kI = PID[2].kI = PID[3].kI = floValue;
                        break;
                    case 'D':
                        if (q) PID[0].kD = PID[1].kD = PID[2].kD = PID[3].kD = floValue;
                        break;
                    case 'O':
                        if (q) PID[0].PWMoffset = PID[1].PWMoffset = PID[2].PWMoffset = PID[3].PWMoffset = (long) floValue;
                        break;
                    case 'R':
                        runMode = REMOTE;
                        printf("REMOTE MODE ON");
                        break;
                    case 'L':
                        runMode = LOCAL;
                        printf("LOCAL ON");
                        break;       
                    case 'J':
                        JogPWM = (long) floValue;
                        printf("\rJOG ON: %d", JogPWM);
                        runMode = JOG;                        
                        break;                        
                    case 'H':
                        initHBridgeSPI();
                        ReadHBridgeData(HBridgeData);
                        printf("\r#%d: HBridge: %02X, %02X, %02X, %02X", SPIcounter++, HBridgeData[0], HBridgeData[1], HBridgeData[2], HBridgeData[3]);
                        SpiChnClose(SPI_CHANNEL);
                        break;
                    case ' ':
                        if (runMode) 
                        {                            
                            previousRunMode = runMode;
                            runMode = HALTED; 
                            printf("\rHALT");
                        }
                        else
                        {
                            runMode = previousRunMode;
                            if (runMode == LOCAL) printf("\rAD MODE");
                            else if (runMode == REMOTE) printf("\rREMOTE MODE");
                            else if (runMode == JOG) printf("\rJOG MODE");
                            else printf("\rMODE ERROR");
                        }
                        break;
                    case 'M':
                        if (displayFlag)
                        {
                            displayFlag = false;
                            printf("\rDisplay OFF");
                        }
                        else {
                            displayFlag = true;
                            printf("\rDisplay ON");
                        }   
                        break;
                    case 'Z':
                        ResetPID();
                        printf("\rPID reset = true");
                        break;
                    case 'T':
                        if (ADdisplay)
                        {
                            ADdisplay = false;
                            printf("\rPot display OFF");
                        }
                        else
                        {
                            ADdisplay = true;
                            printf("\rPot display ON");
                        }
                        break;
                    default:
                        printf("\rCommand: %c", command);
                        break;
                } // end switch command                
                printf("\rkP = %0.1f, kI = %0.3f, kD = %0.1f, Offset: %d\r", PID[0].kP, PID[0].kI, PID[0].kD, PID[0].PWMoffset);
            } // End if command             
        } // End if HOSTRxBufferFull
        
        /*
        if (DATABufferFull)
        {
            DATABufferFull = false;
            success = processPacket(DATARxBuffer, &dataCommand, &dataSubCommand, arrServoPos, &numServos);
            result = success;
        }
        */
    } // End while(1))
} // End main())


void InitializeSystem(void) 
{
	int i; 
    
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
    OC1RS = 0;

    // Set up PWM OC2
    OC2CON = 0x00;
    OC2CONbits.OC32 = 0; // 16 bit PWM
    OC2CONbits.ON = 1; // Turn on PWM
    OC2CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC2CONbits.OCM2 = 1; // PWM runMode enabled, no fault pin
    OC2CONbits.OCM1 = 1;
    OC2CONbits.OCM0 = 0;
    OC2RS = 0;

    // Set up PWM OC3
    OC3CON = 0x00;
    OC3CONbits.OC32 = 0; // 16 bit PWM
    OC3CONbits.ON = 1; // Turn on PWM
    OC3CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC3CONbits.OCM2 = 1; // PWM runMode enabled, no fault pin
    OC3CONbits.OCM1 = 1;
    OC3CONbits.OCM0 = 0;
    OC3RS = 0;

    // Set up PWM OC4
    OC4CON = 0x00;
    OC4CONbits.OC32 = 0; // 16 bit PWM
    OC4CONbits.ON = 1; // Turn on PWM
    OC4CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC4CONbits.OCM2 = 1; // PWM runMode enabled, no fault pin
    OC4CONbits.OCM1 = 1;
    OC4CONbits.OCM0 = 0;
    OC4RS = 0;
    
    // Set up PWM OC5
    OC5CON = 0x00;
    OC5CONbits.OC32 = 0; // 16 bit PWM
    OC5CONbits.ON = 1; // Turn on PWM
    OC5CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC5CONbits.OCM2 = 1; // PWM runMode enabled, no fault pin
    OC5CONbits.OCM1 = 1;
    OC5CONbits.OCM0 = 0;
    OC5RS = 0;
    
    
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
    
    // Set up RS485 UART    
    UARTConfigure(RS485uart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(RS485uart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(RS485uart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    ActualRS485BaudRate = UARTSetDataRate(RS485uart, SYS_FREQ, 921600);
    // ActualRS485BaudRate = UARTSetDataRate(RS485uart, SYS_FREQ, 2000000);
    UARTEnable(RS485uart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure RS485 UART Interrupts
    INTEnable(INT_SOURCE_UART_TX(RS485uart), INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(RS485uart), INT_DISABLED);
    //INTSetVectorPriority(INT_VECTOR_UART(RS485uart), INT_PRIORITY_LEVEL_2);
    //INTSetVectorSubPriority(INT_VECTOR_UART(RS485uart), INT_SUB_PRIORITY_LEVEL_0);  
        
    
    // CONFIGURE DMA CHANNEL 0 FOR UART 5 INTERRUPT ON MATCH       
    for (i = 0; i < MAXBUFFER; i++) RS485RxBufferCopy[i] = 0x00;     
    
    DmaChnOpen(DMA_CHANNEL0, 1, DMA_OPEN_MATCH);    
    DmaChnSetMatchPattern(DMA_CHANNEL0, '\r');	// set < as ending character
    // Set the transfer event control: what event is to start the DMA transfer    
	// We want the UART2 rx interrupt to start our transfer
	// also we want to enable the pattern match: transfer stops upon detection of CR
	DmaChnSetEventControl(DMA_CHANNEL0, DMA_EV_START_IRQ_EN|DMA_EV_MATCH_EN|DMA_EV_START_IRQ(_UART5_RX_IRQ));            
	// Set the transfer source and dest addresses, source and dest sizes and the cell size
	DmaChnSetTxfer(DMA_CHANNEL0, (void*)&U5RXREG, RS485RxBuffer, 1, MAXBUFFER, 1);    
    // Enable the transfer done event flag:
    DmaChnSetEvEnableFlags(DMA_CHANNEL0, DMA_EV_BLOCK_DONE);		// enable the transfer done interrupt: pattern match or all the characters transferred
	INTSetVectorPriority(INT_VECTOR_DMA(DMA_CHANNEL0), INT_PRIORITY_LEVEL_5);		// set INT controller priority
	INTSetVectorSubPriority(INT_VECTOR_DMA(DMA_CHANNEL0), INT_SUB_PRIORITY_LEVEL_3);		// set INT controller sub-priority
	INTEnable(INT_SOURCE_DMA(DMA_CHANNEL0), INT_ENABLED);		// enable the chn interrupt in the INT controller    
    // Once we configured the DMA channel we can enable it
    DmaChnEnable(DMA_CHANNEL0);        
       
    
    //PORTClearBits(IOPORT_B, BIT_3 | BIT_12 | BIT_13 | BIT_14 | BIT_15);
    //mPORTBSetPinsDigitalOut(BIT_0 | BIT_3 | BIT_12 | BIT_13 | BIT_14 | BIT_15);

    // PORTSetPinsDigitalIn(IOPORT_B, BIT_1 | BIT_2);
    // PORTSetPinsDigitalOut(IOPORT_B, BIT_0);
    
    PORTSetPinsDigitalOut(IOPORT_A, BIT_0 | BIT_5);  // RA5 = DIR4
    //PORTClearBits(IOPORT_A, BIT_5);
    
    PORTSetPinsDigitalOut(IOPORT_B, BIT_0);  // RB0 = RS485 control
    RS485_Control = 0;
    
      
    
    /*
    PORTSetPinsDigitalIn(IOPORT_B, BIT_1 | BIT_2 | BIT_3 | BIT_4);
    mCNOpen(CN_ON, CN3_ENABLE | CN4_ENABLE | CN5_ENABLE | CN6_ENABLE, CN3_PULLUP_ENABLE | CN4_PULLUP_ENABLE | CN5_PULLUP_ENABLE | CN6_PULLUP_ENABLE);
    ConfigIntCN(CHANGE_INT_ON | CHANGE_INT_PRI_2);    
    */
    
    
    
    PORTSetPinsDigitalOut(IOPORT_D, BIT_7 | BIT_8 | BIT_11 | BIT_13);  // RD7 = PWM_SPI_CS, RD8 = EEPROM_WP, RD11 = DIR3, RD13 = DIR2
    PWM_SPI_CS = 0;
    EEPROM_WP = 1;
    //PORTClearBits(IOPORT_D, BIT_11 | BIT_13);
    // PORTSetBits(IOPORT_D, BIT_11|BIT_13);    
    
    PORTSetPinsDigitalIn(IOPORT_E, BIT_5 | BIT_7 | BIT_8 | BIT_9);    // RE5 = ENC2 DIR, RE7 = ENC3 DIR, RE8 = ENC4 DIR, RE9 = ENC1 DIR
    PORTSetPinsDigitalOut(IOPORT_E, BIT_3 | BIT_4 | BIT_6);    // RE3 = TEST_OUT, RE4 = SD_CS, RE6 = LED
    
    // TEST_OUT = 0;
    LED = 0;
    SD_CS = 1;
    
    PORTSetPinsDigitalOut(IOPORT_F, BIT_1 | BIT_2);             //RF1 = PWM_DISABLE, RF2: TEST_OUT
    PWM_DISABLE = 0;
    
    PORTSetPinsDigitalOut(IOPORT_G, BIT_12);   // RG9 = TEST_OUT, RG12 = DIR1 

    PORTSetPinsDigitalIn(IOPORT_G, BIT_9 | BIT_15);   // RG9 = SD_DETECT, RG15 = PUSHBUTTON_IN
        
    DIR1 = DIR2 = DIR3 = DIR4 = 0;                
    
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
   
}//end UserInit

// Timer 2 generates an interrupt every 50 microseconds approximately
void __ISR(_TIMER_2_VECTOR, ipl5) Timer2Handler(void) 
{        
    static int intCounter = 0; 
    static int memoryCounter = 625; 
    
    mT2ClearIntFlag(); // clear the interrupt flag    
    
    intCounter++;
    if (intCounter >= 50)
    {
        intCounter = 0;
        intFlag = true;
    }
    
    if (memoryCounter) memoryCounter--;
    if (!memoryCounter)
    {
        memoryCounter=625;
        memoryFlag = true;
        if (timeout)
        {
            timeout--;
            if (timeout == 0) LED = 0;
        }
    }
}

void ConfigAd(void) 
{
    // mPORTBSetPinsAnalogIn(BIT_12 | BIT_13 | BIT_14 | BIT_15); // $$$$
    mPORTBSetPinsAnalogIn(BIT_1 | BIT_2 | BIT_3 | BIT_4 | BIT_12 | BIT_13 | BIT_14 | BIT_15); // $$$$

    // ---- configure and enable the ADC ----

    // ensure the ADC is off before setting the configuration
    CloseADC10();

    // define setup parameters for OpenADC10
    //                 Turn module on | ouput in integer | trigger mode auto | enable autosample
#define PARAM1  ADC_MODULE_ON | ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON

    // ADC ref external    | disable offset test    | enable scan mode | perform  samples | use dual buffers | use only mux A
#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_8 | ADC_ALT_BUF_ON | ADC_ALT_INPUT_OFF

    //                   use ADC internal clock | set sample time
    // #define PARAM3  ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_31
#define PARAM3  ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_31 | ADC_CONV_CLK_32Tcy

/*
#define PARAM4    ENABLE_AN12_ANA | ENABLE_AN13_ANA | ENABLE_AN14_ANA | ENABLE_AN15_ANA
#define PARAM5 SKIP_SCAN_AN0 | SKIP_SCAN_AN1 | SKIP_SCAN_AN2 | SKIP_SCAN_AN3 |\
SKIP_SCAN_AN4 | SKIP_SCAN_AN5 | SKIP_SCAN_AN6 | SKIP_SCAN_AN7 |\
SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 | SKIP_SCAN_AN11   
*/
    

#define PARAM4    ENABLE_AN1_ANA | ENABLE_AN2_ANA | ENABLE_AN3_ANA | ENABLE_AN4_ANA    
#define PARAM5 SKIP_SCAN_AN0 |\
SKIP_SCAN_AN5 | SKIP_SCAN_AN6 | SKIP_SCAN_AN7 |\
SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 | SKIP_SCAN_AN11 
// SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | SKIP_SCAN_AN15


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
    ADready = true;
}



/******************************************************************************
 *	Change Notice Interrupt Service Routine
 *
 *   Note: Switch debouncing is not performed.
 *   Code comes here if SW2 (CN16) PORTD.RD7 is pressed or released.
 *   The user must read the IOPORT to clear the IO pin change notice mismatch
 *	condition first, then clear the change notice interrupt flag.
 ******************************************************************************/

void __ISR(_CHANGE_NOTICE_VECTOR, ipl2) ChangeNotice_Handler(void) 
{    
    // Step #1 - always clear the mismatch condition first
    // dummy = PORTReadBits(IOPORT_B, BIT_4 | BIT_2) & 0x0F;
    PortBRead = PORTB;// & 0x14;
    PortBflag = true;

    // Step #2 - then clear the interrupt flag
    mCNClearIntFlag();
}




// This version uses the PIC 32 SPI port 
unsigned char SendReceiveSPI(unsigned char dataOut)
{
unsigned char dataIn;

	SpiChnPutC(SPI_CHANNEL, dataOut);
	dataIn = SpiChnGetC(SPI_CHANNEL);
    
	return(dataIn);
}

// Set SPI port #2 for 8 bit Master Mode
// Sample Phase for the input bit at the end of the data out time.
// Set the Clock Edge reversed: transmit from active to idle.
// Use 80 Mhz / 1024 = 78.125 Khz clock
void initHBridgeSPI(void)
{
    SpiChnOpen(SPI_CHANNEL, SPI_OPEN_MSTEN | SPI_OPEN_MODE8 | SPI_OPEN_CKE_REV | SPI_OPEN_ON | SPI_OPEN_SMP_END, 1024);  
}



// handler for the DMA channel 1 interrupt
void __ISR(_DMA0_VECTOR, IPL5SOFT) DmaHandler0(void)
{
    int i, j;
    unsigned char ch;
	int	evFlags;				// event flags when getting the interrupt

	INTClearFlag(INT_SOURCE_DMA(DMA_CHANNEL0));	// release the interrupt in the INT controller, we're servicing int

	evFlags=DmaChnGetEvFlags(DMA_CHANNEL0);	// get the event flags

    if(evFlags&DMA_EV_BLOCK_DONE)
    { // just a sanity check. we enabled just the DMA_EV_BLOCK_DONE transfer done interrupt
        i = 0; j = 0;
        do {
            ch = RS485RxBuffer[i]; 
            RS485RxBuffer[i] = 0x00;
            RS485RxBufferCopy[i] = ch;
            i++;
        } while (i < MAXBUFFER && ch != ETX);
        RS485RxBufferFull = true;
        timeout = 32;
        DmaChnClrEvFlags(DMA_CHANNEL0, DMA_EV_BLOCK_DONE);       
        DmaChnEnable(DMA_CHANNEL0);
    }
}


unsigned short decodePacket(unsigned char *ptrInPacket, unsigned char *ptrData)
{
    unsigned short i, j;
    unsigned char escapeFlag = FALSE;
    unsigned char startFlag = false;
    unsigned char ch;

    j = 0;
    for (i = 0; i < MAXBUFFER; i++) 
    {
        ch = ptrInPacket[i];
        // Escape flag not active
        if (!escapeFlag) 
        {
            if (ch == STX) 
            {
                if (!startFlag) 
                {
                    startFlag = true;
                    j = 0;
                }
                else return (0);
            } 
            else if (ch == ETX) 
                return (j);
            else if (ch == DLE)
                escapeFlag = TRUE;
            else if (j < MAXBUFFER)
                ptrData[j++] = ch;
            else return (0);
        } 
        // Escape flag active
        else 
        {
            escapeFlag = FALSE;
            if (ch == ETX-1) ch = ETX;            
            if (j < MAXBUFFER)
                ptrData[j++] = ch;
            else return(0);
        }
    }
    return (j);
}



#define ESC 27
#define CR 13
#define BACKSPACE 8
void __ISR(HOST_VECTOR, IPL2AUTO) IntHostUartHandler(void) 
{
unsigned char ch, inByte;
static unsigned long i = 0;

    if (INTGetFlag(INT_SOURCE_UART_RX(HOSTuart))) 
    {
        INTClearFlag(INT_SOURCE_UART_RX(HOSTuart));                 
        if (HOSTbits.OERR || HOSTbits.FERR) {
            if (UARTReceivedDataIsAvailable(HOSTuart))
                ch = UARTGetDataByte(HOSTuart);
            HOSTbits.OERR = 0;            
        }
        if (UARTReceivedDataIsAvailable(HOSTuart)) 
        {
            inByte = UARTGetDataByte(HOSTuart);
            ch = toupper(inByte);
            if (ch != 0 && ch != '\n') {            
                if (ch == BACKSPACE) 
                {
                    if (i != 0) i--;
                    HOSTRxBuffer[i] = '\0';
                    while(!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte (HOSTuart, ' ');
                    while(!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte (HOSTuart, BACKSPACE);                
                } 
                else if (i < MAXBUFFER) 
                {
                    HOSTRxBuffer[i] = ch;
                    i++;
                }            
                if ('\r' == ch || ' ' == ch) 
                {
                    HOSTRxBufferFull = true;
                    HOSTRxBuffer[i] = '\0';
                    i = 0;
                }
            }
        }
    }         
    
    if (INTGetFlag(INT_SOURCE_UART_TX(HOSTuart))) 
        INTClearFlag(INT_SOURCE_UART_TX(HOSTuart));            
}

long PIDcontrol(long servoID, struct PIDtype *PID)
{
    short Error;     
    short actualPosition;    
    short commandPosition; 
    short pastError;
    short derError;
    static short errIndex = 0;
    float PCorr = 0, ICorr = 0, DCorr = 0;    
    static short displayCounter = 0;
       
    if (servoID < 0 || servoID >= NUMMOTORS) return 0;        
    
    actualPosition = PID[servoID].ADActual;    
    commandPosition = PID[servoID].ADCommand;
    Error = actualPosition - commandPosition;    
    
    pastError = PID[servoID].error[errIndex];    
    PID[servoID].sumError = PID[servoID].sumError + (float)Error; // - pastError;
    PID[servoID].error[errIndex] = Error;
    
    if (PID[servoID].sumError > MAXSUM) PID[servoID].sumError = MAXSUM;
    if (PID[servoID].sumError < -MAXSUM) PID[servoID].sumError = -MAXSUM;
        
    errIndex++;
    if (errIndex >= FILTERSIZE) errIndex = 0;             
    
    derError = Error - pastError;     
    
    PCorr = ((float)Error) * -PID[servoID].kP;    
    ICorr = ((float)PID[servoID].sumError)  * -PID[servoID].kI;
    DCorr = ((float)derError) * -PID[servoID].kD;

    float PIDcorrection = PCorr + ICorr + DCorr;
    
    if (PIDcorrection == 0)
    {
        PID[servoID].PWMvalue = 0;
        return 0;
    }
    else if (PIDcorrection < 0) PID[servoID].PWMvalue = (long) (PIDcorrection - PID[servoID].PWMoffset);            
    else PID[servoID].PWMvalue = (long) (PIDcorrection + PID[servoID].PWMoffset);                    
               
    
    if (PID[servoID].PWMvalue > PWM_MAX) 
        PID[servoID].PWMvalue = PWM_MAX;
    else if (PID[servoID].PWMvalue < -PWM_MAX) 
        PID[servoID].PWMvalue = -PWM_MAX;                
    
    if (servoID == 0 && displayFlag)
    {
        displayCounter++;
        if (displayCounter >= 100)
        {
            displayCounter = 0;
            printf("\rCOM: %d ACT: %d ERR: %d SUM: %0.0f P: %0.1f D: %0.1f I: %0.1f PWM: %d ", commandPosition, actualPosition, Error, PID[servoID].sumError, PCorr, DCorr, ICorr, PID[servoID].PWMvalue); 
        }
    }
    PID[servoID].reset = false;   
    
    return 0;
}


/*
long PIDcontrol(long servoID, struct PIDtype *PID)
{
    long lngError;     
    long actualPosition;    
    long commandPosition; 
    long diffPosition;
    long pastError;
    long derError;
    long lngPosDiff;
    static short errIndex = 0;
    float PCorr = 0, ICorr = 0, DCorr = 0;    
    
    if (servoID < 0 || servoID >= NUMMOTORS) return 0;    
    
    if (PID[servoID].reset) diffPosition = 0;
    else diffPosition = PID[servoID].ADActual - PID[servoID].ADPrevious;        
    PID[servoID].ADPrevious = PID[servoID].ADActual;
    
    actualPosition = PID[servoID].ADActual;    
    commandPosition = PID[servoID].ADCommand;
    lngError = actualPosition - commandPosition;    

    if (lngError > 0x7FFF) lngError = 0x7FFF;
    if (lngError < -0x7FFF) lngError = -0x7FFF;
    
    pastError = PID[servoID].error[errIndex];    
    PID[servoID].sumError = PID[servoID].sumError + lngError - pastError;
    PID[servoID].error[errIndex] = (short) lngError;
    
    if (PID[servoID].sumError > MAXSUM) PID[servoID].sumError = MAXSUM;
    if (PID[servoID].sumError < -MAXSUM) PID[servoID].sumError = -MAXSUM;
        
    errIndex++;
    if (errIndex >= FILTERSIZE) errIndex = 0;             
    
    derError = lngError - pastError;     
    
    PCorr = ((float) lngError) * -PID[servoID].kP;    
    ICorr = ((float) PID[servoID].sumError)  * -PID[servoID].kI;
    DCorr = ((float) derError) * -PID[servoID].kD;

    float PIDcorrection = PCorr + ICorr + DCorr;
    
    if (PIDcorrection < 0) PID[servoID].PWMvalue = (long) (PIDcorrection - PID[servoID].PWMoffset);            
    else PID[servoID].PWMvalue = (long) (PIDcorrection + PID[servoID].PWMoffset);                    
               
    if (PID[servoID].PWMvalue > PWM_MAX) 
        PID[servoID].PWMvalue = PWM_MAX;
    else if (PID[servoID].PWMvalue < -PWM_MAX) 
        PID[servoID].PWMvalue = -PWM_MAX;                
    
    if (servoID == 1 && displayFlag)
    {
        lngPosDiff = abs(lngError - PID[servoID].previousError);
        if (lngPosDiff > 2)
        {
            printf("\rCOM: %d, ACT: %d, ERR: %d, SUM: %d, P: %d, I: %d, D: %d, PWM: %d", commandPosition, actualPosition, lngError, PID[servoID].sumError, (int)PCorr, (int)ICorr, (int)DCorr, PID[servoID].PWMvalue);             
            PID[servoID].previousError = lngError;
        }  
    }
    PID[servoID].reset = false;
    return 0;
}
*/


    /*
    EEPROM_WP = 0;
    initI2C(I2C1);   
    EepromWriteBlock(I2C1, EEPROM_ID, EEPROMmemoryAddress, strEEmesssageOut, EEMessageLength);
    DelayMs(10);
    EEPROM_WP = 1;        
    EepromReadBlock(I2C1, EEPROM_ID, EEPROMmemoryAddress, strEEmesssageIn, EEMessageLength); 
    strEEmesssageIn[EEMessageLength] = '\0';
    printf("\rEEprom Message In: %s", strEEmesssageIn);   
    */
    
    /*
    printf("\rTesting SD card...\r");
    if (MDD_MediaDetect())
    {
        if (FSInit())
        {
            filePtr=FSfopen(filename, FS_READ);        
            if (filePtr==NULL) printf("Error: could not open %s", filename);
            else 
            {
                j = 0;
                do 
                {
                    for (i = 0; i < MAXBUFFER; i++)
                    {                      
                        numBytes = FSfread(&ch, 1, 1, filePtr);
                        if (!numBytes) break;
                        if (ch == '\n') break;
                        else MEMORYBuffer[i] = ch;
                    }                 
                    if (numBytes)
                    {
                        length = i;
                        MEMORYBuffer[length] = '\0';
                        printf("#%d: Mem: %s", j++, MEMORYBuffer);
                    }
                    else printf("\rSD card blank");
                } while(!FSfeof(filePtr) && numBytes);
                FSfclose(filePtr);                                
            }  
        }
    }
    else printf("ERROR: MEDIA DETECT");    
    */    

/*
// RS485 UART interrupt handler it is set at priority level 2
void __ISR(RS485_VECTOR, ipl2) IntRS485UartHandler(void) {
    unsigned char ch;
    static unsigned short RS485RxIndex = 0;

    if (RS485bits.OERR || RS485bits.FERR) {
        if (UARTReceivedDataIsAvailable(RS485uart))
            ch = UARTGetDataByte(RS485uart);
        RS485bits.OERR = 0;
        RS485RxIndex = 0;
    } else if (INTGetFlag(INT_SOURCE_UART_RX(RS485uart))) {
        INTClearFlag(INT_SOURCE_UART_RX(RS485uart));
        if (UARTReceivedDataIsAvailable(RS485uart)) {
            ch = UARTGetDataByte(RS485uart);
            {
                if (ch == LF || ch == 0);
                else if (ch == CR) 
                {
                    if (RS485RxIndex < (MAXBUFFER-1)) 
                    {
                        RS485RxBuffer[RS485RxIndex] = CR;
                        RS485RxBuffer[RS485RxIndex + 1] = '\0'; 
                        RS485RxBufferFull = true;
                    }
                    RS485RxIndex = 0;
                }                
                else 
                {
                    if (RS485RxIndex < (MAXBUFFER-1))
                        RS485RxBuffer[RS485RxIndex++] = ch;                    
                }
            }
        }
    }
    if (INTGetFlag(INT_SOURCE_UART_TX(RS485uart))) {
        INTClearFlag(INT_SOURCE_UART_TX(RS485uart));
    }
}
*/ 


    /*
    if (MDD_MediaDetect())
    {
        printf("\rInitializing SD card...");
        while (!FSInit());            
        printf("\rOpening test file %s to write...", filename);
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
        DelayMs(10);       
    }
    else printf("\rNo SD card found");    
    */

    //   memoryOffset = FSftell(filePtr);
    // if (SUCCESS == FSfseek(filePtr, memoryOffset, SEEK_SET))
            /*
                        initHBridgeSPI();
                        ReadHBridgeData(HBridgeData);
                        SpiChnClose(SPI_CHANNEL);
                        if (previousOffset != memoryOffset) 
                        {                                    
                            printf("HBridge: %02X, %02X, %02X, %02X, ", HBridgeData[0], HBridgeData[1], HBridgeData[2], HBridgeData[3]);                                    
                            printf("Mem @ %d: %s", memoryOffset, MEMORYBuffer);
                        }                                
                        previousOffset = memoryOffset;
                        */

void ClearCopyBuffer()
{
    int i;
    for (i = 0; i < MAXBUFFER; i++)
    {
        RS485RxBufferCopy[i] = '\0';
        // RS485RxBuffer[i] = '\0';
    }
}


unsigned char processPacketData(short packetDataLength, unsigned char *ptrPacketData, short *numServos, short *ptrServoPositions, unsigned char *command, unsigned char *subCommand)
{
    MConvertType servoValue;    
    short j, i = 0;  
    
    if (!CheckCRC(ptrPacketData, packetDataLength)) return false;    
    *command = ptrPacketData[i++];
    *subCommand = ptrPacketData[i++];
    *numServos = ptrPacketData[i++];
    
    
    if (*numServos > MAXSERVOS) return false;
    j = 0;
    while(j < *numServos)
    {
        servoValue.b[0] = ptrPacketData[i++];
        servoValue.b[1] = ptrPacketData[i++];
        ptrServoPositions[j++] = servoValue.integer;
    }
    return true;
}
unsigned char ReadHBridgeData(unsigned char *ptrData)
{
    unsigned char dataOut = 0;
    
    PWM_SPI_CS = 0;
    dataOut = SendReceiveSPI(0b00000000);
    ptrData[0] = dataOut;
    dataOut = SendReceiveSPI(0b00000000);
    ptrData[1] = dataOut;
    dataOut = SendReceiveSPI(0b00000000);
    ptrData[2] = dataOut;
    dataOut = SendReceiveSPI(0b00000000);
    ptrData[3] = dataOut;
    PWM_SPI_CS = 1;
    
    return 0;
}



void makeFloatString(float InValue, int numDecimalPlaces, unsigned char *arrAscii)
{
    int i = 0, j = 0;        
    float floValue = InValue;
    unsigned char digit;
    
    if (floValue < 0)
    {
        floValue = 0 - floValue;
        arrAscii[j++] = '-';
    }
    while (floValue >= 1)
    {
        floValue = floValue / 10.0;
        i++;
    }
    if (i == 0) 
    {
        arrAscii[j++] = '0';
        arrAscii[j++] = '.';
    }
    else
    {
        while (i > 0)
        {
            floValue = floValue * 10;
            digit = (unsigned char)floValue;
            arrAscii[j++] = digit + '0';
            floValue = floValue - (float) digit;
            i--;
        }
        arrAscii[j++] = '.';
    }
    if (numDecimalPlaces > 0)
    {        
        i = numDecimalPlaces;
        while (i > 0)
        {  
            floValue = floValue * 10;
            digit = (unsigned char)floValue;
            arrAscii[j++] = digit + '0';
            floValue = floValue - (float) digit;
            i--;                         
        }
    }
    else arrAscii[j++] = '0';
    arrAscii[j++] = '\0';
}

void ResetPID()
{
    int i, j;
    for (i = 0; i < NUMMOTORS; i++)
    {
        PID[i].sumError = 0; // For 53:1 ratio Servo City motor
        PID[i].kP = 5.0;  
        PID[i].kI = 0.001; 
        PID[i].kD = 5.0; 
        PID[i].PWMoffset = 400;
        PID[i].PWMvalue = 0;
        PID[i].ADActual = 0;
        PID[i].ADCommand = 0;
        PID[i].reset = true;
        for (j = 0; j < FILTERSIZE; j++) PID[i].error[j] = 0;
    }
}

void PrintServoData(short numServos, short *ptrServoPositions, unsigned char command, unsigned char subCommand)
{
    int i;
    printf("\rOK! Com: %d, Sub: %d, servos %d: ", command, subCommand, numServos);
    for (i = 0; i < 10; i++) printf("%d, ", ptrServoPositions[i]);
}

