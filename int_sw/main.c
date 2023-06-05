
//*****************************************************************************
// lab3 hualong Yu, zhongyu Xu;
// Application Name     - int_sw
// Application Overview - The objective of this application is to demonstrate
//							GPIO interrupts using SW2 and SW3.
//							
//
//*****************************************************************************

//****************************************************************************
//
//! \addtogroup int_sw
//! @{
//
//****************************************************************************

// Standard includes
#include <stdio.h>
#include <stdbool.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "hw_apps_rcm.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "gpio.h"
#include "utils.h"
#include "pin.h"
#include "spi.h"
#include "uart.h"
#include "timer_if.h"
#include "timer.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"
#include "glcdfont.h"
#include "test.h"
// Common interface includes
#include "uart_if.h"
#include "gpio_if.h"
#include "pin_mux_config.h"


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
extern void (* const g_pfnVectors[])(void);

#define MASTER_MODE      1
#define SPI_IF_BIT_RATE  100000
#define MAX_BUFFER  80

#define ZERO  0x08f7 //0000 1000 1111 0111
#define ONE   0x8877 //1000 1000 0111 0111
#define TWO   0x48b7 //0100 1000 1011 0111
#define THREE 0xc837 //1100 1000 0011 0111
#define FOUR  0x28d7 //0010 1000 1101 0111
#define FIVE  0xa857 //1001 1000 0101 0111
#define SIX   0x6897 //0110 1000 1001 0111
#define SEVEN 0xe817 //1110 1000 0001 0111
#define EIGHT 0x18e7 //0001 1000 1110 0111
#define NINE  0x9867 //1001 1000 0110 0111
#define ENTER 0x22dd //0010 0010 1101 1101
#define MUTE  0x50af //1001 0000 0110 1111
#define LAST  0x58a7 //0101 1000 1010 0111

// Color definitions
#define BLACK           0x0000
#define BLUE            0x001F
#define GREEN           0x07E0
#define CYAN            0x07FF
#define RED             0xF800
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF

extern void (* const g_pfnVectors[])(void);

volatile long PIN8_intcount;
volatile unsigned char PIN8_intflag;
volatile unsigned long tmp;
volatile long currentPress;
volatile long previousPress;
volatile long currentButton;
volatile long previousButton;
unsigned long buffer[1000]; // buffer to store IR Data
int isButton = -1;  // check if it is a valid button
int sameButton = 0; // check if press the same button consecutively flag
int bufferSize = 0; // the buffer size for composing message
int receiveSize = 0; // the buffer size for received message
int composing_x = 5, composing_y = 68; // composing message position
int received_x = 5, received_y = 4; // received message position
int messageReady = 0; // indicate if the received message is read to display
int previousSize = 0;
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

typedef struct PinSetting {
    unsigned long port;
    unsigned int pin;
} PinSetting;

typedef struct Letter {
    unsigned int x;
    unsigned int y;
    char letter;
} Letter;

static PinSetting PIN8  = { .port = GPIOA2_BASE, .pin = 0x2 }; // GPIOPIN8 for IR_OUT
Letter ComposingLetter[MAX_BUFFER];
Letter ReceivedLetter[MAX_BUFFER];
Letter PreviousLetter[MAX_BUFFER];

//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES                           
//*****************************************************************************
static void BoardInit(void);
unsigned long Decode(unsigned long* buffer);
void Display(unsigned long value);
void MasterMain();
char ToLetter(unsigned long value);
char forwardLetter(char letter, unsigned long value);
void ClearComposingMessage();
void ClearReceivedMessage();
void SendMessage();
void CheckMessage();
void DisplayMessage();
void ProcessIR();

//*****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS                         
//*****************************************************************************
static void GPIOA0IntHandler(void) {    // PIN61 handler
    unsigned long ulStatus;

    ulStatus = MAP_GPIOIntStatus (PIN8.port, true);
    MAP_GPIOIntClear(PIN8.port, ulStatus);     // clear interrupts on GPIOA0

    PIN8_intcount++;

    // wave form counter for a button pressed
    if(PIN8_intcount == 36) {
        PIN8_intflag = 1;
        PIN8_intcount = 0;
        Timer_IF_Start(TIMERA1_BASE, TIMER_A, 400);
    }

    // convert to 0 or 1 for waveforms
    tmp = TimerValueGet(TIMERA0_BASE, TIMER_A) >> 17;

    // when still pressing reset to zero
    if(tmp == 58 || tmp == 59) {
        PIN8_intcount = -1;
        PIN8_intflag = 1;
        Timer_IF_Start(TIMERA1_BASE, TIMER_A, 400);
    }

    buffer[PIN8_intcount] = tmp;
    TimerValueSet(TIMERA0_BASE, TIMER_A, 0);
}

static void ConsecutivePressingHandler(void)
{
    Timer_IF_InterruptClear(TIMERA1_BASE);
    currentPress++;
}

unsigned long Decode(unsigned long* buffer) {
    unsigned long value = 0;
    int i;
    for(i = 0; i < 16; i++) {
        value += *(buffer + i) << (15 - i);
    }
    return value;
}

void Display(unsigned long value) {
    switch(value) {

        case ZERO:
            Report("You pressed 0.\n\r");
            isButton = 0;
            break;
        case ONE:
            Report("You pressed 1.\n\r");
            isButton = 1;
            break;
        case TWO:
            Report("You pressed 2.\n\r");
            isButton = 2;
            break;
        case THREE:
            Report("You pressed 3.\n\r");
            isButton = 3;
            break;
        case FOUR:
            Report("You pressed 4.\n\r");
            isButton = 4;
            break;
        case FIVE:
            Report("You pressed 5.\n\r");
            isButton = 5;
            break;
        case SIX:
            Report("You pressed 6.\n\r");
            isButton = 6;
            break;
        case SEVEN:
            Report("You pressed 7.\n\r");
            isButton = 7;
            break;
        case EIGHT:
            Report("You pressed 8.\n\r");
            isButton = 8;
            break;
        case NINE:
            Report("You pressed 9.\n\r");
            isButton = 9;
            break;
        case ENTER:
            Report("You pressed Enter.\n\r");
            isButton = 10;
            break;
        case MUTE:
            Report("You pressed MUTE.\n\r");
            isButton = 11;
            break;
        case LAST:
            Report("You pressed LAST.\n\r");
            isButton = 12;
            break;
        default:
            isButton = -1;
            break;
    }
}

char toLetter(unsigned long value) {
    char letter;
    switch(value) {
        case ZERO:
            letter = ' ';
            break;
        case ONE:
            letter = '?';
            break;
        case TWO:
            letter = 'a';
            break;
        case THREE:
            letter = 'd';
            break;
        case FOUR:
            letter = 'g';
            break;
        case FIVE:
            letter = 'j';
            break;
        case SIX:
            letter = 'm';
            break;
        case SEVEN:
            letter = 'p';
            break;
        case EIGHT:
            letter = 't';
            break;
        case NINE:
            letter = 'w';
            break;
        case MUTE:
            letter = '-';
            break;
        case LAST:
            letter = '+';
            break;
        default:
            letter = '/';
            break;
    }
    return letter;
}

char forwardLetter(char letter, unsigned long value) {

    char newLetter;
    switch(value) {
        case ONE:
            if(letter == '?') {
                newLetter = '.';
            }

            else if(letter == '.') {
                newLetter = ',';
            }

            else {
                newLetter = '?';
            }
            break;
        case TWO:
            if(letter == 'c') {
                newLetter = 'a';
            }
            else {
                newLetter = letter + 1;
            }
            break;
        case THREE:
            if(letter == 'f') {
                newLetter = 'd';
            }
            else {
                newLetter = letter + 1;
            }
            break;
        case FOUR:
            if(letter == 'i') {
                newLetter = 'g';
            }
            else {
                newLetter = letter + 1;
            }
            break;
        case FIVE:
            if(letter == 'l') {
                newLetter = 'j';
            }
            else {
                newLetter = letter + 1;
            }
            break;
        case SIX:
            if(letter == 'o') {
                newLetter = 'm';
            }
            else {
                newLetter = letter + 1;
            }
            break;
        case SEVEN:
            if(letter == 's') {
                newLetter = 'p';
            }
            else {
                newLetter = letter + 1;
            }
            break;
        case EIGHT:
            if(letter == 'v') {
                newLetter = 't';
            }
            else {
                newLetter = letter + 1;
            }
            break;
        case NINE:
            if(letter == 'z') {
                newLetter = 'w';
            }
            else {
                newLetter = letter + 1;
            }
            break;
        default:
            newLetter = letter;
            break;
    }
    return newLetter;
}


void MasterMain()
{
    Message("test11111111");
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_8));

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);

    //
    Adafruit_Init();
    fillScreen(RED);
    fillScreen(WHITE);

    Message("test hello world");
    testhorizontal();
    delay(100);
    Message("Finish test hello world");
    fillScreen(BLACK);
    delay(100);


}



void SetupCommunication()
{
    PRCMPeripheralClkEnable(PRCM_UARTA1, PRCM_RUN_MODE_CLK);
    PRCMPeripheralClkEnable(PRCM_UARTA0, PRCM_RUN_MODE_CLK);
    PinTypeUART(PIN_58, PIN_MODE_6); //UART1_TX
    PinTypeUART(PIN_59, PIN_MODE_6); //UART1_RX
    PinTypeUART(PIN_55, PIN_MODE_3); //UART0_TX
    PinTypeUART(PIN_57, PIN_MODE_3); //UART0_RX

    //UART Setup
    UARTConfigSetExpClk(UARTA1_BASE, 80000000, 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
    UART_CONFIG_PAR_NONE));
    UARTConfigSetExpClk(UARTA0_BASE, 80000000, 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
    UART_CONFIG_PAR_NONE));
    UARTEnable(UARTA1_BASE);
    UARTEnable(UARTA0_BASE);
    UARTDMADisable(UARTA1_BASE, (UART_DMA_RX | UART_DMA_TX));
    UARTFIFODisable(UARTA1_BASE) ;
}

void ClearComposingMessage() {
    int i;
    for(i = 0; i < bufferSize; i++) {
        drawChar(ComposingLetter[i].x, ComposingLetter[i].y, ComposingLetter[i].letter, BLACK, BLACK, 1);
    }
    bufferSize = 0;
    composing_x = 5;
    composing_y = 68;
}

void ClearPreviousMessage() {
    int i;
    for(i = 0; i < previousSize; i++) {
        drawChar(PreviousLetter[i].x, PreviousLetter[i].y, PreviousLetter[i].letter, BLACK, BLACK, 1);
    }
}

void SendMessage() {


    if(bufferSize > 0) {

        int i;
        for(i = 0; i < bufferSize; i++) {
            // wait for UART to be available
            while(UARTBusy(UARTA1_BASE));
            UARTCharPut(UARTA1_BASE, ComposingLetter[i].letter);
        }

        // end line character
        UARTCharPut(UARTA1_BASE,'\0');
        ClearComposingMessage();
    }
}
void CheckMessage() {

    // clear UART interrupt
    UARTIntClear(UARTA1_BASE,UART_INT_RX);

    // when UART is available
    while(UARTCharsAvail(UARTA1_BASE))
    {
        char c = UARTCharGet(UARTA1_BASE);

        if(c == '\0') {
            messageReady = 1;
        }
        else {
            ReceivedLetter[receiveSize].letter = c;
            ReceivedLetter[receiveSize].x = received_x;
            ReceivedLetter[receiveSize].y = received_y;
            // increase buffer size
            receiveSize++;
            // increment pixel position
            received_x += 7;
            // position boundaries
            if(received_x >= 124) {
                received_x = 5;
                received_y += 10;
            }
        }
    }
}

void DisplayMessage() {
    if(messageReady) {
        // clear flag
        messageReady = 0;

        int i;
        ClearPreviousMessage();
        for(i = 0; i < receiveSize; i++) {
            drawChar(ReceivedLetter[i].x, ReceivedLetter[i].y, ReceivedLetter[i].letter, RED, RED, 1);
            PreviousLetter[i] = ReceivedLetter[i];
        }
        previousSize = receiveSize;
        received_x = 5;
        received_y = 4;
        receiveSize = 0;
    }
}

void ProcessIR() {
    if (PIN8_intflag) {
        PIN8_intflag = 0;  // clear flag
        currentButton = Decode(buffer + 19);
        Display(currentButton);

        // set up same button flag
        if(previousButton == currentButton) {
            sameButton = 1;
        }

        else {
            sameButton = 0;
        }
        // print out composing message on OLED

        // valid button from remote
        if(isButton != -1) {
            // delete a character
            if(toLetter(currentButton) == '-') {

                // draw previous black
                if(bufferSize > 0) {
                    bufferSize--;
                    drawChar(ComposingLetter[bufferSize].x, ComposingLetter[bufferSize].y, ComposingLetter[bufferSize].letter, YELLOW, YELLOW, 1);
                }

                // set new composing position for letters
                if(composing_x >= 12) {
                    composing_x -= 7;
                }
                else if(composing_x == 5) {
                    if(composing_y >= 78) {
                        composing_y -= 10;
                        composing_x = 117;
                    }
                }
            }

            // send a character
            else if(toLetter(currentButton) == '+') {
                SendMessage();
            }

            // add a character
            else {
                char letter;
                letter = toLetter(currentButton);

                if(bufferSize < MAX_BUFFER) {

                    // consecutive button for switching character
                    if(previousPress == currentPress && sameButton) {
                        int index = bufferSize - 1;
                        char l = ComposingLetter[index].letter;

                        // clear previous letter
                        drawChar(ComposingLetter[index].x, ComposingLetter[index].y, l, BLACK, BLACK, 1);

                        // draw the next letter
                        ComposingLetter[index].letter = forwardLetter(l, currentButton);
                        drawChar(ComposingLetter[index].x, ComposingLetter[index].y, ComposingLetter[index].letter, BLUE, BLUE, 1);
                    }

                    else {
                        Letter CL;
                        CL.x = composing_x;
                        CL.y = composing_y;
                        CL.letter = letter;
                        ComposingLetter[bufferSize] = CL;

                        drawChar(composing_x, composing_y, letter, BLUE, BLUE, 1);

                        // adjust pixel positions
                        composing_x += 7;
                        if(composing_x >= 124) {
                            composing_x = 5;
                            composing_y += 10;
                        }

                        // increase buffer size for next input
                        bufferSize++;
                    }
                }
            }
        }
        // update press flag and button
        previousPress = currentPress;
        previousButton = currentButton;
    }
}


//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void) {
	MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
    
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}
//****************************************************************************
//
//! Main function
//!
//! \param none
//! 
//!
//! \return None.
//
//****************************************************************************
int main() {
	unsigned long ulStatus;

    BoardInit();
    
    PinMuxConfig();
    

    //
    // Enable the SPI module clock
    //
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);

    InitTerm();

    ClearTerm();

    // Reset SPI
    MAP_SPIReset(GSPI_BASE);

    MAP_PRCMPeripheralReset(PRCM_GSPI);

    // set up Timer interrupt
    Timer_IF_Init(PRCM_TIMERA1, TIMERA1_BASE, TIMER_CFG_ONE_SHOT, TIMER_A, 0);
    Timer_IF_IntSetup(TIMERA1_BASE, TIMER_A, ConsecutivePressingHandler);

    Timer_IF_Init(PRCM_TIMERA0, TIMERA0_BASE, TIMER_CFG_PERIODIC_UP, TIMER_A, 0);
    // TimerConfigure(TIMERA0_BASE, TIMER_CFG_PERIODIC_UP);
    TimerEnable(TIMERA0_BASE, TIMER_A);
    TimerValueSet(TIMERA0_BASE, TIMER_A, 0);


    //
    // Register the interrupt handlers
    //
    MAP_GPIOIntRegister(PIN8.port, GPIOA0IntHandler);

    // Configure falling edge interrupts on Pin61 from IR output (remote)
    //
    MAP_GPIOIntTypeSet(PIN8.port, PIN8.pin, GPIO_FALLING_EDGE);   // PIN61

    ulStatus = MAP_GPIOIntStatus (PIN8.port, false);
    MAP_GPIOIntClear(PIN8.port, ulStatus);         // clear interrupts on GPIOA0

    // Enable PIN61 interrupts
    MAP_GPIOIntEnable(PIN8.port, PIN8.pin);



    // clear global variables
    PIN8_intcount = 0;
    PIN8_intflag = 0;
    currentPress = 0;
    previousPress = 1;
    currentButton = -2;
    previousButton = -1;

    Message("\t\t************************************************************\t\t\t\n\r");
    Message("\t\t\tPress the Remote button to see what you pressed\t\t\t\n\r");
    Message("\t\t ************************************************************\t\t\t\n\r");
    Message("\n\n\n\r");

    //SetupCommunication();
    MasterMain();
    fillScreen(BLACK);

    // UART interrupt
    UARTIntEnable( UARTA1_BASE, UART_INT_RX) ;
    UARTIntRegister(UARTA1_BASE, CheckMessage);

    MAP_PRCMPeripheralReset(PRCM_GSPI);



    // main for loop
    while (1) {
    	ProcessIR();
    	DisplayMessage();
    }
}

//*****************************************************************************

