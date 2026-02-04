/****************************************************************************
Module
TemplateFSM.c

Revision
1.0.1

Description
This is a template file for implementing flat state machines under the
Gen2 Events and Services Framework.

Notes

History
When Who What/Why
-------------- --- --------
01/15/12 11:12 jec revisions for Gen2 framework
11/07/11 11:26 jec made the queue static
10/30/11 17:59 jec fixed references to CurrentEvent in RunTemplateSM()
10/23/11 18:20 jec began conversion from SMTemplate.c (02/20/07 rev)
 ****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
next lower level in the hierarchy that are sub-machines to this machine
 */
#include "../ProjectHeaders/Lab8_test.h"
#include "../ProjectHeaders/PIC32_SPI_HAL.h"


#include <xc.h>

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_DeferRecall.h"
#include "ES_Port.h"
#include "terminal.h"
#include "dbprintf.h"

/*----------------------------- Module Defines ----------------------------*/

#define PWM_PRESCALE 0b010 // 1:4 prescaler
#define PWM_PERIOD 1249 // For 4000 Hz PWM
#define CG_QUERY_BYTE 0xAA

#define SPI_CLK_NS 10000u

#define CG_SPI_MODULE SPI_SPI1

#define CG_SS_OUT_PIN SPI_RPA0
#define CG_SDO_OUT_PIN SPI_RPA1
#define CG_SDI_IN_PIN SPI_RPB5

#define ROTATION_45_DURATION 250
#define ROTATION_90_DURATION 500
#define FORWARD_DURATION 500
#define BACKWARD_DURATION 500



/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
relevant to the behavior of this state machine
 */
static bool CG_SPI_Init(void);
static bool Motor_PWM_Init(void);
bool InitEventCheckerHardware(void);
void StopMotors(void);
void M1Forward(uint16_t duty);
void M2Forward(uint16_t duty);
void M1Backward(uint16_t duty);
void M2Backward(uint16_t duty);
void RotateCW(uint16_t duty);
void RotateCCW(uint16_t duty);



/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
static TemplateState_t CurrentState;

// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;
uint8_t CG_command = 0x00;
//static uint32_t TapeADCResults[1]; // Used for testing tape sensor
/*------------------------------ Module Code ------------------------------*/

/****************************************************************************
 Function
     InitLab8Service

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 18:55
****************************************************************************/
bool InitLab8Service(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  // put us into the Initial PseudoState
  CurrentState = InitPState;
  // post the initial transition event
  ThisEvent.EventType = ES_INIT;
  
  CG_SPI_Init();
  Motor_PWM_Init();
  InitEventCheckerHardware();
  
  ES_Timer_InitTimer(QUERY_TIMER, 100);

  if (ES_PostToService(MyPriority, ThisEvent) == true)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/****************************************************************************
 Function
     PostLab8FSM

Parameters
EF_Event_t ThisEvent , the event to post to the queue

Returns
boolean False if the Enqueue operation failed, True otherwise

Description
Posts an event to this state machine's queue
Notes

Author
J. Edward Carryer, 10/23/11, 19:25
 ****************************************************************************/
bool PostLab8Service(ES_Event_t ThisEvent) {
    return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunLab8Service

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event_t, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes
   uses nested switch/case to implement the machine.
 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event_t RunLab8Service(ES_Event_t ThisEvent)
{
    ES_Event_t ReturnEvent = { .EventType = ES_NO_EVENT };

    switch (CurrentState)
    {
        case InitPState:
        {
            if (ThisEvent.EventType == ES_INIT)
            {
                CurrentState = Waiting;
                ES_Timer_InitTimer(QUERY_TIMER, 100);
            }
        }
        break;

        case Waiting:
        {
            if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == QUERY_TIMER)
            {
                SPIOperate_SPI1_Send8Wait(CG_QUERY_BYTE);
                CG_command = (uint8_t)SPI1BUF;

                switch (CG_command)
                {
                    case 0x00:
                        StopMotors();
                        break;

                    case 0x02: // CW 90
                        RotateCW(100);
                        ES_Timer_InitTimer(ROTATION_TIMER, ROTATION_90_DURATION);
                        CurrentState = RotatingClockwise;
                        break;

                    case 0x03: // CW 45
                        RotateCW(100);
                        ES_Timer_InitTimer(ROTATION_TIMER, ROTATION_45_DURATION);
                        CurrentState = RotatingClockwise;
                        break;

                    case 0x04: // CCW 90
                        RotateCCW(100);
                        ES_Timer_InitTimer(ROTATION_TIMER, ROTATION_90_DURATION);
                        CurrentState = RotatingCounterClockwise;
                        break;

                    case 0x05: // CCW 45
                        RotateCCW(100);
                        ES_Timer_InitTimer(ROTATION_TIMER, ROTATION_45_DURATION);
                        CurrentState = RotatingCounterClockwise;
                        break;

                    case 0x08: // forward half
                        M1Forward(50);
                        M2Forward(50);
                        ES_Timer_InitTimer(FORWARD_TIMER, FORWARD_DURATION);
                        CurrentState = DrivingForward;
                        break;

                    case 0x09: // forward full
                        M1Forward(100);
                        M2Forward(100);
                        ES_Timer_InitTimer(FORWARD_TIMER, FORWARD_DURATION);
                        CurrentState = DrivingForward;
                        break;

                    case 0x10: // reverse half
                        M1Backward(50);
                        M2Backward(50);
                        ES_Timer_InitTimer(BACKWARD_TIMER, BACKWARD_DURATION);
                        CurrentState = DrivingBackward;
                        break;

                    case 0x11: // reverse full
                        M1Backward(100);
                        M2Backward(100);
                        ES_Timer_InitTimer(BACKWARD_TIMER, BACKWARD_DURATION);
                        CurrentState = DrivingBackward;
                        break;

                    case 0x40: // forward until tape
                        M1Forward(100);
                        M2Forward(100);
                        CurrentState = DrivingForward;
                        break;

                    default:
                        break;
                }

                ES_Timer_InitTimer(QUERY_TIMER, 100);
            }
        }
        break;

        case DrivingForward:
        {
            if (ThisEvent.EventType == ES_TAPE_DETECTED)
            {
                StopMotors();
                CurrentState = Waiting;
            }
            else if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == FORWARD_TIMER)
            {
                StopMotors();
                CurrentState = Waiting;
            }
        }
        break;

        case DrivingBackward:
        {
            if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == BACKWARD_TIMER)
            {
                StopMotors();
                CurrentState = Waiting;
            }
        }
        break;

        case RotatingClockwise:
        {
            if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == ROTATION_TIMER)
            {
                StopMotors();
                CurrentState = Waiting;
            }
        }
        break;

        case RotatingCounterClockwise:
        {
            if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == ROTATION_TIMER)
            {
                StopMotors();
                CurrentState = Waiting;
            }
        }
        break;

        default:
            break;
    }

    return ReturnEvent;
}

/****************************************************************************
Function
QueryTemplateSM

Parameters
None

Returns
TemplateState_t The current state of the Template state machine

Description
returns the current state of the Template state machine
Notes

Author
J. Edward Carryer, 10/23/11, 19:21
 ****************************************************************************/
TemplateState_t QueryTemplateFSM(void) {
    return CurrentState;
}

/***************************************************************************
private functions
 ***************************************************************************/
static bool CG_SPI_Init(void) {
    
    SPISetup_BasicConfig(CG_SPI_MODULE);

    // Leader/Master
    SPISetup_SetLeader(CG_SPI_MODULE, SPI_SMP_END);

    // Bit time
    SPISetup_SetBitTime(CG_SPI_MODULE, SPI_CLK_NS);

    // Start with a common SPI mode:
    // CKP=0 idle low, CKE=1 first edge (often corresponds to Mode 0 behavior on PIC32)
    SPISetup_SetClockIdleState(CG_SPI_MODULE, SPI_CLK_HI);
    SPISetup_SetActiveEdge(CG_SPI_MODULE, SPI_SECOND_EDGE);

    // 8-bit transfers (Command Generator is byte-oriented)
    SPISetup_SetXferWidth(CG_SPI_MODULE, SPI_8BIT);

    // Enhanced buffer optional; start OFF for simplest behavior
    SPISetEnhancedBuffer(CG_SPI_MODULE, false);

    // Map SS/SDO (MSSEN will toggle SS each transfer)
    SPISetup_MapSSOutput(CG_SPI_MODULE, CG_SS_OUT_PIN);
    SPISetup_MapSDOutput(CG_SPI_MODULE, CG_SDO_OUT_PIN);
    SPISetup_MapSDInput(CG_SPI_MODULE, CG_SDI_IN_PIN);
    // - PIC SDO -> CG SDI
    // - PIC SDI <- CG SDO
    // - PIC SCK -> CG SCK
    // - PIC SS -> CG SS

    SPISetup_EnableSPI(CG_SPI_MODULE);

    return true;
}

static bool Motor_PWM_Init(void) {
    // Motor 1 init. Set all to output
    TRISBbits.TRISB8 = 0; // Connects to Enable 1,2
    TRISBbits.TRISB9 = 0; // Connects to 1A
    TRISBbits.TRISB10 = 0; // Connects to 2A
    
    // Motor 2 init. Set all to output
    TRISBbits.TRISB11 = 0; // Connects to Enable 3,4
    TRISBbits.TRISB12 = 0; // Connects to 3A
    TRISBbits.TRISB13 = 0; // Connects to 4A
    
    // Disable analog, turn everything to digital
    ANSELBbits.ANSB12 = 0;
    ANSELBbits.ANSB13 = 0;
    
    // Timer2 Setup
    T2CONbits.ON = 0; // Disable Timer2
    T2CONbits.TCS = 0; // Select internal PBCLK
    T2CONbits.TCKPS = PWM_PRESCALE;
    TMR2 = 0; // Clear timer register
    PR2 = PWM_PERIOD; // Set priod for 4000 hz
    T2CONbits.ON = 1; // Enable Timer2
    
    // Output Compare Module 3 Setup
    OC3CONbits.ON = 0; // Disable OC3 during setup
    OC3CONbits.OCM = 0b110; // PWM mode, fault pin disabled
    OC3CONbits.OCTSEL = 0; // Selects Timer2 as clock source
    OC3RS = 0; // Set initial duty cycle to 0%
    OC3R = 0; // Primary compare register
    RPB9Rbits.RPB9R = 0b0101; // Map OC3 (RB9) to 1A
    OC3CONbits.ON = 1; // Enable OC3
    
    // OC Module 4 Setup
    OC4CONbits.ON = 0; // Disable OC4 during setup
    OC4CONbits.OCM = 0b110; // PWM mode, fault pin disabled
    OC4CONbits.OCTSEL = 0; // Selects Timer2 as clock source
    OC4RS = 0; // Set initial duty cycle to 0%
    OC4R = 0; // Primary compare register
    RPB13Rbits.RPB13R = 0b0101; // Map OC4 (RB13) to 4A
    OC4CONbits.ON = 1; // Enable OC4
    
    // Set initial state off, enable pins are 0
    LATBbits.LATB8 = 0;
    LATBbits.LATB12 = 0;
    return true;
}

void StopMotors(void) {
    OC3RS = 0;
    OC4RS = 0;
    LATBbits.LATB8 = 0; // disable motor 1
    LATBbits.LATB11 = 0; // disable motor 2
}

// void MotorsForward(uint16_t duty)
// {
// OC3RS = duty;
// OC4RS = duty;
// LATBbits.LATB10 = 0; // direction forward
// LATBbits.LATB12 = 0; // direction forward
// LATBbits.LATB8 = 1; // enable
// LATBbits.LATB11 = 1; // enable
// }

void M1Forward(uint16_t duty) {
    OC3RS = (duty * PWM_PERIOD) / 100;      
    LATBbits.LATB10 = 0; // direction forward
    LATBbits.LATB8 = 1; // enable
}

void M2Forward(uint16_t duty) {
    OC4RS = (duty * PWM_PERIOD) / 100;
    LATBbits.LATB12 = 0; // direction forward
    LATBbits.LATB11 = 1; // enable
}

// void MotorsBackward(uint16_t duty)
// {
// OC3RS = 1 - duty; // OR PWM_PERIOD - DUTY * PWM_PERIOD
// OC4RS = 1 - duty;
// LATBbits.LATB10 = 1; // direction backward
// LATBbits.LATB12 = 1; // direction backward
// LATBbits.LATB8 = 1; // enable
// LATBbits.LATB11 = 1; // enable
// }

void M1Backward(uint16_t duty) {
    OC3RS = PWM_PERIOD - ((duty / 100) * PWM_PERIOD);
    LATBbits.LATB10 = 1; // direction backward
    LATBbits.LATB8 = 1; // enable
}

void M2Backward(uint16_t duty) {
    OC4RS = PWM_PERIOD - ((duty / 100) * PWM_PERIOD);
    LATBbits.LATB12 = 1; // direction backward
    LATBbits.LATB11 = 1; // enable
}

void RotateCW(uint16_t duty) {
    M1Forward(duty); // left wheel
    M2Backward(duty); // right wheel
}

void RotateCCW(uint16_t duty) {
    M1Backward((duty / 100) * PWM_PERIOD);       // left wheel
    M2Backward((duty / 100) * PWM_PERIOD);       // right wheel
}
