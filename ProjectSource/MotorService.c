/****************************************************************************
 Module
   TemplateService.c

 Revision
   1.0.1

 Description
   This is a template file for implementing a simple service under the
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/16/12 09:58 jec      began conversion from TemplateFSM.c
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "../ProjectHeaders/MotorService.h"
#include "../ProjectHeaders/EncoderIC.h"
#include "ADService.h"

#include <xc.h>
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_DeferRecall.h"
#include "ES_Port.h"
#include "terminal.h"
#include "dbprintf.h"

/*----------------------------- Module Defines ----------------------------*/

#define PBCLK_HZ 20000000UL

#define PWM_FREQ_HZ 10000UL
#define T2_PRESCALE  4
#define ENCODER_CHECK_MS 10
#define T3_PRESCALE  4
#define PPR 512
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
static void InitPWM(void);
static void SetDuty(uint8_t duty_present);
static void InitLED(void);
static void LED_CLR(void);
static void MapPeriodToBars(uint32_t period_LED);
/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
static uint16_t pr2_value = 0;
static uint16_t duty_counts = 0;
static uint16_t adc;
static uint16_t duty;
static bool newEdge;
static uint32_t period;
/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitTemplateService

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any
     other required initialization for this service
 Notes

 Author
     J. Edward Carryer, 01/16/12, 10:00
****************************************************************************/
bool InitMotorService(uint8_t Priority)
{
  ES_Event_t ThisEvent;
  //DB_printf("Start\n");

  MyPriority = Priority;
  /********************************************
   in here you write your initialization code
   *******************************************/
  // post the initial transition event
  TRISBbits.TRISB9 = 1;
  
  TRISBbits.TRISB8 = 0;
  
  TRISBbits.TRISB10 = 0;
  TRISBbits.TRISB11 = 0;
  LATBbits.LATB10 = 1;
  LATBbits.LATB11 = 0;
  
  InitLED();
  TRISBbits.TRISB15 = 0;
  TRISBbits.TRISB13 = 0;
  TRISBbits.TRISB12 = 0;
  InitPWM();
  SetDuty(0);
  EncoderIC_Init();
  
  ES_Timer_InitTimer(ENCODER_TIMER, ENCODER_CHECK_MS);
  ES_Timer_InitTimer(PRINT_TIMER, 100);
  ThisEvent.EventType = ES_INIT;
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
     PostTemplateService

 Parameters
     EF_Event_t ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostMotorService(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunTemplateService

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes

 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event_t RunMotorService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

  switch(ThisEvent.EventType){
      case ES_INIT:
          break;
                
      case ES_AD:
          adc = ThisEvent.EventParam;
          //DB_printf("%d\n",adc);
          duty = ((uint32_t)adc * 100UL) / 1023UL;
          SetDuty((uint8_t)duty);
          break;
          
      case ES_TIMEOUT:
          if (ThisEvent.EventParam == ENCODER_TIMER){
              ES_Timer_InitTimer(ENCODER_TIMER, ENCODER_CHECK_MS);
              
              period = EncoderIC_GetPeriodTicks32(&newEdge);
              //DB_printf("period = %d, newEdge = %d, RB9 output = %d\n",period,newEdge, PORTBbits.RB9);
              if(newEdge) MapPeriodToBars(period);
              
              break;
              
          }else if (ThisEvent.EventParam == PRINT_TIMER){
              ES_Timer_InitTimer(PRINT_TIMER, 100);
              LATBbits.LATB13 = 1;
              uint16_t RPM = (60 * PBCLK_HZ)/(T3_PRESCALE * period * PPR);
              LATBbits.LATB13 = 0;
              LATBbits.LATB12 = 1;
              DB_printf("Duty_cycle is %d, RPM is %d\n", duty, RPM);
              LATBbits.LATB12 = 0;
          }
          
          
      default:
          break;    
  }
  
  return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/

static void InitPWM(void){
    T2CONbits.ON = 0;
    T2CONbits.TCS = 0;
    
    //T2CON = 0;
    TMR2 = 0;
    
    pr2_value = (uint16_t)((PBCLK_HZ/(T2_PRESCALE * PWM_FREQ_HZ)) - 1);
    PR2 = pr2_value;
    T2CONbits.TCKPS = 0b010;  //pre-scale = 4 
    T2CONbits.ON = 1;
    
    OC2CONbits.ON = 0;
    OC2R = 0;
    OC2RS = 0;
    OC2CONbits.OCTSEL = 0;
    OC2CONbits.OCM = 0b110;
    RPB8Rbits.RPB8R = 0b0101;
    OC2CONbits.ON = 1;
    
}

static void SetDuty(uint8_t duty_precent){
    if (duty_precent > 100) duty_precent = 100;
    
    duty_counts = (uint16_t)(((uint32_t)duty_precent * (uint32_t)(pr2_value + 1U))/100UL);
    
    OC2RS = duty_counts;
       
}

static void InitLED(void){
    TRISAbits.TRISA0 = 0;
    TRISAbits.TRISA1 = 0;
    TRISAbits.TRISA2 = 0;
    TRISAbits.TRISA3 = 0;
    TRISAbits.TRISA4 = 0;
    TRISBbits.TRISB2 = 0;
    TRISBbits.TRISB4 = 0;
    TRISBbits.TRISB5 = 0;
    
    LATAbits.LATA0 = 0;
    LATAbits.LATA1 = 0;
    LATAbits.LATA2 = 0;
    LATAbits.LATA3 = 0;
    LATAbits.LATA4 = 0;
    LATBbits.LATB2 = 0;
    LATBbits.LATB4 = 0;
    LATBbits.LATB5 = 0;
}

static void LED_CLR(void){
    LATAbits.LATA0 = 0;
    LATAbits.LATA1 = 0;
    LATAbits.LATA2 = 0;
    LATAbits.LATA3 = 0;
    LATAbits.LATA4 = 0;
    LATBbits.LATB2 = 0;
    LATBbits.LATB4 = 0;
    LATBbits.LATB5 = 0;
}

static void MapPeriodToBars(uint32_t period_LED){
    //DB_printf("%d\n",period_LED);
    LATBbits.LATB15 = 1;
    if (period_LED <= 2200){
        LATAbits.LATA0 = 1;
        LATAbits.LATA1 = 0;
        LATAbits.LATA2 = 0;
        LATAbits.LATA3 = 0;
        LATAbits.LATA4 = 0;
        LATBbits.LATB2 = 0;
        LATBbits.LATB4 = 0;
        LATBbits.LATB5 = 0;
        //DB_printf("1\n");
    }else if(2200 < period_LED && period_LED <= 2350){
        LATAbits.LATA0 = 1;
        LATAbits.LATA1 = 1;
        LATAbits.LATA2 = 0;
        LATAbits.LATA3 = 0;
        LATAbits.LATA4 = 0;
        LATBbits.LATB2 = 0;
        LATBbits.LATB4 = 0;
        LATBbits.LATB5 = 0;
        //DB_printf("2\n");
    }else if(2350 < period_LED && period_LED <= 2600){
        LATAbits.LATA0 = 1;
        LATAbits.LATA1 = 1;
        LATAbits.LATA2 = 1;
        LATAbits.LATA3 = 0;
        LATAbits.LATA4 = 0;
        LATBbits.LATB2 = 0;
        LATBbits.LATB4 = 0;
        LATBbits.LATB5 = 0;
        //DB_printf("3\n");
    }else if(2600 < period_LED && period_LED <= 2950){
        LATAbits.LATA0 = 1;
        LATAbits.LATA1 = 1;
        LATAbits.LATA2 = 1;
        LATAbits.LATA3 = 1;
        LATAbits.LATA4 = 0;
        LATBbits.LATB2 = 0;
        LATBbits.LATB4 = 0;
        LATBbits.LATB5 = 0;
        //DB_printf("4\n");
    }else if(2950 < period_LED && period_LED <= 3300){
        LATAbits.LATA0 = 1;
        LATAbits.LATA1 = 1;
        LATAbits.LATA2 = 1;
        LATAbits.LATA3 = 1;
        LATAbits.LATA4 = 1;
        LATBbits.LATB2 = 0;
        LATBbits.LATB4 = 0;
        LATBbits.LATB5 = 0;
    }else if(3300 < period_LED && period_LED <= 3650){

        LATAbits.LATA0 = 1;
        LATAbits.LATA1 = 1;
        LATAbits.LATA2 = 1;
        LATAbits.LATA3 = 1;
        LATAbits.LATA4 = 1;
        LATBbits.LATB2 = 1;
        LATBbits.LATB4 = 0;
        LATBbits.LATB5 = 0;
    }else if(3650 < period_LED && period_LED <= 4000){
        LATAbits.LATA0 = 1;
        LATAbits.LATA1 = 1;
        LATAbits.LATA2 = 1;
        LATAbits.LATA3 = 1;
        LATAbits.LATA4 = 1;
        LATBbits.LATB2 = 1;
        LATBbits.LATB4 = 1;
        LATBbits.LATB5 = 0;
    }else if(4000 < period_LED){
        LATAbits.LATA0 = 1;
        LATAbits.LATA1 = 1;
        LATAbits.LATA2 = 1;
        LATAbits.LATA3 = 1;
        LATAbits.LATA4 = 1;
        LATBbits.LATB2 = 1;
        LATBbits.LATB4 = 1;
        LATBbits.LATB5 = 1;
    }
    LATBbits.LATB15 = 0;
    
}
/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

