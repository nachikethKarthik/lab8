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
//#include "../ProjectHeaders/EncoderIC.h"
#include "ADService.h"

#include <xc.h>
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_DeferRecall.h"
#include "ES_Port.h"
#include "terminal.h"
#include "dbprintf.h"
#include <sys/attribs.h>

/*----------------------------- Module Defines ----------------------------*/

#define PBCLK_HZ 20000000UL

#define PWM_FREQ_HZ 7000UL
#define T2_PRESCALE  4
#define ENCODER_CHECK_MS 10
#define T3_PRESCALE  4
#define PPR 512


#define CTRL_TS_MS 2U
#define CTRL_TS_S 0.002f

#define MAX_RPM_CMD   350
#define KP    0.08f
#define KI    0.30f

#define INT_MIN   0.0f
#define INT_MAX   100.0f
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
static void InitPWM(void);
static void SetDuty(uint8_t duty_present);
static void InitControlISR(void);
/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
static uint16_t pr2_value = 0;
static uint16_t duty_counts = 0;
static uint16_t adc;
static uint16_t duty;
static bool newEdge;
static uint32_t period;


static volatile uint16_t speed_cmd_rpm = 0;
static volatile uint16_t speed_meas_rpm = 0;
static volatile uint8_t duty_out = 0;
static volatile float integ = 0.0f;
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
  
  TRISBbits.TRISB15 = 0;
  TRISBbits.TRISB13 = 0;
  TRISBbits.TRISB12 = 0;
  
  TRISBbits.TRISB15 = 0;
  
  TRISBbits.TRISB5 = 1; //Dir change
  
  
  InitPWM();
  
  SetDuty(0);
  EncoderIC_Init();
  InitControlISR();
  
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
          //duty = ((uint32_t)adc * 100UL) / 1023UL;
          speed_cmd_rpm = (uint16_t)(((uint32_t)adc * (uint32_t)MAX_RPM_CMD)/1023UL);
          
          //SetDuty((uint8_t)duty);
          
          /*
          if (PORTBbits.RB5 == 1){
              LATBbits.LATB11 = 1;
              SetDuty(100 - (uint8_t)duty);
          }else if (PORTBbits.RB5 == 0){
              LATBbits.LATB11 = 0;
              SetDuty((uint8_t)duty);
          }
          */
          
          break;
          
      case ES_TIMEOUT:
          if (ThisEvent.EventParam == ENCODER_TIMER){
              ES_Timer_InitTimer(ENCODER_TIMER, ENCODER_CHECK_MS);
              
              //period = EncoderIC_GetPeriodTicks32(&newEdge);
              //DB_printf("period = %d, newEdge = %d, RB9 output = %d\n",period,newEdge, PORTBbits.RB9);
              //DB_printf("RB5 output = %d\n",PORTBbits.RB5);
              if (PORTBbits.RB5 == 1){
                  LATBbits.LATB11 = 1;
              }else if (PORTBbits.RB5 == 0){
                  LATBbits.LATB11 = 0;
              }
              
              
              break;
              
          }else if (ThisEvent.EventParam == PRINT_TIMER){
              ES_Timer_InitTimer(PRINT_TIMER, 100);
              LATBbits.LATB13 = 1;
              //uint16_t RPM = (60 * PBCLK_HZ)/(T3_PRESCALE * period * PPR);
              LATBbits.LATB13 = 0;
              LATBbits.LATB12 = 1;
              DB_printf("Duty_cycle is %d, CMD_RPM is %d, Real_RPM is %d\n", duty_out, speed_cmd_rpm,speed_meas_rpm);
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

}
/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

