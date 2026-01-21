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
#include "../ProjectHeaders/ADService.h"
#include "../ProjectHeaders/MotorService.h"
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "PIC32_AD_Lib.h"
#include "dbprintf.h"
/*----------------------------- Module Defines ----------------------------*/

#define AD_SAMPLE_PERIOD_MS   20
//#define POT_AN                5
#define POT_CHANNEL_SET       (1U << 5)
#define STEP_INTERVAL_MAX_MS  1023
#define STEP_INTERVAL_MIN_MS  1

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
static uint16_t MapADCToInterval(uint16_t adc);
/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
static uint32_t ADCResults[8];
static uint16_t LastADC = 0;
static uint16_t StepIntervalMs = 50;



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
bool InitADService(uint8_t Priority)
{
  ES_Event_t ThisEvent;
  MyPriority = Priority;

  bool ok = ADC_ConfigAutoScan(POT_CHANNEL_SET);
  if (!ok) return false;

  ES_Timer_InitTimer(AD_TIMER, AD_SAMPLE_PERIOD_MS);

  ThisEvent.EventType = ES_INIT;
  return ES_PostToService(MyPriority, ThisEvent);
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
bool PostADService(ES_Event_t ThisEvent)
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
ES_Event_t RunADService(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT;

  switch (ThisEvent.EventType)
  {
    case ES_INIT:
      ADC_MultiRead(ADCResults);
      LastADC = (uint16_t)ADCResults[0];
      StepIntervalMs = MapADCToInterval(LastADC);
      break;

    case ES_TIMEOUT:
      //DB_printf("Start Measure\n");
      if (ThisEvent.EventParam == AD_TIMER)
      {
        ADC_MultiRead(ADCResults);
        
        LastADC = (uint16_t)ADCResults[0];
        //DB_printf("%d\n",LastADC);
        StepIntervalMs = MapADCToInterval(LastADC);

        ES_Timer_InitTimer(AD_TIMER, AD_SAMPLE_PERIOD_MS);
        
        
        ES_Event_t NewEvent;
        NewEvent.EventType   = ES_AD;
        NewEvent.EventParam  = StepIntervalMs;
        PostMotorService(NewEvent);
        
      }
      break;

    default:
      break;
  }

  return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/

static uint16_t MapADCToInterval(uint16_t adc)
{
  const uint16_t adcMax = 1023;

  if (adc > adcMax) adc = adcMax;

  uint32_t span = (uint32_t)(STEP_INTERVAL_MAX_MS - STEP_INTERVAL_MIN_MS);
  //uint32_t interval = (uint32_t)STEP_INTERVAL_MAX_MS - ((uint32_t)adc * span) / adcMax;
  uint32_t interval = (adc * span) / adcMax + STEP_INTERVAL_MIN_MS;
  //if (interval < STEP_INTERVAL_MIN_MS) interval = STEP_INTERVAL_MIN_MS;
  //if (interval > STEP_INTERVAL_MAX_MS) interval = STEP_INTERVAL_MAX_MS;

  return (uint16_t)interval;
}

uint16_t ADService_GetStepIntervalMs(void)
{
  return StepIntervalMs;
}

uint16_t ADService_GetLastADC(void)
{
  return LastADC;
}




/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

