/****************************************************************************

  Header file for template service
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef ADService_H
#define ADService_H

#include <stdint.h>
#include <stdbool.h>

#include "ES_Types.h"
#include "ES_Events.h"
#include "ES_Port.h"
#include "PIC32_AD_Lib.h"
// Public Function Prototypes

//uint16_t StepInterval;

bool InitADService(uint8_t Priority);
bool PostADService(ES_Event_t ThisEvent);
ES_Event_t RunADService(ES_Event_t ThisEvent);
uint16_t ADService_GetStepIntervalMs(void);
uint16_t ADService_GetLastADC(void);

#endif /* ServTemplate_H */

