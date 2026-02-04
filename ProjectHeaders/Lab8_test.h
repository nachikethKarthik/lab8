/****************************************************************************

  Header file for template Flat Sate Machine
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef LAB8_H
#define LAB8_H

// Event Definitions
#include <stdint.h>
#include <stdbool.h>

#include "ES_Events.h"
#include "ES_Port.h"    /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum
{
  InitPState, UnlockWaiting, _1UnlockPress,
  _2UnlockPresses, Locked
}TemplateState_t;

// Public Function Prototypes

bool InitLab8Service(uint8_t Priority);
bool PostLab8Service(ES_Event_t ThisEvent);
ES_Event_t RunLab8Service(ES_Event_t ThisEvent);
TemplateState_t QueryLab8Service(void);

#endif /* FSMTemplate_H */

