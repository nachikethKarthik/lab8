/****************************************************************************

  Header file for template service
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef EnCoderIC_H
#define EnCoderIC_H

#include "ES_Types.h"
#include <xc.h>
#include <sys/attribs.h>
//#include <stdbool.h>
//#include <stdint.h>

void EncoderIC_Init(void);
uint16_t EncoderIC_GetPeriodTicks16(bool *newEdge);
uint32_t EncoderIC_GetPeriodTicks32(bool *newEdge);

#endif

