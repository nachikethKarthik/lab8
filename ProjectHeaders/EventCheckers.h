/****************************************************************************
 Module
     EventCheckers.h
 Description
     header file for the event checking functions
 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 20260204       karthi24    added event checker for tape and beacon detection
 10/18/15 11:50 jec      added #include for stdint & stdbool
 08/06/13 14:37 jec      started coding
*****************************************************************************/

#ifndef EventCheckers_H
#define EventCheckers_H

// the common headers for C99 types
#include <stdint.h>
#include <stdbool.h>

// prototypes for event checkers

bool Check4Keystroke(void);
bool Check4Tape(void);
bool Check4Beacon(void);

bool InitEventCheckerHardware(void);




#endif /* EventCheckers_H */
