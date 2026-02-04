/****************************************************************************
 Module
   EventCheckers.c

 Revision
   1.0.2

 Description
   This is the sample for writing event checkers along with the event
   checkers used in the basic framework test harness.

 Notes
   Note the use of static variables in sample event checker to detect
   ONLY transitions.

 History
 When           Who     What/Why
 -------------- ---     --------
 02032026       karthi24    started coding check4beacon and check4tape
 08/06/13 13:36 jec     initial version
****************************************************************************/

// this will pull in the symbolic definitions for events, which we will want
// to post in response to detecting events
#include "ES_Configure.h"
// This gets us the prototype for ES_PostAll
#include "ES_Framework.h"
// this will get us the structure definition for events, which we will need
// in order to post events in response to detecting events
#include "ES_Events.h"
// if you want to use distribution lists then you need those function
// definitions too.
#include "ES_PostList.h"
// This include will pull in all of the headers from the service modules
// providing the prototypes for all of the post functions
#include "ES_ServiceHeaders.h"
// this test harness for the framework references the serial routines that
// are defined in ES_Port.c
#include "ES_Port.h"
// include our own prototypes to insure consistency between header &
// actual functionsdefinition
#include "EventCheckers.h"

#include "../ProjectHeaders/PIC32_AD_Lib.h"
#include "dbprintf.h"

// This is the event checking function sample. It is not intended to be
// included in the module. It is only here as a sample to guide you in writing
// your own event checkers
#if 0
/****************************************************************************
 Function
   Check4Lock
 Parameters
   None
 Returns
   bool: true if a new event was detected
 Description
   Sample event checker grabbed from the simple lock state machine example
 Notes
   will not compile, sample only
 Author
   J. Edward Carryer, 08/06/13, 13:48
****************************************************************************/
bool Check4Lock(void)
{
  static uint8_t  LastPinState = 0;
  uint8_t         CurrentPinState;
  bool            ReturnVal = false;

  CurrentPinState = LOCK_PIN;
  // check for pin high AND different from last time
  // do the check for difference first so that you don't bother with a test
  // of a port/variable that is not going to matter, since it hasn't changed
  if ((CurrentPinState != LastPinState) &&
      (CurrentPinState == LOCK_PIN_HI)) // event detected, so post detected event
  {
    ES_Event ThisEvent;
    ThisEvent.EventType   = ES_LOCK;
    ThisEvent.EventParam  = 1;
    // this could be any of the service post functions, ES_PostListx or
    // ES_PostAll functions
    ES_PostAll(ThisEvent);
    ReturnVal = true;
  }
  LastPinState = CurrentPinState; // update the state for next time

  return ReturnVal;
}

#endif

/*----------------------------- Module Defines ----------------------------*/

// Tape sensor threshold - adjust this based on actual sensor readings
#define TAPE_THRESHOLD 120  

// Beacon detection parameters
// Beacon frequency is 1427 Hz, so period is ~700 us
// We'll count edges over a window and check if we see enough pulses
#define BEACON_EDGE_WINDOW_MS  10   // Time window to count edges (ms)
#define BEACON_MIN_EDGES       5   // Minimum edges to detect beacon (1427 Hz * 0.01s * 2 edges/cycle = ~28)

/*---------------------------- Module Variables ---------------------------*/
// For tape detection - track last state to detect transitions
static uint8_t LastTapeState = 0;  // 0 = no tape, 1 = tape detected

// For beacon detection - track last state
static uint8_t LastBeaconState = 0;  // 0 = no beacon, 1 = beacon detected
static uint8_t LastRA2State = 0;     // For edge detection
static uint32_t EdgeCount = 0;       // Count of edges seen
static uint32_t LastEdgeCheckTime = 0;  // For timing the edge window

// ADC results array - index 0 will have AN9 result
static uint32_t ADCResults[1];
/****************************************************************************
 Function
   Check4Keystroke
 Parameters
   None
 Returns
   bool: true if a new key was detected & posted
 Description
   checks to see if a new key from the keyboard is detected and, if so,
   retrieves the key and posts an ES_NewKey event to TestHarnessService0
 Notes
   The functions that actually check the serial hardware for characters
   and retrieve them are assumed to be in ES_Port.c
   Since we always retrieve the keystroke when we detect it, thus clearing the
   hardware flag that indicates that a new key is ready this event checker
   will only generate events on the arrival of new characters, even though we
   do not internally keep track of the last keystroke that we retrieved.
 Author
   J. Edward Carryer, 08/06/13, 13:48
****************************************************************************/
bool Check4Keystroke(void)
{
  if (IsNewKeyReady())   // new key waiting?
  {
    ES_Event_t ThisEvent;
    ThisEvent.EventType   = ES_NEW_KEY;
    ThisEvent.EventParam  = GetNewKey();
    ES_PostAll(ThisEvent);
    return true;
  }
  return false;
}

/****************************************************************************
 Function
   Check4Tape
 Parameters
   None
 Returns
   bool: true if tape detection state changed and event was posted
 Description
   Reads the analog value from the tape sensor on RB15 (AN9) and posts
   an ES_TAPE_DETECTED event when tape is newly detected.
 Notes
   - Threshold may need adjustment
 Author
   karthi24, 02032026
****************************************************************************/
bool Check4Tape(void)
{
  bool ReturnVal = false;
  uint8_t CurrentTapeState;
  
  // Read the ADC value from AN9
  ADC_MultiRead(ADCResults);
  uint32_t TapeSensorValue = ADCResults[0];
  
  // Determine current tape state based on threshold
  // Adjust the comparison direction based on your sensor behavior
  if (TapeSensorValue < TAPE_THRESHOLD)
  {
    CurrentTapeState = 0;  // No tape detected
  }
  else
  {
    CurrentTapeState = 1;  // Tape detected 
  }
  
  // Check for transition: no tape -> tape detected
  if ((CurrentTapeState != LastTapeState) && (CurrentTapeState == 1))
  {
    // Post tape detected event
    ES_Event_t ThisEvent;
    ThisEvent.EventType = ES_TAPE_DETECTED;
    ThisEvent.EventParam = (uint16_t)TapeSensorValue;  // Include ADC value as param
    ES_PostAll(ThisEvent);
    
    DB_printf("Tape Event\n");
    ReturnVal = true;
  }
  
  // Update last state for next time
  LastTapeState = CurrentTapeState;
  
  return ReturnVal;
}

/****************************************************************************
 Function
   InitEventCheckerHardware
 Parameters
   None
 Returns
   bool: true if initialization successful
 Description
   Initializes the hardware required for the event checkers:
   - ADC for tape sensor (AN9/RB15)
   - Digital input for beacon detector (RA2)
 Notes
   
 Author
   karthi24, 02032026
****************************************************************************/
bool InitEventCheckerHardware(void)
{
  // Configure RB15 as analog input for tape sensor (AN9)
  // Set as input
  TRISBbits.TRISB15 = 1;
  // Disable digital input buffer (enable analog)
  ANSELBbits.ANSB15 = 1;
  
  // Configure RA2 as digital input for beacon detector
  TRISAbits.TRISA2 = 1;
  // Make sure it's digital (disable analog if applicable)
  // Note: RA2 doesn't have an analog function on PIC32MX170F256B
  
  // Initialize ADC for AN9
  // BIT9HI selects AN9
  if (!ADC_ConfigAutoScan(BIT9HI))
  {
    return false;  // ADC configuration failed
  }
  

  
  return true;
}