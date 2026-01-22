#include "EncoderIC.h"
#include <xc.h>
#include <sys/attribs.h>
#include <stdbool.h>
#include <stdint.h>

#define PBCLK_HZ 20000000UL

#define T3_PRESCALE_BITS 0b010
#define T3_PRESCALE_VAL  4UL

static volatile uint32_t t3_overflows  = 0;
static volatile uint32_t last_ts32     = 0;
static volatile uint32_t period_ticks32 = 0;
static volatile bool     new_edge      = false;

static volatile uint16_t period_ticks16 = 0;

static inline void PPS_Unlock(void){
    SYSKEY = 0;
    SYSKEY = 0xAA996655;
    SYSKEY = 0x556699AA;
    CFGCONbits.IOLOCK = 0;
}
static inline void PPS_Lock(void){
    CFGCONbits.IOLOCK = 1;
    SYSKEY = 0;
}

void EncoderIC_Init(void)
{
    TRISBbits.TRISB9 = 1;

    T3CONbits.ON = 0;
    T3CONbits.TCS = 0;
    TMR3  = 0;
    PR3   = 0xFFFF;
    T3CONbits.TCKPS = T3_PRESCALE_BITS;
    T3CONbits.ON    = 1;

    IPC3bits.T3IP = 3;
    IPC3bits.T3IS = 0;
    IFS0CLR = _IFS0_T3IF_MASK;
    IEC0bits.T3IE = 1;

    PPS_Unlock();
    IC2R = 0b0100;
    PPS_Lock();

    IC2CON = 0;
    IC2CONbits.ON = 0;

    IC2CONbits.ICTMR = 0;

    IC2CONbits.C32 = 0;
    IC2CONbits.ICI = 0;
    IC2CONbits.ICM = 0b011;

    while (IC2CONbits.ICBNE) { (void)IC2BUF; }

    IPC2bits.IC2IP = 4;
    IPC2bits.IC2IS = 0;
    IFS0CLR = _IFS0_IC2IF_MASK;
    IEC0bits.IC2IE = 1;

    IC2CONbits.ON = 1;

    __builtin_enable_interrupts();
}

uint32_t EncoderIC_GetPeriodTicks32(bool *newEdge)
{
    uint32_t p;
    bool ne;

    __builtin_disable_interrupts();
    p  = period_ticks32;
    ne = new_edge;
    new_edge = false;
    __builtin_enable_interrupts();

    if (newEdge) *newEdge = ne;
    return p;
}

uint16_t EncoderIC_GetPeriodTicks16(bool *newEdge)
{
    uint16_t p;
    bool ne;

    __builtin_disable_interrupts();
    p  = period_ticks16;
    ne = new_edge;
    new_edge = false;
    __builtin_enable_interrupts();

    if (newEdge) *newEdge = ne;
    return p;
}


void __ISR(_TIMER_3_VECTOR, IPL3SOFT) T3Handler(void)
{
    IFS0CLR = _IFS0_T3IF_MASK;
    t3_overflows++;
}

void __ISR(_INPUT_CAPTURE_2_VECTOR, IPL4SOFT) IC2Handler(void)
{
    IFS0CLR = _IFS0_IC2IF_MASK;

    while (IC2CONbits.ICBNE) {
        uint16_t cap = IC2BUF;

        uint32_t of = t3_overflows;

        if ((IFS0bits.T3IF != 0) && (cap < 0x8000)) {
            of++;
        }

        uint32_t ts = (of << 16) | (uint32_t)cap;

        uint32_t dt = ts - last_ts32;
        last_ts32 = ts;

        period_ticks32 = dt;
        period_ticks16 = (uint16_t)dt;
        new_edge = true;
    }
}