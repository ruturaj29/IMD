


#ifndef _EXT_INTERRUPTS_H_
#define _EXT_INTERRUPTS_H_


#include <LPC17xx.h>
#include "stdutils.h"





/*************************************************************************************************
                         Costants, Structures and Typedefs for timers 							   
**************************************************************************************************/
#define EINT0          0
#define EINT1          1
#define EINT2          2
#define EINT3          3
#define EINT_MAX       4

//#define LOW     0 already defined in stdutils.h 
//#define HIGH    1
#define FALLING 2
#define RISING  3

typedef void (*extnIntrFunPtr)(void);

typedef struct
{
    extnIntrFunPtr userFunction; 
    IRQn_Type IrqNumber;
    uint8_t pinumber; 
}eintConfig_t;

/*************************************************************************************************/




/*************************************************************************************************
                                Function Prototypes 							   
*************************************************************************************************/
void EINT_AttachInterrupt(uint8_t intNumber_u8, extnIntrFunPtr funPtr, uint8_t intMode_u8);
void EINT_DetachInterrupt(uint8_t intNumber_u8);
void EINT_Enable(uint8_t timerNumber_u8);
void EINT_Disable(uint8_t timerNumber_u8);     
/*************************************************************************************************/    

#endif
