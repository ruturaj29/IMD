
#ifndef _ADC_H
#define _ADC_H

/************************************************************************************************* 
																	ALL header files 
**************************************************************************************************/

#include <lpc17xx.h>
#include "delay.h"
#include "stdutils.h"
#include "gpio.h"
#include "uart.h"
#include "convert.h"
#include "lcd.h"




/***************************************************************************************************
                             Commonly used ADC macros/Constants
***************************************************************************************************/
#define M_AdcClockFreq		1000000		/* set to 1Mhz */
	
#define SBIT_BURST   16u
#define SBIT_START	 24u
#define SBIT_PDN	 21u
#define SBIT_EDGE	 27u

#define SBIT_DONE	 31u
#define SBIT_RESULT	  4u


#define M_GetClkDivValue(pclk)  (( pclk  / M_AdcClockFreq - 1u ) << 8u )



/***************************************************************************************************
                            Constants and Structures
***************************************************************************************************/                        
#define C_MaxAdcChannels_U8   8u

typedef struct
{
  uint8_t pinNumber;
  uint8_t PinFunSel;
}adcChannelConfig_st;
/**************************************************************************************************/

extern int adcValue0,adcValue1;
extern float volt0,volt1, ActualVoltage0, ActualVoltage1;
extern char _BVolt[6];
extern char _PVolt[6];


/***************************************************************************************************
                             Function Prototypes
***************************************************************************************************/
void ADC_Init(void);
uint16_t ADC_GetAdcValue(uint8_t v_adcChannel_u8);

/**************************************************************************************************/
void Read_ADCchannels(void);

#endif
