

#ifndef _SPI_H_
#define _SPI_H_

#include "stdutils.h"


/****************************************************************************************************
                            SPI pin numbers and pin functions
 ****************************************************************************************************/
#define MOSI_PIN	P0_18
#define MISO_PIN	P0_17
#define SSEL_PIN    P0_16
#define SCK_PIN		P0_15

#define SPI_FUNCTION  PINSEL_FUNC_3

//#define SCK_Freq	4000000          // SPI clock frequency
#define SCK_Freq	1000000		  // SPI clock frequency
/***************************************************************************************************/






/****************************************************************************************************
                            SPI SFR bits
 ****************************************************************************************************/
#define SBIT_CPHA    3
#define SBIT_CPOL    4
#define SBIT_MSTR    5
#define SBIT_LSBF    6
#define SBIT_SPIE    7
#define SBIT_SPIF    7
/***************************************************************************************************/






/****************************************************************************************************
                            SPI function prototypes and Macros
 ****************************************************************************************************/
void SPI_Init(void);
void SPI_DeInit(void);
uint8_t SPI_Write (uint8_t var_data_u8);
uint8_t SPI_Read(void);


#define SPI_EnableChipSelect()	 GPIO_PinWrite(SSEL_PIN,LOW)
#define SPI_DisableChipSelect()	 GPIO_PinWrite(SSEL_PIN,HIGH)

uint32_t spi_GetPclk(void);
/***************************************************************************************************/


#endif



