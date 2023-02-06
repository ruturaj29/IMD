
/************************ include Header files *******************************/
#include <lpc17xx.h>
#include "spi.h"
#include "gpio.h"
#include "lpc17xx_clkpwr.h"
#include "uart.h"

/************************* Globle variables *********************************/
uint8_t dummy_u8;

/***************************************************************************************************
                           void SPI_Init(void)
 ****************************************************************************************************
 * I/P Arguments : none

 * Return value  : none

 * description :This function initializes the SPI module of LPC1768.
                 MOSI -- P0.18
                 MISO -- P0.17
                 SCK  -- P0.15
                 SSEL -- P0.16               
 ****************************************************************************************************/
void SPI_Init(void)
{    
    GPIO_PinFunction(SCK_PIN,SPI_FUNCTION);   /* Configure the Pinfunctions for SPI */
    GPIO_PinFunction(MOSI_PIN,SPI_FUNCTION);
    GPIO_PinFunction(MISO_PIN,SPI_FUNCTION);
    GPIO_PinFunction(SSEL_PIN,PINSEL_FUNC_0); /* Configure the SSEL pin as GPIO */

    GPIO_PinDirection(SCK_PIN,OUTPUT);        /* Configure SCK,MOSI,SSEl as Output and MISO as Input */ 
    GPIO_PinDirection(MOSI_PIN,OUTPUT);
    GPIO_PinDirection(MISO_PIN,INPUT);
    GPIO_PinDirection(SSEL_PIN,OUTPUT);

    SPI_DisableChipSelect();                  /* Disable the Slave Select */

    LPC_SC->PCONP |= (1 << 8);                   /* enable power to spi clock */

	console_log("spi_GetPclk():%d \n\r",spi_GetPclk());

    LPC_SPI->SPCCR = spi_GetPclk()/SCK_Freq;  /* Set Spi Clock */     

    LPC_SPI->SPCR = ((0<<SBIT_CPHA) | (1<<SBIT_CPOL) | (1<<SBIT_MSTR));
    dummy_u8  = LPC_SPI->SPSR;               /* Dummy read to clear the flags */
    dummy_u8  = LPC_SPI->SPDR;               /* Dummy read to clear the flags */
}

/***************************************************************************************************
                          uint8_t SPI_Write (uint8_t spiData_u8)
 ****************************************************************************************************
 * I/P Arguments : 
                 uint8_t : Byte of data to be send on SPI.

 * Return value  : 
                 uint8_t : Returns back the data send on SPI, this is used in case of SD card.

 * description :
                 This function is used to send a byte of data through SPI.              
 ****************************************************************************************************/
uint8_t SPI_Write (uint8_t spiData_u8)
{
    LPC_SPI->SPDR = spiData_u8;
    while(util_GetBitStatus(LPC_SPI->SPSR,SBIT_SPIF) == 0); /* wait until data is sent */
    dummy_u8 = LPC_SPI->SPSR;
    dummy_u8 = LPC_SPI->SPDR;

    return spiData_u8;
}

/***************************************************************************************************
                          uint8_t SPI_Read(void)
 ****************************************************************************************************
 * I/P Arguments : none

 * Return value  : 
                  uint8_t : Returns back a byte of data read from SPI.

 * description :
                 This function is used to read a byte of data from SPI.              
 ****************************************************************************************************/
uint8_t SPI_Read(void)
{
    uint8_t spiData_u8;

    LPC_SPI->SPDR = 0xff;

    while(util_GetBitStatus(LPC_SPI->SPSR,SBIT_SPIF) == 0); /* wait until data is received */
    dummy_u8 = LPC_SPI->SPSR;                               /* Dummy read to clear the flags */
    spiData_u8 = (uint8_t)LPC_SPI->SPDR; 

    return spiData_u8;
}

/***************************************************************************************************
                    static uint32_t spi_GetPclk(void)
 ****************************************************************************************************
 * I/P Arguments : none

 * Return value  : 
                  uint32_t : SPI clock frequency set in PCLKSEL register.

 * description :
                 This function is used to determine the pclk value for SPI.              
 ****************************************************************************************************/
uint32_t spi_GetPclk(void)
{
    uint32_t v_spiPclk_u32,v_Pclk_u32;
    /**
       PCLKSELx registers contains the PCLK info for all the clock dependent peripherals.
       Bit16,Bit17 contains the SPI Clock(ie.SPI_PCLK) information.
       The SPI_PCLK and the actual Peripheral Clock(PCLK) is calculated as below.
       (Refer data sheet for more info)

       SPI_PCLK    PCLK
         0x00       SystemFreq/4        
         0x01       SystemFreq
         0x02       SystemFreq/2
         0x03       SystemFreq/8   
     **/

    v_spiPclk_u32 = (LPC_SC->PCLKSEL0 >> 16) & 0x03;

    switch( v_spiPclk_u32 )
    {
    case 0x00:
        v_Pclk_u32 = SystemCoreClock/4;           //SystemFrequency or  SystemCoreClock
        break;
    case 0x01:
        v_Pclk_u32 = SystemCoreClock;
        break;
    case 0x02:
        v_Pclk_u32 = SystemCoreClock/2;
        break;
    case 0x03:
        v_Pclk_u32 = SystemCoreClock/8;
        break;
    }

    return (v_Pclk_u32);
}

/*********************************************************************//**
 * @brief		De-initializes the SPIx peripheral registers to their
*                  default reset values.
 * @param[in]	SPIx	SPI peripheral selected, should be LPC_SPI
 * @return 		None
 **********************************************************************/
void SPI_DeInit(void)
{
//	CHECK_PARAM(PARAM_SPIx(SPIx));

//	if (SPIx == LPC_SPI){
		/* Set up clock and power for SPI module */
		CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCSPI, DISABLE);
		print_DebugMsg(" De-initializes the SPI peripheral registers to their default reset values.\r\n");
//	}
}


