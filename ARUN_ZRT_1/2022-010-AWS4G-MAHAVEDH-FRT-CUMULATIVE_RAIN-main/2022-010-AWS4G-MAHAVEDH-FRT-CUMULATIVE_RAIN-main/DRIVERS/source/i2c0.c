
/************************ include Header files *******************************/
#include <lpc17xx.h>
#include "i2c0.h"
//#include "type.h"
#include"stdutils.h"
#include "uart.h"
 
/************************ Globle Macro Define *******************************/
// IC2 control bits
#define AA      (1 << 2)
#define SI      (1 << 3)
#define STO     (1 << 4)
#define STA     (1 << 5)
#define I2EN    (1 << 6)
 

/************************ DEBUG *********************************************/
//LPC_I2C_TypeDef*  regs;

/*********************************************************************************
*                                  i2c0 init																		 *
**********************************************************************************/ 
void i2c0_init(uint32_t i2c_freq, uint8_t int_pri) { 
 
    // give power to the I2C hardware
    LPC_SC->PCONP |= (1 << 26);
 
  /* set PIO0.10 and PIO0.11 to I2C2 SDA and SCL */
  /* function to 10 on both SDA and SCL. */
  LPC_PINCON->PINSEL0 &= ~((0x03<<20)|(0x03<<22));
  LPC_PINCON->PINSEL0 |= ((0x02<<20)|(0x02<<22));
  LPC_PINCON->PINMODE0 &= ~((0x03<<20)|(0x03<<22));
  LPC_PINCON->PINMODE0 |= ((0x02<<20)|(0x2<<22));	/* No pull-up no pull-down */
  LPC_PINCON->PINMODE_OD0 |= ((0x01<<10)|(0x1<<11));
 
  /*--- clear all flags ---*/
  LPC_I2C2->I2CONCLR = AA | SI | STA | I2EN;
	  /*--- Reset registers ---*/
  LPC_I2C2->I2SCLL   = 0x00000080;
  LPC_I2C2->I2SCLH   = 0x00000080;
 
    // install interrupt handler
 //   NVIC_EnableIRQ(irqn);
 
    // set the priority of the interrupt
 //   NVIC_SetPriority(irqn, int_pri); // '0' is highest
 
    // enable the I2C (master only)
 //   regs->I2CONSET = I2EN;
}

/*********************************************************************************
*                     i2c0 enable				            														 *
**********************************************************************************/
void i2c0_enable(void)
{
	/*--- enable the I2C (master only) ---*/ 
    LPC_I2C2->I2CONSET = I2EN;
}

/*********************************************************************************
*                     i2c0 start				            														 *
**********************************************************************************/
unsigned char i2c0_start(void)
{
		LPC_I2C2->I2CONSET |= 1<< 5;	//START I2C0
		while (!(LPC_I2C2->I2CONSET & (1<<3)));	 // wait until done

		return (LPC_I2C2->I2STAT);
}

/*********************************************************************************
*                     i2c0 stop				              														 *
**********************************************************************************/
void i2c0_stop(void)
{
		LPC_I2C2->I2CONSET |= 1<<4;	//STOP I2C
		LPC_I2C2->I2CONCLR = 1<< 3 ;		//clear SI

	    while (LPC_I2C2->I2CONSET & (1<<4));	//wait until H/w stops I2c
}

/*********************************************************************************
*                     i2c0 Address				           														 *
**********************************************************************************/
unsigned char i2c0_Address(unsigned char add) {
 	
 	LPC_I2C2->I2DAT = add;	//the address
 	LPC_I2C2->I2CONCLR = 1<<5;	//clear start
 	LPC_I2C2->I2CONCLR = 1<< 3;	//clear SI

 	while (!(LPC_I2C2->I2CONSET & (1<<3)));	//wait until change in status
 
 	return (LPC_I2C2->I2STAT);
 	
}

/*********************************************************************************
*               i2c0 Write [Function to Write data to slave]				  					 *
**********************************************************************************/
unsigned char i2c0_Write(char dat) {
 		
 	LPC_I2C2->I2DAT = dat ;	//new data
 	LPC_I2C2->I2CONCLR = 1<< 3 ; // clear SI

	while (!(LPC_I2C2->I2CONSET & (1<<3)));	//wait until change in SI status
 
 	return (LPC_I2C2->I2STAT);	//the data
 }


/*********************************************************************************
*               i2c0 Read [Function to Read data from slave]				  					 *
**********************************************************************************/
unsigned char i2c0_Read(char *retdat,char ack ) {

 	
 	if (ack) LPC_I2C2->I2CONSET =1<<2;	//assert AA -ACK more bytes to come
 	     	else LPC_I2C2->I2CONCLR = 1<<2;	//No ack - last byte

 	LPC_I2C2->I2CONCLR = 1<< 3 ; // clear SI

 	while (!(LPC_I2C2->I2CONSET & (1<<3)));	//wait until change in SI status
 	*retdat = (uint8_t) (LPC_I2C2->I2DAT & 0xFF);
	
	return (LPC_I2C2->I2STAT & 0xF8);
 
 }

