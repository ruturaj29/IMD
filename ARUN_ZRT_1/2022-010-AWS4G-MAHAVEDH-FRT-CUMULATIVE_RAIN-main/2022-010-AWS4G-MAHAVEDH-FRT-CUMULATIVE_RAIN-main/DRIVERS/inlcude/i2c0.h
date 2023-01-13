
#ifndef _I2C0_H_
#define _I2C0_H_
 
/* includes */

#include "LPC17xx.h"
#include"stdutils.h"
 
/* defines */
#define MODE_100kbps 100000
#define MODE_400kbps 400000
#define MODE_1Mbps 1000000
 
/* typedefs */
 
/* functions */
 
// Initialize the I2C hardware.
void i2c0_init(uint32_t i2c_freq, uint8_t int_pri);
void i2c0_enable(void);
unsigned char i2c0_start(void);
void i2c0_stop(void);
unsigned char i2c0_Address(unsigned char add);
unsigned char i2c0_Write(char dat);
unsigned char i2c0_Read(char *retdat,char ack );
												    
#endif /* _I2C0_H_ */

