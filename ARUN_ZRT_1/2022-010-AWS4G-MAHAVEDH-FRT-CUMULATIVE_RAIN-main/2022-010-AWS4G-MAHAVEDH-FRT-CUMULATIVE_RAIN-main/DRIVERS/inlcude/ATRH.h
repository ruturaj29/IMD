#ifndef _ATRH_H_
#define _ATRH_H_

#include "stdutils.h"

#define 		ATRH_INIT_SUCCESSFULL						 0

#define         ATRH_Write_ADD                            	0x80
#define         ATRH_READ_ADD                            	0x81
#define         Config_register_add							0x00
#define         Temperature_register_add	                0x00
#define         Humidity_register_add       	            0x01

void Init_ATRH(void);
uint8_t start_ATRH(uint16_t DevWriteAddress,uint16_t config_register);
void readATRH(uint16_t DevWriteAddress,uint16_t DevReadAddress,uint16_t tempAddress);
void Average_ATRHSamples(void);

extern char Minute_ATRH_COUNTS;
extern char _Temperature[10], _Humidity[10];

extern char _MinTemp[10], _MaxTemp[10],_MinHumid[10], _MaxHumid[10];

extern char ATRH_COUNTS;

#endif
