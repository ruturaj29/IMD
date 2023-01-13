#ifndef _SHT11_H
#define _SHT11_H

#include "stdutils.h"

#define	SDA_PIN			 P0_10
#define SCL_PIN 		 P0_11

#define HUMIDITY_BYTE    0x05
#define TEMPERATURE_BYTE 0x03

//State machine states

#define SHT_DATA_WAIT_INITIAL_STATE     0
#define SHT_DATA_WAIT_TIMEOUT_STATE     1

#define SHT_PROC_RESET_COND_STATE       0   
#define SHT_PROC_START_COND_STATE       1
#define SHT_PROC_SEND_STATE             2
#define SHT_PROC_WAIT_STATE             3
#define SHT_PROC_RETURN_STATE           4
													  
#define SHT_RH_READ_TEMPERATURE_STATE   0
#define SHT_RH_READ_HUMIDITY_STATE      1
#define SHT_RH_CONVERT_RH_STATE         2

#define MCU_cclk						400000000UL
#define SHT_Max_CCLK					100000							   
#define sht_msec_ticks  				MCU_cclk/4
#define SHT_WAIT 						MCU_cclk/SHT_Max_CCLK


extern char Minute_ATRH_COUNTS;
extern char _Temperature[10], _Humidity[10];

extern char _MinTemp[10], _MaxTemp[10],_MinHumid[10], _MaxHumid[10];

extern char ATRH_COUNTS;

void SHT11_Init(void);
void SHT11_wait(void);
void SHT11_start_condition( void );
void SHT11_reset_condition( void );
int SHT11_send_byte( uint8_t sht_data2send );
int SHT11_data_wait(uint32_t sht_timeout );
uint8_t SHT11_read_byte( uint8_t sht_ack );
int SHT11_proc(float sht_param,float *ret_value);
int SHT11_get_temp(float *ret_temperature);
int SHT11_get_temp_RH(volatile float *ret_temperature, volatile float *ret_humidity );
//int get_value(float *ret_temperature, float *ret_humidity)
void read_SHT11(void);
void Average_ATRHSamples(void);




#endif
