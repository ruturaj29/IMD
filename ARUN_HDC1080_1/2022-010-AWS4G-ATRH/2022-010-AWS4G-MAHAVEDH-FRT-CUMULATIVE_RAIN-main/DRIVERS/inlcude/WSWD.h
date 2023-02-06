#ifndef _WSWD_H_
#define _WSWD_H_

/***************************************************************************************************
                              macros define 
***************************************************************************************************/
#define RS485_Enable P1_0

/**************************************************************************************************
                           extern all global variables 
***************************************************************************************************/
//char CurrentWindDirection[4],CurrentSpeed[6];
extern char Minute_WINDSENSOR_COUNTS;
extern char CurrentWindDirection[4],CurrentSpeed[6];
extern char _CurrentWindDir[10], _CurrentWindSpd[10];
//volatile double f_DM, f_DN, f_DX,Sum_of_f_DM,Sum_of_f_DX;
//volatile double f_SM, f_SN, f_SX,Sum_of_f_SM,Sum_of_f_SX;


/***************************************************************************************************
                             Function Prototypes
***************************************************************************************************/
void Read_WindSensor_InterruptBuffer(void);
void Buffer_Flush_UART0( void );

void Read_WindSensor(void);
void CheckCommunication_WindSensor(void);
void SetDefault_WindSensor(void);
void Configure_WindSensor(void);
void Average_WindSamples(void);
void Trignometric_Conversion(void);
void average_windDirection(void);

#endif





