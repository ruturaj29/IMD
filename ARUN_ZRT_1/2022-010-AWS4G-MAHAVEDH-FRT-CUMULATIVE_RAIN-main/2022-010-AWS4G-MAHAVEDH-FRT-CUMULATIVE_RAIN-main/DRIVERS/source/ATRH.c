
/************************ include Header files *********************************/
#include <lpc17xx.h>
#include "convert.h"
#include "ATRH.h"
#include "i2c0.h"
#include "delay.h"
#include "uart.h"

/************************ Globle variables Define ******************************/
volatile float temperature = 0, humidity = 0, Sum_of_temperature = 0, Sum_of_humidity =0;
volatile float fMinTemp = 0, fMaxTemp = 0, fMinHumid = 0, fMaxHumid = 0;
char _Temperature[10]={0}, _Humidity[10]={0},ATRH_COUNTS = 0,Minute_ATRH_COUNTS = 0;
char  _MinTemp[10]={0}, _MaxTemp[10]={0}, _MinHumid[10]={0}, _MaxHumid[10]={0},FirstRead_ATRH = 1;
extern char _LCDTemperature[10], _LCDHumidity[10];

/*********************************************************************************
*                              Init ATRH 		    																 *
**********************************************************************************/
void Init_ATRH(void)
{	
	i2c0_init(MODE_400kbps, 3);
	i2c0_enable();
}

/*********************************************************************************
*                             start ATRH 		    																 *
**********************************************************************************/
uint8_t start_ATRH(uint16_t DevWriteAddress,uint16_t config_register)
{
	uint8_t status;
	
	i2c0_start();
//	console_log("\n\rStart condition(0x08) = %x\n\r",LPC_I2C2->I2STAT); // print response in terminal
	
	status = i2c0_Address(DevWriteAddress);   // Slave address 0x40+R/W bit low
//	console_log("\n\rSla+W(0x18) = %x\n\r",LPC_I2C2->I2STAT); // print response in terminal
	if(status == 0x18) //SLA+W has been transmitted; ACK has been received.
	{
//		print_DebugMsg(" SLA+W, ACK has been received\n\r");
		
		i2c0_Write(config_register);
//		console_log("\n\r First byte transmit(0x28) = %x\n\r",LPC_I2C2->I2STAT); // print response in terminal
		
		i2c0_Write(0x90);
//		console_log("\n\rSecond byte transmit(0x28) = %x\n\r",LPC_I2C2->I2STAT); // print response in terminal
		
		i2c0_Write(0x00);
//		console_log("\n\rThird byte transmit(0x28) = %x\n\r",LPC_I2C2->I2STAT); // print response in terminal
		
		i2c0_stop();
//		console_log("\n\rStop transmit(?) = %x\n\r",LPC_I2C2->I2STAT); // print response in terminal

		return 0;

	}
	else if(status == 0x20) // SLA+W has been transmitted; No ACK has been received.
	{
		print_DebugMsg("ATRH (SLA+W), No ACK has been received\n\r");
		print_DebugMsg("Error : Sensor communication\n\r");
		i2c0_stop();

		return 1;
	}
	else
	{	 	
		i2c0_stop();
		return 1;
	}
}

/*********************************************************************************
*                             read ATRH 		    																 *
**********************************************************************************/
//void readATRH(uint16_t DevWriteAddress,uint16_t DevReadAddress,uint16_t tempAddress,float *temperature,float *humidity)
void readATRH(uint16_t DevWriteAddress,uint16_t DevReadAddress,uint16_t tempAddress)
{
	uint8_t status,i;

    //holds the total contents of the temp register	and humidity register
  	uint16_t temp, humid; 
	char read_buf[4];	 // Buffer to read the byte from ATRH sensor

	for ( i = 0; i < 5; i++ )
  	{
			read_buf[i] = 0x00;
	}
	status = i2c0_start();
//	console_log("\n\rStart condition(0x08) = %x\n\r",LPC_I2C2->I2STAT); // print response in terminal
	
	status = i2c0_Address(DevWriteAddress);   // Slave address 0x40+write
//	console_log("\n\rSLA+W transmit(0x18) = %x\n\r",LPC_I2C2->I2STAT); // print response in terminal
	if(status == 0x18) //SLA+W has been transmitted; ACK has been receivmed
	{
//		print_DebugMsg("\n\r SLA+W, ACK has been received\n\r");
		i2c0_Write(tempAddress);
//		console_log("\n\r Responce for Temp Pointer(0x00) = %x\n\r",LPC_I2C2->I2STAT); // print response in terminal							
		i2c0_stop();
//		console_log("\n\r Stop transmit(?) = %x\n\r",LPC_I2C2->I2STAT); // print response in terminal
		DELAY_ms(18);
	
		status = i2c0_start();
//		console_log("\n\rStart condition(0x08) = %x\n\r",LPC_I2C2->I2STAT); // print response in terminal
		if(status == 0x08)		 //A repeated START condition has been transmitted.
		{
//			print_DebugMsg(" A START condition has been transmitted.\n\r");	
		}
		else	
		print_DebugMsg(" A START condition has not been transmitted.\n\r");
			
		status = i2c0_Address(DevReadAddress);   // Slave address 0x40+read
//		console_log("\n\rSlave address 0x40+read(0x40) = %x\n\r",LPC_I2C2->I2STAT); // print response in terminal
		if(status == 0x40) //SLA+R has been transmitted; ACK has been received
		{
//			print_DebugMsg("\n\r SLA+R, ACK has been received\n\r");
//			read_buf[0]=i2c0_Read(1);
			i2c0_Read(&read_buf[0],1);
//			console_log("\n\rBuf0 = %c\n\r",read_buf[0]);
//			console_log("\n\rRead buffer(0x50) = %x\n\r",LPC_I2C2->I2STAT); // print response in terminal
//		   	read_buf[1]=i2c0_Read(1);
			i2c0_Read(&read_buf[1],1);
//			console_log("\n\rBuf1 = %c\n\r",read_buf[1]);
//			console_log("\n\rRead buffer(0x50) = %x\n\r",LPC_I2C2->I2STAT); // print response in terminal
//		   	read_buf[2]=i2c0_Read(1);
			i2c0_Read(&read_buf[2],1);
//			console_log("\n\rBuf2 = %c\n\r",read_buf[2]);
//			console_log("\n\rRead buffer(0x50) = %x\n\r",LPC_I2C2->I2STAT); // print response in terminal
//		   	read_buf[3]=i2c0_Read(0);
			i2c0_Read(&read_buf[3],0);
//			console_log("\n\rBuf3 = %c\n\r",read_buf[3]);
//			console_log("\n\rRead buffer(0x58) = %x\n\r",LPC_I2C2->I2STAT); // print response in terminal
			i2c0_stop();
//			console_log("\n\r Stop transmit(?) = %x\n\r",LPC_I2C2->I2STAT); // print response in terminal
		}
		else
		{
			print_DebugMsg("\n\r ATRH (SLA+R), ACK has not received\n\r");
			print_DebugMsg("\n\r Error, Sensor Communication\n\r");
			i2c0_stop();
		}
			
	//	read_buf[4] = '\0';
	/*	for ( i = 0; i < 4; i++ )
  	 	{
			console_log("\n\rread_buf[%d]:%x \n\r",i,read_buf[i]); // print response in terminal
  		} */
			
		//Combine the two bytes to make one 16 bit int
		temp = ((read_buf[0] <<8 )| read_buf[1]);

		//Combine the two bytes to make one 16 bit int
		humid = ((read_buf[2] <<8)| read_buf[3]);
		
		//Temp(C) = reading/(2^16)*165(C) - 40(C)
	//	*temperature =(double)(temp)/(16384.0)*165.0-40.0;
	
		//Humidity(%) = reading/(2^16)*100%
	//	*humidity = (double)(humid)/(65536.0)*100.0;
		humidity=(float)((humid/65536.0)*100.0);
	
		temperature=((temp/65536.0)*165.0)-40.0;
		
		humidity = humidity - 4.0;
		temperature = temperature - 0.45;

		/* Conditon aply only first minute reading of ATRH */
		if(FirstRead_ATRH)
		{
			fMaxTemp = 	temperature;
			fMinTemp = 	temperature;
			fMinHumid = humidity;
			fMaxHumid = humidity;
			FirstRead_ATRH = 0;
		}
		/* Check for max temperature */
		if(temperature>fMaxTemp)
		{
		 fMaxTemp =  temperature;
		}
		/* check for min temperature */
		if(humidity<fMinHumid)
		{
		  fMinHumid =  humidity;
		}
		/* Check for max Humidity */
		if(humidity>fMaxHumid)
		{
		 fMaxHumid =  humidity;
		}
		/* check for min Humidity */
		if(temperature<fMinTemp)
		{
		  fMinTemp =  temperature;
		}

		console_log("\n\rTemperature = %f, Humidity = %f\n\r",temperature,humidity);
		Sum_of_temperature = Sum_of_temperature + temperature;
        Sum_of_humidity = Sum_of_humidity + humidity;
		ATRH_COUNTS = ATRH_COUNTS+1;
		Minute_ATRH_COUNTS = 1; //  log flag set to idnetify the sensor reading in a minute

		ftoa_signed(temperature,_Temperature,2);
		ftoa_signed(humidity,_Humidity,2);

		ftoa_signed(temperature,_LCDTemperature,2);
		ftoa_signed(humidity,_LCDHumidity,2);


	}	
	if(status == 0x20) // SLA+W has been transmitted; No ACK has been received.
	{
		print_DebugMsg("ATRH (SLA+W), No ACK has been received\n\r");
		print_DebugMsg("\n\r Error, Sensor Communication\n\r");

	  /* Default error values if no comm happened with ATRH*/
	//	temperature = 125.00;
	//	humidity = 100.00;

		i2c0_stop();


	}
	i2c0_stop();

}

/*********************************************************************************
*                             Average ATRHSamples																 *
**********************************************************************************/
void Average_ATRHSamples(void)
{
	/* Average formula: Average = (Sum of Smaples/Count)*/
	float Average_Temperature = 0, Average_Humidity = 0;

	Average_Temperature = Sum_of_temperature/ATRH_COUNTS;
	Average_Humidity = Sum_of_humidity/ATRH_COUNTS;
	
	ftoa_signed(Average_Temperature,_Temperature,2);
	ftoa_signed(Average_Humidity,_Humidity,2);	

	ftoa_signed(fMinTemp,_MinTemp,2);
	ftoa_signed(fMaxTemp,_MaxTemp,2);	

	ftoa_signed(fMinHumid,_MinHumid,2);
	ftoa_signed(fMaxHumid,_MaxHumid,2);

	/* clear varaibles and counts for next samples */
	Sum_of_temperature = 0;
	Sum_of_humidity = 0;
	ATRH_COUNTS = 0;
	fMinTemp = 0;
	fMaxTemp = 0;
	fMinHumid = 0;
	fMaxHumid = 0;
	FirstRead_ATRH = 1;
}

