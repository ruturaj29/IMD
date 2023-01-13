
/************************ include Header files *********************************/
#include <lpc17xx.h>
#include <stdio.h>
#include <string.h>
#include "uart.h" 
#include "ff.h"
#include "ATRH.h"
#include "stdutils.h"
#include "delay.h"	 
#include "lcd.h"    
#include "rtc.h"    
#include "gprs.h"
#include "adc.h"
#include "extintr.h"
#include "lpc17xx_wdt.h"
#include "convert.h"
#include "WSWD.h"
#include "anomometer.h"
#include "systick.h"
#include "spi.h"
#include "bod.h"
#include "diskio.h"
#include "mmc_176x_ssp.h"
#include "Log.h"

/************************ Globle variables Define ******************************/
/************ Ultrasonic variables *******************/
extern char DN[4],DM[4],DX[4];
extern char SN[6],SM[6],SX[6],Gust[6];
extern char Tan_Inverse_WindDirection[6];
extern char MinSpeed[6], WINDSENSOR_COUNTS;

/************ main.c variables *******************/
extern int Card_Failed;
extern char _buffer[512];
extern rtc_t rtc;	/* rtc structure declare */
extern char _ErrorLogBuffer[512];
extern uint8_t returnStatus,responseStatus,GPSCheck;
extern int PDP;

/************ Log.c variables *******************/	
int PlogCreate	= 1;
char FirmwareRevision[] = "M4GFZRT.0.9";		// storage for Firmware version or software revision

/***************************************************************************************
*    Slog [ Creat packet at every Multiple of logInterval mint - 1 mint interval ]     *
****************************************************************************************/
void Slog( void )
{
	print_DebugMsg("\n\r ---- Slog interval ---- \n\r");
	Read_ADCchannels();	/* Read Solar voltage and Battery Voltage */

	/* Collect - measure Wind Sensor values */
	if (!Minute_WINDSENSOR_COUNTS) {
		strcpy(DN,"999");										// wind direction minimum
		strcpy(CurrentWindDirection,"999");	// wind direction mean
		strcpy(DX,"999");										// wind direction maximum

		strcpy(SN,"999");										// wind speed minimum
		strcpy(CurrentSpeed,"999");					// wind speed mean
		strcpy(SX,"999");	
	}
	Minute_WINDSENSOR_COUNTS = 0;

	/* Collect - measure ATRH values */
	if (!Minute_ATRH_COUNTS) {
		strcpy(_Temperature,"-999");				// ATRH Temperature min , max
		strcpy(_Humidity,"-999");						// ATRH Humidity min , max	
	}
	Minute_ATRH_COUNTS = 0;

	memset(_buffer, 0, 512);	/* Clear the Buffer */

	/* Combine the parameters and store it in _Sbuffer 
	   - for sending it to portal and also storing it to SD card */
	sprintf(_buffer,"#;%s;%s;%02d/%02d/%d;%02d:%02d:00;%04s;%04s;%04s;%04s;%04s;%04s;%04s;%s;%04s;%04s;%s,0",
					imei,FirmwareRevision,
					(uint16_t)rtc.date,(uint16_t)rtc.month,(uint16_t)rtc.year,
					(uint16_t)rtc.hour,(uint16_t)rtc.min,
					_PVolt,_BVolt,
					_Temperature,_Humidity,
					CurrentSpeed,SX,SN,
					CurrentWindDirection,DX,DN,
					_sQuality);

	console_log("%s\n\r",_buffer);			/* print response in terminal */	 
	Createlog(_buffer, "slog.txt");		/* Save packet to Slog - for reference */
					 rtc_1call();
}

/***************************************************************************************************
*    PacketLog [ Creat packet at every Multiple of PacketlogInterval mint - 10 mint interval ]     *
****************************************************************************************************/
void PacketLog ( void )
{
  
	uint8_t i = 0;

	print_DebugMsg("\n\r ---- PacketLog interval ---- \n\r");
	
	/* Collect - measure Wind Sensor values  */
	if (WINDSENSOR_COUNTS)
		Average_WindSamples();											// Average the samples
	else {
		strcpy(DN,"999");													// wind direction minimum
		strcpy(DM,"999");													// wind direction mean
		strcpy(DX,"999");													// wind direction maximum
	
		strcpy(SN,"999");													// wind speed minimum
		strcpy(MinSpeed,"999");										// wind speed minimum
		strcpy(SM,"999");													// wind speed mean
		strcpy(SX,"999");													// wind speed maximum

		strcpy(Gust,"0.00");											// Gust
		strcpy(Tan_Inverse_WindDirection,"0.00");	// average of wind direection
		strcpy(CurrentWindDirection,"999");				// Current wind dierction		
	}

	/* Collect - measure ATRH values */	  	
	if(ATRH_COUNTS)
		Average_ATRHSamples();	 
	else {
		strcpy(_Temperature,"-999");							// ATRH Temperature Current
		strcpy(_MinTemp,"-999");									// ATRH Temperature Minimum									
		strcpy(_MaxTemp,"-999");									// ATRH Temperature maximum

		strcpy(_Humidity,"-999");									// ATRH Humidity current
		strcpy(_MinHumid,"-999");									// ATRH Humidity Minimum
		strcpy(_MaxHumid,"-999");									// ATRH Humidity Maximum
	}

	calcRainFall();						/* Collect - measure Rain gauge (Rain fall value ) */
	Read_ADCchannels();				/* Read Solar voltage and Battery Voltage */
	memset(_buffer, 0, 512);	/* Clear the Buffer */

	/* Combine the parameters and store it in _buffer 
	   - for sending it to portal and also storing it to SD card */
	sprintf(_buffer,"#;%s;%s;%02d/%02d/%d;%02d:%02d:00;%04s;%04s;%04s;%04s;%04s;%02s;%02s;%02s;%04s;%04s;%04s;%04s;%04s;%04s;%04s;%04s;0.00;%s,0;0.00;01/01/1970;00:00:00;0",
					imei,FirmwareRevision,
					(uint16_t)rtc.date,(uint16_t)rtc.month,(uint16_t)rtc.year,
					(uint16_t)rtc.hour,(uint16_t)rtc.min,
					_PVolt,_BVolt,
					_Temperature,_MaxTemp,_MinTemp,				  
					_Humidity,_MaxHumid,_MinHumid,
					SX,Gust,MinSpeed,SM,
					CurrentWindDirection,Tan_Inverse_WindDirection,
					_Rain,_xyz11,_sQuality);
		  	
	console_log("%s\n\r",_buffer);	/* print response in terminal */						
	returnStatus = Writelog();											/* save packet to pack.txt file*/							
	console_log("returnStatus for Write log : %d \n\r",returnStatus);
	DELAY_ms(100);

	if ((Card_Failed)||(returnStatus)||(PacketlogInterval == PacketsendInterval)) {
		print_DebugMsg(" 10 min log loop \n\r");
		UART1_EnableBuffer_Interrupt();	/* Enable Buffer RBR UART 1 interrupt*/
		returnStatus =	Module_awakeup_mode();	/* GSM module wakeup */

		if (returnStatus ) {

			/* Activate PDP context */
			for (i=2;i>0;i--) {
				returnStatus = modemCheck_initialization(APN);
				if (returnStatus) break;
			}
			if (returnStatus) { /* Activate PDP context successful */
				responseStatus = PackOffLog(_buffer); /* Send the packet to the server */ 	
				if((responseStatus == MODEM_RESPONSE_TIMEOUT)
					||(responseStatus == MODEM_RESPONSE_ERROR)||(responseStatus == 0)) {
						if(Check_QIState()) { /* check PDP context status */
							responseStatus = PackOffLog(_buffer); /* Try 2nd time sending packet*/	
						}
				}
			}
			else {
			
				WDT_Feed();

				ShutDownModule();			/* MCU GPIO PIN HIGH to OFF the regulator*/
				RestartModule();		  /* MCU GPIO PIN LOW to ON the regulator*/
				DELAY_ms(8000);				/* wait to start module */
		
				responseStatus = Module_Initializing();
				
				if(responseStatus){
					print_DebugMsg("module activate successfully!\r\n");	
					responseStatus = PackOffLog(_buffer); /* Send the packet to the server */
				}
				else
				{
				  LCD_Clear();
					LCD_GoToLine(0);LCD_Printf("PDP Activation..");
			  	LCD_GoToLine(1);LCD_Printf("failed");
					DELAY_ms(200);
		
				  RTC_GetDateTime(&rtc);
					memset(_buffer, 0, 512);
					sprintf(_buffer,"[%02d/%02d/%d;%02d:%02d]:[ERROR]:MESSAGE DETAIL: %d;PDP failed;",
									(uint16_t)rtc.date,(uint16_t)rtc.month,(uint16_t)rtc.year,(uint16_t)rtc.hour,(uint16_t)rtc.min,ERROR_CODE);
					Createlog(_buffer, "err.txt");	/* Save log as a error - for reference */
					
					print_DebugMsg(" PDP activation failed \n\r");
					responseStatus = MODEM_RESPONSE_ERROR;
					print_DebugMsg("\n\r ---- Packet sending failed ---- \n\r");
	
				}
			}
		
			/* Save packet to pending log - if packet not sent succesful */
			if((responseStatus == MODEM_RESPONSE_TIMEOUT) 
				||(responseStatus == MODEM_RESPONSE_ERROR)||(responseStatus == 0)) {
				 
					print_DebugMsg(" write in Plog \n\r");
				  
					/* save packet to pending log file*/
					Createlog(_buffer, "plog.txt");
					memset(_ErrorLogBuffer,0,512);	/* Clear the Buffer */
					RTC_GetDateTime(&rtc);					/* Get RTC time and Date*/
				
					/* Combine the string */
					sprintf(_ErrorLogBuffer,"[%02d/%02d/%d;%02d:%02d]:[ERROR]:MESSAGE DETAIL: %d;Post failed;",
									(uint16_t)rtc.date,(uint16_t)rtc.month,(uint16_t)rtc.year,(uint16_t)rtc.hour,(uint16_t)rtc.min,ERROR_CODE);
					Createlog(_ErrorLogBuffer, "err.txt");	/* Save packet to Errorlog - for reference*/
								
					LCD_Clear();
					LCD_GoToLine(0);LCD_Printf("Post failed !");
					LCD_GoToLine(1);LCD_Printf("Err : %d",ERROR_CODE);
					ShutDownModule();	/// new add problem of module 
					DELAY_ms(2000);
				
					responseStatus = 0;
					PlogCreate = 1;
			}

			if (responseStatus || returnStatus) {
				
				if (Check_LocalIP()) {
					if (Query_Set_CLTS()) {	/* update network date and time */
						MODEM_GetNetworkTime();
					}		
					MODEMGetSignalStrength(); /* Check Signal Stregnth */
					DELAY_ms(100);
				}

				/* GPS coordinates takes at 12PM and 1st time when DL is on*/
//		 		if (((rtc.hour==12)&&(rtc.min==0)) || (GPSCheck)) {
//					
//					responseStatus = GNSS_Connect(APN);	/* GNSS Enable and Read */
//					
//					if (responseStatus == GNSS_STILL_POWER_ON) {
//			  		print_DebugMsg("GPSS Still power ON \r\n");
//					}
//
//					if (responseStatus == TIME_SYNCHRONIZE_NOT_COMPLETE) {
//		 				print_DebugMsg("TIME_SYNCHRONIZE_NOT_COMPLETE \r\n");
//					}
//
//					DELAY_ms(1000);
//					GPSCheck=ReadNavigation();	/* Check GPS co-ordinations */
//					
//					if (GPSCheck) {
//						GNSS_PowerOff();
//						GPSCheck=0;
//					}
//					else {
//						GPSCheck=1;
//					}
//
//					console_log("GPSCheck: %d\n\r",GPSCheck);
//				
//				}
			}

			SMS_READ_Setting();	/* SET SMS setting */
			readSMS();					/* Read SMS */

			Module_deactivateBearerProfile();	/* Deactivate the PDP context of the module */
			DELAY_ms(100);

			Module_sleep_mode();							/* put GSM module in Sleep mode */	
			UART1_DisableBuffer_Interrupt(); 	/* Disable Buffer RBR UART 1 interrupt */

		}
		else {
			returnStatus = MODEM_RESPONSE_ERROR;
			print_DebugMsg("module Still in Sleep mode,NOT wakeup  \n\r");
		}

	}//if ((Card_Failed)||(PacketlogInterval == PacketsendInterval)) loop end
	
	if ((PacketlogInterval != PacketsendInterval)||(returnStatus == MODEM_RESPONSE_ERROR) )
	{
		/* save packet to pending log file*/
		print_DebugMsg(" write in Plog \n\r");
		Createlog(_buffer, "plog.txt");
		PlogCreate = 1;
	}
}

/****************************************************************************************************
*    SendingLog [ Creat packet at every Multiple of PacketsendInterval mint - hourly interval ]     *
*****************************************************************************************************/
void SendingLog ( void )
{

	uint8_t i = 0;

	print_DebugMsg("\n\r ---- SendingLog interval ---- \n\r");

	/* check & only send if plog is available */
	if (PlogCreate) {
		UART1_EnableBuffer_Interrupt();	/* Enable Buffer RBR UART 1 interrupt*/
		returnStatus =	Module_awakeup_mode();	/* GSM module wakeup */
	
		if (returnStatus ) {
	
			/* Activate PDP context */
			for (i=2;i>0;i--) {
				returnStatus = modemCheck_initialization(APN);
				if (returnStatus) break;
			}
	
			if (returnStatus) {
				responseStatus = retrylog();
			}
			else {

				ShutDownModule();			/* MCU GPIO PIN HIGH to OFF the regulator*/
				RestartModule();		  /* MCU GPIO PIN LOW to ON the regulator*/
				DELAY_ms(8000);				/* wait to start module */
		
				responseStatus = Module_Initializing();
				
				if(responseStatus){
					print_DebugMsg("module activate successfully!\r\n");	
					responseStatus = retrylog();
				}
				else
				{
				  LCD_Clear();
					LCD_GoToLine(0);LCD_Printf("PDP Activation..");
			  	LCD_GoToLine(1);LCD_Printf("failed");
					DELAY_ms(200);
					
					print_DebugMsg(" PDP activation failed \n\r");
				  memset(_ErrorLogBuffer,0,512);	/* Clear the Buffer */
					RTC_GetDateTime(&rtc);					/* Get RTC time and Date*/
					/* Combine the string */
					sprintf(_ErrorLogBuffer,"[%02d/%02d/%d;%02d:%02d]:[ERROR]:MESSAGE DETAIL: %d;PDP failed;",
									(uint16_t)rtc.date,(uint16_t)rtc.month,(uint16_t)rtc.year,(uint16_t)rtc.hour,(uint16_t)rtc.min,ERROR_CODE);
					Createlog(_ErrorLogBuffer, "err.txt");	/* Save packet to Errorlog - for reference*/
					print_DebugMsg("\n\r ---- RetryLog failed ---- \n\r");
				
				}	
			}
	
			if ((PacketlogInterval != PacketsendInterval)||(!responseStatus)) {
			
				if (Check_LocalIP()) {
					
					if (Query_Set_CLTS()) {	/* update network date and time */
						MODEM_GetNetworkTime();
					}
							
					MODEMGetSignalStrength(); /* Check Signal Stregnth */
					DELAY_ms(100);
			  }
	
				/* GPS coordinates takes at 12PM and 1st time when DL is on*/
//				if (((rtc.hour==12)&&(rtc.min==0)) || (GPSCheck)) {
//				
//					responseStatus = GNSS_Connect(APN);	/* GNSS Enable and Read */
//				
//					if (responseStatus == GNSS_STILL_POWER_ON) {
//						print_DebugMsg("GPSS Still power ON \r\n");
//					}
//
//					if (responseStatus == TIME_SYNCHRONIZE_NOT_COMPLETE) {
//			 			print_DebugMsg("TIME_SYNCHRONIZE_NOT_COMPLETE \r\n");
//					}
//
//					DELAY_ms(1000);
//					GPSCheck=ReadNavigation();	/* Check GPS co-ordinations */
//					
//					if (GPSCheck) {
//						GNSS_PowerOff();
//						GPSCheck=0;
//					}
//					else {
//						GPSCheck=1;
//					}
//
//					console_log("GPSCheck: %d\n\r",GPSCheck);
//				
//				}
		
			}
			else {
				memset(_ErrorLogBuffer,0,512);	/* Clear the Buffer */
				RTC_GetDateTime(&rtc);					/* Get RTC time and Date*/
				
				/* Combine the string */
				sprintf(_ErrorLogBuffer,"[%02d/%02d/%d;%02d:%02d]:[ERROR]:MESSAGE DETAIL: %d;Post failed;",
								(uint16_t)rtc.date,(uint16_t)rtc.month,(uint16_t)rtc.year,(uint16_t)rtc.hour,(uint16_t)rtc.min,ERROR_CODE);
				Createlog(_ErrorLogBuffer, "err.txt");	/* Save packet to Errorlog - for reference*/
									
				LCD_Clear();
				LCD_GoToLine(0);LCD_Printf("Post failed !");
				LCD_GoToLine(1);LCD_Printf("Err : %d",ERROR_CODE);
				DELAY_ms(2000);
			} 
	
			SMS_READ_Setting();	/* SET SMS setting */
			readSMS();					/* Read SMS */
	
			Module_deactivateBearerProfile();	/* Deactivate the PDP context of the module */
			DELAY_ms(100);
	
			Module_sleep_mode();							/* put GSM module in Sleep mode */	
			UART1_DisableBuffer_Interrupt(); 	/* Disable Buffer RBR UART 1 interrupt */
	
		}
		else{
			print_DebugMsg("module Still in Sleep mode,NOT wakeup  \n\r");
		}
	}
	else
	{
		print_DebugMsg(" Plog not created ....\n\r");
	}

}


/***************************************************************************************
*    DL Status Function [ creat DL Status for send SMS ]                               *
****************************************************************************************/
//void DL_Status( void )
//{
//	print_DebugMsg("\n\r ---- check Datalogger Status ---- \n\r");
//	Read_ADCchannels();																						/* Read Solar voltage and Battery Voltage */
//	
//	readATRH(ATRH_Write_ADD,ATRH_READ_ADD,Temperature_register_add);	/* read sensor values */
//	Read_WindSensor();			                       									 	/* Read Wind sensor */
//
//	/* Collect - measure Wind Sensor values */
//	if (!Minute_WINDSENSOR_COUNTS) {
//		strcpy(DN,"999");										// wind direction minimum
//		strcpy(CurrentWindDirection,"999");	// wind direction mean
//		strcpy(DX,"999");										// wind direction maximum
//
//		strcpy(SN,"999");										// wind speed minimum
//		strcpy(CurrentSpeed,"999");					// wind speed mean
//		strcpy(SX,"999");	
//	}
//	Minute_WINDSENSOR_COUNTS = 0;
//
//	/* Collect - measure ATRH values */
//	if (!Minute_ATRH_COUNTS) {
//		strcpy(_Temperature,"-999");				// ATRH Temperature min , max
//		strcpy(_Humidity,"-999");						// ATRH Humidity min , max	
//	}
//	Minute_ATRH_COUNTS = 0;
//
//	memset(_buffer, 0, 512);	/* Clear the Buffer */
//
//	/* Combine the parameters and store it in _Sbuffer 
//	   - for sending it to portal and also storing it to SD card */
//	sprintf(_buffer,"#;%s;%s;%02d/%02d/%d;%02d:%02d:00;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%1d;%1d",
//					imei,FirmwareRevision,
//					(uint16_t)rtc.date,(uint16_t)rtc.month,(uint16_t)rtc.year,
//					(uint16_t)rtc.hour,(uint16_t)rtc.min,
//					_PVolt,_BVolt,
//					_Temperature,_Humidity,
//					CurrentSpeed,
//					CurrentWindDirection,
//					_sQuality,
//					_logInterval,_PacketlogInterval,_PacketsendInterval,
//					PDP,
//	        Card_Failed);
//
//	console_log("\n\r ---- Datalogger Status ---- :\n\r   %s\n\r",_buffer);			/* print response in terminal */
//  SMSDefaultSetting();
//	GSM_Send_Msg("9890799318",_buffer);	
////	Createlog(_buffer, "slog.txt");		/* Save packet to Slog - for reference */
//		
//}



/****************************************************************************************************
*                   get fattime [ Date and Time update for FatFS file system ]                      *
*****************************************************************************************************/
DWORD get_fattime(void)
{

	RTC_GetDateTime(&rtc);
	
	/* Pack date and time into a DWORD variable */
	return	  ((DWORD)(rtc.year - 1980) << 25)
			| ((DWORD)rtc.month << 21)
			| ((DWORD)rtc.date  << 16)
			| ((DWORD)rtc.hour  << 11)
			| ((DWORD)rtc.min   << 5)
			| ((DWORD)rtc.sec   >> 1);
}


