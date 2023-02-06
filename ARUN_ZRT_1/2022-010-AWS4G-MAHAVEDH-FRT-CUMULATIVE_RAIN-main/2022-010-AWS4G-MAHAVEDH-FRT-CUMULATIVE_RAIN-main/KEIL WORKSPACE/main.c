/**************************************************************************************************
                               Include Standerd Library 
**************************************************************************************************/
#include <lpc17xx.h>
#include <stdio.h>
#include <string.h>

/***************************************************************************************************
                             	USER Defined header files 
***************************************************************************************************/
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

/*  Watchodog time out is 10 minutes, as calculated
 *  there will be a wait time for response module on command. as calculated it is 600 seconds 
 *  2100000000	==> 35 minutes
 *   600000000  ==>	10 minutes
 *	 240000000  ==> 04 minutes
 *	 180000000	==> 03 minutes
 *   120000000  ==> 02 minutes
 *    60000000  ==> 01 minutes
 */
#define WDT_TIMEOUT  	180000000

/***************************************************************************************
                           Global Variables
****************************************************************************************/

/* variable for external interrupt */
extern volatile uint32_t KeyInterrupt, scrollScreen;
extern volatile unsigned int LCDnumTipsRain;
extern char _LCDCurrentWindDirection[4],_LCDCurrentSpeed[6];
/* RTC 1 minute interrupt */
extern volatile uint32_t alarm_on;

/* rtc structure declare */
rtc_t rtc;			
uint8_t returnStatus = 0, GPSCheck = 1;

int Card_Failed = 0;
char _buffer[512] = {'\0'};		/* main buffer for Storing as a packet sending to Portal */
char iniContent[512] = {'\0'}; 	/* storage for ini content from sd card	 */

/***********************************************************************************************/

/*********** local routines *************/
void NVIC_SetPriorities( void )
{
	/* Set IRQ Priorities */
	NVIC_SetPriority(UART1_IRQn,0); // gsm comm uart
	NVIC_SetPriority(UART0_IRQn,1); // sonic sensor
	NVIC_SetPriority(RTC_IRQn,  2);	// one minute alarm interrupt
	NVIC_SetPriority(EINT3_IRQn,3);	// Ext interrupt 3 - rainguage
	NVIC_SetPriority(EINT2_IRQn,4);	// Ext interrupt 2 - cup anemmometer
	NVIC_SetPriority(EINT1_IRQn,5);	// Ext interrupt 1 - button	
	
	/* Enable IRQ routiens */
	NVIC_EnableIRQ(RTC_IRQn);				// Enable RTC IRQ routine
}

void Configure_GPIO_Peripherals ( void )
{
	/* Configure GSM RESET pin*/
	GPIO_PinFunction(GSM_RESET,PINSEL_FUNC_0);   			// Configure Pin for Gpio
	GPIO_PinDirection(GSM_RESET,OUTPUT);       				// Configure the pin as OUTPUT

	/* Configure DTR pin for SLEEP Mode */
	GPIO_PinFunction(GSM_SLEEP_MODE,PINSEL_FUNC_0);		// Configure Pin for Gpio
	GPIO_PinDirection(GSM_SLEEP_MODE,OUTPUT);				 	// Configure the pin as OUTPUT

	/* Configure RS485 pin*/
	GPIO_PinFunction(RS485_Enable,PINSEL_FUNC_0);			// Configure Pin for Gpio
	GPIO_PinDirection(RS485_Enable,OUTPUT);						// Configure the pin as OUTPUT 

	/* Configure adc winddirection pin*/
	set_ResistorMode(1,31,PINSEL_PINMODE_PULLDOWN); 	// adc winddirection.
	set_OpenDrainMode(1,31,PINSEL_PINMODE_NORMAL);		// no open drain mode
	
	/* Configure adc battery pin*/
	set_ResistorMode(0,23,PINSEL_PINMODE_PULLDOWN); 	// adc battery.
	set_OpenDrainMode(0,23,PINSEL_PINMODE_NORMAL);		// no open drain mode
	
	/* Configure adc solar power pin*/
	set_ResistorMode(0,24,PINSEL_PINMODE_PULLDOWN); 	// adc solar power.
	set_OpenDrainMode(0,24,PINSEL_PINMODE_NORMAL);		// no open drain mode	

}

void Initialize_ALL_Peripherals ( void )
{
	/* Initialize all peripherals */
	WDT_Init(WDT_CLKSRC_IRC, WDT_MODE_INT_ONLY);	/* Initialize WDT, IRC OSC, Reset mode */
	
	WDT_Start(WDT_TIMEOUT);	/* Start watchdog with given timeout */ 
	
	BOD_Init();	/* brown out detect Initialize */
	
	ADC_Init();	/* ADC Initialize */

	/* Setup-Map the controller pins for LCD operation, In 4bit mode D0-D3 are P_NC(Not Connected)
	          RS    RW    EN    D0    D1    D2    D3    D4     D5     D6     D7                 */
	LCD_SetUp(P2_5, P2_6, P2_7, P_NC, P_NC, P_NC, P_NC, P0_19, P0_20, P0_21, P0_22);	  
	
	LCD_Init(2,16);	/* Specify the LCD type(2x16, 4x16 etc) for initialization */
	
	RTC_Init();	/* Internal RTC Initialize */

	SysTick_Init();	/* SysTick for 1ms(default) Initialize */

	/* Initialize All UARTs with Specific Baud rate */
	UART0_Init(19200);	/* Initialize uart0	- Sonic Sensor */
	UART1_Init(115200);	/* Initialize uart1	- GSM Module */
	UART2_Init(115200);	/* Initialize uart2 - Debugging Port */

}

/*-------------------------MAIN FUNCTION------------------------------*/
/*********************************************************************//**
 * @brief		main: Main  program body
 * @param[in]	None
 * @return 		int
 **********************************************************************/	

int main ()
{
	/* status flags are declaerd */
	uint8_t brownOutReset = 0, watchDogReset = 0, response=0, retry = 0 ,responseStatus =0;
	uint8_t i = 0, _LogBatteryLog = 1, BatteryLow_Flag = 0;
	uint32_t retries = 0;
	
	DSTATUS disk_state = STA_NOINIT;	/* SDcard flag declare */

	/* Custom char data (battery symbol) */
	uint8_t custom_char[] = { 0x0E, 0x1B, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1F };		

/**************************************************************************************************/
    
	SystemInit();	/* Clock and PLL configuration */
	 
	/* If BOD was detected set flag to 1*/
//	if ((LPC_SC->RSID & 0x9) == 0x8) {
//		LPC_SC->RSID |= 0x8;	// check Flag for BOD
//		brownOutReset = 1;		// Error writing to a Err log 
//	}
//	else
//		brownOutReset = 0;		// BOD reset not happend
//	
//	/* Read back TimeOut flag to determine previous timeout reset */
	if (WDT_ReadTimeOutFlag()) {
		WDT_ClrTimeOutFlag(); // Clear WDT TimeOut
		watchDogReset = 1;		// Error writing to a Err log
	}
	else 
		watchDogReset = 0;		// WDT reset not happend
	
	Initialize_ALL_Peripherals (); /* Initialize and setup all peripheral */

	Configure_GPIO_Peripherals (); /* Configure ALL peripherals and GPIO pins */ 

  NVIC_SetPriorities();	/* set priorities for interrupt */

/**************************************************************************************************/

	/* Print Welcome message on UART terminal	*/
	print_DebugMsg(" ---- System Reboot ---- \n\r");
	print_DebugMsg(" **** SkySense [AUTOMATIC WEATHER STATION(MAHAVEDH)] **** \n\r");

	/* Print Welcome message on LCD 16x2 */
	LCD_Clear();
	LCD_GoToLine(0);LCD_Printf(" -- SkySense -- "); 
	LCD_GoToLine(1);LCD_Printf(" MAHAVEDH [AWS] "); 

	ShutDownModule();		/* MCU GPIO PIN HIGH to OFF the regulator */
	RestartModule();		/* MCU GPIO PIN LOW to ON the regulator */
	DELAY_ms(2000);				/* wait to start module */

/**************************************************************************************************/

//	Init_ATRH();		/* Initialize ATRH Sensor --> I2C0 Init */
//	DELAY_ms(1000);
//
//	/* Wake up the ATRH sensor from sleep mode, Congif the sensor with register adddress */	
//	/* Try communicating sensor */
//	do {
//        response = start_ATRH(ATRH_Write_ADD,Config_register_add);
//        retry++;
//  }while((response != ATRH_INIT_SUCCESSFULL) && (retry!=10) );
//
//	if(response) { /* If ATRH Communication Failed */
//		print_DebugMsg(" ERROR : NO ATRH COMMINICATION \n\r");	
//		LCD_Clear();
//		LCD_Printf("Check ATRH Conn.");
//		DELAY_ms(2000); 		
//  }

	/* Set the time and Date only once. Once the Time and Date is set, comment these lines
	   and reflash the code. Else the time will be set every time the controller is reset */
//	RTC_SetDateTime(&rtc);

/**************************************************************************************************/

	/* Initialize SD Card and FAT file system */
  print_DebugMsg(" Initializing disk 0 (SDC)... \r\n");
	SysTick_Start();
  for (retries = 5; retries && disk_state; --retries) {
  	disk_state = disk_initialize(0); 
  }
	if (disk_state) { /* If SD Card failed or Disk initializ failed */
  	print_DebugMsg("Disk initialization failed.\r\n");
		LCD_Clear();
		LCD_GoToLine(0);LCD_Printf("SD Failed..");
		DELAY_ms(2000);
		Card_Failed = 1;
  }
	else {
		print_DebugMsg("Disk initialization success.\r\n");
		Card_Failed = 0;	
	}
	SysTick_Stop();

/**************************************************************************************************/

	RTC_GetDateTime(&rtc); /* Get RTC Time and Date */

	/* save system boot time in SD card */
	memset(_buffer, 0, 512);
	sprintf(_buffer,"[%02d/%02d/%d;%02d:%02d]:[INFO]:System Reboot;",
					(uint16_t)rtc.date,(uint16_t)rtc.month,(uint16_t)rtc.year,(uint16_t)rtc.hour,(uint16_t)rtc.min);
  console_log("buff : %s",_buffer);
	Createlog( _buffer , "err.txt");	/* Save log as a error - for reference */

	/* if system resets due to watchdog, write error log */
	if (watchDogReset) {
		RTC_GetDateTime(&rtc);
		memset(_buffer, 0, 512);
		sprintf(_buffer,"[%02d/%02d/%d;%02d:%02d]:[ERROR]:WDT RESET;",
						(uint16_t)rtc.date,(uint16_t)rtc.month,(uint16_t)rtc.year,(uint16_t)rtc.hour,(uint16_t)rtc.min);
		Createlog(_buffer, "err.txt");	/* Save log as a error - for reference */
		print_DebugMsg(" Last MCU reset caused by WDT TimeOut..!\n\r");
	}

	/* if system resets due to brownOutReset, write error log */
	if (brownOutReset) {
	  RTC_GetDateTime(&rtc);
		memset(_buffer, 0, 512);
		sprintf(_buffer,"[%02d/%02d/%d;%02d:%02d]:[ERROR]:BrownOut RESET;",
						(uint16_t)rtc.date,(uint16_t)rtc.month,(uint16_t)rtc.year,(uint16_t)rtc.hour,(uint16_t)rtc.min);
		Createlog(_buffer, "err.txt");	/* Save log as a error - for reference */
		print_DebugMsg("Last MCU reset caused by BrownOut TimeOut!\n\r");
	}

/**************************************************************************************************/

	/* Read INI file content */
	returnStatus = readconfigfile(iniContent);
	if (returnStatus) { /* IF INI file not Found */
    print_DebugMsg(" File Opening Failed \n\r");
	  LCD_Clear();
		LCD_GoToLine(0);LCD_Printf("Config ");
	  LCD_GoToLine(1);LCD_Printf("file not found.");
	  DELAY_ms(2000);

	  strcpy(URLbuffer,"http://iot.skymetweather.com/uniparser/mahavedh/mns"); // store url from sd card
	  strcpy(_logInterval, "1");	  			// store log interval from sd card
	  strcpy(_PacketlogInterval, "10");  	// store send interval from sd card
	  strcpy(_PacketsendInterval, "10"); 	// store pending log interval from sd card
  }
	else {
		console_log("iniContent:%s \n\r",iniContent);
		strcpy(URLbuffer, strtok(iniContent , "|"));		// store url from sd card
		strcpy(_logInterval, strtok(NULL, "|"));	  		// store log interval from sd card
		strcpy(_PacketlogInterval, strtok(NULL, "|"));  // store send interval from sd card
		strcpy(_PacketsendInterval, strtok(NULL, "|")); // store pending log interval from sd card
		strcpy(_apnBuffer, strtok(NULL, "|")); // store pending log interval from sd card
	}

	logInterval = ascii_integer(_logInterval);								/* convert ascii to int	*/
	PacketlogInterval = ascii_integer(_PacketlogInterval);		/* convert ascii to int	*/
	PacketsendInterval = ascii_integer(_PacketsendInterval);	/* convert ascii to int	*/

	/* print the converted values on terminal */
	console_log(" URL:%s \n\r LogInterval:%d \n\r PacketlogInterval:%d \n\r PacketSendInterval:%d \n\r APN:%s \n\r",
								URLbuffer, logInterval, PacketlogInterval, PacketsendInterval,_apnBuffer);

/**************************************************************************************************/

	NVIC_EnableIRQ(UART0_IRQn);			// Enable Uart0 IRQ routine
	UART0_EnableBuffer_Interrupt(); // Enable Uart0 Buffer ISR 

	NVIC_EnableIRQ(UART1_IRQn);			/* Enable UART 1 interrupt */ 
	UART1_EnableBuffer_Interrupt();	/* Enable Buffer RBR UART 1 interrupt*/
	
	/* Excutes the GSM module Initilization related functions */
//	for (i=2; i>0; i--) {
//		returnStatus = modem_initialization();
//		
//		if (returnStatus) break;	/* GSM Module initialize - return 1 on succesful */
//		else {
//			print_DebugMsg("GSM Communication Error !!\r\n");
//			LCD_Clear();
//			LCD_GoToLine(0); LCD_Printf("GSM Comm Error !");
//			LCD_GoToLine(1); LCD_Printf("Check module !?");
//			DELAY_ms(1000); 
//		}
//  }
//	
//	if (returnStatus) {	/* If GSM Module initialize succesful */
//		print_DebugMsg(" Modem Engine Initialization Successful \r\n");
//		LCD_Clear();
//		LCD_GoToLine(0);LCD_Printf("Modem Engine");
//	  LCD_GoToLine(1);LCD_Printf("Initialized !!");
//		DELAY_ms(200);
//
//		/* Initialize GPRS commands */
//		for (i=2; i>0; i--) {
//			returnStatus = modemCheck_initialization(APN); 
//			if (returnStatus) break;
//		}
//
//		if (returnStatus) { /* IF GPRS Initialize succesful */
//			
//			if(Query_Set_CLTS()){
//				MODEM_GetNetworkTime();	/* Update date and Time */ 
//			}
//
//			GSM_Delete_All_Msg();
//			DELAY_ms(200);
//
//			ReadNavigation();	/* Read GPS Co-ordinates */
//			DELAY_ms(200);
//
//			Module_deactivateBearerProfile();	/* Deactivate the PDP context of the module */
//			DELAY_ms(200);
//		}
//		else {	/* PDP context activating failed */
//			
//			WDT_Feed();	
//			ShutDownModule();			/* MCU GPIO PIN HIGH to OFF the regulator*/
//			RestartModule();		  /* MCU GPIO PIN LOW to ON the regulator*/
//			DELAY_ms(8000);				/* wait to start module */
//	
//			responseStatus = Module_Initializing();
//			
//			if(responseStatus){
//				print_DebugMsg("module activate successfully!\r\n");	
//			}
//			else
//			{
//			  LCD_Clear();
//				LCD_GoToLine(0);LCD_Printf("PDP Activation..");
//		  	LCD_GoToLine(1);LCD_Printf("failed");
//				DELAY_ms(200);
//	
//			  RTC_GetDateTime(&rtc);
//				memset(_buffer, 0, 512);
//				sprintf(_buffer,"[%02d/%02d/%d;%02d:%02d]:[ERROR]:MESSAGE DETAIL: %d;PDP failed;",
//								(uint16_t)rtc.date,(uint16_t)rtc.month,(uint16_t)rtc.year,(uint16_t)rtc.hour,(uint16_t)rtc.min,ERROR_CODE);
//				Createlog(_buffer, "err.txt");	/* Save log as a error - for reference */
//				print_DebugMsg(" PDP activation failed \n\r");
//			}	
//		}
//	}
//	else {	/* If GSM Module initialize failed */
//		print_DebugMsg("Modem Engine Initialization Failed \r\n");
//		LCD_Clear();
//		LCD_GoToLine(0);LCD_Printf("Modem Engine");
//	  LCD_GoToLine(1);LCD_Printf("Error Comm.. !!");
//		DELAY_ms(2000);
//	}

	Module_sleep_mode();							/* put GSM module in Sleep mode */	
	UART1_DisableBuffer_Interrupt(); 	/* Disable Buffer RBR UART 1 interrupt */

/**************************************************************************************************/

	/* mask off alarm mask, turn on IMYEAR in the counter increment interrupt register */
 	RTCSetAlarmMask(RTC_AMR_AMRSEC|RTC_AMR_AMRMIN|RTC_AMR_AMRHOUR|RTC_AMR_AMRDOM|
									RTC_AMR_AMRDOW|RTC_AMR_AMRDOY|RTC_AMR_AMRMON |RTC_AMR_AMRYEAR);
  LPC_RTC->CIIR = RTC_CIIR_IMMIN;


	/* Set - Configure and read the wind sensor paraemeterts */
	Configure_WindSensor();
	CheckCommunication_WindSensor();
	
/**************************************************************************************************/
	rtc_2call();  	//* Reset RTC register when bettery low flag set ,function used to check reset of rtc register when battery flag set and also reset rain . */
	ShutDownModule();	
	WDT_Feed();
		while(1)
	{
		 WDT_UpdateTimeOut(WDT_TIMEOUT);
		 Read_WindSensor();	
		 DELAY_ms(200);

		 //if (!Minute_WINDSENSOR_COUNTS)
		//	{
		//		strcpy(_LCDCurrentWindDirection,"NA");	
		//		strcpy(_LCDCurrentSpeed,"NA");			
		//	}
		//		Minute_WINDSENSOR_COUNTS = 0;

				LCD_Clear();
				LCD_GoToLine(0);LCD_Printf("Wdir(deg): %s",_LCDCurrentWindDirection);
				LCD_GoToLine(1);LCD_Printf("Wspd(m/s): %s",_LCDCurrentSpeed);
	
	}	
		
					    
/**************************************************************************************************/

	while(1)
	{
		WDT_UpdateTimeOut(WDT_TIMEOUT);	/* Update WDT timeout */
		Read_ADCchannels();							/* Read Battery voltage and Solar voltage */
		
		if (volt0 >= 1.47) {	/* Check battery voltage is above 9.30 V to activate the below commands */

/**************************************************************************************************/
	
			RTCStart();																					/* Enable RTC interrupt */
			EINT_AttachInterrupt(EINT1,myExtIntrIsr_1,FALLING);	/* Enable External interrupt for LCD scroll [ push button ] */
			EINT_AttachInterrupt(EINT2,myExtIntrIsr_0,FALLING);	/* Enable External interrupt for Wind Speed */
			EINT_AttachInterrupt(EINT3,myExtIntrIsr_3,FALLING);	/* Enable External interrupt for Rain Guage */
			SysTick_Start(); // use for anomometer 

			do{
					if (KeyInterrupt) LCD_ScrollDisplay(); /* Check key pressed */
					if(!KeyInterrupt) {
						LCD_Clear();
						LCD_GoToLine(0);LCD_Printf(" -- SkySense -- ");
						LCD_GoToLine(1);LCD_Printf(" MAHAVEDH [AWS] ");
						DELAY_ms(50);
					}
			}while (alarm_on == 0);/* check 1 minute alarm */

			alarm_on = 0;									/* Clear the RTC 1 minute Interrupt alarm */
			KeyInterrupt = 0;							/* Clear the External interrupt flag */
			scrollScreen = 0;							/* Clear the External interrupt flag */
			LCDnumTipsRain= 0;						/* Clear LCD rain count */
			RTC_IntDisable();							/* Disable Internal RTC Counter increment interrupt */
			EINT_DetachInterrupt(EINT1);	/* Disable External interrupt for push button switch */
			EINT_DetachInterrupt(EINT3);	/* Disable External interrupt for Rain Guage */
			EINT_DetachInterrupt(EINT2);	/* Disable External interrupt for Wind Speed */
			SysTick_Stop();		 						// use for anomometer

/**************************************************************************************************/

			/* read sensor values */
			readATRH(ATRH_Write_ADD,ATRH_READ_ADD,Temperature_register_add);	/* Read ATRH sensor */
			Read_WindSensor();	//FRT Read wind Sensor 
			
/**************************************************************************************************/
																								/* Read Wind sensor */
			RTC_GetDateTime(&rtc);																						/* Read RTC time and Date */
			console_log("%02d/%02d/%d;%02d:%02d \n\r",(uint16_t)rtc.date,(uint16_t)rtc.month,
								  (uint16_t)rtc.year,(uint16_t)rtc.hour,(uint16_t)rtc.min);

			if (rtc.min % logInterval == 0) {
				Slog();
			}

			if (rtc.min % PacketlogInterval == 0) {
				PacketLog();
			}

			if ((rtc.min % PacketsendInterval == 0) && (volt0 > 1.55)) {
				SendingLog();
			}

/**************************************************************************************************/

		}//if (volt0 >= 1.47) 
		else {

			/* Create custom char	*/
    	lcd16x2_create_custom_char(0, custom_char);
			LCD_Clear();

			/* Display custom char */
 	    lcd16x2_put_custom_char(0, 0, 0);
			LCD_Printf("  Battery Low");
			DELAY_ms(500);
			BatteryLow_Flag = 1; /* Battery low flag set */

			 
/**************************************************************************************************/

			/* Below condition is to log only once in sd card error log */
			if((BatteryLow_Flag == 1) && (_LogBatteryLog == 1)) {
			RTC_GetDateTime(&rtc);
			memset(_buffer, 0, 512);
			sprintf(_buffer,"[%02d/%02d/%d;%02d:%02d]:[ERROR]:BATTERY LOW:Voltage:%02s;",
							(uint16_t)rtc.date,(uint16_t)rtc.month,(uint16_t)rtc.year,(uint16_t)rtc.hour,(uint16_t)rtc.min,_BVolt);
			Createlog(_buffer, "err.txt");	/* Save log as a error - for reference */
			print_DebugMsg(" BATTERY LOW \n\r");
			_LogBatteryLog = 0; 			/* One time write excuted in sd card error log */

			RTC_WriteGPREG(2,1); //* Battery low flag set in RTC  register 2 (bettery low problem occure purpose) */
		    console_log(" \n\r Flag RTC 2 RTC_ReadGPREG(2) : %f\n\r" , RTC_ReadGPREG(2));  //check its bettery low flag set or not?//
			}

/**************************************************************************************************/

		}
	}//while(1) loop end
}// main loop close 

