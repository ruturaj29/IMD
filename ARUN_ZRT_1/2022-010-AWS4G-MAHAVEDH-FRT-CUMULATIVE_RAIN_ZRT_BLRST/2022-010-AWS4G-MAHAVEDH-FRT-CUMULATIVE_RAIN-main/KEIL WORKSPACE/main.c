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
	//NVIC_SetPriority(UART1_IRQn,0); // gsm comm uart
	NVIC_SetPriority(UART0_IRQn,1); // sonic sensor
	//NVIC_SetPriority(RTC_IRQn,  2);	// one minute alarm interrupt
	//NVIC_SetPriority(EINT3_IRQn,3);	// Ext interrupt 3 - rainguage
	//NVIC_SetPriority(EINT2_IRQn,4);	// Ext interrupt 2 - cup anemmometer
	//NVIC_SetPriority(EINT1_IRQn,5);	// Ext interrupt 1 - button	
	
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
	//WDT_Init(WDT_CLKSRC_IRC, WDT_MODE_RESET);	/* Initialize WDT, IRC OSC, Reset mode */
	
	//WDT_Start(WDT_TIMEOUT);	/* Start watchdog with given timeout */ 
	
	//BOD_Init();	/* brown out detect Initialize */
	
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
	if ((LPC_SC->RSID & 0x9) == 0x8) {
		LPC_SC->RSID |= 0x8;	// check Flag for BOD
		brownOutReset = 1;		// Error writing to a Err log 
	}
	else
		brownOutReset = 0;		// BOD reset not happend
	
	/* Read back TimeOut flag to determine previous timeout reset */
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

	DELAY_ms(2000);				/* wait to start module */

/**************************************************************************************************/



	/* Set the time and Date only once. Once the Time and Date is set, comment these lines
	   and reflash the code. Else the time will be set every time the controller is reset */
//	RTC_SetDateTime(&rtc);

/**************************************************************************************************/

	/* Initialize SD Card and FAT file system */


/**************************************************************************************************/

	RTC_GetDateTime(&rtc); /* Get RTC Time and Date */



/**************************************************************************************************/



/**************************************************************************************************/

	NVIC_EnableIRQ(UART0_IRQn);			// Enable Uart0 IRQ routine
	UART0_EnableBuffer_Interrupt(); // Enable Uart0 Buffer ISR 

	//NVIC_EnableIRQ(UART1_IRQn);			/* Enable UART 1 interrupt */ 
	//UART1_EnableBuffer_Interrupt();	/* Enable Buffer RBR UART 1 interrupt*/
	
		
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
	Module_sleep_mode();							/* put GSM module in Sleep mode */	
	UART1_DisableBuffer_Interrupt(); 	/* Disable Buffer RBR UART 1 interrupt */
		
	EINT_DetachInterrupt(0);
	EINT_DetachInterrupt(2);
	EINT_DetachInterrupt(3);
	EINT_DetachInterrupt(4);		
	EINT_DetachInterrupt(5);			
	while(1)
	{
	
		 Read_WindSensor();	
		 DELAY_ms(200);

		 if (!Minute_WINDSENSOR_COUNTS)
			{
				strcpy(_LCDCurrentWindDirection,"NA");	
				strcpy(_LCDCurrentSpeed,"NA");			
			}
				Minute_WINDSENSOR_COUNTS = 0;

				LCD_Clear();
				LCD_GoToLine(0);LCD_Printf("Wdir(deg): %s",_LCDCurrentWindDirection);
				LCD_GoToLine(1);LCD_Printf("Wspd(m/s): %s",_LCDCurrentSpeed);
	
	}	
		
}// main loop close 




