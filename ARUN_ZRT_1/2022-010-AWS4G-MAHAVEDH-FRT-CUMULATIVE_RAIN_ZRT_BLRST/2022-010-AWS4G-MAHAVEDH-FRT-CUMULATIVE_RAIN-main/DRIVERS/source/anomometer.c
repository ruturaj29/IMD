
/************************ include Header files *********************************/
#include <string.h>
#include <math.h>
#include "stdutils.h"
#include "systick.h"
#include "WSWD.h"
#include "delay.h"
#include "lcd.h"
#include "adc.h"
#include "uart.h"
#include "convert.h"
#include "extintr.h"
#include "anomometer.h"

/************************ Globle variables Define ******************************/
char _CurrentWindDir_A[10] = {0}, _CurrentWindSpd_A[10] = {0};
volatile unsigned int numRotateWind = 0,LCDnumRotateWind = 0;
volatile unsigned int ContactBounceTime=0;

/*********************************************************************************
*           External Interrupt Wind Speed - sensor		    						 				   *
**********************************************************************************/
void myExtIntrIsr_0(void)
{
  //if((SysTick_GetMsTime()-ContactBounceTime) > 15){
  numRotateWind++;
  LCDnumRotateWind++;
 // ContactBounceTime= SysTick_GetMsTime();
 // }
}

/*********************************************************************************
*           Display WindSensor		    						 				   										 *
**********************************************************************************/
void DisplayWindSensor(void) {
  	int adcValue5 = 0;
	volatile float DISPCurrentWindDir =0, DISPCurrentWindSpd =0;
	adcValue5 = ADC_GetAdcValue(5);
		
	DISPCurrentWindDir = (adcValue5 * 360)/4095; 
	if(DISPCurrentWindDir > 360)
		DISPCurrentWindDir = DISPCurrentWindDir - 360;

	if(DISPCurrentWindDir < 0)
		DISPCurrentWindDir = DISPCurrentWindDir + 360;

	/* Calculating wind speed from the external pulses */
	DISPCurrentWindSpd = LCDnumRotateWind * 2.25; // Speed = no.of Pulse*(2.25/Sample Period in seconds)
	//	DISPCurrentWindSpd = DISPCurrentWindSpd * 0.44704;	// converting mph to m/sec
    DISPCurrentWindSpd  = DISPCurrentWindSpd  * 1.60934; // converting mph to KM/hr
	console_log("ADC5 value:%4d Wind Dir:%f LCD Rotation Counter:%d Speed:%f kmph\n\r",
				adcValue5,DISPCurrentWindDir,LCDnumRotateWind,DISPCurrentWindSpd);     // Send the value on UART
	console_log("number of rotation counter: %d",numRotateWind);
	ftoa(DISPCurrentWindSpd,_CurrentWindSpd_A,1); // ASCII current Wind SPeed
	ftoa(DISPCurrentWindDir,_CurrentWindDir_A,0); // ASCII current Wind Direction

}

