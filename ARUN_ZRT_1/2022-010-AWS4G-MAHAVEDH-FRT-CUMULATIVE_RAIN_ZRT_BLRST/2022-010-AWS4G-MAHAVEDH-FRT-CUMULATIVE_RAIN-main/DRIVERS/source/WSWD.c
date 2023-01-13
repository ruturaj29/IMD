
/************************ include Header files *******************************/
#include <string.h>
#include <math.h>
#include "stdutils.h"
#include "WSWD.h"
#include "delay.h"
#include "lcd.h"
#include "uart.h"
#include "convert.h"

/************************* Globle variables *********************************/
/*********** UART *****************/
extern volatile uint32_t UART0Count;
extern char	UART0Buffer[BUFSIZE];

/*********** LCD Scroll **************/
extern char _LCDCurrentWindDirection[4],_LCDCurrentSpeed[6];

/*********** Ultrasonic Sensors ********/
char DN[4],DM[4],DX[4],FirstRead_WindSensor=1;
char SN[6],SM[6],SX[6],Gust[6],WINDSENSOR_COUNTS,Minute_WINDSENSOR_COUNTS = 0;
char CurrentWindDirection[4]= {0},CurrentSpeed[6]= {0};
char _CurrentWindDir[10] = {0}, _CurrentWindSpd[10] = {0};
char MinSpeed[6];

char Tan_Inverse_WindDirection[6];

double f_DM, f_DN, f_DX,Sum_of_f_DM,Sum_of_f_DX;
double f_SM, f_SN, f_SX,Sum_of_f_SM,Sum_of_f_SX;
double fmax_DX,fmin_DN,favg_DM;
double fmax_SX,fmin_SN,favg_SM;

double sumof_sin_value_direction;
double sumof_cosine_value_direction;

/*****************************************************************************************
*                  Configure Wind Sensor                 									  						 *
*****************************************************************************************/
void Configure_WindSensor(void)
{
  /* Set the Sensor parameters */
	print_DebugMsg("-- Command for configuring Wind Sensor in polled mode --\r\n");
	GPIO_PinWrite(RS485_Enable,1);
//	UART0_TxString("0XU,A=0,M=A,T=1,C=2,I=00115,B=019200,D=8,P=N,S=1,L=00000\r\n");
	UART0_TxString("0XU,A=0,M=P,T=1,C=2,I=0060,B=019200,D=8,P=N,S=1,L=00000\r\n");
	DELAY_ms(10);
	GPIO_PinWrite(RS485_Enable,0);
	DELAY_ms(50);
//	UART3_Printf("%s\n\r",UART0Buffer); // print response in terminal
	Buffer_Flush_UART0();
//	memset(UART0Buffer, 0, BUFSIZE-1);
}

/*****************************************************************************************
*                  Set Default Wind Sensor values        									  						 *
*****************************************************************************************/
void SetDefault_WindSensor(void)
{
/* Reset the sesnor to default state */
	print_DebugMsg("-- Command to Set Wind Sensor in Default --\r\n");
	GPIO_PinWrite(RS485_Enable,1);
	UART0_TxString("0XZ\r\n");
	DELAY_ms(10);
	GPIO_PinWrite(RS485_Enable,0);
	DELAY_ms(500);
	console_log("%s\n\r",UART0Buffer); // print response in terminal
	Buffer_Flush_UART0();
//	memset(UART0Buffer, 0, BUFSIZE-1);
}

/*****************************************************************************************
*                  Check Communication WindSensor        									  						 *
*****************************************************************************************/
void CheckCommunication_WindSensor(void)
{
	/* Check the communiation parameters */
	print_DebugMsg("-- Command to check communication parameters of Wind Sensor\r\n");
  GPIO_PinWrite(RS485_Enable,1);
	UART2_TxString("0XU\r\n");
	UART0_TxString("0XU\r\n");
	DELAY_ms(10);
	GPIO_PinWrite(RS485_Enable,0);
	DELAY_ms(500);
	console_log("%s\n\r",UART0Buffer); // print response in terminal
	Buffer_Flush_UART0();
//	memset(UART0Buffer, 0, BUFSIZE-1);
}

/*****************************************************************************************
*                  Read WindSensor        									  				              		 *
*****************************************************************************************/
void Read_WindSensor(void)
{
	unsigned int i=0;
	
	/* Clear the UART 0 buffer*/
	Buffer_Flush_UART0();

	/* Read the sensor by sending the command */
	UART2_TxString("-- Command for current Wind Sensor readings --\r\n");
 	GPIO_PinWrite(RS485_Enable,1);
	UART2_TxString("0R0\r\n"); //0R1 FRT OLD
	UART0_TxString("0R0\r\n"); //0R1 FRT OLD
	DELAY_ms(10);
	GPIO_PinWrite(RS485_Enable,0);
	DELAY_ms(500);
	console_log("%s\n\r",UART0Buffer); // print response in terminal
	
	i = strlen(UART0Buffer);
//	UART3_Printf("%d\n\r",i); // print response in terminal

	/* Check if UART0 interrupt buffer (Wind sensor) recieved any characters */
	if(i > 0)
	{
	//	UART3_Printf("%s\n\r",UART0Buffer); // print response in terminal
  			
  		for(i=0;i<56;i++)
		{
			/* wind direction minimum */
    		if(UART0Buffer[i] == 'D' && UART0Buffer[i+1] == 'n')
			{
	      	DN[0] = UART0Buffer[i + 3];
		  		DN[1] = UART0Buffer[i + 4];
		  		DN[2] = UART0Buffer[i + 5];
			}
			/* wind direction mean */
    		if(UART0Buffer[i] == 'D' && UART0Buffer[i+1] == 'm')
			{
		  		CurrentWindDirection[0] = UART0Buffer[i + 3];
		  		CurrentWindDirection[1] = UART0Buffer[i + 4];
		  		CurrentWindDirection[2] = UART0Buffer[i + 5];
    		}
			/* wind direction maximum */
	 		if(UART0Buffer[i] == 'D' && UART0Buffer[i+1] == 'x')
			{
	 	  		DX[0] = UART0Buffer[i + 3];
		  		DX[1] = UART0Buffer[i + 4];
		  		DX[2] = UART0Buffer[i + 5];
			}
			/* wind speed minimum */
    		if(UART0Buffer[i] == 'S' && UART0Buffer[i+1] == 'n')
			{
					SN[0] = UART0Buffer[i + 3];
		  		SN[1] = UART0Buffer[i + 4];
		  		SN[2] = UART0Buffer[i + 5];
		  		SN[3] = UART0Buffer[i + 6];
		  		SN[4] = UART0Buffer[i + 7];
			}
			/* wind speed mean */
			if(UART0Buffer[i] == 'S' && UART0Buffer[i+1] == 'm')
			{
	 	 			CurrentSpeed[0] = UART0Buffer[i + 3];
		  		CurrentSpeed[1] = UART0Buffer[i + 4];
		  		CurrentSpeed[2] = UART0Buffer[i + 5];
		  		CurrentSpeed[3] = UART0Buffer[i + 6];
		  		CurrentSpeed[4] = UART0Buffer[i + 7];
			}
			/* wind speed maximum */
			if(UART0Buffer[i] == 'S' && UART0Buffer[i+1] == 'x')
			{
	 	  		SX[0] = UART0Buffer[i + 3];
		  		SX[1] = UART0Buffer[i + 4];
		  		SX[2] = UART0Buffer[i + 5];
		  		SX[3] = UART0Buffer[i + 6];
		  		SX[4] = UART0Buffer[i + 7];
			}
		}

   		DN[3] = '\0';
   		CurrentWindDirection[3] = '\0';
   		DX[3] = '\0'; 
   		SN[5] = '\0';
   		CurrentSpeed[5] = '\0';
   		SX[5] = '\0';
			DELAY_ms(50);
	//	Buffer_Flush_UART0();


		if(FirstRead_WindSensor)
		{
			fmin_DN = stof(DN); // converting min direction string to float 
//			favg_DM = stof(CurrentWindDirection); // converting mean direction string to float
			fmax_DX = stof(DX);	 // converting max direction string to float

			fmin_SN = stof(SN);	// converting min speed string to float
//			favg_SM = stof(CurrentSpeed); // converting mean speed string to float
			fmax_SX = stof(SX);	// converting max direction speed to float

			FirstRead_WindSensor = 0; // Clear the first reading
		}

		f_DN = stof(DN); // converting min direction string to float
		f_DM = stof(CurrentWindDirection); // converting mean direction string to float
		f_DX = stof(DX); // converting max direction string to float

		f_SN = stof(SN); // converting min speed string to float
		f_SM = stof(CurrentSpeed);	// converting mean speed string to float
		f_SX = stof(SX); // converting max direction speed to float

		/* Check for max wind Direction */
		if(f_DX>fmax_DX)
		{
		 	fmax_DX =  f_DX; // max direction
		}
		/* check for min wind Direction */
		if(f_DN<fmin_DN)
		{
		  fmin_DN =  f_DN; // min direction
		}

		/* check for min wind Speed */
		if(f_SN<fmin_SN)
		{
		  fmin_SN =  f_SN; // min speed
		}		
		/* Check for max wind Speed */
		if(f_SX>fmax_SX)
		{
		 	fmax_SX =  f_SX; // max speed
		}


		UART2_TxString("Wind float values: \r");
	  console_log("WDmin:%f Current_WDMean:%f WDmax:%f WSmin:%f WSmean:%f WSmax:%f\n\r",
		f_DN,f_DM,f_DX,f_SN,f_SM,f_SX);// print response in terminal

		Sum_of_f_DM = Sum_of_f_DM + f_DM;
		Sum_of_f_SM = Sum_of_f_SM + f_SM;

		WINDSENSOR_COUNTS = WINDSENSOR_COUNTS+1;
		Minute_WINDSENSOR_COUNTS   = 1; // log flag set to idnetify the sensor reading in a minute

//		ftoa(f_DM,_CurrentWindSpd,2); // ASCII current Wind SPeed
//	    ftoa(f_SM,_CurrentWindDir,2); // ASCII current Wind Direction

		strcpy(_LCDCurrentWindDirection ,CurrentWindDirection);
		strcpy(_LCDCurrentSpeed ,CurrentSpeed);


		/* conversion to sine ans cosine of Wind direction f_DM */
		Trignometric_Conversion();
	}
	
}

/*****************************************************************************************
*                  Read WindSensor Interrupt Buffer        						              		 *
*****************************************************************************************/
void Read_WindSensor_InterruptBuffer()
{
	unsigned int i=0;	

//	UART3_TxString("Read Wind Sensor from interrupt buffer - Automatic\r\n");

	i = strlen(UART0Buffer);
	//UART3_Printf("%d\n\r",i); // print response in terminal

	/* Check if UART0 interrupt buffer (Wind sensor) recieved any characters */
	if(i > 0)
	{
		console_log("%s\n\r",UART0Buffer); // print response in terminal
  			
  		for(i=0;i<56;i++)
		{
			/* wind direction minimum */
    		if(UART0Buffer[i] == 'D' && UART0Buffer[i+1] == 'n')
			{
	      	DN[0] = UART0Buffer[i + 3];
		  		DN[1] = UART0Buffer[i + 4];
		  		DN[2] = UART0Buffer[i + 5];
			}
			/* wind direction mean */
    		if(UART0Buffer[i] == 'D' && UART0Buffer[i+1] == 'm')
			{
		  		DM[0] = UART0Buffer[i + 3];
		  		DM[1] = UART0Buffer[i + 4];
		  		DM[2] = UART0Buffer[i + 5];
    		}
			/* wind direction maximum */
	 		if(UART0Buffer[i] == 'D' && UART0Buffer[i+1] == 'x')
			{
	 	  		DX[0] = UART0Buffer[i + 3];
		  		DX[1] = UART0Buffer[i + 4];
		  		DX[2] = UART0Buffer[i + 5];
			}
			/* wind speed minimum */
    		if(UART0Buffer[i] == 'S' && UART0Buffer[i+1] == 'n')
			{
					SN[0] = UART0Buffer[i + 3];
		  		SN[1] = UART0Buffer[i + 4];
		  		SN[2] = UART0Buffer[i + 5];
		  		SN[3] = UART0Buffer[i + 6];
		  		SN[4] = UART0Buffer[i + 7];
			}
			/* wind speed mean */
			if(UART0Buffer[i] == 'S' && UART0Buffer[i+1] == 'm')
			{
	 	 			SM[0] = UART0Buffer[i + 3];
		  		SM[1] = UART0Buffer[i + 4];
		  		SM[2] = UART0Buffer[i + 5];
		  		SM[3] = UART0Buffer[i + 6];
		  		SM[4] = UART0Buffer[i + 7];
			}
			/* wind speed maximum & Gust*/
			if(UART0Buffer[i] == 'S' && UART0Buffer[i+1] == 'x')
			{
	 	  		SX[0] = UART0Buffer[i + 3];
		  		SX[1] = UART0Buffer[i + 4];
		  		SX[2] = UART0Buffer[i + 5];				 
		  		SX[3] = UART0Buffer[i + 6];
		  		SX[4] = UART0Buffer[i + 7];
			}
		}

   		DN[3] = '\0';
   		DM[3] = '\0';
   		DX[3] = '\0'; 
   		SN[5] = '\0';
   		SM[5] = '\0';
   		SX[5] = '\0';

		strcpy(Gust,SX);	// Copy Maximum speed to Gust


	Buffer_Flush_UART0();
//	UART3_Printf("WDmin:%s WDmean:%s WDmax:%s WSmin:%s WSmean:%s WSmax:%s Gust:%s\n\r",DN,DM,DX,SN,SM,SX,Gust);// print response in terminal
	}
}

/*****************************************************************************************
*                  Buffer Flush UART0        			              			              		 *
*****************************************************************************************/
void Buffer_Flush_UART0(void)			/* Flush all variables */
{
	memset(UART0Buffer, 0, BUFSIZE-1);
	UART0Count=0;
}

/*****************************************************************************************
*                  Average WindSamples        			              			            		 *
*****************************************************************************************/
void Average_WindSamples(void)
{
	/* Average formula: Average = (Sum of Samples/Count)*/
	double Average_f_SM = 0;

//	Sum_of_f_SM = Sum_of_f_SM + f_SM;
	Average_f_SM = Sum_of_f_SM/WINDSENSOR_COUNTS;
	
	ftoa_signed(Average_f_SM,SM,2); // ASCII Mean of Wind Speed
	ftoa_signed(fmax_SX,SX,2); // ASCII Max of Wind Speed
	ftoa_signed(fmin_SN,MinSpeed,2); // ASCII Min of Wind Speed

//	ftoa(Average_f_DM,DM,2); // ASCII Mean of Wind Direction
	ftoa_signed(fmax_DX,DX,2); // ASCII Max of Wind Direction
	ftoa_signed(fmin_DN,DN,2); // ASCII Min of Wind Direction
	average_windDirection();

	strcpy(Gust,SX);
	/* clear varaibles and counts for next samples */
	Sum_of_f_SM = 0;

	fmax_SX = 0;
	fmin_SN = 0;
	fmax_DX = 0;
	fmin_DN = 0;
	FirstRead_WindSensor = 1;
}

/*****************************************************************************************
*                  Trignometric Conversion        		             			            		 *
*****************************************************************************************/
void Trignometric_Conversion(void)
{
 	double sin_value_direction =0,cosine_value_direction = 0;

	/* Convert degree value to radian */
	/* apply sine and cosine values */
	sin_value_direction = sin(f_DM * (3.14/180.0));
	cosine_value_direction = cos(f_DM * (3.14/180.0));

//	UART3_Printf("Sine(%f):%f deg Cosine(%f):%f deg\n\r",
//	f_DM,sin_value_direction,f_DM,cosine_value_direction); // Print in terminal 

  sumof_sin_value_direction = sumof_sin_value_direction + sin_value_direction;
	sumof_cosine_value_direction = sumof_cosine_value_direction + cosine_value_direction;

//	UART3_Printf("Sum of Sine samples:%f Sum of cosine samples:%f SampleCount:%d\n\r",
//	sumof_sin_value_direction,sumof_cosine_value_direction,WINDSENSOR_COUNTS); // Print in terminal

}

/*****************************************************************************************
*                  average windDirection        			             			            		 *
*****************************************************************************************/
void average_windDirection(void)
{
	double avg_sumof_sin_value_direction =0 ,avg_sumof_cosine_value_direction =0;
	double Dividend_of_sum_Sine_Cosine = 0;
	double FinalAvg_Direction=0, RadianAvg_Direction=0;

	avg_sumof_sin_value_direction = sumof_sin_value_direction/WINDSENSOR_COUNTS;
	avg_sumof_cosine_value_direction = 	sumof_cosine_value_direction/WINDSENSOR_COUNTS;
//	UART3_Printf("Average of Sine: %f\n\r",avg_sumof_sin_value_direction); // Print in terminal
//	UART3_Printf("Average of cosine: %f\n\r",avg_sumof_cosine_value_direction); // Print in terminal

	Dividend_of_sum_Sine_Cosine = avg_sumof_sin_value_direction/avg_sumof_cosine_value_direction;			
//	UART3_Printf("Dividend of Sine Cosine: %f\n\r",Dividend_of_sum_Sine_Cosine); // Print in terminal

	//radianAvg = Dividend_of_sum_Sine_Cosine*(3.14/180.0);
	RadianAvg_Direction = atan(Dividend_of_sum_Sine_Cosine); // tan inverse
	DELAY_ms(300);
//	UART3_Printf("TanInverse(%f): %f radian\n\r",Dividend_of_sum_Sine_Cosine,RadianAvg_Direction); // Print in terminal			
		
	/* Convert final average from radian to Degree */
	FinalAvg_Direction = (180.0/3.14)*RadianAvg_Direction;
//	UART3_Printf("TanInverse(%f): %f deg\n\r",Dividend_of_sum_Sine_Cosine,FinalAvg_Direction); // Print in terminal
							 
	if(avg_sumof_sin_value_direction > 0 && avg_sumof_cosine_value_direction > 0 )
	{
		//Do nothing
		UART2_TxString("Both Sine and cosine are positive\r\n");
	}
	if(avg_sumof_cosine_value_direction < 0)
	{
		UART2_TxString("Only cosine is negative\r\n");
		FinalAvg_Direction = FinalAvg_Direction+180.0;	
	}
	if(avg_sumof_sin_value_direction < 0 && avg_sumof_cosine_value_direction > 0)
	{
		UART2_TxString("Sine is negative, Cosine is positive\r\n");
		FinalAvg_Direction = FinalAvg_Direction+360.0;	
	}

	console_log("Final Mean Direction: %f deg\n\r",FinalAvg_Direction); // Print in terminal
	ftoa_signed(FinalAvg_Direction,Tan_Inverse_WindDirection,2);
	

	/* clear varaibles and counts for next samples */
	sumof_sin_value_direction = 0;
	sumof_cosine_value_direction = 0;
	WINDSENSOR_COUNTS = 0;
}





