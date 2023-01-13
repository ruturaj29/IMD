#include<lpc17xx.h>
#include <stdio.h>
#include "SHT.h"
#include "delay.h"
#include "uart.h" 
#include "convert.h"

volatile float temperature, humidity, Sum_of_temperature, Sum_of_humidity;
volatile float fMinTemp,fMaxTemp,fMinHumid,fMaxHumid;
char _Temperature[10]={0}, _Humidity[10]={0},ATRH_COUNTS,Minute_ATRH_COUNTS = 0;
char  _MinTemp[10]={0}, _MaxTemp[10]={0}, _MinHumid[10]={0}, _MaxHumid[10]={0},FirstRead_ATRH = 1;	

void SHT11_Init( void )
{	
	GPIO_PinDirection(SDA_PIN, OUTPUT);
    GPIO_PinDirection(SCL_PIN, OUTPUT);										
	GPIO_PinWrite(SDA_PIN,HIGH);
	GPIO_PinWrite(SCL_PIN,HIGH);						  

}

void SHT11_wait( void ) //SHT_WAIT should be tuned acording to MCU clock
{
    volatile int wait_cnt;
    for ( wait_cnt = 0; wait_cnt < SHT_WAIT ;wait_cnt )
    {
        wait_cnt++;
    }
}

void SHT11_reset_condition( void )
{
	uint8_t i;
  	GPIO_PinWrite(SDA_PIN,HIGH);
    GPIO_PinDirection(SDA_PIN, OUTPUT);							
    for ( i = 0;i < 12;i++ )
    {
        if ( i % 2 == 0 )
            GPIO_PinWrite(SCL_PIN,HIGH);
        else
            GPIO_PinWrite(SCL_PIN,LOW);
        SHT11_wait();
   }
}

void SHT11_start_condition( void )
{

    GPIO_PinWrite(SDA_PIN,HIGH);						
    GPIO_PinDirection(SDA_PIN, OUTPUT);
    SHT11_wait();
    GPIO_PinWrite(SCL_PIN,HIGH);						  
    SHT11_wait();
    GPIO_PinWrite(SDA_PIN,LOW);					 		
    SHT11_wait();
    GPIO_PinWrite(SCL_PIN,LOW);		   				
    SHT11_wait();
    GPIO_PinWrite(SCL_PIN,HIGH);
    SHT11_wait();
    GPIO_PinWrite(SDA_PIN,HIGH);
    SHT11_wait();
    GPIO_PinWrite(SCL_PIN,LOW);
    SHT11_wait();

}

int SHT11_send_byte( uint8_t sht_data2send )
{
    uint8_t i = 0;
    while ( i <= 7 )
    {
        if ( ( ( sht_data2send << i++ ) & 0x80 ) == 0x80 )
            GPIO_PinWrite(SDA_PIN,HIGH);
        else
            GPIO_PinWrite(SDA_PIN,LOW);
        GPIO_PinDirection(SDA_PIN, OUTPUT);
        SHT11_wait();
        GPIO_PinWrite(SCL_PIN,HIGH);
        SHT11_wait();
         GPIO_PinWrite(SCL_PIN,LOW);

    }
     GPIO_PinDirection(SDA_PIN, INPUT);
    SHT11_wait();
    GPIO_PinWrite(SCL_PIN,HIGH);
    SHT11_wait();
    if ( GPIO_PinRead(SDA_PIN) == 0 )
        return 1;

    else
        return -1;
}

int SHT11_data_wait(uint32_t sht_timeout )
{
    static uint8_t sht_wait_state;
    static uint32_t sht_previous_ticks;

    switch ( sht_wait_state )
    {
    case SHT_DATA_WAIT_INITIAL_STATE:
        sht_previous_ticks = sht_msec_ticks;
        sht_wait_state = SHT_DATA_WAIT_TIMEOUT_STATE;
        GPIO_PinDirection(SDA_PIN, INPUT);
         GPIO_PinWrite(SCL_PIN,LOW);

    case SHT_DATA_WAIT_TIMEOUT_STATE:
        if ( GPIO_PinRead(SDA_PIN) == 0 )
        {
            sht_wait_state = SHT_DATA_WAIT_INITIAL_STATE;
            return 1;
        }
        else
        {
            if ( ( sht_msec_ticks - sht_previous_ticks ) > sht_timeout )
            {
                sht_wait_state = SHT_DATA_WAIT_INITIAL_STATE;
                return -1;
            }
            else
                return 0;
        }
    }
	return 0; // missing return statement
}

uint8_t sht_read_byte( uint8_t sht_ack )
{
     
    uint8_t temp_rx_buff = 0;
    int8_t i = 7;
	GPIO_PinDirection(SDA_PIN, INPUT);
    while ( i >= 0 )
    {
        SHT11_wait();
        GPIO_PinWrite(SCL_PIN,HIGH);
        temp_rx_buff |= ( ( GPIO_PinRead(SDA_PIN) & 0x01 ) << i );
        i--;
        SHT11_wait();
        GPIO_PinWrite(SCL_PIN,LOW);

    }

    if ( sht_ack == 1 )
    {
        GPIO_PinWrite(SDA_PIN,LOW);
        GPIO_PinDirection(SDA_PIN, OUTPUT);			
        SHT11_wait();
        GPIO_PinWrite(SCL_PIN,HIGH);
        SHT11_wait();
        GPIO_PinWrite(SCL_PIN,LOW);
        GPIO_PinDirection(SDA_PIN, INPUT);

    }
    return temp_rx_buff;
}

int SHT11_proc( float sht_param, float *ret_value )
{

    static uint8_t sht_proc_state = 0;
    int8_t	wait_status;

    switch ( sht_proc_state )
    {
    case SHT_PROC_RESET_COND_STATE:
        SHT11_reset_condition();
    case SHT_PROC_START_COND_STATE:
        SHT11_start_condition();
    case SHT_PROC_SEND_STATE:
        SHT11_send_byte( sht_param );
    case SHT_PROC_WAIT_STATE:
        wait_status = SHT11_data_wait( 300 );
        if ( wait_status == -1 )
        {
            sht_proc_state = SHT_PROC_RESET_COND_STATE;
            return -1;

        }
        if ( wait_status == 0 )
        {
            sht_proc_state = SHT_PROC_WAIT_STATE;
            return 0;
        }
        else
            sht_proc_state = SHT_PROC_RETURN_STATE;
    case SHT_PROC_RETURN_STATE:
        *ret_value = ( ( uint16_t ) sht_read_byte( 1 ) << 8 );
        SHT11_wait();
        *ret_value += sht_read_byte( 0 );
        sht_proc_state = SHT_PROC_START_COND_STATE;
        return 1;

    }
	return 0;// missing return statement
}

int SHT11_get_temp( float *ret_temperature )
{
    static float tmp_temp;
    if ( SHT11_proc( TEMPERATURE_BYTE, &tmp_temp ) == 1 )
    {
        *ret_temperature = tmp_temp - 3965;
        return 1;
    }
    else
        return 0;
}

int SHT11_get_temp_RH( volatile float *ret_temperature, volatile float *ret_humidity )
{

    static uint16_t sht_humidity_state;
    static float sht_humidity_raw;
    static float sht_temp_C;

    static float RH_linear;
    static float RH_compensated;

    switch ( sht_humidity_state )
    {
	    case SHT_RH_READ_TEMPERATURE_STATE:
	        if ( SHT11_get_temp( &sht_temp_C ) )
	            sht_humidity_state = SHT_RH_READ_HUMIDITY_STATE;
	       break;
	    case SHT_RH_READ_HUMIDITY_STATE:
	        if ( SHT11_proc( HUMIDITY_BYTE, &sht_humidity_raw ) )
	            sht_humidity_state = SHT_RH_CONVERT_RH_STATE;
	       break;
	    case SHT_RH_CONVERT_RH_STATE:
	        RH_linear =  ( ( 0.0405 *  sht_humidity_raw ) - ( 0.0000028 * sht_humidity_raw *  sht_humidity_raw ) - 4 );
	        RH_compensated =  ( ( ( ( (  sht_temp_C ) / 100 ) - 25 ) * ( 0.01 + ( 0.00008 * sht_humidity_raw ) ) ) + RH_linear );
	        sht_humidity_state = SHT_RH_READ_TEMPERATURE_STATE;
	        *ret_temperature = sht_temp_C / 100;
	        *ret_humidity = ( ( RH_compensated  ) );
			
	        return 1;
	    default:
	        sht_humidity_state = SHT_RH_READ_TEMPERATURE_STATE;
    }
	return 0;
}




void read_SHT11(void)
{
	char retry = 0, response = 0;
	
  	do
	{
		response = SHT11_get_temp_RH( &temperature, &humidity ); //send 'reset & go idle' command
		retry++;
		DELAY_ms(50);
	    if(retry>0x10)
			break;		
	} while(response != 0x01);

	if(response == 0x01)
	{
	  console_log("\n\rTemperature = %f, Humidity =  %f\n\r",temperature,humidity);
	  retry = 0;
	  response = 0;

	 // 	humidity = humidity - 4.0;
	//	temperature = temperature - 0.45;

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
	}
	else
	{
		print_DebugMsg(" ERROR : NO SHT11 COMMINICATION \n\r");
		retry = 0;
		response = 0;

	}
	DELAY_ms(300);
}

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


