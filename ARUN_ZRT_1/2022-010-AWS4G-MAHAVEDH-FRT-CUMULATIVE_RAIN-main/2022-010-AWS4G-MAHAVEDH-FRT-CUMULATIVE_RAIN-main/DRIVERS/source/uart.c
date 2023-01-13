/**************************************************************************************************
                             Revision History
 ****************************************************************************************************
15.0: Initial version 
15.1: Updated the UART_TxNumber function to display Bin/Dec/Hex numbers.
      Removed the functions UART_TxHexNumber and UART_TxBinaryNumber.
 ***************************************************************************************************/

/************** ALL header files ***************************************************************************************/
#include <stdarg.h>
#include <stdbool.h>
#include "uart.h"
#include "gpio.h"

/************** Globle Macro  Defines **********************************************************************************/
#define C_MaxUartChannels_U8 4u
#define C_UartOne_U8         1u

/************** Globle Variable Declerations ***************************************************************************/
volatile uint32_t UART0Status, UART1Status, UART3Status;
volatile uint8_t UART0TxEmpty = 1, UART1TxEmpty = 1, UART3TxEmpty = 1;
volatile uint8_t UART0Buffer[BUFSIZE],UART1Buffer[BUFSIZE], UART3Buffer[BUFSIZE]; 
volatile uint32_t UART0Count = 0, UART1Count = 0, UART3Count = 0;

/************************* Set and Conigure Uart Pins ******************************************************************/
const uartChannelConfig_st UartConfig[C_MaxUartChannels_U8]=
{  /* TxPin RxPin UART_PinFun   PCON Bit Associated UART Structure    */
    { P0_2, P0_3, PINSEL_FUNC_1,  3     ,(LPC_UART_TypeDef *)LPC_UART0_BASE}, /* Configure P0_2,P0_3 for UART0 function */
    { P2_0, P2_1, PINSEL_FUNC_2,  4     ,(LPC_UART_TypeDef *)LPC_UART1_BASE}, /* Configure P2_0,P2_1 for UART1 function */
    { P2_8,P2_9,PINSEL_FUNC_2,  24    ,(LPC_UART_TypeDef *)LPC_UART2_BASE}, /* Configure P0_10,P0_11 for UART2 function */
    { P0_25, P0_26, PINSEL_FUNC_3,  25    ,(LPC_UART_TypeDef *)LPC_UART3_BASE}  /* Configure P0_0,P0_1 for UART3 function */ 
};

/******************* Tables for looking up fractional baud rate values. *************************************************/
float FRList[BR_LOOKUP_SIZE] = {1.000,1.067,1.071,1.077,1.083,1.091,1.100,1.111,1.125,1.133,1.143,1.154,1.167,1.182,1.200,1.214,1.222,1.231,1.250,
1.267,1.273,1.286,1.300,1.308,1.333,1.357,1.364,1.375,1.385,1.400,1.417,1.429,1.444,1.455,1.462,1.467,1.500,1.533,1.538,1.545,1.556,
1.571,1.583,1.600,1.615,1.625,1.636,1.643,1.667,1.692,1.700,1.714,1.727,1.733,1.750,1.769,1.778,1.786,1.800,1.818,1.833,1.846,1.857,
1.867,1.875,1.889,1.900,1.909,1.917,1.923,1.929,1.933};
float DIVADDVALList[BR_LOOKUP_SIZE] = {0.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,2.0,1.0,2.0,1.0,2.0,1.0,3.0,2.0,3.0,1.0,4.0,3.0,2.0,3.0,4.0,1.0,5.0,4.0,3.0,
5.0,2.0,5.0,3.0,4.0,5.0,6.0,7.0,1.0,8.0,7.0,6.0,5.0,4.0,7.0,3.0,8.0,5.0,7.0,9.0,2.0,9.0,7.0,5.0,8.0,11.0,3.0,10.0,7.0,11.0,4.0,9.0,5.0,
11.0,6.0,13.0,7.0,8.0,9.0,10.0,11.0,12.0,13.0,14.0};
float MULVALList[BR_LOOKUP_SIZE] = {1.0,15.0,14.0,13.0,12.0,11.0,10.0,9.0,8.0,15.0,7.0,13.0,6.0,11.0,5.0,14.0,9.0,13.0,4.0,15.0,11.0,7.0,10.0,13.0,3.0,
14.0,11.0,8.0,13.0,5.0,12.0,7.0,9.0,11.0,13.0,15.0,2.0,15.0,13.0,11.0,9.0,7.0,12.0,5.0,13.0,8.0,11.0,14.0,3.0,13.0,10.0,7.0,11.0,15.0,
4.0,13.0,9.0,14.0,5.0,11.0,6.0,13.0,7.0,15.0,8.0,9.0,10.0,11.0,12.0,13.0,14.0,15.0};

/*************  Return 1 if the double is an int value, 0 if not *****************/
static int isIntValue(double value) {
	int intValue = (int)value;
	if( value == intValue ) {
		return 1;
	}
	return 0;
}

/************* Get the fraction values for the given FRest value. ****************/
static int getFRValues(double FRest, float *divAddVal, float *mulVal) {
	float lastDiff = -1;
	float thisDiff;
	int index;
	//Look through the lookup table and find the index of the value
	//that provides the smallest difference between the FRest value
	//and the lookup table value.
	for( index=0 ; index<BR_LOOKUP_SIZE ; index++ ) {
		if( FRest > FRList[index] ) {
			thisDiff = FRest-FRList[index];
		}
		else {
			thisDiff = FRList[index]-FRest;
		}
        if( lastDiff != -1 && thisDiff > lastDiff ) {
          //Set the fractional values required
          *divAddVal=DIVADDVALList[index-1];
          *mulVal=MULVALList[index-1];
          return 0;
        }
        lastDiff=thisDiff;
	}
	return -1;
}

/****** Get the fraction values required to set an accurate BR ; Return -1 on error, 0 on success. ****/
static int getFractionValues(int pclk, int baudRate, int *dlEst, float *divAddVal, float *mulVal) {
	double  dlEstFloat = pclk/(16.0*baudRate);
	double 	FRestSeed = 1.5;
	double  FRest;
	int 	DLest;

	//If this pclk and baud rate give and integer division
	//we don't need the fractional calculation
    if( isIntValue(dlEstFloat) ) {
    	*dlEst = (int)dlEstFloat;
    	*divAddVal=0.0;
    	*mulVal=1.0;
    	return 0;
    }

	while(1) {
		DLest = (int)(pclk/(16.0*baudRate*FRestSeed));
		FRest = pclk/(16.0*baudRate*DLest);
		//If we have the required accuracy
		if( FRest >= 1.1 && FRest < 1.9) {
			break;
		}

		if( FRestSeed <= 1.5 ) {
			FRestSeed-=0.001;
			if( FRestSeed < 1.1 ) {
				FRestSeed=1.5001;
			}
		}
		else {
			FRestSeed=FRestSeed+0.001;
        	if( FRestSeed >= 1.9 ) {
        		return -1;
        	}
		}
	}
	*dlEst=(int)DLest;
	return getFRValues(FRest, divAddVal, mulVal);
}

/***************************************************************************************************
                    void UART_Init(uint8_t v_uartChannel_u8, uint32_t v_baudRate_u32)
 ****************************************************************************************************
 * I/P Arguments: uint32_t : Baudrate to be configured.
 * Return value    : none

 * description  :This function is used to initialize the UART at specified baud rate.
                 If the requested baud rate is not within the supported range then
                 the default baud rate of 9600 is set.


            Refer uart.h file for Supported(range) baud rates.        
 ***************************************************************************************************/
void UART_Init(uint8_t v_uartChannel_u8, uint32_t v_baudRate_u32)
{    
    if(v_uartChannel_u8< C_MaxUartChannels_U8)
    {	 
        GPIO_PinFunction(UartConfig[v_uartChannel_u8].TxPin,UartConfig[v_uartChannel_u8].PinFunSel);
        GPIO_PinFunction(UartConfig[v_uartChannel_u8].RxPin,UartConfig[v_uartChannel_u8].PinFunSel);
		util_BitSet(LPC_SC->PCONP,UartConfig[v_uartChannel_u8].pconBit);
        
        /* Enable FIFO and reset Rx/Tx FIFO buffers */
        UartConfig[v_uartChannel_u8].UARTx->FCR = (1<<SBIT_FIFO) | (1<<SBIT_RxFIFO) | (1<<SBIT_TxFIFO); 

        /* 8bit data, 1Stop bit, No parity */
        UartConfig[v_uartChannel_u8].UARTx->LCR = (0x03<<SBIT_WordLenght) | (1<<SBIT_DLAB);

        UART_SetBaudRate(v_uartChannel_u8,v_baudRate_u32);    
    }
}

/***************************************************************************************************
                 void UART_SetBaudRate(uint8_t v_uartChannel_u8, uint32_t v_baudRate_u32)
 ***************************************************************************************************
 * I/P Arguments: uint32_t : v_baudRate_u32 to be configured.
 * Return value    : none

 * description  :This function is used to Set/Change the baudrate on the fly.
                 If the requested baud rate is not within the supported range then
                 the default baudrate of 9600 is set.

            Refer uart.h file for Supported range of baud rates.
 ***************************************************************************************************/
void UART_SetBaudRate(uint8_t v_uartChannel_u8, uint32_t v_baudRate_u32)
{
    uint32_t v_UartPclk_u32,v_Pclk_u32; //v_RegValue_u32;
		int 	dlEest;
	  float 	divAddVal, mulVal;
    
    if(v_uartChannel_u8 < C_MaxUartChannels_U8 )
    {  
        /** Baud Rate Calculation :
       PCLKSELx registers contains the PCLK info for all the clock dependent peripherals.
       Bit6,Bit7 contains the Uart Clock(ie.UART_PCLK) information.
       The UART_PCLK and the actual Peripheral Clock(PCLK) is calculated as below.
       (Refer data sheet for more info)

       UART_PCLK    PCLK
         0x00       SystemFreq/4        
         0x01       SystemFreq
         0x02       SystemFreq/2
         0x03       SystemFreq/8   
         **/

        v_UartPclk_u32 = (LPC_SC->PCLKSEL0 >> 6) & 0x03;

        switch( v_UartPclk_u32 )
        {
        case 0x00:
            v_Pclk_u32 = SystemCoreClock/4;
            break;
        case 0x01:
            v_Pclk_u32 = SystemCoreClock;
            break; 
        case 0x02:
            v_Pclk_u32 = SystemCoreClock/2;
            break; 
        case 0x03:
            v_Pclk_u32 = SystemCoreClock/8;
            break;
        }    

//			v_RegValue_u32 = ( v_Pclk_u32 / (16 * v_baudRate_u32 )); 

//			UartConfig[v_uartChannel_u8].UARTx->DLL = util_ExtractByte0to8(v_RegValue_u32);
//      UartConfig[v_uartChannel_u8].UARTx->DLM = util_ExtractByte8to16(v_RegValue_u32);
				
				// Calculations for 115200
				getFractionValues(v_Pclk_u32, v_baudRate_u32, &dlEest, &divAddVal, &mulVal) ;
				
				UartConfig[v_uartChannel_u8].UARTx->DLL =dlEest % 256;
        UartConfig[v_uartChannel_u8].UARTx->DLM =dlEest / 256;
				UartConfig[v_uartChannel_u8].UARTx->FDR = (((int)mulVal)<<4)|(int)divAddVal;
				
         
				util_BitClear(UartConfig[v_uartChannel_u8].UARTx->LCR, SBIT_DLAB); // Clear DLAB after setting DLL,DLM
    } 
}

/***************************************************************************************************
                    char UART_RxChar(uint8_t v_uartChannel_u8)
 ***************************************************************************************************
 * I/P Arguments: none.
 * Return value    : char: Ascii value of the character received

 * description :This function is used to receive a char from UART module.
                It waits till a char is received and returns it after reception.
 ***************************************************************************************************/
char UART_RxChar(uint8_t v_uartChannel_u8)
{								
    char ch = 0; 
    
    if(v_uartChannel_u8 < C_MaxUartChannels_U8 )
    {
          /* Wait till the data is received */
         while(util_IsBitCleared(UartConfig[v_uartChannel_u8].UARTx->LSR,SBIT_RDR)); 
         ch = UartConfig[v_uartChannel_u8].UARTx->RBR; // Copy the received data
    }
    return ch;    
}

/************************ UART Rx Char3 ********************************/
char UART_RxChar3(uint8_t v_uartChannel_u8)
{
	char ch = 0;
	uint32_t retry=0; 
    
	if(v_uartChannel_u8 < C_MaxUartChannels_U8 )
	{
		while(retry<6900000)
		{
			/* Wait till the data is received */
			while(!(util_IsBitCleared(UartConfig[v_uartChannel_u8].UARTx->LSR,SBIT_RDR)) && retry<6900000)	
			{ 
				ch = UartConfig[v_uartChannel_u8].UARTx->RBR; // Copy the received data
				return ch;
			}
			retry++;
		}
	}
	ch = '$';
    return ch; 								
}

/***************************************************************************************************
                         void UART_TxChar(char v_uartData_u8)
 ****************************************************************************************************
 * I/P Arguments: char--> Ascii value of the character to be transmitted.
 * Return value    : none.

 * description  :This function is used to transmit a char through UART module.
 ***************************************************************************************************/
void UART_TxChar(uint8_t v_uartChannel_u8, char v_uartData_u8)
{
    if(v_uartChannel_u8 < C_MaxUartChannels_U8)
    {
       while(util_IsBitCleared(UartConfig[v_uartChannel_u8].UARTx->LSR,SBIT_THRE)); // Wait for Previous transmission
       UartConfig[v_uartChannel_u8].UARTx->THR=v_uartData_u8;   // Load the data to be transmitted   
    }
}

/***************************************************************************************************
                         void UART_TxString(char *ptr_stringPointer_u8)
 ****************************************************************************************************
 * I/P Arguments: String(Address of the string) to be transmitted.
 * Return value    : none

 * description :This function is used to transmit a NULL terminated string through UART.
               1.The ptr_stringPointer_u8 points to the first char of the string
                    and traverses till the end(NULL CHAR) and transmits a char each time
 ***************************************************************************************************/
#if ((Enable_UART_TxString==1)|| (Enable_UART_Printf == 1))
void UART_TxString(uint8_t v_uartChannel_u8, char *ptr_stringPointer_u8)
{
    while(*ptr_stringPointer_u8)
        UART_TxChar(v_uartChannel_u8, *ptr_stringPointer_u8++);// Loop through the string and transmit char by char
}
#endif

/***************************************************************************************************
                         uint8_t UART_RxString(uint8_t v_uartChannel_u8, char *ptr_string)
 ****************************************************************************************************
 * I/P Arguments: char *:  Address of the string/buffer where the received data needs to be stored
 * Return value	: uint8_t: Number of chars received.

 * description  :
              1.This function is used to receive a ASCII string through UART till the carriage_return/New_line
              2.The stream of data is copied to the buffer till carriage_return/New_line is encountered.
			  3. Once the Carriage_return/New_Line is received the string will be null terminated.

 *****NOTE*******:
  1.The received char is ECHOED back,
    if not required then comment UART_TxChar(ch) in the code.
  2.BackSlash is not taken care.
 ***************************************************************************************************/
#if (Enable_UART_RxString==1)
uint8_t UART_RxString(uint8_t v_uartChannel_u8, char *ptr_string)
{
    char ch;
    uint8_t len = 0;
    while(1)
    {
        ch=UART_RxChar(v_uartChannel_u8);    //Receive a char
        UART_TxChar(v_uartChannel_u8,ch);     //Echo back the received char

        if((ch=='\r') || (ch=='\n')) //read till enter key is pressed
        {   
		  if(len!=0)         /* Wait till atleast 1 char is received */              
           { 
		       ptr_string[len]=0;           //once enter key is pressed null terminate the string and break the loop 			
               break;  
			}                   
        }
        else if((ch=='\b') && (len!=0))
        {
            len--;    //If backspace is pressed then decrement the index to remove the old char
        }
        else
        {
            ptr_string[len]=ch; //copy the char into string and increment the index
            len++;
        }
    }
  return len;   
}
#endif

/***************************************************************************************************
void UART_TxNumber(uint8_t v_numericSystem_u8, uint32_t v_number_u32, uint8_t v_numOfDigitsToTransmit_u8)
 ***************************************************************************************************
 * I/P Arguments: 
                  uint8_t :  specifies type of number C_BINARY_U8(2),C_DECIMAL_U8(10), C_HEX_U8(16)
                  uint32_t: Number to be transmitted on UART.
                  uint8_t : Number of digits to be transmitted

 * Return value    : none

 * description  :This function is used to transmit a max of 10digit decimal number.
                2nd parameter specifies the number of digits from the right side to be transmitted
                 The output for the input combinations is as below

    Binary:     1.(10,4) then 4-LSB will be transmitted ie. 1010
                2.(10,8) then 8-LSB will be transmitted ie. 00001010
                3.(10,2) then 2-LSB will be transmitted ie. 10     

    Decimal            
                4.(12345,4) then 4-digits ie. 2345 will be transmitted
                5.(12345,6) then 6-digits ie. 012345 will be transmitted
                6.(12345,C_DefaultDigitsToTransmit_U8) then 12345 will be transmitted.

    Hex:
                7.(0x12AB,3) then 3-digits ie. 2AB will be transmitted
                8.(0x12AB,6) then 6-digits ie. 0012AB will be transmitted
                9.(0x12AB,C_DefaultDigitsToTransmit_U8) then 12AB will be transmitted.    
 ***************************************************************************************************/
#if ((Enable_UART_TxNumber==1) || (Enable_UART_TxFloatNumber==1) || (Enable_UART_Printf == 1))
void UART_TxNumber(uint8_t v_uartChannel_u8, uint8_t v_numericSystem_u8, uint32_t v_number_u32, uint8_t v_numOfDigitsToTransmit_u8)
{
    uint8_t i=0,a[10];

    if(C_BINARY_U8 == v_numericSystem_u8)
    {
        while(v_numOfDigitsToTransmit_u8!=0)
        {
            /* Start Extracting the bits from the specified bit positions.
             Get the Acsii values of the bits and transmit */
            i = util_GetBitStatus(v_number_u32,(v_numOfDigitsToTransmit_u8-1));
            UART_TxChar(v_uartChannel_u8,util_Dec2Ascii(i));
            v_numOfDigitsToTransmit_u8--;
        }    
    }     
    else if(v_number_u32==0)
    {
        /* If the number is zero then Transmit Specified number of zeros*/
        /*TODO: trsnamit single zero or multiple, Currently single zero is transmitted*/
       // for(i=0;((i<v_numOfDigitsToTransmit_u8) && (i<C_MaxDigitsToTransmit_U8)) ;i++)
             UART_TxChar(v_uartChannel_u8,'0');
    }
    else
    {
        for(i=0;i<v_numOfDigitsToTransmit_u8;i++)
        {
            /* Continue extracting the digits from right side
               till the Specified v_numOfDigitsToTransmit_u8 */
            if(v_number_u32!=0)
            {
                /* Extract the digits from the number till it becomes zero.
                First get the remainder and divide the number by 10 each time.

                example for Decimal number:
                If v_number_u32 = 123 then extracted remainder will be 3 and number will be 12.
                The process continues till it becomes zero or max digits reached*/
                a[i]=util_GetMod32(v_number_u32,v_numericSystem_u8);
                v_number_u32=v_number_u32/v_numericSystem_u8;
            }
            else if( (v_numOfDigitsToTransmit_u8 == C_DefaultDigitsToTransmit_U8) ||
                    (v_numOfDigitsToTransmit_u8 > C_MaxDigitsToTransmit_U8))
            {
                /* Stop the iteration if the Max number of digits are reached or
                 the user expects exact(Default) digits in the number to be transmitted */ 
                break;
            }
            else
            {
                /*In case user expects more digits to be transmitted than the actual digits in number,
                  then update the remaining digits with zero.
                Ex: v_number_u32 is 123 and user wants five digits then 00123 has to be transmitted */
                a[i]=0;
            }
        }

        while(i)
        { 
            /* Finally get the ascii values of the digits and transmit*/
            UART_TxChar(v_uartChannel_u8,util_Hex2Ascii(a[i-1]));
            i--;
        }
    }


}
#endif

/***************************************************************************************************
            void  UART_TxFloatNumber(float v_floatNumber_f32)
 ***************************************************************************************************
 * Function name:  UART_TxFloatNumber()
 * I/P Arguments: float: float Number to be transmitted on UART.

 * Return value    : none

 * description  :This function is used to transmit a floating point number

 * Note         :It supports 6digits of precision.  
         Float will be disabled by default as it takes huge controller resources
         It can be enabled by changing value of Enable_UART_TxFloatNumber to 1 in uart.h     
 ***************************************************************************************************/
#if (Enable_UART_TxFloatNumber==1)
void UART_TxFloatNumber(uint8_t v_uartChannel_u8, float v_floatNumber_f32)
{
    uint32_t v_tempNumber_u32;
    /* Dirty hack to support the floating point by extracting the integer and fractional part.
      1.Type cast the number to int to get the integer part.
      2.transmit the extracted integer part followed by a decimal point(.).
      3.Later the integer part is made zero by subtracting with the extracted integer value.
      4.Finally the fractional part is multiplied by 100000 to support 6-digit precision */

    v_tempNumber_u32 = (uint32_t) v_floatNumber_f32;
    UART_TxNumber(v_uartChannel_u8,C_DECIMAL_U8,v_tempNumber_u32,C_DefaultDigitsToTransmit_U8);

    UART_TxChar(v_uartChannel_u8,'.');

    v_floatNumber_f32 = v_floatNumber_f32 - v_tempNumber_u32;
    v_tempNumber_u32 = v_floatNumber_f32 * 100;
    UART_TxNumber(v_uartChannel_u8,C_DECIMAL_U8,v_tempNumber_u32,C_DefaultDigitsToTransmit_U8);
}
#endif

/***************************************************************************************************
            void UART_Printf(const char *argList, ...)
 ***************************************************************************************************
 * Function name:  UART_Printf()
 * I/P Arguments: variable length arguments similar to printf

 * Return value    : none

 * description  :This function is similar to printf function in C.
                 It takes the arguments with specified format and prints accordingly
                 The supported format specifiers are as below.
                 1. %c: character
                 2. %d: signed 16-bit number
                 3. %D: signed 32-bit number
                 4. %u: unsigned 16-bit number
                 5. %U: unsigned 32-bit number
                 6. %b: 16-bit binary number
                 7. %B: 32-bit binary number
                 8. %f: Float number
                 9. %x: 16-bit hexadecimal number
                 10. %X: 32-bit hexadecimal number
                 11. %s: String

  Note: By default all the functions will be disabled. The required functions can be enabled by 
        setting the respective compiler switch to 1 in uart.h file.
		Ex:  setting Enable_UART_TxDecimalNumber to 1 will enable %d
		     setting Enable_UART_TxHexNumber to 1 will enable %x

  Extra feature is available to specify the number of digits to be transmitted using printf.
	 ex: %4d: will transmit the lower four digits of the decimal number.
	     %12b: will transmit the 12-LSB of the number
		 %d: Will transmit the exact digits of the number
		 
#####: In case of printing the variables(8-bit) its recommended to type cast and promote them to uint16_t.
        uint8_t v_Num_u8;
		UART_Printf("num1:%u",(uint16_t)v_Num_u8); 		 
***************************************************************************************************/
#if ( Enable_UART_Printf   == 1 ) 
void UART_Printf(uint8_t v_uartChannel_u8, const char *argList, ...)
{
    const char *ptr;
    float v_floatNum_f32;
    va_list argp;
    sint16_t v_num_s16;
    sint32_t v_num_s32;
    uint16_t v_num_u16;
    uint32_t v_num_u32;
    char *str;
    char  ch;
    uint8_t v_numOfDigitsToTransmit_u8;

    va_start(argp, argList);

    /* Loop through the list to extract all the input arguments */
    for(ptr = argList; *ptr != '\0'; ptr++)
    {

        ch= *ptr;
        if(ch == '%')         /*Check for '%' as there will be format specifier after it */
        {
            ptr++;
            ch = *ptr;
            if((ch>=0x30) && (ch<=0x39))
            {
                v_numOfDigitsToTransmit_u8 = 0;
                while((ch>=0x30) && (ch<=0x39))
                {
                    v_numOfDigitsToTransmit_u8 = (v_numOfDigitsToTransmit_u8 * 10) + (ch-0x30);
                    ptr++;
                    ch = *ptr;
                }
            }
            else
            {
                v_numOfDigitsToTransmit_u8 = C_MaxDigitsToTransmitUsingPrintf_U8;
            }                


            switch(ch)       /* Decode the type of the argument */
            {

            case 'C':
            case 'c':     /* Argument type is of char, hence read char data from the argp */
                ch = va_arg(argp, int);
                UART_TxChar(v_uartChannel_u8,ch);
                break;



            case 'd':    /* Argument type is of signed integer, hence read 16bit data from the argp */
                v_num_s16 = va_arg(argp, int);
#if (Enable_UART_TxNumber == 1)
                if(v_num_s16<0)
                { /* If the number is -ve then display the 2's complement along with '-' sign */ 
                    v_num_s16 = -v_num_s16;
                    UART_TxChar(v_uartChannel_u8,'-');
                }
                UART_TxNumber(v_uartChannel_u8,C_DECIMAL_U8,v_num_s16,v_numOfDigitsToTransmit_u8);
#endif
                break;


			   
            case 'D':    /* Argument type is of integer, hence read 16bit data from the argp */
                v_num_s32 = va_arg(argp, sint32_t);
#if (Enable_UART_TxNumber == 1)                
                if(v_num_s32<0)
                { /* If the number is -ve then display the 2's complement along with '-' sign */
                    v_num_s32 = -v_num_s32;
                    UART_TxChar(v_uartChannel_u8,'-');
                }
                UART_TxNumber(v_uartChannel_u8,C_DECIMAL_U8,v_num_s32,v_numOfDigitsToTransmit_u8);
#endif                
                break;    



            case 'u':    /* Argument type is of unsigned integer, hence read 16bit unsigned data */
                v_num_u16 = va_arg(argp, int);
#if (Enable_UART_TxNumber == 1)                
                UART_TxNumber(v_uartChannel_u8,C_DECIMAL_U8,v_num_u16,v_numOfDigitsToTransmit_u8);
#endif                
                break;



            case 'U':    /* Argument type is of integer, hence read 32bit unsigend data */
                v_num_u32 = va_arg(argp, uint32_t);
#if (Enable_UART_TxNumber == 1)                
                UART_TxNumber(v_uartChannel_u8,C_DECIMAL_U8,v_num_u32,v_numOfDigitsToTransmit_u8);
#endif                
                break;            


            case 'x':  /* Argument type is of hex, hence hexadecimal data from the argp */
                v_num_u16 = va_arg(argp, int);
#if (Enable_UART_TxNumber == 1)                
                UART_TxNumber(v_uartChannel_u8,C_HEX_U8, v_num_u16,v_numOfDigitsToTransmit_u8);
#endif                
                break;



            case 'X':  /* Argument type is of hex, hence hexadecimal data from the argp */
                v_num_u32 = va_arg(argp, uint32_t);
#if (Enable_UART_TxNumber == 1)                        
                UART_TxNumber(v_uartChannel_u8,C_HEX_U8, v_num_u32,v_numOfDigitsToTransmit_u8);
#endif                
                break;



            case 'b':  /* Argument type is of binary,Read int and convert to binary */
                v_num_u16 = va_arg(argp, int);
#if (Enable_UART_TxNumber == 1)                        
                if(v_numOfDigitsToTransmit_u8 == C_MaxDigitsToTransmitUsingPrintf_U8)
                {
                    v_numOfDigitsToTransmit_u8 = 16;
                }
                UART_TxNumber(v_uartChannel_u8,C_BINARY_U8, v_num_u16,v_numOfDigitsToTransmit_u8);
#endif                
                break;



            case 'B':  /* Argument type is of binary,Read int and convert to binary */
                v_num_u32 = va_arg(argp, uint32_t);
#if (Enable_UART_TxNumber == 1)                
                if(v_numOfDigitsToTransmit_u8 == C_MaxDigitsToTransmitUsingPrintf_U8)
                    v_numOfDigitsToTransmit_u8 = 16;                
                UART_TxNumber(v_uartChannel_u8,C_BINARY_U8, v_num_u32,v_numOfDigitsToTransmit_u8);    
#endif                
                break;

            case 'F':
            case 'f': /* Argument type is of float, hence read double data from the argp */
                v_floatNum_f32 = va_arg(argp,double);
#if (Enable_UART_TxFloatNumber == 1)   
                if(v_floatNum_f32<0)
                { /* If the number is -ve then display the 2's complement along with '-' sign */
                    v_floatNum_f32 = -v_floatNum_f32;
                    UART_TxChar(v_uartChannel_u8,'-');
                }            
                UART_TxFloatNumber(v_uartChannel_u8,v_floatNum_f32);
#endif

                break;



            case 'S':
            case 's': /* Argument type is of string, hence get the pointer to sting passed */
                str = va_arg(argp, char *);
                UART_TxString(v_uartChannel_u8,str);                
                break;



            case '%':
                UART_TxChar(v_uartChannel_u8,'%');
                break;
            }
        }
        else
        {
            /* As '%' is not detected transmit the char passed */
            UART_TxChar(v_uartChannel_u8,ch);
        }
    }

    va_end(argp);
}
#endif

/*****************************************************************************
** Function name:		UART1_IRQHandler
**
** Descriptions:		UART1 interrupt handler
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void UART1_IRQHandler (void) 
{
  uint8_t IIRValue, LSRValue;
  uint8_t Dummy = Dummy;
	
  
  IIRValue = LPC_UART1->IIR;
    
  IIRValue >>= 1;			/* skip pending bit in IIR */
  IIRValue &= 0x07;			/* check bit 1~3, interrupt identification */
  if ( IIRValue == IIR_RLS )		/* Receive Line Status */
  {
	LSRValue = LPC_UART1->LSR;
	/* Receive Line Status */
	if ( LSRValue & (LSR_OE|LSR_PE|LSR_FE|LSR_RXFE|LSR_BI) )
	{
	  /* There are errors or break interrupt */
	  /* Read LSR will clear the interrupt */
	  UART1Status = LSRValue;
	  Dummy = LPC_UART1->RBR;		/* Dummy read on RX to clear 
								interrupt, then bail out */
	  return;
	}
	if ( LSRValue & LSR_RDR )	/* Receive Data Ready */			
	{
	  /* If no error on RLS, normal ready, save into the data buffer. */
	  /* Note: read RBR will clear the interrupt */
	  
	  UART1Buffer[UART1Count] = LPC_UART1->RBR;
	  UART1Count++;
	  if ( UART1Count == BUFSIZE )
	  {
		UART1Count = 0;		/* buffer overflow */
	  }	
	}
  }
  else if ( IIRValue == IIR_RDA )	/* Receive Data Available */
  {
	/* Receive Data Available */
	
	UART1Buffer[UART1Count] = LPC_UART1->RBR;
	UART1Count++;
	if ( UART1Count == BUFSIZE )
	{
	  UART1Count = 0;		/* buffer overflow */
	}
  }
  else if ( IIRValue == IIR_CTI )	/* Character timeout indicator */
  {
	/* Character Time-out indicator */
	UART1Status |= 0x100;		/* Bit 9 as the CTI error */
  }
  else if ( IIRValue == IIR_THRE )	/* THRE, transmit holding register empty */
  {
	/* THRE interrupt */
	LSRValue = LPC_UART1->LSR;		/* Check status in the LSR to see if
								valid data in U0THR or not */
	if ( LSRValue & LSR_THRE )
	{
	  UART1TxEmpty = 1;
	}
	else
	{
	  UART1TxEmpty = 0;
	}
  }

}

/*****************************************************************************
** Function name:		UART0_IRQHandler
**
** Descriptions:		UART0 interrupt handler
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void UART0_IRQHandler (void) 
{
  uint8_t IIRValue, LSRValue;
  uint8_t Dummy = Dummy;
	
  IIRValue = LPC_UART0->IIR;
    
  IIRValue >>= 1;			/* skip pending bit in IIR */
  IIRValue &= 0x07;			/* check bit 1~3, interrupt identification */
  if ( IIRValue == IIR_RLS )		/* Receive Line Status */
  {
	LSRValue = LPC_UART0->LSR;
	/* Receive Line Status */
	if ( LSRValue & (LSR_OE|LSR_PE|LSR_FE|LSR_RXFE|LSR_BI) )
	{
	  /* There are errors or break interrupt */
	  /* Read LSR will clear the interrupt */
	  UART0Status = LSRValue;
	  Dummy = LPC_UART0->RBR;		/* Dummy read on RX to clear 
							interrupt, then bail out */
	  return;
	}
	if ( LSRValue & LSR_RDR )	/* Receive Data Ready */			
	{
	  /* If no error on RLS, normal ready, save into the data buffer. */
	  /* Note: read RBR will clear the interrupt */
	
	  UART0Buffer[UART0Count] = LPC_UART0->RBR;
	  UART0Count++;
	  if ( UART0Count == BUFSIZE )
	  {
		UART0Count = 0;		/* buffer overflow */
	  }	
	}
  }
  else if ( IIRValue == IIR_RDA )	/* Receive Data Available */
  {
	/* Receive Data Available */

	UART0Buffer[UART0Count] = LPC_UART0->RBR;
	UART0Count++;
	if ( UART0Count == BUFSIZE )
	{
	  UART0Count = 0;		/* buffer overflow */
	}
  }
  else if ( IIRValue == IIR_CTI )	/* Character timeout indicator */
  {
	/* Character Time-out indicator */
	UART0Status |= 0x100;		/* Bit 9 as the CTI error */
  }
  else if ( IIRValue == IIR_THRE )	/* THRE, transmit holding register empty */
  {
	/* THRE interrupt */
	LSRValue = LPC_UART0->LSR;		/* Check status in the LSR to see if
									valid data in U0THR or not */
	if ( LSRValue & LSR_THRE )
	{
	  UART0TxEmpty = 1;
	}
	else
	{
	  UART0TxEmpty = 0;
	}
  }    
}

/*****************************************************************************
** Function name:		UART3_IRQHandler
**
** Descriptions:		UART3 interrupt handler
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void UART3_IRQHandler (void) 
{
  uint8_t IIRValue, LSRValue;
  uint8_t Dummy = Dummy;
	
  IIRValue = LPC_UART3->IIR;
    
  IIRValue >>= 1;			/* skip pending bit in IIR */
  IIRValue &= 0x07;			/* check bit 1~3, interrupt identification */
  if ( IIRValue == IIR_RLS )		/* Receive Line Status */
  {
	LSRValue = LPC_UART3->LSR;
	/* Receive Line Status */
	if ( LSRValue & (LSR_OE|LSR_PE|LSR_FE|LSR_RXFE|LSR_BI) )
	{
	  /* There are errors or break interrupt */
	  /* Read LSR will clear the interrupt */
	  UART3Status = LSRValue;
	  Dummy = LPC_UART3->RBR;		/* Dummy read on RX to clear 
							interrupt, then bail out */
	  return;
	}
	if ( LSRValue & LSR_RDR )	/* Receive Data Ready */			
	{
	  /* If no error on RLS, normal ready, save into the data buffer. */
	  /* Note: read RBR will clear the interrupt */
	
	  UART3Buffer[UART3Count] = LPC_UART3->RBR;
	  UART3Count++;
	  if ( UART3Count == BUFSIZE )
	  {
		UART3Count = 0;		/* buffer overflow */
	  }	
	}
  }
  else if ( IIRValue == IIR_RDA )	/* Receive Data Available */
  {
	/* Receive Data Available */

	UART3Buffer[UART3Count] = LPC_UART3->RBR;
	UART3Count++;
	if ( UART3Count == BUFSIZE )
	{
	  UART3Count = 0;		/* buffer overflow */
	}
  }
  else if ( IIRValue == IIR_CTI )	/* Character timeout indicator */
  {
	/* Character Time-out indicator */
	UART3Status |= 0x100;		/* Bit 9 as the CTI error */
  }
  else if ( IIRValue == IIR_THRE )	/* THRE, transmit holding register empty */
  {
	/* THRE interrupt */
	LSRValue = LPC_UART3->LSR;		/* Check status in the LSR to see if
									valid data in U0THR or not */
	if ( LSRValue & LSR_THRE )
	{
	  UART3TxEmpty = 1;
	}
	else
	{
	  UART3TxEmpty = 0;
	}
  }    
}


/*****************************************************************************
** Function name:		UARTSend
**
** Descriptions:		Send a block of data to the UART 0 port based
**						on the data length
**
** parameters:			portNum, buffer pointer, and data length
** Returned value:		None
** 
*****************************************************************************/
//void UARTSend( uint32_t portNum, uint8_t *BufferPtr, uint32_t Length )
//{
//  if ( portNum == 0 )
//  {
//    while ( Length != 0 )
//    {
//	  /* THRE status, contain valid data */
//	  while ( !(UART0TxEmpty & 0x01) );	
//	  LPC_UART0->THR = *BufferPtr;
//	  UART0TxEmpty = 0;	/* not empty in the THR until it shifts out */
//	  BufferPtr++;
//	  Length--;
//	}
//  }
//  else
//  {
//	while ( Length != 0 )
//    {
//	  /* THRE status, contain valid data */
//	  while ( !(UART1TxEmpty & 0x01) );	
//	  LPC_UART1->THR = *BufferPtr;
//	  UART1TxEmpty = 0;	/* not empty in the THR until it shifts out */
//	  BufferPtr++;
//	  Length--;
//    }
//  }
// 
//}

/************** Uart 3 Enable & Disable buffer ***************************/
void UART3_EnableBuffer(void)
{
   LPC_UART3->FCR = (1<<SBIT_FIFO) | (1<<SBIT_RxFIFO) | (1<<SBIT_TxFIFO); // Enable FIFO and reset Rx/Tx FIFO buffers 
}

void UART3_DisableBuffer(void)
{
	LPC_UART3->FCR = (0<<SBIT_FIFO) | (0<<SBIT_RxFIFO) | (0<<SBIT_TxFIFO); // Disable FIFO and reset Rx/Tx FIFO buffers
}

/************** Uart 1 Enable & Disable buffer ***************************/
void UART1_EnableBuffer_Interrupt(void)
{
   LPC_UART1->FCR = (1<<SBIT_FIFO) | (1<<SBIT_RxFIFO) | (1<<SBIT_TxFIFO); /* Enable FIFO and reset Rx/Tx FIFO buffers */
   LPC_UART1->IER = IER_RBR | IER_THRE | IER_RLS;	/* Enable UART1 interrupt */ 
}

void UART1_DisableBuffer_Interrupt(void)
{
	LPC_UART1->FCR = (0<<SBIT_FIFO) | (0<<SBIT_RxFIFO) | (0<<SBIT_TxFIFO); /* Disable FIFO and reset Rx/Tx FIFO buffers */
	LPC_UART1->IER = IER_THRE | IER_RLS;			/* Disable RBR UART 1 interrupt*/
}

/************** Uart 0 Enable & Disable buffer ***************************/
void UART0_EnableBuffer_Interrupt(void)
{
   LPC_UART0->FCR = (1<<SBIT_FIFO) | (1<<SBIT_RxFIFO) | (1<<SBIT_TxFIFO); /* Enable FIFO and reset Rx/Tx FIFO buffers */
   LPC_UART0->IER = IER_RBR | IER_THRE | IER_RLS;	/* Enable UART0 interrupt */ 
}

void UART0_DisableBuffer_Interrupt(void)
{
	LPC_UART0->FCR = (0<<SBIT_FIFO) | (0<<SBIT_RxFIFO) | (0<<SBIT_TxFIFO); /* Disable FIFO and reset Rx/Tx FIFO buffers */
	LPC_UART0->IER = IER_THRE | IER_RLS;			/* Disable RBR UART 1 interrupt*/
}

