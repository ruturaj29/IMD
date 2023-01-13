
/***************************** include Header files ************************************************/
#include <lpc17xx.h>
#include "stdutils.h"
#include "gpio.h"

/***************************************************************************************************
                void GPIO_PinFunction(uint8_t v_pinNumber_u8, uint8_t v_pinFunction_u8)
 ****************************************************************************************************
 * I/P Arguments: 
                uint8_t: pin number for which the pin function needs to be selected.
                             .
                v_pinFunction_u8: Any of the below four pins functions.                                  
                                 PINSEL_FUNC_0
                                 PINSEL_FUNC_1
                                 PINSEL_FUNC_2
                                 PINSEL_FUNC_3    
                                 Refer lpc1768 data sheet for more info. 


 * Return value    : none

 * description :This function sets the specified PinFunction for the selected pin.
                Refer the lpc1768 data sheet for complete pin functions.  
 ****************************************************************************************************/
void GPIO_PinFunction(uint8_t v_pinNumber_u8, uint8_t v_pinFunction_u8)
{
    uint8_t v_portNumber_u8;
    uint32_t *ptr_PINCON;

    if((v_pinNumber_u8 < P_MAX) && (v_pinFunction_u8 <= PINSEL_FUNC_3)) //Handle the request only if it is in range
    {
        v_portNumber_u8 =  (v_pinNumber_u8>>5);  //Divide the pin number by 32 go get the PORT number
        v_pinNumber_u8  =   v_pinNumber_u8 & 0x1f;  //lower 5-bits contains the bit number of a 32bit port  


        v_portNumber_u8 = v_portNumber_u8*2;
        if(v_pinNumber_u8>=16)
        {
            v_portNumber_u8++; 
            v_pinNumber_u8 = v_pinNumber_u8-16;
        }

        v_pinNumber_u8 = v_pinNumber_u8 * 2;

        ptr_PINCON    = ((uint32_t *)&LPC_PINCON->PINSEL0  + v_portNumber_u8);

        *(uint32_t *)(ptr_PINCON) &= ~(0x03UL << v_pinNumber_u8);
        *(uint32_t *)(ptr_PINCON) |= (((uint32_t)v_pinFunction_u8) << v_pinNumber_u8);
    }
}

/***************************************************************************************************
                  void GPIO_PinDirection(uint8_t v_pinNumber_u8, gpioDirection_et enm_pinDirn)
 ****************************************************************************************************
 * I/P Arguments: 
                uint8_t: pin number which needs to be updated.
                             Refer the gpio.h for pin numbering.
                gpioDirection_et: Direction to be configured ie. INPUT/OUTPUT.


 * Return value    : none

 * description :This function sets the specified direction as INPUT/OUTPUT.  
 ****************************************************************************************************/
void GPIO_PinDirection(uint8_t v_pinNumber_u8, uint8_t v_pinDirn_u8)
{

    uint8_t v_portNumber_u8;
    LPC_GPIO_TypeDef *LPC_GPIO_PORT;

    if(v_pinNumber_u8 < P_MAX)               //Handle the request only if it is in range
    {
        v_portNumber_u8 =  (v_pinNumber_u8>>5);  //Divide the pin number by 32 go get the PORT number
        v_pinNumber_u8  =   v_pinNumber_u8 & 0x1f;  //lower 5-bits contains the bit number of a 32bit port  

        /* Go to particular port after decoding from the pin number and 
        set the direction as specified*/

        LPC_GPIO_PORT = (LPC_GPIO_TypeDef*)(LPC_GPIO_BASE + ((v_portNumber_u8) << 5));
        util_UpdateBit(LPC_GPIO_PORT->FIODIR,v_pinNumber_u8,v_pinDirn_u8);   
    }
}

/***************************************************************************************************
            void GPIO_PinWrite(uint8_t v_pinNumber_u8, uint8_t v_pinValue_u8)
 ****************************************************************************************************
 * I/P Arguments: 
                uint8_t: pin number which needs to be updated.
                             Refer the gpio.h for pin numbering.
                uint8_t: Value to be updated(LOW/HIGH) on the selected pin.


 * Return value    : none

 * description :This function updates the specified value on the selected pin.  
                Before updating the pins status, the pin function should be selected and then
                the pin should be configured as OUTPUT 
 ****************************************************************************************************/
void GPIO_PinWrite(uint8_t v_pinNumber_u8, uint8_t v_pinValue_u8)
{

    uint8_t v_portNumber_u8;
    LPC_GPIO_TypeDef *LPC_GPIO_PORT;
    if(v_pinNumber_u8 < P_MAX)               //Handle the request only if it is in range
    {
        v_portNumber_u8 =  (v_pinNumber_u8>>5);  //Divide the pin number by 32 go get the PORT number
        v_pinNumber_u8  =   v_pinNumber_u8 & 0x1f;  //lower 5-bits contains the bit number of a 32bit port  

        /* Go to particular port after decoding from the pin number and 
        update the value of the specified pin*/

        LPC_GPIO_PORT = (LPC_GPIO_TypeDef*)(LPC_GPIO_BASE + ((v_portNumber_u8) << 5));

        if(v_pinValue_u8!=0)
        {
            util_BitSet(LPC_GPIO_PORT->FIOSET,v_pinNumber_u8);        
        }
        else
        {
            util_BitSet(LPC_GPIO_PORT->FIOCLR,v_pinNumber_u8); 
        }
    }
}

/***************************************************************************************************
            void GPIO_PinToggle(uint8_t v_pinNumber_u8)
 ****************************************************************************************************
 * I/P Arguments: 
                uint8_t: pin number which needs to be toggled.
                             Refer the gpio.h for pin numbering.


 * Return value    : none

 * description :This function toggles the selected pin.  
                Before updating the pins status, the pin function should be selected and then
                the pin should be configured as OUTPUT 
 ****************************************************************************************************/
void GPIO_PinToggle(uint8_t v_pinNumber_u8)
{

    uint8_t v_portNumber_u8;
    LPC_GPIO_TypeDef *LPC_GPIO_PORT;

    if(v_pinNumber_u8 < P_MAX)               //Handle the request only if it is in range
    {
        v_portNumber_u8 =  (v_pinNumber_u8>>5);     //Divide the pin number by 32 go get the PORT number
        v_pinNumber_u8  =   v_pinNumber_u8 & 0x1f;  //lower 5-bits contains the bit number of a 32bit port  

        /* Go to particular port after decoding from the pin number and 
        update the value of the specified pin*/

        LPC_GPIO_PORT = (LPC_GPIO_TypeDef*)(LPC_GPIO_BASE + ((v_portNumber_u8) << 5));

        util_BitToggle(LPC_GPIO_PORT->FIOPIN,v_pinNumber_u8);      
    }          
}

/***************************************************************************************************
                    uint8_t GPIO_PinRead(uint8_t v_pinNumber_u8)
 ****************************************************************************************************
 * I/P Arguments: 
                uint8_t: pin number which needs to be read.
                             Refer the gpio.h for pin numbering.

 * Return value    : 
                 uint8_t:    Status of the specified pin.

 * description :This function returns the status of the selected pin.
                Before reading the pins status, the pin function should be selected and accordingly
                the pin should be configured as INPUT 
 ****************************************************************************************************/
uint8_t GPIO_PinRead(uint8_t v_pinNumber_u8)
{
    uint8_t returnStatus = 0;
    uint8_t v_portNumber_u8;
    LPC_GPIO_TypeDef *LPC_GPIO_PORT; 

    if(v_pinNumber_u8 < P_MAX)               //Handle the request only if it is in range
    {
        v_portNumber_u8 =  (v_pinNumber_u8>>5);  //Divide the pin number by 32 go get the PORT number
        v_pinNumber_u8  =   v_pinNumber_u8 & 0x1f;  //lower 5-bits contains the bit number of a 32bit port  

        /* Go to particular port after decoding from the pin number and read the pins status */

        LPC_GPIO_PORT = (LPC_GPIO_TypeDef*)(LPC_GPIO_BASE + ((v_portNumber_u8) << 5));    
        returnStatus = util_IsBitSet(LPC_GPIO_PORT->FIOPIN,v_pinNumber_u8);
    }

    return returnStatus;
}

/*********************************************************************//**
 * @brief 		Setup resistor mode for each pin
 * @param[in]	portnum PORT number,
 * 				should be one of the following:
 * 				- PINSEL_PORT_0	: Port 0
 * 				- PINSEL_PORT_1	: Port 1
 * 				- PINSEL_PORT_2	: Port 2
 * 				- PINSEL_PORT_3	: Port 3
 * @param[in]	pinnum	Pin number,
 * 				should be one of the following:
				- PINSEL_PIN_0 : Pin 0
				- PINSEL_PIN_1 : Pin 1
				- PINSEL_PIN_2 : Pin 2
				- PINSEL_PIN_3 : Pin 3
				- PINSEL_PIN_4 : Pin 4
				- PINSEL_PIN_5 : Pin 5
				- PINSEL_PIN_6 : Pin 6
				- PINSEL_PIN_7 : Pin 7
				- PINSEL_PIN_8 : Pin 8
				- PINSEL_PIN_9 : Pin 9
				- PINSEL_PIN_10 : Pin 10
				- PINSEL_PIN_11 : Pin 11
				- PINSEL_PIN_12 : Pin 12
				- PINSEL_PIN_13 : Pin 13
				- PINSEL_PIN_14 : Pin 14
				- PINSEL_PIN_15 : Pin 15
				- PINSEL_PIN_16 : Pin 16
				- PINSEL_PIN_17 : Pin 17
				- PINSEL_PIN_18 : Pin 18
				- PINSEL_PIN_19 : Pin 19
				- PINSEL_PIN_20 : Pin 20
				- PINSEL_PIN_21 : Pin 21
				- PINSEL_PIN_22 : Pin 22
				- PINSEL_PIN_23 : Pin 23
				- PINSEL_PIN_24 : Pin 24
				- PINSEL_PIN_25 : Pin 25
				- PINSEL_PIN_26 : Pin 26
				- PINSEL_PIN_27 : Pin 27
				- PINSEL_PIN_28 : Pin 28
				- PINSEL_PIN_29 : Pin 29
				- PINSEL_PIN_30 : Pin 30
				- PINSEL_PIN_31 : Pin 31

 * @param[in] 	modenum: Mode number,
 * 				should be one of the following:
				- PINSEL_PINMODE_PULLUP	: Internal pull-up resistor
				- PINSEL_PINMODE_TRISTATE : Tri-state
				- PINSEL_PINMODE_PULLDOWN : Internal pull-down resistor

 * @return 		None
 **********************************************************************/
void set_ResistorMode ( uint8_t portnum, uint8_t pinnum, uint8_t modenum)
{
	uint32_t pinnum_t = pinnum;
	uint32_t pinmodereg_idx = 2 * portnum;
	uint32_t *pPinCon = (uint32_t *)&LPC_PINCON->PINMODE0;

	if (pinnum_t >= 16) {
		pinnum_t -= 16;
		pinmodereg_idx++ ;
	}

	*(uint32_t *)(pPinCon + pinmodereg_idx) &= ~(0x03UL << (pinnum_t * 2));
	*(uint32_t *)(pPinCon + pinmodereg_idx) |= ((uint32_t)modenum) << (pinnum_t * 2);
}

/*********************************************************************//**
 * @brief 		Setup Open drain mode for each pin
 * @param[in]	portnum PORT number,
 * 				should be one of the following:
 * 				- PINSEL_PORT_0	: Port 0
 * 				- PINSEL_PORT_1	: Port 1
 * 				- PINSEL_PORT_2	: Port 2
 * 				- PINSEL_PORT_3	: Port 3
 *
 * @param[in]	pinnum	Pin number,
 * 				should be one of the following:
				- PINSEL_PIN_0 : Pin 0
				- PINSEL_PIN_1 : Pin 1
				- PINSEL_PIN_2 : Pin 2
				- PINSEL_PIN_3 : Pin 3
				- PINSEL_PIN_4 : Pin 4
				- PINSEL_PIN_5 : Pin 5
				- PINSEL_PIN_6 : Pin 6
				- PINSEL_PIN_7 : Pin 7
				- PINSEL_PIN_8 : Pin 8
				- PINSEL_PIN_9 : Pin 9
				- PINSEL_PIN_10 : Pin 10
				- PINSEL_PIN_11 : Pin 11
				- PINSEL_PIN_12 : Pin 12
				- PINSEL_PIN_13 : Pin 13
				- PINSEL_PIN_14 : Pin 14
				- PINSEL_PIN_15 : Pin 15
				- PINSEL_PIN_16 : Pin 16
				- PINSEL_PIN_17 : Pin 17
				- PINSEL_PIN_18 : Pin 18
				- PINSEL_PIN_19 : Pin 19
				- PINSEL_PIN_20 : Pin 20
				- PINSEL_PIN_21 : Pin 21
				- PINSEL_PIN_22 : Pin 22
				- PINSEL_PIN_23 : Pin 23
				- PINSEL_PIN_24 : Pin 24
				- PINSEL_PIN_25 : Pin 25
				- PINSEL_PIN_26 : Pin 26
				- PINSEL_PIN_27 : Pin 27
				- PINSEL_PIN_28 : Pin 28
				- PINSEL_PIN_29 : Pin 29
				- PINSEL_PIN_30 : Pin 30
				- PINSEL_PIN_31 : Pin 31

 * @param[in]	modenum  Open drain mode number,
 * 				should be one of the following:
 * 				- PINSEL_PINMODE_NORMAL : Pin is in the normal (not open drain) mode
 * 				- PINSEL_PINMODE_OPENDRAIN : Pin is in the open drain mode
 *
 * @return 		None
 **********************************************************************/
void set_OpenDrainMode( uint8_t portnum, uint8_t pinnum, uint8_t modenum)
{
	uint32_t *pPinCon = (uint32_t *)&LPC_PINCON->PINMODE_OD0;

	if (modenum == PINSEL_PINMODE_OPENDRAIN){
		*(uint32_t *)(pPinCon + portnum) |= (0x01UL << pinnum);
	} else {
		*(uint32_t *)(pPinCon + portnum) &= ~(0x01UL << pinnum);
	}
}

