
 
#ifndef _GPIO_H
#define _GPIO_H

#include"stdutils.h"


typedef enum
{
    P0_0, P0_1, P0_2, P0_3, P0_4, P0_5, P0_6, P0_7, P0_8, P0_9, P0_10,P0_11,P0_12,P0_13,P0_14,P0_15,
    P0_16,P0_17,P0_18,P0_19,P0_20,P0_21,P0_22,P0_23,P0_24,P0_25,P0_26,P0_27,P0_28,P0_29,P0_30,P0_31,

    P1_0, P1_1, P1_2, P1_3, P1_4, P1_5, P1_6, P1_7, P1_8, P1_9, P1_10,P1_11,P1_12,P1_13,P1_14,P1_15,
    P1_16,P1_17,P1_18,P1_19,P1_20,P1_21,P1_22,P1_23,P1_24,P1_25,P1_26,P1_27,P1_28,P1_29,P1_30,P1_31,

    P2_0, P2_1, P2_2, P2_3, P2_4, P2_5, P2_6, P2_7, P2_8, P2_9, P2_10,P2_11,P2_12,P2_13,P2_14,P2_15,
    P2_16,P2_17,P2_18,P2_19,P2_20,P2_21,P2_22,P2_23,P2_24,P2_25,P2_26,P2_27,P2_28,P2_29,P2_30,P2_31,

    P3_0, P3_1, P3_2, P3_3, P3_4, P3_5, P3_6, P3_7, P3_8, P3_9, P3_10,P3_11,P3_12,P3_13,P3_14,P3_15,
    P3_16,P3_17,P3_18,P3_19,P3_20,P3_21,P3_22,P3_23,P3_24,P3_25,P3_26,P3_27,P3_28,P3_29,P3_30,P3_31,

    P4_0, P4_1, P4_2, P4_3, P4_4, P4_5, P4_6, P4_7, P4_8, P4_9, P4_10,P4_11,P4_12,P4_13,P4_14,P4_15,
    P4_16,P4_17,P4_18,P4_19,P4_20,P4_21,P4_22,P4_23,P4_24,P4_25,P4_26,P4_27,P4_28,P4_29,P4_30,P4_31,

    P_MAX,
    P_NC = 0xff
}gpioPins_et; 



/*************************************************************************************************
                           Constants for PIN Function Selection
*************************************************************************************************/
#define PINSEL_FUNC_0 0x00u   
#define PINSEL_FUNC_1 0x01u
#define PINSEL_FUNC_2 0x02u
#define PINSEL_FUNC_3 0x03u
/*************************************************************************************************/

/***********************************************************************
 * Macros define for Pin mode
 **********************************************************************/
#define PINSEL_PINMODE_PULLUP		((0))	/**< Internal pull-up resistor*/
#define PINSEL_PINMODE_TRISTATE 	((2))	/**< Tri-state */
#define PINSEL_PINMODE_PULLDOWN 	((3)) 	/**< Internal pull-down resistor */

/***********************************************************************
 * Macros define for Pin mode (normal/open drain)
 **********************************************************************/
#define	PINSEL_PINMODE_NORMAL		((0))	/**< Pin is in the normal (not open drain) mode.*/
#define	PINSEL_PINMODE_OPENDRAIN	((1)) 	/**< Pin is in the open drain mode */



/***************************************************************************************************
                             Function prototypes
***************************************************************************************************/
void GPIO_PinFunction(uint8_t v_pinNumber_u8, uint8_t v_pinFunction_u8);
void GPIO_PinDirection(uint8_t v_pinNumber_u8, uint8_t v_pinDirn_u8);
void GPIO_PinWrite(uint8_t v_pinNumber_u8, uint8_t v_pinValue_u8);
void GPIO_PinToggle(uint8_t v_pinNumber_u8);
uint8_t GPIO_PinRead(uint8_t v_pinNumber_u8);
void set_ResistorMode ( uint8_t portnum, uint8_t pinnum, uint8_t modenum);
void set_OpenDrainMode( uint8_t portnum, uint8_t pinnum, uint8_t modenum);
/**************************************************************************************************/
#endif
