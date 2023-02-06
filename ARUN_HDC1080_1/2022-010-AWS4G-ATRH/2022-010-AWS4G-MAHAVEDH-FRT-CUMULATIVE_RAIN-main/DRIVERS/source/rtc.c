/**********************************************************************
* $Id$		lpc17xx_rtc.c				2011-06-06
*//**
* @file		lpc17xx_rtc.c
* @brief	Contains all functions support for RTC firmware library on LPC17xx
* @version	3.1
* @date		6. June. 2011
* @author	NXP MCU SW Application Team
*
* Copyright(C) 2011, NXP Semiconductor
* All rights reserved.
*
***********************************************************************
* Software that is described herein is for illustrative purposes only
* which provides customers with programming information regarding the
* products. This software is supplied "AS IS" without any warranties.
* NXP Semiconductors assumes no responsibility or liability for the
* use of the software, conveys no license or title under any patent,
* copyright, or mask work right to the product. NXP Semiconductors
* reserves the right to make changes in the software without
* notification. NXP Semiconductors also make no representation or
* warranty that such application will be suitable for the specified
* use without further testing or modification.
**********************************************************************/

/***************************** include Header files *********************************************/
#include "rtc.h"
#include "lpc17xx_clkpwr.h"
#include "uart.h" 

/* If this source file built with example, the LPC17xx FW library configuration
 * file in each example directory ("lpc17xx_libcfg.h") must be included,
 * otherwise the default FW library configuration file must be included instead
 */

#ifdef __BUILD_WITH_EXAMPLE__
#include "lpc17xx_libcfg.h"
#else
#include "lpc17xx_libcfg_default.h"
#endif /* __BUILD_WITH_EXAMPLE__ */

/************************* Globle variables *********************************/
volatile uint32_t alarm_on = 0;

#ifdef _RTC

/*****************************************************************************************
*                                 RTC IRQ Handler   																		 *
*****************************************************************************************/
void RTC_IRQHandler(void) {
	//LPC_RTC->ILR |= RTC_IRL_RTCCIF;                /* clear interrupt flag */
//	uint32_t secval;

//	/* This is increment counter interrupt*/
	if (RTC_GetIntPending( RTC_INT_COUNTER_INCREASE))
	{
	//	secval = RTC_GetTime (LPC_RTC, RTC_TIMETYPE_SECOND);
//
//		/* Send debug information */
//	//	_DBG ("Second: "); _DBD(secval);
//	//	_DBG_("");
	//	console_log("SEC:%d\n\r",secval);
//
//		// Clear pending interrupt
	RTC_ClearIntPending( RTC_INT_COUNTER_INCREASE);
	}
//	LPC_RTC->ILR |= RTC_IRL_RTCCIF;                /* clear interrupt flag */
	alarm_on = 1;
	return;
}

/**********************************************************************
 * @brief		Initializes the RTC peripheral.
 * @param[in]	RTCx	RTC peripheral selected, should be LPC_RTC
 * @return 		None
 *********************************************************************/
void RTC_Init (void)
{
	alarm_on = 0;
//	CHECK_PARAM(PARAM_RTCx(RTCx));	
//	/* Enable CLOCK into RTC */
// 	 LPC_SC->PCONP |= (1 << 9);
	/* Set up clock and power for RTC module */
	CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCRTC, ENABLE);

	/* If RTC is stopped, clear STOP bit. */
	  if ( LPC_RTC->RTC_AUX & (0x1<<4) )
 	 {
		LPC_RTC->RTC_AUX |= (0x1<<4);	
 	 }

	// Clear all register to be default
	LPC_RTC->AMR = 0x00;
	LPC_RTC->ILR = 0x00;
	LPC_RTC->CCR = 0x00;
	LPC_RTC->CIIR = 0x00;
//	RTCx->AMR = 0xFF;
	LPC_RTC->CALIBRATION = 0x00;


	/*--- Start RTC counters ---*/
  LPC_RTC->CCR |= RTC_CCR_CLKEN;
  return;
}

/*****************************************************************************
** Function name:		RTCStart
**
** Descriptions:		Start RTC timer
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void RTCStart( void ) 
{
  /*--- Start RTC counters ---*/
 // LPC_RTC->CCR |= CCR_CLKEN;

  /*--- Enable interrupt ---*/
  LPC_RTC->ILR = RTC_IRL_RTCCIF;
  LPC_RTC->CIIR = 0x02;
  return;
}

/*****************************************************************************
** Function name:		RTC_IntDisable
**
** Descriptions:		Start RTC timer
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void RTC_IntDisable( void ) 
{
  LPC_RTC->ILR |= RTC_IRL_RTCCIF;
  LPC_RTC->CIIR = 0x00;
}

/*****************************************************************************
** Function name:		RTCStop
**
** Descriptions:		Stop RTC timer
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void RTCStop( void )
{   
  /*--- Stop RTC counters ---*/
  LPC_RTC->CCR &= ~RTC_CCR_CLKEN;
  return;
}
 
/*************************************************************************
 * @brief		De-initializes the RTC peripheral registers to their
*                  default reset values.
 * @param[in]	RTCx	RTC peripheral selected, should be LPC_RTC
 * @return 		None
 **********************************************************************/
void RTC_DeInit(void)
{
	//CHECK_PARAM(PARAM_RTCx(RTCx));

	LPC_RTC->CCR = 0x00;
	// Disable power and clock for RTC module
	CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCRTC, DISABLE);
}

/*************************************************************************
 * @brief 		Reset clock tick counter in RTC peripheral
 * @param[in]	RTCx	RTC peripheral selected, should be LPC_RTC
 * @return 		None
 **********************************************************************/
void RTC_ResetClockTickCounter(void)
{
	//CHECK_PARAM(PARAM_RTCx(RTCx));

	LPC_RTC->CCR |= RTC_CCR_CTCRST;
	LPC_RTC->CCR &= (~RTC_CCR_CTCRST) & RTC_CCR_BITMASK;
}

/*************************************************************************
 * @brief 		Start/Stop RTC peripheral
 * @param[in]	RTCx	RTC peripheral selected, should be LPC_RTC
 * @param[in]	NewState New State of this function, should be:
 * 				- ENABLE: The time counters are enabled
 * 				- DISABLE: The time counters are disabled
 * @return 		None
 **********************************************************************/
void RTC_Cmd (FunctionalState NewState)
{
//	CHECK_PARAM(PARAM_RTCx(RTCx));
//	CHECK_PARAM(PARAM_FUNCTIONALSTATE(NewState));

	if (NewState == ENABLE)
	{
		LPC_RTC->CCR |= RTC_CCR_CLKEN;
	}
	else
	{
		LPC_RTC->CCR &= (~RTC_CCR_CLKEN) & RTC_CCR_BITMASK;
	}
}

/*************************************************************************
 * @brief 		Enable/Disable Counter increment interrupt for each time type
 * 				in RTC peripheral
 * @param[in]	RTCx	RTC peripheral selected, should be LPC_RTC
 * @param[in]	CntIncrIntType: Counter Increment Interrupt type,
 * 				an increment of this type value below will generates
 * 				an interrupt, should be:
 * 				- RTC_TIMETYPE_SECOND
 * 				- RTC_TIMETYPE_MINUTE
 * 				- RTC_TIMETYPE_HOUR
 * 				- RTC_TIMETYPE_DAYOFWEEK
 * 				- RTC_TIMETYPE_DAYOFMONTH
 * 				- RTC_TIMETYPE_DAYOFYEAR
 * 				- RTC_TIMETYPE_MONTH
 * 				- RTC_TIMETYPE_YEAR
 * @param[in]	NewState New State of this function, should be:
 * 				- ENABLE: Counter Increment interrupt for this
 * 					time type are enabled
 * 				- DISABLE: Counter Increment interrupt for this
 * 					time type are disabled
 * @return 		None
 **********************************************************************/
void RTC_CntIncrIntConfig (uint32_t CntIncrIntType,FunctionalState NewState)
{
//	CHECK_PARAM(PARAM_RTCx(RTCx));
//	CHECK_PARAM(PARAM_FUNCTIONALSTATE(NewState));
//	CHECK_PARAM(PARAM_RTC_TIMETYPE(CntIncrIntType));

	if (NewState ==  ENABLE)
	{
		switch (CntIncrIntType)
		{
		case RTC_TIMETYPE_SECOND:
			LPC_RTC->CIIR |= RTC_CIIR_IMSEC;
			break;
		case RTC_TIMETYPE_MINUTE:
			LPC_RTC->CIIR |= RTC_CIIR_IMMIN;
			break;
		case RTC_TIMETYPE_HOUR:
			LPC_RTC->CIIR |= RTC_CIIR_IMHOUR;
			break;
		case RTC_TIMETYPE_DAYOFWEEK:
			LPC_RTC->CIIR |= RTC_CIIR_IMDOW;
			break;
		case RTC_TIMETYPE_DAYOFMONTH:
			LPC_RTC->CIIR |= RTC_CIIR_IMDOM;
			break;
		case RTC_TIMETYPE_DAYOFYEAR:
			LPC_RTC->CIIR |= RTC_CIIR_IMDOY;
			break;
		case RTC_TIMETYPE_MONTH:
			LPC_RTC->CIIR |= RTC_CIIR_IMMON;
			break;
		case RTC_TIMETYPE_YEAR:
			LPC_RTC->CIIR |= RTC_CIIR_IMYEAR;
			break;
		}
	}
	else
	{
		switch (CntIncrIntType)
		{
		case RTC_TIMETYPE_SECOND:
			LPC_RTC->CIIR &= (~RTC_CIIR_IMSEC) & RTC_CIIR_BITMASK;
			break;
		case RTC_TIMETYPE_MINUTE:
			LPC_RTC->CIIR &= (~RTC_CIIR_IMMIN) & RTC_CIIR_BITMASK;
			break;
		case RTC_TIMETYPE_HOUR:
			LPC_RTC->CIIR &= (~RTC_CIIR_IMHOUR) & RTC_CIIR_BITMASK;
			break;
		case RTC_TIMETYPE_DAYOFWEEK:
			LPC_RTC->CIIR &= (~RTC_CIIR_IMDOW) & RTC_CIIR_BITMASK;
			break;
		case RTC_TIMETYPE_DAYOFMONTH:
			LPC_RTC->CIIR &= (~RTC_CIIR_IMDOM) & RTC_CIIR_BITMASK;
			break;
		case RTC_TIMETYPE_DAYOFYEAR:
			LPC_RTC->CIIR &= (~RTC_CIIR_IMDOY) & RTC_CIIR_BITMASK;
			break;
		case RTC_TIMETYPE_MONTH:
			LPC_RTC->CIIR &= (~RTC_CIIR_IMMON) & RTC_CIIR_BITMASK;
			break;
		case RTC_TIMETYPE_YEAR:
			LPC_RTC->CIIR &= (~RTC_CIIR_IMYEAR) & RTC_CIIR_BITMASK;
			break;
		}
	}
}

/*************************************************************************
 * @brief 		Enable/Disable Alarm interrupt for each time type
 * 				in RTC peripheral
 * @param[in]	RTCx	RTC peripheral selected, should be LPC_RTC
 * @param[in]	AlarmTimeType: Alarm Time Interrupt type,
 * 				an matching of this type value below with current time
 * 				in RTC will generates an interrupt, should be:
 * 				- RTC_TIMETYPE_SECOND
 * 				- RTC_TIMETYPE_MINUTE
 * 				- RTC_TIMETYPE_HOUR
 * 				- RTC_TIMETYPE_DAYOFWEEK
 * 				- RTC_TIMETYPE_DAYOFMONTH
 * 				- RTC_TIMETYPE_DAYOFYEAR
 * 				- RTC_TIMETYPE_MONTH
 * 				- RTC_TIMETYPE_YEAR
 * @param[in]	NewState New State of this function, should be:
 * 				- ENABLE: Alarm interrupt for this
 * 					time type are enabled
 * 				- DISABLE: Alarm interrupt for this
 * 					time type are disabled
 * @return 		None
 **********************************************************************/
void RTC_AlarmIntConfig (uint32_t AlarmTimeType, FunctionalState NewState)
{
//	CHECK_PARAM(PARAM_RTCx(RTCx));
//	CHECK_PARAM(PARAM_FUNCTIONALSTATE(NewState));
//	CHECK_PARAM(PARAM_RTC_TIMETYPE(AlarmTimeType));

	if (NewState == ENABLE)
	{
		switch (AlarmTimeType)
		{
		case RTC_TIMETYPE_SECOND:
			LPC_RTC->AMR &= (~RTC_AMR_AMRSEC) & RTC_AMR_BITMASK;
			break;
		case RTC_TIMETYPE_MINUTE:
			LPC_RTC->AMR &= (~RTC_AMR_AMRMIN) & RTC_AMR_BITMASK;
			break;
		case RTC_TIMETYPE_HOUR:
			LPC_RTC->AMR &= (~RTC_AMR_AMRHOUR) & RTC_AMR_BITMASK;
			break;
		case RTC_TIMETYPE_DAYOFWEEK:
			LPC_RTC->AMR &= (~RTC_AMR_AMRDOW) & RTC_AMR_BITMASK;
			break;
		case RTC_TIMETYPE_DAYOFMONTH:
			LPC_RTC->AMR &= (~RTC_AMR_AMRDOM) & RTC_AMR_BITMASK;
			break;
		case RTC_TIMETYPE_DAYOFYEAR:
			LPC_RTC->AMR &= (~RTC_AMR_AMRDOY) & RTC_AMR_BITMASK;
			break;
		case RTC_TIMETYPE_MONTH:
			LPC_RTC->AMR &= (~RTC_AMR_AMRMON) & RTC_AMR_BITMASK;
			break;
		case RTC_TIMETYPE_YEAR:
			LPC_RTC->AMR &= (~RTC_AMR_AMRYEAR) & RTC_AMR_BITMASK;
			break;
		}
	}
	else
	{
		switch (AlarmTimeType)
		{
		case RTC_TIMETYPE_SECOND:
			LPC_RTC->AMR |= (RTC_AMR_AMRSEC);
			break;
		case RTC_TIMETYPE_MINUTE:
			LPC_RTC->AMR |= (RTC_AMR_AMRMIN);
			break;
		case RTC_TIMETYPE_HOUR:
			LPC_RTC->AMR |= (RTC_AMR_AMRHOUR);
			break;
		case RTC_TIMETYPE_DAYOFWEEK:
			LPC_RTC->AMR |= (RTC_AMR_AMRDOW);
			break;
		case RTC_TIMETYPE_DAYOFMONTH:
			LPC_RTC->AMR |= (RTC_AMR_AMRDOM);
			break;
		case RTC_TIMETYPE_DAYOFYEAR:
			LPC_RTC->AMR |= (RTC_AMR_AMRDOY);
			break;
		case RTC_TIMETYPE_MONTH:
			LPC_RTC->AMR |= (RTC_AMR_AMRMON);
			break;
		case RTC_TIMETYPE_YEAR:
			LPC_RTC->AMR |= (RTC_AMR_AMRYEAR);
			break;
		}
	}
}

/*************************************************************************
 * @brief 		Set current time value for each time type in RTC peripheral
 * @param[in]	RTCx	RTC peripheral selected, should be LPC_RTC
 * @param[in]	Timetype: Time Type, should be:
 * 				- RTC_TIMETYPE_SECOND
 * 				- RTC_TIMETYPE_MINUTE
 * 				- RTC_TIMETYPE_HOUR
 * 				- RTC_TIMETYPE_DAYOFWEEK
 * 				- RTC_TIMETYPE_DAYOFMONTH
 * 				- RTC_TIMETYPE_DAYOFYEAR
 * 				- RTC_TIMETYPE_MONTH
 * 				- RTC_TIMETYPE_YEAR
 * @param[in]	TimeValue Time value to set
 * @return 		None
 **********************************************************************/
void RTC_SetTime (uint32_t Timetype, uint32_t TimeValue)
{
//	CHECK_PARAM(PARAM_RTCx(RTCx));
//	CHECK_PARAM(PARAM_RTC_TIMETYPE(Timetype));

	switch ( Timetype)
	{
	case RTC_TIMETYPE_SECOND:
		//CHECK_PARAM(TimeValue <= RTC_SECOND_MAX);
		if(TimeValue <= RTC_SECOND_MAX){
		LPC_RTC->SEC = TimeValue & RTC_SEC_MASK;}
		break;

	case RTC_TIMETYPE_MINUTE:
	//	CHECK_PARAM(TimeValue <= RTC_MINUTE_MAX);
		if(TimeValue <= RTC_MINUTE_MAX){
		LPC_RTC->MIN = TimeValue & RTC_MIN_MASK;}
		break;

	case RTC_TIMETYPE_HOUR:
	//	CHECK_PARAM(TimeValue <= RTC_HOUR_MAX);
		if(TimeValue <= RTC_HOUR_MAX){
		LPC_RTC->HOUR = TimeValue & RTC_HOUR_MASK;}
		break;

	case RTC_TIMETYPE_DAYOFWEEK:
	//	CHECK_PARAM(TimeValue <= RTC_DAYOFWEEK_MAX);
		if(TimeValue <= RTC_DAYOFWEEK_MAX){
		LPC_RTC->DOW = TimeValue & RTC_DOW_MASK;}
		break;

	case RTC_TIMETYPE_DAYOFMONTH:
//		CHECK_PARAM((TimeValue <= RTC_DAYOFMONTH_MAX) \
//				&& (TimeValue >= RTC_DAYOFMONTH_MIN));
		if((TimeValue <= RTC_DAYOFMONTH_MAX) && (TimeValue >= RTC_DAYOFMONTH_MIN)){
		LPC_RTC->DOM = TimeValue & RTC_DOM_MASK;}
		break;

	case RTC_TIMETYPE_DAYOFYEAR:
//		CHECK_PARAM((TimeValue >= RTC_DAYOFYEAR_MIN) \
//				&& (TimeValue <= RTC_DAYOFYEAR_MAX));
		if((TimeValue >= RTC_DAYOFYEAR_MIN) && (TimeValue <= RTC_DAYOFYEAR_MAX)) {
		LPC_RTC->DOY = TimeValue & RTC_DOY_MASK; }
		break;

	case RTC_TIMETYPE_MONTH:
//		CHECK_PARAM((TimeValue >= RTC_MONTH_MIN) \
//				&& (TimeValue <= RTC_MONTH_MAX));
		if((TimeValue >= RTC_MONTH_MIN) && (TimeValue <= RTC_MONTH_MAX)){
		LPC_RTC->MONTH = TimeValue & RTC_MONTH_MASK;}
		break;

	case RTC_TIMETYPE_YEAR:
//		CHECK_PARAM(TimeValue <= RTC_YEAR_MAX);
		if(TimeValue <= RTC_YEAR_MAX){
		LPC_RTC->YEAR = TimeValue & RTC_YEAR_MASK;}
		break;
	}
}

/*************************************************************************
 * @brief 		Get current time value for each type time type
 * @param[in]	RTCx	RTC peripheral selected, should be LPC_RTC
 * @param[in]	Timetype: Time Type, should be:
 * 				- RTC_TIMETYPE_SECOND
 * 				- RTC_TIMETYPE_MINUTE
 * 				- RTC_TIMETYPE_HOUR
 * 				- RTC_TIMETYPE_DAYOFWEEK
 * 				- RTC_TIMETYPE_DAYOFMONTH
 * 				- RTC_TIMETYPE_DAYOFYEAR
 * 				- RTC_TIMETYPE_MONTH
 * 				- RTC_TIMETYPE_YEAR
 * @return 		Value of time according to specified time type
 **********************************************************************/
uint32_t RTC_GetTime( uint32_t Timetype)
{
//	CHECK_PARAM(PARAM_RTCx(RTCx));
//	CHECK_PARAM(PARAM_RTC_TIMETYPE(Timetype));

	switch (Timetype)
	{
	case RTC_TIMETYPE_SECOND:
		return (LPC_RTC->SEC & RTC_SEC_MASK);
	case RTC_TIMETYPE_MINUTE:
		return (LPC_RTC->MIN & RTC_MIN_MASK);
	case RTC_TIMETYPE_HOUR:
		return (LPC_RTC->HOUR & RTC_HOUR_MASK);
	case RTC_TIMETYPE_DAYOFWEEK:
		return (LPC_RTC->DOW & RTC_DOW_MASK);
	case RTC_TIMETYPE_DAYOFMONTH:
		return (LPC_RTC->DOM & RTC_DOM_MASK);
	case RTC_TIMETYPE_DAYOFYEAR:
		return (LPC_RTC->DOY & RTC_DOY_MASK);
	case RTC_TIMETYPE_MONTH:
		return (LPC_RTC->MONTH & RTC_MONTH_MASK);
	case RTC_TIMETYPE_YEAR:
		return (LPC_RTC->YEAR & RTC_YEAR_MASK);
	default:
		return (0);
	}
}

/*************************************************************************
 * @brief 		Set full of time in RTC peripheral
 * @param[in]	RTCx	RTC peripheral selected, should be LPC_RTC
 * @param[in]	pFullTime Pointer to a RTC_TIME_Type structure that
 * 				contains time value in full.
 * @return 		None
 **********************************************************************/
void RTC_SetFullTime (RTC_TIME_Type *pFullTime)
{
//	CHECK_PARAM(PARAM_RTCx(RTCx));

	LPC_RTC->DOM = pFullTime->DOM & RTC_DOM_MASK;
	LPC_RTC->DOW = pFullTime->DOW & RTC_DOW_MASK;
	LPC_RTC->DOY = pFullTime->DOY & RTC_DOY_MASK;
	LPC_RTC->HOUR = pFullTime->HOUR & RTC_HOUR_MASK;
	LPC_RTC->MIN = pFullTime->MIN & RTC_MIN_MASK;
	LPC_RTC->SEC = pFullTime->SEC & RTC_SEC_MASK;
	LPC_RTC->MONTH = pFullTime->MONTH & RTC_MONTH_MASK;
	LPC_RTC->YEAR = pFullTime->YEAR & RTC_YEAR_MASK;
}

/*************************************************************************
 * @brief 		Get full of time in RTC peripheral
 * @param[in]	RTCx	RTC peripheral selected, should be LPC_RTC
 * @param[in]	pFullTime Pointer to a RTC_TIME_Type structure that
 * 				will be stored time in full.
 * @return 		None
 **********************************************************************/
void RTC_GetFullTime (RTC_TIME_Type *pFullTime)
{
	//CHECK_PARAM(PARAM_RTCx(RTCx));

	pFullTime->DOM = LPC_RTC->DOM & RTC_DOM_MASK;
	pFullTime->DOW = LPC_RTC->DOW & RTC_DOW_MASK;
	pFullTime->DOY = LPC_RTC->DOY & RTC_DOY_MASK;
	pFullTime->HOUR = LPC_RTC->HOUR & RTC_HOUR_MASK;
	pFullTime->MIN = LPC_RTC->MIN & RTC_MIN_MASK;
	pFullTime->SEC = LPC_RTC->SEC & RTC_SEC_MASK;
	pFullTime->MONTH = LPC_RTC->MONTH & RTC_MONTH_MASK;
	pFullTime->YEAR = LPC_RTC->YEAR & RTC_YEAR_MASK;
}

/************************************************************************
 * @brief 		Set alarm time value for each time type
 * @param[in]	RTCx	RTC peripheral selected, should be LPC_RTC
 * @param[in]	Timetype: Time Type, should be:
 * 				- RTC_TIMETYPE_SECOND
 * 				- RTC_TIMETYPE_MINUTE
 * 				- RTC_TIMETYPE_HOUR
 * 				- RTC_TIMETYPE_DAYOFWEEK
 * 				- RTC_TIMETYPE_DAYOFMONTH
 * 				- RTC_TIMETYPE_DAYOFYEAR
 * 				- RTC_TIMETYPE_MONTH
 * 				- RTC_TIMETYPE_YEAR
 * @param[in]	ALValue Alarm time value to set
 * @return 		None
 **********************************************************************/
void RTC_SetAlarmTime (uint32_t Timetype, uint32_t ALValue)
{
	// (PARAM_RTCx(RTCx));

	switch (Timetype)
	{
	case RTC_TIMETYPE_SECOND:
		//CHECK_PARAM(ALValue <= RTC_SECOND_MAX);
		if(ALValue <= RTC_SECOND_MAX){
		LPC_RTC->ALSEC = ALValue & RTC_SEC_MASK;}
		break;

	case RTC_TIMETYPE_MINUTE:
		//CHECK_PARAM(ALValue <= RTC_MINUTE_MAX);
		if(ALValue <= RTC_MINUTE_MAX){
		LPC_RTC->ALMIN = ALValue & RTC_MIN_MASK;}
		break;

	case RTC_TIMETYPE_HOUR:
		//CHECK_PARAM(ALValue <= RTC_HOUR_MAX);
		if(ALValue <= RTC_HOUR_MAX){
		LPC_RTC->ALHOUR = ALValue & RTC_HOUR_MASK;}
		break;

	case RTC_TIMETYPE_DAYOFWEEK:
		//CHECK_PARAM(ALValue <= RTC_DAYOFWEEK_MAX);
		if(ALValue <= RTC_DAYOFWEEK_MAX){
		LPC_RTC->ALDOW = ALValue & RTC_DOW_MASK;}
		break;

	case RTC_TIMETYPE_DAYOFMONTH:
	//	CHECK_PARAM((ALValue <= RTC_DAYOFMONTH_MAX) \
		//		&& (ALValue >= RTC_DAYOFMONTH_MIN));
		if((ALValue <= RTC_DAYOFMONTH_MAX) 	&& (ALValue >= RTC_DAYOFMONTH_MIN)){
		LPC_RTC->ALDOM = ALValue & RTC_DOM_MASK; }
		break;

	case RTC_TIMETYPE_DAYOFYEAR:
//		CHECK_PARAM((ALValue >= RTC_DAYOFYEAR_MIN) \
//				&& (ALValue <= RTC_DAYOFYEAR_MAX));
		if((ALValue >= RTC_DAYOFYEAR_MIN) && (ALValue <= RTC_DAYOFYEAR_MAX)){
		LPC_RTC->ALDOY = ALValue & RTC_DOY_MASK; }
		break;

	case RTC_TIMETYPE_MONTH:
//		CHECK_PARAM((ALValue >= RTC_MONTH_MIN) \
//				&& (ALValue <= RTC_MONTH_MAX));
		if((ALValue >= RTC_MONTH_MIN) && (ALValue <= RTC_MONTH_MAX)){
		LPC_RTC->ALMON = ALValue & RTC_MONTH_MASK; }
		break;

	case RTC_TIMETYPE_YEAR:
//		CHECK_PARAM(ALValue <= RTC_YEAR_MAX);
		if(ALValue <= RTC_YEAR_MAX){
		LPC_RTC->ALYEAR = ALValue & RTC_YEAR_MASK;}
		break;
	}
}

/*************************************************************************
 * @brief 		Get alarm time value for each time type
 * @param[in]	RTCx	RTC peripheral selected, should be LPC_RTC
 * @param[in]	Timetype: Time Type, should be:
 * 				- RTC_TIMETYPE_SECOND
 * 				- RTC_TIMETYPE_MINUTE
 * 				- RTC_TIMETYPE_HOUR
 * 				- RTC_TIMETYPE_DAYOFWEEK
 * 				- RTC_TIMETYPE_DAYOFMONTH
 * 				- RTC_TIMETYPE_DAYOFYEAR
 * 				- RTC_TIMETYPE_MONTH
 * 				- RTC_TIMETYPE_YEAR
  * @return 	Value of Alarm time according to specified time type
 **********************************************************************/
uint32_t RTC_GetAlarmTime (uint32_t Timetype)
{
	switch (Timetype)
	{
	case RTC_TIMETYPE_SECOND:
		return (LPC_RTC->ALSEC & RTC_SEC_MASK);
	case RTC_TIMETYPE_MINUTE:
		return (LPC_RTC->ALMIN & RTC_MIN_MASK);
	case RTC_TIMETYPE_HOUR:
		return (LPC_RTC->ALHOUR & RTC_HOUR_MASK);
	case RTC_TIMETYPE_DAYOFWEEK:
		return (LPC_RTC->ALDOW & RTC_DOW_MASK);
	case RTC_TIMETYPE_DAYOFMONTH:
		return (LPC_RTC->ALDOM & RTC_DOM_MASK);
	case RTC_TIMETYPE_DAYOFYEAR:
		return (LPC_RTC->ALDOY & RTC_DOY_MASK);
	case RTC_TIMETYPE_MONTH:
		return (LPC_RTC->ALMON & RTC_MONTH_MASK);
	case RTC_TIMETYPE_YEAR:
		return (LPC_RTC->ALYEAR & RTC_YEAR_MASK);
	default:
		return (0);
	}
}

/*************************************************************************
 * @brief 		Set full of alarm time in RTC peripheral
 * @param[in]	RTCx	RTC peripheral selected, should be LPC_RTC
 * @param[in]	pFullTime Pointer to a RTC_TIME_Type structure that
 * 				contains alarm time value in full.
 * @return 		None
 **********************************************************************/
void RTC_SetFullAlarmTime (RTC_TIME_Type *pFullTime)
{
	//CHECK_PARAM(PARAM_RTCx(RTCx));

	LPC_RTC->ALDOM = pFullTime->DOM & RTC_DOM_MASK;
	LPC_RTC->ALDOW = pFullTime->DOW & RTC_DOW_MASK;
	LPC_RTC->ALDOY = pFullTime->DOY & RTC_DOY_MASK;
	LPC_RTC->ALHOUR = pFullTime->HOUR & RTC_HOUR_MASK;
	LPC_RTC->ALMIN = pFullTime->MIN & RTC_MIN_MASK;
	LPC_RTC->ALSEC = pFullTime->SEC & RTC_SEC_MASK;
	LPC_RTC->ALMON = pFullTime->MONTH & RTC_MONTH_MASK;
	LPC_RTC->ALYEAR = pFullTime->YEAR & RTC_YEAR_MASK;
}

/*************************************************************************
 * @brief 		Get full of alarm time in RTC peripheral
 * @param[in]	RTCx	RTC peripheral selected, should be LPC_RTC
 * @param[in]	pFullTime Pointer to a RTC_TIME_Type structure that
 * 				will be stored alarm time in full.
 * @return 		None
 **********************************************************************/
void RTC_GetFullAlarmTime (RTC_TIME_Type *pFullTime)
{
	//CHECK_PARAM(PARAM_RTCx(RTCx));

	pFullTime->DOM = LPC_RTC->ALDOM & RTC_DOM_MASK;
	pFullTime->DOW = LPC_RTC->ALDOW & RTC_DOW_MASK;
	pFullTime->DOY = LPC_RTC->ALDOY & RTC_DOY_MASK;
	pFullTime->HOUR = LPC_RTC->ALHOUR & RTC_HOUR_MASK;
	pFullTime->MIN = LPC_RTC->ALMIN & RTC_MIN_MASK;
	pFullTime->SEC = LPC_RTC->ALSEC & RTC_SEC_MASK;
	pFullTime->MONTH = LPC_RTC->ALMON & RTC_MONTH_MASK;
	pFullTime->YEAR = LPC_RTC->ALYEAR & RTC_YEAR_MASK;
}

/*************************************************************************
 * @brief 		Check whether if specified Location interrupt in
 * 				RTC peripheral is set or not
 * @param[in]	RTCx	RTC peripheral selected, should be LPC_RTC
 * @param[in]	IntType Interrupt location type, should be:
 * 						- RTC_INT_COUNTER_INCREASE: Counter Increment Interrupt
 * 							block generated an interrupt.
 * 						- RTC_INT_ALARM: Alarm generated an
 * 							interrupt.
 * @return 		New state of specified Location interrupt in RTC peripheral
 * 				(SET or RESET)
 **********************************************************************/
IntStatus RTC_GetIntPending (uint32_t IntType)
{
//	CHECK_PARAM(PARAM_RTCx(RTCx));
//	CHECK_PARAM(PARAM_RTC_INT(IntType));

	return ((LPC_RTC->ILR & IntType) ? SET : RESET);
}

/*************************************************************************
 * @brief 		Clear specified Location interrupt pending in
 * 				RTC peripheral
 * @param[in]	RTCx	RTC peripheral selected, should be LPC_RTC
 * @param[in]	IntType Interrupt location type, should be:
 * 						- RTC_INT_COUNTER_INCREASE: Clear Counter Increment
 * 						Interrupt pending.
 * 						- RTC_INT_ALARM: Clear alarm interrupt pending
 * @return 		None
 **********************************************************************/
void RTC_ClearIntPending (uint32_t IntType)
{
//	CHECK_PARAM(PARAM_RTCx(RTCx));
//	CHECK_PARAM(PARAM_RTC_INT(IntType));

	LPC_RTC->ILR |= IntType;
}

/*************************************************************************
 * @brief 		Enable/Disable calibration counter in RTC peripheral
 * @param[in]	RTCx	RTC peripheral selected, should be LPC_RTC
 * @param[in]	NewState New State of this function, should be:
 * 				- ENABLE: The calibration counter is enabled and counting
 * 				- DISABLE: The calibration counter is disabled and reset to zero
 * @return 		None
 **********************************************************************/
void RTC_CalibCounterCmd(FunctionalState NewState)
{
//	CHECK_PARAM(PARAM_RTCx(RTCx));
//	CHECK_PARAM(PARAM_FUNCTIONALSTATE(NewState));

	if (NewState == ENABLE)
	{
		LPC_RTC->CCR &= (~RTC_CCR_CCALEN) & RTC_CCR_BITMASK;
	}
	else
	{
		LPC_RTC->CCR |= RTC_CCR_CCALEN;
	}
}

/*************************************************************************
 * @brief 		Configures Calibration in RTC peripheral
 * @param[in]	RTCx	RTC peripheral selected, should be LPC_RTC
 * @param[in]	CalibValue Calibration value, should be in range from
 * 					0 to 131,072
 * @param[in]	CalibDir Calibration Direction, should be:
 * 					- RTC_CALIB_DIR_FORWARD: Forward calibration
 * 					- RTC_CALIB_DIR_BACKWARD: Backward calibration
 * @return 		None
 **********************************************************************/
void RTC_CalibConfig( uint32_t CalibValue, uint8_t CalibDir)
{
//	CHECK_PARAM(PARAM_RTCx(RTCx));
//	CHECK_PARAM(PARAM_RTC_CALIB_DIR(CalibDir));
//	CHECK_PARAM(CalibValue < RTC_CALIBRATION_MAX);
	if(CalibValue < RTC_CALIBRATION_MAX){
	LPC_RTC->CALIBRATION = ((CalibValue - 1) & RTC_CALIBRATION_CALVAL_MASK) \
			| ((CalibDir == RTC_CALIB_DIR_BACKWARD) ? RTC_CALIBRATION_LIBDIR : 0);	 }
}

/*************************************************************************
 * @brief 		Write value to General purpose registers
 * @param[in]	RTCx	RTC peripheral selected, should be LPC_RTC
 * @param[in]	Channel General purpose registers Channel number,
 * 				should be in range from 0 to 4.
 * @param[in]	Value Value to write
 * @return 		None
 * Note: These General purpose registers can be used to store important
 * information when the main power supply is off. The value in these
 * registers is not affected by chip reset.
 **********************************************************************/
void RTC_WriteGPREG (uint8_t Channel, float Value)
{
	uint32_t *preg;

//	CHECK_PARAM(PARAM_RTCx(RTCx));
//	CHECK_PARAM(PARAM_RTC_GPREG_CH(Channel));

	preg = (uint32_t *)&LPC_RTC->GPREG0;
	preg += Channel;
//	*preg = Value;
	*preg =	*((uint32_t *)&Value);
}

/*************************************************************************
 * @brief 		Read value from General purpose registers
 * @param[in]	RTCx	RTC peripheral selected, should be LPC_RTC
 * @param[in]	Channel General purpose registers Channel number,
 * 				should be in range from 0 to 4.
 * @return 		Read Value
 * Note: These General purpose registers can be used to store important
 * information when the main power supply is off. The value in these
 * registers is not affected by chip reset.
 **********************************************************************/
float RTC_ReadGPREG (uint8_t Channel)
{
	uint32_t *preg;
//	float *preg;
	//uint32_t value;
	 float value;
//	CHECK_PARAM(PARAM_RTCx(RTCx));
//	CHECK_PARAM(PARAM_RTC_GPREG_CH(Channel));

	preg = (uint32_t *)&LPC_RTC->GPREG0;
//	preg = (float *)&LPC_RTC->GPREG0;
	preg += Channel;
	value = *((float *)preg);
//	value = *((float)preg);
	return (value);
}

/***************************************************************************************************
                    void RTC_SetDateTime(rtc_t *rtc)
****************************************************************************************************
 * I/P Arguments: rtc_t *: Pointer to structure of type rtc_t. Structure contains hour,min,sec,day,date,month and year 
 * Return value    : none

 * description  :This function is used to set Date(dd,mm,yy) into the  RTC.
                 The new Date is updated into the non volatile memory of RTC .
		Note: The I/P arguments should of hex or decimal 
	      like 15,08,2047 for 15th day,8th month and 2047th year.				 
***************************************************************************************************/
void RTC_SetDateTime(rtc_t *rtc)
{

     LPC_RTC->SEC   =  rtc->sec;       // Update sec value
	 LPC_RTC->MIN   =  rtc->min;	   // Update min value
     LPC_RTC->HOUR	=  rtc->hour;	   // Update hour value 
	 LPC_RTC->DOW	=  rtc->weekDay;   // Update day value 
	 LPC_RTC->DOM	=  rtc->date;	   // Update date value 
	 LPC_RTC->MONTH =  rtc->month;	   // Update month value
	 LPC_RTC->YEAR  =  rtc->year;	   // Update year value

//     LPC_RTC->SEC   =  00;       // Update sec value
//	 LPC_RTC->MIN   =  15;	   // Update min value
//     LPC_RTC->HOUR	=  19;	   // Update hour value 
//	 LPC_RTC->DOW	=  rtc->weekDay;   // Update day value 
//	 LPC_RTC->DOM	=  17;	   // Update date value 
//	 LPC_RTC->MONTH =  3;	   // Update month value
//	 LPC_RTC->YEAR  =  2020;	   // Update year value

}

/***************************************************************************************************
                     void RTC_GetDateTime(rtc_t *rtc)
****************************************************************************************************
 * I/P Arguments: rtc_t *: Pointer to structure of type rtc_t. Structure contains hour,min,sec,day,date,month and year 
 * Return value    : none

 * description  :This function is used to get the Time(hh,mm,ss) And Date from  RTC.

	Note: The time read from  will be of hex or decimal, 
	      like 12,39,26 for 12hr,39min and 26sec.	
***************************************************************************************************/
void RTC_GetDateTime(rtc_t *rtc)
{
     rtc->sec     = LPC_RTC->SEC ;	   // Read sec value
	 rtc->min     = LPC_RTC->MIN ;	   // Read min value
     rtc->hour    = LPC_RTC->HOUR;	   // Read hour value 
	 rtc->weekDay = LPC_RTC->DOW;      // Read day value 
	 rtc->date    = LPC_RTC->DOM;	   // Read date value 
	 rtc->month   = LPC_RTC->MONTH;	   // Read month value
	 rtc->year    = LPC_RTC->YEAR;	   // Read year value

}

/*****************************************************************************
** Function name:		RTCSetAlarmMask
**
** Descriptions:		Set RTC timer alarm mask
**
** parameters:			Alarm mask setting
** Returned value:		None
** 
*****************************************************************************/
void RTCSetAlarmMask( uint32_t AlarmMask ) 
{
  /*--- Set alarm mask ---*/    
  LPC_RTC->AMR = AlarmMask;
  return;
}

#endif /* _RTC */

/* --------------------------------- End Of File ------------------------------ */

