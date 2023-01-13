#ifndef _GPRS_H_
#define _GPRS_H_

#include <stdbool.h>
#include "stdutils.h"
#include "rtc.h"


//#define DEFAULT_TIMEOUT		15000	/* Define default timeout */
#define DEFAULT_CRLF_COUNT	2	/* Define default CRLF count */

#define POST			1	/* Define method */
#define GET				0

/* Select Demo */
//#define GET_DEMO			/* Define GET demo */
#define POST_DEMO			/* Define POST demo */
#define USERNAME		""
#define PASSWORD		""

#define GSM_RESET		P2_2 // GSM POWER PIN to reset the module
#define GSM_SLEEP_MODE	P1_21 // GSM POWER PIN to reset the module

extern char URLbuffer[100];  // Buffer - Stores the received bytes from Sd card ini.txt file
extern char imei[16];
extern char* APN;
extern char latitude_buffer[16],longitude_buffer[16];
extern char altitude_buffer[8], Satellites_In_View[3];
extern char _sQuality[5];
extern char _Rain[8]; 
extern char _xyz11[8];//_xyz is Cummuletive rain store varable used in packet format   //

extern char _apnBuffer[15];
/* Send interval variable for packet to send on this defined variable time*/
extern	char logInterval, PacketlogInterval, PacketsendInterval;
extern	char _logInterval[4], _PacketlogInterval[4], _PacketsendInterval[4];


enum MODEM_RESPONSE_STATUS		/* Enumerate response status */
{
	MODEM_RESPONSE_WAITING,
	MODEM_RESPONSE_FINISHED,
	MODEM_RESPONSE_TIMEOUT,
	MODEM_RESPONSE_BUFFER_FULL,
	MODEM_RESPONSE_STARTING,
	MODEM_RESPONSE_ERROR
};

#define GNSS_STILL_POWER_ON 			3
#define TIME_SYNCHRONIZE_NOT_COMPLETE	4

/************************** INITIALIZATION REALTED ***************************************************/

bool modem_initialization(void); // SIM900 related functions excutes under this function
bool MODEM_CheckAttention(void);
bool Query_Sim_Insert(void); /* Function to check SIM availaibilty*/
bool Wait_Till_SimModule_Communication(void); /* wait till it become 3 which means sim and module have 
											communicated and ready for sms ,call and data , 
											we can send it multiple times till we get +QINISTAT:3 */
bool MODEMGetIMEI(void);
bool Query_Set_CLTS(void);
bool MODEM_GetNetworkTime(void);
bool MODEM_SetPhoneFunctionality(void);

void Module_PowerOff(void);
void ShutDownModule(void);
void rtc_1call(void);  /* reset comulative rain at 8:21*/
void rtc_2call(void); /* reset comulative rain at low battery condition */

/************************** GNSS RELATED **********************************************/

bool GNSS_PowerON(void);
bool GNSS_PowerOff(void);
uint8_t GNSS_Connect(char* _APN); /* Connect to GPRS */
bool ReadNavigation(void); /* Read Navigation */


/************************** GPRS INITIALIZATION RELATED **********************************************/

bool modemCheck_initialization(char* _APN);
bool MODEMGetSignalStrength(void);
bool Query_NetworkRegistration(void);
bool Query_GPRSRegistrationStatus(void);
bool MODEMGetAPN(void);

/************************** HTTP POST RELATED **********************************************/
uint8_t PackOffLog(char* ServerPack);
bool Check_LocalIP(void);
bool HTTP_SetURL(char * url);
uint8_t HTTP_Post(char* Parameters,char input_time, char read_time );
bool Check_QIState(void);
bool Module_deactivateBearerProfile(void);
bool Module_PDP_Activation(void);

/************************** Other routines **********************************************/

bool WaitForExpectedResponse(char* ExpectedResponse,unsigned int default_timeout);
void Read_Response(unsigned int default_timeout);
void Start_Read_Response(unsigned int default_timeout);
void Buffer_Flush(void);
//void GetResponseBody(char* Response, uint16_t ResponseLength);
bool SendATandExpectResponse(char* ATCommand, char* ExpectedResponse, unsigned int Wait_time);
bool WaitForStatus200_302(char* ExpectedResponse1,char* ExpectedResponse2, unsigned int default_timeout);

bool Update_RTC(char *time);

//void Update_RTC(void); // Update the RTC  - clock time stamp recieved modem network
void LCD_ScrollDisplay(void); // scrolling the screen on LCD
void myExtIntrIsr_1(void);
void myExtIntrIsr_3(void);
void sampleTestlog(void); // sample test log rountine
static void store_gps( char *response );   // Store GPS information in a buffer
static void gsm_gnss_get_param( char* response, char* out, uint8_t index);
void RestartModule(void);

void calcRainFall(void);
void _24hr_RainFall(void);
void Yearly_Rain(void);


/************************* GSM Sleep mode Function Declaration ****************/
void Module_sleep_mode(void);
bool Module_awakeup_mode(void);
bool Wakup_PDP_Activation(char* APN);
bool Module_Initializing(void);

/************************* SMS Function Declarations **************************/

void SMSDefaultSetting(void);							
void GSM_Calling(char *);
void GSM_HangCall(void);
void GSM_Response(void);
void GSM_Response_Display(void);
void GSM_Msg_Read(int);
bool GSM_Wait_for_Msg(void);
void GSM_Msg_Display(void);
void GSM_Msg_Delete(unsigned int);
bool GSM_Send_Msg(char* , char*);
void GSM_Delete_All_Msg(void);
void Get_SMSC_Number(void);
void SMS_READ_Setting (void);

extern int ERROR_CODE;
void readSMS(void);

#endif
