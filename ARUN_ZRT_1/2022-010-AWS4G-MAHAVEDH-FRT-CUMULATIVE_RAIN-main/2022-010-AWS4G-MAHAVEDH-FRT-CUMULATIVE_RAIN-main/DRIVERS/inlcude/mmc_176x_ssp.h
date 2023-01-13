#ifndef _MMC_176X_SSP_H_
#define _MMC_176X_SSP_H_



#define SSP_CH	0	/* SSP channel to use (0:SSP0, 1:SSP1) */

#define	CCLK		100000000UL	/* cclk frequency [Hz] */
#define PCLK_SSP	50000000UL	/* PCLK frequency to be supplied for SSP [Hz] */
#define SCLK_FAST	25000000UL	/* SCLK frequency under normal operation [Hz] */
#define	SCLK_SLOW	400000UL	/* SCLK frequency under initialization [Hz] */

#define	MMC_CD		((FIO2PIN1 & _BV(3)))	/* Card detect (yes:true, no:false, default:true) */
#define	MMC_WP		0						/* Write protected (yes:true, no:false, default:false) */

#if SSP_CH == 0
#define	SSPxDR		SSP0DR
#define	SSPxSR		SSP0SR
#define	SSPxCR0		SSP0CR0
#define	SSPxCR1		SSP0CR1
#define	SSPxCPSR	SSP0CPSR
#define	CS_LOW()	{FIO0CLR2 = _BV(0);}	/* Set P0.16 low */
#define	CS_HIGH()	{FIO0SET2 = _BV(0);}	/* Set P0.16 high */
#define PCSSPx		PCSSP0
#define	PCLKSSPx	PCLK_SSP0
#define ATTACH_SSP() {\
		__set_PINSEL(0, 15, 2U);	/* SCK0 */\
		__set_PINSEL(0, 17, 2);	/* MISO0 */\
		__set_PINSEL(0, 18, 2);	/* MOSI0 */\
		FIO0DIR |= _BV(16);		/* CS# (P0.16) */\
		}
#elif SSP_CH == 1
#define	SSPxDR		SSP1DR
#define	SSPxSR		SSP1SR
#define	SSPxCR0		SSP1CR0
#define	SSPxCR1		SSP1CR1
#define	SSPxCPSR	SSP1CPSR
#define	CS_LOW()	{FIO0CLR0 = _BV(6);}	/* Set P0.6 low */
#define	CS_HIGH()	{FIO0SET0 = _BV(6);}	/* Set P0.6 high */
#define PCSSPx		PCSSP1
#define	PCLKSSPx	PCLK_SSP1
#define ATTACH_SSP() {\
		__set_PINSEL(0, 7, 2);	/* SCK1 */\
		__set_PINSEL(0, 8, 2);	/* MISO1 */\
		__set_PINSEL(0, 9, 2);	/* MOSI1 */\
		FIO0DIR |= _BV(6);		/* CS# (P0.6) */\
		}
#endif

#if PCLK_SSP * 1 == CCLK
#define PCLKDIV_SSP	PCLKDIV_1
#elif PCLK_SSP * 2 == CCLK
#define PCLKDIV_SSP	PCLKDIV_2
#elif PCLK_SSP * 4 == CCLK
#define PCLKDIV_SSP	PCLKDIV_4
#elif PCLK_SSP * 8 == CCLK
#define PCLKDIV_SSP	PCLKDIV_8
#else
#error Invalid CCLK:PCLK_SSP combination.
#endif


#define FCLK_FAST() { SSPxCR0 = (SSPxCR0 & 0x00FF) | ((PCLK_SSP / 2 / SCLK_FAST) - 1) << 8; }
#define FCLK_SLOW() { SSPxCR0 = (SSPxCR0 & 0x00FF) | ((PCLK_SSP / 2 / SCLK_SLOW) - 1) << 8; }



/*--------------------------------------------------------------------------

   Module Private Functions

---------------------------------------------------------------------------*/

/* MMC/SD command */
#define CMD0	(0)			/* GO_IDLE_STATE */
#define CMD1	(1)			/* SEND_OP_COND (MMC) */
#define	ACMD41	(0x80+41)	/* SEND_OP_COND (SDC) */
#define CMD8	(8)			/* SEND_IF_COND */
#define CMD9	(9)			/* SEND_CSD */
#define CMD10	(10)		/* SEND_CID */
#define CMD12	(12)		/* STOP_TRANSMISSION */
#define ACMD13	(0x80+13)	/* SD_STATUS (SDC) */
#define CMD16	(16)		/* SET_BLOCKLEN */
#define CMD17	(17)		/* READ_SINGLE_BLOCK */
#define CMD18	(18)		/* READ_MULTIPLE_BLOCK */
#define CMD23	(23)		/* SET_BLOCK_COUNT (MMC) */
#define	ACMD23	(0x80+23)	/* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24	(24)		/* WRITE_BLOCK */
#define CMD25	(25)		/* WRITE_MULTIPLE_BLOCK */
#define CMD32	(32)		/* ERASE_ER_BLK_START */
#define CMD33	(33)		/* ERASE_ER_BLK_END */
#define CMD38	(38)		/* ERASE */
#define	CMD48	(48)		/* READ_EXTR_SINGLE */
#define	CMD49	(49)		/* WRITE_EXTR_SINGLE */
#define CMD55	(55)		/* APP_CMD */
#define CMD58	(58)		/* READ_OCR */

int f_CreateFolder(const char* Parameters, const char* path);
//void sd_append_file( void );
bool create_file( void );
bool create_folder( void );
bool sd_append_file( void );
void getdetails( void );
void get_name( void );
void getdetails( void );
void set_date_time( void );
void sd_filesize( void );
void sd_create_file( void );
void sd_file_remove( void );
void sd_create_dir( void );
void sd_time_date( void );
void write_data( void );
void get_foldername( void );
void sd_format( void );
int xatoi (			/* 0:Failed, 1:Successful */
	char **str,		/* Pointer to pointer to the string */
	long *res		/* Pointer to the valiable to store the value */
);
	
void show_volume_stat(char *ptr); /* fs [<path>] - Show volume status */
void dir_list(char *ptr); /* fl [<path>] - ditectory listing */
void read_file( void ); /* get name - read file */
	
bool Writelog( void ); /* A log to the file */
int Createlog(
	const char* ,  /* A log to the file */
	const char* 	 /* Pointer to the file object */
);
	
int log_SMS( 
	//const char* 	 /* Pointer to the file object */
 void
);
 
int readconfigfile(char*);
int retrylog( void );
int smsUpdateConfig ( const char* ,const char* );

	
#endif
