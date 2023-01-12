/*------------------------------------------------------------------------*/
/* LPCXpresso176x: MMCv3/SDv1/SDv2 (SPI mode) control module              */
/*------------------------------------------------------------------------*/
/*
/  Copyright (C) 2020, ChaN, all right reserved.
/
/ * This software is a free software and there is NO WARRANTY.
/ * No restriction on use. You can use, modify and redistribute it for
/   personal, non-profit or commercial products UNDER YOUR RESPONSIBILITY.
/ * Redistributions of source code must retain the above copyright notice.
/
/--------------------------------------------------------------------------/
/ Code blocks enclosed in #if VIRTUAL_DISK ... #endif is hooking of
/ the feature of disk image in FAT volume.
/-------------------------------------------------------------------------*/

/************************ include Header files *******************************/
#include <stdbool.h>              
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "LPC176x.h"
#include "diskio.h"
#include "mmc_176x_ssp.h"
#include "uart.h"
#include "rtc.h"
#include "delay.h"
#include "convert.h"
#include "gprs.h"
#include "lpc17xx_wdt.h"
#include "systick.h"

/************************* Globle variables *********************************/
static volatile DSTATUS Stat = STA_NOINIT;	/* Physical drive status */
static volatile UINT Timer1, Timer2;				/* 1kHz decrement timer stopped at zero (disk_timerproc()) */
static BYTE CardType;												/* Card type flags */
static FILINFO fno;
static FIL file;
static DIR dir;
static FATFS fs;

uint32_t bytes_written,size=0;
TCHAR fileName[30]={0},fileNme[13]={0};
char foldername [20]={0},foldername1 [20]={'/','\0'};
int n,eof=0;
FRESULT ff_result;
bool fault = false,first_flag=true;
FILINFO Finfo;

FATFS FatFs[FF_VOLUMES];					/* File system object for each logical drive */
TCHAR line_buffer[512];   				/* file copy buffer */
BYTE FailedFirstPendinglog = 0;  	/* flags for files available in sd card */
//FIL File[2];										/* File objects */

extern char _buffer[512]; 				/* main packet buffer initialized in main */
extern int PlogCreate;

/********************** Function routine declarations *********************************/
BYTE Buff[16384] __attribute__ ((aligned (4))) ;	/* Working buffer */
static void put_rc (FRESULT rc);
static FRESULT scan_files ( char*, DWORD*, DWORD*, DWORD*);

/*-----------------------------------------------------------------------*/
/* Send/Receive data to the MMC  (Platform dependent)                    */
/*-----------------------------------------------------------------------*/

/*******************************************************************************
*                        Exchange a byte																			 *
********************************************************************************/
static BYTE xchg_spi (
	BYTE dat	/* Data to send */
)
{
	SSPxDR = dat;
	while (SSPxSR & 0x10) ;
	return SSPxDR;
}

/*******************************************************************************
*                     Receive multiple byte																		 *
********************************************************************************/
static void rcvr_spi_multi (
	BYTE *buff,		/* Pointer to data buffer */
	UINT btr		/* Number of bytes to receive (16, 64 or 512) */
)
{
	UINT n;
	WORD d;


	SSPxCR0 |= 0x000F;	/* Select 16-bit mode */

	for (n = 0; n < 8; n++) {		/* Push dummy frames into Tx FIFO  */
		SSPxDR = 0xFFFF;
	}
	btr -= 16;

	while (btr >= 2) {				/* Receive the data block into buffer */
		btr -= 2;
		while (!(SSPxSR & _BV(2))) ;	/* Wait for a frame in Rx FIFO */
		d = SSPxDR;			/* Read a frame from Rx FIFO */
		SSPxDR = 0xFFFF;	/* Push a dummy frame into TxFIFO */
		*buff++ = d >> 8;
		*buff++ = d;
	}

	for (n = 0; n < 8; n++) {		/* Receive remaining frames in the pipeline */
		while (!(SSPxSR & _BV(2))) ;
		d = SSPxDR;
		*buff++ = d >> 8;
		*buff++ = d;
	}

	SSPxCR0 &= 0xFFF7;				/* Select 8-bit mode */
}

/*******************************************************************************
*                     Send multiple byte						  												 *
********************************************************************************/
static void xmit_spi_multi (
	const BYTE *buff,	/* Pointer to the data */
	UINT btx			/* Number of bytes to send (multiple of 16) */
)
{
	UINT n;
	WORD d;


	SSPxCR0 |= 0x000F;			/* Select 16-bit mode */

	for (n = 0; n < 8; n++) {	/* Fill Tx FIFO */
		d = *buff++;
		d = d << 8 | *buff++;
		SSPxDR = d;
	}
	btx -= 16;

	while (btx >= 2) {			/* Transmit data block */
		btx -= 2;
		d = *buff++;
		d = d << 8 | *buff++;
		while (!(SSPxSR & _BV(2))) ;	/* Wait for a frame in Rx FIFO */
		SSPxDR;		/* Discard a fram in Rx FIFO */
		SSPxDR = d;	/* Push a frame into Tx FIFO */
	}

	for (n = 0; n < 8; n++) {	/* Flush pipeline */
		while (!(SSPxSR & _BV(2))) ;
		SSPxDR;
	}

	SSPxCR0 &= 0xFFF7;			/* Select 8-bit mode */
}

/*-----------------------------------------------------------------------*/
/* Wait for card ready                                                   */
/*-----------------------------------------------------------------------*/
static int wait_ready (	/* 1:Ready, 0:Timeout */
	UINT wt			/* Timeout [ms] */
)
{
	BYTE d;


	Timer2 = wt;
	do {
		d = xchg_spi(0xFF);

		/* This loop takes a time. Insert rot_rdq() here for multitask envilonment. */

	} while (d != 0xFF && Timer2);	/* Wait for card goes ready or timeout */

	return (d == 0xFF) ? 1 : 0;
}

/*-----------------------------------------------------------------------*/
/* Deselect card and release SPI                                         */
/*-----------------------------------------------------------------------*/
static void deselect (void)
{
	CS_HIGH();		/* CS = H */
	xchg_spi(0xFF);	/* Dummy clock (force DO hi-z for multiple slave SPI) */
}

/*-----------------------------------------------------------------------*/
/* Select card and wait for ready                                        */
/*-----------------------------------------------------------------------*/
static int select (void)	/* 1:OK, 0:Timeout */
{
	CS_LOW();		/* CS = L */
	xchg_spi(0xFF);	/* Dummy clock (force DO enabled) */
	if (wait_ready(500)) return 1;	/* Leading busy check: Wait for card ready */
	deselect();		/* Timeout */
	return 0;
}

/*-----------------------------------------------------------------------*/
/* Control SPI module (Platform dependent)                               */
/*-----------------------------------------------------------------------*/

/**************************************************************************
*                     power on SPI						  		 										  *
***************************************************************************/
static void power_on (void)	/* Enable SSP module and attach it to I/O pads */
{
	__set_PCONP(PCSSPx, 1);	/* Enable SSP module */
	__set_PCLKSEL(PCLKSSPx, PCLKDIV_SSP);	/* Select PCLK frequency for SSP */
	SSPxCPSR = 2;			/* CPSDVSR=2 */
	SSPxCR0 = 0x0007;		/* Set mode: SPI mode 0, 8-bit */
	SSPxCR1 = 0x2;			/* Enable SSP with Master */
	ATTACH_SSP();			/* Attach SSP module to I/O pads */
	CS_HIGH();				/* Set CS# high */

	for (Timer1 = 10; Timer1; ) ;	/* 10ms */
}

/**************************************************************************
*                     power off SPI						  		 										  *
***************************************************************************/
static void power_off (void)		/* Disable SPI function */
{
	select();				/* Wait for card ready */
	deselect();
}

/*-----------------------------------------------------------------------*/
/* Receive a data packet from the MMC                                    */
/*-----------------------------------------------------------------------*/
static int rcvr_datablock (	/* 1:OK, 0:Error */
	BYTE *buff,			/* Data buffer */
	UINT btr			/* Data block length (byte) */
)
{
	BYTE token;


	Timer1 = 200;
	do {							/* Wait for DataStart token in timeout of 200ms */
		token = xchg_spi(0xFF);

		/* This loop will take a time. Insert rot_rdq() here for multitask envilonment. */

	} while ((token == 0xFF) && Timer1);
	if(token != 0xFE) return 0;		/* Function fails if invalid DataStart token or timeout */

	rcvr_spi_multi(buff, btr);		/* Store trailing data to the buffer */
	xchg_spi(0xFF); xchg_spi(0xFF);	/* Discard CRC */

	return 1;						/* Function succeeded */
}

/*-----------------------------------------------------------------------*/
/* Send a data packet to the MMC                                         */
/*-----------------------------------------------------------------------*/
#if !FF_FS_READONLY
static int xmit_datablock (	/* 1:OK, 0:Failed */
	const BYTE *buff,	/* Ponter to 512 byte data to be sent */
	BYTE token			/* Token */
)
{
	BYTE resp;


	if (!wait_ready(500)) return 0;		/* Leading busy check: Wait for card ready to accept data block */

	xchg_spi(token);					/* Send token */
	if (token == 0xFD) return 1;		/* Do not send data if token is StopTran */

	xmit_spi_multi(buff, 512);			/* Data */
	xchg_spi(0xFF); xchg_spi(0xFF);		/* Dummy CRC */

	resp = xchg_spi(0xFF);				/* Receive data resp */

	return (resp & 0x1F) == 0x05 ? 1 : 0;	/* Data was accepted or not */

	/* Busy check is done at next transmission */
}
#endif

/*-----------------------------------------------------------------------*/
/* Send a command packet to the MMC                                      */
/*-----------------------------------------------------------------------*/
static BYTE send_cmd (		/* Return value: R1 resp (bit7==1:Failed to send) */
	BYTE cmd,		/* Command index */
	DWORD arg		/* Argument */
)
{
	BYTE n, res;


	if (cmd & 0x80) {	/* Send a CMD55 prior to ACMD<n> */
		cmd &= 0x7F;
		res = send_cmd(CMD55, 0);
		if (res > 1) return res;
	}

	/* Select the card and wait for ready except to stop multiple block read */
	if (cmd != CMD12) {
		deselect();
		if (!select()) return 0xFF;
	}

	/* Send command packet */
	xchg_spi(0x40 | cmd);				/* Start + command index */
	xchg_spi((BYTE)(arg >> 24));		/* Argument[31..24] */
	xchg_spi((BYTE)(arg >> 16));		/* Argument[23..16] */
	xchg_spi((BYTE)(arg >> 8));			/* Argument[15..8] */
	xchg_spi((BYTE)arg);				/* Argument[7..0] */
	n = 0x01;							/* Dummy CRC + Stop */
	if (cmd == CMD0) n = 0x95;			/* Valid CRC for CMD0(0) */
	if (cmd == CMD8) n = 0x87;			/* Valid CRC for CMD8(0x1AA) */
	xchg_spi(n);

	/* Receive command resp */
	if (cmd == CMD12) xchg_spi(0xFF);	/* Diacard following one byte when CMD12 */
	n = 10;								/* Wait for response (10 bytes max) */
	do
		res = xchg_spi(0xFF);
	while ((res & 0x80) && --n);

	return res;							/* Return received response */
}

/*-----------------------------------------------------------------------*/
/* Virtual Drive Feature                                                 */
/*-----------------------------------------------------------------------*/
#if VIRTUAL_DISK

static FIL VD_Fil;									/* Image file */
static BYTE VD_Stat = STA_NOINIT;		/* Disk status */
static WORD VD_SectorSize;					/* Sector size */
static DWORD VD_Sectors;						/* Size of virtural disk [sectors] (0:closed) */

/**************************************************************************
*             vd init	[ virtual drive init ]				  									  *
***************************************************************************/
static DSTATUS vd_init (void)
{
	if (VD_Sectors) VD_Stat &= ~STA_NOINIT;
	return VD_Stat;
}

/**************************************************************************
*             vd stat	[ virtual drive stat ]				  									  *
***************************************************************************/
static DSTATUS vd_stat (void)
{
	return VD_Stat;
}

/**************************************************************************
*             vd read	[ virtual drive read ]				  									  *
***************************************************************************/
static DRESULT vd_read (BYTE* buff, LBA_t sector, UINT count)
{
	DRESULT res = RES_ERROR;
	UINT br;


	if (!(VD_Stat & STA_NOINIT) &&
		sector + count <= VD_Sectors &&
		f_lseek(&VD_Fil, sector * VD_SectorSize) == FR_OK &&
		f_read(&VD_Fil, buff, count * VD_SectorSize, &br) == FR_OK) {
		res = RES_OK;
	}
	return res;
}

/**************************************************************************
*             vd ioctl	[ virtual drive ioctl ]				 									  *
***************************************************************************/
static DRESULT vd_ioctl (BYTE cmd, void* buff)
{
	DRESULT res = RES_ERROR;
#if FF_MAX_SS != FF_MIN_SS
	BYTE buf[2];
	WORD ss;
	UINT rb;
#endif

	switch (cmd) {
	case GET_SECTOR_COUNT :
		if (VD_Sectors) {
			*(DWORD*)buff = VD_Sectors;
			res = RES_OK;
		}
		break;
#if FF_MAX_SS != FF_MIN_SS
	case GET_SECTOR_SIZE :
		if (VD_Sectors) {
			*(WORD*)buff = VD_SectorSize;
			res = RES_OK;
		}
		break;
#endif
	case OPEN_VIRTUAL_DISK :
		VD_Sectors = 0;
		VD_Stat = STA_NOINIT | STA_PROTECT;
		if (f_open(&VD_Fil, (const TCHAR*)buff, FA_READ) == FR_OK) {
			VD_SectorSize = 512;
#if FF_MAX_SS != FF_MIN_SS
			if (f_lseek(&VD_Fil, 11) == FR_OK && f_read(&VD_Fil, buf, 2, &rb) == FR_OK) {	/* Check BPB_BytsPerSec */
				ss = buf[0] | buf[1] << 8;
				if (!(ss & (ss - 1)) && ss >= FF_MIN_SS && ss <= FF_MAX_SS) {
					VD_SectorSize = ss;
				}
			}
#endif
			VD_Sectors = f_size(&VD_Fil) / VD_SectorSize;
			res = RES_OK;
		}
		break;
	}
	return res;
}
#endif	/* #if VIRTUAL_DISK */

/*--------------------------------------------------------------------------

   Public Functions

---------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
/* Initialize disk drive                                                 */
/*-----------------------------------------------------------------------*/
DSTATUS disk_initialize (
	BYTE pdrv		/* Physical drive number (0:SD, 1:Image file in SD) */
)
{
	BYTE n, cmd, ty, ocr[4];

#if VIRTUAL_DISK
	if (pdrv == VIRTUAL_DISK) return vd_init();
#endif
	if (pdrv != 0) return STA_NOINIT;	/* Supports only drive 0 */

	power_on();							/* Initialize memory card interface */
	
	if (Stat & STA_NODISK) return Stat;	/* Is a card existing in the soket? */
	FCLK_SLOW();
	for (n = 10; n; n--) xchg_spi(0xFF);	/* Send 80 dummy clocks */

	ty = 0;
	if (send_cmd(CMD0, 0) == 1) {			/* Put the card SPI state */
		Timer1 = 1000;						/* Initialization timeout = 1 sec */
		if (send_cmd(CMD8, 0x1AA) == 1) {	/* Is the catd SDv2? */
			for (n = 0; n < 4; n++) ocr[n] = xchg_spi(0xFF);	/* Get 32 bit return value of R7 resp */
			if (ocr[2] == 0x01 && ocr[3] == 0xAA) {				/* Does the card support 2.7-3.6V? */
				while (Timer1 && send_cmd(ACMD41, 1UL << 30)) ;	/* Wait for end of initialization with ACMD41(HCS) */
				if (Timer1 && send_cmd(CMD58, 0) == 0) {		/* Check CCS bit in the OCR */
					for (n = 0; n < 4; n++) ocr[n] = xchg_spi(0xFF);
					ty = (ocr[0] & 0x40) ? CT_SDC2 | CT_BLOCK : CT_SDC2;	/* Check if the card is SDv2 */
				}
			}
		} else {	/* Not an SDv2 card */
			if (send_cmd(ACMD41, 0) <= 1) 	{	/* SDv1 or MMCv3? */
				ty = CT_SDC1; cmd = ACMD41;	/* SDv1 (ACMD41(0)) */
			} else {
				ty = CT_MMC3; cmd = CMD1;	/* MMCv3 (CMD1(0)) */
			}
			while (Timer1 && send_cmd(cmd, 0)) ;		/* Wait for the card leaves idle state */
			if (!Timer1 || send_cmd(CMD16, 512) != 0)	/* Set block length: 512 */
				ty = 0;
		}
	}
	CardType = ty;	/* Card type */
	deselect();

	if (ty) {		/* OK */
		FCLK_FAST();			/* Set fast clock */
		Stat &= ~STA_NOINIT;	/* Clear STA_NOINIT flag */
	} else {		/* Failed */
		power_off();
		Stat = STA_NOINIT;
	}

	return Stat;
}

/*-----------------------------------------------------------------------*/
/* Get disk status                                                       */
/*-----------------------------------------------------------------------*/
DSTATUS disk_status (
	BYTE pdrv		/* Physical drive number (0) */
)
{
#if VIRTUAL_DISK
	if (pdrv == VIRTUAL_DISK) return vd_stat();
#endif
	if (pdrv != 0) return STA_NOINIT;	/* Supports only drive 0 */

	return Stat;	/* Return disk status */
}

/*-----------------------------------------------------------------------*/
/* Read sector(s)                                                        */
/*-----------------------------------------------------------------------*/
DRESULT disk_read (
	BYTE pdrv,		/* Physical drive number (0) */
	BYTE *buff,		/* Pointer to the data buffer to store read data */
	LBA_t sector,	/* Start sector number (LBA) */
	UINT count		/* Number of sectors to read (1..128) */
)
{
	BYTE cmd;
	DWORD sect = (DWORD)sector;

#if VIRTUAL_DISK
	if (pdrv == VIRTUAL_DISK) return vd_read(buff, sector, count);
#endif
	if (pdrv != 0 || count == 0) return RES_PARERR;		/* Check parameter */
	if (Stat & STA_NOINIT) return RES_NOTRDY;	/* Check if drive is ready */
	if (!(CardType & CT_BLOCK)) sect *= 512;	/* LBA ot BA conversion (byte addressing cards) */

	cmd = count > 1 ? CMD18 : CMD17;			/*  READ_MULTIPLE_BLOCK : READ_SINGLE_BLOCK */
	if (send_cmd(cmd, sect) == 0) {
		do {
			if (!rcvr_datablock(buff, 512)) break;
			buff += 512;
		} while (--count);
		if (cmd == CMD18) send_cmd(CMD12, 0);	/* STOP_TRANSMISSION */
	}
	deselect();

	return count ? RES_ERROR : RES_OK;	/* Return result */
}

/*-----------------------------------------------------------------------*/
/* Write sector(s)                                                       */
/*-----------------------------------------------------------------------*/
#if !FF_FS_READONLY
DRESULT disk_write (
	BYTE pdrv,			/* Physical drive number (0) */
	const BYTE *buff,	/* Ponter to the data to write */
	LBA_t sector,		/* Start sector number (LBA) */
	UINT count			/* Number of sectors to write (1..128) */
)
{
	DWORD sect = (DWORD)sector;


	if (pdrv != 0 || count == 0) return RES_PARERR;		/* Check parameter */
	if (Stat & STA_NOINIT) return RES_NOTRDY;	/* Check drive status */
	if (Stat & STA_PROTECT) return RES_WRPRT;	/* Check write protect */

	if (!(CardType & CT_BLOCK)) sect *= 512;	/* LBA ==> BA conversion (byte addressing cards) */

	if (count == 1) {	/* Single sector write */
		if ((send_cmd(CMD24, sect) == 0)	/* WRITE_BLOCK */
			&& xmit_datablock(buff, 0xFE)) {
			count = 0;
		}
	}
	else {				/* Multiple sector write */
		if (CardType & CT_SDC) send_cmd(ACMD23, count);	/* Predefine number of sectors */
		if (send_cmd(CMD25, sect) == 0) {	/* WRITE_MULTIPLE_BLOCK */
			do {
				if (!xmit_datablock(buff, 0xFC)) break;
				buff += 512;
			} while (--count);
			if (!xmit_datablock(0, 0xFD)) count = 1;	/* STOP_TRAN token */
		}
	}
	deselect();

	return count ? RES_ERROR : RES_OK;	/* Return result */
}
#endif

/*-----------------------------------------------------------------------*/
/* Miscellaneous drive controls other than data read/write               */
/*-----------------------------------------------------------------------*/
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive number (0) */
	BYTE cmd,		/* Control command code */
	void *buff		/* Pointer to the conrtol data */
)
{
	DRESULT res = RES_ERROR;
	BYTE n, csd[16], *ptr = buff;
	DWORD st, ed, csize;
	LBA_t *dp;
#if DISKIO_ISDIO
	SDIO_CMD *sdio = buff;
	BYTE rc, *buf;
	UINT dc;
#endif


#if VIRTUAL_DISK
	if (pdrv == VIRTUAL_DISK) return vd_ioctl(cmd, buff);
#endif
	if (pdrv != 0) return RES_PARERR;			/* Check parameter */
	if (Stat & STA_NOINIT) return RES_NOTRDY;	/* Check if drive is ready */

	switch (cmd) {
	case CTRL_SYNC:			/* Wait for end of internal write process of the drive */
		if (select()) res = RES_OK;
		break;

	case GET_SECTOR_COUNT:	/* Get drive capacity in unit of sector (DWORD) */
		if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {
			if ((csd[0] >> 6) == 1) {	/* SDC ver 2.00 */
				csize = csd[9] + ((WORD)csd[8] << 8) + ((DWORD)(csd[7] & 63) << 16) + 1;
				*(LBA_t*)buff = csize << 10;
			} else {					/* SDC ver 1.XX or MMC ver 3 */
				n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
				csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
				*(LBA_t*)buff = csize << (n - 9);
			}
			res = RES_OK;
		}
		break;
#if FF_MAX_SS != FF_MIN_SS
	case GET_SECTOR_SIZE :
		*(WORD*)buff = 512;
		res = RES_OK;
		break;
#endif
	case GET_BLOCK_SIZE:	/* Get erase block size in unit of sector (DWORD) */
		if (CardType & CT_SDC2) {	/* SDC ver 2.00 */
			if (send_cmd(ACMD13, 0) == 0) {	/* Read SD status */
				xchg_spi(0xFF);
				if (rcvr_datablock(csd, 16)) {				/* Read partial block */
					for (n = 64 - 16; n; n--) xchg_spi(0xFF);	/* Purge trailing data */
					*(DWORD*)buff = 16UL << (csd[10] >> 4);
					res = RES_OK;
				}
			}
		} else {					/* SDC ver 1.XX or MMC */
			if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {	/* Read CSD */
				if (CardType & CT_SDC1) {	/* SDC ver 1.XX */
					*(DWORD*)buff = (((csd[10] & 63) << 1) + ((WORD)(csd[11] & 128) >> 7) + 1) << ((csd[13] >> 6) - 1);
				} else {					/* MMC */
					*(DWORD*)buff = ((WORD)((csd[10] & 124) >> 2) + 1) * (((csd[11] & 3) << 3) + ((csd[11] & 224) >> 5) + 1);
				}
				res = RES_OK;
			}
		}
		break;

	case CTRL_TRIM:		/* Erase a block of sectors (used when _USE_TRIM in ffconf.h is 1) */
		if (!(CardType & CT_SDC)) break;				/* Check if the card is SDC */
		if (disk_ioctl(pdrv, MMC_GET_CSD, csd)) break;	/* Get CSD */
		if (!(csd[10] & 0x40)) break;					/* Check if ERASE_BLK_EN = 1 */
		dp = buff; st = (DWORD)dp[0]; ed = (DWORD)dp[1];	/* Load sector block */
		if (!(CardType & CT_BLOCK)) {
			st *= 512; ed *= 512;
		}
		if (send_cmd(CMD32, st) == 0 && send_cmd(CMD33, ed) == 0 && send_cmd(CMD38, 0) == 0 && wait_ready(30000)) {	/* Erase sector block */
			res = RES_OK;	/* FatFs does not check result of this command */
		}
		break;

	/* Following commands are never used by FatFs module */

	case MMC_GET_TYPE:		/* Get MMC/SDC type (BYTE) */
		*ptr = CardType;
		res = RES_OK;
		break;

	case MMC_GET_CSD:		/* Read CSD (16 bytes) */
		if (send_cmd(CMD9, 0) == 0 && rcvr_datablock(ptr, 16)) {	/* READ_CSD */
			res = RES_OK;
		}
		break;

	case MMC_GET_CID:		/* Read CID (16 bytes) */
		if (send_cmd(CMD10, 0) == 0 && rcvr_datablock(ptr, 16)) {	/* READ_CID */
			res = RES_OK;
		}
		break;

	case MMC_GET_OCR:		/* Read OCR (4 bytes) */
		if (send_cmd(CMD58, 0) == 0) {	/* READ_OCR */
			for (n = 4; n; n--) *ptr++ = xchg_spi(0xFF);
			res = RES_OK;
		}
		break;

	case MMC_GET_SDSTAT:	/* Read SD status (64 bytes) */
		if (send_cmd(ACMD13, 0) == 0) {	/* SD_STATUS */
			xchg_spi(0xFF);
			if (rcvr_datablock(ptr, 64)) res = RES_OK;
		}
		break;
#if DISKIO_ISDIO
	case ISDIO_READ:
		sdio = buff;
		if (send_cmd(CMD48, 0x80000000 | sdio->func << 28 | sdio->addr << 9 | ((sdio->ndata - 1) & 0x1FF)) == 0) {
			for (Timer1 = 1000; (rc = xchg_spi(0xFF)) == 0xFF && Timer1; ) ;
			if (rc == 0xFE) {
				for (buf = sdio->data, dc = sdio->ndata; dc; dc--) *buf++ = xchg_spi(0xFF);
				for (dc = 514 - sdio->ndata; dc; dc--) xchg_spi(0xFF);
				res = RES_OK;
			}
		}
		break;
	case ISDIO_WRITE:
		sdio = buff;
		if (send_cmd(CMD49, 0x80000000 | sdio->func << 28 | sdio->addr << 9 | ((sdio->ndata - 1) & 0x1FF)) == 0) {
			xchg_spi(0xFF); xchg_spi(0xFE);
			for (buf = sdio->data, dc = sdio->ndata; dc; dc--) xchg_spi(*buf++);
			for (dc = 514 - sdio->ndata; dc; dc--) xchg_spi(0xFF);
			if ((xchg_spi(0xFF) & 0x1F) == 0x05) res = RES_OK;
		}
		break;
	case ISDIO_MRITE:
		sdio = buff;
		if (send_cmd(CMD49, 0x84000000 | sdio->func << 28 | sdio->addr << 9 | sdio->ndata >> 8) == 0) {
			xchg_spi(0xFF); xchg_spi(0xFE);
			xchg_spi(sdio->ndata);
			for (dc = 513; dc; dc--) xchg_spi(0xFF);
			if ((xchg_spi(0xFF) & 0x1F) == 0x05) res = RES_OK;
		}
		break;
#endif
	default:
		res = RES_PARERR;
	}

	deselect();
	return res;
}

/*-----------------------------------------------------------------------*/
/* Device timer function                                                 */
/*-----------------------------------------------------------------------*/
/* This function must be called from timer interrupt routine in period
 * of 1 ms to generate card control timing.
 */

void disk_timerproc (void)
{
	WORD n;
	BYTE s;


	n = Timer1;						/* 1kHz decrement timer stopped at 0 */
	if (n) Timer1 = --n;
	n = Timer2;
	if (n) Timer2 = --n;

	s = Stat;
	if (MMC_WP) {	/* Write protected */
		s |= STA_PROTECT;
	} else {		/* Write enabled */
		s &= ~STA_PROTECT;
	}
	if (MMC_CD) {	/* Card is in socket */
		s &= ~STA_NODISK;
	} else {		/* Socket empty */
		s |= (STA_NODISK | STA_NOINIT);
	}
	Stat = s;
}

/**************************************************************************
*             f Create Folder	[ Creat the folder in SDCard ]						  *
***************************************************************************/
int f_CreateFolder(	
	const char* path,
	const char* Parameters
)
{
 
	FRESULT fr;
	FATFS fat;	   /* fileConfig (FAT) structure declare */
 
	if((fr = f_mount(&fat,"",1))!= FR_OK) {
		console_log("File mouting failed, FileName: %s -- ErrorStatus: %d \n\r",path,fr);
		return fr;
	}
	// Create folder/directory
	fr = f_mkdir(path);
	if(fr == FR_OK){
 
		// Success
		// TODO: f_mkdir Something to happen when successful
		console_log("Directory Name: %s\n\r",path);
 
	}
	else{
 
		// Error
		// TODO: f_CreateFolder make directory error
 
	}
 
	return fr;
} // END


/*-----------------------------------------------------------------------*/
/* Miscellaneous sdc card function controls other than data read/write	 */
/*-----------------------------------------------------------------------*/

/**************************************************************************
*             create file	[ Creat the file in SDCard ]	      					  *
***************************************************************************/
bool create_file()
{  
		rtc_t rtc;
		
		/* Get RTC */
		RTC_GetDateTime(&rtc);

		n = sprintf(fileNme,"%02d%02d%d",(uint16_t)rtc.date,(uint16_t)rtc.month,(uint16_t)rtc.year);

    ff_result = f_stat(fileNme, &fno);
    switch (ff_result) 
    {
        case FR_OK:
              ff_result = f_open(&file,fileNme, FA_READ | FA_WRITE | FA_OPEN_APPEND);
              if (ff_result != FR_OK)
              {
                  console_log("Unable to open or create file: %s\n\r",fileNme);
                  fault = true;
                  return fault;
              }
              sd_append_file();
            break;

        case FR_NO_FILE:
            ff_result = f_open(&file,fileNme, FA_READ | FA_WRITE | FA_OPEN_APPEND);
            if (ff_result != FR_OK)
            {
                console_log("Unable to open or create file: %s\n\r",fileNme);
                fault = true;
                return fault;
            }
            if (ff_result == FR_OK)
            {
                console_log("File Created: %s\n\r",fileNme);
            }
            
            sd_append_file();
            break;
            
        default:
            console_log("An error occured. (%d)\n\r", ff_result);
            fault = true;
            return fault;
    }
}

/**************************************************************************
*             create folder	[ Creat the folder in SDCard ]	   					  *
***************************************************************************/
bool create_folder()
{
//  get_time_date();  
    rtc_t rtc;
	
		/* Get RTC */
		RTC_GetDateTime(&rtc);
    memset(foldername,0,sizeof(foldername));
    n = sprintf(foldername,"%02d%c%d",(uint16_t)rtc.month,0x2D,(uint16_t)rtc.year);
	
		console_log("Mounting volume...\n\r");
		ff_result = f_mount(&fs,"",1);
    if (ff_result) {
        console_log("Mount failed.\n\r");
        return ff_result;
    }
		
    ff_result = f_stat(foldername, &fno);
   
    switch (ff_result) 
    {
        case FR_OK:
								console_log("FOLDER exist..\n\r");
								f_chdir(foldername);               // go to new created folder directory
                create_file();
            break;

        case FR_NO_FILE:                        // if folder not exists 
          
            //f_chdir("/");                       // then go to root directory
          
            ff_result = f_mkdir(foldername);    // create the folder
            if (ff_result != FR_OK) {
              console_log("Directory not created\n\r.");
              fault = true;
              return fault;
            }
            else
              console_log("create DIR successful = %s\n\r",foldername);
            
       //     strcat(foldername1,foldername) ;           
       //     f_chdir(foldername1);            // go to new created folder directory
						f_chdir(foldername);               // go to new created folder directory
            create_file();
            break;

        default:
            console_log("An error occured. (%d)\n\r", ff_result);
            fault = true;
						return fault;
    }    
}

/**************************************************************************
*             sd append file	[ write the content in the file ]	   				*
***************************************************************************/
bool sd_append_file()
{
//		rtc_t rtc;
		
    ff_result = f_lseek(&file, f_size(&file));		/* Seek to end of the file to append data */
    if (ff_result != FR_OK)
           f_close(&file);

		f_puts(_buffer,&file);
		f_puts("\n",&file);
		
    if (ff_result != FR_OK) {
			console_log("Write failed.\n\r");
      fault = true;
      return fault;
    }
    else
		{
      console_log("Write Successful...\n\r");
			fault = false;
		}

    f_close(&file);
		f_unmount(""); /* Unregister a work area before discard it */
			return fault;
}

/**************************************************************************
*             get name	[ get the file name from SDcrad ]      	   				*
***************************************************************************/
void get_name()
{
    uint8_t ii=0;
    unsigned char data = 0;
	memset(fileName,0,sizeof(fileName));
    
    
    console_log("Enter the name of file/Directory...\n\r");
    
    while(1)
    {
        data = UART0_RxChar();
      //  transmitByte(data);
        
        if(data == 0x0d) 
        {
            (fileName[ii]) = '\0';
            break;  //'ENTER' key pressed
        }
        
        (fileName[ii]) = data;
        ii++; 
    }
    console_log("entered string is = %s\n\r",fileName);
}

/**************************************************************************
*             get folder name	[ get the folder name from SDcrad ]  				*
***************************************************************************/
void get_foldername()
{
   
    unsigned char data = 0;
    uint8_t ii=0;
    memset(foldername,0,sizeof(foldername));

    console_log("Enter the name of file/Directory...\n\r");
    
    while(1)
    {
        data = UART0_RxChar();
      //  transmitByte(data);
        
        if(data == 0x0d) 
        {
            (foldername[ii]) = '\0';
            break;  //'ENTER' key pressed
        }
        
        (foldername[ii]) = data;
        ii++; 
    }
    console_log("entered string is = %s\n\r",foldername);
}


/**************************************************************************
*             write data	[ write data in the file ]          	   				*
***************************************************************************/
void write_data()
{
    
    unsigned char data = 0;
    uint8_t ii=0;
    memset(fileName,0,sizeof(fileName));

    console_log("Enter the name of file/Directory...\n\r");
    
    while(1)
    {
        data = UART0_RxChar();
      //  transmitByte(data);
        
        if(data == 0x0d) 
        {
            (fileName[ii]) = '\0';
            break;  //'ENTER' key pressed
        }
        
        (fileName[ii]) = data;
        ii++; 
    }
    console_log("entered string is = %s\n\r",fileName);
}

/**************************************************************************
*             get details	[ get details from the file ]        	   				*
***************************************************************************/
void  getdetails()
{
    TCHAR temp_dir[13] = {0};
		
		ff_result = f_mount(&fs,"",1);
    if (ff_result)
    {
        console_log("Mount failed.\n\r");
        return;
    }
    
    get_name();
    if(fileName[0]!= '/')
    {
        memcpy(&temp_dir[1],&fileName[0],(sizeof(fileName)-1));
        temp_dir[0] = '/';
    }
    else
    {
        memcpy(temp_dir,fileName,sizeof(fileName));
    }
    
    console_log("\n\rOpening directory: \n\r");
    ff_result = f_opendir(&dir, temp_dir);
    if (ff_result != FR_OK)
    {
        console_log("Unable to Open DIR\n\r");
        return;
    }
    do
    {
        ff_result = f_readdir(&dir, &fno);
        if (ff_result != FR_OK)
        {
            console_log("Directory read failed.\n\r");
            return;
        }
        
        if (fno.fname[0])
        {
            if (fno.fattrib & AM_DIR)
            {
                console_log("   <DIR>   %s\n\r",fno.fname);
            }
            else
            {
							console_log("File Size: %u  File Name: %s\n\r", fno.fsize, fno.fname);
            }
        }
    }
    while (fno.fname[0]);
    
    ff_result = f_closedir (&dir);
		f_unmount(""); // Unregister a work area before discard it
    if (ff_result != FR_OK)
    {
        console_log("Unable to Close DIR\n\r");
        return;
    }
    
    
}

/**************************************************************************
*             set_date_time	[ set date and time for the file sytem ]			*
***************************************************************************/
void set_date_time()
{
    uint8_t val[7],mul_fact=0x64;
    //char buffer [12]={0};
    char temp[3]={0};//temp2=0;
    
    TCHAR fileName[13]={0};
    unsigned char data;
    uint8_t ii=0;
	rtc_t rtc;

	console_log("Enter the date and time in ddmmyyyyhhmmss format\n\r");
    
    while(1)
    {
				data = UART0_RxChar();
       // transmitByte(data);
        
        if(data == 0x0d) 
        {
            (fileName[ii]) = '\0';
            break;  //'ENTER' key pressed
        }
        
        (fileName[ii]) = data;
        ii++; 
    }
    console_log("entered string is = %s\n\r",fileName);
        
    for(ii=0;ii<14;ii+=2)
    {
        temp[0] = fileName[ii];
        temp[1] = fileName[ii+1];
				temp[2] = '\0';
        val[ii/2] =(ascii_integer(temp));
    }
		
    console_log("%x %x %x %x %x %x %x\n\r",val[0],val[1],val[2],val[3],val[4],val[5],val[6]);
    
    rtc.date 	= val[0];
    rtc.month = val[1];
    rtc.year 	= (val[2] * mul_fact) + val[3];
    rtc.hour	= val[4];
    rtc.min   = val[5];
    rtc.sec   = val[6];
    
    console_log("START DATE\r\nDATE =  %d-%d-%d TIME = %d:%d:%d\r\n",rtc.date,rtc.month,rtc.year,rtc.hour,rtc.min,rtc.sec);
		/*##### Set the time and Date only once. Once the Time and Date is set, comment these lines
         and reflash the code. Else the time will be set every time the controller is reset*/
    RTC_SetDateTime(&rtc);  //  10:40:20 am, 1st Jan 2016
}

/**************************************************************************
*             sd file size	[ get sd file size  ]		                    	*
***************************************************************************/
void sd_filesize()
{
    FATFS *fs1;
    DWORD fre_clust, fre_sect, tot_sect;
    
    
    /* Get volume information and free clusters of drive 1 */
    ff_result = f_getfree("", &fre_clust, &fs1);
    if (ff_result != FR_OK)
    {
        console_log("Unable to get volume information...\n\r");
    }
    
    /* Get total sectors and free sectors */
    tot_sect = (fs1->n_fatent - 2) * fs1->csize;
    fre_sect = fre_clust * fs1->csize;

    /* Print the free space (assuming 512 bytes/sector) */
    console_log("%u KiB total drive space.\n\r%u KiB available.\n\r", tot_sect / 2, fre_sect / 2);

}

/**************************************************************************
*             sd file size	[ get sd file size  ]		                    	*
***************************************************************************/
void sd_create_file()
{    
    get_name();   
    
		ff_result = f_mount(&fs,"",1);
    if (ff_result)
    {
        console_log("Mount failed.\n\r");
        return;
    }
    ff_result = f_open(&file,fileName, FA_READ | FA_WRITE | FA_OPEN_APPEND);
		//ff_result = open_append(&file,fileName);
    if (ff_result != FR_OK)
    {
        console_log("Unable to open or create file: %s\n\r",fileName);
        return;
    }
    if (ff_result == FR_OK)
    {
        console_log("File Created: %s\n\r",fileName);
	//			f_puts("Hello World !!",&file);
	//			f_puts("\n",&file);
				f_close(&file);	
				f_unmount(""); // Unregister a work area before discard it
    }
}

/**************************************************************************
*             sd_file_remove	[ remove file from SD card ]	             	*
***************************************************************************/
void sd_file_remove()
{
    get_name();
  
    //remove perticular file 
    ff_result = f_unlink (fileName);
    if (ff_result != FR_OK)
    {
      console_log("NoFile : Write failed\n\r.");
    }
    else
    {
      console_log("file removed = %s \n\r",fileName);
    }
}

/**************************************************************************
*             sd_create_dir	[ create directory in SDCard ]	             	*
***************************************************************************/
void sd_create_dir()
{
    get_name();                 //Get DIR name from USER
    
		ff_result = f_mount(&fs,"",1);
    if (ff_result)
    {
        console_log("Mount failed.\n\r");
        return;
    }
		
    ff_result = f_mkdir(fileName);
    if (ff_result != FR_OK)
    {
      console_log("Directory not created\n\r.");
    }
    else
    {
      console_log("create DIR successful = %s\n\r",fileName);
    }
}

/**************************************************************************
*             set_date_time	[ set date and time for the file sytem ]			*
***************************************************************************/
void sd_time_date()
{  
    uint8_t val[3],buff[5]={0},i;
    char buffer [12]={0};
    char temp[3]={0};//temp2=0;
    
    TCHAR fileName[13]={0};
    unsigned char data;
    uint8_t ii=0;
    console_log("Enter current date and time in DDMMYY and HHMMSS\n\r");

    while(1)
    {
        data = UART0_RxChar();
      //  transmitByte(data);
        
        if(data == 0x0d) 
        {
            (fileName[ii]) = '\0';
            break;  //'ENTER' key pressed
        }
        
        (fileName[ii]) = data;
        ii++; 
    }
    console_log("entered string is = %s\n\r",fileName);
    
    for(i=0;i<6;i+=2)
    {
        temp[0] = fileName[i];
        temp[1] = fileName[i+1];
				temp[2] = '\0';
        val[i/2] =(ascii_integer(temp));
    }
    
    console_log("%x %x %x \n\r",val[0],val[1],val[2]);
    
    buff[0] = val[0];
    buff[1] = 0x2D;
    buff[2] = val[1];
    buff[3] = 0x2D;
    buff[4] = val[2];
    
    n=sprintf (buffer,"%d%c%d%c%d",buff[0],buff[1],buff[2],buff[3],buff[4]);
   
    
    console_log("Timestamp: %u/%02u/%02u, %02u:%02u\n\r",
    (fno.fdate >> 9) + 1980, fno.fdate >> 5 & 15, fno.fdate & 31,
    fno.ftime >> 11, fno.ftime >> 5 & 63);

    console_log("Attributes: %c%c%c%c%c\n\r",
    (fno.fattrib & AM_DIR) ? 'D' : '-',
    (fno.fattrib & AM_RDO) ? 'R' : '-',
    (fno.fattrib & AM_HID) ? 'H' : '-',
    (fno.fattrib & AM_SYS) ? 'S' : '-',
    (fno.fattrib & AM_ARC) ? 'A' : '-');
    
    ff_result = f_mkdir(buffer);
    if (ff_result != FR_OK)
    {
      console_log("Directory not created\n\r.");
    }
    else
    {
        console_log("create DIR successful = %s\n\r",buffer);
    }
}

/**************************************************************************
*             sd_format	[ Format the SD card work area ]	             		*
***************************************************************************/
void sd_format()
{
    BYTE work[FF_MAX_SS]; /* Work area (larger is better for processing time) */
    
    /* Create FAT volume */
    ff_result = f_mkfs("", 0, work, sizeof work);
    if (ff_result != FR_OK)
    {
        console_log("Unable to format Disk\r\n");
        return;
    }
    if (ff_result == FR_OK)
    {
        console_log("Format Complete...\r\n");
        return;
    }
}

/**************************************************************************
*             xatoi	[ String to integer  ]              	             		*
***************************************************************************/
int xatoi (			/* 0:Failed, 1:Successful */
	char **str,		/* Pointer to pointer to the string */
	long *res		/* Pointer to the valiable to store the value */
)
{
	unsigned long val;
	unsigned char c, r, s = 0;


	*res = 0;

	while ((c = **str) == ' ') (*str)++;	/* Skip leading spaces */

	if (c == '-') {		/* negative? */
		s = 1;
		c = *(++(*str));
	}

	if (c == '0') {
		c = *(++(*str));
		switch (c) {
		case 'x':		/* hexdecimal */
			r = 16; c = *(++(*str));
			break;
		case 'b':		/* binary */
			r = 2; c = *(++(*str));
			break;
		default:
			if (c <= ' ') return 1;	/* single zero */
			if (c < '0' || c > '9') return 0;	/* invalid char */
			r = 8;		/* octal */
		}
	} else {
		if (c < '0' || c > '9') return 0;	/* EOL or invalid char */
		r = 10;			/* decimal */
	}

	val = 0;
	while (c > ' ') {
		if (c >= 'a') c -= 0x20;
		c -= '0';
		if (c >= 17) {
			c -= 7;
			if (c <= 9) return 0;	/* invalid char */
		}
		if (c >= r) return 0;		/* invalid char for current radix */
		val = val * r + c;
		c = *(++(*str));
	}
	if (s) val = 0 - val;			/* apply sign if needed */

	*res = val;
	return 1;
}

/**************************************************************************
*             put_rc	[ Print Error respective code for FatFs system  ] 	*
***************************************************************************/
static void put_rc (FRESULT rc)
{
	const char *str =
		"OK\0" "DISK_ERR\0" "INT_ERR\0" "NOT_READY\0" "NO_FILE\0" "NO_PATH\0"
		"INVALID_NAME\0" "DENIED\0" "EXIST\0" "INVALID_OBJECT\0" "WRITE_PROTECTED\0"
		"INVALID_DRIVE\0" "NOT_ENABLED\0" "NO_FILE_SYSTEM\0" "MKFS_ABORTED\0" "TIMEOUT\0"
		"LOCKED\0" "NOT_ENOUGH_CORE\0" "TOO_MANY_OPEN_FILES\0" "INVALID_PARAMETER\0";
	FRESULT i;

	for (i = 0; i != rc && *str; i++) {
		while (*str++) ;
	}
	console_log("rc=%u FR_%s\n\r", (UINT)rc, str);
}

/**************************************************************************
*      show_volume_stat	[ fs [<path>] - Show volume status  ]           	*
***************************************************************************/
void show_volume_stat(char *ptr)
{
	long p1, p2;
	FRESULT res;
	DWORD ncl, acc_dirs, acc_files;
	DWORD acc_size;
	WORD ss;
	FATFS *fs;
	static const char *ft[] = {"", "FAT12", "FAT16", "FAT32", "exFAT"};
	
	ff_result = f_mount(&FatFs[p1],"",1);
  if (ff_result)
  {
      console_log("Mount failed.\n\r");
      return;
  }
	while (*ptr == ' ') ptr++;
	res = f_getfree(ptr, &ncl, &fs);
	if (res) { put_rc(res); return;}
#if FF_MIN_SS != FF_MAX_SS
							ss = fs->ssize;
#else
							ss = FF_MAX_SS;
#endif
		console_log("FAT type = %s\n\r", ft[fs->fs_type]);
		console_log("Bytes/Cluster = %U\n\r", (DWORD)fs->csize * ss);
		console_log("Number of FATs = %u\n\r", fs->n_fats);
		if (fs->fs_type < FS_FAT32) console_log("Root DIR entries = %u\n", fs->n_rootdir);
		console_log("Sectors/FAT = %U\n\r", fs->fsize);
		console_log("Number of clusters = %U\n\r", (DWORD)fs->n_fatent - 2);
		console_log("Volume start (lba) = %U\n\r", fs->volbase);
		console_log("FAT start (lba) = %U\n\r", fs->fatbase);
		console_log("DIR start (lba,clustor) = %U\n\r", fs->dirbase);
		console_log("Data start (lba) = %U\n\r\n\r", fs->database);
#if FF_USE_LABEL
		res = f_getlabel(ptr, (char*)Buff, (DWORD*)&p2);
		if (res) { put_rc(res); return; }
		console_log(Buff[0] ? "Volume name is %s\n\r" : "No volume label\n\r", (char*)Buff);
		console_log("Volume S/N is %04X-%04X\n\r", (DWORD)p2 >> 16, (DWORD)p2 & 0xFFFF);
#endif
		acc_files = acc_size = acc_dirs = 0;
		console_log("...");
		res = scan_files(ptr, &acc_dirs, &acc_files, &acc_size);
		if (res) { put_rc(res); return; }
		console_log("\r%u files, %U bytes.\n\r%u folders.\n\r"
								"%U KiB total disk space.\n\r%U KiB available.\n\r",
								acc_files, acc_size, acc_dirs,
								(fs->n_fatent - 2) * (DWORD)((DWORD)fs->csize * ss / 1024), ncl * (DWORD)((DWORD)fs->csize * ss / 1024)
		);
		f_unmount(""); // Unregister a work area before discard it
}

/*--------------------------------------------------------------------------*/
/* Monitor                                                                  */
/*--------------------------------------------------------------------------*/
/**************************************************************************
*      scan_files	[ Scan files ]                                        	*
***************************************************************************/
static FRESULT scan_files (
	char* path,		/* Pointer to the path name working buffer */
	DWORD* n_dir,
	DWORD* n_file,
	DWORD* sz_file
)
{
	DIR dirs;
	FRESULT res;
	BYTE i;


	if ((res = f_opendir(&dirs, path)) == FR_OK) {
		while (((res = f_readdir(&dirs, &Finfo)) == FR_OK) && Finfo.fname[0]) {
			if (Finfo.fattrib & AM_DIR) {
				(*n_dir)++;
				i = strlen(path);
				path[i] = '/'; strcpy(&path[i+1], Finfo.fname);
				res = scan_files(path, n_dir, n_file, sz_file);
				path[i] = '\0';
				if (res != FR_OK) break;
			} else {
			/*	xprintf("%s/%s\n", path, fn); */
				(*n_file)++;
				*sz_file += Finfo.fsize;
			}
		}
	}

	return res;
}

/**************************************************************************
*      dir_list	[ fl [<path>] - directory listing  ]                     	*
***************************************************************************/
void dir_list(char *ptr)
{
	FRESULT res;
	DWORD dw, acc_dirs, acc_files;
	DWORD acc_size;
	FATFS *fs1;
	
	ff_result = f_mount(&fs,"",1);
  if (ff_result)
  {
      console_log("Mount failed.\n\r");
      return;
  }
	while (*ptr == ' ') ptr++;
	res = f_opendir(&dir, ptr);
	if (res) { put_rc(res); return; }
	acc_size = acc_dirs = acc_files = 0;
	for(;;) {
			res = f_readdir(&dir, &Finfo);
			if ((res != FR_OK) || !Finfo.fname[0]) break;
			if (Finfo.fattrib & AM_DIR) {
				acc_dirs++;
			} else {
				acc_files++; acc_size += Finfo.fsize;
			}
			console_log("%c%c%c%c%c %u/%02u/%02u %02u:%02u %9u  %s\n\r",
										(Finfo.fattrib & AM_DIR) ? 'D' : '-',
										(Finfo.fattrib & AM_RDO) ? 'R' : '-',
										(Finfo.fattrib & AM_HID) ? 'H' : '-',
										(Finfo.fattrib & AM_SYS) ? 'S' : '-',
										(Finfo.fattrib & AM_ARC) ? 'A' : '-',
										(Finfo.fdate >> 9) + 1980, (Finfo.fdate >> 5) & 15, Finfo.fdate & 31,
										(Finfo.ftime >> 11), (Finfo.ftime >> 5) & 63,
										Finfo.fsize, Finfo.fname);
	}
	console_log("%u File(s),%u bytes total\n\r%u Dir(s)", acc_files, acc_size, acc_dirs);
	res = f_getfree(ptr, &dw, &fs1);
	if (res == FR_OK) {
				console_log(", %U bytes free\n\r", (DWORD)dw * fs.csize * 512);
	} else {
		put_rc(res);
	}
	f_unmount(""); // Unregister a work area before discard it
}

/**************************************************************************
*      Writelog	[  log are write in floder -> file of SDcard  ]          	*
***************************************************************************/
bool Writelog(
	void/* A log to the file */
)
{ 
	int res;
	SysTick_Start();
	res = create_folder();
	SysTick_Stop();
	return res;
}

/**************************************************************************
*      Createlog	[  log are write in file of SDcard  ]                  	*
***************************************************************************/
int Createlog ( 	
	const char* Parameters,  /* A log to the file */
	const char* path   /* Pointer to the file object */)
{
//    n = sizeof(Parameters);
		SysTick_Start();
		ff_result = f_mount(&fs,"",1);
    if (ff_result)
    {
        console_log("Mount failed.\n\r");
        return ff_result;
    }
		
    ff_result = f_open(&file, path, FA_WRITE | FA_OPEN_ALWAYS);
    if (ff_result == FR_OK) {
        /* Seek to end of the file to append data */
        ff_result = f_lseek(&file, f_size(&file));
        if (ff_result != FR_OK)
            f_close(&file);
    }
    
		//ff_result = f_write(&file, Parameters, n, (UINT *) &bytes_written);
		f_puts(Parameters,&file);
		f_puts("\n",&file);
    if (ff_result != FR_OK)
    {
        console_log("Write failed.\n\r");
        return ff_result;
    }
    else
    {
        console_log("Write Successful...\n\r");
    } 
    
    /* Append a line */
//    f_printf(&file, "%02u/%02u/%u, %2u:%02u\r\n");

    /* Close the file */
    f_close(&file);
		f_unmount(""); // Unregister a work area before discard it
		SysTick_Stop();
		return ff_result;
}

/*-----------------------------------------------------------------------*/
/* send the pending logs to the server                                   */
/*-----------------------------------------------------------------------*/
int retrylog( void )
{
	static FIL fsrc, fdst;		// file objects
  int res;					// FatFs function common result code
	uint16_t responseStatus, packetcounter = 0;
	
	SysTick_Start();
	memset(line_buffer, 0, 512);
  
  ff_result = f_mount(&fs,"",1);	// Register a work area for logical drive 0  
  if (ff_result) {
		put_rc(ff_result);
		console_log("Mount Failed !!\n\r");
    return ff_result;
	}

	ff_result = f_open(&fsrc,"PLOG.TXT", FA_OPEN_EXISTING | FA_READ);
	if (ff_result) {
		console_log("Source Opening Failed !!\n\r");
		PlogCreate = 0;
		put_rc(ff_result);
		return ff_result;
	}
	else put_rc(ff_result);

	while (f_gets(line_buffer, sizeof(line_buffer), &fsrc))	{	// Copy source to destination
		WDT_Feed(); // Feed WDT 
		
		/* Send the packet to the server for the first pending log */
		if (FailedFirstPendinglog == 0) {
			responseStatus = PackOffLog(line_buffer);
		}
		
		/* increment counter to indicate packet read once sucessfully */
		if(responseStatus == 1)	packetcounter = 1;			
							

		/* if first pending log failed, don't try to send others logs,
			just copy the remaing logs in temp.txt */
		if(FailedFirstPendinglog == 1) {
			print_DebugMsg("First pending log failed, dump logs to Temporary file\n\r");
			responseStatus = MODEM_RESPONSE_ERROR;
		}
		/* Save packet to temporary log - if packet sent not succesful */
		if((responseStatus == MODEM_RESPONSE_TIMEOUT) || (responseStatus == MODEM_RESPONSE_ERROR) 
			 || (responseStatus == 0)) {
			FailedFirstPendinglog = 1;
			packetcounter = 1;
			/* log form pending log - not succesful to hit, save it in temporary log*/
					
			ff_result = f_open(&fdst, "temp.txt", FA_WRITE | FA_OPEN_ALWAYS);
			if (ff_result == FR_OK) {
        /* Seek to end of the file to append data */
					ff_result = f_lseek(&fdst, f_size(&fdst));
					if (ff_result != FR_OK)
							f_close(&fdst);
			}
			if (ff_result) {
					FailedFirstPendinglog = 0;
					console_log("Destination Opening Failed, FileName: temp.txt -- ErrorStatus: %d \n\r",ff_result);
					f_close(&fsrc);			
					f_unmount("");	// Unregister a work area before discard it
					return ff_result;
			}

			res = f_puts(line_buffer,&fdst);
			console_log("Read String: %s \n\r",line_buffer);
			if ( res == EOF ) 
			{
				FailedFirstPendinglog = 0;
			
				f_close(&fsrc);	// Close source file
				f_close(&fdst); // Close destination file
				f_unlink("temp.txt");
				print_DebugMsg("TEMP.txt deleted\n\r");	
		
				f_unmount("");	// Unregister a work area before discard it
				console_log("Writing string failed, FileName: temp.txt -- ErrorStatus: %d \n\r",res);
				return res;
			}
			else {
				console_log("Put String, FileName: temp.txt -- %s \n\r",line_buffer);
			}
			DELAY_ms(10);
			f_sync(&fdst);
		}
	}

	f_close(&fsrc);	// Close source file
  f_close(&fdst);	// Close destination file

	/* delete pending log only when atleast the packet read once from SD card*/
	if(packetcounter){
		res = f_unlink("plog.txt");
		if(res == FR_OK) {
			PlogCreate = 0;
			print_DebugMsg("Pending log file plog.txt deleted\n\r");
		}
		DELAY_ms(100);
		res = f_rename("temp.txt", "plog.txt");
		if(res == FR_OK) {
			PlogCreate = 1;
			print_DebugMsg("Temp log renamed, plog set\n\r");
		}
	}
	

	FailedFirstPendinglog = 0;
	f_unmount("");	// Unregister a work area before discard it
	SysTick_Stop();
	return res;	
}

/*-----------------------------------------------------------------------*/
/* send the pending logs to the server                                   */
/*-----------------------------------------------------------------------*/
int log_SMS( 
	//	const char* path   /* Pointer to the file object */
void
)
{
	static FIL fsrc;		// file objects
  int res;					// FatFs function common result code
	uint16_t responseStatus; 
	char packetcounter = 0;

  // To Do systick handler timeout for sd card	

	memset(line_buffer, 0, 512);
  
  ff_result = f_mount(&fs,"",1);	// Register a work area for logical drive 0  
  if (res) {
				put_rc(res);
        return res;
	}

	//res = f_open(&fsrc,"/01-2021/05012021.TXT", FA_OPEN_EXISTING | FA_READ);fileName
	res = f_open(&fsrc,fileName, FA_OPEN_EXISTING | FA_READ);
	if (ff_result) {
		console_log("Source Opening Failed !!\n\r");
		put_rc(res);
		return res;
	}
	else put_rc(res);

	while (f_gets(line_buffer, sizeof(line_buffer), &fsrc))	{	// Copy source to destination
		//WDT_UpdateTimeOut(600000000);	/* Feed WDT */
		
		if(FailedFirstPendinglog == 0) 	/* Send the packet to the server for the first pending log */
		{ 			
			//To-do
			//Send SMS
			//responseStatus = sendSMS(line_buffer);
			console_log("%s \n\r",line_buffer);
			responseStatus = 1; // delete this line after testing
		}
		
		if(responseStatus == 1)					/* increment counter to indicate packet read once sucessfully */
			packetcounter = 1;				

		/* if first pending log failed, don't try to send others logs,
			just copy the remaing logs in temp.txt */
		if(FailedFirstPendinglog == 1) {
			print_DebugMsg("First record SMS failed..\n\r");
			responseStatus = MODEM_RESPONSE_ERROR;
		}
		/* Save packet to temporary log - if packet sent not succesful */
		if((responseStatus == MODEM_RESPONSE_TIMEOUT) || (responseStatus == MODEM_RESPONSE_ERROR) || 
							(responseStatus == 0)) {
			FailedFirstPendinglog = 1; packetcounter = 1;
		}
	}
	f_close(&fsrc);	// Close source file

	FailedFirstPendinglog = 0;
	f_unmount("");	// Unregister a work area before discard it
	return res;	
}

/*-----------------------------------------------------------------------*/
/* Read configuration file                                    */
/*-----------------------------------------------------------------------*/
int readconfigfile(char* iniContent)
{
	//FATFS fs;     // Work area (file system object) for logical drive
    FIL fil;      // file objects
	int res;         // FatFs function common result code
//	UINT br;         /* File read count */
	SysTick_Start();
	ff_result = f_mount(&fs,"",1);	// Register a work area for logical drive 0 
	if (ff_result) {
		put_rc(ff_result);
		console_log("File mouting failed, FileName: INI.txt -- ErrorStatus: %d \n\r",ff_result);
        return ff_result;
	}
		
	res = f_open(&fil, "INI.txt", FA_OPEN_EXISTING | FA_READ);
	if (res) {
		console_log("Destination Opening Failed, FileName: INI.txt -- ErrorStatus: %d \n\r",res);
		return res;
	}
	/* get string  to destination */
	while (f_gets(iniContent,512, &fil)){
		f_close( &fil);
		return 0;
   	}
	f_close( &fil );
	f_unmount("");	// Unregister a work area before discard it
	SysTick_Stop();
	return 1;
}

/*-----------------------------------------------------------------------*/
/* 						INI configuration file update through SMS                  */
/*-----------------------------------------------------------------------*/
int smsUpdateConfig ( 	
	const char* Parameters,  /* A log to the file */
	const char* path   /* Pointer to the file object */)
{
//    n = sizeof(Parameters);
		int res=0;
		SysTick_Start();
		ff_result = f_mount(&fs,"",1);
    if (ff_result)
    {
        console_log("Mount failed.\n\r");
        return ff_result;
    }
		
    ff_result = f_open(&file,"temp.txt", FA_WRITE | FA_OPEN_ALWAYS);

	
		if (ff_result == FR_OK){
			f_puts(Parameters,&file);
			f_puts("\n",&file);
		}
    if (ff_result != FR_OK)
    {
        console_log("Write failed.\n\r");
        return ff_result;
    }
    else
    {
        console_log("Write Successful...\n\r");
				res=1;
    } 
    
    /* Close the file */
    f_close(&file);

		if(res){
			 res = f_unlink("INI.txt");
		if(res == FR_OK) print_DebugMsg(" existing file INI.txt deleted\n\r");
		DELAY_ms(100);
		f_rename("temp.txt", "INI.txt");
		}

		f_unmount(""); // Unregister a work area before discard it
		SysTick_Stop();
		return res;
}

