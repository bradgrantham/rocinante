/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2019        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "ff.h"
#include "diskio.h"
#include "defs.h"
#include "logprintf.h"
#include "sd_spi.h"

/* Definitions of physical drive number for each drive */
#define DEV_RAM		0	/* Example: Map Ramdisk to physical drive 0 */
#define DEV_MMC		1	/* Example: Map MMC/SD card to physical drive 1 */
#define DEV_USB		2	/* Example: Map USB MSD to physical drive 2 */


/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
    if(pdrv != 0)
        return STA_NODISK;

    return 0;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
    if(pdrv != 0)
        return RES_ERROR;

    return RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	LBA_t sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
    if(pdrv != 0)
        return RES_ERROR;

    for(int i = 0; i < count; i++)
        if(!SDCARD_readblock(sector + i, buff + SD_BLOCK_SIZE * i))  {
            logprintf(DEBUG_ERRORS, "ERROR: failed reading SD block %d\n", sector + i);
            return RES_ERROR;
	}

    return RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	LBA_t sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
    if(pdrv != 0)
        return RES_ERROR;

    for(int i = 0; i < count; i++)
        if(!SDCARD_writeblock(sector + i, buff + SD_BLOCK_SIZE * i)) {
            logprintf(DEBUG_ERRORS, "ERROR: failed reading SD block %d\n", sector + i);
            return RES_ERROR;
	}

    return RES_OK;
}

#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
    DRESULT result = RES_OK;

    if(pdrv != 0)
        return RES_ERROR;

    switch(cmd) {
        case CTRL_SYNC:
            result = RES_OK;
            break;

        default:
            logprintf(DEBUG_ERRORS, "ERROR: unexpected FatFS ioctl %d\n", cmd);
            result = RES_ERROR;
            break;
    }

    return result;
}

DWORD get_fattime(void)
{
    // returns FatFS formatted date-time
    // hardcoded to Mar 15 2016, midnight
    return
        ((FF_NORTC_YEAR - 1980) << 25) | 
        (FF_NORTC_MON << 21) | 
        (FF_NORTC_MDAY << 16) | 
        (0 << 11) | 
        (0 << 5) | 
        (0 << 0)
        ;
}
