/*
 * File:       sd_card.c
 * Author:     Peter Thornton
 * Purpose:    Functions to interface with SD card on Pumpkin DB / MBM
 *             SPI interface and file I/O based on Chapters 13-14 in
 *             Di Jasio, "Learning to fly the PIC24"
 * Created on: 15 May 2020
 */

#include "xc.h"
#include "clock.h"
#include "spi.h"
#include "sd_card.h"
#include <stdlib.h>     // malloc...
#include <ctype.h>      // toupper...
#include <string.h>     // memcpy...

//-------------------------------------------------------------
// Master Boot Record key fields offsets
#define FO_MBR          0L   // master boot record sector LBA
#define FO_FIRST_P    0x1BE  // offset of first partition table
#define FO_FIRST_TYPE 0x1C2  // offset of first partition type
#define FO_FIRST_SECT 0x1C6  // first sector of first partition
#define FO_FIRST_SIZE 0x1CA  // number of sectors in partition
#define FO_SIGN       0x1FE  // MBR signature location (55,AA)

#define FAT_EOF       0xffff // last cluster in a file
#define FAT_MCLST     0xfff8 // max cluster value in a fat 

// Partition Boot Record key fields offsets
#define BR_SXC      0xd      // (byte) sector per cluster 
#define BR_RES      0xe      // (word) res sectors boot record
#define BR_FAT_SIZE 0x16     // (word) FAT size sectors 
#define BR_FAT_CPY  0x10     // (byte) number of FAT copies
#define BR_MAX_ROOT 0x11     // (odd word) max entries root dir

// directory entry management
#define DIR_ESIZE   32      // size of a directory entry(bytes)
#define DIR_NAME    0       // offset file name
#define DIR_EXT     8       // offset file extension
#define DIR_ATTRIB  11      // offset attribute( 00ARSHDV)
#define DIR_CTIME   14      // creation time
#define DIR_CDATE   16      // creation date
#define DIR_ADATE   18      // last access date
#define DIR_TIME    22      // offset last use time  (word)
#define DIR_DATE    24      // offset last use date  (word)
#define DIR_CLST    26      // offset first cluster FAT (word)
#define DIR_SIZE    28      // offset of file size (dword)
#define DIR_DEL     0xE5    // marker deleted entry
#define DIR_EMPTY   0       // marker last entry in directory

// global definitions
char FError;                // error mail box   
MEDIA *D;                   // mounting info for storage device

// SD card commands
#define RESET          0  // aka GO_IDLE (CMD0)
#define INIT           1  // aka SEND_OP_COND (CMD1)
#define READ_SINGLE   17  // read a block of data
#define WRITE_SINGLE  24  // write a block of data

// SD card return values
#define DATA_START   0xfe // indicates card is ready for reading
#define DATA_ACCEPT  0x05 // indicates card has accepted a written data block

// function to send a 6-byte command to the SD card via SPI
int SendSDCmd(int c, LBA a, int crc)
{
    int i,r;
    
    // Enable SD card, via SS (active low). Clocking one byte after selecting gives time for the
    // device to recognize SS signal, and maybe also time for the level translators 
    // to stabilize.
    CS_SD = 0;
    write_spi1(0xff);

    // send the 6-byte command packet
    write_spi1(c | 0x40);      // send command plus frame bit (bit 6)
    write_spi1(a>>24);         // MSB of the address
    write_spi1(a>>16);
    write_spi1(a>>8);
    write_spi1(a);             // LSB of address
    write_spi1(crc | 0x01);    // CRC (for CMD0 and CMD8, other commands don't care)
    
    // wait for a response (up to 8 bytes delay)
    // device returns 0xff (keeps the line high) until it provides a proper
    // response code
    i = 9;
    do {
        r = write_spi1(0xff);
        if (r != 0xff) break;
    } while (--i > 0);
    
    return (r);
    /* return responses:
     0xff = timeout, no answer
     0x00 = command accepted
     0x01 = command received, in idle state (proper response for RESET)
     other values indicate errors
    */
}

// Initialize the SD card. There are multiple steps, several of which can
// generate errors. The return value indicates card ready, or which step
// is returning an error:
// Return   Indicates
// ------   ---------
//  0x00    No errors, card ready for use
//  0x03    CMD0 error (Reset)
//  0x04    CMD8 error: old version
//  0x05    CMD8 error: wrong voltage or incorrect test pattern
//  0x06    CMD58 error (check OCR)
//  0x08    CMD55 error (ACMD next)
//  0x09    ACMD41 error (test idle)
//  0x0a    CMD58 error (check OCR)
//  0x0b    CMD58 error: high capacity card (not supported by RamSat code)
//  0xff    Timeout error: exceeded the count limit for CMD55+ACMD41 sequence
int SD_init(void)
{
    int i,r;
    int c8_1, c8_2, c8_3, c8_4;
    int c58_1, c58_2, c58_3, c58_4;
    
    // 1. power up the SD slot (-ON_SD, active low), and wait one second.
    _RE4 = 0;
    unsigned long wait = 1000L * DELAYMSEC;
    TMR1 = 0;
    while (TMR1 < wait);
    
    // 2. send 80 clock cycles, required to begin device initialization.
    // Guidance from others is to have the device de-selected during this
    // step. However, due to the use of SD_SS as the signal to power on the level
    // translators between PIC and SD card (active low), with SDSS de-selected
    // the following clock pulses don't get to the device, and the initialization stalls.
    // Testing shows that selecting the device (which also powers the level translators)
    // does not interfere with the needed clocking cycles, and the card initializes
    // without error.
    CS_SD = 0;    
    for (i=0 ; i<10 ; i++)
        write_spi1(0xff);
    
    // 3. send CMD0 (RESET) to enter SPI mode
    r = SendSDCmd(0, 0, 0x94); 
    CS_SD = 1; write_spi1(0xff);
    if (r != 1)
        return 0x03;
    
    // 4. send CMD8 to check version (1 or 2)
    r = SendSDCmd(8, 0x1aa, 0x86);
    if (r > 1)
        return 0x04;
    // 5. CMD8 recognized if r = 0 or 1. So it's a v2.0 card. Read the next
    // four bytes of the response (it's an R7 5-byte response if v2)
    c8_1 = write_spi1(0xff);  // should be 0x00
    c8_2 = write_spi1(0xff);  // should be 0x00
    c8_3 = write_spi1(0xff);  // should be 0x01
    c8_4 = write_spi1(0xff);  // should be 0xaa (test pattern)
    CS_SD = 1; write_spi1(0xff);
    if (c8_3 != 1 || c8_4 != 0xaa)
        return 0x05;
    
    // 6. CMD58, check OCR (operation condition register)
    r = SendSDCmd(58, 0, 0);
    if (r > 1)
        return 0x06;
    // 7. CMD58 recognized, no error flags. Read the next 4 bytes
    // as an R3 response
    c58_1 = write_spi1(0xff);
    c58_2 = write_spi1(0xff);
    c58_3 = write_spi1(0xff);
    c58_4 = write_spi1(0xff);
    CS_SD = 1; write_spi1(0xff);
     
    // 8-9. repeatedly send CMD55 + ACMD41, until ACMD41 returns 0 (exit IDLE state)
    i = 10000;   // allow for up to 1s before timeout
    do {
        // send CMD55, which enables ACMD. Check for errors
        r = SendSDCmd(55, 0, 0);
        CS_SD = 1; write_spi1(0xff);
        if (r > 1)
            return 0x08;
        
        // send ACMD41, and check for errors
        r = SendSDCmd(41, 0, 0);
        CS_SD = 1; write_spi1(0xff);
        if (r > 1)
            return 0x09;
        // check if exited idle state (return value 0)
        if (!r) break;  // card is ready!
    } while (--i > 0);
    if (i == 0)
        return 0xff;  // timeout indicator - card not ready
    
    // 10. Send CMD58, to see if this is an SD, or SDHC card
    r = SendSDCmd(58, 0, 0);
    if (r > 0)
        return 0x0a;
    // 11. CMD58 recognized, no error flags. Read the next 4 bytes as an R3 response
    c58_1 = write_spi1(0xff);
    c58_2 = write_spi1(0xff);
    c58_3 = write_spi1(0xff);
    c58_4 = write_spi1(0xff);
    CS_SD = 1; write_spi1(0xff);
    if (c58_1 != 0x80)  // high capacity card - not supported
        return 0x0b;
    
    // 12. Initialization complete, can now increase SPI speed to 4MHz
    SPI1STATbits.SPIEN = 0;      // temporarily disable the peripheral
    SPI1CON1bits.SPRE  = 0b111;  // pre2 = 1:1
    SPI1CON1bits.PPRE  = 0b10;   // pre1 = 4:1
    SPI1STATbits.SPIEN = 1;      // re-enable the peripheral
    
    return 0; // return value 0 indicates card is ready
}

// read a single 512-byte sector at a given LBA (logic block address)
int SD_read_sector(LBA a, unsigned char *p)
{
    int i, r;
    
    // The a<<9 converts LBA to byte value (multiplies by 512)
    r = SendSDCmd(READ_SINGLE, (a<<9), 0);
    if (r == 0) // command accepted
    {
        // wait for a response
        i=10000;
        do {
            r=write_spi1(0xff);
            if (r == DATA_START) break;
        } while (--i > 0);
        
        // if the response loop did not timeout, read 512-byte sector
        if (i)
        {
            for (i=0 ; i<512 ; i++)
                *p++ = write_spi1(0xff);
            // ignore 16-bit CRC value
            write_spi1(0xff);
            write_spi1(0xff);
        } // data arrived
    } //command accepted
    
    // de-select device
    CS_SD = 1; write_spi1(0xff);
    
    return (r == DATA_START);  // return true if successful
}

// write a single 512-byte sector to a given LBA (logic block address)
int SD_write_sector(LBA a, unsigned char *p)
{
    unsigned int r,i;
    r = SendSDCmd(WRITE_SINGLE, (a<<9), 0);
    if (r == 0)  // command accepted
    {
        write_spi1(DATA_START);
        // write sector
        for (i=0 ; i<512 ; i++)
            write_spi1(*p++);
        // send dummy CRC
        write_spi1(0xff);
        write_spi1(0xff);
        
        // check if data accepted
        if ((r = write_spi1(0xff) & 0xf) == DATA_ACCEPT)
        {
            for (i=65000 ; i>0 ; i--)
            {  // wait for end of write operation (SDO goes high)
                if ((r = write_spi1(0xff)))
                    break;
            }
        }  // data block accepted
        else
            r = 0x0a;  // failed to receive data accept response
    }  // command accepted
    CS_SD = 1; write_spi1(0xff);
    
    return r;  // returns TRUE if successful
}

//-------------------------------------------------------------
// mount    initializes a MEDIA structure for file IO access
//
// will mount only the first partition on the disk/card
MEDIA * SD_mount( void)
{ 
    LBA  psize;      // number of sectors in partition
    LBA  firsts;     // first sector inside the first partition
    int i;
    unsigned char *buffer;
    
    // 0. init the I/Os
    // Skip this here - already done in main()

    // 1. check if the card is in the slot 
    // skip this -assuming card always installed

    // 2. initialize the card    
    if ( SD_init())
    { 
        FError = FE_CANNOT_INIT;
        return NULL;
    }

    // 3. allocate space for a MEDIA structure 
    D = (MEDIA *) malloc( sizeof( MEDIA));
    if ( D == NULL)            // report an error  
    {   
        FError = FE_MALLOC_FAILED;
        return NULL;
    }

    // 4. allocate space for a temp sector buffer
    buffer = ( unsigned char *) malloc( 512);
    if ( buffer == NULL)        // report an error  
    {   
        FError = FE_MALLOC_FAILED;
        free( D);
        return NULL;
    }

    // 5.  get the Master Boot Record 
    if ( !SD_read_sector( FO_MBR, buffer))
    {
        FError = FE_CANNOT_READ_MBR;
        free( D); free( buffer);
        return NULL;
    }
        
    // 6. check if the MBR sector is valid 
    //      verify the signature word
    if (( buffer[ FO_SIGN] != 0x55) ||
        ( buffer[ FO_SIGN +1] != 0xAA))
    {
        FError = FE_INVALID_MBR;
        free( D); free( buffer);
        return NULL;
    }

    //    Valid Master Boot Record Loaded
    
    // 7. read the number of sectors in partition
    psize = ReadL( buffer, FO_FIRST_SIZE);
    
    // 8. check if the partition type is acceptable
    i = buffer[ FO_FIRST_TYPE];
    switch ( i)
    {
        case 0x04:
        case 0x06:
        case 0x0E:
            // valid FAT16 options
            break;
        default:
            FError = FE_PARTITION_TYPE;
            free( D); free( buffer);
            return NULL;
    } // switch
        
    // 9. get the first partition first sector -> Boot Record
    //  get the 32 bit offset to the first partition 
    firsts = ReadL( buffer, FO_FIRST_SECT); 
    // assuming FO_MBR == 0 
    
    // 10. get the sector loaded (boot record)
    if ( !SD_read_sector( firsts, buffer))
    {
        free( D); free( buffer);
        return NULL;
    }

    // 11. check if the boot record is valid 
    //      verify the signature word
    if (( buffer[ FO_SIGN] != 0x55) ||
        ( buffer[ FO_SIGN +1] != 0xAA))
    {
        FError = FE_INVALID_BR;
        free( D); free( buffer);
        return NULL;
    }

    // Valid Partition Boot Record Loaded       
    // get the full partition/drive layout

    // 12. determine the size of a cluster
    D->sxc = buffer[ BR_SXC];
    // this will also act as flag that the disk is mounted
    
    // 13. determine fat, root and data LBAs
    // FAT = first sector in partition 
    // (boot record) + reserved records
    D->fat = firsts + ReadW( buffer, BR_RES);
    D->fatsize = ReadW( buffer, BR_FAT_SIZE);
    D->fatcopy = buffer[ BR_FAT_CPY];
    
    // 14. ROOT = FAT + (sectors per FAT *  copies of FAT)
    D->root = D->fat + ( D->fatsize * D->fatcopy);
    
    // 15. MAX ROOT is the maximum number of entries 
    //      in the root directory
    D->maxroot = ReadOddW( buffer, BR_MAX_ROOT) ;
    
    // 16. DATA = ROOT + (MAXIMUM ROOT *32 / 512) 
    D->data = D->root + ( D->maxroot >> 4); 
    // assuming maxroot % 16 == 0!!!

    // 17. max clusters in this partition 
    //       = (tot sectors - sys sectors )/sxc
    D->maxcls = (psize - (D->data - firsts)) / D->sxc;
    
    // 18. free up the temporary buffer
    free( buffer);
    return D;

} // mount

//-------------------------------------------------------------
// umount    initializes a MEDIA structure for file IO access
//
void SD_umount( void)
{ 
    free( D);
    D = NULL;
} // umount

//-------------------------------------------------------------
// Open File on the default storage device D
//
MFILE *fopenM( const char *filename, const char *mode)
{
    char c;
    int i, r, e;
    unsigned char *b;       
    MFILE *fp;              

    // 1.  check if storage device is mounted 
    if ( D == NULL)       // unmounted
    {
        FError = FE_MEDIA_NOT_MNTD;
        return NULL;
    }
    
    // 2. allocate a buffer for the file
    b = (unsigned char*)malloc( 512);
    if ( b == NULL)
    {   
        FError = FE_MALLOC_FAILED;
        return NULL;
    }
    
    // 3. allocate a MFILE structure on the heap
    fp = (MFILE *) malloc( sizeof( MFILE));
    if ( fp == NULL)            // report an error  
    {   
        FError = FE_MALLOC_FAILED;
        free( b);
        return NULL;
    }
    
    // 4. set pointers to the MEDIA structure and buffer
    fp->mda = D;
    fp->buffer = b;

    // 5. format the filename into name 
    for( i=0; i<8; i++)
    {
        // read a char and convert to upper case
        c = toupper( *filename++);      
        // extension or short name no-extension
        if (( c == '.') || ( c == '\0'))
            break;
        else 
            fp->name[i] = c;
    } // for
    // if short fill the rest up to 8 with spaces
    while ( i<8) fp->name[i++] = ' ';

    // 6. if there is an extension
    if ( c != '\0') 
    {    
        for( i=8; i<11; i++)
        {
            // read a char and convert to upper case
            c = toupper( *filename++);      
            if ( c == '.')
                c = toupper( *filename++);
            if ( c == '\0')                 // short extension
                break;
            else 
                fp->name[i] = c;
        } // for
        // if short fill the rest up to 3 with spaces
        while ( i<11) fp->name[i++] = ' ';
    } // if

    // 7. copy the file mode character  (r, w) 
    if ((*mode == 'r')||(*mode == 'w'))
        fp->mode = *mode;
    else
    { 
        FError = FE_INVALID_MODE;
        goto ExitOpen;
    }

    // 8. Search for the file in current directory
    if ( ( r = FindDIR( fp)) == FAIL)
    { 
        FError = FE_FIND_ERROR;
        goto ExitOpen;
    }
    
    // 9. init all counters to the beginning of the file
    fp->seek = 0;               // first byte in file
    fp->sec = 0;                // first sector in the cluster
    fp->pos = 0;                // first byte in sector/cluster
    
    // 10. depending on the mode (read or write) 
    if ( fp->mode == 'r') 
    {   // 10.1 'r' open for reading
        if  ( r == NOT_FOUND)
        {
            FError = FE_FILE_NOT_FOUND;
            goto ExitOpen;
        }
        else
        {  // found             
        // 10.2 set current cluster pointer on first cluster
            fp->ccls = fp->cluster;     

        // 10.3 read a sector of data from the file
            if ( !ReadDATA( fp))
            {
                goto ExitOpen;
            }
        
        // 10.4 determine how much data is really inside buffer
            if ( fp->size-fp->seek < 512)
                fp->top = fp->size - fp->seek;
            else
                fp->top = 512;
        } // found
    } // 'r' 
  
    else // 11.  open for 'write'
    { 
        if  ( r == NOT_FOUND)
        {
            // 11.1 allocate a first cluster to it
            fp->ccls = 0;            // indicate brand new file
            if ( NewFAT( fp) != TRUE)
            { // must be media full 
                FError = FE_MEDIA_FULL;
                goto ExitOpen;
            }
            fp->cluster = fp->ccls; 

            // 11.2 create a new entry
            // search again, for an empty entry this time
            if ( (r = NewDIR( fp)) == FAIL)
            {   // report any error
                FError = FE_IDE_ERROR;
                goto ExitOpen;
            }
            // 11.3 new entry not found 
            if ( r == NOT_FOUND)
            { 
                FError = FE_DIR_FULL;
                goto ExitOpen;
            }
            else // 11.4 new entry identified fp->entry filled 
            {
                // 11.4.1
                fp->size = 0;

                // 11.4.2    determine offset in DIR sector
                e = (fp->entry & 0xf) * DIR_ESIZE;    
        
                // 11.4.3 init all fields to 0
                for (i=0; i<32; i++)
                        fp->buffer[ e + i]  = 0;
        
                // 11.4.4 set date and time
                fp->date = 0x378A; // Dec 10th, 2007
                fp->buffer[ e + DIR_CDATE]  = fp->date;
                fp->buffer[ e + DIR_CDATE+1]= fp->date>>8;
                fp->buffer[ e + DIR_DATE]  = fp->date;
                fp->buffer[ e + DIR_DATE+1]= fp->date>>8;

                fp->time = 0x6000; // 12:00:00 PM
                fp->buffer[ e + DIR_CTIME]  = fp->time;
                fp->buffer[ e + DIR_CTIME+1]= fp->time>>8;
                fp->buffer[ e + DIR_TIME]  = fp->time+1;
                fp->buffer[ e + DIR_TIME+1]= fp->time>>8;
        
                // 11.4.5 set first cluster
                fp->buffer[ e + DIR_CLST]  = fp->cluster;
                fp->buffer[ e + DIR_CLST+1]= (fp->cluster>>8);
                
                // 11.4.6 set name
                for ( i = 0; i<DIR_ATTRIB; i++)
                    fp->buffer[ e + i] = fp->name[i];
                
                // 11.4.7 set attrib
                fp->buffer[ e + DIR_ATTRIB] = ATT_ARC;
                
                // 11.4.8  update the directory sector;
                if ( !WriteDIR( fp, fp->entry))
                {
                    FError = FE_IDE_ERROR;
                    goto ExitOpen;
                }
            } // new entry
        } // not found

        else // file exist already, report error
        {
            FError = FE_FILE_OVERWRITE;
            goto ExitOpen;
        }
            
    } // w request

    // 12. Exit with success    
    return fp;

    // 13. Exit with error
ExitOpen:
    free( fp->buffer); 
    free( fp);
    return NULL;
        
} // fopenM

//-------------------------------------------------------------
// Delete existing file on the default storage device D
//
int fdeleteM( const char *filename, const char *mode)
{
    char c;
    int i, r;
    unsigned e;
    unsigned short cur_cluster, next_cluster;
    unsigned char *b;       
    MFILE *fp;
    

    // check if storage device is mounted 
    if ( D == NULL)       // unmounted
    {
        FError = FE_MEDIA_NOT_MNTD;
        return FError;
    }
    
    // allocate a buffer for the file
    b = (unsigned char*)malloc( 512);
    if ( b == NULL)
    {   
        FError = FE_MALLOC_FAILED;
        return FError;
    }
    
    // allocate a MFILE structure on the heap
    fp = (MFILE *) malloc( sizeof( MFILE));
    if ( fp == NULL)            // report an error  
    {   
        FError = FE_MALLOC_FAILED;
        free( b);
        return FError;
    }
    
    // set pointers to the MEDIA structure and buffer
    fp->mda = D;
    fp->buffer = b;

    // format the filename into name 
    for( i=0; i<8; i++)
    {
        // read a char and convert to upper case
        c = toupper( *filename++);      
        // extension or short name no-extension
        if (( c == '.') || ( c == '\0'))
            break;
        else 
            fp->name[i] = c;
    } // for
    // if short fill the rest up to 8 with spaces
    while ( i<8) fp->name[i++] = ' ';

    // if there is an extension
    if ( c != '\0') 
    {    
        for( i=8; i<11; i++)
        {
            // read a char and convert to upper case
            c = toupper( *filename++);      
            if ( c == '.')
                c = toupper( *filename++);
            if ( c == '\0')                 // short extension
                break;
            else 
                fp->name[i] = c;
        } // for
        // if short fill the rest up to 3 with spaces
        while ( i<11) fp->name[i++] = ' ';
    } // if

    // copy the file mode character  (d for delete) 
    if (*mode == 'd')
        fp->mode = *mode;
    else
    { 
        FError = FE_INVALID_MODE;
        goto ExitDelete;
    }

    // Search for the file in current directory
    if ( ( r = FindDIR( fp)) == FAIL)
    { 
        FError = FE_FIND_ERROR;
        goto ExitDelete;
    }
    
    // found or not found 
    if  ( r == NOT_FOUND)
    {
        FError = FE_FILE_NOT_FOUND;
        goto ExitDelete;
    }
    else
    {  // found             
        // set current cluster pointer on first cluster
        fp->ccls = fp->cluster;
        
        // at this point, fp->buffer holds the root directory sector containing
        // the target file. Modify the first character of filename with the 
        // "deleted file" flag value, and write the entire sector back to root dir.
        // First determine the offset in current buffer
        e = (fp->entry&0xf) * DIR_ESIZE;
        // Then write the deleted flag to first char of file name
        fp->buffer[ e + DIR_NAME] = DIR_DEL;
        // Now write the modified buffer back to root dir sector
        if ( !WriteDIR( fp, fp->entry))
        {
            FError = FE_IDE_ERROR;
            goto ExitDelete;
        }
        
        // Begin traverse of FAT clusters, clearing entries for this file
        // (in all FAT copies)
        cur_cluster = fp->cluster;
        while (1)
        {
            next_cluster = ReadFAT(fp, cur_cluster);
            if (!WriteFAT(fp, cur_cluster, 0))
            {
                FError = FE_WFAT_ERROR;
                goto ExitDelete;
            }
            // advance if this is not the end of the cluster chain
            if (next_cluster != FAT_EOF)
            {
                cur_cluster = next_cluster;
            }
            else  // free memory and return success
            {
                free( fp->buffer); 
                free( fp);
                return 0;
            }
        }
    } // found
    
// Exit with error
ExitDelete:
    free( fp->buffer); 
    free( fp);
    return FError;
        
} // fdeleteM

//-------------------------------------------------------------
// Read File
//
// returns      number of bytes actually transferred
//
unsigned freadM( void * dest, unsigned size, MFILE *fp)
// fp       pointer to MFILE structure
// dest     pointer to destination buffer
// count    number of bytes to transfer
// returns  number of bytes actually transferred
{
    MEDIA * mda = fp->mda;      
    unsigned count=size;        // counts bytes to be transfer
    unsigned len;          
    

    // 1. check if fp points to a valid open file structure
    if (( fp->mode != 'r')) 
    {   // invalid file or not open in read mode
        FError = FE_INVALID_FILE; 
        return 0;
    }

    // 2. loop to transfer the data
    while ( count>0)
    {               

        // 2.1 check if EOF reached 
        if ( fp->seek >= fp->size)
        {
            FError = FE_EOF;  // reached the end
            break;
        }
        
        // 2.2 load a new sector if necessary
        if (fp->pos == fp->top) 
        {   
            fp->pos = 0;
            fp->sec++;
            
            // 2.2.1 get a new cluster if necessary
            if ( fp->sec == mda->sxc)
            {
                fp->sec = 0;
                if ( !NextFAT( fp, 1))
                    break;
            }
            // 2.2.2 load a sector of data
            if ( !ReadDATA( fp))
                break;

            // 2.2.3 determine how much data is inside buffer
            if ( fp->size-fp->seek < 512)
                fp->top = fp->size - fp->seek;
            else
                fp->top = 512;
         } //  load new sector

        // 2.3 copy as many bytes as possible in a single chunk
        // take as much as fits in the current sector
        if ( fp->pos+count < fp->top)
            // fits all in current sector
            len = count;          
        else 
            // take a first chunk, there is more
            len = fp->top - fp->pos;    

        memcpy( dest, fp->buffer + fp->pos, len);

        // 2.4 update all counters and pointers
        count-= len;            // compute what is left
        dest += len;            // advance destination pointer
        fp->pos += len;         // advance pointer in sector
        fp->seek += len;        // advance the seek pointer
        
    } // while count
    
    // 3. return number of bytes actually transferred 
    return size-count;

} // freadM

//-------------------------------------------------------------
// Write data to a File
//
unsigned fwriteM(  void *src, unsigned count, MFILE * fp)
// src      points to source data (buffer) 
// count    number of bytes to write
// returns  number of bytes actually written
{
    MEDIA *mda = fp->mda;
    unsigned len, size = count;
    
    // 1.  check if file is open
    if ( fp->mode != 'w') 
    {   // file not valid or not open for writing
        FError = FE_INVALID_FILE; 
        return FAIL;
    }

    // 2. loop writing count bytes 
    while ( count>0)
    {               
        // 2.1 copy as many bytes at a time as possible
        if ( fp->pos+count < 512)
            len = count;
        else 
            len = 512- fp->pos ;

        memcpy( fp->buffer+ fp->pos, src, len);

        // 2.2 update all pointers and counters
        fp->pos+=len;       // advance buffer position
        fp->seek+=len;      // count the added bytes
        count-=len;         // update the counter
        src+=len;           // advance the source pointer

        // 2.3 update the file size too
        if (fp->seek > fp->size)
            fp->size = fp->seek;
        
        // 2.4 if buffer full, write buffer to current sector 
        if (fp->pos == 512) 
        {   
            // 2.4.1 write buffer full of data
            if ( !WriteDATA( fp))
                return FAIL;

            // 2.4.2 advance to next sector in cluster 
            fp->pos = 0;
            fp->sec++;
            
            // 2.4.3 get a new cluster if necessary
            if ( fp->sec == mda->sxc)
            {
                fp->sec = 0;
                if ( NewFAT( fp)== FAIL)
                    return FAIL;
            }
         } //  store sector
    } // while count
        
    // 3.  number of bytes actually written 
    return size-count;

} // fwriteM

//-------------------------------------------------------------
// Close a File
//
unsigned fcloseM( MFILE *fp)
{
    unsigned e, r;     

    r = FAIL;

    // 1. check if it was open for write
    if ( fp->mode == 'w')   
    {
        // 1.1 if the current buffer contains data, flush it
        if ( fp->pos >0)
        {   
            if ( !WriteDATA( fp))
                goto ExitClose; 
        } 
        
        // 1.2      finally update the dir entry, 
        // 1.2.1    retrive the dir sector
        if ( !ReadDIR( fp, fp->entry))
            goto ExitClose;
    
        // 1.2.2    determine position in DIR sector
        e = (fp->entry & 0xf) * DIR_ESIZE;    

        // 1.2.3 update file size
        fp->buffer[ e + DIR_SIZE]  = fp->size;
        fp->buffer[ e + DIR_SIZE+1]= fp->size>>8;
        fp->buffer[ e + DIR_SIZE+2]= fp->size>>16;
        fp->buffer[ e + DIR_SIZE+3]= fp->size>>24;

        // 1.2.4    update the directory sector;
        if ( !WriteDIR( fp, fp->entry))
            goto ExitClose;
    } // write

    // 2. exit with success
    r = TRUE;

ExitClose:        

    // 3. free up the buffer and the MFILE struct
    free( fp->buffer);
    free( fp);
    return( r);
    
} // fcloseM

//-------------------------------------------------------------
// Find a File entry in current directory
//
unsigned FindDIR( MFILE *fp)
// fp       file structure
// return   found/not_found/fail 
{
    unsigned eCount;            // current entry counter
    unsigned e;                 // current entry offset 
    int i, a;             
    MEDIA *mda = fp->mda;
    
    // 1. start from the first entry
    eCount = 0;

    // load the first sector of root
    if ( !ReadDIR( fp, eCount))
        return FAIL;
            
    // 2. loop until you reach the end or find the file
    while ( 1)
    {
    // 2.0 determine the offset in current buffer
        e = (eCount&0xf) * DIR_ESIZE;

    // 2.1 read the first char of the file name 
        a = fp->buffer[ e + DIR_NAME]; 

    // 2.2 terminate if it is empty (end of the list)        
        if ( a == DIR_EMPTY)
        {
            return NOT_FOUND;
        } // empty entry

    // 2.3 skip erased entries if looking for a match
        if ( a != DIR_DEL)
        {
        // 2.3.1 if not VOLume or DIR compare the names
            a = fp->buffer[ e + DIR_ATTRIB];
            if ( !(a & (ATT_DIR | ATT_HIDE)) )
            {   
                // compare file name and extension
                for (i=DIR_NAME; i<DIR_ATTRIB; i++)
                {
                   if ( fp->buffer[ e + i] !=  fp->name[i])
                        break;  // difference found
                }

                if ( i == DIR_ATTRIB)
                {
                  // entry found, fill the file structure 
                  fp->entry = eCount;         // store index
                  fp->time = ReadW( fp->buffer, e+DIR_TIME);
                  fp->date = ReadW( fp->buffer, e+DIR_DATE);
                  fp->size = ReadL( fp->buffer, e+DIR_SIZE);
                  fp->cluster = ReadL( fp->buffer, e+DIR_CLST);
                  return FOUND;      
                }
            } // not dir nor vol
        } // not deleted
        
    // 2.4 get the next entry
        eCount++;
        if ( (eCount & 0xf) == 0)
        {   // load a new sector from the Dir
            if ( !ReadDIR( fp, eCount))
                return FAIL; 
        }

    //  2.5. exit the loop if reached the end or error
        if ( eCount >= mda->maxroot)
            return NOT_FOUND;       // last entry reached
    
    }// while 
    
} // FindDIR

//-------------------------------------------------------------
// 
unsigned ReadDIR( MFILE *fp, unsigned e)
// loads current entry sector in file buffer
// returns      FAIL/TRUE
{        
    LBA l;

    // load the root sector containing the DIR entry "e"
    l = fp->mda->root + (e >> 4);
    fp->fpage = -1;             // invalidate FAT cache
    
    return ( SD_read_sector( l, fp->buffer));

} // ReadDIR

//-------------------------------------------------------------
// 
unsigned WriteDIR( MFILE *fp, unsigned entry)
// write current entry sector 
// returns      FAIL/TRUE
{        
    LBA l = fp->mda->root + (entry >> 4);
    // write the root sector 
    return ( SD_write_sector( l, fp->buffer)) ;
} // WriteDIR


//-------------------------------------------------------------
// Find a New entry in root directory or an empty entry
//
unsigned NewDIR( MFILE *fp)
// fp       file structure
// return   found/fail,  fp->entry filled
{
    unsigned eCount;            // current entry counter
    unsigned e;                 // current entry offset 
    int a;             
    MEDIA *mda = fp->mda;
    
    // 1. start from the first entry
    eCount = 0;
    // load the first sector of root
    if ( !ReadDIR( fp, eCount))
        return FAIL;
            
    // 2. loop until you reach the end or find the file
    while ( 1)
    {
    // 2.0 determine the offset in current buffer
        e = (eCount&0xf) * DIR_ESIZE;

    // 2.1 read the first char of the file name 
        a = fp->buffer[ e + DIR_NAME]; 

    // 2.2 terminate if it is empty (end of the list)or deleted 
        if (( a == DIR_EMPTY) ||( a == DIR_DEL))
        {
            fp->entry = eCount; 
            return FOUND;   
        } // empty or deleted entry found

        
    // 2.3 get the next entry
        eCount++;
        if ( (eCount & 0xf) == 0)
        { // load a new sector from the root
            if ( !ReadDIR( fp, eCount))
                return FAIL;                   
        }

    // 2.4 exit the loop if reached the end or error
        if ( eCount > mda->maxroot)
            return NOT_FOUND;       // last entry reached
    }// while 
    
    return FAIL;
} // NewDIR

//-------------------------------------------------------------
//
unsigned ReadDATA( MFILE *fp)
{
    LBA l;

    // calculate lba of cluster/sector
    l = fp->mda->data+(LBA)(fp->ccls-2) * fp->mda->sxc+fp->sec;
    fp->fpage = -1;         // invalidate FAT cache
    
    return( SD_read_sector( l, fp->buffer));

} // ReadDATA

//-------------------------------------------------------------
//
unsigned WriteDATA( MFILE *fp)
{
    LBA l;

    // calculate lba of cluster/sector
    l = fp->mda->data+(LBA)(fp->ccls-2) * fp->mda->sxc+fp->sec;

    return ( SD_write_sector( l, fp->buffer));

} // WriteDATA

//-------------------------------------------------------------
// Allocate New Cluster
//
unsigned NewFAT( MFILE * fp)
// fp       file structure
// fp->ccls     ==0 if first cluster to be allocated
//              !=0 if additional cluster 
// return   TRUE/FAIL
//  fp->ccls new cluster number 
{
    unsigned i, c = fp->ccls;
    
    // sequentially scan through the FAT 
    //    looking for an empty cluster
    do {
        c++;    // check next cluster in FAT
        // check if reached last cluster in FAT, 
        // re-start from top
        if ( c >= fp->mda->maxcls)
            c = 0;  
            
        // check if full circle done, media full
        if ( c == fp->ccls)
        {
            FError = FE_MEDIA_FULL;
            return FAIL;
        }
            
        // look at its value 
        i = ReadFAT( fp, c);
            
    } while ( i!=0);    // scanning for an empty cluster
    
    // mark the new cluster as taken, and last in chain
    WriteFAT( fp, c, FAT_EOF);
    
    // if not first cluster, link current cluster to new one
    if ( fp->ccls >0)
        WriteFAT( fp, fp->ccls, c);
    
    // update the MFILE structure
    fp->ccls = c;
    
    // invalidate the FAT cache 
    //   (since it will soon be overwritten with data)
    fp->fpage = -1;
        
    return TRUE;
} // NewFAT

//-------------------------------------------------------------
// Get Next Cluster
//
unsigned NextFAT( MFILE * fp, unsigned n)
// fp   file structure
// n    number of links in FAT cluster chain to jump through
//      n==1, next cluster in the chain
{
    unsigned    c;
    MEDIA * mda = fp->mda;
    
    // loop n times
    do {    
        // get the next cluster link from FAT
        c = ReadFAT( fp, fp->ccls);
            
        // compare against max value of a cluster in FATxx        
        // return if eof
        if ( c >= FAT_MCLST)    // check against eof
        {
            FError = FE_FAT_EOF;
            return FAIL;    // seeking beyond EOF
        }

        // check if cluster value is valid
        if ( c >= mda->maxcls)
        {
            FError = FE_INVALID_CLUSTER;
            return FAIL;
        }
        
    } while (--n > 0);// loop end

    // update the MFILE structure
    fp->ccls = c;
        
    return TRUE;
} // NextFAT

//-------------------------------------------------------------
// Read the content of cluster link in FAT
//
// return : 0000 cluster is empty
//          0FF8 last cluster in a chain
//          0xxx next cluster in a chain
//          FFFF FAIL

unsigned ReadFAT( MFILE *fp, unsigned ccls)
// mda      disk structure
// ccls     current cluster 
// return   next cluster value, 
//          0xffff if failed or last 
{
    unsigned p, c;
    LBA l;
        
    // get page of current cluster in fat
    p = ccls >>8;               // 256 clusters per sector  

    // check if already cached
    if (fp->fpage != p)
    {   
        // load the fat sector containing the cluster
        l = fp->mda->fat + p;   
        if ( !SD_read_sector( l, fp->buffer))
            return FAT_EOF;      // failed
        
        // note the sector contains a valid FAT page cache 
        fp->fpage = ccls>>8;
    }
        
    // get the next cluster value
    // cluster = 0xabcd
    // packed as:     0   |   1    |   2   |  3    |
    // word p       0   1 |  2   3 | 4   5 | 6   7 |..
    //              cd  ab|  cd  ab| cd  ab| cd  ab| 
    c = ReadOddW( fp->buffer, ((ccls & 0xFF)<<1));
    
    return c;
     
} // ReadFAT

//-------------------------------------------------------------
// Write a cluster link in all FAT copies
//
unsigned WriteFAT( MFILE *fp, unsigned cls, unsigned v)
// mda      disk structure
// cls      current cluster 
// v        next value
// return   TRUE if successful, or FAIL
{
    int i;
    unsigned p;
    LBA l;
    
    // load the FAT page containing the requested cluster
    ReadFAT( fp, cls);
    
    // cluster = 0xabcd
    // packed as:     0   |   1    |   2   |  3    |
    // word p       0   1 |  2   3 | 4   5 | 6   7 |..
    //              cd  ab|  cd  ab| cd  ab| cd  ab| 
       
    // locate the cluster within the FAT page
    p = (cls & 0xff)*2;    // 2 bytes per cluster 

    // get the next cluster value
    fp->buffer[ p] = v;       // lsb 
    fp->buffer[ p+1] = (v>>8);// msb
    
    // update all FAT copies
    l = fp->mda->fat + (cls>>8);
    for ( i=0; i<fp->mda->fatcopy; i++, l += fp->mda->fatsize)
       if ( !SD_write_sector( l, fp->buffer))
           return FAIL; 
    
    return TRUE;
     
} // WriteFAT


