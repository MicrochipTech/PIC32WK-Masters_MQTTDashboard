/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Microchip technologies Inc.

  @File Name
    app_flash_helper.c

  @Summary
    helper functions for MQTT dashboard demo flash functionality.

  @Description
    helper functions for MQTT dashboard demo flash functionality.
 */
/* ************************************************************************** */

#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "app_flash_helper.h"

static DRV_HANDLE ipfHandle;
static DRV_IPF_BLOCK_COMMAND_HANDLE commandHandle;
static bool ipfInUse = false;
APP_FLASH_STATUS_t *gStatus;

static void APP_IPFEventHandler(DRV_IPF_BLOCK_EVENT event,DRV_IPF_BLOCK_COMMAND_HANDLE commandHandle,uintptr_t contextHandle) {
    switch (event) {
        case DRV_IPF_EVENT_BLOCK_COMMAND_COMPLETE:
        {
            *gStatus=APP_FLASH_STATUS_SUCCESS;
            DRV_IPF_Close(ipfHandle);
            ipfInUse=false;
            break;
        }
        case DRV_IPF_EVENT_BLOCK_COMMAND_ERROR:
        {
            *gStatus=APP_FLASH_STATUS_ERROR;
            DRV_IPF_Close(ipfHandle);
            ipfInUse=false;
            break;
        }
        default:
            break;
    }
}


/*
 * read nBytes bytes of data from startAddres of IPF into buffer
 * Once the function is called, and the read operation is scheduled, it returns with APP_FLASH_RETVAL_SUCCESS .
 *    in case of error in scheduling the operation, APP_FLASH_RETVAL_ERROR is returned. 
 * 
 * The status monitoring flag passed to the function can be used to poll the read status .
 * once data has been read into the buffer, the status flag will be set to APP_FLASH_STATUS_SUCCESS.
 * in case of error, it will be set to APP_FLASH_STATUS_ERROR.
 * while the operation is in progress, the flag will be set to APP_FLASH_STATUS_IN_PROGRESS.
 */

APP_FLASH_RETVAL_t APP_read_flash(unsigned int startAddress, uint8_t* buffer,uint32_t nBytes, APP_FLASH_STATUS_t *status) {
    if (false == ipfInUse) {
        ipfInUse=true;
        gStatus=status;
        *gStatus=APP_FLASH_STATUS_NULL;
        
        ipfHandle = DRV_IPF_Open(DRV_IPF_INDEX_0, DRV_IO_INTENT_READWRITE);
        if (ipfHandle != DRV_HANDLE_INVALID) {
            DRV_IPF_BlockEventHandlerSet(ipfHandle,APP_IPFEventHandler,0);
            DRV_IPF_BlockRead( ipfHandle,&commandHandle,buffer,startAddress,nBytes);
            if(DRV_IPF_BLOCK_COMMAND_HANDLE_INVALID == commandHandle)
            {
                SYS_CONSOLE_PRINT("\r\nFailed to issue read command to IPF (%s [%u])", __FUNCTION__, __LINE__);
                DRV_IPF_Close(ipfHandle);
                ipfInUse=false;
                return APP_FLASH_RETVAL_ERROR;
            }
            else
            {
                *gStatus=APP_FLASH_STATUS_IN_PROGRESS;
                ipfInUse=true;
                return APP_FLASH_RETVAL_SUCCESS;
            }

        } else {
            SYS_CONSOLE_PRINT("\r\nFailed to open flash for app config read (%s [%u])", __FUNCTION__, __LINE__);
            ipfInUse=false;
            return APP_FLASH_RETVAL_ERROR;
        }
    }
    else
    {
        return APP_FLASH_RETVAL_BUSY;
    }
}

APP_FLASH_RETVAL_t APP_write_flash(unsigned int startAddress, uint8_t* buffer,uint32_t nBytes, APP_FLASH_STATUS_t *status) {
    if (false == ipfInUse) {
        ipfInUse=true;
        gStatus=status;
        *gStatus=APP_FLASH_STATUS_NULL;
        
        ipfHandle = DRV_IPF_Open(DRV_IPF_INDEX_0, DRV_IO_INTENT_READWRITE);
        if (ipfHandle != DRV_HANDLE_INVALID) {
            DRV_IPF_BlockEventHandlerSet(ipfHandle,APP_IPFEventHandler,0);
            DRV_IPF_BlockWrite( ipfHandle,&commandHandle,buffer,startAddress,nBytes);
            if(DRV_IPF_BLOCK_COMMAND_HANDLE_INVALID == commandHandle)
            {
                SYS_CONSOLE_PRINT("\r\nFailed to issue read command to IPF (%s [%u])", __FUNCTION__, __LINE__);
                DRV_IPF_Close(ipfHandle);
                ipfInUse=false;
                return APP_FLASH_RETVAL_ERROR;
            }
            else
            {
                *gStatus=APP_FLASH_STATUS_IN_PROGRESS;
                ipfInUse=true;
                return APP_FLASH_RETVAL_SUCCESS;
            }

        } else {
            SYS_CONSOLE_PRINT("\r\nFailed to open flash for app config read (%s [%u])", __FUNCTION__, __LINE__);
            ipfInUse=false;
            return APP_FLASH_RETVAL_ERROR;
        }
    }
    else
    {
        return APP_FLASH_RETVAL_BUSY;
    }
}


APP_FLASH_RETVAL_t APP_erase_flash(unsigned int startAddress, uint32_t nBlocks, APP_FLASH_STATUS_t *status) {
    if (false == ipfInUse) {
        ipfInUse=true;
        gStatus=status;
        *gStatus=APP_FLASH_STATUS_NULL;
        
        ipfHandle = DRV_IPF_Open(DRV_IPF_INDEX_0, DRV_IO_INTENT_READWRITE);
        if (ipfHandle != DRV_HANDLE_INVALID) {
            DRV_IPF_BlockEventHandlerSet(ipfHandle,APP_IPFEventHandler,0);
            DRV_IPF_BlockErase( ipfHandle,&commandHandle,startAddress,nBlocks);
            if(DRV_IPF_BLOCK_COMMAND_HANDLE_INVALID == commandHandle)
            {
                SYS_CONSOLE_PRINT("\r\nFailed to issue erase command to IPF (%s [%u])", __FUNCTION__, __LINE__);
                DRV_IPF_Close(ipfHandle);
                ipfInUse=false;
                return APP_FLASH_RETVAL_ERROR;
            }
            else
            {
                *gStatus=APP_FLASH_STATUS_IN_PROGRESS;
                ipfInUse=true;
                return APP_FLASH_RETVAL_SUCCESS;
            }

        } else {
            SYS_CONSOLE_PRINT("\r\nFailed to open flash for app config erase (%s [%u])", __FUNCTION__, __LINE__);
            ipfInUse=false;
            return APP_FLASH_RETVAL_ERROR;
        }
    }
    else
    {
        return APP_FLASH_RETVAL_BUSY;
    }
}


/* *****************************************************************************
 End of File
 */
