/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Microchip Technologies Inc.
  @File Name
    app_flash_helper.h

  @Summary
        helper functions for MQTT dashboard demo.

  @Description
        helper functions for MQTT dashboard demo.
 */
/* ************************************************************************** */

#ifndef _APP_FLASH_HELPER_H    /* Guard against multiple inclusion */
#define _APP_FLASH_HELPER_H

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

typedef enum{
    APP_FLASH_RETVAL_SUCCESS=0,
    APP_FLASH_RETVAL_BUSY=-1,
    APP_FLASH_RETVAL_ERROR=-2, 
}APP_FLASH_RETVAL_t;

typedef enum{
    APP_FLASH_STATUS_NULL=0,
    APP_FLASH_STATUS_IN_PROGRESS,
    APP_FLASH_STATUS_SUCCESS,
    APP_FLASH_STATUS_ERROR,
}APP_FLASH_STATUS_t;

APP_FLASH_RETVAL_t APP_read_flash(unsigned int startAddress, uint8_t* buffer,uint32_t nBytes, APP_FLASH_STATUS_t *status);
APP_FLASH_RETVAL_t APP_erase_flash(unsigned int startAddress, uint32_t nBlocks, APP_FLASH_STATUS_t *status);
APP_FLASH_RETVAL_t APP_write_flash(unsigned int startAddress, uint8_t* buffer,uint32_t nBytes, APP_FLASH_STATUS_t *status);

#ifdef __cplusplus
}
#endif

#endif /* _APP_FLASH_HELPER_H */

/* *****************************************************************************
 End of File
 */