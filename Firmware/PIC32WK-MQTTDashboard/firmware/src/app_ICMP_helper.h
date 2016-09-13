/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Microchip Technologies Inc.
  @File Name
    app_iCMP_helper.h

  @Summary
        helper functions for MQTT dashboard demo.

  @Description
        helper functions for MQTT dashboard demo.
 */
/* ************************************************************************** */

#ifndef _APP_ICMP_HELPER_H    /* Guard against multiple inclusion */
#define _APP_ICMP_HELPER_H

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

typedef enum{
    APP_ICMP_STATUS_SUCCESS=0,
    APP_ICMP_STATUS_BUSY=-1,
    APP_ICMP_STATUS_ERROR=-2, 
}APP_ICMP_RETVAL_t;

typedef enum{
    APP_CONN_STATUS_NULL=0,
    APP_CONN_STATUS_IN_PROGRESS,
    APP_CONN_STATUS_REACHABLE,
    APP_CONN_STATUS_NOT_REACHABLE,
}APP_CONN_STATUS_t;

APP_ICMP_RETVAL_t APP_ping(IPV4_ADDR remoteAddress,unsigned int timeoutMs,APP_CONN_STATUS_t *connStatus);
    
#ifdef __cplusplus
}
#endif

#endif /* _APP_ICMP_HELPER_H */

/* *****************************************************************************
 End of File
 */