/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Microchip technologies Inc.

  @File Name
    helpers.c

  @Summary
    helper functions for MQTT dashboard demo.

  @Description
    helper functions for MQTT dashboard demo.
 */
/* ************************************************************************** */

#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system/tmr/sys_tmr.h"
#include "tcpip/tcpip.h"
#include "app_ICMP_helper.h"
#include "system_config.h"
#include "system_definitions.h"

static bool icmpInUse = false;
static APP_CONN_STATUS_t *gConnStatus=NULL;
SYS_TMR_HANDLE tmrHandle;
ICMP_HANDLE hIcmp;

static void APP_timerCallback ( uintptr_t context, uint32_t currTick )
{
    *gConnStatus=APP_CONN_STATUS_NOT_REACHABLE;
    icmpInUse=false;
    TCPIP_ICMP_CallbackDeregister(hIcmp);
}
static void APP_ICMPCallbackFunction(TCPIP_NET_HANDLE hNetIf, IPV4_ADDR * remoteIP, void * data)
{
    *gConnStatus=APP_CONN_STATUS_REACHABLE;
    SYS_TMR_CallbackStop(tmrHandle);
    TCPIP_ICMP_CallbackDeregister(hIcmp);
    icmpInUse=false;
}

/*******************************************************************************
 * Application ICMP helper to test connectivity to an address
 * eg:
 * APP_ping(0x04020202,500,&connStatus);
 * connStatus is a bool that will be
 *  - NULL while operation is in progress
 *  - false if there is no ICMP response within timeoutMs milli seconds
 *  - true if we get a response for the ICMP request.
 * 
 * return value:
 *  - APP_ICMP_STATUS_SUCCESS : on successfully registering the ICMP request
 *  - APP_ICMP_STATUS_BUSY : if a previous operation is in progress
 *  - APP_ICMP_STATUS_SUCCESS : in case of error 
 */
APP_ICMP_RETVAL_t APP_ping(IPV4_ADDR remoteAddress,unsigned int timeoutMs,APP_CONN_STATUS_t *connStatus)
{
    if(false==icmpInUse)
    {
        icmpInUse=true;
        gConnStatus=connStatus;
        *gConnStatus=APP_CONN_STATUS_IN_PROGRESS;
    
        // Register callback function
        hIcmp = TCPIP_ICMP_CallbackRegister(&APP_ICMPCallbackFunction);
        if(0==hIcmp)
        {
            icmpInUse=false;
            return APP_ICMP_STATUS_ERROR; 
        }
        else
        {
            tmrHandle = SYS_TMR_CallbackSingle ( timeoutMs, 1, APP_timerCallback );
            if (SYS_TMR_HANDLE_INVALID == tmrHandle)
            {
                icmpInUse=false;
                return APP_ICMP_STATUS_ERROR; 
            }
            else
            {
                if(ICMP_ECHO_OK==TCPIP_ICMP_EchoRequestSend(0,&remoteAddress,1,0x1234))
                {
                    return APP_ICMP_STATUS_SUCCESS;
                }
                else
                {
                    //deregister x 2
                    SYS_TMR_CallbackStop(tmrHandle);
                    TCPIP_ICMP_CallbackDeregister(hIcmp);
                    icmpInUse=false;
                    return APP_ICMP_STATUS_ERROR; 
                }
            }
        }
    }
    else{
        return APP_ICMP_STATUS_BUSY;
    }
}

/* *****************************************************************************
 End of File
 */
