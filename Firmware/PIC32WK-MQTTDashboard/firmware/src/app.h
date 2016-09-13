/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
//DOM-IGNORE-END

#ifndef _APP_H
#define _APP_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "app_ICMP_helper.h"
#include "app_flash_helper.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif

/*WAN check configs*/
#define APP_WAN_CHECK_IP                   "4.2.2.2"
#define APP_WAN_CHECK_TIMEOUT              3000

/*User config and flash related params*/
#define APP_USER_CONFIG_FLASH_AREA         (0x001F7000+0x1000) /*4K reserved for user config*/
#define APP_MAX_THING_ID_LENGTH            20                  /*Max string length of thingID*/
#define APP_DEFAULT_SERVER_PORT            (uint16_t)80                  /*default piort if user has not configured the port*/
#define APP_DEFAULT_THING_ID_PREFIX        "PIC32WK-"          /*default prefix if user has not configured thingID*/
#define APP_WIFI_MACADDRESS_START          (0x001FE200+6)      /*we just need last 6*/
#define APP_WIFI_MACADDRESS_LENGTH          7                   /*including last \0*/

/*poll timeout token configs*/
#define APP_SWITCH_TOKEN_TIMEOUT            600                 /*switches engagte only once in x ms*/
#define APP_ANALOG_SCAN_PERIOD              300                 /*Analog conversion triggered every x ms*/

/*US related configs*/
#define APP_DEFAULT_DRIVE_NAME              "<unnamedDevice>"
#define APP_EMPTY_DRIVE_STRING              "*"

/*Cloud server related configs*/
#define APP_SERVER_LISTEN_PATH              "api/listen/to"
#define APP_SERVER_TRANSMIT_PATH            "api/mdtweet/for"
#define APP_SERRVER_PROTOCOL_STRING         "http://"
#define APP_SERVER_KEEPALIVE_TIMEOUT        1500 //50000               /*periodic timer ms to send app level keepalive*/
#define APP_SERVER_PER_UPDATE_TIMEOUT       205                 /*periodic per update delay*/
#define APP_SERVER_MAX_CON_HEADER_SIZE      (256*3)             /*3x to support slave devices (unielem)*/
#define APP_LED1_CLOUD_STAT_STRING          "led1Stat"
#define APP_LED2_CLOUD_STAT_STRING          "led2Stat"
#define APP_POT_CLOUD_STRING                "pot"
#define APP_TEMP_CLOUD_STRING               "temp"
#define APP_USB_NAME_CLOUD_STRING           "usbName"    
#define APP_USB_SERIAL_CLOUD_STRING         "usbSer"        
#define APP_LED_CLOUD_ON_STRING             "ON"
#define APP_LED_CLOUD_OFF_STRING            "OFF"
#define APP_HEARTBEAT_STRING                "hb"
#define APP_SERVER_CHECK_TIMEOUT            1000

/*mobile APP server configs*/
#define APP_MAX_NUM_CLIENT                  3
#define APP_SOC_SERVER_PORT                 16669
#define LED1_APP_STAT_STRING                "LED1"
#define LED2_APP_STAT_STRING                "LED2"
#define LED_APP_ON_STRING                   "true"
#define LED_APP_OFF_STRING                  "false"
#define MAX_JSON_SIZE                       256


// <editor-fold defaultstate="collapsed" desc="APP_TASK data">  
typedef enum
{
	APP_STATE_INIT=0,
	APP_STATE_WAIT_CONFIG,
	APP_STATE_WAIT_NET,
	APP_STATE_ACTIVE,
	APP_STATE_ERROR,
} APP_TASK_STATES;
typedef struct
{
    APP_TASK_STATES state;
} APP_TASK_DATA;
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="UC_TASK data">
typedef enum
{
	APP_UC_STATE_INIT=0,
    APP_UC_STATE_READ_CONFIG,
	APP_UC_STATE_WAIT_READ_CONFIG,
    APP_UC_STATE_VALIDATE_CONFIG,
    APP_UC_STATE_READ_MAC,
    APP_UC_STATE_WAIT_READ_MAC,
	APP_UC_STATE_IDLE,
	APP_UC_STATE_ERASE_CONFIG,
	APP_UC_STATE_WAIT_ERASE_CONFIG,
	APP_UC_STATE_WRITE_CONFIG,
	APP_UC_STATE_WAIT_WRITE_CONFIG,
	APP_UC_STATE_RESET,
    APP_UC_STATE_ERROR,
} APP_UC_TASK_STATES;
typedef struct
{
    APP_UC_TASK_STATES state;
    APP_FLASH_STATUS_t flashStat;
    char appMacAddress[APP_WIFI_MACADDRESS_LENGTH];
} APP_UC_TASK_DATA;
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="NET_MASTER_DATA">
typedef enum
{
    APP_NET_MASTER_STATE_INIT=0,
	APP_NET_MASTER_WAIT_FOR_TCPIP_INIT,
    APP_NET_MASTER_WAIT_FOR_IP,
    APP_NET_MASTER_STATE_CHECK_WAN,
    APP_NET_MASTER_STATE_CHECK_WAN_WAIT,
	APP_NET_MASTER_STATE_IDLE,
	APP_NET_MASTER_STATE_ERROR,
} APP_NET_MASTER_TASK_STATES;
typedef struct
{
    APP_NET_MASTER_TASK_STATES state;
} APP_NET_MASTER_TASK_DATA;
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="NET_SERVER_DATA">
typedef enum
{
        APP_NET_SERVER_STATE_INIT=0,
		APP_NET_SERVER_STATE_SOCKET_CREATE,
        APP_NET_SERVER_STATE_BIND,
        APP_NET_SERVER_STATE_LISTEN,
        APP_NET_SERVER_STATE_SEND_PER_STATUS,
        APP_NET_SERVER_STATE_ERROR,
} APP_NET_SERVER_TASK_STATES;
typedef struct
{
    uint32_t       clientCount;
    bool           isSocketServerActive;
    SOCKET         clientSocket[APP_MAX_NUM_CLIENT];
    SOCKET         bsdServerSocket;
    APP_NET_SERVER_TASK_STATES state;
} APP_NET_SERVER_TASK_DATA;
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="NET_CLIENT_DATA">
typedef enum
{
    APP_NET_CLIENT_STATE_INIT=0,
    APP_NET_CLIENT_STATE_CHECK_CONNECTIVITY,
    APP_NET_CLIENT_STATE_WAIT_CHECK_CONNECTIVITY,
    APP_NET_CLIENT_STATE_SOCKET_CREATE,
    APP_NET_CLIENT_STATE_CONNECT_SERVER,
    APP_NET_CLIENT_STATE_SEND,
    APP_NET_CLIENT_STATE_LISTEN,
    APP_NET_CLIENT_STATE_SEND_PER_STATUS,
    APP_NET_CLIENT_STATE_SOCKET_CLOSE,
    APP_NET_CLIENT_STATE_ERROR,
} APP_NET_CLIENT_TASK_STATES;
typedef struct
{
    SOCKET   listenSocket;
    SOCKET   transmitSocket;
    struct   sockaddr_in addr;
    char thingID[APP_MAX_THING_ID_LENGTH];
    int prevErrnoT;
    int prevErrnoL;
    bool listenSocketStat;
    bool transmitSocketStat;
    bool lastUpdateFailed;
    bool serverSockConnected;
    bool perUpdateToken;
    SYS_TMR_HANDLE keepAliveTimerHandle;
    SYS_TMR_HANDLE perTokenTimerHandle;
    APP_NET_CLIENT_TASK_STATES state;
} APP_NET_CLIENT_TASK_DATA;
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="USB_DATA">
typedef enum
{
	APP_USB_STATE_INIT=0,
	APP_USB_STATE_BUS_ENABLE,
    APP_USB_STATE_WAIT_BUS_ENABLE_COMPLETE,
    APP_USB_STATE_WAIT_DEVICE_MOUNT,
    APP_USB_STATE_DEVICE_MOUNT,
    APP_USB_STATE_UNMOUNT_DISK,
    APP_USB_STATE_READ_DATA,
    APP_USB_STATE_IDLE,
    APP_USB_STATE_ERROR
} APP_USB_TASK_STATES;
typedef struct
{
    APP_USB_TASK_STATES state;
} APP_USB_TASK_DATA;
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="ANALOG_DATA">
typedef enum
{
	APP_ANALOG_STATE_INIT=0,
	APP_ANALOG_STATE_IDLE,
    APP_ANALOG_STATE_ERROR,
} APP_ANALOG_TASK_STATES;
typedef struct
{
    SYS_TMR_HANDLE tmrHandle;
    APP_ANALOG_TASK_STATES state;
} APP_ANALOG_TASK_DATA;
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="SWITCH_DATA">

typedef enum
{
	APP_SWITCH_STATE_INIT=0,
	APP_SWITCH_STATE_WAIT_TOKEN,
	APP_SWITCH_STATE_CHECK_N_TOGGLE,
	APP_SWITCH_STATE_START_TIMER,
    APP_SWITCH_STATE_ERROR,
} APP_SWITCH_TASK_STATES;
typedef struct
{
    bool switchToken;
    SYS_TMR_HANDLE tmrHandle;
    APP_SWITCH_TASK_STATES state;
} APP_SWITCH_TASK_DATA;
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="xDATA">
typedef enum {
    APP_DEVICE_MODE_NONE=-1,
    APP_DEVICE_MODE_MASTER=0,
    APP_DEVICE_MODE_SLAVE=1,
}APP_DEVICE_MODE_t;
typedef struct
{
    IPV4_ADDR serverAddress;
    uint16_t serverPort;
    bool isPortConfigured;
    bool isThingIdConfigured;
    bool isServerConfigured;
    APP_DEVICE_MODE_t opMode;
    char thingID[APP_MAX_THING_ID_LENGTH];
}appConfigData_t;
typedef struct
{
    uint32_t potPosition;
    int32_t temperature;
    bool ledFlip;
    bool forceUpdateTrig;               /*a change in this will trigger an update*/
    bool isUsbDevMounted;
    bool isUsbDataValid;
    char usbDriveName[64];
    uint32_t usbDriveSerialNo;
}appPerData_t;
struct
{
    bool isNetUp;                       /*xData to indicate network link status*/
    APP_CONN_STATUS_t wanConnStatus;    /*xData to indicate WAN connectivity status*/
    APP_CONN_STATUS_t serverConnStatus; /*xData to indicate server connectivity status*/
    bool isConfigured;                  /*xData to indicate whether system has valid user configuration from flash*/
    bool isNewConfig;                   /*xData to indicate whether a new user configuration has been applied*/
    appConfigData_t appConfigData;      /*xData that holds user configuration data. This data will be changed by commands. Have local copies to avoid discrepancy*/
    appPerData_t appPerData;
}xData;
// </editor-fold>

/*task data instances*/
APP_TASK_DATA appTaskData;
APP_UC_TASK_DATA appUcData;
APP_NET_MASTER_TASK_DATA appNetMasterData;
APP_NET_SERVER_TASK_DATA appNetServerData;
APP_NET_CLIENT_TASK_DATA appNetClientData;
APP_USB_TASK_DATA appUsbData;
APP_ANALOG_TASK_DATA appAnalogData;
APP_SWITCH_TASK_DATA appSwitchData;

/*function prototypes*/
void APP_Initialize ( void );
void APP_Tasks( void );
void APP_TASK_UC(void);
void APP_TASK_NET_MASTER(void);
void APP_TASK_NET_SERVER(void);
void APP_TASK_NET_CLIENT(void);
void APP_TASK_USB(void);
void APP_TASK_ANALOG(void);
void APP_TASK_SWITCH(void);
#endif /* _APP_H */

#ifdef __cplusplus
}
#endif

/*******************************************************************************
 End of File
 */

