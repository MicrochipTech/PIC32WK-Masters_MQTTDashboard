/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
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
// DOM-IGNORE-END

/*
 * TODO: add periodic wan check
 * TODO: add periodic server check
 * TODO: add periodic link status checks using TCPIP_STACK_NetIsLinked(netH)
 * TODO: add a config helper??
 * TODO: add a wan check helper??
 */


#include "app.h"
#include "app_commands.h"
#include<sys/errno.h>
#include <errno.h>

void APP_Initialize(void) {
    SYS_CONSOLE_PRINT("\r\n\r\n********PIC32WK MQTT Dashboard demo (%s,%s)********\r\n", __DATE__, __TIME__);
    /*xdataInits*/
    xData.isNetUp = false;
    xData.wanConnStatus = APP_CONN_STATUS_NULL;
    xData.serverConnStatus = APP_CONN_STATUS_NULL;
    xData.isConfigured = false;
    xData.isNewConfig = false;
    xData.appConfigData.isPortConfigured = false;
    xData.appConfigData.isThingIdConfigured = false;
    xData.appConfigData.isServerConfigured = false;
    xData.appConfigData.opMode=APP_DEVICE_MODE_NONE;
    xData.appPerData.temperature = 0;
    xData.appPerData.potPosition = 0;
    xData.appPerData.ledFlip = true;
    xData.appPerData.forceUpdateTrig = true;
    xData.appPerData.isUsbDevMounted = false;
    xData.appPerData.isUsbDataValid = false;
    xData.appPerData.usbDriveSerialNo = 0;
    xData.appPerData.usbDriveName[0] ='\0';
    /*state inits*/
    appTaskData.state = APP_STATE_INIT;
    appNetMasterData.state = APP_NET_MASTER_STATE_INIT;
    appUcData.state = APP_UC_STATE_INIT;
    appSwitchData.state = APP_SWITCH_STATE_INIT;
    appAnalogData.state = APP_ANALOG_STATE_INIT;

    if (false == app_commands_init()) {
        SYS_CONSOLE_PRINT("Failed to register test commands! (%s:%u)\r\n", __FUNCTION__, __LINE__);
        BSP_LEDOn(BSP_LED_1);
    } else {
        BSP_LEDOff(BSP_LED_2);
        BSP_LEDOn(BSP_LED_1);
    }
}

// <editor-fold defaultstate="collapsed" desc="APP_TASK_UC">

void __attribute__((optimize("-O1"))) APP_TASK_UC(void) {  /*lower optimization to prevent flash variable opt*/
    switch (appUcData.state) {
            // <editor-fold defaultstate="collapsed" desc="APP_UC_STATE_INIT">
        case APP_UC_STATE_INIT:
        {
            appUcData.flashStat = APP_FLASH_STATUS_NULL;
            appUcData.state = APP_UC_STATE_READ_CONFIG;
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_UC_STATE_READ_CONFIG">
        case APP_UC_STATE_READ_CONFIG:
        {
            SYS_CMD_READY_TO_READ();
            appUcData.flashStat = APP_FLASH_STATUS_NULL;

            APP_FLASH_RETVAL_t retval;
            retval = APP_read_flash(APP_USER_CONFIG_FLASH_AREA, (uint8_t*) & xData.appConfigData, sizeof (xData.appConfigData), &appUcData.flashStat);
            switch (retval) {
                case(APP_FLASH_RETVAL_BUSY):
                    break;
                    ;
                case (APP_FLASH_RETVAL_ERROR):
                    SYS_CONSOLE_PRINT("\r\n!!!Failed to read flash (%s:%u)!\r\n", __FUNCTION__, __LINE__);
                    appUcData.state = APP_UC_STATE_ERROR;
                    break;
                case (APP_FLASH_RETVAL_SUCCESS):
                    appUcData.state = APP_UC_STATE_WAIT_READ_CONFIG;
                    break;
            }
            break;
        }
            //</editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_UC_STATE_WAIT_READ_CONFIG">
        case APP_UC_STATE_WAIT_READ_CONFIG:
        {
            SYS_CMD_READY_TO_READ();
            switch (appUcData.flashStat) {
                case APP_FLASH_STATUS_IN_PROGRESS:
                {
                    appUcData.state = APP_UC_STATE_WAIT_READ_CONFIG;
                    break;
                }
                case APP_FLASH_STATUS_ERROR:
                {
                    SYS_CONSOLE_PRINT("\r\n!!!Failed to read config from flash (%s:%u)!\r\n", __FUNCTION__, __LINE__);
                    appUcData.state = APP_UC_STATE_ERROR;
                    break;
                }
                case APP_FLASH_STATUS_SUCCESS:
                {
                    appUcData.state = APP_UC_STATE_VALIDATE_CONFIG;
                }
            }
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_UC_STATE_VALIDATE_CONFIG">
        case APP_UC_STATE_VALIDATE_CONFIG:
        {
            char serverIpAddress[18];
            bool convRetval;
            bool validServerIp = false;
            /*Validate IP address*/
            convRetval = TCPIP_Helper_IPAddressToString(&(xData.appConfigData.serverAddress), serverIpAddress, sizeof (serverIpAddress));
            if ((true == convRetval)&&(0xffffffff != xData.appConfigData.serverAddress.Val)&&((0 != xData.appConfigData.serverAddress.Val))) {
                xData.appConfigData.isServerConfigured = true;
                SYS_CONSOLE_PRINT("\r\nServer IP   : %s", serverIpAddress);
                validServerIp = true;
            } else {
                xData.appConfigData.isServerConfigured = false;
                SYS_CONSOLE_PRINT("\r\n****No valid server configuration found in flash!!****\r\n");
            }

            /*Validate port; default to APP_DEFAULT_SERVER_PORT if not configured*/
            if ((0xFF == xData.appConfigData.isPortConfigured) || (false == xData.appConfigData.isPortConfigured)) {
                xData.appConfigData.serverPort = APP_DEFAULT_SERVER_PORT;
                xData.appConfigData.isPortConfigured = true;
            }
            if (true == validServerIp) {
                SYS_CONSOLE_PRINT("\r\nServer Port : %d", xData.appConfigData.serverPort);
            }

            /*Validate thingID; if not configured, fallback to PREFIX-MAC*/
            if ((0xFF == xData.appConfigData.isThingIdConfigured) || (false == xData.appConfigData.isThingIdConfigured)) {
                appUcData.state = APP_UC_STATE_READ_MAC;
                break;
            }

            xData.isConfigured = true;
            SYS_CONSOLE_PRINT("\r\nthingID     : %s", xData.appConfigData.thingID);
            SYS_CONSOLE_PRINT("\r\nmode        : %d\r\n\r\n", xData.appConfigData.opMode);
            appUcData.state = APP_UC_STATE_IDLE;
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_UC_STATE_READ_MAC">
        case APP_UC_STATE_READ_MAC:
        {
            SYS_CMD_READY_TO_READ();
            appUcData.flashStat = APP_FLASH_STATUS_NULL;
            APP_FLASH_RETVAL_t retval;

            retval = APP_read_flash(APP_WIFI_MACADDRESS_START, (uint8_t*) & appUcData.appMacAddress, sizeof (appUcData.appMacAddress), &appUcData.flashStat);
            switch (retval) {
                case(APP_FLASH_RETVAL_BUSY):
                    break;
                case (APP_FLASH_RETVAL_ERROR):
                    SYS_CONSOLE_PRINT("\r\n!!!Failed to read MAC from flash! (%s:%u)\r\n", __FUNCTION__, __LINE__);
                    appUcData.state = APP_UC_STATE_ERROR;
                    break;
                case (APP_FLASH_RETVAL_SUCCESS):
                    appUcData.state = APP_UC_STATE_WAIT_READ_MAC;
                    break;
            }
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_UC_STATE_WAIT_READ_MAC">
        case APP_UC_STATE_WAIT_READ_MAC:
        {
            SYS_CMD_READY_TO_READ();
            switch (appUcData.flashStat) {
                case APP_FLASH_STATUS_IN_PROGRESS:
                {
                    appUcData.state = APP_UC_STATE_WAIT_READ_MAC;
                    break;
                }
                case APP_FLASH_STATUS_ERROR:
                {
                    SYS_CONSOLE_PRINT("\r\n!!!Failed to read MAC from flash (cb)! (%s:%u)\r\n", __FUNCTION__, __LINE__);
                    appUcData.state = APP_UC_STATE_ERROR;
                    break;
                }
                case APP_FLASH_STATUS_SUCCESS:
                {
                    snprintf(xData.appConfigData.thingID, sizeof (xData.appConfigData.thingID), APP_DEFAULT_THING_ID_PREFIX"%s", appUcData.appMacAddress);
                    xData.appConfigData.isThingIdConfigured = true;
                    xData.isConfigured = true;
                    xData.appConfigData.opMode=APP_DEVICE_MODE_MASTER;  /*if thingID is self configured, be in master mode*/
                    SYS_CONSOLE_PRINT("\r\nthingID: %s\r\n\r\n", xData.appConfigData.thingID);
                    SYS_CONSOLE_PRINT("\r\nmode        : %d\r\n\r\n", xData.appConfigData.opMode);
                    appUcData.state = APP_UC_STATE_IDLE;
                }
            }
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_UC_STATE_IDLE">
        case APP_UC_STATE_IDLE:
        {
            SYS_CMD_READY_TO_READ();
            if (false == xData.isNewConfig) {
                break;
            } else {
                xData.isConfigured = false;
                appUcData.state = APP_UC_STATE_ERASE_CONFIG;
                break;
            }
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_UC_STATE_ERASE_CONFIG">
        case APP_UC_STATE_ERASE_CONFIG:
        {
            appUcData.flashStat = APP_FLASH_STATUS_NULL;

            APP_FLASH_RETVAL_t retval;
            retval = APP_erase_flash(APP_USER_CONFIG_FLASH_AREA, 1, &appUcData.flashStat);
            switch (retval) {
                case(APP_FLASH_RETVAL_BUSY):
                    break;
                    ;
                case (APP_FLASH_RETVAL_ERROR):
                    SYS_CONSOLE_PRINT("\r\n!!!Failed to erase flash! (%s,%u)\r\n", __FUNCTION__, __LINE__);
                    appUcData.state = APP_UC_STATE_ERROR;
                    break;
                case (APP_FLASH_RETVAL_SUCCESS):
                    appUcData.state = APP_UC_STATE_WAIT_ERASE_CONFIG;
                    break;
            }
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_UC_STATE_WAIT_ERASE_CONFIG">
        case APP_UC_STATE_WAIT_ERASE_CONFIG:
        {
            switch (appUcData.flashStat) {
                case APP_FLASH_STATUS_IN_PROGRESS:
                {
                    break;
                }
                case APP_FLASH_STATUS_ERROR:
                {
                    SYS_CONSOLE_PRINT("\r\n!!!Failed to erase config from flash! (%s:%u)\r\n", __FUNCTION__, __LINE__);
                    appUcData.state = APP_UC_STATE_ERROR;
                    break;
                }
                case APP_FLASH_STATUS_SUCCESS:
                {
                    appUcData.state = APP_UC_STATE_WRITE_CONFIG;
                    break;
                }
            }
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_UC_STATE_WRITE_CONFIG">
        case APP_UC_STATE_WRITE_CONFIG:
        {
            appUcData.flashStat = APP_FLASH_STATUS_NULL;

            APP_FLASH_RETVAL_t retval;
            retval = APP_write_flash(APP_USER_CONFIG_FLASH_AREA, (uint8_t*) & xData.appConfigData, sizeof (xData.appConfigData), &appUcData.flashStat);
            switch (retval) {
                case(APP_FLASH_RETVAL_BUSY):
                    break;
                    ;
                case (APP_FLASH_RETVAL_ERROR):
                    SYS_CONSOLE_PRINT("\r\n!!!Failed to write flash! (%s:%u)\r\n", __FUNCTION__, __LINE__);
                    appUcData.state = APP_UC_STATE_ERROR;
                    break;
                case (APP_FLASH_RETVAL_SUCCESS):
                    appUcData.state = APP_UC_STATE_WAIT_WRITE_CONFIG;
                    break;
            }
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_UC_STATE_WAIT_WRITE_CONFIG">
        case APP_UC_STATE_WAIT_WRITE_CONFIG:
        {
            switch (appUcData.flashStat) {
                case APP_FLASH_STATUS_IN_PROGRESS:
                {
                    break;
                }
                case APP_FLASH_STATUS_ERROR:
                {
                    SYS_CONSOLE_PRINT("\r\n!!!Failed to write config to flash! (%s,%u)\r\n", __FUNCTION__, __LINE__);
                    appUcData.state = APP_UC_STATE_ERROR;
                    break;
                }
                case APP_FLASH_STATUS_SUCCESS:
                {
                    SYS_CONSOLE_MESSAGE("\r\n!!!New config saved. Resetting device!\r\n");
                    appUcData.state = APP_UC_STATE_RESET;
                    break;
                }
            }
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_UC_STATE_RESET">
        case APP_UC_STATE_RESET:
        {
            SYS_RESET_SoftwareReset();
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_UC_STATE_ERROR">
        case APP_UC_STATE_ERROR:
        {
            SYS_CMD_READY_TO_READ();
            BSP_LEDOn(BSP_LED_1);
            break;
        }
            // </editor-fold>
        default:
        {
            break;
        }
    }
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="APP_TASK_NET_MASTER">

void APP_TASK_NET_MASTER(void) {
    static bool flash_open_status = 0;
    switch (appNetMasterData.state) {
            // <editor-fold defaultstate="collapsed" desc="APP_NET_MASTER_STATE_INIT">
        case APP_NET_MASTER_STATE_INIT:
        {
            SYS_CMD_READY_TO_READ();
            flash_open_status = WIFI_initialization();
            if (flash_open_status) {
                appNetMasterData.state = APP_NET_MASTER_WAIT_FOR_TCPIP_INIT;
            }
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_NET_MASTER_WAIT_FOR_TCPIP_INIT">
        case APP_NET_MASTER_WAIT_FOR_TCPIP_INIT:
        {
            SYS_CMD_READY_TO_READ();
            SYS_STATUS tcpipStat;
            tcpipStat = TCPIP_STACK_Status(sysObj.tcpip);
            const char *netName, *netBiosName;
            int i, nNets;
            TCPIP_NET_HANDLE netH;
            if (tcpipStat < 0) { // some error occurred
                SYS_CONSOLE_MESSAGE("APP: TCP/IP stack initialization failed!\r\n");
                appNetMasterData.state = APP_NET_MASTER_STATE_ERROR;
                break;
            } else if (tcpipStat == SYS_STATUS_READY) {
                nNets = TCPIP_STACK_NumberOfNetworksGet();
                for (i = 0; i < nNets; i++) {
                    netH = TCPIP_STACK_IndexToNet(i);
                    TCPIP_STACK_NetBiosNameSet(netH, xData.appConfigData.thingID);  /*setting on all interfaces for now*/
                    netName = TCPIP_STACK_NetNameGet(netH);
                    netBiosName = TCPIP_STACK_NetBIOSName(netH);
                    SYS_CONSOLE_PRINT("Interface %s on host %s - NBNS enabled\r\n", netName, netBiosName);
                }
                appNetMasterData.state = APP_NET_MASTER_WAIT_FOR_IP; //APP_NETWORK_WIFI_CONNECT;
                SYS_CONSOLE_MESSAGE("\r\nWaiting for IP");
            }
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_NET_MASTER_WAIT_FOR_IP">
        case APP_NET_MASTER_WAIT_FOR_IP:
        {
            SYS_CMD_READY_TO_READ();

            int i, nNets;
            TCPIP_NET_HANDLE netH;
            IPV4_ADDR ipAddr;
            static IPV4_ADDR dwLastIP[2] = {
                {-1},
                {-1}
            }; // array size depends on net interface number
            // display if IP address has changed.
            nNets = TCPIP_STACK_NumberOfNetworksGet();

            for (i = 0; i < nNets; i++) {
                netH = TCPIP_STACK_IndexToNet(i);
                if (true == TCPIP_DHCP_IsEnabled(netH)) {
                    if (false == TCPIP_DHCP_IsBound(netH)) //wait until all DHCP enabled interfaces have obtained IP
                    {
                        return;
                    }
                }
            }
            for (i = 0; i < nNets; i++) {
                netH = TCPIP_STACK_IndexToNet(i);
                ipAddr.Val = TCPIP_STACK_NetAddress(netH);
                if (dwLastIP[i].Val != ipAddr.Val) {
                    dwLastIP[i].Val = ipAddr.Val;
                    SYS_CONSOLE_MESSAGE(TCPIP_STACK_NetNameGet(netH));
                    SYS_CONSOLE_MESSAGE("\r\nIP Address: ");
                    SYS_CONSOLE_PRINT("%d.%d.%d.%d \r\n", ipAddr.v[0], ipAddr.v[1], ipAddr.v[2], ipAddr.v[3]);
                }
            }
            xData.isNetUp = true;
                /*** Random Service Initialization Code for further ports ***/
                SYS_RANDOM_INIT rngInit = {0x12345678, NULL, 32};
                SYS_MODULE_OBJ rngObject;
                rngInit.seedPseudo = (uint32_t) (TMR2 + DRV_ADC_SamplesRead((uint8_t) ADCHS_AN15));
                rngObject = SYS_RANDOM_Initialize(0, (SYS_MODULE_INIT*) & rngInit);

            appNetMasterData.state = APP_NET_MASTER_STATE_CHECK_WAN;
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_NET_MASTER_STATE_CHECK_WAN">
        case APP_NET_MASTER_STATE_CHECK_WAN:
        {
            SYS_CMD_READY_TO_READ();
            APP_ICMP_RETVAL_t wanPingStatus;
            IPV4_ADDR wanCheckIp;

            TCPIP_Helper_StringToIPAddress(APP_WAN_CHECK_IP, &wanCheckIp);
            wanPingStatus = APP_ping(wanCheckIp, APP_WAN_CHECK_TIMEOUT, &xData.wanConnStatus);
            switch (wanPingStatus) {
                case(APP_ICMP_STATUS_BUSY):
                {
                    break;
                }
                case(APP_ICMP_STATUS_SUCCESS):
                {
                    SYS_CONSOLE_MESSAGE("\r\nTesting WAN Connection:  ");
                    appNetMasterData.state = APP_NET_MASTER_STATE_CHECK_WAN_WAIT;
                    break;
                }
                case(APP_ICMP_STATUS_ERROR):
                {
                    SYS_CONSOLE_PRINT("\r\nICMP ERROR - please reset (%s:%u)", __FUNCTION__, __LINE__);
                    appNetMasterData.state = APP_NET_MASTER_STATE_ERROR;
                }
            }
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_NET_MASTER_STATE_CHECK_WAN_WAIT">
        case APP_NET_MASTER_STATE_CHECK_WAN_WAIT:
        {
            if (APP_CONN_STATUS_IN_PROGRESS == xData.wanConnStatus) {
                break;
            } else {
                if (APP_CONN_STATUS_REACHABLE == xData.wanConnStatus) {
                    SYS_CONSOLE_MESSAGE("Connected to WAN\r\n");
                } else if (APP_CONN_STATUS_NOT_REACHABLE == xData.wanConnStatus) {
                    SYS_CONSOLE_MESSAGE("Not connected to WAN\r\n");
                }
                appNetMasterData.state = APP_NET_MASTER_STATE_IDLE;
            }
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_NET_MASTER_STATE_IDLE">
        case APP_NET_MASTER_STATE_IDLE:
        {
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_NET_MASTER_STATE_ERROR">
        case APP_NET_MASTER_STATE_ERROR:
        {
            xData.isNetUp = false;
            BSP_LEDOn(BSP_LED_1);
            break;
        }
            // </editor-fold>
        default:
        {
            break;
        }
    }
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="APP_TASK_NET_SERVER">

// <editor-fold defaultstate="collapsed" desc="_APP_process_app_Json">

void _APP_process_app_Json(const char *recvBuffer) {
    char *deviceRet = NULL;
    char *stateRet = NULL;
    unsigned char err = 1;
    BSP_LED controlLed;

    /*get starting point of device name string in JSON*/
    deviceRet = strstr(recvBuffer, LED1_APP_STAT_STRING);
    if (NULL != deviceRet) {
        err = 0;
        controlLed = BSP_LED_2;
        SYS_CONSOLE_MESSAGE("\r\nLED1 ");
    } else {
        deviceRet = strstr(recvBuffer, LED2_APP_STAT_STRING);
        if (NULL != deviceRet) {
            err = 0;
            controlLed = BSP_LED_3;
            SYS_CONSOLE_MESSAGE("\r\nLED2 ");
        }
    }

    /*Start parsing from device name to get ON/OFF token if a LED name is received*/
    if (0 == err) {
        stateRet = strstr((const char*) deviceRet, LED_APP_ON_STRING);
        if (NULL != stateRet) {
            BSP_LEDOn(controlLed);
            SYS_CONSOLE_MESSAGE("ON (APP)");
            xData.appPerData.ledFlip=!xData.appPerData.ledFlip;
        } else {
            stateRet = strstr((const char*) deviceRet, LED_APP_OFF_STRING);
            if (NULL != stateRet) {
                BSP_LEDOff(controlLed);
                SYS_CONSOLE_MESSAGE("OFF (APP)");
                xData.appPerData.ledFlip=!xData.appPerData.ledFlip;
            }
        }
    }
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="_APP_mobile_app_update">

void _APP_mobile_app_update() {
    //app update
    if (0 != appNetServerData.clientCount) {
        char buffer[MAX_JSON_SIZE];
        int j;
        
        //if (APP_DEVICE_MODE_MASTER==xData.appConfigData.opMode){
        sprintf(buffer, "[{\"cmd\":\"LED1\",\"demo\":1,\"value\":%d},"
                "{\"cmd\":\"LED2\",\"demo\":1,\"value\":%d},"
                "{\"cmd\":\"temp\",\"demo\":1,\"value\":%d},"
                "{\"cmd\":\"potentiomenet\",\"demo\":1,\"value\":%d},"
                "{\"cmd\":\"usbname\",\"demo\":1,\"value\":\"%s\"},"
                "{\"cmd\":\"usbno\",\"demo\":1,\"value\":\"%u\"}]",\
                           BSP_LEDStateGet(BSP_LED_2), BSP_LEDStateGet(BSP_LED_3),
                           xData.appPerData.temperature,xData.appPerData.potPosition,
                           xData.appPerData.usbDriveName,xData.appPerData.usbDriveSerialNo);
        //}
        //else if (APP_DEVICE_MODE_SLAVE==xData.appConfigData.opMode){
        //    sprintf(buffer, "[{\"cmd\":\"LED1\",\"demo\":1,\"value\":%d},"
        //        "{\"cmd\":\"LED2\",\"demo\":1,\"value\":%d}]",\
        //                   BSP_LEDStateGet(BSP_LED_2), BSP_LEDStateGet(BSP_LED_3));
        //}
        //send to all connected clients
        for (j = 0; j < APP_MAX_NUM_CLIENT; j++) {
            if (appNetServerData.clientSocket[j] == INVALID_SOCKET) {
                continue;
            } else {
                send(appNetServerData.clientSocket[j], (const char*) buffer, strlen((char*) buffer), 0);
            }
        }
    }
}
// </editor-fold>

void APP_TASK_NET_SERVER(void) {
    switch (appNetServerData.state) {
            // <editor-fold defaultstate="collapsed" desc="APP_NET_SERVER_STATE_INIT">
        case APP_NET_SERVER_STATE_INIT:
        {
            SYS_CMD_READY_TO_READ();
            appNetServerData.clientCount = 0;
            appNetServerData.isSocketServerActive = false;
            appNetServerData.state = APP_NET_SERVER_STATE_SOCKET_CREATE;
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_NET_SERVER_STATE_SOCKET_CREATE">
        case APP_NET_SERVER_STATE_SOCKET_CREATE:
        {
            SYS_CMD_READY_TO_READ();
            /*init and create socket for mobile APP communication*/
            int i = 0;
            int tcpSkt;

            /*initialize all sockets to invalid*/
            for (i = 0; i < APP_MAX_NUM_CLIENT; i++)
                appNetServerData.clientSocket[i] = INVALID_SOCKET;

            tcpSkt = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
            if (tcpSkt == INVALID_SOCKET) {
                SYS_CONSOLE_MESSAGE("\r\nsocket server: INVALID_SOCKET\r\n");
                appNetServerData.state = APP_NET_SERVER_STATE_ERROR;
                break;
            }
            appNetServerData.bsdServerSocket = (SOCKET) tcpSkt;
            appNetServerData.state = APP_NET_SERVER_STATE_BIND;
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_NET_SERVER_STATE_BIND">
        case APP_NET_SERVER_STATE_BIND:
        {
            SYS_CMD_READY_TO_READ();
            /*Bind socket to a local port*/
            SOCKADDR_IN addr;
            addr.sin_family = AF_INET;
            addr.sin_port = APP_SOC_SERVER_PORT;
            addr.sin_addr.S_un.S_addr = IP_ADDR_ANY;
            if (bind(appNetServerData.bsdServerSocket, (struct sockaddr*) &addr, sizeof (SOCKADDR_IN)) == SOCKET_ERROR) {
                SYS_CONSOLE_PRINT("\r\nserver port bind() failed: %s\r\n", strerror(errno));
                appNetServerData.state = APP_NET_SERVER_STATE_ERROR;
                break;
            }
            if (listen(appNetServerData.bsdServerSocket, APP_MAX_NUM_CLIENT) == 0) {
                SYS_CONSOLE_PRINT("\r\nWaiting for Client Connection on port: %d\r\n", APP_SOC_SERVER_PORT);
                appNetServerData.state = APP_NET_SERVER_STATE_LISTEN;
                break;
            }
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_NET_SERVER_STATE_LISTEN">
        case APP_NET_SERVER_STATE_LISTEN:
        {
            SYS_CMD_READY_TO_READ();
            /*socket server operation*/
            int i = 0;
            struct sockaddr_in addRemote;
            int addrlen = sizeof (struct sockaddr_in);
            char recvBuffer[128];
            int length;

            for (i = 0; i < APP_MAX_NUM_CLIENT; i++) {
                /*Accept any pending connection requests, assuming we have a place to store the socket descriptor*/
                if (appNetServerData.clientSocket[i] == INVALID_SOCKET) {
                    appNetServerData.clientSocket[i] = accept(appNetServerData.bsdServerSocket, (struct sockaddr*) &addRemote, &addrlen);
                    if (appNetServerData.clientSocket[i] != INVALID_SOCKET) {
                        appNetServerData.clientCount += 1;
                        SYS_CONSOLE_PRINT("\r\nclientCount=%d\r\n", appNetServerData.clientCount);
                        
                        /*send update as soon as a client connects*/
                        xData.appPerData.forceUpdateTrig=!xData.appPerData.forceUpdateTrig;     
                    }
                }

                // If this socket is not connected then no need to process anything
                if (appNetServerData.clientSocket[i] == INVALID_SOCKET)
                    continue;

                // For all connected sockets, receive and process the data
                length = recv(appNetServerData.clientSocket[i], recvBuffer, sizeof (recvBuffer), 0);

                if (length > 0) {
                    recvBuffer[length] = '\0';
                    _APP_process_app_Json((const char*) recvBuffer);
                } else if (length == 0 || errno != EWOULDBLOCK) {
                    closesocket(appNetServerData.clientSocket[i]);
                    appNetServerData.clientSocket[i] = INVALID_SOCKET;
                    appNetServerData.clientCount -= 1;
                    SYS_CONSOLE_PRINT("\r\nclientCount=%d\r\n", appNetServerData.clientCount);
                }
            }
            appNetServerData.state = APP_NET_SERVER_STATE_SEND_PER_STATUS;
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_NET_SERVER_STATE_SEND_PER_STATUS">
        case APP_NET_SERVER_STATE_SEND_PER_STATUS:
        {
            static appPerData_t prevPerData;
            if (0 != memcmp(&prevPerData, &xData.appPerData, sizeof (appPerData_t))) {
                prevPerData = xData.appPerData;
                _APP_mobile_app_update();
            }
            appNetServerData.state = APP_NET_SERVER_STATE_LISTEN;
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_NET_SERVER_STATE_ERROR">
        case APP_NET_SERVER_STATE_ERROR:
        {
            BSP_LEDOn(BSP_LED_1);
            appNetServerData.isSocketServerActive = false;
            appNetServerData.state = APP_NET_SERVER_STATE_ERROR;
            break;
        }
            // </editor-fold>
        default:
        {
            break;
        }
    }
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="APP_TASK_NET_CLIENT">

// <editor-fold defaultstate="collapsed" desc="_APP_process_cloud_Json">

int32_t _APP_process_cloud_Json(const char *recvBuffer) {
    char *deviceRet = NULL;
    char *stateRet = NULL;
    unsigned char err = 1;
    BSP_LED controlLed;

    if (strstr(recvBuffer, "MULTI")) //parse only uni-elem messages
    {
        return 0;
    } else if (strstr(recvBuffer, "StatREQ")) {
        if(APP_DEVICE_MODE_MASTER==xData.appConfigData.opMode){
            xData.appPerData.forceUpdateTrig = !xData.appPerData.forceUpdateTrig; /*flip the trigger in the bag*/
            xData.appPerData.ledFlip=!xData.appPerData.ledFlip; /*flip LED stat as well*/
        }
        return 0;
    }
    /*get starting point of device name string in JSON*/
    deviceRet = strstr(recvBuffer, APP_LED1_CLOUD_STAT_STRING);
    if (NULL != deviceRet) {
        err = 0;
        controlLed = BSP_LED_2;
        SYS_CONSOLE_MESSAGE("\r\nLED1 ");
    } else {
        deviceRet = strstr(recvBuffer, APP_LED2_CLOUD_STAT_STRING);
        if (NULL != deviceRet) {
            err = 0;
            controlLed = BSP_LED_3;
            SYS_CONSOLE_MESSAGE("\r\nLED2 ");
        }
    }

    /*Start parsing from device name to get ON/OFF token if a LED name is received*/
    if (0 == err) {
        stateRet = strstr((const char*) deviceRet, APP_LED_CLOUD_ON_STRING);
        if (NULL != stateRet) {
            BSP_LEDOn(controlLed);
            SYS_CONSOLE_MESSAGE("ON (CLOUD)");
        } else {
            stateRet = strstr((const char*) deviceRet, APP_LED_CLOUD_OFF_STRING);
            if (NULL != stateRet) {

                BSP_LEDOff(controlLed);
                SYS_CONSOLE_MESSAGE("OFF (CLOUD)");
            }
        }
    }
    return 0;
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="_APP_send_per_status">

void _APP_send_per_status(void) {
    if (true == appNetClientData.transmitSocketStat) {
        char buffer[APP_SERVER_MAX_CON_HEADER_SIZE];
        char *led1Str, *led2Str, *usbString;

        static bool pLedFlip=false;
        int length = 0;        
        led1Str = (BSP_LED_STATE_ON == BSP_LEDStateGet(BSP_LED_2)) ? APP_LED_CLOUD_ON_STRING : APP_LED_CLOUD_OFF_STRING;
        led2Str = (BSP_LED_STATE_ON == BSP_LEDStateGet(BSP_LED_3)) ? APP_LED_CLOUD_ON_STRING : APP_LED_CLOUD_OFF_STRING;
        usbString = (true == xData.appPerData.isUsbDataValid) ? xData.appPerData.usbDriveName : APP_EMPTY_DRIVE_STRING;

        /*this is already validated*/
        char serverIpAddress[18];
        TCPIP_Helper_IPAddressToString((IPV4_ADDR*)&(appNetClientData.addr.sin_addr.S_un.S_addr), serverIpAddress, sizeof (serverIpAddress));

        if(APP_DEVICE_MODE_MASTER==xData.appConfigData.opMode){
        length+=snprintf(buffer, APP_SERVER_MAX_CON_HEADER_SIZE-length,
                "GET /"APP_SERVER_TRANSMIT_PATH"/%s?ctx=MULTI&"
                APP_POT_CLOUD_STRING"=%d&"
                APP_TEMP_CLOUD_STRING"=%d&"
                APP_USB_NAME_CLOUD_STRING"=%s&"
                APP_USB_SERIAL_CLOUD_STRING"=%u HTTP/1.1\r\n"
                "Host: %s\r\n"
                "Connection: keep-alive\r\n\r\n", appNetClientData.thingID,
                xData.appPerData.potPosition,
                xData.appPerData.temperature, usbString,
                xData.appPerData.usbDriveSerialNo,
                serverIpAddress);
        }

        /*LED stat to account for Master and Slave behavior*/
        if(pLedFlip!=xData.appPerData.ledFlip){
        /*send additional uni-message for slave sync LED1*/
        length+=snprintf(buffer+length,APP_SERVER_MAX_CON_HEADER_SIZE-length,
                "GET /"APP_SERVER_TRANSMIT_PATH"/%s?"
                APP_LED1_CLOUD_STAT_STRING"=%s HTTP/1.1\r\n"
                "Host: %s\r\n"
                "Connection: keep-alive\r\n\r\n", appNetClientData.thingID, led1Str ,serverIpAddress);

        length+=snprintf(buffer+length,APP_SERVER_MAX_CON_HEADER_SIZE-length,
                "GET /"APP_SERVER_TRANSMIT_PATH"/%s?"
                APP_LED2_CLOUD_STAT_STRING"=%s HTTP/1.1\r\n"
                "Host: %s\r\n"
                "Connection: keep-alive\r\n\r\n", appNetClientData.thingID, led2Str ,serverIpAddress);
        
        pLedFlip=xData.appPerData.ledFlip;
        }
        
        buffer[APP_SERVER_MAX_CON_HEADER_SIZE - 1] = '\0'; /*safety net*/
        if(0!=length){
            send(appNetClientData.transmitSocket, (const char*) buffer, strlen((char*) buffer), 0);
        }
        return;
    }
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="APP_keepAlive_tmrCallback">

void APP_keepAlive_tmrCallback(uintptr_t context, uint32_t currTick) {
    static int heartBeatVal=1;
    if (true == appNetClientData.transmitSocketStat) {
        char buffer[APP_SERVER_MAX_CON_HEADER_SIZE];
        char serverIpAddress[18];

        /*this is already validated*/
        TCPIP_Helper_IPAddressToString((IPV4_ADDR*)&(appNetClientData.addr.sin_addr.S_un.S_addr), serverIpAddress, sizeof (serverIpAddress));
        
        heartBeatVal=!heartBeatVal;
        if(APP_DEVICE_MODE_MASTER==xData.appConfigData.opMode){
        sprintf(buffer, "GET %s%s/%s/%s?"APP_HEARTBEAT_STRING"=%d HTTP/1.1\r\n"
                "Host: %s\r\n"
                "Connection: keep-alive\r\n\r\n", APP_SERRVER_PROTOCOL_STRING, serverIpAddress,
                APP_SERVER_TRANSMIT_PATH, appNetClientData.thingID,heartBeatVal, serverIpAddress);
        }
        
        else if (APP_DEVICE_MODE_SLAVE==xData.appConfigData.opMode){    /*only master sends HB*/
        sprintf(buffer, "GET %s%s/%s/%s HTTP/1.1\r\n"
                "Host: %s\r\n"
                "Connection: keep-alive\r\n\r\n", APP_SERRVER_PROTOCOL_STRING, serverIpAddress,
                APP_SERVER_TRANSMIT_PATH, appNetClientData.thingID,serverIpAddress);
        }
        
        buffer[APP_SERVER_MAX_CON_HEADER_SIZE - 1] = '\0'; /*safety net*/
        send(appNetClientData.transmitSocket, (const char*) buffer, strlen((char*) buffer), 0);
    }
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="APP_perToken_tmrCallback">

void APP_perToken_tmrCallback(uintptr_t context, uint32_t currTick) {
    appNetClientData.perUpdateToken = true;
}
// </editor-fold>

void APP_TASK_NET_CLIENT(void) {
    switch (appNetClientData.state) {
            // <editor-fold defaultstate="collapsed" desc="APP_NET_CLIENT_STATE_INIT">
        case APP_NET_CLIENT_STATE_INIT:
        {
            SYS_CMD_READY_TO_READ();
            /*init SM specific data.*/
            appNetClientData.prevErrnoT = -1;
            appNetClientData.prevErrnoL = -1;
            appNetClientData.listenSocketStat = false;
            appNetClientData.transmitSocketStat = false;
            appNetClientData.lastUpdateFailed = true;
            appNetClientData.serverSockConnected = false;
            appNetClientData.perUpdateToken = true;

            /*Check if all preconditions are met*/
            if ((false == xData.isConfigured) ||
                    (false == xData.appConfigData.isThingIdConfigured) ||
                    (false == xData.appConfigData.isPortConfigured) ||
                    (false == xData.appConfigData.isServerConfigured)) {
                appNetClientData.state = APP_NET_CLIENT_STATE_INIT;
                break;
            } else { /*Take a local copy of config data since it might change on the run*/
                appNetClientData.addr.sin_addr.S_un.S_addr = (uint32_t) xData.appConfigData.serverAddress.Val;
                appNetClientData.addr.sin_port = xData.appConfigData.serverPort;
                strncpy(appNetClientData.thingID, xData.appConfigData.thingID, APP_MAX_THING_ID_LENGTH);

                /*app level periodic keepalive timer*/
                appNetClientData.keepAliveTimerHandle = SYS_TMR_CallbackPeriodic(APP_SERVER_KEEPALIVE_TIMEOUT, 0, APP_keepAlive_tmrCallback);
                if (SYS_TMR_HANDLE_INVALID == appNetClientData.keepAliveTimerHandle) {
                    SYS_CONSOLE_PRINT("Failed to set periodic keepalive timer (%s:%u)", __FUNCTION__, __LINE__);
                    appNetClientData.state = APP_NET_CLIENT_STATE_ERROR;
                    break;
                }

                /*TODO: init random number*/
                appNetClientData.state = APP_NET_CLIENT_STATE_CHECK_CONNECTIVITY;
                break;
            }
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_NET_CLIENT_STATE_CHECK_CONNECTIVITY">
        case APP_NET_CLIENT_STATE_CHECK_CONNECTIVITY:
        {
            SYS_CMD_READY_TO_READ();
            APP_ICMP_RETVAL_t serverPingStatus;

            /*start a connection to the server only if it is reachable*/
            serverPingStatus = APP_ping((IPV4_ADDR) appNetClientData.addr.sin_addr.S_un.S_addr, APP_SERVER_CHECK_TIMEOUT, &xData.serverConnStatus);
            switch (serverPingStatus) {
                case(APP_ICMP_STATUS_BUSY):
                {
                    break;
                }
                case(APP_ICMP_STATUS_SUCCESS):
                {
                    SYS_CONSOLE_MESSAGE("\r\nTesting Server Connection:  ");
                    appNetClientData.state = APP_NET_CLIENT_STATE_WAIT_CHECK_CONNECTIVITY;
                    break;
                }
                case(APP_ICMP_STATUS_ERROR):
                {
                    SYS_CONSOLE_PRINT("\r\nICMP ERROR - please reset (%s:%u)", __FUNCTION__, __LINE__);
                    appNetClientData.state = APP_NET_CLIENT_STATE_ERROR;
                }
            }
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_NET_CLIENT_STATE_WAIT_CHECK_CONNECTIVITY">
        case APP_NET_CLIENT_STATE_WAIT_CHECK_CONNECTIVITY:
        {
            if (APP_CONN_STATUS_IN_PROGRESS == xData.serverConnStatus) {
                break;
            } else {
                if (APP_CONN_STATUS_REACHABLE == xData.serverConnStatus) {
                    SYS_CONSOLE_MESSAGE("Server reachable.\r\n");
                    appNetClientData.state = APP_NET_CLIENT_STATE_SOCKET_CREATE;
                    break;
                } else if (APP_CONN_STATUS_NOT_REACHABLE == xData.serverConnStatus) {
                    SYS_CONSOLE_PRINT("Server unreachable. Please reconfigure or reset. [we are %s to WAN]\r\n",
                            ((APP_CONN_STATUS_REACHABLE == xData.wanConnStatus) ? "connected" : "not connected"));
                    appNetClientData.state = APP_NET_CLIENT_STATE_ERROR;
                    break;
                }
            }
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_NET_CLIENT_STATE_SOCKET_CREATE">
        case APP_NET_CLIENT_STATE_SOCKET_CREATE:
        {
            SYS_CMD_READY_TO_READ();
            int tcpSkt;
            /* Create a socket for this client to listen */
            if ((tcpSkt = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) == SOCKET_ERROR) {
                SYS_CONSOLE_PRINT("\r\nFailed to create listen socket (%s:%u)\r\n", __FUNCTION__, __LINE__);
                appNetClientData.state = APP_NET_CLIENT_STATE_ERROR;
                break;
            } else {
                appNetClientData.listenSocket = (SOCKET) tcpSkt;
            }

            /* Create a socket for the control pipe (KA etc) */
            if ((tcpSkt = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) == SOCKET_ERROR) {
                SYS_CONSOLE_PRINT("\r\nFailed to create transmit socket (%s:%u)\r\n", __FUNCTION__, __LINE__);
                appNetClientData.state = APP_NET_CLIENT_STATE_ERROR;
                break;
            } else {
                appNetClientData.transmitSocket = (SOCKET) tcpSkt;
            }

            SYS_CONSOLE_PRINT("\r\nClient [%s]: Connecting...\r\n", appNetClientData.thingID);
            appNetClientData.state = APP_NET_CLIENT_STATE_CONNECT_SERVER;
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_NET_CLIENT_STATE_CONNECT_SERVER">
        case APP_NET_CLIENT_STATE_CONNECT_SERVER:
        {
            SYS_CMD_READY_TO_READ();
            int addrlen;
            TCP_SOCKET_INFO tcpSktInfo;
            addrlen = sizeof (struct sockaddr);

            /*Listen socket*/
            if (connect(appNetClientData.listenSocket, (struct sockaddr*) &appNetClientData.addr, addrlen) < 0) {
                if (errno != EINPROGRESS) {
                    SYS_CONSOLE_PRINT("\r\nListenSocket closing : %s\r\n", strerror(errno));
                    appNetClientData.state = APP_NET_CLIENT_STATE_ERROR; // true error/close condition
                    break;
                }

                if (appNetClientData.prevErrnoL != errno) {
                    TCPIP_TCP_SocketInfoGet(appNetClientData.listenSocket, &tcpSktInfo);
                    SYS_CONSOLE_PRINT("\r\nWaiting for 3WH from [%d] (ListenSocket)...", tcpSktInfo.localPort);
                    appNetClientData.prevErrnoL = errno;
                }
                break;
            } else {
                appNetClientData.listenSocketStat = true;
            }
            /*Transmit socket*/
            if (connect(appNetClientData.transmitSocket, (struct sockaddr*) &appNetClientData.addr, addrlen) < 0) {
                if (errno != EINPROGRESS) {
                    SYS_CONSOLE_PRINT("\r\nTransmitSocket closing : %s\r\n", strerror(errno));
                    appNetClientData.state = APP_NET_CLIENT_STATE_ERROR; // true error/close condition
                    break;
                }
                if (appNetClientData.prevErrnoT != errno) {
                    TCPIP_TCP_SocketInfoGet(appNetClientData.transmitSocket, &tcpSktInfo);
                    SYS_CONSOLE_PRINT("\r\nWaiting for 3WH from [%d] (TransmitSocket)...", tcpSktInfo.localPort);
                    appNetClientData.prevErrnoT = errno;
                }
                break;
            } else {
                appNetClientData.transmitSocketStat = true;
            }
            /*done establishing connection*/
            appNetClientData.serverSockConnected = true;
            BSP_LEDOff(BSP_LED_1);
            appNetClientData.state = APP_NET_CLIENT_STATE_SEND;
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_NET_CLIENT_STATE_SEND">
        case APP_NET_CLIENT_STATE_SEND:
        {
            SYS_CMD_READY_TO_READ();
            /*send HTTP connection request for listen socket*/
            char buffer[APP_SERVER_MAX_CON_HEADER_SIZE];
            char serverIpAddress[18];

            /*this is already validated*/
            TCPIP_Helper_IPAddressToString((IPV4_ADDR*)&(appNetClientData.addr.sin_addr.S_un.S_addr), serverIpAddress, sizeof (serverIpAddress));

            sprintf(buffer, "GET %s%s/%s/%s HTTP/1.1\r\n"
                    "Host: %s\r\n"
                    "Connection: keep-alive\r\n\r\n", APP_SERRVER_PROTOCOL_STRING, serverIpAddress,
                    APP_SERVER_LISTEN_PATH, appNetClientData.thingID, serverIpAddress);
            send(appNetClientData.listenSocket, (const char*) buffer, strlen((char*) buffer), 0);

            /*send HTTP connection request for transmit socket*/
            sprintf(buffer, "GET %s%s/%s/%s HTTP/1.1\r\n"
                    "Host: %s\r\n"
                    "Connection: keep-alive\r\n\r\n", APP_SERRVER_PROTOCOL_STRING, serverIpAddress,
                    APP_SERVER_TRANSMIT_PATH, appNetClientData.thingID, serverIpAddress);
            send(appNetClientData.transmitSocket, (const char*) buffer, strlen((char*) buffer), 0);

            BSP_LEDOff(BSP_LED_1);
            appNetClientData.serverSockConnected = true;
            SYS_CONSOLE_MESSAGE("\r\nSetup done. Listening to messages...\r\n");
            appNetClientData.state = APP_NET_CLIENT_STATE_LISTEN;
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_NET_CLIENT_STATE_LISTEN">
        case APP_NET_CLIENT_STATE_LISTEN:
        {
            char recvBuffer[256];
            unsigned char retVal;
            int length;

            //read and ack tx result 
            length = recv(appNetClientData.transmitSocket, recvBuffer, sizeof (recvBuffer) - 1, 0); //get the data from the recv queue
            if (length <= 0) // close or error condition
            {
                if (length <= 0 && errno != EWOULDBLOCK) {
                    SYS_CONSOLE_PRINT("\r\nClosing connection : transmitSocket close/error\r\n");
                    appNetClientData.state = APP_NET_CLIENT_STATE_SOCKET_CLOSE; // true error/close condition
                    break;
                }
            } else {
                recvBuffer[length] = '\0'; // Null terminate data
                if (strstr(recvBuffer, "X-RateLimit-Limit")) {
                    appNetClientData.lastUpdateFailed = true;
                }
            }

            /*if there are messages on listen socket, read and process them*/
            length = recv(appNetClientData.listenSocket, recvBuffer, sizeof (recvBuffer) - 1, 0); //get the data from the recv queue
            if (length <= 0) // close or error condition
            {
                if (length <= 0 && errno != EWOULDBLOCK) {
                    SYS_CONSOLE_PRINT("\r\nClosing connection : listenSocket error/close\r\n");
                    appNetClientData.state = APP_NET_CLIENT_STATE_SOCKET_CLOSE; // true error/close condition
                    break;
                }
            } else {
                recvBuffer[length] = '\0'; // Null terminate data
                retVal = _APP_process_cloud_Json((const char*) recvBuffer);
            }

            appNetClientData.state = APP_NET_CLIENT_STATE_SEND_PER_STATUS;
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_NET_CLIENT_STATE_SEND_PER_STATUS">
        case APP_NET_CLIENT_STATE_SEND_PER_STATUS:
        {
            static appPerData_t prevPerData;
            if (true == appNetClientData.perUpdateToken) {
                if (0 != memcmp(&prevPerData, &xData.appPerData, sizeof (appPerData_t))) {
                    prevPerData = xData.appPerData;
                    _APP_send_per_status();
                    appNetClientData.perUpdateToken = false; /*this will be enabled in the next callback*/

                    appNetClientData.perTokenTimerHandle = SYS_TMR_CallbackSingle(APP_SERVER_PER_UPDATE_TIMEOUT, 0, APP_perToken_tmrCallback);
                    if (SYS_TMR_HANDLE_INVALID == appNetClientData.perTokenTimerHandle) {
                        SYS_CONSOLE_PRINT("\r\nFailed to set per token timer (%s:%u)\r\n", __FUNCTION__, __LINE__);
                        appNetClientData.state = APP_NET_CLIENT_STATE_ERROR;
                        break;
                    } else {
                        appNetClientData.state = APP_NET_CLIENT_STATE_LISTEN;
                        break;
                    }
                } else {
                    appNetClientData.state = APP_NET_CLIENT_STATE_LISTEN;
                    break;
                }
            } else {
                appNetClientData.state = APP_NET_CLIENT_STATE_LISTEN;
                break;
            }
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_NET_CLIENT_STATE_SOCKET_CLOSE">
        case APP_NET_CLIENT_STATE_SOCKET_CLOSE:
        {
            appNetClientData.serverSockConnected = false;
            closesocket(appNetClientData.listenSocket);
            appNetClientData.listenSocketStat = false;
            closesocket(appNetClientData.transmitSocket);
            appNetClientData.transmitSocketStat = false;

            /*the timer will be re-init in*/
            SYS_TMR_CallbackStop(appNetClientData.keepAliveTimerHandle);

            BSP_LEDOn(BSP_LED_1);
            SYS_CONSOLE_MESSAGE("\r\nConnection Closed. Retrying\r\n");
            appNetClientData.state = APP_NET_CLIENT_STATE_INIT;
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_NET_CLIENT_STATE_ERROR">
        case APP_NET_CLIENT_STATE_ERROR:
        {
            SYS_CMD_READY_TO_READ();
            BSP_LEDOn(BSP_LED_1);
            appNetClientData.state = APP_NET_CLIENT_STATE_ERROR;
        }
            // </editor-fold>
        default:
        {
            break;
        }
    }
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="APP_TASK_USB">
// <editor-fold defaultstate="collapsed" desc="APP_USBHostEventHandler">

USB_HOST_EVENT_RESPONSE APP_USBHostEventHandler(USB_HOST_EVENT event, void * eventData, uintptr_t context) {
    switch (event) {
        case USB_HOST_EVENT_DEVICE_UNSUPPORTED:
            xData.appPerData.isUsbDevMounted = false;
            break;
        default:
            break;
    }
    return (USB_HOST_EVENT_RESPONSE_NONE);
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="APP_SYSFSEventHandler">

void APP_SYSFSEventHandler(SYS_FS_EVENT event, void * eventData, uintptr_t context) {
    switch (event) {
        case SYS_FS_EVENT_MOUNT:
            xData.appPerData.isUsbDevMounted = true;
            break;
        case SYS_FS_EVENT_UNMOUNT:
            xData.appPerData.isUsbDataValid = false;
            xData.appPerData.isUsbDevMounted = false;
            xData.appPerData.usbDriveSerialNo = 0;
            xData.appPerData.usbDriveName[0] ='\0';
            break;
        default:
            break;
    }
}
// </editor-fold>

void APP_TASK_USB(void) {
    switch (appUsbData.state) {
            // <editor-fold defaultstate="collapsed" desc="APP_USB_STATE_INIT">
        case APP_USB_STATE_INIT:
        {
            SYS_CMD_READY_TO_READ();
            xData.appPerData.isUsbDataValid = false;
            appUsbData.state = APP_USB_STATE_BUS_ENABLE;
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_USB_STATE_BUS_ENABLE">
        case APP_USB_STATE_BUS_ENABLE:
        {
            SYS_CMD_READY_TO_READ();
            // CNPUBbits.CNPUB12=1;
            SYS_FS_EventHandlerSet(APP_SYSFSEventHandler, (uintptr_t) NULL);
            USB_HOST_EventHandlerSet(APP_USBHostEventHandler, 0);
            USB_HOST_BusEnable(0);
            appUsbData.state = APP_USB_STATE_WAIT_BUS_ENABLE_COMPLETE;
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_USB_STATE_WAIT_BUS_ENABLE_COMPLETE">
        case APP_USB_STATE_WAIT_BUS_ENABLE_COMPLETE:
        {
            SYS_CMD_READY_TO_READ();
            if (USB_HOST_BusIsEnabled(0)) {
                SYS_CONSOLE_MESSAGE(" Waiting for USB device FS mount...\r\n");
                appUsbData.state = APP_USB_STATE_WAIT_DEVICE_MOUNT;
                break;
            } else {
                appUsbData.state = APP_USB_STATE_WAIT_BUS_ENABLE_COMPLETE;
                break;
            }
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_USB_STATE_WAIT_DEVICE_MOUNT">
        case APP_USB_STATE_WAIT_DEVICE_MOUNT:
        {
            SYS_CMD_READY_TO_READ();
            if (true == xData.appPerData.isUsbDevMounted) {
                SYS_CONSOLE_MESSAGE("Device FS Mounted\r\n");
                appUsbData.state = APP_USB_STATE_DEVICE_MOUNT;
                break;
            } else {
                appUsbData.state = APP_USB_STATE_WAIT_DEVICE_MOUNT;
                break;
            }
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_USB_STATE_DEVICE_MOUNT">
        case APP_USB_STATE_DEVICE_MOUNT:
        {
            SYS_CMD_READY_TO_READ();
            appUsbData.state = APP_USB_STATE_READ_DATA;
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_USB_STATE_READ_DATA">
        case APP_USB_STATE_READ_DATA:
        {
            SYS_CMD_READY_TO_READ();
            SYS_FS_RESULT res;
            res = SYS_FS_DriveLabelGet("/mnt/myDrive1", xData.appPerData.usbDriveName, &xData.appPerData.usbDriveSerialNo);
            if (res == SYS_FS_RES_FAILURE) {
                SYS_CONSOLE_PRINT("Device label Read failed (%s:%u)\r\n", __FUNCTION__, __LINE__);
                appUsbData.state = APP_USB_STATE_ERROR;
            } else {
                if (0 == strlen(xData.appPerData.usbDriveName)) {
                    strcpy(xData.appPerData.usbDriveName, APP_DEFAULT_DRIVE_NAME);
                }
                xData.appPerData.isUsbDataValid = true;
                appUsbData.state = APP_USB_STATE_IDLE;
            }
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_USB_STATE_IDLE">
        case APP_USB_STATE_IDLE:
        {
            SYS_CMD_READY_TO_READ();
            if (false == xData.appPerData.isUsbDevMounted) {
                SYS_CONSOLE_MESSAGE("Device disconnected.\r\n");
                appUsbData.state = APP_USB_STATE_WAIT_DEVICE_MOUNT;
                break;
            } else {
                appUsbData.state = APP_USB_STATE_IDLE;
                break;
            }
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_USB_STATE_ERROR">
        case APP_USB_STATE_ERROR:
        {
            SYS_CMD_READY_TO_READ();
            if (SYS_FS_Unmount("/mnt/myDrive1") != 0) {
                appUsbData.state = APP_USB_STATE_ERROR;
                break;
            } else {
                appUsbData.state = APP_USB_STATE_IDLE;
                break;
            }
        }
            // </editor-fold>
        default:
        {
            break;
        }
    }
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="APP_TASK_ANALOG">

void APP_analogScan_tmrCallback(uintptr_t context, uint32_t currTick) {
    DRV_ADC_Start();
}

void APP_TASK_ANALOG(void) {
    switch (appAnalogData.state) {
            // <editor-fold defaultstate="collapsed" desc="APP_ANALOG_STATE_INIT">
        case APP_ANALOG_STATE_INIT:
        {
            /*TODO: to be fixed in boot*/
            ANSELAbits.ANSA11 = 1;
            CNPUAbits.CNPUA11 = 0;
            CNPDAbits.CNPDA11 = 0;
            CNENAbits.CNIEA11 = 0;
            TRISAbits.TRISA11 = 1;

            SYS_CMD_READY_TO_READ();
            DRV_ADC0_Open();
            appAnalogData.tmrHandle = SYS_TMR_CallbackPeriodic(APP_ANALOG_SCAN_PERIOD, 0, APP_analogScan_tmrCallback);
            if (SYS_TMR_HANDLE_INVALID == appAnalogData.tmrHandle) {
                SYS_CONSOLE_PRINT("\r\nFailed to set analog scan timer (%s:%u)\r\n", __FUNCTION__, __LINE__);
                appAnalogData.state = APP_ANALOG_STATE_ERROR;
                break;
            } else {
                appAnalogData.state = APP_ANALOG_STATE_IDLE;
                break;
            }
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_ANALOG_STATE_IDLE">
        case APP_ANALOG_STATE_IDLE:
        {
            SYS_CMD_READY_TO_READ();
            appAnalogData.state = APP_ANALOG_STATE_IDLE;
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_ANALOG_STATE_ERROR">
        case APP_ANALOG_STATE_ERROR:
        {
            SYS_CMD_READY_TO_READ();
            DRV_ADC0_Close();
            BSP_LEDOn(BSP_LED_1);
            break;
        }
            // </editor-fold>
        default:
        {
            break;
        }
    }
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="APP_TASK_SWITCH">

void APP_switchToken_tmrCallback(uintptr_t context, uint32_t currTick) {
    appSwitchData.switchToken = true;
}

void APP_TASK_SWITCH(void) {
    switch (appSwitchData.state) {
            // <editor-fold defaultstate="collapsed" desc="APP_SWITCH_STATE_INIT">
        case APP_SWITCH_STATE_INIT:
        {
            SYS_CMD_READY_TO_READ();
            appSwitchData.switchToken = true;
            appSwitchData.state = APP_SWITCH_STATE_WAIT_TOKEN;
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_SWITCH_STATE_WAIT_TOKEN">        
        case APP_SWITCH_STATE_WAIT_TOKEN:
        {
            SYS_CMD_READY_TO_READ();
            if (true == appSwitchData.switchToken) {
                appSwitchData.state = APP_SWITCH_STATE_CHECK_N_TOGGLE;
            } else {
                appSwitchData.state = APP_SWITCH_STATE_WAIT_TOKEN;
            }
            break;
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_SWITCH_STATE_CHECK_N_TOGGLE">        
        case APP_SWITCH_STATE_CHECK_N_TOGGLE:
        {
            SYS_CMD_READY_TO_READ();
            bool toggled = false;
            if (BSP_SWITCH_STATE_PRESSED == BSP_SwitchStateGet(BSP_SWITCH_1)) {
                BSP_LEDToggle(BSP_LED_2);
                toggled = true;
            }
            if (BSP_SWITCH_STATE_PRESSED == BSP_SwitchStateGet(BSP_SWITCH_3)) {
                BSP_LEDToggle(BSP_LED_3);
                toggled = true;
            }

            if (true == toggled) {
                xData.appPerData.ledFlip=!xData.appPerData.ledFlip;
                appSwitchData.switchToken = false;
                appSwitchData.state = APP_SWITCH_STATE_START_TIMER;
                break;
            } else {
                appSwitchData.state = APP_SWITCH_STATE_CHECK_N_TOGGLE;
                break;
            }
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_SWITCH_STATE_START_TIMER">        
        case APP_SWITCH_STATE_START_TIMER:
        {
            SYS_CMD_READY_TO_READ();
            appSwitchData.tmrHandle = SYS_TMR_CallbackSingle(APP_SWITCH_TOKEN_TIMEOUT, 0, APP_switchToken_tmrCallback);
            if (SYS_TMR_HANDLE_INVALID == appSwitchData.tmrHandle) {
                SYS_CONSOLE_PRINT("\r\nFailed to set switch token timer (%s:%u)\r\n", __FUNCTION__, __LINE__);
                appSwitchData.state = APP_SWITCH_STATE_ERROR;
                break;
            } else {
                appSwitchData.state = APP_SWITCH_STATE_WAIT_TOKEN;
                break;
            }
        }
            // </editor-fold>
            // <editor-fold defaultstate="collapsed" desc="APP_SWITCH_STATE_ERROR">        
        case APP_SWITCH_STATE_ERROR:
        {
            SYS_CMD_READY_TO_READ();
            BSP_LEDOn(BSP_LED_1);
            break;
        }
            // </editor-fold>
        default:
        {
            break;
        }
    }
}
// </editor-fold>

void APP_Tasks(void) {
    switch (appTaskData.state) {
        case APP_STATE_INIT:
        {
            SYS_CMD_READY_TO_READ();
            APP_TASK_UC();
            APP_TASK_NET_MASTER();
            if (true == xData.isNetUp) {
                APP_TASK_NET_CLIENT();
                APP_TASK_NET_SERVER();
            }
            APP_TASK_SWITCH();
            APP_TASK_ANALOG();
            APP_TASK_USB();
            break;
        }
        default:
        {
            break;
        }
    }
}