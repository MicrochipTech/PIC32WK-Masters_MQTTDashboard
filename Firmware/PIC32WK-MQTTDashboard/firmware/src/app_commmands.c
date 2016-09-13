/*******************************************************************************
 MQTT dashboard demo app commands

  File Name:
    app_commands.c

  Summary:
    commands for demo app.

  Description:
    
 *******************************************************************************/


// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
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
#include <ctype.h>
#include "system_config.h"
#include "system_definitions.h"
#include "app_commands.h"
#include "app.h"

static int _app_cmd_constat(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static int _app_cmd_erase(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static int _app_cmd_set_server(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static int _app_cmd_set_port(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static int _app_cmd_set_id(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static int _app_cmd_set_mode(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static int _app_cmd_save_cfg(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static int _app_cmd_read_config(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static int _app_cmd_read_per(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);

static const SYS_CMD_DESCRIPTOR appCmdTbl[] ={
    {"app_constat", _app_cmd_constat,               "            :get connection status"},
    {"app_erase_cfg", _app_cmd_erase,               "          :erase config from flash"},
    {"app_set_server", _app_cmd_set_server,         "         :set server IP address"},
    {"app_set_port", _app_cmd_set_port,             "           :set server port"},
    {"app_set_id", _app_cmd_set_id,                 "             :set thingID"},
    {"app_set_mode", _app_cmd_set_mode,             "           :set device mode"},
    {"app_save_cfg", _app_cmd_save_cfg,             "           :save cfg to flash and restart"},
    {"app_read_cfg", _app_cmd_read_config,          "           :read application configuration"},
    {"app_read_per", _app_cmd_read_per,             "           :read peripheral data"},
};

bool app_commands_init() {
    if (!SYS_CMD_ADDGRP(appCmdTbl, sizeof (appCmdTbl) / sizeof (*appCmdTbl), "app", ": app commands")) {
        SYS_ERROR(SYS_ERROR_ERROR, "Failed to create app Commands\r\n", 0);
        return false;
    }

    return true;
}

int _app_cmd_constat(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv) {
    const void* cmdIoParam = pCmdIO->cmdIoParam;
    (APP_CONN_STATUS_REACHABLE == xData.wanConnStatus)?
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "WAN            : reachable\r\n"):
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "WAN            : not reachable\r\n");
    
    (APP_CONN_STATUS_REACHABLE == xData.serverConnStatus)?
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "server         : reachable\r\n"):
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "server         : not reachable\r\n");
    
    (true == appNetClientData.serverSockConnected)?     /*arch violation waiver*/
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "server sockets : established\r\n"):
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "server sockets : not established\r\n");
 
 
    return 0;
}

static int _app_cmd_erase(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv) {
    const void* cmdIoParam = pCmdIO->cmdIoParam;
    APP_FLASH_STATUS_t status;

    /*TODO: have proper checks here*/
    APP_erase_flash(APP_USER_CONFIG_FLASH_AREA, 1, &status);

    (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Application configuration erased from flash\r\n");
}

static int _app_cmd_set_server(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv) {
    const void* cmdIoParam = pCmdIO->cmdIoParam;

    if (argc != 2) {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "app_set_server Usage: app_set_server <address>\r\n");
    } else {
        IPV4_ADDR IPAddress;
        if (true == TCPIP_Helper_StringToIPAddress(argv[1], &IPAddress)) {
            xData.appConfigData.serverAddress = IPAddress;
            (*pCmdIO->pCmdApi->print)(cmdIoParam, "Server IP stored (assuming port: %d).\r\nUse app_save_cfg to write to flash.\r\n",xData.appConfigData.serverPort);
            return;
        } else {
            (*pCmdIO->pCmdApi->msg)(cmdIoParam, "app_set_server Usage: app_set_server <address>\r\n");
            return;
        }
    }
}

static int _app_cmd_set_port(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv) {
    const void* cmdIoParam = pCmdIO->cmdIoParam;

    if (argc != 2) {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "app_set_port Usage: app_set_port <port>\r\n");
    } else {
        uint16_t port;
        port=atoi(argv[1]); 
        if(port>0){
            xData.appConfigData.serverPort=port;
            xData.appConfigData.isPortConfigured = true;        /*take current or default*/
            (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Port setting stored. use app_save_cfg to write to flash.\r\n");
            return 0;
        }
        else{
            (*pCmdIO->pCmdApi->msg)(cmdIoParam, "app_set_port Usage: app_set_port <port>\r\n");
            return -1;
        }
    }
}

static int _app_cmd_set_id(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv) {
    const void* cmdIoParam = pCmdIO->cmdIoParam;

    if (argc != 2) {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "app_set_id Usage: app_set_id <thingID>\r\n");
    } else {
            strncpy(xData.appConfigData.thingID,argv[1],APP_MAX_THING_ID_LENGTH);
            xData.appConfigData.thingID[APP_MAX_THING_ID_LENGTH-1]='\0'; /*safety net*/
            xData.appConfigData.isThingIdConfigured = true;     /*take current or default*/
            xData.appConfigData.opMode=APP_DEVICE_MODE_SLAVE;   /*default mode is slave if device id is user configured*/
            (*pCmdIO->pCmdApi->msg)(cmdIoParam, "thingID stored in slave mode. use app_save_cfg to write to flash.\r\n");
            return 0;
        }
}

static int _app_cmd_set_mode(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv) {
    const void* cmdIoParam = pCmdIO->cmdIoParam;

    if (argc != 2) {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "app_set_mode Usage: app_set_mode <0/1> [Master/Slave]\r\n");
    } else {
        int mode;
        mode=atoi(argv[1]); 
        if(0==mode){
            xData.appConfigData.opMode=APP_DEVICE_MODE_MASTER;
            (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Mode setting stored (Master). use app_save_cfg to write to flash.\r\n");
            return 0;
        }
        else if(1==mode){
            xData.appConfigData.opMode=APP_DEVICE_MODE_SLAVE;
            (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Mode setting stored (Slave). use app_save_cfg to write to flash.\r\n");
            return 0;
        }
        else{
            (*pCmdIO->pCmdApi->msg)(cmdIoParam, "app_set_mode Usage: app_set_mode <0/1> [Master/Slave]\r\n");
            return -1;
        }
    }
}

static int _app_cmd_save_cfg(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv) {
    const void* cmdIoParam = pCmdIO->cmdIoParam;
    xData.isNewConfig = true;
    (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Saving configuration to flash. Application will reset now");
    return 0;
}

static int _app_cmd_read_config(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    const void* cmdIoParam = pCmdIO->cmdIoParam;
    char serverIpAddress[18];
    bool convRetval;
    convRetval=TCPIP_Helper_IPAddressToString(&(xData.appConfigData.serverAddress),serverIpAddress,sizeof(serverIpAddress));
    (*pCmdIO->pCmdApi->print)(cmdIoParam, "Current settings. use app_save_cfg to write to flash\r\n\r\n",serverIpAddress);
    if((true==convRetval)&&(0xffffffff != xData.appConfigData.serverAddress.Val)&&(0 != xData.appConfigData.serverAddress.Val))
    {
        (*pCmdIO->pCmdApi->print)(cmdIoParam, "server IP address : %s\r\n",serverIpAddress);
        (*pCmdIO->pCmdApi->print)(cmdIoParam, "server Port       : %d\r\n",xData.appConfigData.serverPort);
        (*pCmdIO->pCmdApi->print)(cmdIoParam, "thingID           : %s\r\n",xData.appConfigData.thingID);   
        (*pCmdIO->pCmdApi->print)(cmdIoParam, "mode              : %d\r\n",xData.appConfigData.opMode);   
    }
    else
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "No valid server configuration found in Flash\r\n");
        (*pCmdIO->pCmdApi->print)(cmdIoParam, "thingID    : %s\r\n",xData.appConfigData.thingID);
        (*pCmdIO->pCmdApi->print)(cmdIoParam, "mode       : %d\r\n",xData.appConfigData.opMode);   
    }
    return 0;
}

static int _app_cmd_read_per(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    const void* cmdIoParam = pCmdIO->cmdIoParam;
    (*pCmdIO->pCmdApi->print)(cmdIoParam, "Pot Position      : %d\r\n",xData.appPerData.potPosition);
    (*pCmdIO->pCmdApi->print)(cmdIoParam, "Temperature       : %d\r\n",xData.appPerData.temperature);
    (*pCmdIO->pCmdApi->print)(cmdIoParam, "Led 2             : %d\r\n",(int)BSP_LEDStateGet(BSP_LED_2));   
    (*pCmdIO->pCmdApi->print)(cmdIoParam, "Led 3             : %d\r\n",(int)BSP_LEDStateGet(BSP_LED_3));   
    (*pCmdIO->pCmdApi->print)(cmdIoParam, "USB mount status  : %d\r\n",xData.appPerData.isUsbDevMounted);   
    (*pCmdIO->pCmdApi->print)(cmdIoParam, "USB data status   : %d\r\n",xData.appPerData.isUsbDataValid);
    if (true==xData.appPerData.isUsbDataValid){
    (*pCmdIO->pCmdApi->print)(cmdIoParam, "USB Name          : %s\r\n",xData.appPerData.usbDriveName);
    (*pCmdIO->pCmdApi->print)(cmdIoParam, "USB Serial Number : %u\r\n",xData.appPerData.usbDriveSerialNo);
    }
    return 0;
}