# PIC32WK MQTT dashboard demo

## Overview

This demo is used to illustreate the use of PIC32WK starterkit as an IoT platform for WiFi connectivity and device control over an MQTT based Dashboard

## Setup

### Device firmware

Download Bootstrap/BFL.X.production.hex as well as PIC32WK-MQTTDashboard.X.production.hex from the latest release Tag into the PIC32WK starter kit.

### Server

This demo uses a self hosted server for the dashboard webserver as well as messaging service. To setup this server, run the following commands from the server.

The preferred and tested server setup uses a Digitalocean Ubuntu 16.04 instance with atleast 1GB of Memory (RAM)

```bash
apt-get update
apt-get upgrade -y
apt-get install git docker.io -y
git clone https://github.com/MicrochipTech/PIC32WK-Masters_MQTTDashboard.git
cd Server/masters-Docker
docker build -t myiotdashboard .
docker run -d -p 80:80 -p 443:443  --name iot_dashboard myiotdashboard
```

Once this is done, onte down the public IP address of the server. This is to be configured into the device.

### Device

Once the device is flashed with bootstrap and firmware code, issue the following commands into the console prompt. [console operates at 9200 8N1 baud settings]

**WiFi setup**

```bash
wlan open
wlan set ssid <ssid>
wlan set psk <password>
wlan set imode 0x31
wlan save config
```

**Server details**

```bash
app_set_server <public ip of server>
app_save_cfg
```

Once the device resets, the "Thing-ID" will be displayed in the bootup logs. Make a note of this Thing-ID.

## Operation

To execute the demo after configuration, load the dashboard onto a browser by enteringt the public IP address of the server. Once the device is up and running with a valid WLAN connection (with WAN connectivity) and server configuration, the following operations can be performed:

- POT status will be displayed in the dashboard
- Bidirectional LED control from dashboard and starter kit (SK) using web page buttons and SK switches (SW1 and SW3)
- Temperature sensor readings will be updated in the dashboard
- When a USB flash drive (masss storage device) is plugged into the OTG port of the starter kit, its configured name as well as serial number will be displayed on the dashboard.
