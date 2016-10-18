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

#### Server without Docker

To setup a server instance without Docker, run the following commands. You would ahve to run as a previlaged user to install and configure the tools.

```bash
#install tools
apt-get update && apt-get install -y apache2 mosquitto-clients mosquitto build-essential git 

#clone repo
git clone https://github.com/MicrochipTech/PIC32WK-Masters_MQTTDashboard.git
cd Server/masters-Docker

#compile and install APIs
mkdir -p /var/www/api/listen
cp configs/listen.sh /var/www/api/listen/to
chmod +x /var/www/api/listen/to

mkdir -p /var/www/api/mdtweet
cp configs/sendMessage.c /var/www/api/mdtweet
gcc /var/www/api/mdtweet/sendMessage.c -o /var/www/api/mdtweet/for
rm /var/www/api/mdtweet/sendMessage.c

#configure Apache
cp configs/serve-cgi-bin.conf /etc/apache2/conf-available/serve-cgi-bin.conf
cp configs/serve-cgi-bin.conf /etc/apache2/conf-enabled/serve-cgi-bin.conf
cp configs/apache2.conf /etc/apache2/apache2.conf
mkdir -p /var/www/html
rm -rf /var/www/html/*
cp html /var/www/html 
a2enmod cgi
service apache2 restart

#configure Mosquitto
cp configs/mosquitto.conf /etc/mosquitto/mosquitto.conf
cp configs/supervisord.conf /etc/supervisor/conf.d/supervisord.conf
service mosquitto restart
```

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

When the device is not connected to the server the Red LED will be lit up on the starter kit. Once connection is successfully established, the red LED will be switched off. 

## Operation

To execute the demo after configuration, load the dashboard onto a browser by enteringt the public IP address of the server. Once the device is up and running with a valid WLAN connection (with WAN connectivity) and server configuration, the following operations can be performed:

- POT status will be displayed in the dashboard
- Bidirectional LED control from dashboard and starter kit (SK) using web page buttons and SK switches (SW1 and SW3)
- Temperature sensor readings will be updated in the dashboard
- When a USB flash drive (masss storage device) is plugged into the OTG port of the starter kit, its configured name as well as serial number will be displayed on the dashboard.
- The Android App available in the APP folder can be used to discover the boards within the same WiFi network and have the same level of control over the device as from the Dashboard.

## Application commands

```
- app_constat          :          fetch connection status of device
- app_erase_cfg        :          erase configutrations stored in flash
- app_set_server       :          set server IP address
- app_set_port         :          set server port. Default is assumed as 80
- app_set_id           :          set thingID. Default is derived from Mac address
- app_set_mode         :          set device in master or slave mode
- app_save_cfg         :          save configurations to flash and restart to apply them
- app_read_cfg         :          display current configuration set (from struct , no tFlash)
- app_read_per         :          display peripheral data
```
