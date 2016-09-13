#!/bin/bash

echo "Content-Type: application/json"
echo -e "Connection: keep-alive\n\n"

path=($(echo $REQUEST_URI | tr "/" "\n"))
thingID=${path[-1]}
mosquitto_sub -t $thingID -q 1

