#!/bin/sh
echo "Starting data collection script"

sudo killall pigpiod
sudo pigpiod


/home/myuser/Documents/iotenv/bin/python3 /home/myuser/Desktop/iot_lab01/Scripts/video_allSensors.py 
cd /
