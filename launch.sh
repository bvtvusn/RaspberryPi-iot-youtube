#!/bin/sh
echo "Starting data collection script"

sudo killall pigpiod
sudo pigpiod


/home/bvtv/Documents/iotenv/bin/python3 /home/bvtv/Desktop/iot_lab01/Scripts/video_allSensors.py 
cd /