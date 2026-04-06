#!/bin/bash
sudo killall slcand
sudo slcand -o -c -s8 /dev/ttyACM0 can1
sudo slcand -o -c -s8 /dev/ttyACM1 can1
sudo ip link set can1 up type can bitrate 1000000
sudo ifconfig can1 txqueuelen 1000
