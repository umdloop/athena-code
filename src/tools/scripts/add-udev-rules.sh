#!/bin/bash

if [ "$EUID" -ne 0 ]
  then echo "Please run as root"
  exit
fi

cp umdloop-udev.rules /etc/udev/rules.d/99-umdloop-udev.rules

echo "Reboot for the changes to take effect!"
