#!/bin/bash
teensy_serial=$1
lidar_serial=$2

find /etc/udev/rules.d -name "58-lino.rules" -delete
touch /etc/udev/rules.d/58-lino.rules

cat  > /etc/udev/rules.d/58-lino.rules <<EOL
KERNEL=="ttyACM?", SUBSYSTEM=="tty", ATTRS{idVendor}=="16c0",
ATTRS{idProduct}=="0483", ATTRS{serial}=="${teensy_serial}", MODE="0660" SYMLINK+="linobase"
KERNEL=="ttyACM?", SUBSYSTEM=="tty", ATTRS{idVendor}=="16c0",
ATTRS{idProduct}=="0483", ATTRS{serial}=="${lidar_serial}", MODE="0660" SYMLINK+="linolidar"
EOL
cat /etc/udev/rules.d/58-lino.rules
