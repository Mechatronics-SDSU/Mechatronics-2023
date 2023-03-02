#!/bin/bash
sudo busybox devmem 0x0c303010 w 0xc400
sudo busybox devmem 0x0c303018 w 0xc458
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
sudo ip link set can0 type can bitrate 500000 berr-reporting on loopback off
sudo ip link set up can0
exit 0

