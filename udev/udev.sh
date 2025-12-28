#!/bin/bash
udevadm test -a add $(udevadm info -q path -n /dev/ttyACM0)
