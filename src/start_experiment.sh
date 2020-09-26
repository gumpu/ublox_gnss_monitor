#!/usr/bin/env bash

/bin/sleep 30
/bin/echo gpio > /sys/class/leds/led0/trigger
/bin/echo 1    > /sys/class/leds/led0/brightness

cd /home/pi/GNSS/
./mon -n 14480 -z

status=$?
if test $status -eq 0
then
    # Experiment went OK, shutdown the system
    /bin/sleep 30
    /bin/echo 0    > /sys/class/leds/led0/brightness
    /sbin/shutdown -h now
else
    # mon failed, blink led to indicate the problem
    while true
    do
        sleep 1
        /bin/echo 0    > /sys/class/leds/led0/brightness
        sleep 1
        /bin/echo 1    > /sys/class/leds/led0/brightness
    done
fi

