#!/bin/bash
inotifywait -e modify,create,delete -r -m -q --format '%w%f' mobilebot-f19 | while read FILE
do
rsync /home/pi/eecs467_a1/$FILE debian@192.168.4.2:/home/debian/$FILE
done
