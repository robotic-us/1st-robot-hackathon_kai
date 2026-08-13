#!/bin/sh
set -eu

project_dir=/home/phorce/hackathon/soldering_robot_ws

/usr/bin/install -o root -g root -m 0755 \
    "$project_dir/config/walkon-pcm-usb-reset" \
    /usr/local/sbin/walkon-pcm-usb-reset
/usr/bin/install -o root -g root -m 0440 \
    "$project_dir/config/90-walkon-pcm-reset" \
    /etc/sudoers.d/90-walkon-pcm-reset
/usr/sbin/visudo -cf /etc/sudoers.d/90-walkon-pcm-reset
/usr/bin/install -o root -g root -m 0644 \
    "$project_dir/config/99-walkon-pcm.rules" \
    /etc/udev/rules.d/99-walkon-pcm.rules
/usr/bin/udevadm control --reload-rules

if [ -e /dev/ttyACM0 ]; then
    /usr/bin/chown phorce:dialout /dev/ttyACM0
    /usr/bin/chmod 0660 /dev/ttyACM0
fi
