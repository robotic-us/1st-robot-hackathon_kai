#!/bin/sh
set -eu

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
operator=${SUDO_USER:-phorce}

case "$operator" in
    *[!A-Za-z0-9_-]*|'')
        echo "unsupported operator user: $operator" >&2
        exit 2
        ;;
esac

stage_dir=$(mktemp -d /tmp/pcm-permissions.XXXXXX)
trap 'rm -rf "$stage_dir"' EXIT HUP INT TERM

sed "s/@OPERATOR@/$operator/g" \
    "$script_dir/90-walkon-pcm-reset" > "$stage_dir/90-walkon-pcm-reset"
sed "s/@OPERATOR@/$operator/g" \
    "$script_dir/99-walkon-pcm.rules" > "$stage_dir/99-walkon-pcm.rules"

/usr/bin/install -o root -g root -m 0755 \
    "$script_dir/walkon-pcm-usb-reset" \
    /usr/local/sbin/walkon-pcm-usb-reset
/usr/bin/install -o root -g root -m 0440 \
    "$stage_dir/90-walkon-pcm-reset" \
    /etc/sudoers.d/90-walkon-pcm-reset
/usr/sbin/visudo -cf /etc/sudoers.d/90-walkon-pcm-reset
/usr/bin/install -o root -g root -m 0644 \
    "$stage_dir/99-walkon-pcm.rules" \
    /etc/udev/rules.d/99-walkon-pcm.rules
/usr/bin/udevadm control --reload-rules

if [ -e /dev/ttyACM0 ]; then
    /usr/bin/chown "$operator":dialout /dev/ttyACM0
    /usr/bin/chmod 0660 /dev/ttyACM0
fi
