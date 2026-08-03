#!/bin/bash
set -e

uid="${HOST_UID:-1000}"
gid="${HOST_GID:-1000}"

if ! getent group "$gid" >/dev/null; then
    groupadd -g "$gid" hostgroup
fi

if ! getent passwd "$uid" >/dev/null; then
    useradd \
        -u "$uid" \
        -g "$gid" \
        -d /tmp \
        -s /bin/bash \
        hostuser
fi

exec gosu "$uid:$gid" "$@"