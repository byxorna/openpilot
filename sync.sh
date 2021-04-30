#!/usr/bin/env bash
target="root@192.168.1.14:/data/openpilot/"
echo "syncing $(git rev-parse HEAD) to ${target}" >&2
set -ex
rsync -rv . "${target}"
ssh "${target%%:*}" scons
