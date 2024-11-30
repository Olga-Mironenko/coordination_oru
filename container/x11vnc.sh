#!/bin/bash

set -eux -o pipefail

[ $# = 0 ]

[ "$DISPLAY" ]

args=(
  -rfbport 5900
  -forever
  -shared
  -nopw  # to disable the warning about no password
)
exec x11vnc "${args[@]}"