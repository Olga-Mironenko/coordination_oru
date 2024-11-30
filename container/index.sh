#!/bin/bash

set -eux -o pipefail

[ $# = 0 ]

export DISPLAY=:0

root=$(dirname "$0")
prefixer=$root/prefixer.sh

"$prefixer" xvfb "$root"/xvfb.sh &
sleep 1

"$prefixer" x11vnc "$root"/x11vnc.sh &
"$prefixer" fluxbox fluxbox &
"$prefixer" chromium "$root"/chromium.sh &

wait