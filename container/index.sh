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
"$prefixer" chromium "$root"/chromium.sh --start-maximized &
sleep 1

"$root"/build-planners.sh

"$prefixer" record-screen "$root"/record-screen.sh "screen-$WORKER.mp4" &
# TODO: Call the script at the beginning of simulation and send it SIGTERM/...
# at the end of simulation.

"$prefixer" scenario "$root"/scenario.sh &

wait
# TODO: SIGINT to the container should stop everything. (Use `supervisor(1)`?)