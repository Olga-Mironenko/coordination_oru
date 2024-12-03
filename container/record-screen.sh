#!/bin/bash

set -eu -o pipefail

[ $# = 1 ]
file=$1

w_screen=1847
h_screen=1135

h_top_panel=109
h_bottom_panel=20

w_video=$w_screen
h_video=$((h_screen - h_top_panel - h_bottom_panel))

args=(
  -video_size "${w_video}x${h_video}"
  -framerate 30
  -f x11grab
  -i ":0+0,$h_top_panel"
  -y  # "yes, overwrite"
  "$file"
)

exec ffmpeg "${args[@]}"