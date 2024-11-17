#!/bin/bash

set -eu -o pipefail

[ $# = 0 ]

for i in {1..10}; do
  for f in screenshots/*_scenario1-"$i".*.png; do
    convert "$f" -crop 704x529+54+570 screenshots/cropped/"$i".png
  done
done