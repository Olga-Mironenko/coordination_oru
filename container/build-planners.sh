#!/bin/bash

set -eux -o pipefail

[ $# = 0 ]

cd SimpleReedsSheppCarPlanner

(
  flock 9

  rm -f CMakeCache.txt
  cmake -DOUTPUT_DIR="build-c${I_CONTAINER}" .
  make
) 9>build-planners.flock