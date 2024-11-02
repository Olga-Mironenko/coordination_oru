#!/bin/bash

set -eu

[ $# = 0 ]

root=$(dirname "$0")

"$root"/stop-all-demos.sh

timeout=60m
demo=GridTest
scenarios=(
#  BASELINE_IDEAL_DRIVER_AUTOMATED_FIRST #DONE
#  BASELINE_IDEAL_DRIVER_HUMAN_FIRST     #DONE
#  BASELINE_IDEAL_DRIVER_FIRST_COME      #DONE
)

scenarios+=(
#  S_DP1C #DONE
#  S_DP1M #DONE
#  S_DP2C #DONE
#  S_DP2M #DONE
#  S_DP3C #DONE
#  S_DP3M #DONE
#  S_DPGC #DONE
#  S_DPGM #DONE
#  S_DS1C #DONE
#  S_DS1M #DONE
#  S_DS2C #DONE
#  S_DS2M #DONE
#  S_DS3C #DONE
#  S_DS3M #DONE
#  S_DSGC #DONE
#  S_DSGM #DONE
#  S_UP1C #DONE
#  S_UP1M #DONE
#  S_UP2C #DONE
#  S_UP2M #DONE
#  S_UP3C #DONE
#  S_UP3M #DONE
#  S_UPGC #DONE
#  S_UPGM #DONE
#  S_US1C #DONE
#  S_US1M #DONE
#  S_US2C #DONE
#  S_US2M #DONE
#  S_US3C #DONE
#  S_US3M #DONE
#  S_USGC #DONE
#  S_USGM #DONE
)

## S_{D,U}{P,S}{1,2,3,G}{C,M}
#for a in D U; do
#  for b in P S; do
#    for c in 1 2 3 G; do
#      for d in C M; do
#        scenarios+=(S_"$a$b$c$d")
#      done
#    done
#  done
#done

## S_{D,U}{P,S}{1,2,3,G}{C,M}
#for c in 1 2 3 G; do
#  for d in C M; do
#    for a in D U; do
#      for b in P S; do
#        echo -n "  -$a$b$c$d"
#      done
#    done
#    echo
#  done
#done
#table=(
#  -DP1C  -DS1C  -UP1C  -US1C
#  -DP1M  -DS1M  -UP1M  -US1M
#  -DP2C  +DS2C  -UP2C  -US2C
#  -DP2M  +DS2M  -UP2M  -US2M
#  -DP3C  -DS3C  -UP3C  -US3C
#  -DP3M  -DS3M  -UP3M  -US3M
#  -DPGC  -DSGC  -UPGC  -USGC
#  -DPGM  -DSGM  -UPGM  -USGM
#)

reference=$(mktemp --tmpdir run-and-consolidate.XXXX)
trap 'rm -f "$reference"' EXIT

trap 'pkill -f "^[^ ]*java .*coordination_oru"' INT

set -x
"$root"/run-scenarios.sh "$timeout" "$demo" "${scenarios[@]}"
"$root"/consolidate-rundirs-to-sorted-csv.sh "$reference"  # will consolidate everything created after the reference file