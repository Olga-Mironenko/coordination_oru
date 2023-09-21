#!/bin/bash

set -eu

[ $# = 0 ]

root=$(dirname "$0")

timeout=60m
demo=GridTest
scenarios=(
  BASELINE_IDEAL_DRIVER_AUTOMATED_FIRST
  BASELINE_IDEAL_DRIVER_HUMAN_FIRST
  BASELINE_IDEAL_DRIVER_FIRST_COME
)

scenarios+=(
  S_DP1C
  S_DP1M
  S_DP2C
  S_DP2M
  S_DP3C
  S_DP3M
  S_DPGC
  S_DPGM
  S_DS1C
  S_DS1M
  S_DS2C
  S_DS2M
  S_DS3C
  S_DS3M
  S_DSGC
  S_DSGM
  S_UP1C
  S_UP1M
  S_UP2C
  S_UP2M
  S_UP3C
  S_UP3M
  S_UPGC
  S_UPGM
  S_US1C
  S_US1M
  S_US2C
  S_US2M
  S_US3C
  S_US3M
  S_USGC
  S_USGM
)

# S_{D,U}{P,S}{1,2,3,G}{C,M}
#for a in D U; do
#  for b in P S; do
#    for c in 1 2 3 G; do
#      for d in C M; do
#        scenarios+=(S_"$a$b$c$d")
#      done
#    done
#  done
#done

reference=$(mktemp --tmpdir run-and-consolidate.XXXX)
trap 'rm -f "$reference"' EXIT

trap 'pkill -f "^[^ ]*java .*coordination_oru"' INT

set -x
"$root"/run-scenarios.sh "$timeout" "$demo" "${scenarios[@]}"
"$root"/consolidate-rundirs-to-sorted-csv.sh "$reference"