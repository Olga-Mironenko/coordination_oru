#!/bin/bash

set -eux -o pipefail

[ $# = 0 ]

runnames=(
  '20241203_170129_all600'
  '20241213_104400_racing'
  '20241214_122216_racing_passhum'
)

indexes=(
#  {1..10}
  1
)

fails=()
for runname in "${runnames[@]}"; do
  for i_map in "${indexes[@]}"; do
    opts=(
      -p RUNNAME "$runname"
      -p I_MAP "$i_map"
    )
    if ! papermill "${opts[@]}" analysis.ipynb generated_analysis_map"$i_map".ipynb; then
      fails+=("$runname:$i_map")
    fi
  done
done

[ ! "${fails[*]}" ]