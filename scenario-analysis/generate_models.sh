#!/bin/bash

set -eux -o pipefail

[ $# = 0 ]

experiments=(
  100
  010
  001
  110
  101
  011
  111
)

fails=()
for experiment in "${experiments[@]}"; do
  opts=(
    -p ID_EXPERIMENT_TEXT "$experiment"
  )
  if ! papermill "${opts[@]}" model.ipynb generated_model_"$experiment".ipynb; then
    fails+=("$experiment")
  fi
done

[ ! "${fails[*]}" ]