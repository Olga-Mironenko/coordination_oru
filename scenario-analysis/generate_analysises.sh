#!/bin/bash

set -eu -o pipefail

[ $# -ge 0 ]
if [ $# = 0 ]; then
  indexes=({1..10})
else
  indexes=("$@")
fi

fails=()
for i_map in "${indexes[@]}"; do
  if ! papermill -p I_MAP "$i_map" analysis.ipynb generated_analysis_map"$i_map".ipynb; then
    fails+=("$i_map")
  fi
done

[ ! "${fails[*]}" ]