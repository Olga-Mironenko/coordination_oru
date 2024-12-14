#!/bin/bash

set -eu -o pipefail

[ $# = 0 ]

for i_map in {1..10}; do
  papermill analysis.ipynb generated_analysis_map"$i_map".ipynb -i_map I_MAP "$i_map"
done