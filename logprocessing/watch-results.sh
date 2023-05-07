#!/bin/bash

set -eux -o pipefail

[ $# = 0 ]

root=$(dirname "$0")
dir=$root/../results
watch -n1 -d "paste \"$dir\"/*.txt | pr -t -e38"