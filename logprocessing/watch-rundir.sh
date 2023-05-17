#!/bin/bash

set -eux -o pipefail

[ $# = 0 ]

root=$(dirname "$0")
watch -n1 "$root"/show-rundir.sh "$root"/../logs/rundirs/current