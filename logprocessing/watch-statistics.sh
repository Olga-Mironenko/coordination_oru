#!/bin/bash

set -eux -o pipefail

[ $# = 0 ]

root=$(dirname "$0")
cd "$root"/../logs/statistics
watch -n1 -d 'd=$(readlink current); echo $d; paste $d/*.txt | pr -t -e38'