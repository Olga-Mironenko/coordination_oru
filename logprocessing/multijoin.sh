#!/bin/bash

set -eu -o pipefail

run_join() {
  join -t$'\t' "$@"
}

run_join_multi() {
    if [ $# = 1 ]; then
        run_join - "$1"
    else
        f=$1
        shift
        run_join - "$f" | run_join_multi "$@"
    fi
}

if [ $# -le 2 ]; then
    run_join "$@"
else
    f1=$1
    f2=$2
    shift 2
    run_join "$f1" "$f2" | run_join_multi "$@"
fi