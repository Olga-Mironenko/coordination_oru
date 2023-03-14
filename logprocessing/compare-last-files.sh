#!/bin/bash

set -eux -o pipefail

# shellcheck disable=SC2012
files_text=$(ls -t -p | grep -v '/$' | head -n2)
mapfile -t files <<<"$files_text"

file_new=${files[0]}
file_old=${files[1]}

root="$(dirname "$0")"
meld --label="$file_old | $file_new" <("$root"/clean-log.sh <"$file_old") <("$root"/clean-log.sh <"$file_new")