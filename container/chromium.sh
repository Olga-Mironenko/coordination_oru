#!/bin/bash

set -eux -o pipefail

[ $# -ge 0 ]
args=("$@")

exec gosu nonroot chromium --no-sandbox --test-type "${args[@]}"