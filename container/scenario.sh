#!/bin/bash

set -eux -o pipefail

[ $# = 0 ]

root=$(dirname "$0")
cd "$root"/..
exec \time ./gradlew run -Passert -Pdemo=GeneratedMapTest