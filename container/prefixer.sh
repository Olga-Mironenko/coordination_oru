#!/bin/bash

set -eu -o pipefail

[ $# -gt 1 ]
name=$1
shift

format="%H:%M:%S [$name]"
exec "$@" \
  1> >(exec ts "$format") \
  2> >(exec ts "$format" >&2)