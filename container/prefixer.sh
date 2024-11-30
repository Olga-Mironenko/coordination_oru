#!/bin/bash

set -eu -o pipefail

[ $# -gt 1 ]
name=$1
shift

exec "$@" \
  1> >(exec ts "[$name]") \
  2> >(exec ts "[$name]" >&2)