#!/bin/bash

set -ex

[ $# = 0 ]

strace -f -e openat,write ./run-scenarios.sh 60m GeneratedMapTest "..." >/dev/null 2>tmp.log
grep -o 'openat(.*) =' tmp.log | grep WRONLY | sort -u | less