#!/bin/bash

set -ex

[ $# = 0 ]

pkill -f "^[^ ]*java .*coordination_oru" || true
pkill -f run-scenarios.sh || true