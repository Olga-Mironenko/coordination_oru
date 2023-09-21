#!/bin/bash

pkill -f "^[^ ]*java .*coordination_oru"
pkill -f run-scenarios.sh
pkill -f run-and-consolidate.sh
