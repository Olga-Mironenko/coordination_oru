#!/bin/bash

perl -pe 's{ *([0-9]+) ms\b}{ ... ms}g' | grep -e 'SWITCHING' -e 'TrajectoryEnvelopeCoordinator'