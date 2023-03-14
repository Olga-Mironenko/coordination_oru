#!/bin/bash

perl -pe 's{ *([0-9]+) ms\b}{ ... ms}g' |
  grep -w -e 'SWITCHING' -e 'TrajectoryEnvelopeCoordinator' |
  grep -v -w -e "constraints" -e "digraph"