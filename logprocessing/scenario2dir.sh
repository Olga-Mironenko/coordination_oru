#!/bin/bash

set -eu -o pipefail

[ $# = 1 ]
scenario=$1

# See `getScenarioIdAsBasename`: `scenarioId.replace('/', '_').replace(' ', '_')`.
perl -pe 's{[/ ]}{_}g' <<<"$scenario"