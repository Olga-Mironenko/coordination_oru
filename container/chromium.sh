#!/bin/bash

set -eux -o pipefail

[ $# = 0 ]

gosu nonroot chromium --no-sandbox --test-type