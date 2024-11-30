#!/bin/bash

set -eux -o pipefail

opts=(
  -screen 0 1847x1135x24
  -ac -r -cc 4 -accessx -xinerama
  +extension Composite -extension RANDR +extension GLX
)
exec Xvfb "$DISPLAY" "${opts[@]}"