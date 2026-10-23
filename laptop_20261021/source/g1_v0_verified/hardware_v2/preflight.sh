#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
python g1_arm.py preflight
python g1_arm.py nudge --joint 22 --delta-deg -1
