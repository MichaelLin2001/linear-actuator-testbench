#!/usr/bin/env bash
set -euo pipefail

if pgrep -x motor_test >/dev/null || pgrep -x jc_driver_j3_single >/dev/null; then
    echo "a jc_driver process is already running" >&2
    exit 1
fi

exec "$(dirname "$0")/driver_session.exp"
