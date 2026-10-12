#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
    echo "usage: $0 COMMAND [ARG ...]" >&2
    exit 2
fi

runtime_dir=/tmp/j3-testbench
mkdir -p "$runtime_dir"
temp_file=$(mktemp "$runtime_dir/command.XXXXXX")
printf '%s\n' "$*" >"$temp_file"
mv "$temp_file" "$runtime_dir/command"
