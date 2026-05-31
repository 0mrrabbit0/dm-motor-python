#!/bin/bash
# Run each FD probe combo as a fresh Python process (avoids SDK state issues).
set -u
PY="/home/ubuntu/miniforge3/bin/python"
export LD_LIBRARY_PATH=/home/ubuntu/miniforge3/lib
cd "$(dirname "$0")"

combos=(
    "0 0.75 0.75"
    "1 0.75 0.75"
    "1 0.80 0.80"
    "1 0.875 0.875"
    "1 0.875 0.75"
    "1 0.75 0.875"
    "1 0.70 0.70"
)

echo
for combo in "${combos[@]}"; do
    echo "----- combo: $combo -----"
    "$PY" -u probe_one.py $combo
    sleep 0.7
done

echo
echo "==== SUMMARY ===="
grep -h "^RESULT" /tmp/dm_probe_out.* 2>/dev/null
