#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
ROUNDS="${1:-5}"
python3 "$VALIDATION_DIR/prepare_loop.py" --scene "$SCENE_DIR" --out "$OUTPUT_DIR" --rounds "$ROUNDS"
GOALS="$(python3 -c 'import sys; print(",".join(map(str, range(2 * int(sys.argv[1])))))' "$ROUNDS")"
python3 "$SCENE_DIR/verify_navigation.py" --goals "$GOALS" --timeout 60 \
  --manifest "$OUTPUT_DIR/loop_manifest.json" --out "$OUTPUT_DIR/navigation" \
  2>&1 | tee "$OUTPUT_DIR/probe.log"
python3 "$SCENE_DIR/analyze_clearance.py" "$OUTPUT_DIR/navigation" | tee "$OUTPUT_DIR/clearance.log"
python3 "$VALIDATION_DIR/check_results.py" "$OUTPUT_DIR/navigation/clearance_summary.json" \
  --expected-legs "$((2 * ROUNDS))"
