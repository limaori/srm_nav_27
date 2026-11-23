#!/usr/bin/env python3
"""Return nonzero when arrivals or the existing 5 cm clearance criterion fail."""
import argparse
import json
from pathlib import Path


def accepted(summary, expected_legs):
    legs = summary.get("legs", [])
    margin = summary.get("minimum_envelope_margin_m")
    return (
        len(legs) == expected_legs
        and all(leg.get("status") == "SUCCEEDED" for leg in legs)
        and all(0 <= leg.get("endpoint_error_m", float("inf")) <= 0.5 for leg in legs)
        and all(leg.get("envelope_margin_m") is not None for leg in legs)
        and margin is not None
        and margin >= 0.05
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("summary", type=Path)
    parser.add_argument("--expected-legs", type=int, default=10)
    args = parser.parse_args()
    if args.expected_legs < 1:
        parser.error("--expected-legs must be positive")
    summary = json.loads(args.summary.read_text(encoding="utf-8"))
    ok = accepted(summary, args.expected_legs)
    print("PASS" if ok else "FAIL: check arrivals, endpoint errors, and geometric envelope clearance")
    raise SystemExit(0 if ok else 1)
