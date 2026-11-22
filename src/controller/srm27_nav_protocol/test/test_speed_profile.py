#!/usr/bin/env python3
"""串口功率档限幅契约：YAML 默认档 + C++ 两档硬上限必须一致。

当前下位机是 100 W；180 W 是升级固件后的预设。任何一项被误改，
这里都会失败，避免"上层调了参数、下位机实际能力没跟上"。
"""

import re
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]

PROFILES = {
    "100w": [2.6, 2.6, 2.6, 8.5, 7.0, 25.0],
    "180w": [3.5, 3.5, 3.5, 12.0, 12.0, 40.0],
}
KEYS = [
    "max_vx",
    "max_vy",
    "max_linear_speed",
    "max_wz",
    "max_linear_accel",
    "max_angular_accel",
]


def main() -> int:
    errors = []

    params = yaml.safe_load(
        (ROOT / "config/srm27_nav_protocol.yaml").read_text()
    )["/srm27_nav_protocol"]["ros__parameters"]

    if params.get("power_profile") != "100w":
        errors.append(
            f"power_profile 应为 '100w'（当前下位机），实际 {params.get('power_profile')!r}"
        )

    expected = dict(zip(KEYS, PROFILES["100w"]))
    for key, value in expected.items():
        if params.get(key) != value:
            errors.append(f"{key} 应为 {value}，实际 {params.get(key)!r}")

    cpp = (ROOT / "src/srm27_nav_protocol.cpp").read_text()
    for name, values in PROFILES.items():
        match = re.search(rf'"{name}"\s*,\s*([^}}]+)\}}', cpp)
        if not match:
            errors.append(f"C++ 中找不到功率档 {name} 的硬上限")
            continue
        got = [float(item) for item in match.group(1).split(",")]
        if got != values:
            errors.append(f"C++ 功率档 {name} 应为 {values}，实际 {got}")

    if errors:
        print("FAIL: 串口功率档契约不一致")
        for error in errors:
            print(f"  - {error}")
        return 1

    print("PASS: 串口功率档契约一致（默认 100w；180w 预设可用）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
