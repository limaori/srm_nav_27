#!/usr/bin/env python3
"""Generate A/B/C configs while preserving the controller and behavior tree."""
import argparse
import copy
from pathlib import Path

import yaml


def prepare(workspace, output):
    scene = workspace / "maps/227_1116_remap/simulation"
    base = yaml.safe_load((scene / "nav2_params_srm_tdt_fakeyaw.yaml").read_text(encoding="utf-8"))
    original = yaml.safe_load((workspace / "src/bringup/srm27_nav_bringup/config/simulation/current/nav2_params_srm_fakeyaw.yaml").read_text(encoding="utf-8"))
    simple = original["smoother_server"]["ros__parameters"]["simple_smoother"]
    output.mkdir(parents=True, exist_ok=True)
    for name in ["A_thetastar_simple", "B_yastar_simple", "C_yastar_tdt"]:
        config = copy.deepcopy(base)
        if name.startswith("A_"):
            config["planner_server"] = copy.deepcopy(original["planner_server"])
        if name.startswith(("A_", "B_")):
            params = config["smoother_server"]["ros__parameters"]
            params["smoother_plugins"] = ["simple_smoother"]
            params["simple_smoother"] = copy.deepcopy(simple)
        target = output / (name + ".yaml")
        target.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
        print(target)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path(__file__).resolve().parents[4])
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    prepare(args.workspace, args.out)
