#!/usr/bin/env python3

import re
from pathlib import Path

import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONTRACT_PATH = PACKAGE_ROOT / "config" / "common" / "frame_contract.yaml"


def _parameters(data, node):
    return data[node][node]["ros__parameters"]


def test_current_costmap_frames_match_the_shared_contract():
    contract = yaml.safe_load(CONTRACT_PATH.read_text(encoding="utf-8"))
    files = sorted((PACKAGE_ROOT / "config").glob("*/current/nav2_params_srm*.yaml"))
    assert files, "no current SRM Nav2 parameter files found"

    for path in files:
        profile = "fakeyaw" if "fakeyaw" in path.name else "standard"
        expected = contract["frames"]["profiles"][profile]
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        local = _parameters(data, "local_costmap")
        global_map = _parameters(data, "global_costmap")
        assert local["global_frame"] == contract["frames"]["odometry"], path
        assert global_map["global_frame"] == contract["frames"]["global"], path
        assert local["robot_base_frame"] == expected["navigation_base"], path
        assert global_map["robot_base_frame"] == expected["navigation_base"], path


def test_fakeyaw_velocity_topics_match_launch_and_parameter_files():
    contract = yaml.safe_load(CONTRACT_PATH.read_text(encoding="utf-8"))
    topics = contract["velocity_topics"]
    bringup = PACKAGE_ROOT / "launch"
    launch_sources = [
        (bringup / "nav_srm_simulation_launch.py").read_text(encoding="utf-8"),
        (bringup / "nav_srm_real_launch.py").read_text(encoding="utf-8"),
    ]
    for source in launch_sources:
        assert re.search(rf'FAKEYAW_NAV_TOPIC\s*=\s*"{topics["fakeyaw_nav_input"]}"', source)
        assert re.search(rf'FAKEYAW_MUX_TOPIC\s*=\s*"{topics["fakeyaw_body_output"]}"', source)

    for path in (
        PACKAGE_ROOT / "config/simulation/current/nav2_params_srm_fakeyaw.yaml",
        PACKAGE_ROOT / "config/real/current/nav2_params_srm_minco_fakeyaw.yaml",
    ):
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        adapter = data["fake_vel_transform"]["ros__parameters"]
        assert adapter["input_cmd_vel_topic"] == topics["fakeyaw_nav_input"], path
        assert adapter["output_cmd_vel_topic"] == topics["fakeyaw_body_output"], path


def test_tf_publisher_ownership_and_localization_mode_contracts_are_explicit():
    contract = yaml.safe_load(CONTRACT_PATH.read_text(encoding="utf-8"))
    pairs = {(item["parent"], item["child"]): item["active_publishers"]
             for item in contract["tf_ownership"]}
    assert pairs[("map", "odom")] == 1
    assert pairs[("odom", "localization_base")] == 1
    assert contract["localization_modes"]["exactly_one_active"] == [
        "point_lio", "zlio", "gicp", "ground_truth"
    ]
