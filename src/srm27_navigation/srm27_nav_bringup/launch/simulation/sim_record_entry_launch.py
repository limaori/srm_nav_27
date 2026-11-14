#!/usr/bin/env python3
# Copyright 2026 SRM
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""前缀别名：sim_record_entry_launch.py -> srm_simulation_record_launch.py。

ROS 2 的 `ros2 launch <pkg> <file>` 会在 share/launch 下按 basename 递归搜索；
同一 basename 出现两次会直接报 "found more than once"。因此分组入口使用**唯一前缀名**，
逻辑仍只有根目录 canonical 一份。

用法：
  ros2 launch srm27_nav_bringup sim_record_entry_launch.py
或直接：
  ros2 launch srm27_nav_bringup srm_simulation_record_launch.py
"""

import importlib.util
from pathlib import Path

_CANONICAL = Path(__file__).resolve().parent.parent / "srm_simulation_record_launch.py"
_SPEC = importlib.util.spec_from_file_location("srm27_canonical_srm_simulation_record_launch", _CANONICAL)
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)


def generate_launch_description():
    return _MODULE.generate_launch_description()
