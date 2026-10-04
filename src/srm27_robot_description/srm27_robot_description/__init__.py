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

"""SRM 机器人描述包。

几何、安装外参和 URDF/SDF 生成集中在本包，仿真与实车不再各自维护一份模型。
"""

import os

import yaml

__all__ = ["get_share_directory", "get_geometry_file", "load_geometry"]


def get_share_directory():
    """返回本包 share 目录（源码树运行时同样可用）。"""
    try:
        from ament_index_python.packages import get_package_share_directory

        return get_package_share_directory("srm27_robot_description")
    except Exception:  # pragma: no cover - 未安装时回退到源码树
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def get_geometry_file():
    """返回几何参数 YAML 的完整路径。"""
    return os.path.join(get_share_directory(), "config", "srm27_sentry_geometry.yaml")


def load_geometry(path=None):
    """读取并返回几何参数字典。"""
    geometry_file = path or get_geometry_file()
    with open(geometry_file, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)
