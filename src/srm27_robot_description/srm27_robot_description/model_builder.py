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

"""由 config/srm27_sentry_geometry.yaml 生成 SRM 的 URDF 与 SDF。

设计约定：

* **URDF 只有一份**：``urdf/srm27_sentry.urdf.xacro``。``build_urdf()`` 不自己拼
  XML，而是把几何 YAML 的数值灌进那份 xacro 再展开 —— 实车链路（建图/导航/独立
  调试）和仿真显示用的是同一个 XML 结构，只有坐标系契约参数不同
  （实车 : 轮子与云台 fixed；仿真 : 轮子 continuous，关节名与 SDF 一致）。
* SDF 仍由本文件的 ``build_sdf()`` 生成（Gazebo 需要插件、传感器、摩擦等 SDF 专属内容），
  几何数值同样只来自那份 YAML；URDF 与 YAML 的数值一致性由测试强制保证。
* 只生成 SRM 自有部件（圆柱底盘、四个轮组、MID360 雷达、底盘 IMU）。
  老步兵的装甲、灯条、射击、云台关节链和旧底盘控制器都不再生成。
* ``base_link`` 是与实际底盘刚性固定的根坐标系，随底盘自转，原点在地面；
  导航速度参考系和 ``odom -> base_link`` 都使用它，模型里没有 ``base_footprint``、
  ``gimbal_yaw_fake`` 这些中间坐标系。
* 雷达帧名保持 ``front_mid360``，传感器名保持 ``front_mid360_lidar`` /
  ``front_mid360_imu``，使现有 ros_gz_bridge、点云处理与定位链路无需改动。
* 底盘速度执行由 ``srm27_gazebo_simulator`` 的 ``srm_velocity_system`` 插件负责。
"""

import math
import os
from pathlib import Path

import xacro

from srm27_robot_description import load_geometry

__all__ = ["build_urdf", "build_sdf", "format_vector", "urdf_model_path"]

DEFAULT_PLUGIN_LIBRARY = "libsrm_velocity_system.so"
DEFAULT_PLUGIN_NAME = "srm27::gazebo::systems::SrmVelocitySystem"

#: 唯一 URDF 模型在包内的相对路径（实车链路用的是同一个文件）
URDF_MODEL_RELPATH = Path("urdf") / "srm27_sentry.urdf.xacro"

#: 轮子安装参数（轴向 yaw, 外观 yaw），与 xacro 里 omni_wheel 的调用一一对应。
#: 对角两组共线：front_left/rear_right 沿 +45°，rear_left/front_right 沿 135°。
WHEEL_MOUNTING = {
    "front_left": (math.pi / 4.0, -math.pi / 4.0),
    "rear_left": (3.0 * math.pi / 4.0, math.pi / 4.0),
    "front_right": (-math.pi / 4.0, -3.0 * math.pi / 4.0),
    "rear_right": (math.pi / 4.0, 3.0 * math.pi / 4.0),
}

#: 旋转标记的形状（与 urdf/ 那份 xacro 的 ``rotation_marks`` 宏调用**逐项一致**，
#: 改一处必须两处一起改）：侧壁竖条弧长 arc / 径向厚度 thick、顶面刻线宽 tick_w /
#: 厚 tick_t、定位球半径 dot_r。单位米。
ROTATION_MARK_STYLE = {
    "chassis": {"arc": 0.055, "thick": 0.012, "tick_w": 0.030, "tick_t": 0.010, "dot_r": 0.016},
    "gimbal": {"arc": 0.045, "thick": 0.010, "tick_w": 0.024, "tick_t": 0.008, "dot_r": 0.014},
}

#: 旋转标记颜色：底盘灰 → 红，云台半透明黄 → 蓝（与 xacro 的 rgba 一致）
CHASSIS_MARK_COLOR = [0.90, 0.13, 0.10, 1.0]
GIMBAL_MARK_COLOR = [0.05, 0.45, 1.0, 1.0]

#: 顶面刻线的内端留白：底盘刻线要避开云台圆柱（yaw_radius + 这个量），
#: 云台刻线从中心附近起步。与 xacro 里 tick_in 的写法一致。
CHASSIS_TICK_INNER_GAP = 0.012
GIMBAL_TICK_INNER = 0.020


def urdf_model_path():
    """定位唯一 URDF 模型文件（安装后的 share 优先，源码树兜底）。"""
    candidates = []
    override = os.environ.get("SRM27_SENTRY_URDF")
    if override:
        candidates.append(Path(override))
    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.append(
            Path(get_package_share_directory("srm27_robot_description"))
            / URDF_MODEL_RELPATH
        )
    except Exception:  # 未 source 环境时退回源码树，测试可以在裸 python 下跑
        pass
    candidates.append(Path(__file__).resolve().parents[1] / URDF_MODEL_RELPATH)
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(
        "找不到唯一 URDF 模型，试过: " + ", ".join(str(path) for path in candidates)
    )


def format_vector(values):
    """把浮点序列格式化为 SDF/URDF 接受的空格分隔字符串。"""
    return " ".join(repr(float(value)) for value in values)


def _cylinder_inertia(mass, radius, length, axis="z"):
    """返回轴线沿指定轴的实心圆柱惯量 (ixx, iyy, izz)。"""
    radial = mass * (3.0 * radius * radius + length * length) / 12.0
    axial = mass * radius * radius / 2.0
    if axis == "z":
        return radial, radial, axial
    if axis == "y":
        return radial, axial, radial
    if axis == "x":
        return axial, radial, radial
    raise ValueError(f"unsupported cylinder axis: {axis}")


def _sdf_inertia(mass, inertia):
    ixx, iyy, izz = inertia
    return (
        "<inertia>"
        f"<ixx>{ixx!r}</ixx><ixy>0</ixy><ixz>0</ixz>"
        f"<iyy>{iyy!r}</iyy><iyz>0</iyz>"
        f"<izz>{izz!r}</izz>"
        "</inertia>"
    )


def _sdf_material(color, transparency=None):
    """SDF 材质块。<transparency> 0=不透明、1=全透明。"""
    rgb = " ".join(repr(float(value)) for value in list(color)[:3])
    text = (
        "          <material>\n"
        f"            <ambient>{rgb} 1</ambient>\n"
        f"            <diffuse>{rgb} 1</diffuse>\n"
        f"            <specular>0.1 0.1 0.1 1</specular>\n"
    )
    if transparency is not None:
        text += f"            <transparency>{float(transparency)!r}</transparency>\n"
    return text + "          </material>"


def _rotation_marks_sdf(prefix, radius, length, base_z, style, color, tick_in, tick_out):
    """旋转标记（SDF 版），与 xacro 的 ``rotation_marks`` 宏逐项对应。

    底盘和云台都是绕 z 轴的**旋转体**：转到哪个角度看起来都一样，光圆柱在 RViz /
    Gazebo 里转起来像没动。所以每个旋转体挂三块不对称的标记，全部朝 +x（车头）：

      ① ``<prefix>_mark_stripe`` 侧壁竖条（抬高 thick/2，一半埋进圆柱）
      ② ``<prefix>_mark_tick``   顶面径向刻线（俯视时的"时针"）
      ③ ``<prefix>_mark_dot``    刻线末端定位球

    ⚠ 三块都只是 visual，**不生成 collision**：Gazebo 的碰撞体与导航 footprint 不变。
    """
    radius = float(radius)
    length = float(length)
    base_z = float(base_z)
    thick = float(style["thick"])
    # 弧长 → 弦长：贴在外壁上的那条带子的横向宽度
    chord = 2.0 * radius * math.sin(float(style["arc"]) / (2.0 * radius))
    height = length - 0.008
    tick_in = float(tick_in)
    tick_out = float(tick_out)
    tick_w = float(style["tick_w"])
    tick_t = float(style["tick_t"])
    dot_r = float(style["dot_r"])
    material = _sdf_material(color)
    return f"""      <visual name="{prefix}_mark_stripe">
        <pose>{radius!r} 0 {base_z + length / 2.0!r} 0 0 0</pose>
        <geometry>
          <box><size>{thick!r} {chord!r} {height!r}</size></box>
        </geometry>
{material}
      </visual>
      <visual name="{prefix}_mark_tick">
        <pose>{(tick_in + tick_out) / 2.0!r} 0 {base_z + length + tick_t / 2.0!r} 0 0 0</pose>
        <geometry>
          <box><size>{tick_out - tick_in!r} {tick_w!r} {tick_t!r}</size></box>
        </geometry>
{material}
      </visual>
      <visual name="{prefix}_mark_dot">
        <pose>{tick_out - dot_r!r} 0 {base_z + length + dot_r / 2.0!r} 0 0 0</pose>
        <geometry>
          <sphere><radius>{dot_r!r}</radius></sphere>
        </geometry>
{material}
      </visual>"""


def _lidar_visual_sdf(lidar):
    """SDF 版雷达外观，与 _lidar_visual_urdf 保持同一套形状。"""
    spec = lidar.get("visual") or {}
    if spec.get("mode", "sphere") == "mesh":
        return f"""      <visual name="{lidar['link_name']}_mesh">
        <pose>0 0 0 {format_vector(lidar['mesh_sdf_origin_rpy'])}</pose>
        <geometry>
          <mesh>
            <uri>{lidar['mesh_sdf']}</uri>
          </mesh>
        </geometry>
{_sdf_material([0.0, 0.0, 1.0])}
      </visual>"""

    blocks = []
    bracket_length = float(spec.get("bracket_length", 0.0) or 0.0)
    if bracket_length > 0.0:
        blocks.append(
            f"""      <visual name="{lidar['link_name']}_bracket">
        <pose>0 0 {-bracket_length / 2.0!r} 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>{float(spec['bracket_radius'])!r}</radius>
            <length>{bracket_length!r}</length>
          </cylinder>
        </geometry>
{_sdf_material(spec['bracket_color'])}
      </visual>"""
        )
    blocks.append(
        f"""      <visual name="{lidar['link_name']}_dot">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <sphere>
            <radius>{float(spec['radius'])!r}</radius>
          </sphere>
        </geometry>
{_sdf_material(spec['color'])}
      </visual>"""
    )
    return "\n".join(blocks)


def yaw_link_name(config):
    """云台 link 的名字（雷达挂在它下面，所以必须始终存在）。"""
    return (config.get("yaw_envelope") or {}).get("link_name", "gimbal_yaw")


def yaw_top_z(config, base_link):
    """云台基座面高度 = 底盘顶面（云台与底盘同轴、端面贴合）。"""
    chassis = config["chassis"]
    del base_link
    return float(chassis["center_z"]) + float(chassis["length"]) / 2.0


def _yaw_visual_sdf(config, base_link):
    """SDF 版云台：joint + link **始终生成**（雷达是它的子节点），
    示意圆柱只在 yaw_envelope.enabled 时画，不建 collision、不加惯性。"""
    yaw = config.get("yaw_envelope") or {}
    top_z = yaw_top_z(config, base_link)
    name = yaw_link_name(config)
    joint_name = f"{name}_joint"
    if not yaw.get("enabled"):
        visual = ""
    else:
        length = float(yaw["length"])
        radius = float(yaw["radius"])
        # 旋转标记：云台单独转（实车云台角来自 /joint_states）时，蓝色标记相对
        # 底盘的红色标记错开一个角度，"谁在转"一眼可见。同样只有 visual。
        marks = _rotation_marks_sdf(
            "gimbal",
            radius,
            length,
            0.0,
            ROTATION_MARK_STYLE["gimbal"],
            GIMBAL_MARK_COLOR,
            GIMBAL_TICK_INNER,
            radius - 0.006,
        )
        visual = f"""
      <visual name="{name}_visual">
        <pose>0 0 {length / 2.0!r} 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>{radius!r}</radius>
            <length>{length!r}</length>
          </cylinder>
        </geometry>
{_sdf_material(yaw['color'], yaw.get('transparency'))}
      </visual>
{marks}"""
    return f"""
    <joint name="{joint_name}" type="fixed">
      <parent>{base_link}</parent>
      <child>{name}</child>
      <pose relative_to="{base_link}">0 0 {top_z!r} 0 0 0</pose>
    </joint>
    <link name="{name}">
      <pose relative_to="{joint_name}">0 0 0 0 0 0</pose>{visual}
    </link>"""


def _urdf_mappings(config):
    """把几何 YAML 的数值映射成唯一 xacro 的参数。

    仿真侧与实车侧只有坐标系契约不同（见 urdf/srm27_sentry.urdf.xacro 文件头）：
    仿真的轮子是 continuous（Gazebo 会按 SDF 的关节名发 /joint_states），
    云台仍是 fixed（仿真里没有云台驱动）。
    """
    chassis = config["chassis"]
    wheel = config["wheel"]
    lidar = config["lidar"]
    yaw = config.get("yaw_envelope") or {}
    visual = lidar.get("visual") or {}
    front_left = wheel["positions"]["front_left"]
    bracket_length = float(visual.get("bracket_length", 0.0) or 0.0)

    return {
        # 坐标系契约
        "root_frame": "base_link",
        "wheel_joint_type": "continuous",
        "gimbal_joint_type": "fixed",
        # 底盘 / 轮组
        "chassis_radius": repr(float(chassis["radius"])),
        "chassis_length": repr(float(chassis["length"])),
        "chassis_centre_z": repr(float(chassis["center_z"])),
        "chassis_mass": repr(float(chassis["mass"])),
        "wheel_radius": repr(float(wheel["radius"])),
        "wheel_width": repr(float(wheel["width"])),
        "wheel_mass": repr(float(wheel["mass"])),
        "wheel_effort": repr(float(wheel["joint_effort"])),
        "wheel_velocity": repr(float(wheel["joint_velocity"])),
        "wheel_x": repr(abs(float(front_left[0]))),
        "wheel_y": repr(abs(float(front_left[1]))),
        "wheel_z": repr(float(front_left[2])),
        # 云台示意包络
        "use_yaw": "true" if yaw.get("enabled") else "false",
        "yaw_radius": repr(float(yaw.get("radius", 0.150))),
        "yaw_length": repr(float(yaw.get("length", 0.110))),
        # 雷达（注意 visual.mode: mesh 只影响 SDF 外观，URDF 固定用 CAD 示意版）
        "lidar_link_name": lidar["link_name"],
        "lidar_xyz": format_vector(lidar["xyz"]),
        "lidar_roll": repr(float(lidar["rpy"][0])),
        "lidar_pitch": repr(float(lidar["rpy"][1])),
        "lidar_yaw": repr(float(lidar["rpy"][2])),
        "lidar_mass": repr(float(lidar["mass"])),
        "lidar_radius": repr(float(visual.get("radius", 0.014))),
        "lidar_post_length": repr(bracket_length),
    }


def build_urdf(geometry=None):
    """展开唯一 xacro 得到机器人描述（所有链路共用同一份 XML 结构）。

    与实车链路的差别只在映射：实车用 ``fixed`` 轮子、``livox_frame`` 帧名；
    这里用 ``continuous`` 轮子、``front_mid360`` 帧名，其余完全相同。
    """
    config = geometry or load_geometry()
    return xacro.process_file(
        str(urdf_model_path()), mappings=_urdf_mappings(config)
    ).toxml()


def build_sdf(
    geometry=None,
    plugin_library=DEFAULT_PLUGIN_LIBRARY,
    plugin_name=DEFAULT_PLUGIN_NAME,
    velocity_topic="cmd_vel",
    odometry_topic="odometry",
    command_timeout=0.1,
):
    """生成供 Gazebo Fortress 加载的完整 SRM 模型 SDF 文本。

    ``velocity_topic`` / ``odometry_topic`` 为相对话题名，Gazebo Transport 会
    自动加上模型名前缀，例如 ``<model>/cmd_vel`` 与 ``<model>/odometry``。
    """
    config = geometry or load_geometry()
    model_name = config["robot"]["name"]
    base_link = config["robot"]["base_link"]
    chassis = config["chassis"]
    wheel = config["wheel"]
    lidar = config["lidar"]
    imu = config["imu"]

    chassis_inertia = _cylinder_inertia(chassis["mass"], chassis["radius"], chassis["length"], "z")
    wheel_inertia = _cylinder_inertia(wheel["mass"], wheel["radius"], wheel["width"], "y")
    lidar_inertia = _cylinder_inertia(lidar["mass"], 0.04, 0.04, "z")
    # 外观与实车链路 xacro 对齐（见几何 YAML 的 color / visual / yaw_envelope）。
    chassis_material = _sdf_material(chassis.get("color", [0.75, 0.77, 0.80, 1.0]))
    wheel_material = _sdf_material(wheel.get("color", [0.05, 0.05, 0.05, 1.0]))
    lidar_visual = _lidar_visual_sdf(lidar)
    yaw_visual = _yaw_visual_sdf(config, base_link)
    # 底盘的旋转标记（红）：刻线只画在云台外面的那圈顶面上，免得和云台圆柱打架。
    yaw_radius = float((config.get("yaw_envelope") or {}).get("radius", 0.150))
    chassis_marks = _rotation_marks_sdf(
        "chassis",
        chassis["radius"],
        chassis["length"],
        float(chassis["center_z"]) - float(chassis["length"]) / 2.0,
        ROTATION_MARK_STYLE["chassis"],
        CHASSIS_MARK_COLOR,
        yaw_radius + CHASSIS_TICK_INNER_GAP,
        float(chassis["radius"]) - 0.008,
    )
    # 雷达长在云台上：SDF 的关节父节点改成云台 link，pose 相对**云台基座面**给。
    # 零位下与"相对 base_link"完全等价，云台一转传感器就跟着转。
    yaw_link = yaw_link_name(config)
    lidar_pose_rel_yaw = "{xyz} {rpy}".format(
        xyz=format_vector(
            [lidar["xyz"][0], lidar["xyz"][1], float(lidar["xyz"][2]) - yaw_top_z(config, base_link)]
        ),
        rpy=format_vector(lidar["rpy"]),
    )

    wheel_blocks = []
    for name, position in wheel["positions"].items():
        # 关节名 / link 名与 URDF（唯一 xacro）完全一致：Gazebo 按 SDF 的关节名发
        # /joint_states，robot_state_publisher 按 URDF 的关节名匹配，名字不一致时
        # 那几个 link 就永远没有 TF。
        # 轴向也按 CAD 的对角布局摆放（X 型全向轮），与 URDF 一致。
        axis_yaw, rpy_yaw = WHEEL_MOUNTING[name]
        joint_name = f"wheel_{name}_joint"
        link_name = f"wheel_{name}"
        wheel_blocks.append(
            f"""
      <joint name="{joint_name}" type="revolute">
        <parent>{base_link}</parent>
        <child>{link_name}</child>
        <pose relative_to="{base_link}">{format_vector(position)} 0 0 0</pose>
        <axis>
          <xyz>{math.cos(axis_yaw)!r} {math.sin(axis_yaw)!r} 0</xyz>
          <limit>
            <lower>-1.7976931348623157e+308</lower>
            <upper>1.7976931348623157e+308</upper>
            <effort>{wheel['joint_effort']!r}</effort>
            <velocity>{wheel['joint_velocity']!r}</velocity>
          </limit>
        </axis>
      </joint>
      <link name="{link_name}">
        <pose relative_to="{joint_name}">0 0 0 0 0 0</pose>
        <inertial>
          <mass>{wheel['mass']!r}</mass>
          {_sdf_inertia(wheel['mass'], wheel_inertia)}
        </inertial>
        <visual name="{link_name}_visual">
          <pose>0 0 0 -1.5707963267948966 0 {rpy_yaw!r}</pose>
          <geometry>
            <cylinder>
              <radius>{wheel['radius']!r}</radius>
              <length>{wheel['width']!r}</length>
            </cylinder>
          </geometry>
{wheel_material}
        </visual>
        <collision name="{link_name}_collision">
          <pose>0 0 0 -1.5707963267948966 0 {rpy_yaw!r}</pose>
          <geometry>
            <cylinder>
              <radius>{wheel['radius']!r}</radius>
              <length>{wheel['width']!r}</length>
            </cylinder>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>{wheel['mu']!r}</mu>
                <mu2>{wheel['mu']!r}</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
      </link>"""
        )

    return f"""<?xml version="1.0"?>
<!-- 由 srm27_robot_description/model_builder.py 生成，请勿手工修改。 -->
<sdf version="1.7">
  <model name="{model_name}">
    <self_collide>false</self_collide>
    <link name="{base_link}">
      <inertial>
        <pose>0 0 {chassis['center_z']!r} 0 0 0</pose>
        <mass>{chassis['mass']!r}</mass>
        {_sdf_inertia(chassis['mass'], chassis_inertia)}
      </inertial>
      <visual name="chassis_visual">
        <pose>0 0 {chassis['center_z']!r} 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>{chassis['radius']!r}</radius>
            <length>{chassis['length']!r}</length>
          </cylinder>
        </geometry>
{chassis_material}
      </visual>
      <collision name="chassis_collision">
        <pose>0 0 {chassis['center_z']!r} 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>{chassis['radius']!r}</radius>
            <length>{chassis['length']!r}</length>
          </cylinder>
        </geometry>
      </collision>
{chassis_marks}
      <sensor name="{imu['sensor_name']}" type="imu">
        <ignition_frame_id>{base_link}</ignition_frame_id>
        <always_on>1</always_on>
        <update_rate>{imu['update_rate']!r}</update_rate>
      </sensor>
    </link>
{''.join(wheel_blocks)}
{yaw_visual}

    <joint name="gimbal_yaw_to_{lidar['link_name']}" type="fixed">
      <parent>{yaw_link}</parent>
      <child>{lidar['link_name']}</child>
      <pose relative_to="{yaw_link}">{lidar_pose_rel_yaw}</pose>
    </joint>
    <link name="{lidar['link_name']}">
      <pose relative_to="gimbal_yaw_to_{lidar['link_name']}">0 0 0 0 0 0</pose>
      <inertial>
        <mass>{lidar['mass']!r}</mass>
        {_sdf_inertia(lidar['mass'], lidar_inertia)}
      </inertial>
{lidar_visual}
      <sensor name="{lidar['link_name']}_imu" type="imu">
        <ignition_frame_id>{lidar['link_name']}</ignition_frame_id>
        <always_on>1</always_on>
        <update_rate>200</update_rate>
        <visualize>true</visualize>
      </sensor>
      <sensor name="{lidar['link_name']}_lidar" type="gpu_lidar">
        <ignition_frame_id>{lidar['link_name']}</ignition_frame_id>
        <pose>0 0 0.03 0 0 0</pose>
        <always_on>true</always_on>
        <visualize>true</visualize>
        <update_rate>{lidar['update_rate']!r}</update_rate>
        <ray>
          <scan>
            <horizontal>
              <samples>{lidar['samples']!r}</samples>
              <resolution>1.0</resolution>
              <min_angle>{lidar['horizontal_min_angle']!r}</min_angle>
              <max_angle>{lidar['horizontal_max_angle']!r}</max_angle>
            </horizontal>
            <vertical>
              <samples>{lidar['vertical_samples']!r}</samples>
              <min_angle>{lidar['vertical_min_angle']!r}</min_angle>
              <max_angle>{lidar['vertical_max_angle']!r}</max_angle>
            </vertical>
          </scan>
          <range>
            <min>{lidar['range_min']!r}</min>
            <max>{lidar['range_max']!r}</max>
          </range>
        </ray>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>{lidar['noise_stddev']!r}</stddev>
        </noise>
      </sensor>
    </link>

    <plugin filename="ignition-gazebo-joint-state-publisher-system"
            name="ignition::gazebo::systems::JointStatePublisher"/>
    <plugin filename="{plugin_library}" name="{plugin_name}">
      <chassis_link>{base_link}</chassis_link>
      <command_topic>{velocity_topic}</command_topic>
      <odom_topic>{odometry_topic}</odom_topic>
      <odom_publish_frequency>50</odom_publish_frequency>
      <command_timeout>{command_timeout!r}</command_timeout>
    </plugin>
  </model>
</sdf>
"""


def yaw_from_quaternion(x, y, z, w):
    """四元数转 yaw，仅用于自检与调试。"""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
