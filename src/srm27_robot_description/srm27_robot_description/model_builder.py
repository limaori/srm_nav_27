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

* 只生成 SRM 自有部件（圆柱底盘、四个轮组、MID360 雷达、底盘 IMU）。
  老步兵的装甲、灯条、射击、云台关节链和旧底盘控制器都不再生成。
* ``base_link`` 是与实际底盘刚性固定的根坐标系，随底盘自转；导航速度参考系
  和 ``odom -> base_link`` 都使用它，模型里不再有 ``base_footprint``、
  ``gimbal_yaw``、``gimbal_yaw_fake`` 这些中间坐标系。
* 雷达帧名保持 ``front_mid360``，传感器名保持 ``front_mid360_lidar`` /
  ``front_mid360_imu``，使现有 ros_gz_bridge、点云处理与定位链路无需改动。
* 底盘速度执行由 ``srm27_gazebo_simulator`` 的 ``srm_velocity_system`` 插件负责。
"""

import math

from srm27_robot_description import load_geometry

__all__ = ["build_urdf", "build_sdf", "format_vector"]

DEFAULT_PLUGIN_LIBRARY = "libsrm_velocity_system.so"
DEFAULT_PLUGIN_NAME = "srm27::gazebo::systems::SrmVelocitySystem"


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


def build_urdf(geometry=None):
    """生成供 robot_state_publisher 使用的 URDF 文本。"""
    config = geometry or load_geometry()
    base_link = config["robot"]["base_link"]
    chassis = config["chassis"]
    wheel = config["wheel"]
    lidar = config["lidar"]

    wheel_links = []
    wheel_joints = []
    for name, position in wheel["positions"].items():
        link_name = f"{name}_wheel"
        wheel_links.append(
            f"""
  <link name="{link_name}">
    <visual>
      <origin xyz="0 0 0" rpy="1.5707963267948966 0 0"/>
      <geometry>
        <cylinder radius="{wheel['radius']!r}" length="{wheel['width']!r}"/>
      </geometry>
      <material name="black">
        <color rgba="0.05 0.05 0.05 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="1.5707963267948966 0 0"/>
      <geometry>
        <cylinder radius="{wheel['radius']!r}" length="{wheel['width']!r}"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="{wheel['mass']!r}"/>
      <inertia ixx="{_cylinder_inertia(wheel['mass'], wheel['radius'], wheel['width'], 'y')[0]!r}"
               ixy="0" ixz="0"
               iyy="{_cylinder_inertia(wheel['mass'], wheel['radius'], wheel['width'], 'y')[1]!r}"
               iyz="0"
               izz="{_cylinder_inertia(wheel['mass'], wheel['radius'], wheel['width'], 'y')[2]!r}"/>
    </inertial>
  </link>"""
        )
        wheel_joints.append(
            f"""
  <joint name="{base_link}_to_{name}_wheel" type="revolute">
    <parent link="{base_link}"/>
    <child link="{link_name}"/>
    <origin xyz="{format_vector(position)}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.7976931348623157e+308" upper="1.7976931348623157e+308"
           effort="{wheel['joint_effort']!r}" velocity="{wheel['joint_velocity']!r}"/>
    <dynamics damping="0.01" friction="0.0"/>
  </joint>"""
        )

    chassis_inertia = _cylinder_inertia(chassis["mass"], chassis["radius"], chassis["length"], "z")
    lidar_inertia = _cylinder_inertia(lidar["mass"], 0.04, 0.04, "z")

    return f"""<?xml version="1.0"?>
<!-- 由 srm27_robot_description/model_builder.py 生成，请勿手工修改。 -->
<robot name="{config['robot']['name']}">
  <link name="{base_link}">
    <visual>
      <origin xyz="0 0 {chassis['center_z']!r}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="{chassis['radius']!r}" length="{chassis['length']!r}"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 {chassis['center_z']!r}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="{chassis['radius']!r}" length="{chassis['length']!r}"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 {chassis['center_z']!r}" rpy="0 0 0"/>
      <mass value="{chassis['mass']!r}"/>
      <inertia ixx="{chassis_inertia[0]!r}" ixy="0" ixz="0"
               iyy="{chassis_inertia[1]!r}" iyz="0"
               izz="{chassis_inertia[2]!r}"/>
    </inertial>
  </link>
{''.join(wheel_links)}
{''.join(wheel_joints)}

  <link name="{lidar['link_name']}">
    <visual>
      <origin xyz="{format_vector(lidar['mesh_urdf_origin_xyz'])}"
              rpy="{format_vector(lidar['mesh_urdf_origin_rpy'])}"/>
      <geometry>
        <mesh filename="{lidar['mesh_urdf']}" scale="{format_vector([lidar['mesh_urdf_scale']] * 3)}"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="{lidar['mass']!r}"/>
      <inertia ixx="{lidar_inertia[0]!r}" ixy="0" ixz="0"
               iyy="{lidar_inertia[1]!r}" iyz="0"
               izz="{lidar_inertia[2]!r}"/>
    </inertial>
  </link>

  <joint name="{base_link}_to_{lidar['link_name']}" type="fixed">
    <parent link="{base_link}"/>
    <child link="{lidar['link_name']}"/>
    <origin xyz="{format_vector(lidar['xyz'])}" rpy="{format_vector(lidar['rpy'])}"/>
  </joint>
</robot>
"""


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

    wheel_blocks = []
    for name, position in wheel["positions"].items():
        wheel_blocks.append(
            f"""
      <joint name="{name}_wheel_joint" type="revolute">
        <parent>{base_link}</parent>
        <child>{name}_wheel</child>
        <pose relative_to="{base_link}">{format_vector(position)} 0 0 0</pose>
        <axis>
          <xyz>0 1 0</xyz>
          <limit>
            <lower>-1.7976931348623157e+308</lower>
            <upper>1.7976931348623157e+308</upper>
            <effort>{wheel['joint_effort']!r}</effort>
            <velocity>{wheel['joint_velocity']!r}</velocity>
          </limit>
        </axis>
      </joint>
      <link name="{name}_wheel">
        <pose relative_to="{name}_wheel_joint">0 0 0 0 0 0</pose>
        <inertial>
          <mass>{wheel['mass']!r}</mass>
          {_sdf_inertia(wheel['mass'], wheel_inertia)}
        </inertial>
        <visual name="{name}_wheel_visual">
          <pose>0 0 0 1.5707963267948966 0 0</pose>
          <geometry>
            <cylinder>
              <radius>{wheel['radius']!r}</radius>
              <length>{wheel['width']!r}</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.05 0.05 0.05 1</ambient>
            <diffuse>0.05 0.05 0.05 1</diffuse>
          </material>
        </visual>
        <collision name="{name}_wheel_collision">
          <pose>0 0 0 1.5707963267948966 0 0</pose>
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
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
        </material>
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
      <sensor name="{imu['sensor_name']}" type="imu">
        <ignition_frame_id>{base_link}</ignition_frame_id>
        <always_on>1</always_on>
        <update_rate>{imu['update_rate']!r}</update_rate>
      </sensor>
    </link>
{''.join(wheel_blocks)}

    <joint name="{base_link}_to_{lidar['link_name']}" type="fixed">
      <parent>{base_link}</parent>
      <child>{lidar['link_name']}</child>
      <pose relative_to="{base_link}">{format_vector(lidar['xyz'])} {format_vector(lidar['rpy'])}</pose>
    </joint>
    <link name="{lidar['link_name']}">
      <pose relative_to="{base_link}_to_{lidar['link_name']}">0 0 0 0 0 0</pose>
      <inertial>
        <mass>{lidar['mass']!r}</mass>
        {_sdf_inertia(lidar['mass'], lidar_inertia)}
      </inertial>
      <visual name="visual">
        <pose>{format_vector([0.0, 0.0, 0.0])} {format_vector(lidar['mesh_sdf_origin_rpy'])}</pose>
        <geometry>
          <mesh>
            <uri>{lidar['mesh_sdf']}</uri>
          </mesh>
        </geometry>
      </visual>
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
