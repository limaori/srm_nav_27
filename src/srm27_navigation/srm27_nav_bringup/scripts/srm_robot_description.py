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

"""实车车体描述（robot_description）的唯一构建入口。

所有需要 **实车** robot_description 的 launch 都走这里，保证同一台车在
"建图 / 导航 / 独立调试"三条链路上发布的 TF 完全一致：

  1. 处理唯一模型 ``srm27_robot_description/urdf/srm27_sentry.urdf.xacro``，
     按实车契约注入参数：
        root_frame        = base_link   （原点在地面，odom -> base_link 直接指向它）
        wheel_joint_type  = fixed       （实车没有 /joint_states 发布者）
        gimbal_joint_type = fixed       （云台只是示意包络，没人发云台角）
        lidar_link_name   = livox_frame （雷达驱动 frame_id / Point-LIO frame_id）
        lidar_xyz / lidar_roll / lidar_pitch / lidar_yaw
  2. 追加两个实车专用坐标系（模型文件里没有，因为它们只服务实车链路）：
        livox_imu  : 挂在 livox_frame 下，偏移取 -point_lio.mapping.extrinsic_T
        livox_scan : 挂在 base_link 下，偏移取 lidar_xyz（二维投影用的水平系）
  3. 保留 mesh 前缀改写（历史遗留模型的 ``package://pb_rm_simulation/`` 安全网）。

雷达安装位姿的**唯一来源是唯一模型本身**：``build_description()`` 展开 xacro 之后
会把 ``base_link -> <lidar_link>`` 的组合位姿解析出来返回（`Mounting`），
launch 用它算 Point-LIO 的 gravity 和 pointcloud_to_laserscan 的切片高度，
所以不再需要在 launch 里再抄一遍 CAD 数值。

``lidar_xyz`` / ``lidar_rpy`` 仍然可以显式传入（现场临时改安装角时用）；
传空字符串表示"跟着模型走"。
"""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

#: 唯一模型文件在 srm27_robot_description 包内的相对路径
MODEL_RELPATH = Path("urdf") / "srm27_sentry.urdf.xacro"

#: 本链路要求的雷达-IMU 旋转（MID360 内部 IMU 与雷达同姿态）
IDENTITY_ROTATION = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

LEGACY_MESH_PREFIX = "package://pb_rm_simulation/"


def robot_model_path():
    """返回唯一模型文件的绝对路径。"""
    return Path(get_package_share_directory("srm27_robot_description")) / MODEL_RELPATH


def parse_triplet(text, name):
    """把 "x y z" 解析成三个有限浮点数，解析失败给出看得懂的错。"""
    values = [float(value) for value in str(text).split()]
    if len(values) != 3:
        raise ValueError(f"{name} 必须包含三个数, 收到: {text!r}")
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{name} 必须是有限数, 收到: {text!r}")
    return values


def load_imu_translation(params_file):
    """从 nav2 参数文件读 Point-LIO 的雷达->IMU 平移（livox_imu 的偏移来源）。"""
    with open(params_file, encoding="utf-8") as params_stream:
        params = yaml.safe_load(params_stream)
    try:
        mapping = params["point_lio"]["ros__parameters"]["mapping"]
    except (KeyError, TypeError) as error:
        raise ValueError(
            f"{params_file} 缺少 point_lio.ros__parameters.mapping 段: {error}"
        ) from error
    translation = parse_triplet(_as_vector_text(mapping["extrinsic_T"]), "extrinsic_T")
    if list(mapping["extrinsic_R"]) != IDENTITY_ROTATION:
        raise ValueError(
            "This MID360 model requires identity LiDAR-to-IMU rotation "
            f"(extrinsic_R), got {mapping['extrinsic_R']}"
        )
    return translation


def _as_vector_text(values):
    if isinstance(values, str):
        return values
    return " ".join(str(value) for value in values)


def _rewrite_legacy_meshes(robot):
    """把历史遗留的 package://pb_rm_simulation/ 引用改写到描述包内。

    当前模型是纯基本体（无 mesh），这段是给"万一又引入旧 mesh"留的安全网。
    """
    description_dir = Path(get_package_share_directory("srm27_robot_description"))
    for mesh in robot.iter("mesh"):
        filename = mesh.attrib["filename"]
        if not filename.startswith(LEGACY_MESH_PREFIX):
            continue
        relative = filename.removeprefix(LEGACY_MESH_PREFIX)
        if not (description_dir / relative).is_file():
            raise FileNotFoundError(description_dir / relative)
        mesh.set("filename", f"package://srm27_robot_description/{relative}")


def _quaternion_from_rpy(roll, pitch, yaw):
    """rpy（弧度）-> 四元数 (x, y, z, w)。"""
    half_roll, half_pitch, half_yaw = roll * 0.5, pitch * 0.5, yaw * 0.5
    cr, sr = math.cos(half_roll), math.sin(half_roll)
    cp, sp = math.cos(half_pitch), math.sin(half_pitch)
    cy, sy = math.cos(half_yaw), math.sin(half_yaw)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _rpy_from_quaternion(q):
    """四元数 (x, y, z, w) -> rpy（弧度），ZYX 分解。"""
    x, y, z, w = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def _quaternion_multiply(first, second):
    ax, ay, az, aw = first
    bx, by, bz, bw = second
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _rotate(quaternion, vector):
    """用四元数旋转向量。"""
    x, y, z, w = quaternion
    vx, vy, vz = vector
    # v' = q * v * q^-1，展开成不含矩阵库的形式
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def _joint_origin(joint):
    """关节 origin 的 (xyz, rpy)，缺省为恒等。"""
    origin = joint.find("origin")
    if origin is None:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    xyz = [float(value) for value in (origin.get("xyz") or "0 0 0").split()]
    rpy = [float(value) for value in (origin.get("rpy") or "0 0 0").split()]
    return xyz, rpy


def lidar_mounting(robot, base_link, lidar_link):
    """从展开后的 URDF 解析 ``base_link -> lidar_link`` 的组合位姿。

    路径上可能是多段固定关节（例如 ``base_link -> gimbal_yaw -> livox_frame``），
    这里把每段 origin 依次复合；遇到非固定关节（云台放开成 revolute 时）按**零位**
    处理——那正是"安装位姿"的语义（安装角与云台角是两回事）。
    """
    parents = {}
    for joint in robot.findall("joint"):
        parents[joint.find("child").get("link")] = joint

    chain = []
    link = lidar_link
    while link != base_link:
        joint = parents.get(link)
        if joint is None:
            raise ValueError(f"URDF 里 {link!r} 到 {base_link!r} 没有关节链")
        chain.append(joint)
        link = joint.find("parent").get("link")
        if len(chain) > 32:
            raise ValueError("URDF 关节链异常（疑似成环）")
    chain.reverse()

    position = [0.0, 0.0, 0.0]
    orientation = (0.0, 0.0, 0.0, 1.0)
    for joint in chain:
        xyz, rpy = _joint_origin(joint)
        rotated = _rotate(orientation, xyz)
        position = [position[i] + rotated[i] for i in range(3)]
        orientation = _quaternion_multiply(orientation, _quaternion_from_rpy(*rpy))
    return {"xyz": position, "rpy": list(_rpy_from_quaternion(orientation))}


def build_description(params_file, lidar_xyz, lidar_rpy, lidar_link_name="livox_frame"):
    """构建实车 robot_description。

    返回 ``(urdf_text, mounting)``：``mounting`` 是解析出来的 ``base_link -> 雷达``
    组合位姿（``{"xyz": [...], "rpy": [...]}``），launch 用它算 gravity 与切片高度。

    ``lidar_xyz`` / ``lidar_rpy`` 传空字符串表示"用模型里的默认值"。
    """
    mappings = {
        "root_frame": "base_link",
        "wheel_joint_type": "fixed",
        "gimbal_joint_type": "fixed",
        "lidar_link_name": lidar_link_name,
    }
    if str(lidar_xyz).strip():
        position = parse_triplet(lidar_xyz, "lidar_xyz")
        mappings["lidar_xyz"] = " ".join(repr(value) for value in position)
    if str(lidar_rpy).strip():
        roll, pitch, yaw = parse_triplet(lidar_rpy, "lidar_rpy")
        mappings["lidar_roll"] = repr(roll)
        mappings["lidar_pitch"] = repr(pitch)
        mappings["lidar_yaw"] = repr(yaw)

    imu_translation = load_imu_translation(params_file)
    robot_xml = xacro.process_file(str(robot_model_path()), mappings=mappings).toxml()
    robot = ET.fromstring(robot_xml)
    _rewrite_legacy_meshes(robot)

    # 安装位姿从模型本身解析（含云台零位），不再由 launch 另抄一份数值。
    mounting = lidar_mounting(robot, "base_link", lidar_link_name)

    for child, parent, translation in [
        ("livox_imu", lidar_link_name, [-value for value in imu_translation]),
        # livox_scan 是**水平**的二维切片系：留在 base_link 下，高度取雷达高度。
        ("livox_scan", "base_link", mounting["xyz"]),
    ]:
        ET.SubElement(robot, "link", name=child)
        joint = ET.SubElement(robot, "joint", name=f"{child}_joint", type="fixed")
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=child)
        ET.SubElement(
            joint,
            "origin",
            xyz=" ".join(str(value) for value in translation),
            rpy="0 0 0",
        )

    return ET.tostring(robot, encoding="unicode"), mounting


def gravity_from_mounting(lidar_rpy):
    """雷达安装姿态对应的 Point-LIO 重力向量（重力在雷达系下的表达）。

    雷达装歪 roll 时，"世界重力"在雷达系里就不再是 -z，必须与 URDF 外参用
    **同一组** roll/pitch 算出来，否则两处各错一半。
    """
    roll, pitch, _ = parse_triplet(
        " ".join(str(value) for value in lidar_rpy), "lidar_rpy"
    )
    return [
        9.81 * math.sin(pitch),
        -9.81 * math.cos(pitch) * math.sin(roll),
        -9.81 * math.cos(pitch) * math.cos(roll),
    ]
