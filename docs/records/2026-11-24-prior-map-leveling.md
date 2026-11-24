# 2026-11-24 先验地图按地面摆正 + 代价地图高度门槛

> 起因：RViz 里 global_costmap（map 系）和 local_costmap（odom 系，`static_layer`
> 又复制了同一张 `/map`）明显"两层"。`tf2_echo map odom` 读到
> `z=0.063 m`、`RPY=0.215/0.991/6.785°`；yaw 6.8° 是出生点相对地图的正常朝向，
> 但 6.3 cm 的 z 和 0.99° 的 pitch 让局部代价地图平面整体抬高并前倾。
> 8 m 局部窗口上高度差可到 ~10 cm。

## 1. 根因

1. `maps/pcd/219_1121_mapframe.pcd` 是按 CAD 外参 `MOUNT_XYZ.z=0.346388` 摆正的，
   而实车桥接算出的雷达离地是 `gimbal_yaw 0.2681 + livox 0.0285 = 0.2966 m`。
   对地图地面做平面拟合得到 `z≈0.067~0.081 m`、`roll≈0.4° / pitch≈1.0°`（RMS 0.9 cm）。
2. backend 先验模式没有重力/tilt 因子（`addGravityAlignment` 只在无先验 UWB 模式调用），
   6-DoF 配准把这张**本身倾斜**的地图如实对齐，`map→odom` 就带上同样的 z/pitch，
   且输出未平面化（[localization_backend.md §9](../architecture/localization_backend.md)）。
3. 另一个独立问题：local/global 的 `intensity_voxel_layer` 用
   `min_obstacle_height: 0.0`，把 `terrain_map`/`terrain_map_ext` 里的
   **2D free-space 网格点**（`z=vehicleZ≈0`、`intensity=vehicleHeight`）也标成障碍，
   局部图在机器人周围铺出一大片青/蓝 inscribed/inflated。

## 2. 改动

| 文件/产物 | 内容 |
|---|---|
| `scripts/mapping/level_map_pcd.py` | 新增：最低层带内 RANSAC + 最小二乘拟合地面，旋到 +Z、地面平移到 z=0；输出 `*_mapframe_leveled.pcd`，原始件不动 |
| `maps/pcd/219_1121_mapframe_leveled.pcd` | 新产物：拟合 `tilt=1.463°`、`dz=-0.0674 m`、`RMS 0.85 cm`；复拟合残差 `0.044°`、地面 `z≈0` |
| `maps/pcd/227_1113pro_mapframe_leveled.pcd` | 新产物：拟合 `tilt=1.060°`、`dz=-0.0670 m`；复拟合残差 `0.035°`、地面 `z≈0` |
| `maps/pcd/map_bundles/219_1121/` | 用 leveled PCD + `--sensor-z 0.2966` 重建：48 帧，keyframe `tz=0.2966`，描述子 `lidar_height=0.2966` |
| `maps/pcd/map_bundles/227_1113pro/` | 同上：108 帧，keyframe `tz=0.2966` |
| `config/srm27_localization_real.yaml` | `common.scan_context.lidar_height: 0.2966`（不改进 `backend.yaml` 上游基线；仿真 profile 不变） |
| `scripts/mapping/build_zloc_map_bundle.sh` | `--sensor-z` 与配置 `scan_context.lidar_height` 不一致时，描述子改用 `--sensor-z` 并告警（此前会分别取两个值，建出"关键帧 z 一个高度、描述子另一个高度"的包） |
| `config/real/current/nav2_params_srm_minco_fakeyaw.yaml` | local/global 的 `intensity_voxel_layer` 与两个 observation source：`min_obstacle_height: 0.0 → 0.10` |
| backend `output/planar_map_to_odom` | 新增 4-DoF 投影开关（默认 false）：对外 `map→odom` 只保留 x/y/yaw，z/roll/pitch 置零；内部估计仍是 6-DoF。实车 profile 打开，仿真保持 false |

旧 bundle 已备份为：

```text
maps/pcd/map_bundles/219_1121_prelevel_20261124/
maps/pcd/map_bundles/227_1113pro_prelevel_20261124/
```

## 3. 验证

- 两个 leveled PCD 复拟合：残差倾斜 0.044° / 0.035°，地面 `z(0,0)≈0`。
- backend 用实车 profile + 新 bundle 启动：
  `loaded immutable prior: keyframes=48/108, descriptor=Scan Context`，无 FATAL。
- 摆正后第一次实车 `tf2_echo map odom`：`z=-0.006 m`（原 0.063 m，已修复），
  但 `roll/pitch=1.446°/2.384°` —— 这部分是 **odom（出生时 gimbal_yaw）自身的倾角**，
  不是先验地图的；因此加 `output/planar_map_to_odom`，在 TF 输出端只保留 x/y/yaw。
- `backend_localization_test --gtest_filter=PoseOutput.*`：3/3 通过（新增 4-DoF 投影单测）。
- 2026-11-24 13:41 实车复测：`tf2_echo map odom` =
  `Translation [0.485, -0.161, 0.000]`、`RPY [0.000, 0.000, 23.522°]`。
  z/roll/pitch 已归零，xy/yaw 与定位起点一致；4-DoF 投影生效，
  global/local costmap 共面。

## 4. 回退

- 地图：把 `_prelevel_20261124` 目录换回同名 bundle；`scan_context.lidar_height` 改回
  `0.3464`；两处 PCD 原始件未动。
- 配置：`git checkout --` 本记录涉及的两个 YAML 和 `build_zloc_map_bundle.sh`。
