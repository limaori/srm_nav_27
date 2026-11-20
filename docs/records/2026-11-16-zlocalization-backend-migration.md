# 2026-11-16 Z-Localization backend 迁入与接线

> 状态：代码迁入与项目接线完成；验收 A（构建/安装/测试）与 B（地图工具与地图坐标）已跑通，
> 验收 C–F（录包契约、算法回放与 reset 故障注入、仿真、实车）**未做**。
> backend 模式默认关闭，默认定位行为不变。
> 依据：[Z-Localization 整体迁移实施方案](../design/Z-Localization整体迁移实施方案.md)；
> 契约：[backend 接入契约](../architecture/localization_backend.md)。

## 1. 做了什么

一次性完成"码迁入 + 项目接线"（方案 §3），再按验收条目推进：

| # | 内容 | 位置 |
|---|---|---|
| 1 | 上游 `src/backend` 全量迁入并改名 `srm27_zlocalization_backend` | `src/localization/backend/srm27_zlocalization_backend/` |
| 2 | 6 个地图/标定工具与完整测试一并迁入，可执行统一前缀 | 同上 `tools/`、`test/` |
| 3 | 构建资源包内自持：`cmake/Warnings.cmake`、`config/backend.yaml`、`LICENSE`（GPL-2.0-only） | 同上 |
| 4 | backend 专用输入适配（位姿/协方差/点云/时间/QoS/session/reset 边界） | `srm27_localization_adapter/scripts/zlocalization_backend_adapter.py` |
| 5 | reset / epoch 唯一所有权：适配层代理 reset 并把边界改写为**恒等** | 同上 + `zlio_bridge.py` |
| 6 | `localization_launch.py` 接入实验模式、互斥冲突检查、默认关闭 | `src/bringup/srm27_nav_bringup/launch/` |
| 7 | 车体位姿仍由 `sensor_scan_generation` 产生；backend 车体位姿输出视为诊断 | 配置关闭 legacy / `map→body` |
| 8 | 只发布需要的输出：legacy、`world_cloud`、UWB 默认关；路径必须绝对 | `config/backend.yaml` |
| 9 | GTSAM 4.2.x 版本锁定、依赖声明、用户级安装脚本与 ABI 记录 | `scripts/setup/install_gtsam_4_2.sh`、本文件 §5 |
| 10 | 项目侧测试：接线/冲突、适配数学、frame/time/QoS、reset 边界、配置契约 | 见 §4 |

迁入时只改包名、构建/安装路径与接线；算法文件与上游逐行一致（`backend_core` 等**内部
target 名**也加了包前缀，C++ namespace 仍是 `backend`）。

### 1.1 与上游的差异清单

- 包名 / CMake project / 安装资源：`backend` → `srm27_zlocalization_backend`。
- 可执行：`backend_node` → `srm27_zlocalization_backend_node`；6 个工具统一加前缀。
- `include(../../cmake/Warnings.cmake)` → 包内 `cmake/Warnings.cmake`。
- `install(FILES ../../config/backend.yaml)` / `../../LICENSE` → 包内 `config/`、`LICENSE`。
- `backend.launch.py` 增加 `sensor_frame` / `odom_frame` / `trajectory_directory` 三个覆盖参数，
  并对 `prior_map`、`trajectory.save_directory` 强制绝对路径（相对路径直接报错）。
- `config/backend.yaml`：话题改相对名；先验地图/轨迹目录留空；UWB、legacy、`world_cloud`
  默认关闭；`reset.service` 指向适配层；`services.reset` 改相对名。
- 新增平台 profile：`srm27_localization_real.yaml`（`livox_imu`）与
  `srm27_localization_sim.yaml`（`front_mid360`），只覆盖 `sensor_frame`。
- 测试：可执行名加前缀（`PREFIX` 常量）、launch 测试断言新可执行名。

## 2. reset / epoch 所有权（方案 §3.2 第 5 条）

决策：**连续性补偿的唯一所有者是本仓库 `zlio_bridge` 的 `T_off` 锚定**；
backend 只按"恒等边界"接手，不再做第二次坐标补偿。

- 前端：backend 模式 `coordinated_reset:=true`（不自己换代，等后端协调）。
- 适配层：作为 reset 服务代理（`zlocalization/reset_frontend` → `/zlio/reset_internal`），
  把前端回复的 `old_from_new` **改写成恒等**，`source_epoch` / `boundary_stamp` 透传；
  冷重置没有边界时用适配层最后一帧的代次/时间补恒等边界；一帧都没发过则不冒充边界。
- 为什么是恒等：bridge 已经让适配后 odom 在换代接缝处连续，新旧坐标关系**就是**恒等；
  backend 收到恒等边界后 `reframeOdometry` 是 no-op，同时走"边界有效"分支
  （比"假设停车"更强，移动中 reset 不会累计误差）。

细节与边界情况见 [backend 接入契约](../architecture/localization_backend.md) §4。

## 3. 为什么适配层必须长在 `zlio_bridge` 里

backend 按 `map_from_sensor · (T_odom_sensor)⁻¹` 求 `map→odom`，`T_odom_sensor` 必须
逐位等于 bridge 发布的 `lidar_odometry`。若另起节点自己算 `T_off`，它抓到的"第一帧"
可能与 bridge 不同（订阅晚几帧、TF 精确时刻查询退化到最新值），两个 odom 系差一个常量
刚体变换，整条 TF 链稳定地错而 `localization_state` 仍是 `NORMAL`。
因此 `BackendInputAdapter` 由 `zlio_bridge` 持有，`backend_enabled:=true` 时启用。

## 4. 验证证据

### 4.1 干净构建（新增的独立 build/install 目录）

环境：GTSAM 由 `scripts/setup/install_gtsam_4_2.sh` 装到工作区前缀（无 root），
`MAKEFLAGS=-j4`、`--parallel-workers 1`（逐包串行；见 §7 的机器限制）。

```bash
GTSAM_PREFIX=$PWD/.deps/gtsam-4.2.0 ./scripts/setup/install_gtsam_4_2.sh
source .deps/gtsam-4.2.0/env.sh
colcon build --build-base build_zloc --install-base install_zloc \
  --parallel-workers 1 \
  --packages-select srm27_zlocalization_backend srm27_localization_adapter srm27_nav_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

结果：3 个包 3 min 04 s 全部构建成功；96 个 C++ 编译单元 + 12 个 gtest 可执行；
构建期间最低 `MemAvailable` 5.35 GiB（15.5 GiB 机器）。

产物：`install_zloc/srm27_zlocalization_backend/lib/srm27_zlocalization_backend/`
下的 7 个可执行（节点 + 6 个工具）、`share/.../config/{backend,srm27_localization_real,
srm27_localization_sim}.yaml`、`launch/`、`rviz/`、`docs/`、`README.md`、`LICENSE`。

> 说明：工作区其余包本轮没有重编（它们的 `build/` 缓存指向 2026-11-15 重组前的目录，
> 见 §7），因此这是一次"**受影响包的干净构建**"，不是 33 包全量重建。

### 4.2 测试

| 包 | 测试数 | 结果 |
|---|---:|---|
| `srm27_zlocalization_backend` | 184 | 0 errors / 0 failures |
| `srm27_localization_adapter` | 4 | 0 errors / 0 failures |
| `srm27_nav_bringup` | 14 | 0 errors / 0 failures |

backend 的 184 项 = 上游 12 个 gtest（融合/生命周期/图/创新/UWB/重定位/定位/描述子/
标定/存储/清理/测量）+ 2 个 pytest（CLI 工具、launch 配置）+ 2 个 launch_test
（标定 launch、地图服务），全部在干净构建目录下通过。

新增的项目侧测试：

- `srm27_localization_adapter/test/test_zlocalization_backend_adapter.py`（14 项）：
  协方差旋转与参考点平移、`frame/time/session` 契约、reset 边界改写为恒等、
  冷重置/无帧兜底、配置契约（legacy/world_cloud/UWB 关闭、相对话题名、绝对路径要求）。
- `srm27_nav_bringup/test/test_localization_backend_wiring.py`（8 项）：
  默认关闭且静态 `map→odom` 不变；backend 模式关静态 TF、include 参数正确；
  `sensor_frame` 跟随 `zlio_body_frame`；GICP/point_lio/缺地图/相对路径/缺配置
  五种冲突在启动前失败；缺包时默认链路仍可启动；`nav2_stack` 与 `slam` 互斥；
  两个正式入口透传参数且默认 `False`。

### 4.3 运行期（干净安装前缀）

```bash
ros2 launch srm27_zlocalization_backend backend.launch.py \
  config_file:=$SHARE/config/srm27_localization_sim.yaml mode:=localization \
  prior_map:=<bundle> sensor_frame:=front_mid360
```

- 日志：`loaded validated map: keyframes=108`、`unified backend ready: prior_map=enabled,
  loop=disabled, uwb=disabled, frontend=zlio, atomic_frame=frontend/frame`。
- `~/localization_state` 输出 `status=INITIALIZING;pose_trusted=false;epoch=0`（等待首帧），
  说明状态机、参数 schema 与话题链路都成立。
- `zlio_bridge` 打开 `backend_enabled:=true` 后：节点 `/zlio_bridge`、
  服务 `/zlocalization/reset_frontend`、话题 `/frontend/{frame,odometry,session}` 均建立，
  日志打印"backend 输入适配已启用"与"backend 模式已启用"。
- 运行不依赖上游工作区：`grep -r /home/srm/z-localization` 在包与安装前缀中无命中
  （README 只保留上游提交号）。

### 4.4 地图工具与地图坐标（验收 B）

从正式地图 `maps/pcd/227_1113pro_mapframe.pcd`（PGM 地图系，非 `camera_init`）：

```bash
srm27_zlocalization_backend_build_pcd_map_bundle \
  --input maps/pcd/227_1113pro_mapframe.pcd --out-bundle /abs/227_1113pro_bundle \
  --x-min -8 --x-max 4.5 --y-min -7.6 --y-max 10.1 \
  --sensor-z 0.3464 --query-step 2.0 --bundle-step 1.5 \
  --map-voxel 0.15 --scan-voxel 0.15 --sc-lidar-height 0.3464
```

| 检查 | 结果 |
|---|---|
| 输入点云 | 251 165 点，x[-8.06, 4.61] y[-7.74, 10.23] z[-0.92, 3.97] |
| 生成 keyframes | 108（`keyframes.txt` + `clouds/*.pcd` + `sc_descriptors.bin`，23 MB） |
| 重建描述子 | `rebuild_scan_context_descriptors`：108 条，重载校验通过 |
| 导出参考图 | `filter_map_bundle --export-reference-map`：1 200 340 点 |
| 坐标变换 | `transform_map_bundle --translation 0 0 0 --quaternion 0 0 0 1` 成功 |
| 清理工具 | `clean_map_bundle` 对生成 bundle 不报错但 0 点被清理（无可见性记录）→ 需要清理必须用建图期 runtime bundle |

雷达离地高度取 URDF 的 `0.346388 m`（`srm27_sentry.urdf.xacro` 的 `lidar_xyz`），
与 `--sensor-z` / `--sc-lidar-height` 一致。

## 5. 依赖与部署记录（验收 A 的"部署复现"）

| 项 | 值 |
|---|---|
| GTSAM 来源 | ROS 2 Humble 官方 deb `ros-humble-gtsam 4.2.0-3jammy.20260226.005743`（amd64） |
| 安装方式 | `dpkg-deb -x` 解到工作区前缀 `.deps/gtsam-4.2.0`（不写 dpkg 数据库，删目录即回退；`~/.local` 在沙箱内不可写） |
| 构建期 | `GTSAM_DIR=<prefix>/lib/cmake/GTSAM`（版本 4.2.0，CMake 拒绝 ≥4.3） |
| 运行期 | `libgtsam.so.4` + `libmetis-gtsam.so` 由可执行文件 RPATH 解析（`--disable-new-dtags` + `BUILD/INSTALL_RPATH=$<TARGET_FILE_DIR:gtsam>`）；`ldd` 在**不带** `LD_LIBRARY_PATH` 的情况下能解析两者 |
| 其它依赖 | PCL 1.12、Eigen 3.4、nanoflann 1.4、oneTBB 2021.5、OpenMP、Boost 1.74（系统已有） |
| 许可证 | 包内 `LICENSE` = 上游 GPL-2.0-only 原文；发布/分发责任见 §6 |

## 6. 许可证

迁入代码整体 **GPL-2.0-only**。它默认关闭、不参与产品默认链路，但一旦纳入正式入口，
仓库发布、产品分发与维护责任由本队承担；对外分发前需要先过合规评审。

## 7. 环境限制与本轮踩到的坑（重要）

1. **机器内存**：15.5 GiB + 2 GiB swap。本轮两次"全工作区并行干净构建"
   （16 并发、后一次 8 并发）把机器压到死机（`last` 显示 21:0x 与 22:09 两次 crash）。
   之后改为"逐包串行 + 单包 -j4"，最低 `MemAvailable` 5.3 GiB，未再出问题。
   → 本机不要再做全量并行构建。
2. **`build/` 缓存不完整**：13 个包的 `CMakeCache.txt` 仍指向 2026-11-15 模块重组前的
   `src/perception_localization/...`、`src/trajectory_planning/...`，直接重建会报
   "source does not match the source used to generate cache"。另有 6 个包（含
   `srm27_minco_core`）当时没装完整，且 `srm27_nav_behavior_plugins` 缓存里
   `srm27_minco_core_DIR-NOTFOUND`。本轮逐个删 `build/<pkg>` 后重建补齐，
   现在 34 个包都有完整 `install/`。
3. **用户级 pytest 9 与 `launch_testing` 不兼容**：`colcon test` 需要
   `PYTHONNOUSERSITE=1`（退回系统 pytest 6.2），否则 pytest 插件校验直接报错。
4. **`/opt/MVS/lib/64` 的旧 libusb** 会盖住系统 `libusb-1.0.so.0`，让 PCL 的
   `libpcl_io` 报 `undefined symbol: libusb_set_option`；跑 PCL 相关测试前把它从
   `LD_LIBRARY_PATH` 去掉。
5. **沙箱内 `$HOME` 只读**：`~/.ros/log` 不可写，测试/节点需要 `ROS_HOME`/`ROS_LOG_DIR`
   指到可写目录。这是本次执行环境的限制，不是仓库问题。

## 8. 未做的验收与风险

| 验收 | 状态 | 说明 |
|---|---|---|
| A 构建/安装/测试 | ✅ | 受影响包的干净构建 + 202 项测试全通过；GTSAM ABI 有记录 |
| B 地图工具与地图坐标 | ✅ | 227_1113pro 正式地图生成/加载/重建/变换/导出均验证 |
| C 输入契约与适配数学 | 部分 | 数值测试已覆盖（协方差旋转、参考点、frame/time/QoS、reset 边界）；**固定 rosbag 逐帧对照未做** |
| D 算法回放与 reset 故障注入 | ❌ | 需要固定 bag + 先验图回放；事务幂等/冻结/恢复未实测 |
| E 仿真导航链 | ❌ | 只验证了 backend 单独起停，未接 Nav2 |
| F 实车低速与基线比较 | ❌ | 未做；指标未定义 |

已知风险（与方案 §6 一致）：3D `map→odom` 与 2D 消费侧的平面投影、
退化几何（走廊/单面墙）下的恢复行为、云台运动下的配准质量、CPU/内存与点云带宽，
都需要验收 C–F 才能下结论。

## 9. 现场操作与回退

```bash
# 启用（实验）
./scripts/bringup/start_real_nav.sh --lio --lio-frontend zlio \
  --zloc-backend --zloc-prior --zloc-map /abs/path/to/bundle
./scripts/bringup/start_sim_nav.sh --lio --zloc-backend --zloc-prior --zloc-map /abs/path/to/bundle
# 注：当时 `--zloc-map` 隐含开启先验；2026-11-19 起需显式 `--zloc-prior`，
#     回环/UWB 分别用 `--zloc-loop` / `--zloc-uwb`。

# 回退：去掉 --zloc-backend 即回到 --lio + 静态 map->odom
# 或改用 GICP：--reloc（需要先验 PCD）
```

冲突检查三层（脚本 → `nav2_stack_launch.py` → `localization_launch.py`）保证"两个
`map→odom` 所有者"在启动前就失败；`srm27_zlocalization_backend` 未构建时默认链路照常启动。
