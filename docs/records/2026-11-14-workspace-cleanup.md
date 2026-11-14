# 2026-11-14 工作区清理记录

> 状态：已完成
> 范围：`/home/srm/srm_nav_27` 工作区结构、运行数据、脚本、launch/config、文档、Git 忽略规则
> 未改动：ROS 包物理层级（`src/srm27_navigation` 不拆）、核心算法与正式测试

## 1. 为什么做

清理前，功能代码、兼容链路、一次性脚本、实验数据、历史文档混在同一层：

- `log_diag/` 约 23 GB，全部是建图录包和建图日志；
- `src/srm27_navigation/point_lio/PCD/` 约 890 MB 运行生成点云；
- `user_test_log/` 约 99 MB 实验 CSV/PNG；
- `.mapcheck/`、`.tmp_diag/`、`.tmp_roslog/` 中有已提交的临时脚本和结果；
- `script/` 28 个脚本平铺；`docs/` 31 篇文档没有状态区分；
- `srm27_nav_bringup` 的 16 个 launch 平铺，实车/仿真/旧入口混在一起。

## 2. 运行数据迁出（工作区外）

数据根目录：`~/srm_nav_27_data/`，可用 `SRM27_DATA_DIR` 覆盖。

| 原位置 | 新位置 | 体积 |
|---|---|---|
| `log_diag/mapping_bag_*`（9 份） | `~/srm_nav_27_data/bags/`（后按用户确认删除） | 约 23 GB，已释放 |
| `log_diag/*.log` | `~/srm_nav_27_data/diagnostics/` | 约 148 KB |
| `user_test_log/` | `~/srm_nav_27_data/experiments/user_test_log_20261113/` | 约 99 MB |
| `src/.../point_lio/PCD/*.pcd` | `~/srm_nav_27_data/generated_pcd/point_lio_PCD_20261028/` | 迁出约 890 MB；删 6 份 `scans.prev_*.pcd` 备份后保留 86 MB |
| `.sim_run/` | `~/srm_nav_27_data/archive/sim_run_20261114/` | 12 MB |
| `.rm_scratch/` | `~/srm_nav_27_data/archive/rm_scratch_20261114/` | 14 MB |
| `.tmp_roslog/` | `~/srm_nav_27_data/archive/tmp_roslog_20261114/` | 188 KB |
| `.mapcheck/` | `~/srm_nav_27_data/archive/mapcheck_20261114/` | 372 KB |
| `.tmp_diag/` | `~/srm_nav_27_data/archive/tmp_diag_20261114/` | 300 KB |
| `build.log` | `~/srm_nav_27_data/diagnostics/` | 54 KB |

处理原则：

- 9 份建图录包先迁出，随后按用户确认删除，释放约 23 GB；删除清单见 `~/srm_nav_27_data/bags_DELETED_20261114.txt`。
- 实验 CSV/PNG 只**迁出**，不在本轮删除；后续确认可弃的重复实验再单独清理。
- Point-LIO 的 `scans.prev_*.pcd` 自动备份已删除，只保留最新完整 `scans_*.pcd`（约 86 MB）。
- 临时验证脚本/截图/日志在迁出后从源码树删除；可复用的部分已迁回正式目录（见 §3）。
- `log/`、`.pytest_cache/`、`__pycache__/` 直接删除（可再生成）。
- `build/`、`install/` 保留，避免本轮清理后无法立即构建/运行。

## 3. 脚本重整

`script/` → `scripts/{bringup,mapping,diagnostics,manual,test,lib}/`。

- 根目录 `script` 改为指向 `scripts/` 的符号链接；
- `scripts/<name>` 为每个公开脚本保留兼容符号链接，指向子目录；
- 三种写法等价：`./script/start_real_nav.sh`、`./scripts/start_real_nav.sh`、
  `./scripts/bringup/start_real_nav.sh`；
- 脚本内 `SCRIPT_DIR` 改为 `readlink -f` 解析，`WS_DIR` 改为两级上级，
  保证从旧路径/新路径/兼容链接执行都能找到工作区；
- `scripts/bringup/start_real_nav.sh`、`start_sim_nav.sh` 等对
  `lib/`、`manual/`、`mapping/`、`test/` 的引用已改为 `../` 相对路径。

可复用诊断脚本从 `.tmp_diag/` 迁入 `scripts/diagnostics/`：
`plan_clearance.py`、`pose_watch.py`、`stuck_capture.py`、`stuck_probe.py`。

一次性问题定位脚本（`.mapcheck/test_tree_fix*.sh`、`start_bg.sh`）不再放进源码树，
原始件在 `~/srm_nav_27_data/archive/mapcheck_20261114/`。
`test_lib_path_guard.sh` 提升为 `tests/smoke/test_lib_path_guard.sh`，路径改为相对仓库根。

## 4. launch 与参数分组

launch 采用“canonical 根目录 + 唯一前缀别名”：

```text
srm27_nav_bringup/launch/
├── nav_srm_real_launch.py                 # canonical（旧命令）
├── real_mapping_launch.py
├── nav_srm_simulation_launch.py
├── sim_slam_launch.py
├── common/                                # 前缀别名 common_*.py
├── real/                                  # real_*.py
├── simulation/                            # sim_*.py
└── compat/                                # compat_*.py
```

为什么不在子目录放同名文件：`ros2 launch <pkg> <file>` 在 `share/launch` 下按 basename
递归搜索，同名文件出现两次会报 `found more than once`。因此分组别名统一加 `real_` /
`sim_` / `common_` / `compat_` 前缀，逻辑只在根目录一份。旧的
`ros2 launch srm27_nav_bringup nav_srm_real_launch.py` 继续可用；
新命令 `real_nav_srm_real_launch.py` 等价。

- 16 个 launch 的 current/compat/deprecated/experimental 分类、默认参数、速度出口见
  [`docs/architecture/launch_matrix.md`](../architecture/launch_matrix.md)；
- `config/{real,simulation}/{current,legacy}/` 分组，原路径保留兼容符号链接。

**未做**：不物理拆分 `src/srm27_navigation` 的 ROS 包。包路径、`dependencies.repos`、
launch 引用和构建缓存仍按原路径工作。

## 5. 文档重整

`docs/` 分为 `architecture/`、`operations/`、`design/`、`records/`、`archive/`：

- 当前架构与入口表：`architecture/`
- 操作指南：`operations/`
- 未完成方案：`design/`
- 已完成实施记录与现场日志：`records/`
- 旧 fake 底盘、TDT 调研、旧 viz：`archive/`（文首加“历史资料/不再适用/替代入口”）

文档间相对链接已按新位置批量修正；根 README 重写为简版，旧完整版归档到
`docs/records/README_full_20261114.md`。

## 6. Point-LIO 点云输出改为外部目录

- `point_lio/src/laserMapping.cpp` 新增运行期环境变量 `SRM27_PCD_DIR`：
  优先写到该目录，未设置时回退到 `ROOT_DIR/PCD`；
- `real_mapping_launch.py` 默认 `pcd_save_path` 改为
  `~/srm_nav_27_data/generated_pcd/point_lio/scans.pcd`；
- `start_real_slam.sh`、`start_sim_slam.sh`、`build_pcd_from_bag.sh` 统一设置
  `SRM27_PCD_DIR` 并创建目录；
- 源码包 `point_lio/PCD/` 只保留 `readme.md` 说明。

## 7. Git 清理

- `.mapcheck/`、`.tmp_diag/`、`.tmp_roslog/`、`user_test_log/` 中已跟踪的临时文件
  从 Git 移除（工作区文件已迁出或删除）；
- `.gitignore` 增加运行数据/temp 目录规则；
- 兼容符号链接（`script`、`scripts/<name>`、`config/<area>/<name>`）与 launch 前缀别名
  需要随本轮变更一起提交，否则旧路径/新分组入口会失效。

## 8. 验证

```bash
# 脚本语法
for f in scripts/*/*.sh tests/smoke/*.sh; do bash -n "$f"; done
python3 -m py_compile scripts/*/*.py debug_tools/*.py

# 冒烟
bash tests/smoke/test_lib_path_guard.sh

# 构建 + 包内回归
colcon build --packages-select srm27_nav_bringup --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select srm27_nav_bringup
```

实际结果（2026-11-14 本轮）：

- `srm27_nav_bringup`：7/7 通过（含 launch wiring、参数、TF 契约、速度限幅）；
- `srm27_nav_protocol`：2/2 通过；
- `point_lio` 重新编译通过（`SRM27_PCD_DIR` 改动）；
- 旧/新 launch 名 `--show-args` 均通过：`nav_srm_real_launch.py` 与
  `real_nav_srm_real_launch.py`、`nav_srm_simulation_launch.py` 与
  `sim_nav_srm_simulation_launch.py`、`sim_slam_launch.py` 与
  `sim_slam_entry_launch.py`；
- `tests/smoke/test_lib_path_guard.sh` 全部通过；
- 脚本 `bash -n`、Python `py_compile` 全部通过；
- 文档本地链接 0 broken。

若后续改动 launch/参数，重新跑上面两条。

## 9. 未完成 / 下一轮

1. 清点 `build/`、`install/`、`log/` 的长期保留策略（当前保留 build/install）。
2. ~~对 `~/srm_nav_27_data/bags/` 做去重/保留策略~~（已按用户确认删除全部 9 份，2026-11-14）。
3. 确认现场脚本全部改用 `scripts/<group>/` 后，删除根目录兼容符号链接，并同步更新
   `test_cli_parity.py` 等路径常量。
4. 评估是否把 `src/srm27_navigation` 拆成 `src/navigation/` 等物理层级（本轮明确不做）。
5. `docs/archive/` 中仍有一些 AI 生成的长文，后续按“是否还有引用价值”逐篇决定保留/删除。

## 10. 回滚

- 源码/Git：`git checkout -- <path>` 或按本轮提交 `git revert`；
- 数据：从 `~/srm_nav_27_data/` 按上表反向 `mv` 回原路径；已删除的 9 份 bag 无法回滚，清单见数据根目录 `bags_DELETED_20261114.txt`；
- 脚本：删除 `script`/`scripts` 兼容链接前，canonical 文件仍在 `scripts/<group>/`；
- launch：根目录 canonical 不动；删除 `common|real|simulation|compat` 前缀别名不影响旧命令。
- config：删除根目录兼容链接前，canonical 文件仍在 `current|legacy` 分组目录。
