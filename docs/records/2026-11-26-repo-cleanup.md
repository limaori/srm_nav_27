# 2026-11-26 仓库清理记录：一次性诊断目录、零引用样例图与 .gitignore 整理

> 状态：已完成
> 范围：工作区内的一次性诊断/实验目录、零引用样例图片、`.gitignore` 规则结构
> 未改动：`src/` 包结构与代码、`maps/` 已版本化地图、现场实验数据、`docs/` 正文结论
> 承接：[`2026-11-14-workspace-cleanup.md`](2026-11-14-workspace-cleanup.md)（数据迁出、脚本/launch/docs 重整）

## 1. 为什么做

最近几次 `wip:` 提交用 `git add -A` 把调试期的临时目录一起带进了版本库：

| 目录 | 已跟踪文件 | 工作区占用 | 内容 |
|---|---|---|---|
| `.diag_tmp/` | 46 | 159 M | 探针脚本、截图、npz、CSV、当天 A/B 实验数据 |
| `.dsh_run/` | 146 | 9.5 M | repro 脚本、日志，含整套 `sg_probe/` CMake 构建树（`.o` 2.4 M、`.so` 1.8 M） |
| `.diag2/` | 3 | 2.0 M | 旧截图 |
| `.revert_scratch/` | 1 | 96 K | 备份 yaml |

`.gitignore` 里没有这四个目录的规则，于是每次 `git status` 都混着几百个探针脚本、截图、编译器产物，
真正要提交的改动反而看不清。同一时期 `debug_tools/example_gimbal_spin.png` 也成了全仓库零引用的孤儿图。

清理原则沿用 2026-11-14：**结论进 `docs/records/`，数据进 `~/srm_nav_27_data/`，源码树只留可复现的东西。**

## 2. 删除清单

| 路径 | 已跟踪 | 释放 | 说明 |
|---|---|---|---|
| `.dsh_run/` | 146 | 9.5 M | 整目录删除：`repro*.sh`、`clean_e2e*.sh`、各类 `*.log`、`bench/`，以及误提交的 `sg_probe/` CMake 构建树 |
| `.diag2/` | 3 | 2.0 M | 整目录删除：旧截图 |
| `.revert_scratch/` | 1 | 96 K | 整目录删除：`fy_610.yaml` 备份 |
| `.diag_tmp/` | 46 | 97 M | 删旧留新，见 §3。其中 `capture_corner.py`、`map_clearance_report.py` 仍在流程里用，**只出索引，文件留在工作区** |
| `debug_tools/example_gimbal_spin.png` | 1 | 0.14 M | 零引用（README 未引用） |

结果：索引记录移除 197 条（1442 → 1246，另新增本文），磁盘释放约 108 M（`.diag_tmp` 159 M → 62 M）。

## 3. `.diag_tmp/` 删旧留新的判据

判据：**只保留 2026-11-26 当天产出、且当前 A/B 实验还在用的件**。

保留：

- `ab2/`（A/B 全量结果、5 个变体 yaml、`ab_table.md`、`ab_corner.png`、bags 39 M）；
- `ab_shrink0/`、`ab_shrink40/`、`bagana/`、`navcap_{min,turn,rm2,ab,dense}/`、`rmuc2025/`；
- 当天脚本 `capture_corner.py`、`analyze_nav_bag.py`、`plot_bag_layers.py`、`run_one.sh`；
- `realmatch*.yaml` 变体、`clean*_A/B.log`、`shot*.png`、`mapview/` 当天图；
- `roslog_probe/` 留**空目录**（`run_one.sh` 把 `ROS_LOG_DIR` 指到这里）。

删除：

- 11-25 及以前的探针：`probe{2..7}.sh`、`probe_slam.sh`、`replay_*`、`check_map_vs_pcd.py`、
  `compare_pgm.py`、`grid_from_pcd.py`、`map_clearance_report.py`、`verify_map_vs_pcd.py`、
  `sim_global_plan.py`、`width_probe.py`、`custommsg_to_pointcloud2.py`；
- 旧截图/npz/中间结果：`bag202to227_*.png`、`mapview/*`（11-24/25）、`passes.npz`、
  `dense_align.npz`、`tf_202to227.npz`、`plans_{1..6}.txt`、`yastar_path.txt`、
  `nav2_params_yastar.before_stuckfix.yaml`、`navcap/`、`navcap_rm/`、`sim227to219/`；
- 当天可再生成物：`tb/`（colcon build 树 58 M）、`ti/`（install 前缀 11 M）、
  `mplconfig/`（matplotlib 字体缓存）、`__pycache__/`、`*.pid`。

两点补充：

- 删掉的 `custommsg_to_pointcloud2.py`、`verify_map_vs_pcd.py` 其实**已转正**到
  `scripts/mapping/`（`rebuild_map_from_bag.sh`、`prepare_prior_map.sh` 引用的是正式副本），
  `.diag_tmp` 里只是早期原型，删掉不影响脚本链路。
- 清理时仍有分析进程在 `import map_clearance_report`（`analyze_nav_bag.py`、`ab2/summarize.py`
  需要 `load_pgm`/`write_png`）。该文件在工作区被重建成最小版并继续使用，但**只留本地、不入库**，
  与 `capture_corner.py` 同样是“出索引、留文件”。

## 4. `.gitignore` 整理

原来是按时间追加的段落，存在重复规则；改为按用途分栏：
构建产物 / Python 缓存 / 第三方依赖 / 一次性诊断目录 / 运行数据与录包 / 现场产物 / 编辑器 / 地图与点云。

新增：

| 规则 | 原因 |
|---|---|
| `/.diag_tmp/`、`/.diag2/`、`/.dsh_run/`、`/.revert_scratch/` | 本轮清理的四个临时目录，防止再次 `git add -A` 误提交 |
| `*.pid`、`*.npz` | 诊断脚本的进程号/中间数组，无版本化价值 |
| `/compile_commands.json` | colcon 在包根生成的编译数据库软链接 |
| `*.swp`、`*.swo`、`.DS_Store`、`*.egg-info/`、`/.venv/`、`/venv/` | 编辑器/工具残留 |

去掉的重复规则：`/log_diag/`（两处）、`/.sim_run/`（两处）、`/.rm_scratch/`（两处）、
`/maps/pcd/map_bundles/`（两处）、`/.pytest_cache/` 与 `/**/.pytest_cache/` 合并为 `**/.pytest_cache/`、
`/data/map_bundles/`（被 `/data/` 覆盖）。

语义保持不变：`*.data` / `*.posegraph` 之后的否定规则 `!maps/**/*.data`、`!maps/**/*.posegraph`
仍在原位，`point_lio/PCD/*` 后仍保留 `!readme.md`；`/src/bringup/srm27_nav_bringup/` 下三条实车地图规则保留。

## 5. 未改动

| 项目 | 占用 | 原因 |
|---|---|---|
| `user_test_log/` | 76 M | 现场实验 CSV/PNG，不可再生成；已在忽略规则内。确有需要可自行 `rm -rf` |
| `user_log/` | 1.6 M | 同上（`plot_backend_correction.py` 等工具的默认输出） |
| `log/`、`.tmp_roslog/`、`build/`、`install/` | 约 450 M | 可再生成的运行缓存，未在 Git 内；按需 `rm -rf` 即可 |
| `maps/` | 12 G | 绝大多数是未跟踪的 `pcd/` 与 `map_bundles/`，已被规则覆盖 |
| `docs/records/2026-11-21-long-path/` 4 张图 | 0.2 M | 全仓库零引用，但属记录证据，本轮未删 |

## 6. 验证

```bash
# 索引与工作区
git status --porcelain -uall            # 只剩 196 项删除 + .gitignore + 5 个在改文件
git ls-files .diag_tmp                  # 空

# 语法
for f in $(git ls-files 'scripts/*.sh' 'tests/*.sh'); do bash -n "$f"; done
python3 -m py_compile debug_tools/*.py

# 忽略规则抽查（应全部 IGNORED）
for p in build/ install/ log/ .cache/ .deps/ .pytest_cache/ .diag_tmp/ \
         user_log/ user_test_log/ .tmp_roslog/ .idea/ build_zloc/ install_zloc/; do
  git check-ignore -q "$p" || echo "NOT ignored: $p"
done

# 否定规则仍生效（应全部 ok，不被忽略）
git ls-files maps | grep -E '\.(data|posegraph)$' | xargs -r git check-ignore -v
```

实际结果（2026-11-26 本轮）：

- `git status` 未跟踪项从 473 项降到 5 项，且都是正在改的正式文件
  （`docs/records/2026-11-26-对齐navi_minco_bit语义.md`、`maps/227to219_1125/simulation/` 三个 realmatch 脚本、
  `scripts/bringup/record_nav_bag.sh`）；
- `bash -n`、`py_compile` 全部通过；
- `git check-ignore` 13 项运行目录全部命中；`maps/**/*.data|posegraph` 10 个已跟踪文件与 `PCD/readme.md`
  均未被忽略（否定规则有效）；
- Markdown 图片链接：删除的示例图无任何引用，未新增断链。

## 7. 回滚

- 已跟踪文件（本轮删除）：`git checkout HEAD -- <path>`，或提交前 `git restore --staged --worktree <path>`；
- `.dsh_run/`、`.diag2/`、`.revert_scratch/`、`.diag_tmp/` 的旧探针都在历史提交里，可按提交取回：

  ```bash
  git show e40d885:.dsh_run/repro7.sh    > /tmp/repro7.sh      # .dsh_run
  git show 61b03a4:.diag2/zoom_robot.png > /tmp/zoom_robot.png # .diag2 / .revert_scratch
  git show 558123e:.diag_tmp/probe5.sh   > /tmp/probe5.sh      # .diag_tmp 旧探针
  git show HEAD:debug_tools/example_gimbal_spin.png > /tmp/example_gimbal_spin.png
  ```

- `.diag_tmp/` 当天保留件未入库，属于本机实验数据，删了无法从 Git 恢复，按需自行备份。
