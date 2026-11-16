# scripts

现场脚本按用途分组：

| 目录 | 内容 |
|---|---|
| `bringup/` | `start_real_nav.sh`、`start_sim_nav.sh`、`start_real_slam.sh`、`start_sim_slam.sh`、`start_waypoints.sh`、`kill_*.sh` |
| `mapping/` | 录包、离线重建 PCD、点云转地图系、查看 PCD |
| `manual/` | 键盘遥控、自转控制、自转正弦、航点任务 |
| `test/` | 需要人工接串口/底盘的测试 |
| `lib/` | 公共 shell 函数（`srm_lib_path_guard.sh`） |

> 诊断探针（`check_*`、`capture_minco_run`、`analyze_minco_bag`、`pose_watch`、
> `stuck_*`、`inject_recovery_stuck`）已按维护要求删除；需要复现时从 Git 历史取。

## 路径

旧的 `script/` 兼容链接和 `scripts/<name>` 快捷链接已删除，脚本只保留一种路径：

```bash
./scripts/bringup/start_real_nav.sh
./scripts/bringup/start_real_slam.sh
./scripts/mapping/build_pcd_from_bag.sh
```

新脚本/新文档统一使用 `scripts/<group>/<name>`。

## 运行数据

脚本默认把运行数据放到 `~/srm_nav_27_data/`（可用 `SRM27_DATA_DIR` 覆盖）：

- `bags/`：建图录包
- `diagnostics/`：建图/运行日志
- `generated_pcd/point_lio/`：Point-LIO 累积点云
- `experiments/`、`plots/`、`archive/`：实验产物与历史数据

源码树内不再新增 `log_diag/`、`user_test_log/`、`point_lio/PCD/*.pcd`。
