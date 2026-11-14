# scripts

现场脚本按用途分组：

| 目录 | 内容 |
|---|---|
| `bringup/` | `start_real_nav.sh`、`start_sim_nav.sh`、`start_real_slam.sh`、`start_sim_slam.sh`、`start_waypoints.sh`、`kill_*.sh` |
| `mapping/` | 录包、离线重建 PCD、点云转地图系、查看 PCD |
| `diagnostics/` | `check_*`、`capture_minco_run.py`、`analyze_minco_bag.py`、MINCO 净空/卡死诊断 |
| `manual/` | 键盘遥控、自转控制、自转正弦、脱困注入、航点任务 |
| `test/` | 需要人工接串口/底盘的测试 |
| `lib/` | 公共 shell 函数（`srm_lib_path_guard.sh`） |

## 兼容路径

- 仓库根目录 `script` 是指向 `scripts/` 的符号链接；
- `scripts/<name>` 保留每个公开脚本的兼容符号链接，指向对应子目录。

因此以下三种写法等价：

```bash
./script/start_real_nav.sh
./scripts/start_real_nav.sh
./scripts/bringup/start_real_nav.sh
```

新脚本/新文档请使用 `scripts/<group>/<name>`；旧路径在确认现场流程全部切换前不要删除。

## 运行数据

脚本默认把运行数据放到 `~/srm_nav_27_data/`（可用 `SRM27_DATA_DIR` 覆盖）：

- `bags/`：建图录包
- `diagnostics/`：抓包、MINCO 复盘、姿态 CSV、日志
- `generated_pcd/point_lio/`：Point-LIO 累积点云
- `experiments/`、`plots/`、`archive/`：实验产物与历史数据

源码树内不再新增 `log_diag/`、`user_test_log/`、`point_lio/PCD/*.pcd`。
