# tests

这里放**跨包、可重复执行**的冒烟测试；ROS 包内的单元/回归测试仍在各包 `test/` 下：

- `src/srm27_navigation/srm27_minco_core/test/`
- `src/srm27_navigation/srm27_minco_controller/test/`
- `src/srm27_navigation/srm27_nav_plugins/test/`
- `src/srm27_navigation/srm27_nav_bringup/test/`（launch wiring、参数、TF 契约）
- `src/srm27_navigation/fake_vel_transform/test/`
- `src/srm27_chassis_control/test/`
- `src/srm27_nav_protocol/test/test_feedback_protocol.py`

## smoke/

| 文件 | 说明 |
|---|---|
| `smoke/test_lib_path_guard.sh` | 验证 `scripts/lib/srm_lib_path_guard.sh` 剔除海康 MVS 库路径、幂等、未设置变量等行为；不启动 ROS |

运行：

```bash
bash tests/smoke/test_lib_path_guard.sh
```

一次性问题定位脚本（例如已解决的 TF 树修复脚本）不再放进源码树；结论与命令片段
记录在 `docs/records/`，原始脚本归档在 `~/srm_nav_27_data/archive/`。
