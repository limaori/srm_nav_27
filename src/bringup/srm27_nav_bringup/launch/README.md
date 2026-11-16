# srm27_nav_bringup/launch

## 目录约定

- 根目录的 `<name>_launch.py` 是 **canonical**，也是旧命令继续使用的入口。
- `common/`、`real/`、`simulation/`、`compat/` 放**唯一前缀名**的分组别名，
  例如 `real/real_nav_srm_real_launch.py`、`simulation/sim_slam_entry_launch.py`。

为什么不在分组目录里放同名文件？ROS 2 的 `ros2 launch <pkg> <file>` 会在
`share/launch` 下按 basename 递归搜索；同一 basename 出现两次会直接报
`file ... was found more than once`。因此分组入口必须换一个唯一名字，逻辑仍只有
根目录一份。

## 入口

| 用途 | 旧命令（canonical） | 分组别名 |
|---|---|---|
| 实车导航 | `nav_srm_real_launch.py` | `real_nav_srm_real_launch.py` |
| 仿真导航 | `nav_srm_simulation_launch.py` | `sim_nav_srm_simulation_launch.py` |
| 实车建图 | `real_mapping_launch.py` | `real_mapping_entry_launch.py` |
| 仿真建图 | `sim_slam_launch.py` | `sim_slam_entry_launch.py` |

```bash
ros2 launch srm27_nav_bringup nav_srm_real_launch.py
ros2 launch srm27_nav_bringup real_nav_srm_real_launch.py
```

两条命令等价。完整状态、默认参数、速度链与 fake_vel_transform 职责见
[`docs/architecture/launch_matrix.md`](../../../docs/architecture/launch_matrix.md)。
