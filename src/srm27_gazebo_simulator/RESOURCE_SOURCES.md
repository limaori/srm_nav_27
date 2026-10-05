# 仿真资源来源

`resource/`、`launch/gazebo.launch.py` 和 `env-hooks/gazebo.dsv.in` 迁自
[SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator](https://github.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator)
在本工作空间中的版本，保留原有作者信息和 Apache-2.0 许可证（见 `LICENSE`）。

资源包括 RMUC/RMUL 2024、2025 场地及网格/纹理、MID-360 模型、空场和 Gazebo GUI 配置。
世界和模型文件保持迁移前内容，原有 `model://` URI 通过本包安装的资源搜索路径解析。

启动 SRM 机器人：

```bash
ros2 launch srm27_gazebo_simulator srm_sim.launch.py world:=rmuc_2025
```

只启动场地和时钟桥接（默认 RMUL 2024，不生成机器人）：

```bash
ros2 launch srm27_gazebo_simulator gazebo.launch.py
```

该场地入口保留 `world_sdf_path`、`ign_config_path` 参数，可指定绝对路径。
