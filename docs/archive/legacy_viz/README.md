# legacy_viz

从工作区根目录 `viz/` 迁入的旧可视化材料：

- `srm_sentry_new.launch.py` —— 旧 SRM 哨兵模型 + RViz 启动文件；
- `srm_sentry_new.rviz`、`cloud_only.rviz` —— 对应 RViz 配置。

这些文件不在 `srm27_nav_bringup` 的四个正式入口内，保留仅供追溯。当前导航/建图
请使用 `srm27_nav_bringup/launch/real|simulation/` 下的入口与自带 RViz 配置。
