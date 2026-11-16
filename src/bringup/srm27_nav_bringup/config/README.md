# srm27_nav_bringup/config

参数按 `current/` 与 `legacy/` 分组，原路径保留兼容符号链接：

```text
config/
├── real/
│   ├── current/     # 当前实车链路：MINCO/fakeyaw、Omni 回退、建图、限幅、ZLIO
│   ├── legacy/      # 上游基线，只给旧 nav_real_launch.py
│   └── *.yaml -> current|legacy/...   # 兼容旧路径
└── simulation/
    ├── current/
    ├── legacy/
    └── *.yaml -> current|legacy/...
```

`mid360_user_config.json` 是雷达驱动网络配置，不属于 current/legacy 参数集，保留在 `real/`。

哪个文件对应哪个入口、哪些只是 A/B 对照，见
[`docs/architecture/launch_matrix.md`](../../../docs/architecture/launch_matrix.md) §5。
