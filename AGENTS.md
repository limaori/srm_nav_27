# 项目注意事项（每次会话自动加载）

> 本文件是 `AGENTS.md`，agent 每次会话开始时自动读取。**第 1 节是硬性规则**，
> 违反会导致整机假死/重启，优先级高于任何"想快点验证"的理由。
> 详细事故记录与原理见 [编译注意事项.md](编译注意事项.md)。

## 1. 编译并发：必须限制（最高优先级）

本机：16 线程（i7-1260P）/ 15 GiB 内存 / 2 GiB swap。
`colcon build --parallel-workers N` **只限制同时构建几个包**，每个包内部仍是
`make -j$(nproc)`＝16 个 g++。两个内存大户一旦并行，30+ 编译进程会把内存+swap
打满，系统假死、只能重启（2026-11-24 已复现一次全栈构建跑约 25 分钟后整机重启）。

必须遵守：

- **只用**受限并发入口构建，默认 `JOBS=2 PKG_WORKERS=1`：
  ```bash
  ./scripts/build/build_safe.sh --packages-select srm27_map_core
  JOBS=3 PKG_WORKERS=1 ./scripts/build/build_safe.sh --packages-select srm27_minco_core
  ```
- **不要裸跑 `colcon build`**。手工构建必须同时卡住两层（`MAKEFLAGS` 是关键）：
  ```bash
  export MAKEFLAGS="-j2"
  colcon build --parallel-workers 1 --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_BUILD_PARALLEL_LEVEL=2
  ```
- 禁止 `make -j$(nproc)`、`-j8`、`-j16` 及任何不设上限的并行；单包并发最高 `-j3`。
- 内存大户 `frontend`(ZLIO)、`srm27_zlocalization_backend`(GTSAM)、
  `srm27_gazebo_simulator` **永远单独构建**（`JOBS=2 PKG_WORKERS=1`），不与任何包并行。
- 不要为了验证小改动跑 `--packages-up-to` 全栈：只构建改动包及其直接下游。
- 构建前先确认无残留进程，构建中盯内存：
  ```bash
  ps aux | grep -E "cc1plus|make|colcon" | grep -v grep   # 有残留先处理
  free -h                                                  # available < 1 GiB 立即停
  ```

## 2. 运行期也不要堆进程

- 同一时间只跑一套仿真/导航实例；重启节点前先 `pkill` 或 `ros2 node list` 确认旧进程退出，别让 gazebo/rviz 叠几层。
- 后台任务（含 subagent、`&` 作业）不要同时挂多个重编译/重构建，串行执行。
- 日志读增量不读全量：`log/`、`.tmp_roslog/` 已有数百 MB，用 `tail -n 200`、`grep`，不要 `cat` 整个文件或 `tail -f` 刷屏。
- 不做全盘扫描（`find /`、`du /`、`grep -r /`），一律限定在仓库或用例目录内。

## 3. 拿不准时的做法

- 先单包小规模验证（`--packages-select <pkg>`），确认没问题再扩大范围。
- 任何"一次性并行跑完"的诱惑都先按第 1 节降并发；慢一点没关系，死机要重来。
