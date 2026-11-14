# 基准工具

两个口径分开测，避免再出现"拿墙钟比纯函数"的错误。

## 1. 纯算法（不含 ROS）

对应上游 README 的「Astar 平均/最大」那一栏，可直接对比。

```bash
g++ -O3 -std=c++17 bench_yastar.cpp ../vendor/src/YAstar/yastar.cpp \
    -I/usr/include/eigen3 -I../vendor/src/YAstar -o bench
./bench /path/to/map.pgm 200
```

输出：

```
地图 583x324（188892 格）
initCostMap（代价图）: 6.74 ms
search x200：平均 11.00 ms，最大 18.79 ms，末次路径 173 点
```

注意 PGM 头里可能有 `#` 注释行，脚本已处理。

## 2. ROS 侧（含动作传输）

```bash
python3 bench_planner.py -n 50 --start 3.4 5.0 --goal 12 3
```

输出：

```
规划 x50：平均 35.6 ms，中位 31.6 ms，p95 62.8 ms，最大 78.4 ms，路径点数 10~10
```

> 这个数字包含 action 请求/响应、TF、节点调度，**不等于算法耗时**。
> 想知道算法占多少，用 `debug_timing: true` 看插件日志里的分段计时。

## 参考值（rmuc_2025，583×324，起点 (3.4,5.0) → 终点 (12,3)）

| 口径 | 平均 | 最大 | 说明 |
| --- | --- | --- | --- |
| 纯 YAstar search | 11.00 ms | 18.79 ms | 200 次，离线 |
| initCostMap | 6.74 ms | — | 纯函数，每次规划都要重算 |
| 插件内合计 | 23~32 ms | — | `debug_timing` 日志 |
| ROS 动作墙钟 | 35.6 ms | 78.4 ms | 50 次，含传输 |

上游 README 在 RMUC2025 地图上的参考值是 Astar 平均 5.45 ms / 最大 14.76 ms，
代价图 1.59 ms。差异主要来自查询本身（本条是 37 m 跨图长路径）。
