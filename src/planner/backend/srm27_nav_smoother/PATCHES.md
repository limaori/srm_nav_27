# 上游源码最小补丁登记 · MinimumSnap + SfcSquare

对象：本包 `vendor/` 内 `tdt-nav-kit` @ `814d5ce0237acc7faa8c070e5c5e61e2ddc81996`（tree `3274cebb...`）。
行号均为**补丁前上游原始文件**的行号。所有补丁以 `// [TDT-NAV-KIT PATCH Pn]` 注释在源码中就地标注。
目标：统一 **C++17**、消除隐式 OpenCV 依赖、修正非零原点下的坐标不一致。

> 2026-11-24：本表从 `srm27_nav_common/PATCHES.md` 按算法拆分而来。YAstar 的补丁
> （P1、P2、P4、P5、P8、P9）见相邻包 `src/planner/frontend/srm27_nav_planner/PATCHES.md`。

| ID | 文件 | 上游行号 | 改动 | 验证 |
| --- | --- | --- | --- | --- |
| P3 | `vendor/src/MinimumSnapOsqp/sfcSquare.cpp` | 3 | 删 `#include <ranges>` | 编译 |
| P6 | `vendor/src/MinimumSnapOsqp/sfcSquare.hpp` | 12-16 | OpenCV `__has_include` 守卫改为显式开关（同 P5） | 编译 |
| P7 | `vendor/src/MinimumSnapOsqp/sfcSquare.cpp` | 46-47, 145-146, 236-237, 297-298, 356-357 | 10 行 `(x + originPos)/mapping` → `(x - originPos)/mapping` | **单测验证** |
| P10 | `vendor/src/MinimumSnapOsqp/sfcSquare.cpp` | 226-231、287-292、469-474 | 3 处 `index * mapping - originPos` → `+ originPos`（索引 → 世界坐标补回原点） | **`test_sfc_bound_frame` + 实车地图实测** |
| P11 | `vendor/src/MinimumSnapOsqp/minimumSnap.cpp` | `iterate()`、`osqpExecute()` | 新增节流诊断日志（`[smooth-iter]` / `[osqp-fail]` / `[osqp-execute]`）+ `<iomanip>`/`<limits>` | 编译 + 现场日志 |

---

## P3 · 删除未使用的 C++20 头

上游 `sfcSquare.cpp:3` 引了 `<ranges>`，但全仓**没有** `std::ranges::` / `views::` 调用点。
C++17 下该头可能不存在，直接删除。

```diff
- #include <ranges>
  #include <utility>
```

## P6 · 显式关闭 OpenCV

上游 `sfcSquare.hpp:12-16` 用 `#if __has_include(<opencv2/opencv.hpp>)` 自动包含 OpenCV。
与 YAstar 的 P5 采用**同一个显式开关** `TDT_NAV_KIT_ENABLE_OPENCV`（本包 CMake 不定义）：

```diff
- #if __has_include(<opencv2/opencv.hpp>)
-     #include <opencv2/opencv.hpp>
- #endif
+ #if defined(TDT_NAV_KIT_ENABLE_OPENCV)
+     #if defined(__has_include)
+         #if __has_include(<opencv2/opencv.hpp>)
+             #include <opencv2/opencv.hpp>
+         #endif
+     #endif
+ #endif
```

关掉后，`sfcSquare` 的 `cv::Mat` 重载与 `Tools::fillPoly` 一起消失，不会出现
「声明在、定义不在」的链接错误。

## P7 · SfcSquare 原点符号统一（已单测验证）

上游 `main.cpp` 里 `originPos` 恒为 `(0,0)`，掩盖了一处不一致：

- `SfcSquare::isOutside`（`sfcSquare.cpp:40-41`）与 `MinimumSnap::lineInObsticle` 的
  `toCell`（`minimumSnap.cpp:711-714`）用 `(point - originPos)/mapping`；
- 但 `SfcSquare::valueAt` 与 4 个 `_getBound*` 用 `(x + originPos)/mapping`。

Nav2 Costmap 的 `origin_x/origin_y` 通常为**负值**，两者不等价，走廊会整体偏移。
故把 5 处共 10 行统一为减号：

```diff
- int xint = static_cast<int>((x + originPos.x()) / mapping);
+ int xint = static_cast<int>((x - originPos.x()) / mapping);
```

`minimumSnap.cpp` 已是减法，不改；`sfcSquare.cpp:39-40` 的 `isOutside` 已是减法，不动。

**验证**：`test/test_sfc_origin.cpp` 用**负原点**（`-2, -3`，分辨率 0.5）断言：

- `isOutside(-2, -3)` 为假（原点本身在图内），原点外一格为真 —— 补丁前 `(x + originPos)`
  会把原点算成栅格 `-8`，必判图外；
- `valueAt()` 在原点格与偏移 (4,7)、(7,4) 格处取到预期值；
- `MinimumSnap::lineInObsticle`（`toCell` 原本就是减号）与 `SfcSquare::valueAt` 对同一
  世界坐标给出一致结论，说明两套换算已统一。

注意：该测试需开 `setStrictCollision(true)`，否则上游硬编码的 `obstacleTolerance = 2`
会放过穿 1 格薄墙的情况（见下节）。

## P10 · SfcSquare 走廊方块的「索引 → 世界」反向换算（2026-11-23，已实测）

P7 只统一了**世界 → 索引**方向，漏掉了反方向，两个方向因此不再互逆：

```
正向（P7）：idx = (x - origin) / mapping
反向（正确）：x  = idx * mapping + origin
反向（上游）：x  = idx * mapping - origin    ← 负原点时整块平移 2*|origin|
```

原点为 0 时三者等价（上游 `main.cpp` 正是这种情况），所以一直没暴露。实车地图原点
`(-1.99, -2.04)`、分辨率 0.05 m，方块会整体偏 3.98 m / 4.08 m：

- **RViz 里走廊与路径完全错开**——这就是「走廊画不出来 / 画歪」的直接原因；
- `backend: corridor` 时 OSQP 的位置约束落在偏移后的方块上，与真实路径点不相容，
  这就是 corridor 后端长期不收敛的原因：P10 之后在实车地图同一路径上，corridor 后端
  由失败变为 3~4 ms 收敛（`round=0 bad=2/12` 后一轮即通过）。

```diff
-        static_cast<float>(left) * mapping - originPos.x(),
-        static_cast<float>(top) * mapping - originPos.y(),
-        static_cast<float>(right) * mapping - originPos.x(),
-        static_cast<float>(bottom) * mapping - originPos.y()
+        static_cast<float>(left) * mapping + originPos.x(),
+        static_cast<float>(top) * mapping + originPos.y(),
+        static_cast<float>(right) * mapping + originPos.x(),
+        static_cast<float>(bottom) * mapping + originPos.y()
```

**验证**：

- `test/test_sfc_bound_frame.cpp`（负原点、非整栅格原点）：`getBound()` 的方块必须包含
  查询点、四条边落在 `origin + k*res` 上，`getCorridor()` 每块必须包含对应路径点；
  补丁前 `BoundContainsQueryPointWithNegativeOrigin` 必然失败（实测偏移 3~5 m）。
- 2026-11-23 实车地图端到端：`/global_costmap/costmap_raw` + 2.5 m 空闲走廊路径，
  发布的 10 个方块全部含平滑路径点、中心到路径最近 0.28 m、中心全部落在空闲格上。

### P10 暴露出的上游方块扩张缺陷（未修，登记）

`_getBoundSquare2` 在某一侧贴到障碍后会**每轮把另一侧扩 2 格，却只检查 1 格**：

```cpp
if(freeTop){top--;}else{bottom++;}     // 贴顶后：bottom 先 +1
if(freeBottom){bottom++;}else if(freeTop){top--;}   // 再 +1，共 +2
```

被跳过的那一行/列永远不会再做障碍检查，所以**方块内部可能含障碍**（实车地图实测
约 4.5% 的方块内部采样格为障碍）。这与「SFC 不保证无碰撞」的既有登记一致，也是
`backend: corridor` 仍不建议上车的另一半原因。本包不修它：修了会改变 corridor 后端的
求解语义，需要单独评估；可视化里如实画出原始方块，不做美化。

> 2026-11-24：按“参数和逻辑直接抄开源 main.cpp”的要求，默认后端已切到
> `OSQPCorridor`、`order=6`、起点速度 0，并移除了路径加密/切角/fallback。
> 上述方块含障风险仍然存在；实车使用前必须在现场地图上验证。

## P11 · OSQP 求解诊断日志（2026-11-22，原表漏登记，迁包时补记）

现场排查 `backend: path/corridor` 不收敛时，上游只在失败时打印一句「OSQP未收敛」，
拿不到问题规模与失败轮次；这里补了三条**节流**诊断输出（每秒至多 1 条，不影响控制周期）：

| 位置 | 输出 | 用途 |
| --- | --- | --- |
| `iterate()` 碰撞迭代 | `[smooth-iter] round=N/M bad=k/n first_seg=i p=(x,y)` | 失败轮次的坏段数量与首个位置 |
| `osqpExecute()` 求解失败 | `[osqp-fail] axis=… status=… vars=… cons=…` | 区分「QP 解不出」与「解出来但穿障」 |
| `osqpExecute()` 解越界 | `[osqp-execute] 解无效: finite=… 下界违反=… 上界违反=…` | 数值残差导致的伪成功 |

```diff
+ // 诊断（节流）：每秒至多 1 条，记录失败轮次的 bad 段规模与首个位置。
+ static std::chrono::steady_clock::time_point lastLog;
+ if(!bad.empty() &&
+    std::chrono::steady_clock::now() - lastLog > std::chrono::seconds(1)){
+     ...
+ }
```

同时补 `#include <iomanip>`（`std::fixed`/`setprecision`）与 `#include <limits>`
（`quiet_NaN`）。这些输出只写 stdout，不改变求解语义；`srm27_nav_smoother` 的
`minimum_snap_smoother.cpp` 仍按 `result.success` 与 `result.path` 决定回退。

> 补记原因：该修改在 2026-11-22 直接做在 vendored 文件里，未同步 `PATCHES.md` 与
> sha256 表，导致登记表与实际内容不一致。迁包时已核对 diff 并重算「当前工作区 sha256」。
> 后续修改 vendored 源码必须同时改本表与 `THIRD_PARTY_NOTICES.md`。

## 未打补丁的已知上游问题（登记备查）

- `SolveInput` 默认 `backend=Close`、`collisionIteration=2`、`normTime=false`、`TL=1 s`；
  README 示例值与之不同，插件必须显式 set 全套。
- `autoBackend()` 返回 `OSQPCorridor`，但仅在 `backend==Invalid` 时被调用（默认 `Close`）。
- OSQP 未收敛 `return {}` → `success=false`，**并无**上游 `main.cpp:163` 注释所说的闭式 fallback。
- `strictCollision` 默认 `false`，`obstacleTolerance=2` 硬编码；SFC「安全走廊」不保证无碰撞。
- `_getBoundSquare2` 一侧贴墙时另一侧每轮扩 2 格只查 1 格，会跳过一行/列的障碍检查，
  方块内部可能含障碍（P10 节末实测约 4.5%），`backend: corridor` 的约束因此不严格。
- `setDt()` 在当前 `solve()` 路径实际不生效（`lineDecoder` 每段固定采样 11 点）。
- 上游 `CMakeLists.txt` 注释掉了 `CXX_STANDARD`，却无条件 `find_package(OpenCV REQUIRED)`；
  本工作区不使用该文件。
