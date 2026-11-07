# 上游源码最小补丁登记

对象：`vendor/` 内 `tdt-nav-kit` @ `814d5ce0237acc7faa8c070e5c5e61e2ddc81996`（tree `3274cebb...`）。
行号均为**补丁前上游原始文件**的行号。所有补丁以 `// [TDT-NAV-KIT PATCH Pn]` 注释在源码中就地标注。
目标：新包统一 **C++17**、消除隐式 OpenCV 依赖、修正非零原点下的坐标不一致。

| ID | 文件 | 上游行号 | 改动 | 本阶段是否编译/验证 |
| --- | --- | --- | --- | --- |
| P1 | `vendor/src/YAstar/yastar.cpp` | 6, 7 | 删 `#include <ranges>`、`#include <span>`（`<ranges>` 全仓无调用点） | 是，编译 |
| P2 | `vendor/src/YAstar/yastar.hpp` | 328 | 删 inl 段 `#include <ranges>` | 是，编译 |
| P3 | `vendor/src/MinimumSnapOsqp/sfcSquare.cpp` | 3 | 删 `#include <ranges>` | 否（入库不编译） |
| P4 | `vendor/src/YAstar/yastar.cpp` | 681-688、710-716 | 8 行 `std::span` 视图改为等价 `std::vector` 构造 | 是，编译 + SDF 一致性单测 |
| P5 | `vendor/src/YAstar/yastar.hpp` | 15-17 | OpenCV `__has_include` 守卫改为显式开关 `TDT_NAV_KIT_ENABLE_OPENCV` | 是，编译 |
| P6 | `vendor/src/MinimumSnapOsqp/sfcSquare.hpp` | 12-16 | 同 P5 | 否（入库不编译） |
| P7 | `vendor/src/MinimumSnapOsqp/sfcSquare.cpp` | 46-47, 145-146, 236-237, 297-298, 356-357 | 10 行 `(x + originPos)/mapping` → `(x - originPos)/mapping` | **是，阶段二已单测验证** |
| P8 | `yastar.hpp` / `yastar.cpp` / `sfcSquare.hpp` | 见下节 | 补齐 P5 关闭 OpenCV 后失去的隐式依赖：`<mutex>`、`<iostream>`、`u_char` | 是，编译 + 运行验证 |

---

## P1 / P2 / P3 · 删除未使用的 C++20 头

上游 `yastar.cpp:6-7`、`yastar.hpp:328`、`sfcSquare.cpp:3` 引了 `<ranges>` / `<span>`，但全仓**没有** `std::ranges::` / `views::` 调用点；`std::span` 只在 P4 的两处使用。C++17 下 `<ranges>` 头文件可能不存在，直接删除。

```diff
- #include <ranges>
- #include <span>
  #include <utility>
```

## P4 · `std::span` → C++17 等价写法（行为不变）

`getSDF()` 的纯 C++ Felzenszwalb 分支中，列向 pass（上游 `:681-691`）与行向 pass（`:710-719`）各建 4 个 span。核对后确认：

- 每组只有 `f_view` 在 `:688` / `:716` 被读，用于 `std::vector<float> f_vec(f_view.begin(), f_view.end())`；
- `dt_view` / `v_view` / `z_view` 定义后**从未被读取**，真正的输出是紧接着独立构造的 `dt_vec` / `v_vec` / `z_vec`；
- `dt_1d_felzenszwalb` 的签名是 `(const std::vector<float>& f, std::vector<float>& dt, std::vector<int>& v, std::vector<float>& z)`。

因此删掉 8 行 span，把 `f_vec` 直接由 `buffer` 构造，输入数据与长度完全一致：

```diff
- std::span<float> f_view(buffer.data(), height);
- std::span<float> dt_view(dt_buffer.data(), height);
- std::span<int>   v_view(v_buffer.data(), height);
- std::span<float> z_view(z_buffer.data(), height + 1);
- std::vector<float> f_vec(f_view.begin(), f_view.end());
+ std::vector<float> f_vec(buffer.data(), buffer.data() + height);
  std::vector<float> dt_vec(height);
  std::vector<int>   v_vec(height);
  std::vector<float> z_vec(height + 1);
```

行向 pass 把 `height` 换成 `width`。`buffer` 长度为 `max(width,height)`，`buffer.data()+height/width` 与原 `span(begin,end)` 范围逐字节一致。

> 副作用：`dt_buffer` / `v_buffer` / `z_buffer`（上游 `:669-671`）在补丁后不再被使用，会产生 unused-variable 警告。本包**不开 `-Wall -Werror`**，故仅为警告、不影响构建；保留它们是为了把补丁缩到最小。

**验证**：`test/test_yastar_chain.cpp` 用一张含自由/占用格的小地图，把 `getSDF()` 与独立实现的「到最近障碍格的欧氏距离 × `mapping`」暴力参考逐元素 `EXPECT_NEAR(..., 1e-4f)`，并断言同一 start/goal 下 `search()` 的点序列可复现。这比预生成数值表更强：参考实现不依赖补丁本身。

## P5 / P6 · 显式关闭 OpenCV（计划中「三处 `#if 0`」的等价实现）

上游 `yastar.hpp:15-17` 与 `sfcSquare.hpp:12-16` 用 `#if __has_include(<opencv2/opencv.hpp>)` 自动包含 OpenCV；`opencv2/opencv.hpp` 会定义 `OPENCV_ALL_HPP`，进而启用 `yastar.cpp:553` 的 `cv::distanceTransform` 快路径。若构建机装了 OpenCV 而新包 CMake 不链接 OpenCV，会编译/链接失败，且 SDF 数值与纯 C++ 分支不同。

计划原写「把两个 hpp 守卫置恒假 + 把 `yastar.cpp:553` 改 `#if 0`」。实际采用**单一显式开关**，效果相同但更一致：

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

- 新包 CMake **不定义** `TDT_NAV_KIT_ENABLE_OPENCV` → 不 include OpenCV → `OPENCV_ALL_HPP` 未定义 → `yastar.cpp:553` 自动走 `#else` 的纯 C++ Felzenszwalb 分支；
- `yastar.cpp:508` 的 `setMaskPoly` 定义与 `yastar.hpp` 里它的声明、`Tools::fillPoly` 声明/实现、`sfcSquare` 的 `cv::Mat` 重载，**都由同一个 `OPENCV_ALL_HPP` 宏统一消失**，不会出现「声明在、定义不在」的链接错误（这正是逐个改 `#if 0` 会引入的风险）；
- 阶段二若要评估 OpenCV 快路径，只需在 CMake 定义该宏并显式 `find_package(OpenCV)`，无需再改 vendored 源码。

## P7 · SfcSquare 原点符号统一（阶段二，已单测验证）

上游 `main.cpp` 里 `originPos` 恒为 `(0,0)`，掩盖了一处不一致：

- `SfcSquare::isOutside`（`sfcSquare.cpp:40-41`）与 `MinimumSnap::lineInObsticle` 的 `toCell`（`minimumSnap.cpp:711-714`）用 `(point - originPos)/mapping`；
- 但 `SfcSquare::valueAt` 与 4 个 `_getBound*` 用 `(x + originPos)/mapping`。

Nav2 Costmap 的 `origin_x/origin_y` 通常为**负值**，两者不等价，走廊会整体偏移。故把 5 处共 10 行统一为减号：

```diff
- int xint = static_cast<int>((x + originPos.x()) / mapping);
+ int xint = static_cast<int>((x - originPos.x()) / mapping);
```

`minimumSnap.cpp` 已是减法，不改；`sfcSquare.cpp:39-40` 的 `isOutside` 已是减法，不动。

**阶段二已验证**：`sfcSquare.cpp` / `minimumSnap.cpp` 已加入编译，`test/test_sfc_origin.cpp` 用**负原点**（`-2, -3`，分辨率 0.5）断言：

- `isOutside(-2, -3)` 为假（原点本身在图内），原点外一格为真 —— 补丁前 `(x + originPos)` 会把原点算成栅格 `-8`，必判图外；
- `valueAt()` 在原点格与偏移 (4,7)、(7,4) 格处取到预期值；
- `MinimumSnap::lineInObsticle`（`toCell` 原本就是减号）与 `SfcSquare::valueAt` 对同一世界坐标给出一致结论，说明两套换算已统一。

注意：该测试需开 `setStrictCollision(true)`，否则上游硬编码的 `obstacleTolerance = 2` 会放过穿 1 格薄墙的情况（见下节）。

---

## P8 · 关闭 OpenCV 后补齐隐式依赖（实施期发现，已本机验证）

P5 把 `opencv2/opencv.hpp` 从编译单元里拿掉后，上游**依赖 OpenCV 头间接提供**的三样东西立即暴露出来。本机用 `g++ -std=c++17 -fsyntax-only` 逐个定位并补齐：

| 缺失 | 位置 | 补齐 |
| --- | --- | --- |
| `std::mutex`（`mutex` 不在 `std` 中） | `yastar.hpp:336` 的 `mutable std::mutex mapLocker` | `yastar.hpp` 加 `#include <mutex>` |
| `std::cout` / `std::endl` | `yastar.cpp` 多处掩码告警输出 | `yastar.cpp` 加 `#include <iostream>` |
| `u_char`（BSD/POSIX 类型，Windows 无） | `yastar.hpp`、`sfcSquare.hpp` 的 `TMatrix<u_char>` / `setMap(..., u_char*)` | 两个 hpp 加条件兜底：Linux 走 `#include <sys/types.h>`；`_WIN32`/`__MINGW32__` 补 `using u_char = unsigned char;` |

```cpp
// [TDT-NAV-KIT PATCH P8]
#if defined(_WIN32) || defined(__MINGW32__)
  #include <cstdint>
using u_char = unsigned char;
#else
  #include <sys/types.h>
#endif
```

这不是"顺手改动"，而是 P5 的必要配套：不补，关闭 OpenCV 后在 Ubuntu 上也可能因 `<sys/types.h>` 未被间接包含而编译失败。`sfcSquare.hpp` 本阶段不编译，但一并按同一规则补齐，保证入库即正确。

## 本机验证结果（2026-11-04）

本机无 ROS，但用 MinGW `g++` + Eigen 3.4.0 头文件对**补丁后的 `yastar.cpp`** 做了实际编译与运行：

```
g++ -std=c++17 -fsyntax-only ... yastar.cpp        -> exit 0
g++ -std=c++17 -O1 ... yastar.cpp + 独立验证程序 -> 编译、链接通过
```

运行结果（与 `test/test_yastar_chain.cpp` 同一张 12x8 带墙地图）：

```
SDF max_err = 0 | mismatches(>2e-4) = 0
search points = 10
simplified points = 4 | colliding segments = 0
out-of-map search points = 1 (expect < 2)
RESULT: PASS
```

即：P1–P5、P8 全部生效后，`getSDF()` 与独立实现的「到最近障碍格欧氏距离 × resolution」**逐元素完全一致（误差 0）**，`search()` / `simplifyPath()` / `simplifyPathHypot()` / 越界失败返回单点路径的行为均符合预期。插件包装层与 ROS 侧签名仍需在 ROS 2 Humble 构建机上验证。

## 未打补丁的已知上游问题（阶段二处理，登记备查）

- `SolveInput` 默认 `backend=Close`、`collisionIteration=2`、`normTime=false`、`TL=1 s`；README 示例值与之不同，插件必须显式 set 全套。
- `autoBackend()` 返回 `OSQPCorridor`，但仅在 `backend==Invalid` 时被调用（默认 `Close`）。
- OSQP 未收敛 `return {}` → `success=false`，**并无** `main.cpp:163` 注释所说的闭式 fallback。
- `strictCollision` 默认 `false`，`obstacleTolerance=2` 硬编码；SFC「安全走廊」不保证无碰撞。
- `setDt()` 在当前 `solve()` 路径实际不生效（`lineDecoder` 每段固定采样 11 点）。
- 上游 `CMakeLists.txt` 注释掉了 `CXX_STANDARD`，却无条件 `find_package(OpenCV REQUIRED)`；本包不使用该文件。
