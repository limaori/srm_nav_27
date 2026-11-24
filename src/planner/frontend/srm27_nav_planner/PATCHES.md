# 上游源码最小补丁登记 · YAstar

对象：本包 `vendor/` 内 `tdt-nav-kit` @ `814d5ce0237acc7faa8c070e5c5e61e2ddc81996`（tree `3274cebb...`）。
行号均为**补丁前上游原始文件**的行号。所有补丁以 `// [TDT-NAV-KIT PATCH Pn]` 注释在源码中就地标注。
目标：统一 **C++17**、消除隐式 OpenCV 依赖。

> 2026-11-24：本表从 `srm27_nav_common/PATCHES.md` 按算法拆分而来。MinimumSnap/SfcSquare
> 的补丁（P3、P6、P7、P10 及走廊扩张缺陷）见相邻包
> `src/planner/backend/srm27_nav_smoother/PATCHES.md`。

| ID | 文件 | 上游行号 | 改动 | 验证 |
| --- | --- | --- | --- | --- |
| P1 | `vendor/src/YAstar/yastar.cpp` | 6, 7 | 删 `#include <ranges>`、`#include <span>`（`<ranges>` 全仓无调用点） | 编译 |
| P2 | `vendor/src/YAstar/yastar.hpp` | 328 | 删 inl 段 `#include <ranges>` | 编译 |
| P4 | `vendor/src/YAstar/yastar.cpp` | 681-688、710-716 | 8 行 `std::span` 视图改为等价 `std::vector` 构造 | 编译 + SDF 一致性单测 |
| P5 | `vendor/src/YAstar/yastar.hpp` | 15-17 | OpenCV `__has_include` 守卫改为显式开关 `TDT_NAV_KIT_ENABLE_OPENCV` | 编译 |
| P8 | `yastar.hpp` / `yastar.cpp` | 见下节 | 补齐 P5 关闭 OpenCV 后失去的隐式依赖：`<mutex>`、`<iostream>`、`u_char` | 编译 + 运行验证 |
| P9 | `vendor/src/YAstar/yastar.cpp` | 八邻域搜索 | 对角边相邻两格占据检查（禁止穿障碍格角） | `test_yastar_chain` |

---

## P1 / P2 · 删除未使用的 C++20 头

上游 `yastar.cpp:6-7`、`yastar.hpp:328` 引了 `<ranges>` / `<span>`，但全仓**没有**
`std::ranges::` / `views::` 调用点；`std::span` 只在 P4 的两处使用。C++17 下
`<ranges>` 可能不存在，直接删除。

```diff
- #include <ranges>
- #include <span>
  #include <utility>
```

## P4 · `std::span` → C++17 等价写法（行为不变）

`getSDF()` 的纯 C++ Felzenszwalb 分支中，列向 pass（上游 `:681-691`）与行向 pass
（`:710-719`）各建 4 个 span。核对后确认：

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

行向 pass 把 `height` 换成 `width`。`buffer` 长度为 `max(width,height)`，
`buffer.data()+height/width` 与原 `span(begin,end)` 范围逐字节一致。

> 副作用：`dt_buffer` / `v_buffer` / `z_buffer`（上游 `:669-671`）补丁后不再被使用，
> 会产生 unused-variable 警告。本包**不开 `-Wall -Werror`**，仅为警告；保留它们是为把
> 补丁缩到最小。

**验证**：`test/test_yastar_chain.cpp` 用一张含自由/占用格的小地图，把 `getSDF()` 与
独立实现的「到最近障碍格的欧氏距离 × `mapping`」暴力参考逐元素 `EXPECT_NEAR(..., 1e-4f)`，
并断言同一 start/goal 下 `search()` 的点序列可复现。

## P5 · 显式关闭 OpenCV

上游 `yastar.hpp:15-17` 用 `#if __has_include(<opencv2/opencv.hpp>)` 自动包含 OpenCV；
`opencv2/opencv.hpp` 会定义 `OPENCV_ALL_HPP`，进而启用 `yastar.cpp:553` 的
`cv::distanceTransform` 快路径。若构建机装了 OpenCV 而本包 CMake 不链接 OpenCV，
会编译/链接失败，且 SDF 数值与纯 C++ 分支不同。改为**单一显式开关**：

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

- 本包 CMake **不定义** `TDT_NAV_KIT_ENABLE_OPENCV` → 不 include OpenCV →
  `OPENCV_ALL_HPP` 未定义 → `yastar.cpp:553` 走纯 C++ Felzenszwalb 分支；
- `yastar.cpp:508` 的 `setMaskPoly`、`yastar.hpp` 里它的声明、`Tools::fillPoly`
  声明/实现，**都由同一个 `OPENCV_ALL_HPP` 宏统一消失**，不会出现「声明在、定义不在」
  的链接错误（逐个改 `#if 0` 会引入这种风险）；
- 若要评估 OpenCV 快路径，只需在 CMake 定义该宏并显式 `find_package(OpenCV)`，
  无需再改 vendored 源码。

## P8 · 关闭 OpenCV 后补齐隐式依赖

P5 把 `opencv2/opencv.hpp` 从编译单元里拿掉后，上游**依赖 OpenCV 头间接提供**的三样
东西立即暴露。本机用 `g++ -std=c++17 -fsyntax-only` 逐个定位并补齐：

| 缺失 | 位置 | 补齐 |
| --- | --- | --- |
| `std::mutex`（`mutex` 不在 `std` 中） | `yastar.hpp:336` 的 `mutable std::mutex mapLocker` | `yastar.hpp` 加 `#include <mutex>` |
| `std::cout` / `std::endl` | `yastar.cpp` 多处掩码告警输出 | `yastar.cpp` 加 `#include <iostream>` |
| `u_char`（BSD/POSIX 类型，Windows 无） | `yastar.hpp` 的 `TMatrix<u_char>` / `setMap(..., u_char*)` | 条件兜底：Linux 走 `#include <sys/types.h>`；`_WIN32`/`__MINGW32__` 补 `using u_char = unsigned char;` |

```cpp
// [TDT-NAV-KIT PATCH P8]
#if defined(_WIN32) || defined(__MINGW32__)
  #include <cstdint>
using u_char = unsigned char;
#else
  #include <sys/types.h>
#endif
```

这是 P5 的必要配套：不补，关闭 OpenCV 后在 Ubuntu 上也可能因 `<sys/types.h>` 未被
间接包含而编译失败。

## P9 · 对角穿角检查（2026-11-16）

`vendor/src/YAstar/yastar.cpp` 的八邻域搜索新增对角边相邻两格的占据检查，禁止从障碍格
角穿过。对应的包装层配套（栅格中心换算、稳定末点、连续线段碰撞检查）在
`srm27_nav_planner` 的 `yastar_path.cpp` / `goal_resolver.cpp`，回归见
`test/test_planner_helpers.cpp` 与 `test/test_yastar_chain.cpp`。

## 本机验证结果（2026-11-04，P1–P5、P8 生效后）

用 MinGW `g++` + Eigen 3.4.0 头文件对**补丁后的 `yastar.cpp`** 做了实际编译与运行：

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

即：`getSDF()` 与独立实现的「到最近障碍格欧氏距离 × resolution」**逐元素完全一致（误差 0）**，
`search()` / `simplifyPath()` / `simplifyPathHypot()` / 越界失败返回单点路径的行为均符合预期。
