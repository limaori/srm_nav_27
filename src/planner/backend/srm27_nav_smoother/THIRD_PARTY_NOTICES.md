# 第三方源码登记表 · MinimumSnap + SfcSquare（tdt-nav-kit）

本包 `vendor/` 内置入东北大学 T-DT 上游 `tdt-nav-kit` 的 MinimumSnap（OSQP 求解）
与 SfcSquare（方形安全走廊）源码。引入时**先原样拷贝，再按 `PATCHES.md` 打最小补丁**。

> 2026-11-24：上源代码从共享包 `srm27_nav_common` 按算法拆到各自的插件包——
> MinimumSnap/SfcSquare 进本包（`srm27_nav_smoother`），YAstar 进 `srm27_nav_planner`。
> 本包因此**自包含**：只依赖 Eigen、OSQP/OsqpEigen、ROS/Nav2 与同工作区的
> `srm27_nav_common`（路径工具）、`srm27_map_*`（地图类型）。

## 来源与版本

| 项 | 值 |
| --- | --- |
| 上游仓库 | `https://github.com/T-DT-Algorithm-2026/tdt-nav-kit` |
| 引入 commit | `814d5ce0237acc7faa8c070e5c5e61e2ddc81996` |
| 该 commit 的 tree sha | `3274cebb0459c8d650466d675da6774c514bf268` |
| commit 时间 / 标题 | 2026-10-09 `Update README with installation commands` |
| 许可证 | **MIT**，原文件 `vendor/LICENCE`（英式拼写；上游 README 链接的 `LICENSE` 并不存在，此处不改名） |
| 版权 | Copyright (c) 2026 Rongxuan Ye and BoLin Li（东北大学 T-DT） |
| 取源通道 | `https://codeload.github.com/T-DT-Algorithm-2026/tdt-nav-kit/zip/<commit sha>` 整仓 zip，再从内存解出指定文件；`api.github.com` 用于单文件校验。本环境 `raw.githubusercontent.com` 不可达 |
| 引入日期 | 2026-11-04（2026-11-24 迁入本包） |

## 引入文件清单（sha256 为**上游原始、补丁前**内容）

| 包内路径（相对 `vendor/`） | 原始字节 | 上游原始 sha256 | 当前工作区 sha256 |
| --- | ---: | --- | --- |
| `src/MinimumSnapOsqp/minimumSnap.hpp` | 11916 | `1dff6839799fe7f8a6193eabbec2d9adce985d2c69bc5f0c423fb838b9439b3c` | 同左（未改） |
| `src/MinimumSnapOsqp/minimumSnap.cpp` | 40612 | `368e220c8cfc4928decd8668c3d09c7bebaf8492b59be134a8337b5f5a56d1c0` | `f11c7bb98ac988446e573dc775197627753963c0fad7f42e6c3b60a7141f6112` |
| `src/MinimumSnapOsqp/sfcSquare.hpp` | 4674 | `bfaee73cbfda1f46aea794b80da29d7ea3651ed80a45efd50a5f05fe57145c8b` | `b4f98ca8ef7e741bb33312eb79902d764068c2f145227e1f1e20af028d6bc3f7` |
| `src/MinimumSnapOsqp/sfcSquare.cpp` | 19545 | `05f9d5c028af2d0a353abf2a85ff44ef7569877758cf3c374cc735e8050986dd` | `84c516d4bc1637cfbce78aeb75e190f61319ba30cd2eb4d29f25935c08cfa43a` |
| `LICENCE` | 1091 | `fab3c8fd7e83f54e1d8a45d08d3683eb8ba47a52a08bb7dbaac550d7160677a0` | 同左（**原样不改**） |

> 「当前工作区 sha256」含全部已登记补丁（P3/P6/P7/P10/P11）。最初引入时该列记的是
> P3/P6/P7 之后的值；2026-11-23 追加 P10（索引→世界补原点）、P11（OSQP 失败诊断日志）
> 后变了，2026-11-24 迁包时按实际内容重算。**新增/修改补丁时必须同步更新本列**。
>
> 上游的 `main.cpp`（示例 + 可视化）、`CMakeLists.txt`、`scripts/setup.sh`、`.gitmodules`、
> `doc/`、`images/` **未引入**。
>
> **关于 sha256 的可复现性**：上表按 **LF 内容**计算。仓库根无 `.gitattributes` 且本机
> `core.autocrlf=true`，Windows 上重新 checkout 会把工作区文件转成 CRLF，直接对工作区文件
> 求哈希会得到不同结果。校验请用 `git show HEAD:<path> | sha256sum`（取仓库 blob），
> 或临时以 `-c core.autocrlf=false` checkout。

## 编译范围

`CMakeLists.txt` 编译 `minimumSnap.cpp` 与 `sfcSquare.cpp`，两者需要 OSQP
（ROS 包 `osqp_vendor`）与 OsqpEigen（无 rosdep key，需源码安装，见本包 README）。

## 许可证适用边界

- `vendor/**` 与 `vendor/LICENCE` 适用 **MIT**，保留原版权声明，未添加任何 Apache-2.0 头，未重新授权。
- 本包其余文件（`include/`、`src/`、`test/`、`CMakeLists.txt`、`package.xml` 等）
  为本工作区新增，适用仓库根 `src/bringup/srm27_navigation/LICENSE` 的 **Apache-2.0**。
- 两套许可各自独立存放、互不混改。上游依赖另有：OSQP Apache-2.0、OsqpEigen BSD-3-Clause、Eigen MPL-2.0。

## 升级流程（禁止「顺手同步」）

1. 下载目标 commit，先与 `vendor/` 生成 diff；
2. 逐条核对 `PATCHES.md` 的每条补丁是否仍必要、行号是否漂移；
3. 更新本表的 commit sha / tree sha / 文件 sha256；
4. 更新 `PATCHES.md`，并跑 `test/test_sfc_origin.cpp`、`test/test_sfc_bound_frame.cpp`；
5. `vendor/` 位于 `src/planner/backend/srm27_nav_smoother/`，不在 `dependencies.repos`
   的任何映射中，`vcs import` 不会接管。
