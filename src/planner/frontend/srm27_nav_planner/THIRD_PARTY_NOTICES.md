# 第三方源码登记表 · YAstar（tdt-nav-kit）

本包 `vendor/` 内置入东北大学 T-DT 上游 `tdt-nav-kit` 的 YAstar 源码。
引入时**先原样拷贝，再按 `PATCHES.md` 打最小补丁**。

> 2026-11-24：上源代码从共享包 `srm27_nav_common` 按算法拆到各自的插件包——
> YAstar 进本包（`srm27_nav_planner`），MinimumSnap/SfcSquare 进
> `srm27_nav_smoother`。本包因此**自包含**：只依赖 Eigen、ROS/Nav2 与同工作区的
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
| `src/YAstar/yastar.hpp` | 17575 | `0baf0b584d33f586bb20b269fd85b3a6356921cae5b9346189341a85375bce8d` | `73344120bb510e7bd185af9afd65a567cc1d01cccd52922e62a12b1cb5f94a2c` |
| `src/YAstar/yastar.cpp` | 35949 | `382e95043448f42493a7b5b79052e7cb103ac7406ec4428cdfb6d03e989b90a3` | `66a05bcaa841576521559d0af739e62ae1de749b79d05ec915354de07c37c348` |
| `src/YAstar/kinodynamicAstar.hpp` | 3029 | `6479ee7ba65e0b4b6bced996672dbd0a06646bcc9525ec4e572bdf9ccd893b4f` | 同左（未改） |
| `src/YAstar/kinodynamicAstar.cpp` | 13303 | `a3640eb5b8591c94ada574f3cdf33391f5e81e92619944075e35d853c55965c7` | 同左（未改） |
| `LICENCE` | 1091 | `fab3c8fd7e83f54e1d8a45d08d3683eb8ba47a52a08bb7dbaac550d7160677a0` | 同左（**原样不改**） |

> 「当前工作区 sha256」含全部已登记补丁（P1/P2/P4/P5/P8/P9）。最初引入时该列记的是
> P1–P5/P8 之后的值；2026-11-16 追加 P9（对角穿角）后 yastar.cpp 变了，2026-11-24 迁包时
> 按实际内容重算。**新增/修改补丁时必须同步更新本列**。
>
> 上游的 `main.cpp`（示例 + 可视化）、`CMakeLists.txt`、`scripts/setup.sh`、`.gitmodules`、
> `doc/`、`images/` **未引入**。
>
> **关于 sha256 的可复现性**：上表按 **LF 内容**计算。仓库根无 `.gitattributes` 且本机
> `core.autocrlf=true`，Windows 上重新 checkout 会把工作区文件转成 CRLF，直接对工作区文件
> 求哈希会得到不同结果。校验请用 `git show HEAD:<path> | sha256sum`（取仓库 blob），
> 或临时以 `-c core.autocrlf=false` checkout。

## 编译范围

`CMakeLists.txt` 只编译 `vendor/src/YAstar/yastar.cpp`。`kinodynamicAstar.cpp` 入库但
**不编译**：上游自述全局搜索约 2 s，主链路不用普通 A* 之外的搜索。

## 许可证适用边界

- `vendor/**` 与 `vendor/LICENCE` 适用 **MIT**，保留原版权声明，未添加任何 Apache-2.0 头，未重新授权。
- 本包其余文件（`include/`、`src/`、`test/`、`tools/`、`CMakeLists.txt`、`package.xml` 等）
  为本工作区新增，适用仓库根 `src/bringup/srm27_navigation/LICENSE` 的 **Apache-2.0**。
- 两套许可各自独立存放、互不混改。上游依赖另有：Eigen MPL-2.0。

## 升级流程（禁止「顺手同步」）

1. 下载目标 commit，先与 `vendor/` 生成 diff；
2. 逐条核对 `PATCHES.md` 的每条补丁是否仍必要、行号是否漂移；
3. 更新本表的 commit sha / tree sha / 文件 sha256；
4. 更新 `PATCHES.md`，并跑 `test/test_yastar_chain.cpp` 的 SDF 一致性测试；
5. `vendor/` 位于 `src/planner/frontend/srm27_nav_planner/`，不在 `dependencies.repos`
   的任何映射中，`vcs import` 不会接管。
