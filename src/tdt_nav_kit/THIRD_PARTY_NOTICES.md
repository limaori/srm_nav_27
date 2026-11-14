# 第三方源码登记表 · tdt-nav-kit

本包 `vendor/` 内包含从上游仓库按固定提交引入的源码。引入时**先原样拷贝，再按 `PATCHES.md` 打最小补丁**。

## 来源与版本

| 项 | 值 |
| --- | --- |
| 上游仓库 | `https://github.com/T-DT-Algorithm-2026/tdt-nav-kit` |
| 引入 commit | `814d5ce0237acc7faa8c070e5c5e61e2ddc81996` |
| 该 commit 的 tree sha | `3274cebb0459c8d650466d675da6774c514bf268` |
| commit 时间 / 标题 | 2026-10-09 `Update README with installation commands` |
| 许可证 | **MIT**，原文件 `vendor/LICENCE`（英式拼写；上游 README 链接的 `LICENSE` 并不存在，此处不改名） |
| 版权 | Copyright (c) 2026 Rongxuan Ye and BoLin Li（东北大学 T-DT） |
| 取源通道 | `https://codeload.github.com/T-DT-Algorithm-2026/tdt-nav-kit/zip/<commit sha>` 整仓 zip，再从内存解出指定 8 个文件；`api.github.com` 用于单文件校验。本环境 `raw.githubusercontent.com` 不可达 |
| 引入日期 | 2026-11-04 |

## 引入文件清单（sha256 为**上游原始、补丁前**内容）

| 包内路径（相对 `vendor/`） | 原始字节 | 上游原始 sha256 | 补丁后 sha256 |
| --- | ---: | --- | --- |
| `src/YAstar/yastar.hpp` | 17575 | `0baf0b584d33f586bb20b269fd85b3a6356921cae5b9346189341a85375bce8d` | `73344120bb510e7bd185af9afd65a567cc1d01cccd52922e62a12b1cb5f94a2c` |
| `src/YAstar/yastar.cpp` | 35949 | `382e95043448f42493a7b5b79052e7cb103ac7406ec4428cdfb6d03e989b90a3` | `b14f03fb9c4b45765b2feb967bdbc31d03d6d8d1eb2034bf856d11078a941133` |
| `src/YAstar/kinodynamicAstar.hpp` | 3029 | `6479ee7ba65e0b4b6bced996672dbd0a06646bcc9525ec4e572bdf9ccd893b4f` | 同左（未改） |
| `src/YAstar/kinodynamicAstar.cpp` | 13303 | `a3640eb5b8591c94ada574f3cdf33391f5e81e92619944075e35d853c55965c7` | 同左（未改） |
| `src/MinimumSnapOsqp/minimumSnap.hpp` | 11916 | `1dff6839799fe7f8a6193eabbec2d9adce985d2c69bc5f0c423fb838b9439b3c` | 同左（未改） |
| `src/MinimumSnapOsqp/minimumSnap.cpp` | 40612 | `368e220c8cfc4928decd8668c3d09c7bebaf8492b59be134a8337b5f5a56d1c0` | 同左（未改） |
| `src/MinimumSnapOsqp/sfcSquare.hpp` | 4674 | `bfaee73cbfda1f46aea794b80da29d7ea3651ed80a45efd50a5f05fe57145c8b` | `b4f98ca8ef7e741bb33312eb79902d764068c2f145227e1f1e20af028d6bc3f7` |
| `src/MinimumSnapOsqp/sfcSquare.cpp` | 19545 | `05f9d5c028af2d0a353abf2a85ff44ef7569877758cf3c374cc735e8050986dd` | `2a10c6812da195faab88fe2573ded02dc83f05b94f9de6acef3144f8610b49aa` |
| `LICENCE` | 1091 | `fab3c8fd7e83f54e1d8a45d08d3683eb8ba47a52a08bb7dbaac550d7160677a0` | 同左（**原样不改**） |

> 上游的 `main.cpp`（示例 + 可视化）、`CMakeLists.txt`、`scripts/setup.sh`、`.gitmodules`、`doc/`、`images/` **未引入**。
>
> **关于 sha256 的可复现性**：上表按 **LF 内容**计算。仓库根无 `.gitattributes` 且本机 `core.autocrlf=true`，Windows 上重新 checkout 会把工作区文件转成 CRLF，直接对工作区文件求哈希会得到不同结果。校验请用 `git show HEAD:<path> | sha256sum`（取仓库 blob），或临时以 `-c core.autocrlf=false` checkout。

## 编译范围

`CMakeLists.txt` 编译 `vendor/src/YAstar/yastar.cpp`、`vendor/src/MinimumSnapOsqp/minimumSnap.cpp`
与 `vendor/src/MinimumSnapOsqp/sfcSquare.cpp`；后两者需要 OSQP（`osqp_vendor`）与
OsqpEigen（无 rosdep key，源码安装，见 README）。`kinodynamicAstar.cpp` 仍不编译：
上游自述全局搜索约 2 s，主链路不用普通 A* 之外的搜索。

## 许可证适用边界

- `vendor/**` 下的上游源码与 `vendor/LICENCE` 适用 **MIT**，保留原版权声明，未添加任何 Apache-2.0 头，未重新授权。
- 本包其余文件（`include/`、`src/`、`config/`、`launch/`、`test/`、`CMakeLists.txt`、`package.xml` 等）为本工作区新增，适用仓库根 `src/srm27_navigation/LICENSE` 的 **Apache-2.0**。
- 两套许可各自独立存放、互不混改。上游依赖另有：OSQP Apache-2.0、OsqpEigen BSD-3-Clause、Eigen MPL-2.0（阶段二才会引入 OSQP / OsqpEigen）。

## 升级流程（禁止「顺手同步」）

1. 下载目标 commit，先与 `vendor/` 生成 diff；
2. 逐条核对 `PATCHES.md` 的每条补丁是否仍必要、行号是否漂移；
3. 更新本表的 commit sha / tree sha / 文件 sha256；
4. 更新 `PATCHES.md`，并在 `test/test_yastar_chain.cpp` 跑 SDF 一致性测试；
5. `vendor/` 位于 `src/tdt_nav_kit/`，不在 `dependencies.repos` 的任何映射中，`vcs import` 不会接管。
