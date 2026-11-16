# 第三方来源与许可证记录

本文件记录 `src/` 下随仓库一起版本化的第三方源码。它们的版权归原作者所有，
按各自许可证使用；SRM 侧不修改这些文件（如需修改，必须在此登记补丁说明）。

## 1. srm27_minco_vendor — GCOPTER MINCO / L-BFGS 头文件

| 项目 | 内容 |
|---|---|
| 上游仓库 | https://github.com/ZJU-FAST-Lab/GCOPTER |
| 固定 commit | `e0444f6d47b84f972ced91746b05feb36ce1fd4f` |
| 许可证 | MIT（`srm27_minco_vendor/LICENSE`，版权 2021 Zhepei Wang, Fei Gao） |
| 引入文件 | `include/gcopter/minco.hpp`、`include/gcopter/trajectory.hpp`、`include/gcopter/root_finder.hpp`、`include/gcopter/lbfgs.hpp` |
| 引入方式 | 从上述 commit 原样复制，未修改 |

被引入文件在 GCOPTER 中的原始路径为 `gcopter/include/gcopter/<name>`。

## 2. srm27_qpoases_vendor — qpOASES QP 求解器

| 项目 | 内容 |
|---|---|
| 上游仓库 | https://github.com/coin-or/qpOASES |
| 固定 commit | `9e40af7d170f440b7887fc4f9cf162f3f3ae24e8` |
| 许可证 | LGPL-2.1（`srm27_qpoases_vendor/LICENSE`） |
| 引入文件 | `include/qpOASES.hpp`、`include/qpOASES/**`、`src/*.cpp`（除下表排除项） |
| 引入方式 | 从上述 commit 原样复制，未修改源码 |

本包把 qpOASES 编译为**独立共享库** `libsrm27_qpoases.so`（LGPL-2.1 第 6 条允许的
动态链接使用方式）。使用方只通过 `qpOASES.hpp` 公开接口调用，不得静态链接进
本项目自有二进制分发。

**关于稀疏求解路径：** `SolutionAnalysis.cpp` 会引用
`SQProblemSchur::resetSchurComplement()`，因此 `SQProblemSchur.cpp` 与
`SparseSolver.cpp` **必须一起编译**，否则 `libsrm27_qpoases.so` 会带一个未定义符号，
导致链接方必须使用 `--allow-shlib-undefined` 才能链接。两者都是上游原样文件，只增加
少量编译时间，不改变本项目实际使用的稠密 `QProblem` / `SQProblem` 求解路径。

## 3. 算法参考（未直接复制代码）

`srm27_minco_core` 的二维精确欧氏距离变换（EDT）按 Felzenszwalb–Huttenlocher
两遍一维抛物线距离变换实现。该算法为公开算法；本项目在实现时以北京理工大学
追梦战队 `navi_minco_bit`（本地 commit `a860fa4cb17876e0f5d318c60143c943581d86f9`，
上游 https://github.com/limaori/navi_minco_bit ，根许可证 Apache-2.0）中的
`src/perception/rog_map/src/rog_map/esdf_utils.cpp::ESDFUtils::computeEDT2D()`
作为对照实现，并按项目接口重写为 `srm27_minco_core::Esdf2D`。

其它参考但未复制代码的来源见 `docs/迁移minco实施方案(ai).md` §3。
