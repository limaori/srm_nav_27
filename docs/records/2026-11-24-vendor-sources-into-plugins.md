# 2026-11-24 上游算法源码拆入各自插件包

> 状态：已完成（构建 + 单测 + bringup 回归）
> 范围：`srm27_nav_common/vendor/**` → `srm27_nav_planner/vendor/**` 与
> `srm27_nav_smoother/vendor/**`
> 未改动：算法源码本身（仍是同一批 sha256）、补丁内容、插件 ID、Nav2 参数、启动入口

## 1. 动机

上一轮把地图拆出 `src/planner/map` 后，`srm27_nav_common` 里仍同时躺着 YAstar 与
MinimumSnap/SfcSquare 两套 MIT 源码，并通过 `ament_export_include_directories`
把 `vendor/` 头导出给两个插件包。结果是：

- 想只改 YAstar，却要动一个"共享底座"包；
- `srm27_nav_common` 既是 Apache-2.0 又是 MIT，许可边界只能靠目录约定；
- 两个插件不算 self-contained：单独 `--packages-select` 构建前必须先有 common 的导出；
- `find_package(OsqpEigen)` 挂在 common 上，导致只想用 YAstar 的包也要装 OSQP 依赖。

## 2. 新归属

```text
src/planner/frontend/srm27_nav_planner/
├── vendor/LICENCE
├── vendor/src/YAstar/{yastar.hpp,yastar.cpp,kinodynamicAstar.hpp,kinodynamicAstar.cpp}
├── PATCHES.md                 # P1/P2/P4/P5/P8/P9
├── THIRD_PARTY_NOTICES.md     # 来源、commit、sha256、许可边界
├── tools/{bench_yastar.cpp,bench_planner.py,README.md}
└── test/test_yastar_chain.cpp

src/planner/backend/srm27_nav_smoother/
├── vendor/LICENCE
├── vendor/src/MinimumSnapOsqp/{minimumSnap.*,sfcSquare.*}
├── PATCHES.md                 # P3/P6/P7/P10 + 已知上游缺陷
├── THIRD_PARTY_NOTICES.md
└── test/{test_sfc_origin,test_sfc_bound_frame}.cpp

src/planner/common/srm27_nav_common/   # 只剩路径工具（Apache-2.0，无第三方代码）
├── include/srm27_nav_common/{path_utils,path_conversion}.hpp
└── test/{test_path_utils,test_path_conversion}.cpp
```

## 3. 构建与导出变化

| 项 | 之前（common 集中） | 现在（各包自带） |
|---|---|---|
| 编译 | `libsrm27_nav_common.so` 同时含 YAstar + MinimumSnap | `libsrm27_nav_planner.so` 含 YAstar；`libsrm27_nav_smoother.so` 含 MinimumSnap/SfcSquare |
| vendor 头 | `PUBLIC` + `ament_export_include_directories`，下游可见 | `PRIVATE`，只本包源码与测试可见；安装到 `include/<pkg>/vendor/` 仅供溯源 |
| OSQP/OsqpEigen | common 的 `find_package` 与 `package.xml` | 移到 `srm27_nav_smoother`（planner 不再需要 OSQP） |
| 许可 | 一个包双许可 | 两包各 `Apache-2.0` + `MIT`，各有 `vendor/LICENCE` 与 `THIRD_PARTY_NOTICES.md` |
| 依赖 common | 需要（算法） | 只需要路径工具（`path_utils`/`path_conversion`） |

补丁与出处登记表按算法拆成两份：

- `PATCHES.md`：YAstar 侧 P1/P2/P4/P5/P8/P9；MinimumSnap 侧 P3/P6/P7/P10/P11 + 上游缺陷。
- `THIRD_PARTY_NOTICES.md`：各自列本包引入的 4 个文件 + `LICENCE` 的原始/当前 sha256，
  上游 commit / tree / 引入日期 / 取源通道保持一致。

### 迁包时发现并修正的登记缺口

拆分时把 vendored 文件与 `d658b83`（最初入库）做了 diff，发现登记表有两处与实物不一致：

| 问题 | 事实 | 处置 |
|---|---|---|
| 「补丁后 sha256」是 2026-11-04 的快照 | `yastar.cpp` 在 P9（2026-11-16）、`sfcSquare.cpp` 在 P10（2026-11-23）后已变 | 列名改为「当前工作区 sha256」，按实际内容重算 |
| `minimumSnap.cpp` 登记为「未改」 | 实际有 +39 行的节流诊断日志（`[smooth-iter]`/`[osqp-fail]`/`[osqp-execute]`）与 `<iomanip>`/`<limits>` | 补登记为 **P11**，并写入 `PATCHES.md` |

上游原始 sha256 一列未动，仍是校验依据；`test_yastar_chain`、`test_sfc_origin`、
`test_sfc_bound_frame` 迁移后全部通过，说明补丁行为一致。

## 4. 验证

```bash
./scripts/build/build_safe.sh --packages-select srm27_nav_common \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
./scripts/build/build_safe.sh --packages-select srm27_nav_planner --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
./scripts/build/build_safe.sh --packages-select srm27_nav_smoother --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
```

| 包 | gtest | 结果 |
|---|---|---|
| `srm27_nav_common` | `test_path_utils`、`test_path_conversion` | **14 tests, 0 failures** |
| `srm27_nav_planner` | `test_pluginlib_load`、`test_planner_helpers`、`test_yastar_chain` | **17 tests, 0 failures** |
| `srm27_nav_smoother` | `test_pluginlib_load`、`test_sfc_markers`、`test_sfc_origin`、`test_sfc_bound_frame` | **19 tests, 0 failures** |

- `srm27_minco_core`、`srm27_minco_controller` 重编通过（它们只依赖地图包）；
- `srm27_nav_bringup` 9 个 Python 回归脚本全部 PASS（插件 ID 未变）。

安装后布局：

```text
install/srm27_nav_planner/include/srm27_nav_planner/vendor/YAstar/*.hpp
install/srm27_nav_planner/share/srm27_nav_planner/{LICENCE,PATCHES.md,THIRD_PARTY_NOTICES.md}
install/srm27_nav_smoother/include/srm27_nav_smoother/vendor/MinimumSnapOsqp/*.hpp
install/srm27_nav_smoother/share/srm27_nav_smoother/{LICENCE,PATCHES.md,THIRD_PARTY_NOTICES.md}
install/srm27_nav_common/include/srm27_nav_common/{path_utils,path_conversion}.hpp
```

## 5. 遗留

- 仓库外若有人 `#include "yastar.hpp"` 并指望从 `srm27_nav_common` 拿到 include 路径，
  现在需要直接依赖对应插件包（vendor 头已不导出）；
- `srm27_nav_common` 仍是两个插件共用的路径工具包。若后续希望"一个共同依赖都没有"，
  可以把 `path_utils`/`path_conversion` 也复制进两个包，但会引入重复实现，暂不做；
- 实车与 Gazebo 闭环未复测；算法与插件 ID 均未变。
