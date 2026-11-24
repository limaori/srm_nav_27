# 2026-11-24 决策模块按 common / frontend / backend 分层

> 状态：已完成（构建 + gtest + 决策场景回归脚本；实车未复测）
> 范围：`src/decision/srm27_behavior` → `src/decision/{common,frontend,backend}`
> 未改动：决策树 XML、参数语义、BT 节点注册名、ExecuteTree action 名、启动入口
> （`ros2 launch srm27_behavior srm27_behavior_launch.py`、`start_*_nav.sh --behavior`）

## 1. 背景

原决策模块只有一个巨型包 `srm27_behavior`（76 个文件）：

- 执行服务端/客户端、裁判输入、22 个 BehaviorTree 插件、决策树、参数、launch、
  现场验证脚本全在一个 `CMakeLists.txt` 里；
- 共享状态/算法（航点加载、语义区、到点滞回、自转参考、姿态预算）以
  `include/srm27_behavior/*.hpp` 混在服务端头文件旁，没有"共享底座"边界；
- `test_decision_core` 里唯一需要真实 BT 节点的姿态用例与纯算法用例混在一起；
- 22 个插件库平铺安装到 `share/srm27_behavior/bt_plugins`，服务端参数写死包名。

本轮按 planner 的 `common / frontend / backend` 框架重排，不改变决策逻辑。

## 2. 目标结构

```text
src/decision/
├── common/srm27_behavior_core/        # header-only 共享底座
├── frontend/srm27_behavior_plugins/   # BehaviorTree 插件（动作/条件/控制/装饰器）
└── backend/srm27_behavior/            # 执行服务端/客户端、裁判输入、决策树/参数/launch/验证
```

依赖方向：

```text
srm27_behavior_plugins ──► srm27_behavior_core ◄── srm27_behavior（服务端）
                                    ▲
                                    └── 服务端运行时再 dlopen 插件 .so
```

前端（插件）与后端（服务端）没有编译依赖；服务端通过参数
`plugins: [srm27_behavior_plugins/bt_plugins]` 枚举插件目录。

## 3. 文件归属

| 原路径（`src/decision/srm27_behavior/`） | 新路径 |
|---|---|
| `include/srm27_behavior/{navigation_spin_tracker,semantic_map,sentry_posture_state,spin_reference,waypoint_utils}.hpp` | `common/srm27_behavior_core/include/srm27_behavior_core/` |
| `test/test_decision_core.cpp` 的纯算法用例 | `common/srm27_behavior_core/test/test_decision_core.cpp` |
| `include/srm27_behavior/plugins/{action,condition,control,decorator}/*.hpp` | `frontend/srm27_behavior_plugins/include/srm27_behavior_plugins/{action,condition,control,decorator}/` |
| `include/srm27_behavior/custom_types.hpp` | `frontend/srm27_behavior_plugins/include/srm27_behavior_plugins/custom_types.hpp` |
| `plugins/{action,condition,control,decorator}/*.cpp` | `frontend/srm27_behavior_plugins/plugins/`（源码目录保留） |
| `test/test_decision_core.cpp` 的姿态用例 | `frontend/srm27_behavior_plugins/test/test_set_sentry_posture.cpp` |
| `src/{srm27_behavior_server,srm27_behavior_client,referee_inputs}.cpp`、对应头文件 | `backend/srm27_behavior/`（包名不变） |
| `decision_trees/`、`params/`、`launch/`、`scripts/`、`test/{test_decision_ros,test_decision_nav2}.py`、`Dockerfile` | `backend/srm27_behavior/` |

## 4. 代码层面的整合

| 改动 | 原因 |
|---|---|
| 共享状态/算法抽成 header-only `srm27_behavior_core` | 服务端与插件都要用，避免任何一方反向依赖另一方 |
| 插件头文件迁到 `include/srm27_behavior_plugins/{action,condition,control,decorator}` | 插件包拥有自己的公开头；服务端不再编译插件源码 |
| 姿态用例从 core 测试移到插件包 | 该用例需要真实 `SetSentryPosture` BT 节点；core 测试只保留无 BT 节点的算法/状态用例 |
| 插件库仍按插件单独编 `.so`（不合并） | 实测合并后 `BT_RegisterRosNodeFromPlugin` 与 `convertFromString` 特化在多个 TU 中重复定义；按插件分库也便于单独替换 |
| C++ 命名空间统一保留 `srm27_behavior` | 三个包同属一个决策模块；避免改动所有类名/测试，降低回归面 |
| 决策树 XML、`models.xml`、`Project.btproj` 留在后端 | 它们是服务端的策略资产，路径不变 |

## 5. 装配与路径更新

- `params/sentry_behavior.yaml`：`plugins: [srm27_behavior/bt_plugins]` →
  `plugins: [srm27_behavior_plugins/bt_plugins]`（服务端仍用同一个参数键）；
- `srm27_navigation/package.xml` 增加 `srm27_behavior_core`、`srm27_behavior_plugins`
  的 `exec_depend`；
- `scripts/validate_decision.sh`：`ROOT` 多上一级（包下沉到 `backend/`），
  构建显式带上 `srm27_behavior_plugins`，ctest 分别指向
  `build/srm27_behavior_core`（`test_decision_core`）与
  `build/srm27_behavior_plugins`（`test_set_sentry_posture`）；
- `test/test_decision_ros.py`、README、`tests/README.md` 的源码路径同步到
  `src/decision/backend/srm27_behavior/...`；
- 启动脚本、`kill_nav.sh`、`bringup` 测试只依赖**包名** `srm27_behavior`
  与节点名，未改动。

## 6. 验证

```bash
colcon build --packages-select srm27_behavior_core srm27_behavior_plugins srm27_behavior \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  --parallel-workers 2
colcon test --packages-select srm27_behavior_core srm27_behavior_plugins \
  --event-handlers console_direct+
```

| 项 | 结果 |
|---|---|
| 构建 | `srm27_behavior_core`、`srm27_behavior_plugins`、`srm27_behavior` 3/3 成功；插件包 22 个 `.so` 全部安装到 `share/srm27_behavior_plugins/bt_plugins` |
| `srm27_behavior_core` gtest | `test_decision_core` **9/9**：到点/离点滞回与坐标系切换、内联/YAML/CSV 航点拒绝、导航完成后按停车点判到点、`SpinReference` 平滑/限幅/可复现/无固定周期 |
| `srm27_behavior_plugins` gtest | `test_set_sentry_posture` **1/1**：姿态 4 秒冷却、累计时间预算、回退姿态 |
| 决策 ROS 集成回归 | `test_decision_ros.py` **30/30 passed**（真实服务端 + 真实 mux + Action 抢占/断流/停机场景；约 159 s） |

> 本机 `pytest` 的 `launch_testing` 插件与新版 `pluggy` 不兼容（仓库既有环境问题），
> 运行集成回归时用了 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`；与本次拆包无关。
> `test_decision_nav2.py`（真实 Nav2/MINCO 闭环）本轮未跑，需 Gazebo 环境。

> 拆包后第一次构建 `srm27_behavior` 会命中 CMake 缓存指向旧源码路径的错误
> （`CMakeLists.txt does not match the source ... used to generate cache`）。
> 这是目录迁移的已知现象：删掉 `build/srm27_behavior`、`install/srm27_behavior`
> 后重编即可。

> `srm27_behavior_core` / `srm27_behavior_plugins` 的构建日志会提示
> “headers install destination is set to `include`”。这是 Humble 的
> `ament_auto_package` 默认行为（与拆包前一致），不是错误；包内 include 路径
> `srm27_behavior_core/...`、`srm27_behavior_plugins/...` 依赖这个非 scoped 布局。

现场验证入口不变：

```bash
./src/decision/backend/srm27_behavior/scripts/validate_decision.sh --nav2
```

## 7. 兼容与遗留

- 启动入口、Action 名（`srm27_behavior`）、节点名（`srm27_behavior_server` /
  `srm27_behavior_client`）不变；BT 节点注册名不变，决策树 XML 不需要改；
- 插件库安装目录变为 `share/srm27_behavior_plugins/bt_plugins`，仓库外若有
  直接写 `srm27_behavior/bt_plugins` 的自定义参数文件需同步；
- 未在本轮复测实车与完整 Nav2 闭环；决策逻辑本身未改；
- 导航侧的同名近似包 `srm27_nav_behavior_plugins`（Nav2 行为/目标检查器）与
  决策侧 `srm27_behavior_plugins` 是**两个不同包**，注意区分。
