# srm27_nav_common

轨迹规划前端（`srm27_nav_planner`）与后端（`srm27_nav_smoother`）共用的**路径工具包**。
它**不注册任何 Nav2 插件**、**不含第三方源码**，只提供两边都要用的路径处理。

> 2026-11-24 两次拆分后本包只剩路径工具：
> - 地图/栅格 → `src/planner/map/`（[`srm27_map_core`](../../map/srm27_map_core/README.md)、
>   [`srm27_map_adapter`](../../map/srm27_map_adapter/README.md)）；
> - 上游 YAstar 源码 → [`srm27_nav_planner`](../../frontend/srm27_nav_planner/vendor)；
> - 上游 MinimumSnap/SfcSquare 源码 → [`srm27_nav_smoother`](../../backend/srm27_nav_smoother/vendor)。

## 内容

| 路径 | 说明 |
|---|---|
| `include/srm27_nav_common/path_utils.hpp` | 按最大段长线性加密 `resamplePath`、折角二次贝塞尔 `roundCorners` |
| `include/srm27_nav_common/path_conversion.hpp` | `nav_msgs/Path` ↔ `Eigen::Vector2f` 点列，`identity/tangent` 朝向语义 |
| `test/` | `test_path_utils`（加密/切角）、`test_path_conversion`（朝向与 header） |

全部为本工作区 Apache-2.0 代码，只依赖 Eigen 与 `nav_msgs`/`geometry_msgs`。
历史调参记录见
[docs/archive/2026-11-24-tdt-nav-kit-README.md](../../../../docs/archive/2026-11-24-tdt-nav-kit-README.md)。

## 构建与测试

```bash
./scripts/build/build_safe.sh --packages-select srm27_nav_common \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
colcon test --packages-select srm27_nav_common --event-handlers console_direct+
```

（`build_safe.sh` 把编译并发限制在 `-j2`/单包，见仓库根 `编译注意事项.md`。）
