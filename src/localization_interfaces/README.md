# Localization Interfaces · 定位消息与服务

![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros)
![ROSIDL](https://img.shields.io/badge/Interfaces-ROSIDL-287A6B)
![Messages and Services](https://img.shields.io/badge/Protocol-Messages%20%2B%20Services-536A8A)

`localization_interfaces` 定义 [Z-Localization](../../README.md) 前后端共享的观测、预测、会话通知和重置协议，
并提供重置事务缓存辅助类。消息与服务通过 ROSIDL 生成，供 [frontend](../frontend/README.md)
和 [backend](../backend/README.md) 使用。

[项目结构](#项目结构) · [环境与依赖](#环境与依赖) · [对外接口](#对外接口) · [重置事务缓存](#重置事务缓存)

## 项目结构

```text
src/localization_interfaces/
├── README.md
├── msg/
│   ├── FrontendFrame.msg              同步后验观测
│   ├── FrontendOdometry.msg           高频预测
│   └── FrontendSession.msg            会话通知
├── srv/
│   └── ResetFrontend.srv              前端重置请求与响应
├── include/localization_interfaces/
│   └── reset_transaction_cache.hpp    重置事务缓存
├── test/                              事务与消息序列化测试
├── CMakeLists.txt
├── package.xml
└── LICENSE
```

## 环境与依赖

| 类别 | 依赖 |
| --- | --- |
| 系统环境 | Ubuntu 22.04、ROS 2 Humble |
| 构建 | CMake ≥ 3.8、ament_cmake、rosidl_default_generators |
| 消息与运行库 | builtin_interfaces、geometry_msgs、nav_msgs、sensor_msgs、rosidl_default_runtime |
| 测试 | ament_cmake_gtest、ament_cmake_pytest、rclpy |

## 对外接口

以下为本工作区中的默认话题与服务名称，发布和服务处理由前端节点负责。

| 类型 | 默认话题 / 服务 | 内容 |
| --- | --- | --- |
| `localization_interfaces/msg/FrontendFrame` | `/frontend/frame` | 同一观测端点的后验、去畸变点云与更新状态 |
| `localization_interfaces/msg/FrontendOdometry` | `/frontend/odometry` | 带会话身份的高频里程计预测 |
| `localization_interfaces/msg/FrontendSession` | `/frontend/session` | 前端进程与重置代次通知 |
| `localization_interfaces/srv/ResetFrontend` | `/zlio/reset_internal` | 协调前端重置并返回执行结果及坐标边界 |

### FrontendFrame

一次已提交的前端观测，定义见 [FrontendFrame.msg](msg/FrontendFrame.msg)。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `instance_id` | `uint64` | 前端进程标识，进程重启后更换 |
| `epoch` | `uint64` | 当前进程内的重置代次 |
| `sequence` | `uint64` | 当前代次的观测序号，从 `1` 开始递增 |
| `begin` | `builtin_interfaces/Time` | 本次新增观测区间的开始时间 |
| `odometry` | `nav_msgs/Odometry` | 观测端点的后验位姿、速度及协方差 |
| `cloud` | `sensor_msgs/PointCloud2` | 去畸变至观测端点的点云；时间与 `odometry.header.stamp` 相同，坐标系为 `odometry.child_frame_id` |
| `matches` | `uint32` | 最后一次观测线性化的有效点面匹配数 |
| `lidar_updated` | `bool` | `true` 表示激光更新被接受；`false` 表示本次输出采用传播结果 |

`odometry.header.frame_id` 为位姿参考坐标系，默认 `odom`；`child_frame_id` 为 IMU 坐标系，默认 `imu`。
同一消息中的位姿与点云共同构成一次观测，接收方按 `instance_id`、`epoch` 和 `sequence` 判断会话及观测顺序。

### FrontendOdometry

随 IMU 更新的高频预测，定义见 [FrontendOdometry.msg](msg/FrontendOdometry.msg)。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `instance_id` | `uint64` | 前端进程标识 |
| `epoch` | `uint64` | 当前重置代次 |
| `odometry` | `nav_msgs/Odometry` | 传播至最新可用 IMU 时刻的预测位姿、速度及协方差 |

### FrontendSession

前端在进程启动及每次重置后发布会话通知，定义见 [FrontendSession.msg](msg/FrontendSession.msg)。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `instance_id` | `uint64` | 前端进程标识 |
| `epoch` | `uint64` | 当前重置代次 |

会话通知在估计初始化完成前即可发布。前端采用 reliable、transient local、深度 `1` 的 QoS，
使匹配持久性配置的后加入订阅者能够获取最近一次会话通知。

### ResetFrontend

前端重置的内部服务，定义见 [ResetFrontend.srv](srv/ResetFrontend.srv)。

#### 请求

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `requester_instance_id` | `uint64` | 请求者进程标识，非零；请求者重启后更换 |
| `transaction_id` | `uint64` | 请求者内的重置事务号，非零；同一事务重试沿用原值 |
| `target_epoch` | `uint64` | 要求前端切换到的目标代次，须大于前端当前代次 |

`requester_instance_id` 与 `transaction_id` 共同标识一次事务。同一事务的 `target_epoch` 保持一致。

#### 响应

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `outcome` | `uint8` | 重置结果，取值见下表 |
| `message` | `string` | 执行结果说明 |
| `requester_instance_id` | `uint64` | 请求者标识回显 |
| `transaction_id` | `uint64` | 事务号回显 |
| `target_epoch` | `uint64` | 请求目标代次回显 |
| `frontend_instance_id` | `uint64` | 实际响应的前端进程标识 |
| `reset_generation` | `uint64` | 前端进程内的重置执行计数；每个 `COMPLETED` 或 `INDETERMINATE` 事务递增一次 |
| `has_boundary` | `bool` | `true` 表示成功暖重置提供了有效坐标边界；`false` 表示仅使用执行结果及身份字段 |
| `source_epoch` | `uint64` | 重置前的前端代次，随有效坐标边界返回 |
| `boundary_stamp` | `builtin_interfaces/Time` | 重置前后位姿共同对应的后验时间 |
| `old_from_new` | `geometry_msgs/Transform` | 将新 odom 系坐标转换到旧 odom 系的刚体变换 |

| 结果 | 值 | 含义 |
| --- | --- | --- |
| `REJECTED` | `0` | 在操作估计器状态前拒绝请求，保持原会话状态 |
| `COMPLETED` | `1` | 重置完成，估计器和输入队列已切换至目标代次 |
| `INDETERMINATE` | `2` | 已接受操作且可能改变状态，完成情况不确定；接收方按已跨越代次边界处理 |
| `IN_PROGRESS` | `3` | 同一事务仍在执行，后续查询或重试继续使用原事务标识 |

当 `has_boundary=true` 时，接收方结合 `source_epoch`、`boundary_stamp` 与 `old_from_new`
接续新旧里程计坐标系。

## 重置事务缓存

[ResetTransactionCache](include/localization_interfaces/reset_transaction_cache.hpp) 提供请求去重、
在途事务冲突检查和终态响应复用，默认缓存容量为 `32`。

| 接口 | 含义 |
| --- | --- |
| `begin()` | 登记请求；返回 `Execute` 时执行重置，返回 `Responded` 时直接使用已生成的响应 |
| `finish()` | 保存终态响应并更新执行计数；完整响应重载同时保存成功暖重置的坐标边界 |
| `frontendInstanceId()` | 获取缓存所属的前端进程标识 |

缓存有效期间，相同事务复用完整响应，包括暖重置坐标边界；容量不足时淘汰最早登记的已完成事务。
相关测试位于 [test/](test)，覆盖重复请求、在途冲突、执行计数、边界复用及消息序列化。

## 维护者

mozihe · [zhujunheng2005@gmail.com](mailto:zhujunheng2005@gmail.com)

## 许可证

见 [LICENSE](LICENSE)。
