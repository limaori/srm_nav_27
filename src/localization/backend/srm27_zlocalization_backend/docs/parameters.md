# 参数与调参建议

[返回 README](../README.md) · [参数列表](#参数列表) · [离线工具参数](#离线工具参数) · [调参建议](#调参建议)

## 参数列表

### YAML 参数

运行配置见 [backend.yaml](../../../config/backend.yaml)。完整参数名为“小节名.键名”，如 `common.frontend`。
默认值以仓库 YAML 为准；其中未列出的参数使用节点默认值。删减 YAML 或直接运行节点时，默认值可能不同。
数值须有限。参数在启动时读取，修改后需重启。

| 键 | 类型 | 默认值 | 含义 |
| --- | --- | --- | --- |
| `extends` | 路径 | 未设置 | 继承另一个 YAML；相对路径从当前文件所在目录算起，同名参数以当前文件为准，由 launch 解析 |

#### `common` · 坐标与输入（16 项）

| 键 | 类型 / 单位 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `frontend` | 字符串 | `zlio` | 前端接口：`zlio` 接收打包的观测、预测和会话消息；`legacy` 分别接收里程计与点云 |
| `map_frame` | 字符串 | `map` | 全局地图坐标系名称，也是全局位姿输出的参考系 |
| `odom_frame` | 字符串 | `odom` | 前端里程计父坐标系名称，须与输入消息一致 |
| `sensor_frame` | 字符串 | `imu` | 前端位姿所描述的传感器坐标系，须与里程计子坐标系一致 |
| `frame_topic` | 字符串 | `/frontend/frame` | ZLIO 同步观测话题，一条消息包含位姿和局部去畸变点云 |
| `prediction_topic` | 字符串 | `/frontend/odometry` | ZLIO 高频里程计预测话题 |
| `session_topic` | 字符串 | `/frontend/session` | ZLIO 会话通知话题，用于识别进程、重置代次和输入故障 |
| `odom_topic` | 字符串 | `/Odometry` | `legacy` 高频里程计话题，用于输出实时位姿 |
| `sync_odom_topic` | 字符串 | `/Odometry_scan_end` | `legacy` 扫描末端里程计话题，用于与点云配对 |
| `cloud_topic` | 字符串 | `/cloud_registered` | `legacy` 去畸变点云输入话题 |
| `input_cloud_in_local_frame` | 布尔 | `true` | 输入点云是否在传感器系；关闭时按 odom 系读取并转回传感器系，ZLIO 必须开启 |
| `odom_cloud_sync.subscription_queue_depth` | 整数，消息 | `50` | 同步输入订阅最多缓存的消息数，也用于 ZLIO 观测订阅；须 `≥1` |
| `odom_cloud_sync.synchronizer_queue_depth` | 整数，组 | `20` | `legacy` 时间同步器最多缓存的待配对消息数；须 `≥1` |
| `odom_cloud_sync.max_interval` | 浮点，s | `0.05` | `legacy` 里程计与点云允许的最大时间戳差；须 `≥0.001` |
| `maintenance_period` | 浮点，s | `0.2` | 检查重置事务、调度跟踪与恢复任务的周期；须 `≥0.01` |
| `descriptor_type` | 字符串 | `scan_context` | 地点识别方法：`scan_context` 高度栅格，`gprobe` 概率占据；须与地图包中的描述子类型一致 |

#### `common.body` · 车体外参（7 项）

| 键 | 类型 / 单位 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `frame` | 字符串 | `body` | 车体坐标系名称，用于车体位姿和 TF 输出 |
| `rotation.roll_deg` | 浮点，° | `-14.94` | 将传感器坐标转到车体系的横滚角；按 Z-Y-X 欧拉角组合旋转 |
| `rotation.pitch_deg` | 浮点，° | `0.523` | 将传感器坐标转到车体系的俯仰角 |
| `rotation.yaw_deg` | 浮点，° | `-150.6` | 将传感器坐标转到车体系的偏航角 |
| `offset.x` | 浮点，m | `-0.098` | 从传感器原点到车体原点的位移，在车体系中的 X 分量 |
| `offset.y` | 浮点，m | `0.094` | 从传感器原点到车体原点的位移，在车体系中的 Y 分量 |
| `offset.z` | 浮点，m | `0.0` | 从传感器原点到车体原点的位移，在车体系中的 Z 分量 |

#### `common.scan_context` · Scan Context（14 项）

地点描述子是用于识别重访位置的点云摘要。本组仅在 `common.descriptor_type=scan_context` 时使用。
配置中只保留所选描述子的参数组；未知参数或多余的描述子参数组会导致启动失败。

| 键 | 类型 / 单位 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `enabled` | 布尔 | `true` | 是否生成和检索 Scan Context 地点描述子 |
| `num_rings` | 整数，环 | `30` | 将水平距离分成多少个同心环；越多径向划分越细，须 `≥1` |
| `num_sectors` | 整数，扇区 | `90` | 将水平一周分成多少个扇区；越多角度划分越细，须 `≥1` |
| `max_radius` | 浮点，m | `80.0` | 构建描述子时使用的最大水平点距；须 `≥1` |
| `downsample` | 浮点，m | `0.6` | 描述子输入点云的降采样体素边长；须 `≥0`，`0` 不降采样 |
| `lidar_height` | 浮点，m | `0.35` | 构建高度栅格前加到每个点 Z 坐标上的偏移，通常填安装高度 |
| `min_bin_points` | 整数，点 | `1` | 一个栅格至少包含多少点才视为有效；须 `≥1` |
| `augment_count` | 整数，份 | `3` | 每个关键帧生成的描述子总数，含原视点和横向平移视点；范围 `[1,7]`，`1` 不增强 |
| `lateral_shift` | 浮点，m | `2.0` | 生成增强描述子时模拟的横向移动距离；须 `≥0`，`0` 不移动视点 |
| `search_ratio` | 浮点，无量纲 | `1.0` | 精匹配的航向搜索范围占整周的比例；范围 `[0,1]`，`0` 只检查粗估航向，`1` 搜索整周 |
| `exclude_recent` | 整数，帧 | `15` | 回环检索时跳过最近多少个关键帧；须 `≥1` |
| `candidates` | 整数，候选 | `15` | 描述子检索保留的候选数上限；须 `≥1` |
| `distance_threshold` | 浮点，无量纲 | `0.8` | 描述子差异的接受上限，越小越严格；须 `≥0`，`0` 仅接受零差异 |
| `level_with_odom_roll_pitch` | 布尔 | `true` | 是否按里程计横滚、俯仰角将点云转为水平，再计算描述子 |

#### `common.gprobe` · G-PROBE（17 项）

仅在 `common.descriptor_type=gprobe` 时使用。以下为节点默认值，离线工具的部分默认值不同。

| 键 | 类型 / 单位 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `enabled` | 布尔 | `true` | 是否生成和检索 G-PROBE 地点描述子 |
| `level_with_odom_roll_pitch` | 布尔 | `true` | 是否按里程计横滚、俯仰角将点云转为水平，再计算描述子 |
| `num_rings` | 整数，环 | `40` | 将水平距离分成多少个同心环；范围 `[1,512]` |
| `num_sectors` | 整数，扇区 | `60` | 将水平一周分成多少个扇区；范围 `[4,720]` |
| `max_range` | 浮点，m | `80.0` | 构建描述子时使用的最大水平点距；须 `≥1` |
| `voxel_size` | 浮点，m | `0.5` | 描述子输入点云的降采样体素边长；须 `≥0`，`0` 不降采样 |
| `sigma_t` | 浮点，m | `2.0` | 为容忍视点平移而平滑占据栅格的距离尺度；须 `≥0`，`0` 仍保留最小角向平滑 |
| `query_fov_deg` | 浮点，° | `360.0` | 查询点云参与匹配的水平视场宽度；范围 `[1,360]`，`360` 表示整周 |
| `query_fov_center_deg` | 浮点，° | `0.0` | 查询视场的中心方向，以局部 X 轴正向为 `0°` |
| `branch_top_k` | 整数，候选 | `15` | 每个检索分支先保留的候选数；须 `≥1` |
| `verification_top_k` | 整数，候选 | `60` | 多分支检索后进入描述子精匹配的候选数上限；须 `≥1` |
| `spatial_nms_radius` | 浮点，m | `2.0` | 在此半径内优先保留得分更高的候选，减少位置重复；须 `≥0`，`0` 关闭 |
| `heading_window_deg` | 浮点，° | `50.0` | 在候选航向两侧各搜索的角度范围；范围 `[0,180]`，`0` 只检查该航向 |
| `column_trim_ratio` | 浮点，无量纲 | `0.1` | 计算匹配得分时剔除差异最大扇区的比例；范围 `[0,0.95]`，`0` 不剔除 |
| `sgrt_kappa` | 浮点，无量纲 | `10.0` | 航向得分的集中系数；越大越偏重得分占优的航向，须 `≥0` |
| `exclude_recent` | 整数，帧 | `15` | 回环检索时跳过最近多少个关键帧；须 `≥1` |
| `candidates` | 整数，候选 | `30` | 描述子检索最终返回的候选数上限；须 `≥1` |

#### `prior_map` · 先验地图（4 项）

| 键 | 类型 / 单位 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `enabled` | 布尔 | `false` | 是否加载固定先验地图定位；可与运行回环和 UWB 共存；先验模式关闭退出保存和运行地图显示 |
| `initialization_only` | 布尔 | `false` | 先验只在初始化时匹配一次：首次验收的先验全局位姿建立 `map->odom` 后锁存，跟踪和恢复都不再产生先验观测，全局位姿完全由前端里程计传播（入口 `--zloc-prior-once`）；需 `enabled`，无先验地图时由配置解析自动关闭并记录 override |
| `directory` | 路径 | `pcd/rmuc` | 先验关键帧地图包目录；启用先验时必填，相对路径从运行目录算起 |
| `voxel_resolution` | 浮点，m | `0.15` | 先验地图显示点云的体素边长，不影响配准；须 `≥0`，`0` 不降采样 |

#### `trajectory` · 运行地图与关键帧（8 项）

| 键 | 类型 / 单位 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `save_on_shutdown` | 布尔 | `false` | 是否在正常退出时保存运行地图；关闭时仍可调用保存服务，先验定位时不保存 |
| `save_directory` | 路径 | `pcd/runtime_zlio` | 建图结果的保存目录，相对路径从运行目录算起 |
| `save_global_pcd` | 布尔 | `true` | 保存地图包时是否额外生成全局 `map.pcd` |
| `map_voxel_resolution` | 浮点，m | `0.15` | 运行地图显示及全局 PCD 导出的体素边长；须 `≥0`，`0` 不降采样 |
| `keyframe.distance` | 浮点，m | `-1.0` | 相对上一关键帧移动超过此距离时插入关键帧；`≤0` 关闭此触发条件 |
| `keyframe.angle_deg` | 浮点，° | `-1.0` | 相对上一关键帧旋转超过此角度时插入关键帧；`≤0` 关闭此触发条件 |
| `keyframe.min_interval` | 浮点，s | `0.0` | 同一重置代次内两关键帧的最短时间间隔；须 `≥0`，`0` 不限制插入频率 |
| `keyframe.max_interval` | 浮点，s | `0.5` | 距上一关键帧达到此时长即触发插入；`≤0` 关闭定时触发，启用时不得小于 `min_interval` |

达到 `min_interval` 后，平移、旋转或时间条件满足任意一项即可插入关键帧。
若三个条件都关闭，代码会将定时间隔恢复为 `max(1 s, min_interval)`。

#### `registration` · 点到面配准（17 项）

| 键 | 类型 / 单位 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `voxel_resolution` | 浮点，m | `0.4` | 配准点云的降采样体素边长；越小保留细节越多，须 `≥0`，`0` 不降采样 |
| `max_iterations` | 整数，次 | `16` | 每个粗、细配准层的最大迭代次数；须 `≥1` |
| `max_correspondence_distance` | 浮点，m | `1.0` | 源点到目标近邻点的最大距离，超过则不建立点面对应；须 `≥0.001` |
| `plane_knn` | 整数，点 | `8` | 为每个目标点拟合平面时查找的近邻点数；须 `≥3` |
| `max_plane_eigenvalue_ratio` | 浮点，无量纲 | `0.2` | 拟面点集最小 / 最大特征值的上限；越小要求越平整，范围 `[0,1]` |
| `min_plane_middle_eigenvalue_ratio` | 浮点，无量纲 | `0.08` | 拟面点集中间 / 最大特征值的下限，用于排除细长线状结构；范围 `[0,1]` |
| `max_plane_neighbor_distance` | 浮点，m | `1.2` | 拟面近邻中最远点到查询点的距离上限；`≤0` 不限制 |
| `min_correspondences` | 整数，点 | `80` | 几何检查要求的最少有效点面对应数；须 `≥1` |
| `min_correspondence_ratio` | 浮点，无量纲 | `0.5` | 有效点面对应数占源点数的最低比例；范围 `[0,1]` |
| `coarse_trigger_min_correspondence_ratio` | 浮点，无量纲 | `0.35` | 初始细配准的对应比例低于此值时，允许尝试粗配准；范围 `[0,min_correspondence_ratio]` |
| `max_rmse` | 浮点，m | `0.25` | 恢复、回环和标定允许的最大点面均方根误差；须 `≥0`，`0` 关闭此检查 |
| `huber_delta` | 浮点，m | `0.25` | 点面残差超过此值后降低其权重；须 `≥0`，`0` 不降权 |
| `residual_std_floor` | 浮点，m | `0.03` | 配准残差噪声标准差的下限，避免残差过小时给出过高置信度；须 `≥0.0001` |
| `measurement_translation_std_floor` | 浮点，m | `0.005` | 整次配准平移观测的标准差下限，避免因点数多而低估误差；须 `≥0`，`0` 关闭 |
| `measurement_rotation_std_floor_deg` | 浮点，° | `0.02` | 整次配准旋转观测的标准差下限，避免因点数多而低估误差；须 `≥0`，`0` 关闭 |
| `min_coverage_sectors` | 整数，扇区 | `6` | 有效对应至少覆盖多少个水平扇区，共 36 区；范围 `[0,36]`，`0` 不检查覆盖数 |
| `degeneracy_condition_threshold` | 浮点，无量纲 | `10.0` | 判断方向约束强弱差异的条件数阈值，也用于稳定求解；须 `>1`，不直接改变图因子权重 |

#### `tracking` · 常规跟踪（14 项）

| 键 | 类型 / 单位 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `correction_period` | 浮点，s | `0.5` | 两次常规全局观测校正的最小间隔；须 `≥0`，实际至少 `0.05 s`，并受维护周期限制 |
| `target_keyframes` | 整数，帧 | `11` | 拼接目标子图时最多使用的邻近先验关键帧数；须 `≥1` |
| `target_radius` | 浮点，m | `8.0` | 围绕参考关键帧选取目标子图的半径；须 `≥0`，`0` 不限半径 |
| `nearest_reference_radius` | 浮点，m | `7.0` | 当前预测位置到先验参考关键帧的最大距离；须 `≥0`，`0` 不限距离 |
| `target_hysteresis_distance` | 浮点，m | `0.5` | 新参考帧比旧参考帧至少近多少才切换；须 `≥0`，`0` 始终选最近帧 |
| `source.min_frames` | 整数，帧 | `1` | 至少累积多少帧点云才进行匹配；须在 `[1,source.max_frames]` |
| `source.max_frames` | 整数，帧 | `5` | 一次匹配最多累积的同步观测帧数，含当前帧；须 `≥1` |
| `source.max_time` | 浮点，s | `0.5` | 参与累积的历史帧与当前帧的最大时间差；须 `≥0`，`0` 不限 |
| `source.max_translation` | 浮点，m | `1.5` | 参与累积的历史帧相对当前帧的最大位移；须 `≥0`，`0` 不限 |
| `source.max_rotation_deg` | 浮点，° | `0.0` | 参与累积的历史帧相对当前帧的最大转角；须 `≥0`，`0` 不限 |
| `max_translation_correction` | 浮点，m | `1.5` | 匹配结果相对预测位姿的最大平移修正；超出视为冲突，须 `≥0`，`0` 不检查 |
| `max_rotation_correction_deg` | 浮点，° | `15.0` | 匹配结果相对预测位姿的最大旋转修正；超出视为冲突，须 `≥0`，`0` 不检查 |
| `suspicious_linear_speed` | 浮点，m/s | `10.0` | 线速度超过此值时进入 `SUSPECT`，仅有先验或 UWB 时生效；须 `≥0`，`0` 对任意非零速度触发 |
| `suspicious_angular_speed` | 浮点，rad/s | `30.0` | 角速度超过此值时进入 `SUSPECT`，仅有先验或 UWB 时生效；须 `≥0`，`0` 对任意非零角速度触发 |

#### `innovation` · 观测一致性（2 项）

| 键 | 类型 / 单位 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `rejection_probability` | 浮点，无量纲 | `0.001` | 观测与预测的卡方一致性检验 p 值低于此值时判为冲突；越大越严格，范围 `[1e-12,0.5]` |
| `conflicts_before_suspect` | 整数，次 | `7` | 所有可用全局观测均冲突，连续达到多少次后进入 `SUSPECT`；须 `≥1` |

#### `recovery` · 初始化与重定位（21 项）

| 键 | 类型 / 单位 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `period` | 浮点，s | `0.2` | 两次重定位尝试的最小间隔；须 `≥0`，实际至少 `0.05 s`，且需要新同步帧 |
| `suspect_search_radius` | 浮点，m | `30.0` | `SUSPECT` 时围绕最后可信位置搜索候选的半径；须 `≥0`，`0` 不限半径 |
| `lost_search_radius` | 浮点，m | `35.0` | `LOST` 时围绕预测位置搜索候选的半径；须 `≥0`，`0` 不限半径 |
| `global_fallback_candidates` | 整数，候选 | `5` | 局部搜索失败后补充的全局描述子候选数；须 `≥0`，`0` 关闭全局补充 |
| `suspect_failures_before_lost` | 整数，次 | `3` | `SUSPECT` 下连续重定位失败多少次后转为 `LOST`；须 `≥1` |
| `automatic_when_lost` | 布尔 | `true` | 丢失后是否自动重定位；关闭则等待重定位服务请求 |
| `max_translation_correction` | 浮点，m | `2.5` | ★本工作区新增。自动恢复提交相对**最后可信位姿**允许的最大平移；须 `≥0`，`0` 关闭。已有可信位姿时生效，冷启动（`INITIALIZING`）不设限；越过预算的恢复不提交、保持 `tf` 冻结，并按一次恢复失败累计（`suspect_failures_before_lost`） |
| `max_rotation_correction_deg` | 浮点，° | `20.0` | ★本工作区新增。同上，允许的最大旋转；须 `≥0`，`0` 关闭 |
| `manual_bypasses_correction_budget` | 布尔 | `true` | ★本工作区新增。人工下发 `/backend_node/relocalize` 是否绕过上面两项预算；`true` 时越过预算仍提交，但打 WARN 并打印实际幅度 |
| `source.min_frames` | 整数，帧 | `2` | 至少累积多少帧点云才尝试重定位；须在 `[1,source.max_frames]` |
| `source.max_frames` | 整数，帧 | `5` | 重定位最多累积的同步观测帧数，含当前帧；须 `≥1` |
| `source.max_time` | 浮点，s | `0.5` | 参与累积的历史帧与当前帧的最大时间差；须 `≥0`，`0` 不限 |
| `source.max_translation` | 浮点，m | `1.5` | 参与累积的历史帧相对当前帧的最大位移；须 `≥0`，`0` 不限 |
| `source.max_rotation_deg` | 浮点，° | `0.0` | 参与累积的历史帧相对当前帧的最大转角；须 `≥0`，`0` 不限 |
| `initial_max_descriptor_distance` | 浮点，无量纲 | `0.8` | 初始定位时允许的最大描述子差异；越小越严格，须 `≥0`，`0` 关闭此筛选 |
| `descriptor_distance_limit` | 浮点，无量纲 | `0.8` | `SUSPECT` / `LOST` 恢复时允许的最大描述子差异；须 `≥0`，`0` 关闭此筛选 |
| `yaw_sweep_count` | 整数，方向 | `6` | 需要搜索航向时，在整周均匀尝试的方向数；须 `≥1` |
| `parallel_candidates` | 整数，工作线程 | `16` | 同时验证恢复候选的工作线程数上限；须 `≥0`，`0` 按硬件线程数选择 |
| `max_candidates` | 整数，候选 | `15` | 每批最多进入几何验证的候选数；须 `≥1` |
| `target_keyframes` | 整数，帧 | `11` | 为一个恢复候选拼接目标子图时，最多使用的先验关键帧数；须 `≥1` |
| `target_radius` | 浮点，m | `8.0` | 围绕恢复候选关键帧拼接目标子图的半径；须 `≥0`，`0` 不限半径 |

#### `uwb` · 输入与时空标定（23 项）

| 键 | 类型 / 单位 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `enabled` | 布尔 | `false` | 是否订阅并融合 UWB 位置；导航入口由 `--zloc-uwb` 开启 |
| `use_z` | 布尔 | `false` | 是否使用 UWB 高度；关闭时输入、标定和融合均只使用 XY |
| `topic` | 字符串 | `/uwb_pose` | UWB 原始天线位置话题，消息类型为 `PoseStamped` |
| `frame` | 字符串 | `uwb` | UWB 原始位置的坐标系，须非空且与消息 `frame_id` 一致；填 map 时须无坐标变换且关闭在线空间标定 |
| `estimate_zero_pose_online` | 布尔 | `true` | 是否用先验定位结果估计 UWB 到 map 的坐标变换；需启用 UWB 和先验地图，关闭则使用 `zero_pose` |
| `zero_pose.x` | 浮点，m | `0.0` | UWB 原点在 map 系中的 X 坐标；在线标定完成前也用于恢复初值 |
| `zero_pose.y` | 浮点，m | `0.0` | UWB 原点在 map 系中的 Y 坐标 |
| `zero_pose.z` | 浮点，m | `0.0` | UWB 原点在 map 系中的 Z 坐标 |
| `zero_pose.yaw_deg` | 浮点，° | `0.0` | 将 UWB 坐标旋转到 map 系的水平偏航角 |
| `effective_rate_hz` | 浮点，Hz | `1.0` | UWB 真实解算频率；一个周期内的相同位置重发不重复融合，须 `≥0`，`0` 关闭此过滤 |
| `time_offset_seconds` | 浮点，s | `-0.15` | 实际测量时刻 = 消息时间戳 + 此值；测量早于消息时间时填负数 |
| `estimate_time_offset_online` | 布尔 | `false` | 是否在线估计固定时间偏置；需启用 UWB 和先验地图，关闭则固定使用 `time_offset_seconds` |
| `time_offset_prior_std_seconds` | 浮点，s | `0.1` | 在线估计对初始时间偏置的不确定度；越小越信任配置值，须 `>0` |
| `time_jitter_std_seconds` | 浮点，s | `0.05` | 每次 UWB 测量时间的随机抖动标准差，用于折算位置噪声；须 `≥0`，`0` 不计此项 |
| `max_age` | 浮点，s | `0.25` | 将 UWB 用作恢复位置提示时，相对当前帧允许的最大滞后；须 `≥0`，按修正后的测量时间计算 |
| `odom_sync_tolerance` | 浮点，s | `0.25` | UWB 测量时刻到里程计插值端点允许的最大时间差；须 `≥0`，`0` 要求时间重合 |
| `recovery_search_radius` | 浮点，m | `2.0` | 围绕 UWB 推算位置搜索先验关键帧的半径；须 `≥0`，`0` 不限半径 |
| `recovery_candidates` | 整数，候选 | `8` | 围绕 UWB 位置最多保留的恢复候选数；须 `≥1` |
| `antenna_position_body.x` | 浮点，m | `-0.245` | UWB 天线相位中心在车体系中的 X 坐标 |
| `antenna_position_body.y` | 浮点，m | `-0.0` | UWB 天线相位中心在车体系中的 Y 坐标 |
| `antenna_position_body.z` | 浮点，m | `0.0` | UWB 天线相位中心在车体系中的 Z 坐标 |
| `position_std_xy` | 浮点，m | `0.15` | UWB 单次位置测量的 X、Y 标准差；越小融合权重越高，须 `>0` |
| `position_std_z` | 浮点，m | `0.3` | UWB 单次高度测量的标准差；仅 `use_z=true` 时使用，须 `>0` |

#### `uwb.bootstrap` · 启动对齐（10 项）

| 键 | 类型 / 单位 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `max_pairs` | 整数，组 | `120` | 对齐计算最多保留的 UWB 与里程计位置配对数；须 `≥2` |
| `min_pairs` | 整数，组 | `5` | 至少积累多少组位置配对才开始对齐；范围 `[2,max_pairs]` |
| `min_baseline` | 浮点，m | `3.0` | 两组轨迹各自至少需要的水平位置跨度，用于确定航向；须 `>0`，非累计路程 |
| `inlier_threshold` | 浮点，m | `0.75` | 对齐后两路位置相差不超过此值的配对才计为内点；须 `>0` |
| `max_rmse` | 浮点，m | `0.45` | 对齐内点的位置均方根误差上限；超过则不接受，须 `>0` |
| `max_yaw_std_deg` | 浮点，° | `10.0` | 对齐航向估计的标准差上限；超过则认为航向不够确定，须 `>0` |
| `stable_solutions` | 整数，次 | `2` | 连续多少次结果稳定后接受对齐；须 `≥1` |
| `stable_translation_delta` | 浮点，m | `0.25` | 相邻两次对齐结果允许的最大平移变化；须 `>0` |
| `stable_yaw_delta_deg` | 浮点，° | `3.0` | 相邻两次对齐结果允许的最大航向变化；须 `>0` |
| `gravity_alignment_std_deg` | 浮点，° | `1.0` | 对齐中横滚、俯仰的不确定度，也用于无先验地图时的重力方向约束；须 `>0` |

#### `loop` · 回环检索（13 项）

| 键 | 类型 / 单位 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `enabled` | 布尔 | `false` | 是否检测运行轨迹的历史回环；可与先验地图共存，关闭描述子时会自动关闭；开启后保留完整图和运行历史 |
| `cooldown_keyframes` | 整数，帧 | `5` | 成功回环后至少间隔多少个关键帧才再次检索；须 `≥0`，`0` 不等待 |
| `candidate_max_distance` | 浮点，m | `7.0` | 按当前估计位置筛选回环候选的最大距离；须 `≥0`，`0` 不限距离 |
| `descriptor_ratio_threshold` | 浮点，无量纲 | `1.0` | 最优 / 次优描述子差异的比值上限；越小越要求最优候选明显胜出，范围 `[0,1]`，`1` 关闭 |
| `source.min_frames` | 整数，帧 | `1` | 至少累积多少个关键帧点云才验证回环；须在 `[1,source.max_frames]` |
| `source.max_frames` | 整数，帧 | `4` | 回环源点云最多累积的关键帧数，含当前关键帧；须 `≥1` |
| `source.max_time` | 浮点，s | `0.0` | 参与累积的历史关键帧与当前帧的最大时间差；须 `≥0`，`0` 不限 |
| `source.max_translation` | 浮点，m | `0.0` | 参与累积的历史关键帧相对当前帧的最大位移；须 `≥0`，`0` 不限 |
| `source.max_rotation_deg` | 浮点，° | `0.0` | 参与累积的历史关键帧相对当前帧的最大转角；须 `≥0`，`0` 不限 |
| `target_keyframes` | 整数，帧 | `17` | 为回环候选拼接目标子图时最多使用的历史关键帧数；须 `≥1` |
| `target_radius` | 浮点，m | `12.0` | 围绕回环候选拼接目标子图的半径；须 `≥0`，`0` 不限半径 |
| `max_parallel_verifications` | 整数，工作线程 | `4` | 同时验证回环候选的工作线程数上限；须 `≥1` |
| `history_size` | 整数，条 | `200` | 一致性检查及可视化保留的回环数，不删除已入图因子；须 `≥0`，`0` 保留全部 |

#### `loop.validation` · 回环几何验收（6 项）

| 键 | 类型 / 单位 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `max_translation` | 浮点，m | `3.0` | 配准得到的两回环关键帧间距离上限；须 `≥0`，`0` 不检查 |
| `max_z` | 浮点，m | `0.5` | 回环测量与预测位姿的高度偏差上限；须 `≥0`，`0` 不检查 |
| `max_roll_pitch_deg` | 浮点，° | `10.0` | 回环测量与预测位姿的横滚、俯仰偏差上限；须 `≥0`，`0` 不检查 |
| `max_yaw_residual_deg` | 浮点，° | `60.0` | 配准航向与描述子提示航向的最大偏差；须 `≥0`，`0` 不检查 |
| `overlap_distance` | 浮点，m | `0.5` | 源点变换后距最近目标点不超过此值时计为重叠；须 `≥0` |
| `min_overlap_ratio` | 浮点，无量纲 | `0.45` | 配准后重叠源点占全部源点的最低比例；范围 `[0,1]`，`0` 不检查比例 |

#### `loop.pcm` · 回环一致性（5 项）

| 键 | 类型 / 单位 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `enabled` | 布尔 | `true` | 是否检查不同回环测量之间的一致性 |
| `translation_threshold` | 浮点，m | `1.0` | 两条回环与里程计组成闭合路径后，允许的最大平移误差；须 `≥0` |
| `rotation_threshold_deg` | 浮点，° | `15.0` | 两条回环与里程计组成闭合路径后，允许的最大旋转误差；须 `≥0` |
| `min_consistent_loops` | 整数，条 | `1` | 新回环至少要与多少条历史回环一致；须 `≥1`，没有可比较历史时跳过 |
| `min_candidate_clique_size` | 整数，条 | `1` | 同批候选中至少需要多少条两两一致的回环；须 `≥1` |

#### `graph` · 因子图（10 项）

| 键 | 类型 / 单位 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `prior_rotation_variance` | 浮点，rad² | `0.0001` | 初始位姿旋转先验的各轴方差；越小约束越强，须 `>0` |
| `prior_translation_variance` | 浮点，m² | `0.01` | 初始位姿位置先验的各轴方差；越小约束越强，须 `>0` |
| `odom_noise.rotation_variance_per_second` | 浮点，rad²/s | `1e-06` | 两关键帧间每经过一秒增加的旋转方差；须 `>0` |
| `odom_noise.translation_variance_per_second` | 浮点，m²/s | `0.0005` | 两关键帧间每经过一秒增加的平移方差；须 `>0` |
| `odom_noise.rotation_variance_per_radian` | 浮点，rad²/rad | `3e-07` | 两关键帧间每转动一弧度额外增加的旋转方差；须 `≥0`，`0` 不附加 |
| `odom_noise.translation_variance_per_meter` | 浮点，m²/m | `0.0002` | 两关键帧间每移动一米额外增加的平移方差；须 `≥0`，`0` 不附加 |
| `robust.enabled` | 布尔 | `true` | 是否降低大残差回环因子的权重 |
| `robust.kernel` | 字符串 | `cauchy` | 回环鲁棒核：`huber` 对大残差采用线性惩罚，`cauchy` 对大残差降权更强 |
| `robust.size` | 浮点，无量纲 | `0.5` | 以噪声标准差归一化后的残差尺度；越小越早降低回环权重，须 `>0` |
| `fixed_lag_seconds` | 浮点，s | `30.0` | 图中保留位姿变量的历史时长；须 `≥0`，`0` 保留全图，开启回环时自动取 `0` |

#### `reset` · 协调重置（5 项）

| 键 | 类型 / 单位 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `service` | 字符串 | `/zlio/reset_internal` | 后端调用的前端内部重置服务名 |
| `min_gap` | 浮点，s | `10.0` | 两次独立重置请求之间的最短间隔；须 `≥0`，`0` 不限制 |
| `response_timeout` | 浮点，s | `5.0` | 等待重置响应的超时时长；到期以同一事务 ID 重试，须 `≥0.05` |
| `stationary_translation_std` | 浮点，m | `0.05` | 无暖重置坐标边界、按停车假设衔接时增加的平移标准差；须 `>0` |
| `stationary_rotation_std_deg` | 浮点，° | `1.0` | 无暖重置坐标边界、按停车假设衔接时增加的旋转标准差；须 `>0` |

#### `pure_odometry` · 纯里程计原点（2 项）

| 键 | 类型 / 单位 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `origin_x` | 浮点，m | `0.0` | 纯里程计的 odom 原点在 map 系中的 X 坐标；仅无先验、UWB、回环和启动提示时生效 |
| `origin_y` | 浮点，m | `0.0` | 纯里程计的 odom 原点在 map 系中的 Y 坐标；仅无先验、UWB、回环和启动提示时生效 |

#### `startup_hint` · 初始位姿提示（6 项）

| 键 | 类型 / 单位 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `enabled` | 布尔 | `false` | 是否使用下列车体初始位姿；仅首次启动使用，前端重置后不再套用 |
| `force_until_matched` | 布尔 | `true` | 是否持续在提示位姿附近匹配直至成功，期间不尝试其他候选；仅提示与先验地图同时开启时生效 |
| `x` | 浮点，m | `4.6` | 启动时车体原点在 map 系中的 X 坐标 |
| `y` | 浮点，m | `7.5` | 启动时车体原点在 map 系中的 Y 坐标 |
| `z` | 浮点，m | `0.15` | 启动时车体原点在 map 系中的 Z 坐标 |
| `yaw_deg` | 浮点，° | `0.0` | 启动时车体在 map 系中的航向角 |

提示只给出**车体**位置与航向；传感器相对车体的 roll/pitch/yaw 由 backend 从启动时的实时
odom 姿态取（odom 出生系即车体系），因此云台转角、倾斜安装不需要写进配置——把提示当成
`T_map_sensor` 的 level 位姿会在倾斜安装下让匹配种子差一整个安装角。有先验地图 +
`force_until_matched: true` 时，backend 只在提示位置附近做几何验证，不进入全局 Scan Context
检索（`--zloc-start-hint` 就是这个用法）。

#### `visualization` · 可视化（7 项）

| 键 | 类型 / 单位 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `enabled` | 布尔 | `false` | 是否发布 UWB 位置和状态标记；其他可视化开关未填写时继承此值 |
| `publish_prior_map` | 布尔 | `false` | 是否发布先验地图显示点云；需启用先验地图 |
| `publish_runtime_map` | 布尔 | `false` | 是否发布当前建图结果的显示点云；先验定位时关闭 |
| `publish_map_axes` | 布尔 | `false` | 是否显示地图关键帧坐标轴 |
| `publish_paths` | 布尔 | `false` | 是否发布原始轨迹、优化轨迹及回环连线 |
| `publish_debug_registration` | 布尔 | `false` | 是否发布最近一次配准的对齐后源点云与目标点云 |
| `map_publish_period` | 浮点，s | `2.0` | 运行地图显示点云的发布周期，实际至少 `0.1 s` |

#### `services` · 服务名称（3 项）

| 键 | 类型 / 单位 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `save_map` | 字符串 | `~/save_map` | 保存运行地图的服务名 |
| `relocalize` | 字符串 | `~/relocalize` | 手动重定位服务名，仅在 `LOST` 状态接受请求 |
| `reset` | 字符串 | `/localization_reset` | 对外提供的前后端协调重置服务名 |

#### `output.legacy` · 兼容输出（13 项）

| 键 | 类型 / 单位 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `enabled` | 布尔 | `true` | 是否启用本组兼容话题及 map → body TF |
| `publish_pose` | 布尔 | `true` | 是否发布兼容车体位姿 |
| `publish_twist` | 布尔 | `true` | 是否发布兼容车体速度，速度在 map 系表达 |
| `publish_path` | 布尔 | `false` | 是否发布兼容车体轨迹 |
| `publish_tf` | 布尔 | `true` | 是否发布 map → body TF |
| `publish_initializing` | 布尔 | `false` | 是否允许初始化期间发布兼容位姿、速度和路径；TF 还需已有全局位姿 |
| `publish_untrusted` | 布尔 | `false` | 是否允许 `SUSPECT` / `LOST` 时发布兼容运动输出；发布时位姿冻结、速度清零 |
| `pose_topic` | 字符串 | `/tdt_lio_pose` | 兼容车体位姿话题名 |
| `twist_topic` | 字符串 | `/tdt_lio_twist` | 兼容车体速度话题名 |
| `path_topic` | 字符串 | `/tdt_lio_path` | 兼容车体轨迹话题名 |
| `danger_topic` | 字符串 | `/LAM_danger_situation` | 危险码话题名；可信状态发 `0`，其余发 `9` |
| `path_period` | 浮点，s | `0.1` | 兼容轨迹的最小发布间隔；`≤0` 每次有效更新都可发布 |
| `path_max_size` | 整数，位姿 | `2000` | 兼容轨迹最多保留的位姿数；`≤0` 不限长度 |

#### `output.world_cloud` · 世界点云输出（5 项）

| 键 | 类型 / 单位 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `enabled` | 布尔 | `true` | 是否将实时点云转换到 map 系并发布 |
| `publish_period` | 浮点，s | `0.0` | 实时世界点云的最小发布间隔；须 `≥0`，`0` 每个有效帧都可发布 |
| `publish_initializing` | 布尔 | `false` | 是否允许初始定位尚未完成时发布世界点云 |
| `publish_untrusted` | 布尔 | `false` | 是否允许 `SUSPECT` / `LOST` 时发布世界点云；此时使用最后可信位姿变换 |
| `topic` | 字符串 | `/localization/lio_world_cloud` | 实时世界点云的话题名 |

#### `storage` · 点云缓存（2 项）

| 键 | 类型 / 单位 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `cloud_cache_mb` | 整数，MiB | `64` | 从磁盘加载的关键帧点云缓存容量；须 `≥0`，`0` 用完即释放缓存引用 |
| `submap_cache_mb` | 整数，MiB | `64` | 拼接子图的缓存容量；须 `≥0`，`0` 用完即释放缓存引用 |

缓存容量不包含正在使用的点云，以及尚未保存的建图点云，因此不等于进程总内存上限。

### Launch 参数

#### `backend.launch.py`

| 参数 | 类型 | 默认值 | 含义 |
| --- | --- | --- | --- |
| `config_file` | 路径 | `<backend share>/config/backend.yaml` | 后端 YAML 路径，支持 `extends` 继承 |
| `mode` | 字符串 | `config` | `config` 沿用 YAML；`mapping` 关闭先验地图；`localization` 开启先验地图，均不改变 UWB 开关 |
| `prior_map` | 路径 | `""` | 覆盖 YAML 中的先验地图目录；留空则沿用 YAML |
| `prior_initialization_only` | `config` / 布尔 | `config` | `true` 让先验只在初始化时匹配一次（覆盖 `prior_map.initialization_only`）；`config` 沿用 YAML |
| `use_sim_time` | 布尔 | `false` | 后端和 RViz 是否使用 `/clock` 仿真时间 |
| `rviz` | 布尔 | `false` | 是否启动 RViz，并开启当前地图、坐标轴、轨迹和配准点云显示 |

#### `body_calibration.launch.py`

这些选项也可作为 `body_calibration_node` 的 ROS 参数传入。直接运行节点时，`odom_frame`、`odom_topic`
分别默认为 `camera_init`、`/Odometry`，其余与下表一致。

| 参数 | 类型 / 单位 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `map_frame` | 字符串 | `map` | 标定使用的全局参考坐标系；非空 |
| `odom_frame` | 字符串 | `odom` | 输入里程计的父坐标系；非空 |
| `odom_topic` | 字符串 | `/odometry` | 传感器里程计输入话题；非空 |
| `use_sim_time` | 布尔 | `false` | 是否使用 `/clock` 仿真时间 |
| `max_sync_dt` | 浮点，s | `0.08` | 按里程计时刻查询 map → odom TF 时最多等待多久；须 `≥0`，`0` 不等待 |
| `sample_period` | 浮点，s | `0.05` | 两次有效标定采样的最短间隔；须 `>0` |
| `print_period` | 浮点，s | `1.0` | 在终端刷新标定结果的周期；须 `>0` |
| `angle_sample_sec` | 浮点，s | `5.0` | 角度标定至少收集多长时间的数据；须 `≥0`，`0` 只检查样本数 |
| `angle_min_samples` | 整数，样本 | `80` | 角度标定至少需要的样本数；须 `≥1` |
| `offset_min_samples` | 整数，样本 | `80` | 水平偏移标定至少需要的样本数；须 `≥2` |
| `offset_min_yaw_span_deg` | 浮点，° | `180.0` | 水平偏移标定期间航向至少需要覆盖的角度；须 `>0` |
| `offset_radius_warn_m` | 浮点，m | `0.02` | 拟合旋转半径低于此值时提示偏移较小；须 `≥0`，`0` 不提示 |
| `target_roll_deg` | 浮点，° | `0.0` | 角度标定时车体相对 map 的已知横滚角 |
| `target_pitch_deg` | 浮点，° | `0.0` | 角度标定时车体相对 map 的已知俯仰角 |
| `target_yaw_deg` | 浮点，° | `0.0` | 角度标定时车体相对 map 的已知航向角 |

## 离线工具参数

下列五个命令均支持 `--help` / `-h`。标为“开关”的选项只写选项名即可；
布尔选项须填写 `true` / `false` 或 `1` / `0`。工具使用下表中的默认值，不读取后端 YAML。

### `build_pcd_map_bundle`

#### 输入、采样与输出

| 参数 | 类型 / 单位 | 默认值 | 含义 |
| --- | --- | --- | --- |
| `--input` / `--map` | 路径 | 必填 | 用于生成局部扫描的全局 PCD 地图 |
| `--out-samples` | 路径 | `""` | 模拟查询扫描的输出目录；与 `--out-bundle` 至少填写一个 |
| `--out-bundle` | 路径 | `""` | 关键帧地图包的输出目录；留空则不生成地图包 |
| `--x-min` | 浮点，m | `0.0` | 模拟传感器采样位置的 X 坐标下界 |
| `--x-max` | 浮点，m | `28.0` | 模拟传感器采样位置的 X 坐标上界；须 `≥ x-min` |
| `--y-min` | 浮点，m | `0.0` | 模拟传感器采样位置的 Y 坐标下界 |
| `--y-max` | 浮点，m | `15.0` | 模拟传感器采样位置的 Y 坐标上界；须 `≥ y-min` |
| `--sensor-z` | 浮点，m | `0.35` | 模拟传感器原点在地图中的 Z 坐标 |
| `--query-step` | 浮点，m | `2.0` | 生成查询扫描时，相邻采样位置的网格间距；须 `>0` |
| `--bundle-step` | 浮点，m | `1.5` | 生成地图关键帧时，相邻采样位置的网格间距；须 `>0` |
| `--query-yaws` | 逗号分隔角度，° | `0,90,180,-90` | 在每个查询位置分别模拟的航向角列表 |
| `--bundle-yaws` | 逗号分隔角度，° | `0` | 在每个地图采样位置分别生成关键帧的航向角列表 |
| `--max-samples` | 整数，帧 | `0` | 最多生成的查询扫描数；`≤0` 不限数量 |
| `--max-keyframes` | 整数，帧 | `0` | 最多生成的地图关键帧数；`≤0` 不限数量 |
| `--map-voxel` | 浮点，m | `0.4` | 输入全局地图及输出 `map.pcd` 的降采样体素边长；`≤0` 不降采样 |
| `--scan-voxel` | 浮点，m | `0.4` | 生成局部扫描后的降采样体素边长；`≤0` 不降采样 |
| `--min-distance` | 浮点，m | `0.5` | 模拟扫描保留的最小点距；须 `≥0` |
| `--max-distance` | 浮点，m | `80.0` | 模拟扫描保留的最大点距；须 `> min-distance` |
| `--fov-down` | 浮点，° | `-7.0` | 模拟扫描的最低仰角，以水平面为 `0°` |
| `--fov-up` | 浮点，° | `52.0` | 模拟扫描的最高仰角；须 `> fov-down` |
| `--azimuth-bins` | 整数，格 | `1440` | 水平一周划分的射线格数；越多角度分辨率越高，须 `>0` |
| `--elevation-bins` | 整数，格 | `160` | 垂直视场划分的射线格数；越多角度分辨率越高，须 `>0` |
| `--occlusion` | 布尔 | `false` | 是否模拟遮挡：开启后每个射线格只保留最近点，关闭则保留视场及量程内全部点 |
| `--write-map-pcd` | 布尔 | `true` | 是否在地图包中附带全局 `map.pcd` |
| `--descriptor-type` | 字符串 | `scan_context` | 生成的描述子类型：`scan_context`、`gprobe`，或 `both` 同时生成两种 |

#### 描述子选项

这些选项与 YAML 中的 `common.scan_context`、`common.gprobe` 参数含义和单位相同。
`--sc-enabled`、`--gprobe-enabled` 与 `--descriptor-type` 同时出现时，后写的选项覆盖前面的设置。

| 参数 | 对应键 | 默认值 |
| --- | --- | --- |
| `--sc-enabled` | `common.scan_context.enabled` | `true` |
| `--sc-rings` | `common.scan_context.num_rings` | `30` |
| `--sc-sectors` | `common.scan_context.num_sectors` | `90` |
| `--sc-max-radius` | `common.scan_context.max_radius` | `80.0` |
| `--sc-downsample` | `common.scan_context.downsample` | `0.6` |
| `--sc-lidar-height` | `common.scan_context.lidar_height` | 跟随 `--sensor-z` |
| `--sc-min-bin-points` | `common.scan_context.min_bin_points` | `1` |
| `--sc-augment-count` | `common.scan_context.augment_count` | `3` |
| `--sc-lateral-shift` | `common.scan_context.lateral_shift` | `2.0` |
| `--sc-search-ratio` | `common.scan_context.search_ratio` | `1.0` |
| `--sc-exclude-recent` | `common.scan_context.exclude_recent` | `15` |
| `--sc-candidates` | `common.scan_context.candidates` | `15` |
| `--sc-distance-threshold` | `common.scan_context.distance_threshold` | `0.6` |
| `--sc-level` | `common.scan_context.level_with_odom_roll_pitch` | `true` |
| `--gprobe-enabled` | `common.gprobe.enabled` | `false` |
| `--gprobe-rings` | `common.gprobe.num_rings` | `40` |
| `--gprobe-sectors` | `common.gprobe.num_sectors` | `60` |
| `--gprobe-max-range` | `common.gprobe.max_range` | `80.0` |
| `--gprobe-voxel-size` | `common.gprobe.voxel_size` | `0.5` |
| `--gprobe-sigma-t` | `common.gprobe.sigma_t` | `2.0` |
| `--gprobe-query-fov` | `common.gprobe.query_fov_deg` | `105.0` |
| `--gprobe-query-fov-center` | `common.gprobe.query_fov_center_deg` | `7.5` |
| `--gprobe-branch-top-k` | `common.gprobe.branch_top_k` | `15` |
| `--gprobe-verification-top-k` | `common.gprobe.verification_top_k` | `60` |
| `--gprobe-spatial-nms-radius` | `common.gprobe.spatial_nms_radius` | `2.0` |
| `--gprobe-heading-window` | `common.gprobe.heading_window_deg` | `50.0` |
| `--gprobe-column-trim` | `common.gprobe.column_trim_ratio` | `0.1` |
| `--gprobe-sgrt-kappa` | `common.gprobe.sgrt_kappa` | `10.0` |
| `--gprobe-level` | `common.gprobe.level_with_odom_roll_pitch` | `true` |
| `--gprobe-exclude-recent` | `common.gprobe.exclude_recent` | `15` |
| `--gprobe-candidates` | `common.gprobe.candidates` | `30` |

### `transform_map_bundle`

可选手动填写变换、四点定轴或目标地图配准，每次只能使用一种。变换方向均为旧地图 → 新地图。

| 参数 | 类型 / 单位 | 默认值 | 含义 |
| --- | --- | --- | --- |
| `--in` | 路径 | 必填 | 待转换坐标的输入地图包目录 |
| `--out` | 路径 | 必填 | 转换后的地图包目录 |
| `--translation` / `--t` | 3 元数值，m | 显式变换时必填 | 旧地图原点在新坐标系中的位置，按 `x y z` 填写 |
| `--quaternion` / `--q` | 4 元数值 | 显式变换时必填 | 将旧坐标旋转到新坐标系的四元数，按 `qx qy qz qw` 填写；须非零，读取后归一化 |
| `--tx`、`--ty`、`--tz` | 浮点，m | 显式变换时完整提供 | 分别填写平移的三个分量，等价于 `--translation` |
| `--qx`、`--qy`、`--qz`、`--qw` | 浮点 | 显式变换时完整提供 | 分别填写旋转四元数的四个分量，等价于 `--quaternion` |
| `--origin` | 3 元数值，m | 四点定轴时必填 | 新坐标原点在旧地图中的坐标 |
| `--x-axis` | 3 元数值，m | 四点定轴时必填 | 新 X 轴正方向上任一点在旧地图中的坐标 |
| `--y-axis` | 3 元数值，m | 四点定轴时必填 | 新 Y 轴正方向上任一点在旧地图中的坐标 |
| `--z-axis` | 3 元数值，m | 四点定轴时必填 | 新 Z 轴正方向上任一点在旧地图中的坐标 |
| `--target-map` | 路径 | 地图配准时必填 | 要对齐到的全局 PCD，使用目标坐标系 |
| `--source-map` | 路径 | `""` | 用于配准的源 PCD；留空则优先读取地图包的 `map.pcd`，没有时由关键帧重建 |
| `--align-voxel` | 浮点，m | `0.2` | 最细一层配准的降采样体素边长；须 `≥0`，`0` 不降采样 |
| `--align-max-correspondence` | 浮点，m | `3.0` | 建立配准对应时允许的最大点间距离；须 `>0` |
| `--align-overlap-distance` | 浮点，m | `0.3` | 变换后源点距最近目标点不超过此值时计为重叠；须 `>0` |
| `--align-min-overlap` | 浮点，无量纲 | `0.4` | 配准结果所需的最低重叠比例；范围 `[0,1]` |
| `--align-rmse-threshold` | 浮点，m | `0.3` | 配准结果允许的最大点面均方根误差；须 `>0` |
| `--align-max-iterations` | 整数，次 | `40` | 每一层配准的最大迭代次数；须 `>0` |
| `--yaw-search-step-deg` | 浮点，° | `15.0` | 搜索整周航向时，相邻初值的角度间隔；须 `≥0`，`0` 仅尝试零航向 |
| `--initial-yaw-deg` | 浮点，° | 未启用 | 指定唯一的航向初值；填写后不再搜索整周 |
| `--verbose-alignment` | 开关 | 关闭 | 输出每个航向候选的配准详情 |

### `filter_map_bundle`

| 参数 | 类型 / 单位 | 默认值 | 含义 |
| --- | --- | --- | --- |
| `--in` | 路径 | 必填 | 待过滤的输入地图包目录 |
| `--out` | 路径 | 过滤时必填 | 过滤后的地图包目录 |
| `--reference-map` | 路径 | 过滤时必填 | 用于判断哪些点应保留的已清理全局 PCD |
| `--export-reference-map` | 路径 | `""` | 仅将地图包导出为完整分辨率的全局 PCD，供外部清理；留空则执行过滤 |
| `--keep-distance` | 浮点，m | `0.2` | 原始点距参考地图不超过此值时保留；须 `>0` |
| `--exact` | 开关 | 关闭 | 按原始点坐标匹配，改用 `--match-epsilon` 作为距离容差 |
| `--match-epsilon` | 浮点，m | `0.00001` | `--exact` 模式允许的点坐标误差；须 `≥0`，`0` 要求重合 |
| `--map-voxel` | 浮点，m | `0.15` | 输出全局 PCD 的降采样体素边长；`≤0` 不降采样 |
| `--min-points-per-keyframe` | 整数，点 | `20` | 过滤后点数少于此值的关键帧将被删除；须 `≥0`，`0` 不筛除，但保存时仍不允许空点云 |

### `clean_map_bundle`

| 参数 | 类型 / 单位 | 默认值 | 含义 |
| --- | --- | --- | --- |
| `--in` / `--input` | 路径 | 必填 | 含实测局部扫描和对应传感器位姿的地图包，不能只提供融合 PCD |
| `--out` / `--output` | 路径 | 必填 | 清理后的地图包目录 |
| `--lidar-origin` | 3 元数值，m | `0 0 0` | LiDAR 光心在关键帧点云坐标系中的坐标，按 `x y z` 填写 |
| `--voxel-resolution` | 浮点，m | `0.2` | 判断空间是否被占据的体素边长；范围 `[0.05,2]` |
| `--raycast-range` | 浮点，m | `20.0` | 每条扫描射线最多检查的距离，不裁剪范围外点；范围 `[voxel-resolution,200]` |
| `--free-observations` | 整数，次 | `3` | 至少被多少帧观测为空闲，才将体素判为空闲；范围 `[1,1000]` |
| `--threads` | 整数，线程 | `4` | 清理计算使用的 CPU 工作线程数；范围 `[1,32]` |
| `--map-voxel` | 浮点，m | `0.15` | 清理后全局 PCD 的降采样体素边长；须 `≥0`，`0` 保留全部点 |

### `rebuild_scan_context_descriptors`

在指定地图包中重新生成 `sc_descriptors.bin`。未填写的选项使用下表默认值，不沿用原文件中的配置。
各描述子参数的单位和含义见 `common.scan_context` 表。

| 参数 | 对应键 / 含义 | 默认值 |
| --- | --- | --- |
| `--bundle` | 输入及输出地图包目录 | 必填 |
| `--enabled` | `common.scan_context.enabled` | `true` |
| `--level` | `common.scan_context.level_with_odom_roll_pitch` | `true` |
| `--num-rings` | `common.scan_context.num_rings` | `30` |
| `--num-sectors` | `common.scan_context.num_sectors` | `90` |
| `--max-radius` | `common.scan_context.max_radius` | `80.0` |
| `--downsample` | `common.scan_context.downsample` | `0.6` |
| `--lidar-height` | `common.scan_context.lidar_height` | `0.35` |
| `--min-bin-points` | `common.scan_context.min_bin_points` | `1` |
| `--augment-count` | `common.scan_context.augment_count` | `3` |
| `--lateral-shift` | `common.scan_context.lateral_shift` | `2.0` |
| `--search-ratio` | `common.scan_context.search_ratio` | `1.0` |
| `--exclude-recent` | `common.scan_context.exclude_recent` | `15` |
| `--candidates` | `common.scan_context.candidates` | `15` |
| `--distance-threshold` | `common.scan_context.distance_threshold` | `0.6` |

## 调参建议

先核对输入坐标、安装外参和时间，再根据场景调整描述子、关键帧与配准参数。其余参数通常可以保持默认值。

### 传感器与安装参数

| 参数 | 范围 / 当前值 | 作用 | 设置建议 |
| --- | --- | --- | --- |
| `common.descriptor_type` | `scan_context` / `gprobe`；当前 `scan_context` | 地点识别与重定位使用的描述子类型 | Mid-360 可优先选 Scan Context；Avia、奥丁之眼可优先选 G-PROBE，再比较实际重定位成功率和耗时。切换时只保留对应参数组，并使用匹配的地图描述子；见 [地图加载](../apps/node_map.cpp) |
| `common.body.rotation.roll_deg`、`common.body.rotation.pitch_deg`、`common.body.rotation.yaw_deg` | 有限角度，°；当前依次为 `-14.94`、`0.523`、`-150.6` | 前端传感器坐标系到车体坐标系的固定旋转 | 用已知车体姿态运行 [标定工具](#body_calibrationlaunchpy)，更改安装后重新标定；检查输出朝向、横滚和俯仰是否符合实物 |
| `common.body.offset.x`、`common.body.offset.y`、`common.body.offset.z` | 有限位移，m；当前依次为 `-0.098`、`0.094`、`0.0` | 从传感器原点指向车体原点、在车体系表达的位移 | XY 可通过绕固定车体中心旋转标定，Z 按安装尺寸测量；标定后检查原地旋转时车体位置是否稳定。方向见 [外参实现](../src/body.cpp) |
| `uwb.antenna_position_body.x`、`uwb.antenna_position_body.y`、`uwb.antenna_position_body.z` | 有限坐标，m；当前依次为 `-0.245`、`0.0`、`0.0` | UWB 天线相位中心在车体坐标系中的位置 | 按车体原点和坐标轴测量，移动天线后更新；转弯时若偏差随朝向变化，应检查此项。见 [天线位置换算](../apps/fusion_parameters.cpp) |

使用 Scan Context 时，`common.scan_context.lidar_height` 按安装高度设置；使用 G-PROBE 时，
`common.gprobe.query_fov_deg`、`common.gprobe.query_fov_center_deg` 按输入点云的实际水平视场及朝向设置。

### UWB 参数

| 参数 | 范围 / 当前值 | 作用 | 设置建议 |
| --- | --- | --- | --- |
| `uwb.estimate_zero_pose_online`、`uwb.zero_pose.x`、`uwb.zero_pose.y`、`uwb.zero_pose.z`、`uwb.zero_pose.yaw_deg` | 布尔、有限平移 m 与航向 °；当前在线估计开启，固定变换为零平移、零航向 | UWB 场地坐标系到地图坐标系的对齐 | 先验地图经过手动平移、旋转，且未参考 UWB 坐标时，建议开启在线估计；需同时启用 UWB 和先验地图。已准确测得变换时，可填写 `zero_pose` 并关闭在线估计。运动覆盖不同位置和方向后，检查两路轨迹是否一致；见 [UWB 融合](../src/uwb_fusion.cpp) |
| `uwb.time_offset_seconds` | 有限秒值；当前 `-0.15 s` | UWB 消息时间到实际测量时刻的固定偏置，运动时刻 = 消息时间戳 + 此值 | 尽量在部署前离线标定。先核对时间戳约定和外参，用含直线加减速、转弯的数据，对齐 UWB 与 LiDAR 推算的天线位置。在设备可能的延迟范围内搜索，选各运动段残差稳定较小的偏置，再换一段数据验证；见 [时间换算](../src/uwb_timing.cpp) |
| `uwb.effective_rate_hz` | `≥0` Hz；当前 `1 Hz`；`0` 关闭按周期识别保持重发的过滤 | UWB 独立位置解的实际更新频率 | 结合设备输出设置和记录数据，填写实际产生新定位解的频率；同一解多次转发仍按原始解算频率计算。设备频率或传输链路改变后重新核对；见 [重复数据过滤](../src/uwb_fusion.cpp) |

### 场景参数

| 参数 | 范围 / 当前值 | 作用 | 设置建议 |
| --- | --- | --- | --- |
| `prior_map.directory` | 可读的地图包目录；当前 `pcd/rmuc` | 当前场地的全局定位参考地图 | 换场地时更换地图包，并核对坐标方向、覆盖范围和描述子配置。先验定位开启 `prior_map.enabled`，重新建图时关闭；见 [地图加载](../apps/node_map.cpp) |
| `common.scan_context.distance_threshold`、`recovery.initial_max_descriptor_distance`、`recovery.descriptor_distance_limit` | 非负描述子距离；当前均为 `0.8` | 分别控制 SC 回环、初始定位和恢复时的候选筛选，越大越宽松 | 根据描述子质量和环境调整：正确候选常被过滤时增大，重复结构多、错误候选过多时减小；结合召回效果和几何验证耗时确认 |
| `trajectory.save_directory` | 可写目录；当前 `pcd/runtime_zlio` | 本次建图结果的保存位置 | 按场地和采集批次分目录保存，与先验地图目录分开，便于比较不同建图结果；见 [保存入口](../apps/node_map.cpp) |
| `trajectory.keyframe.distance`、`trajectory.keyframe.angle_deg` | 有限距离 m、角度 °；当前均为 `-1`；`≤0` 关闭对应触发 | 相对上一关键帧的平移、转角触发门限 | 建图时按速度和扫描重叠设置较小正值，保证转弯处有足够关键帧。先验定位可设为 `-1`，改按时间选帧；见 [选帧逻辑](../src/keyframe_map.cpp) |
| `trajectory.keyframe.min_interval`、`trajectory.keyframe.max_interval` | `min_interval ≥0` s；`max_interval >0` 时启用定时触发；当前为 `0`、`0.5 s` | 关键帧的最小插入间隔与定时触发间隔 | `max_interval` 补充定时关键帧，`min_interval` 限制插入频率。当前 `0.5 s` 在输入充足时约为 `2 Hz`；结合匹配连续性和耗时调整，见 [选帧逻辑](../src/keyframe_map.cpp) |
| `registration.voxel_resolution` | `≥0` m；当前 `0.4 m`；`0` 保留原始点密度 | 参与配准的点云空间采样尺度 | 细节不足时减小，计算过慢时增大；在同一数据上比较误差、匹配连续性和耗时。见 [配准实现](../src/registration.cpp) |
| `tracking.target_keyframes`、`recovery.target_keyframes`、`loop.target_keyframes` | 均为 `≥1` 的整数；当前依次为 `11`、`11`、`17` | 常规跟踪、重定位、回环验证的目标子图关键帧数量上限 | 子图稀疏或覆盖不足时增加，点云密集时可减少；同时检查 `target_radius` 是否足够。比较匹配质量、内存与耗时，见 [跟踪](../src/tracking_matcher.cpp)、[重定位](../src/relocalization.cpp)、[回环](../src/loop.cpp) |
| `loop.enabled` | 布尔；默认 `false`，由 `--zloc-loop` 显式开启 | 建图轨迹的历史回环检索与约束 | 建图时可开启以约束重访区域的累计漂移，同时会增加历史图的计算和内存开销。可与先验定位同时开启，先验和运行描述子独立；见 [开关关系](../src/config.cpp) |
| `startup_hint.enabled`、`startup_hint.x`、`startup_hint.y`、`startup_hint.z`、`startup_hint.yaw_deg` | 布尔、有限位置 m 与航向 °；当前提示关闭 | 机器人启动时，车体原点在期望世界坐标系中的位置和航向 | 能在已知位置和朝向启动时填写。无先验、无 UWB 时，它决定初始世界坐标，摆放误差会带入轨迹，后续仍可能漂移；见 [初始化](../src/localization.cpp) |

每次调整一组参数，用同一段数据比较定位连续性、位置误差和处理耗时。
