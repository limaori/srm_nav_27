# Point-LIO PCD output (compatibility directory)

Point-LIO 的累积点云默认写到运行期环境变量 `SRM27_PCD_DIR`：

    SRM27_PCD_DIR=${SRM27_DATA_DIR:-~/srm_nav_27_data}/generated_pcd/point_lio

未设置该变量时才回退到本目录（`ROOT_DIR/PCD`），仅用于不经过启动脚本的
单独调试。正式建图/导航请用 `scripts/bringup/start_real_slam.sh` 或
`scripts/bringup/start_sim_slam.sh`，它们会设置该变量并把 PCD 留在工作区外。

本目录保留在源码包里只为兼容旧路径；不要在这里长期堆放 GB 级点云。
