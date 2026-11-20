/**
 * @file parameters.hpp
 * @brief 从 ROS 参数树读取并校验后端有效配置。
 */
#pragma once

#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <type_traits>
#include <unordered_set>
#include <vector>

#include "apps/config.hpp"

namespace backend::ros {

/// 负责参数声明、类型兼容和统一 schema 校验。
class ParameterReader {
public:
    /// 绑定参数声明、读取和日志所使用的 ROS 节点。
    explicit ParameterReader(rclcpp::Node& node) : node_(node) {}

    /// 加载并校验完整参数树；未知或已废弃参数会抛出异常。
    NodeConfig load();

private:
    /// 读取输入传输、坐标系和 body 外参。
    void loadCommonInput();
    /// 读取地图、关键帧和统一配准参数。
    void loadMapTrajectoryAndRegistration();
    /// 读取连续跟踪与联合创新参数。
    void loadTracking();
    /// 读取状态机恢复路径参数。
    void loadRecovery();
    /// 读取 UWB 测量、外参和在线标定参数。
    void loadUwb();
    /// 读取在线坐标标定开关及固定 UWB 坐标变换。
    void loadUwbFrameCalibration();
    /// 读取天线杆臂及位置测量噪声模型。
    void loadUwbMeasurementModel();
    /// 读取 UWB 启动对齐的配对与稳定性参数。
    void loadUwbBootstrap();
    /// 读取回环与因子图参数。
    void loadLoopAndGraph();
    /// 读取图先验、过程噪声、鲁棒核及固定窗口参数。
    void loadGraph();
    /// 读取 reset 事务和启动提示参数。
    void loadResetAndStartup();
    /// 读取可视化、服务和兼容输出参数。
    void loadOutputs();
    /// 读取所选地点描述子的参数。
    void loadDescriptor();
    /// 校验跨参数能力组合并执行已声明覆盖。
    void validateRelationships();
    /// 拒绝未被 schema 消费的未知参数。
    void validateParameterSchema() const;

    /// 结构体初始化值是默认值的唯一来源；只有有意覆盖或依赖其他参数时传第三参。
    template <class T>
    void getParam(const std::string& path, T& value) {
        getParam(path, value, value);
    }

    /// 读取一个参数并记录其 schema 所有权；兼容整数和浮点 YAML 写法。
    template <typename T>
    void getParam(const std::string& path, T& value, const T& default_value) {
        std::string name = path;
        std::replace(name.begin(), name.end(), '/', '.');
        consumed_parameter_names_.insert(name);
        if (!node_.has_parameter(name)) {
            node_.declare_parameter<T>(name, default_value);
        }
        const rclcpp::Parameter parameter = node_.get_parameter(name);
        if constexpr (std::is_same_v<T, double>) {
            value = parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER
                        ? static_cast<double>(parameter.as_int())
                        : parameter.as_double();
        } else if constexpr (std::is_same_v<T, int>) {
            value = parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE
                        ? static_cast<int>(parameter.as_double())
                        : static_cast<int>(parameter.as_int());
        } else if constexpr (std::is_same_v<T, std::vector<double>>) {
            if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
                const auto& integers = parameter.as_integer_array();
                value.assign(integers.begin(), integers.end());
            } else {
                value = parameter.as_double_array();
            }
        } else {
            value = parameter.get_value<T>();
        }
    }

    rclcpp::Node& node_;
    NodeConfig parameters_;
    std::unordered_set<std::string> consumed_parameter_names_;
};

}  // namespace backend::ros
