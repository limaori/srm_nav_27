/**
 * @file bundle.hpp
 * @brief 地图工具共用的描述子配置、输出路径校验及一致快照写出。
 */
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "backend/gprobe.hpp"
#include "backend/scan_context.hpp"
#include "backend/types.hpp"

namespace backend::tools {
/// 可选值表示文件是否存在；配置中的 enabled 保留描述子自身的启用语义。
struct DescriptorConfigs {
    std::optional<ScanContextDescriptorConfig> scan_context;
    std::optional<GProbeDescriptorConfig> gprobe;

    /// 判断地图包是否未携带任何受支持的描述子。
    bool empty() const { return !scan_context && !gprobe; }
};

/// 读取已有描述子配置；缺失返回空项，文件损坏或符号链接失效时抛出异常。
DescriptorConfigs loadDescriptorConfigs(const std::string& directory);
/// 拒绝空路径、输入输出别名以及位于输入目录内部的输出位置。
void validateOutputLocation(const std::string& input, const std::string& output);
/// 在调用方的暂存目录写出同一快照的关键帧与描述子；其他文件及最终提交由调用方负责。
void writeStagedBundle(const std::string& directory, const std::vector<Keyframe>& frames,
                       const DescriptorConfigs& descriptors);
}  // namespace backend::tools
