/**
 * @file bundle.cpp
 * @brief 实现地图工具共用的配置加载、路径隔离与快照写出。
 */
#include "bundle.hpp"

#include <filesystem>
#include <stdexcept>

#include "backend/directory.hpp"
#include "backend/keyframe_map.hpp"
#include "backend/map_bundle.hpp"

namespace backend::tools {
namespace {
/// 仅在描述子路径存在时读取配置，并将损坏文件与真正缺失的文件区分。
template <class Descriptor, class Config>
std::optional<Config> loadConfig(const std::string& directory, const char* filename) {
    const auto path = std::filesystem::path(directory) / filename;
    if (!std::filesystem::exists(path) && !std::filesystem::is_symlink(path))
        return std::nullopt;
    Config config;
    if (!Descriptor::loadConfig(directory, &config))
        throw std::runtime_error("cannot read descriptor configuration: " + path.string());
    return config;
}
}  // namespace

/// 读取地图包中存在的两类描述子配置，保留各自启用状态。
DescriptorConfigs loadDescriptorConfigs(const std::string& directory) {
    return {loadConfig<ScanContextDescriptor, ScanContextDescriptorConfig>(directory,
                                                                           "sc_descriptors.bin"),
            loadConfig<GProbeDescriptor, GProbeDescriptorConfig>(directory,
                                                                 GProbeDescriptor::fileName())};
}

/// 规范化路径后检查输入输出是否重合或包含，避免修改输入地图包。
void validateOutputLocation(const std::string& input, const std::string& output) {
    if (input.empty() || output.empty())
        throw std::invalid_argument("input and output paths must be nonempty");
    const auto source = normalizedAbsolutePath(input);
    const auto target = normalizedAbsolutePath(output);
    const auto relative = target.lexically_relative(source);
    if (source == target || (!relative.empty() && *relative.begin() != ".."))
        throw std::invalid_argument("output must be outside the input bundle");
}

/// 写出关键帧及所选描述子；任一步骤失败均阻止调用方提交目录。
void writeStagedBundle(const std::string& directory, const std::vector<Keyframe>& frames,
                       const DescriptorConfigs& descriptors) {
    if (!KeyframeMap::saveSnapshot(directory, frames))
        throw std::runtime_error("cannot write keyframe snapshot: " + directory);
    if (descriptors.scan_context &&
        !saveMapDescriptor<ScanContextDescriptor>(directory, frames, *descriptors.scan_context))
        throw std::runtime_error("cannot write Scan Context descriptors: " + directory);
    if (descriptors.gprobe &&
        !saveMapDescriptor<GProbeDescriptor>(directory, frames, *descriptors.gprobe))
        throw std::runtime_error("cannot write G-PROBE descriptors: " + directory);
}
}  // namespace backend::tools
