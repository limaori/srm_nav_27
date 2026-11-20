/**
 * @file storage.cpp
 * @brief 读写带版本与几何配置的 Scan Context 描述子文件。
 */

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <utility>

#include "backend/scan_context.hpp"

namespace backend {
namespace {

constexpr std::uint32_t kScDescFileMagic = 0x53435859;  // 'SCXY'
constexpr std::uint32_t kScDescFileVersion = 1;
constexpr std::uint32_t kMaxStoredKeyframes = 1000000;

/// 返回 Scan Context 描述子文件在地图目录中的路径。
std::string descriptorsFile(const std::string& root) {
    return root + "/sc_descriptors.bin";
}

/// 按持久化格式写入定长标量的原始字节。
template <typename T>
void writePod(std::ostream& os, const T& v) {
    os.write(reinterpret_cast<const char*>(&v), sizeof(T));
}

/// 按持久化格式读取定长标量并检查流状态。
template <typename T>
bool readPod(std::istream& is, T* v) {
    is.read(reinterpret_cast<char*>(v), sizeof(T));
    return static_cast<bool>(is);
}

/// 按既定字段顺序写入描述子配置。
void writeConfig(std::ostream& out, const ScanContextDescriptorConfig& config) {
    const std::uint8_t enabled = config.enabled ? 1 : 0;
    const std::uint8_t level = config.level_with_odom_roll_pitch ? 1 : 0;
    const std::int32_t rings = config.num_rings;
    const std::int32_t sectors = config.num_sectors;
    const std::int32_t min_bin_points = config.min_bin_points;
    const std::int32_t augment_count = config.augment_count;
    writePod(out, enabled);
    writePod(out, level);
    writePod(out, rings);
    writePod(out, sectors);
    writePod(out, config.max_radius);
    writePod(out, config.downsample);
    writePod(out, config.lidar_height);
    writePod(out, min_bin_points);
    writePod(out, augment_count);
    writePod(out, config.lateral_shift);
    writePod(out, config.search_ratio);
}

/// 按既定字段顺序读取并验收描述子配置。
bool readConfig(std::istream& in, ScanContextDescriptorConfig* config) {
    std::uint8_t enabled = 0;
    std::uint8_t level = 0;
    std::int32_t rings = 0;
    std::int32_t sectors = 0;
    std::int32_t min_bin_points = 0;
    std::int32_t augment_count = 0;
    if (!readPod(in, &enabled) || !readPod(in, &level) || !readPod(in, &rings) ||
        !readPod(in, &sectors) || !readPod(in, &config->max_radius) ||
        !readPod(in, &config->downsample) || !readPod(in, &config->lidar_height) ||
        !readPod(in, &min_bin_points) || !readPod(in, &augment_count) ||
        !readPod(in, &config->lateral_shift) || !readPod(in, &config->search_ratio)) {
        return false;
    }
    config->enabled = static_cast<bool>(enabled);
    config->level_with_odom_roll_pitch = static_cast<bool>(level);
    config->num_rings = rings;
    config->num_sectors = sectors;
    config->min_bin_points = min_bin_points;
    config->augment_count = augment_count;
    return enabled <= 1 && level <= 1 && rings > 0 && sectors > 0 && min_bin_points > 0 &&
           augment_count >= 1 && augment_count <= 7 && std::isfinite(config->max_radius) &&
           config->max_radius >= 1.0 && std::isfinite(config->downsample) &&
           config->downsample >= 0.0 && std::isfinite(config->lidar_height) &&
           std::isfinite(config->lateral_shift) && config->lateral_shift >= 0.0 &&
           std::isfinite(config->search_ratio) && config->search_ratio >= 0.0 &&
           config->search_ratio <= 1.0;
}

/// 比较影响 Scan Context 几何编码的配置字段。
bool sameDescriptorGeometry(const ScanContextDescriptorConfig& lhs,
                            const ScanContextDescriptorConfig& rhs) {
    constexpr double kEps = 1e-6;
    return lhs.enabled == rhs.enabled &&
           lhs.level_with_odom_roll_pitch == rhs.level_with_odom_roll_pitch &&
           lhs.num_rings == rhs.num_rings && lhs.num_sectors == rhs.num_sectors &&
           lhs.min_bin_points == rhs.min_bin_points && lhs.augment_count == rhs.augment_count &&
           std::abs(lhs.max_radius - rhs.max_radius) < kEps &&
           std::abs(lhs.downsample - rhs.downsample) < kEps &&
           std::abs(lhs.lidar_height - rhs.lidar_height) < kEps &&
           std::abs(lhs.lateral_shift - rhs.lateral_shift) < kEps &&
           std::abs(lhs.search_ratio - rhs.search_ratio) < kEps;
}

/// 写入描述子网格尺寸及其连续数值数据。
void writeDescriptor(std::ostream& out, const ScanContextDescriptor::Descriptor& descriptor) {
    const std::int32_t rows = descriptor.image.rows();
    const std::int32_t cols = descriptor.image.cols();
    writePod(out, rows);
    writePod(out, cols);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            const float value = descriptor.image(r, c);
            writePod(out, value);
        }
    }
}

/// 读取描述子网格并检查尺寸和数值合法性。
bool readDescriptor(std::istream& in, ScanContextDescriptor::Descriptor* descriptor,
                    const ScanContextDescriptorConfig& config) {
    std::int32_t rows = 0;
    std::int32_t cols = 0;
    if (!readPod(in, &rows) || !readPod(in, &cols) || rows != config.num_rings ||
        cols != config.num_sectors) {
        return false;
    }
    descriptor->image.resize(rows, cols);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            float value = 0.0f;
            if (!readPod(in, &value) || !std::isfinite(value)) {
                return false;
            }
            descriptor->image(r, c) = value;
        }
    }
    descriptor->sector_key = descriptor->image.colwise().mean().transpose();
    return descriptor->sector_key.allFinite();
}

/// 解码指定帧的描述子变体并核对当前几何配置要求的网格尺寸。
bool readConfiguredDescriptor(std::ifstream& in, ScanContextDescriptor::Descriptor& descriptor,
                              const ScanContextDescriptorConfig& config, std::uint32_t i,
                              std::uint32_t j) {
    if (!readDescriptor(in, &descriptor, config)) {
        std::cerr << "ScanContextDescriptor::load: invalid descriptor at "
                  << "keyframe " << i << " variant " << j << "\n";
        return false;
    }
    return true;
}

/// 按每帧固定编码长度校验数量、变体及文件剩余字节，限制解码前的分配规模。
bool validPayloadSize(std::ifstream& in, std::streampos end, std::uint32_t count,
                      const ScanContextDescriptorConfig& config) {
    const auto position = in.tellg();
    if (position < 0 || end < position || count > kMaxStoredKeyframes)
        return false;
    const auto remaining = static_cast<std::uint64_t>(end - position);
    const std::uint64_t variants = config.enabled ? config.augment_count : 0;
    const std::uint64_t cells = static_cast<std::uint64_t>(config.num_rings) * config.num_sectors;
    const std::uint64_t descriptor_bytes = 2 * sizeof(std::int32_t) + cells * sizeof(float);
    if (variants != 0 &&
        descriptor_bytes >
            (std::numeric_limits<std::uint64_t>::max() - sizeof(std::uint32_t)) / variants)
        return false;
    const std::uint64_t frame_bytes = sizeof(std::uint32_t) + variants * descriptor_bytes;
    return count <= remaining / frame_bytes && count * frame_bytes == remaining;
}

/// 输出不匹配的持久化几何配置与当前配置，保持逐字段诊断顺序。
void reportGeometryMismatch(const ScanContextDescriptorConfig& saved_config,
                            const ScanContextDescriptorConfig& config) {
    std::cerr << "ScanContextDescriptor::load: saved SC config does not "
                 "match runtime config"
              << " (saved rings=" << saved_config.num_rings
              << ", sectors=" << saved_config.num_sectors
              << ", max_radius=" << saved_config.max_radius
              << ", downsample=" << saved_config.downsample
              << ", lidar_height=" << saved_config.lidar_height
              << ", min_bin_points=" << saved_config.min_bin_points
              << ", augment_count=" << saved_config.augment_count
              << ", lateral_shift=" << saved_config.lateral_shift
              << ", search_ratio=" << saved_config.search_ratio
              << ", level_with_odom_roll_pitch=" << saved_config.level_with_odom_roll_pitch
              << "; runtime rings=" << config.num_rings << ", sectors=" << config.num_sectors
              << ", max_radius=" << config.max_radius << ", downsample=" << config.downsample
              << ", lidar_height=" << config.lidar_height
              << ", min_bin_points=" << config.min_bin_points
              << ", augment_count=" << config.augment_count
              << ", lateral_shift=" << config.lateral_shift
              << ", search_ratio=" << config.search_ratio
              << ", level_with_odom_roll_pitch=" << config.level_with_odom_roll_pitch << ")\n";
}

}  // namespace

/// 保存当前描述子配置及数据，失败时返回 false。
bool ScanContextDescriptor::save(const std::string& directory) const {
    std::ofstream out(descriptorsFile(directory), std::ios::binary);
    if (!out.is_open()) {
        std::cerr << "ScanContextDescriptor::save: cannot open " << descriptorsFile(directory)
                  << "\n";
        return false;
    }

    const std::uint32_t count = static_cast<std::uint32_t>(descriptors_.size());
    writePod(out, kScDescFileMagic);
    writePod(out, kScDescFileVersion);
    writePod(out, count);
    writeConfig(out, config_);
    for (const auto& variants : descriptors_) {
        const std::uint32_t variant_count = static_cast<std::uint32_t>(variants.size());
        writePod(out, variant_count);
        for (const auto& descriptor : variants) {
            writeDescriptor(out, descriptor);
        }
    }
    out.close();
    return static_cast<bool>(out);
}

/// 仅读取持久化描述子配置，不加载关键帧数据。
bool ScanContextDescriptor::loadConfig(const std::string& directory,
                                       ScanContextDescriptorConfig* config) {
    if (config == nullptr) {
        return false;
    }
    std::ifstream in(descriptorsFile(directory), std::ios::binary);
    if (!in.is_open()) {
        std::cerr << "ScanContextDescriptor::loadConfig: cannot open " << descriptorsFile(directory)
                  << "\n";
        return false;
    }

    std::uint32_t magic = 0;
    std::uint32_t version = 0;
    std::uint32_t count = 0;
    if (!readPod(in, &magic) || magic != kScDescFileMagic) {
        std::cerr << "ScanContextDescriptor::loadConfig: bad magic in "
                  << descriptorsFile(directory) << "\n";
        return false;
    }
    if (!readPod(in, &version) || version != kScDescFileVersion) {
        std::cerr << "ScanContextDescriptor::loadConfig: unsupported version " << version << "\n";
        return false;
    }
    if (!readPod(in, &count) || count > kMaxStoredKeyframes) {
        return false;
    }
    ScanContextDescriptorConfig loaded = *config;
    if (!readConfig(in, &loaded))
        return false;
    *config = loaded;
    return true;
}

/// 加载与当前几何配置兼容的描述子数据，失败时返回 false。
bool ScanContextDescriptor::load(const std::string& directory) {
    std::ifstream in(descriptorsFile(directory), std::ios::binary | std::ios::ate);
    if (!in.is_open()) {
        std::cerr << "ScanContextDescriptor::load: cannot open " << descriptorsFile(directory)
                  << "\n";
        return false;
    }
    const auto end = in.tellg();
    in.seekg(0);

    std::uint32_t magic = 0;
    std::uint32_t version = 0;
    std::uint32_t count = 0;
    if (!readPod(in, &magic) || magic != kScDescFileMagic) {
        std::cerr << "ScanContextDescriptor::load: bad magic in " << descriptorsFile(directory)
                  << "\n";
        return false;
    }
    if (!readPod(in, &version) || version != kScDescFileVersion) {
        std::cerr << "ScanContextDescriptor::load: unsupported version " << version << "\n";
        return false;
    }
    if (!readPod(in, &count)) {
        return false;
    }

    ScanContextDescriptorConfig saved_config = config_;
    if (!readConfig(in, &saved_config)) {
        return false;
    }
    if (!sameDescriptorGeometry(config_, saved_config)) {
        reportGeometryMismatch(saved_config, config_);
        return false;
    }

    if (!validPayloadSize(in, end, count, saved_config))
        return false;
    const std::uint32_t expected_variants = config_.enabled ? config_.augment_count : 0;
    std::vector<DescriptorVariants> loaded;
    loaded.reserve(count);
    for (std::uint32_t i = 0; i < count; ++i) {
        std::uint32_t variant_count = 0;
        if (!readPod(in, &variant_count) || variant_count != expected_variants) {
            std::cerr << "ScanContextDescriptor::load: invalid variant count at keyframe " << i
                      << "\n";
            return false;
        }
        DescriptorVariants variants;
        variants.reserve(variant_count);
        for (std::uint32_t j = 0; j < variant_count; ++j) {
            Descriptor descriptor;
            if (!readConfiguredDescriptor(in, descriptor, config_, i, j))
                return false;
            variants.push_back(std::move(descriptor));
        }
        loaded.push_back(std::move(variants));
    }
    descriptors_.swap(loaded);
    revision_ = nextRevision();
    return true;
}

}  // namespace backend
