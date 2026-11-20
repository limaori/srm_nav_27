/**
 * @file storage.cpp
 * @brief 读写带版本和几何配置的 G-PROBE 二进制描述子。
 */

#include <cmath>
#include <fstream>
#include <iostream>

#include "gprobe.hpp"

namespace backend::gprobe_detail {
namespace {

/// 生成描述子文件的固定路径。
std::string descriptorPath(const std::string& directory) {
    return directory + "/" + GProbeDescriptor::fileName();
}

/// 写出一个平凡可复制的标量。
template <typename T>
void writePod(std::ostream& out, const T& value) {
    out.write(reinterpret_cast<const char*>(&value), sizeof(T));
}

/// 读取一个平凡可复制的标量。
template <typename T>
bool readPod(std::istream& in, T* value) {
    in.read(reinterpret_cast<char*>(value), sizeof(T));
    return static_cast<bool>(in);
}

/// 以稳定的单字节格式写出布尔值。
void writeBool(std::ostream& out, const bool value) {
    writePod(out, static_cast<std::uint8_t>(value ? 1U : 0U));
}

/// 读取并校验单字节布尔值。
bool readBool(std::istream& in, bool* value) {
    std::uint8_t encoded = 0U;
    if (!readPod(in, &encoded) || encoded > 1U) {
        return false;
    }
    *value = encoded != 0U;
    return true;
}

/// 按文件版本约定写出全部描述子参数。
void writeConfig(std::ostream& out, const GProbeDescriptorConfig& config) {
    writeBool(out, config.enabled);
    writeBool(out, config.level_with_odom_roll_pitch);
    writePod(out, static_cast<std::int32_t>(config.num_rings));
    writePod(out, static_cast<std::int32_t>(config.num_sectors));
    writePod(out, config.max_range);
    writePod(out, config.voxel_size);
    writePod(out, config.sigma_t);
    writePod(out, config.query_fov_deg);
    writePod(out, config.query_fov_center_deg);
    writePod(out, static_cast<std::int32_t>(config.branch_top_k));
    writePod(out, static_cast<std::int32_t>(config.verification_top_k));
    writePod(out, config.spatial_nms_radius);
    writePod(out, config.heading_window_deg);
    writePod(out, config.column_trim_ratio);
    writePod(out, config.sgrt_kappa);
    writePod(out, static_cast<std::int32_t>(config.exclude_recent));
    writePod(out, static_cast<std::int32_t>(config.candidates));
}

/// 按文件版本约定读取参数并验证网格尺寸上限。
bool readConfig(std::istream& in, GProbeDescriptorConfig* config) {
    std::int32_t rings = 0;
    std::int32_t sectors = 0;
    std::int32_t branch_top_k = 0;
    std::int32_t verification_top_k = 0;
    std::int32_t exclude_recent = 0;
    std::int32_t candidates = 0;
    if (config == nullptr || !readBool(in, &config->enabled) ||
        !readBool(in, &config->level_with_odom_roll_pitch) || !readPod(in, &rings) ||
        !readPod(in, &sectors) || !readPod(in, &config->max_range) ||
        !readPod(in, &config->voxel_size) || !readPod(in, &config->sigma_t) ||
        !readPod(in, &config->query_fov_deg) || !readPod(in, &config->query_fov_center_deg) ||
        !readPod(in, &branch_top_k) || !readPod(in, &verification_top_k) ||
        !readPod(in, &config->spatial_nms_radius) || !readPod(in, &config->heading_window_deg) ||
        !readPod(in, &config->column_trim_ratio) || !readPod(in, &config->sgrt_kappa) ||
        !readPod(in, &exclude_recent) || !readPod(in, &candidates)) {
        return false;
    }
    config->num_rings = rings;
    config->num_sectors = sectors;
    config->branch_top_k = branch_top_k;
    config->verification_top_k = verification_top_k;
    config->exclude_recent = exclude_recent;
    config->candidates = candidates;
    return rings > 0 && rings <= 512 && sectors >= 4 && sectors <= 720 && branch_top_k > 0 &&
           verification_top_k > 0 && exclude_recent > 0 && candidates > 0 &&
           std::isfinite(config->max_range) && config->max_range >= 1.0 &&
           std::isfinite(config->voxel_size) && config->voxel_size >= 0.0 &&
           std::isfinite(config->sigma_t) && config->sigma_t >= 0.0 &&
           std::isfinite(config->query_fov_deg) && config->query_fov_deg >= 1.0 &&
           config->query_fov_deg <= 360.0 && std::isfinite(config->query_fov_center_deg) &&
           std::abs(config->query_fov_center_deg) <= 180.0 &&
           std::isfinite(config->spatial_nms_radius) && config->spatial_nms_radius >= 0.0 &&
           std::isfinite(config->heading_window_deg) && config->heading_window_deg >= 0.0 &&
           config->heading_window_deg <= 180.0 && std::isfinite(config->column_trim_ratio) &&
           config->column_trim_ratio >= 0.0 && config->column_trim_ratio <= 0.95 &&
           std::isfinite(config->sgrt_kappa) && config->sgrt_kappa >= 0.0;
}

/// 判断持久化描述子与运行时配置是否共享同一几何模型。
bool sameGeometry(const GProbeDescriptorConfig& lhs, const GProbeDescriptorConfig& rhs) {
    /// 用持久化几何配置的数值容差比较两个标量。
    const auto same = [](const double a, const double b) { return std::abs(a - b) <= 1.0e-6; };
    return lhs.level_with_odom_roll_pitch == rhs.level_with_odom_roll_pitch &&
           lhs.num_rings == rhs.num_rings && lhs.num_sectors == rhs.num_sectors &&
           same(lhs.max_range, rhs.max_range) && same(lhs.voxel_size, rhs.voxel_size) &&
           same(lhs.sigma_t, rhs.sigma_t) && same(lhs.query_fov_deg, rhs.query_fov_deg) &&
           same(lhs.query_fov_center_deg, rhs.query_fov_center_deg);
}

/// 连续写出一个分支的高度、占用概率和不确定度网格。
void writeBranch(std::ostream& out, const Branch& branch) {
    out.write(reinterpret_cast<const char*>(branch.grid.data()),
              static_cast<std::streamsize>(branch.grid.size() * sizeof(float)));
    out.write(reinterpret_cast<const char*>(branch.mu.data()),
              static_cast<std::streamsize>(branch.mu.size() * sizeof(float)));
    out.write(reinterpret_cast<const char*>(branch.sigma.data()),
              static_cast<std::streamsize>(branch.sigma.size() * sizeof(float)));
}

/// 按预期网格大小读取一个描述子分支。
bool readBranch(std::istream& in, const std::size_t cells, Branch* branch) {
    branch->grid.resize(cells);
    branch->mu.resize(cells);
    branch->sigma.resize(cells);
    in.read(reinterpret_cast<char*>(branch->grid.data()),
            static_cast<std::streamsize>(cells * sizeof(float)));
    in.read(reinterpret_cast<char*>(branch->mu.data()),
            static_cast<std::streamsize>(cells * sizeof(float)));
    in.read(reinterpret_cast<char*>(branch->sigma.data()),
            static_cast<std::streamsize>(cells * sizeof(float)));
    if (!in)
        return false;
    for (std::size_t cell = 0; cell < cells; ++cell) {
        if (!std::isfinite(branch->grid[cell]) || !std::isfinite(branch->mu[cell]) ||
            branch->mu[cell] < 0.0F || branch->mu[cell] > 1.0F ||
            !std::isfinite(branch->sigma[cell]) || branch->sigma[cell] < 0.0F ||
            branch->sigma[cell] > 0.5F)
            return false;
    }
    return true;
}

}  // namespace

/// 将配置与全部关键帧描述子写入版本化二进制文件。
bool saveBundle(const std::string& directory, const GProbeDescriptorConfig& config,
                const std::vector<FrameDescriptor>& frames) {
    std::ofstream out(descriptorPath(directory), std::ios::binary | std::ios::trunc);
    if (!out.is_open()) {
        std::cerr << "Failed to write " << descriptorPath(directory) << '\n';
        return false;
    }
    writePod(out, kFileMagic);
    writePod(out, kFileVersion);
    writeConfig(out, config);
    writePod(out, static_cast<std::uint64_t>(frames.size()));
    for (const FrameDescriptor& frame : frames) {
        writePod(out, frame.position.x());
        writePod(out, frame.position.y());
        writePod(out, frame.position.z());
        for (const Branch& branch : frame.map_branches) {
            writeBranch(out, branch);
        }
        writeBranch(out, frame.query_branch);
    }
    out.close();
    return static_cast<bool>(out);
}

/// 读取版本化描述子文件并拒绝不兼容的几何配置。
bool loadBundle(const std::string& directory, const GProbeDescriptorConfig& expected_config,
                std::vector<FrameDescriptor>* frames) {
    if (frames == nullptr) {
        return false;
    }
    std::ifstream in(descriptorPath(directory), std::ios::binary | std::ios::ate);
    if (!in.is_open()) {
        std::cerr << "Failed to open " << descriptorPath(directory) << '\n';
        return false;
    }
    const auto end = in.tellg();
    in.seekg(0);
    std::uint32_t magic = 0U;
    std::uint32_t version = 0U;
    GProbeDescriptorConfig stored;
    std::uint64_t count = 0U;
    if (!readPod(in, &magic) || !readPod(in, &version) || magic != kFileMagic ||
        version != kFileVersion || !readConfig(in, &stored) || !readPod(in, &count) ||
        count > 1000000U) {
        std::cerr << "Invalid G-PROBE descriptor file " << descriptorPath(directory) << '\n';
        return false;
    }
    if (!sameGeometry(expected_config, stored)) {
        std::cerr << "G-PROBE map geometry does not match runtime config\n";
        return false;
    }

    const std::size_t cells = static_cast<std::size_t>(stored.num_rings * stored.num_sectors);
    const std::uint64_t frame_bytes =
        3 * sizeof(double) + (kMapBranchCount + 1) * 3 * cells * sizeof(float);
    const auto position = in.tellg();
    if (position < 0 || end < position)
        return false;
    const auto remaining = static_cast<std::uint64_t>(end - position);
    if (count > remaining / frame_bytes || count * frame_bytes != remaining)
        return false;
    std::vector<FrameDescriptor> loaded(static_cast<std::size_t>(count));
    for (FrameDescriptor& frame : loaded) {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        if (!readPod(in, &x) || !readPod(in, &y) || !readPod(in, &z) || !std::isfinite(x) ||
            !std::isfinite(y) || !std::isfinite(z)) {
            return false;
        }
        frame.position = Eigen::Vector3d(x, y, z);
        for (Branch& branch : frame.map_branches) {
            if (!readBranch(in, cells, &branch)) {
                return false;
            }
        }
        if (!readBranch(in, cells, &frame.query_branch)) {
            return false;
        }
    }
    *frames = std::move(loaded);
    return true;
}

/// 从版本化描述子文件仅读取配置。
bool loadBundleConfig(const std::string& directory, GProbeDescriptorConfig* config) {
    if (config == nullptr)
        return false;
    std::ifstream in(descriptorPath(directory), std::ios::binary);
    std::uint32_t magic = 0U;
    std::uint32_t version = 0U;
    GProbeDescriptorConfig loaded;
    if (!in.is_open() || !readPod(in, &magic) || !readPod(in, &version) || magic != kFileMagic ||
        version != kFileVersion || !readConfig(in, &loaded))
        return false;
    *config = loaded;
    return true;
}

}  // namespace backend::gprobe_detail
