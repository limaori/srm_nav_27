// 几何驱动仅读取公开快照，不接触标签；原点索引通过 intensity 无损携带。
#include <Eigen/Geometry>
#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "backend/map_bundle.hpp"
#include "cleanup/cleanup.hpp"

namespace {
template <class Value>
Value read(std::istream& input) {
    Value value{};
    if (!input.read(reinterpret_cast<char*>(&value), sizeof(value)))
        throw std::runtime_error("truncated geometry snapshot");
    return value;
}

int run(const std::filesystem::path& input_path, const std::filesystem::path& output, int stride) {
    if (stride < 1 || std::filesystem::exists(output))
        throw std::invalid_argument("positive stride and absent output directory required");
    std::ifstream input(input_path, std::ios::binary);
    std::array<char, 8> magic{};
    input.read(magic.data(), magic.size());
    if (std::string(magic.data(), magic.size()) != "ZLIOMOS1")
        throw std::runtime_error("unsupported geometry snapshot format");
    std::vector<backend::Keyframe> frames;
    std::vector<std::vector<std::uint8_t>> masks;
    std::size_t ordinal = 0;
    while (input.peek() != std::char_traits<char>::eof()) {
        const auto stamp = read<std::int64_t>(input);
        const auto sequence = read<std::uint32_t>(input);
        const auto count = read<std::uint32_t>(input);
        if (count == 0 || count > 1000000 || sequence != ordinal)
            throw std::runtime_error("invalid snapshot sequence or size");
        const auto pose = read<std::array<double, 7>>(input);
        backend::Keyframe frame;
        frame.index = static_cast<int>(frames.size());
        frame.timestamp = static_cast<double>(stamp) * 1e-9;
        frame.optimized_pose.block<3, 1>(0, 3) = Eigen::Vector3d(pose[0], pose[1], pose[2]);
        frame.optimized_pose.block<3, 3>(0, 0) =
            Eigen::Quaterniond(pose[6], pose[3], pose[4], pose[5]).normalized().toRotationMatrix();
        frame.odom_pose = frame.optimized_pose;
        backend::Cloud::Ptr cloud(new backend::Cloud);
        std::vector<std::uint8_t> mask(count, 255);
        const bool selected = ordinal % static_cast<std::size_t>(stride) == 0;
        for (std::uint32_t index = 0; index < count; ++index) {
            const auto xyz = read<std::array<float, 3>>(input);
            const Eigen::Vector3d point(xyz[0], xyz[1], xyz[2]);
            if (!selected || !point.allFinite() || point.norm() < .5 || point.norm() > 100)
                continue;
            backend::PointType value;
            value.x = xyz[0];
            value.y = xyz[1];
            value.z = xyz[2];
            value.intensity = static_cast<float>(index);
            cloud->push_back(value);
            mask[index] = 0;
        }
        if (selected) {
            if (cloud->empty())
                throw std::runtime_error("selected snapshot has no valid points");
            frame.cloud = backend::CloudBlock(cloud);
            frames.push_back(frame);
            masks.push_back(std::move(mask));
        }
        ++ordinal;
    }
    const auto cleaned = backend::tools::cleanup::cleanFrames(frames, {});
    for (std::size_t index = 0; index < cleaned.frames.size(); ++index) {
        auto& mask = masks.at(static_cast<std::size_t>(cleaned.source_indices[index]));
        for (const auto& point : *cleaned.frames[index].cloud.load())
            mask.at(static_cast<std::size_t>(point.intensity)) = 1;
    }
    std::filesystem::create_directory(output);
    std::ofstream mask_file(output / "mask.bin", std::ios::binary);
    for (const auto& mask : masks)
        mask_file.write(reinterpret_cast<const char*>(mask.data()), mask.size());
    mask_file.close();
    if (!mask_file || !backend::saveMapCloud((output / "map.pcd").string(), cleaned.frames, .05))
        throw std::runtime_error("cannot write evaluation output");
    backend::tools::cleanup::saveReport(output.string(), {}, cleaned);
    std::cout << "frames=" << frames.size() << " points=" << cleaned.statistics.input_points
              << " removed=" << cleaned.statistics.removed_points << '\n';
    return 0;
}
}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc != 4)
            throw std::invalid_argument("usage: cleanup_dataset geometry.bin output stride");
        return run(argv[1], argv[2], std::stoi(argv[3]));
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
