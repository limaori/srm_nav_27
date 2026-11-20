/**
 * @file temporary_directory.hpp
 * @brief 独占测试临时目录并在作用域结束时安全清理。
 */
#pragma once

#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace backend::testing {
class TemporaryDirectory {
public:
    /// 在系统临时目录下创建独占工作目录。
    TemporaryDirectory() {
        const auto pattern =
            (std::filesystem::temp_directory_path() / "backend-test-XXXXXX").string();
        std::vector<char> buffer(pattern.begin(), pattern.end());
        buffer.push_back('\0');
        const char* created = ::mkdtemp(buffer.data());
        if (!created)
            throw std::runtime_error("cannot create test directory");
        path_ = created;
    }
    /// 清理本实例创建的目录，析构不传播文件系统异常。
    ~TemporaryDirectory() {
        std::error_code ignored;
        std::filesystem::remove_all(path_, ignored);
    }
    /// 禁止复制临时目录的独占所有权。
    TemporaryDirectory(const TemporaryDirectory&) = delete;
    /// 禁止赋值复制临时目录的独占所有权。
    TemporaryDirectory& operator=(const TemporaryDirectory&) = delete;
    /// 返回临时工作目录路径。
    const std::filesystem::path& path() const { return path_; }

private:
    std::filesystem::path path_;
};
}  // namespace backend::testing
