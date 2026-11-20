/**
 * @file directory.hpp
 * @brief 管理地图目录的独占暂存、同步落盘及原子提交。
 */
#pragma once

#include <filesystem>
#include <string>

namespace backend {
/// 返回路径的稳定绝对表示。
std::filesystem::path normalizedAbsolutePath(const std::string& path);
/// 判断两个非空路径是否指向同一位置。
bool pathsReferToSameLocation(const std::string& lhs, const std::string& rhs);

/// 独占暂存目录，在文件和目录同步完成后原子安装目标地图。
class DirectoryTransaction {
public:
    enum class Policy { Create, Replace };
    /// 设置输出路径与已有目标的处理方式。
    explicit DirectoryTransaction(const std::string& target, Policy policy = Policy::Create);
    /// 清理本事务拥有的暂存内容，保留持久化确认失败时的旧地图。
    ~DirectoryTransaction();
    /// 禁止复制目录的唯一所有权。
    DirectoryTransaction(const DirectoryTransaction&) = delete;
    /// 禁止复制目录的唯一所有权。
    DirectoryTransaction& operator=(const DirectoryTransaction&) = delete;

    /// 创建独占工作目录及其待提交内容目录。
    bool prepare(std::string* message = nullptr);
    /// 返回最终输出路径。
    const std::filesystem::path& targetPath() const;
    /// 返回本次写入使用的独占内容目录。
    const std::filesystem::path& stagingPath() const;
    /// 同步暂存内容并原子安装；发布后的同步失败保留旧图并报告其位置。
    bool commit(std::string* message = nullptr);

private:
    /// 检查新建模式下目标不存在或为空目录。
    bool acceptEmptyTarget(std::string* message) const;
    /// 写出失败原因并返回失败状态。
    static bool fail(std::string* destination, const std::string& message);

    std::filesystem::path target_, workspace_, staging_, parent_anchor_;
    Policy policy_;
    bool prepared_ = false;
    bool published_ = false;
    bool preserve_previous_ = false;
};
}  // namespace backend
