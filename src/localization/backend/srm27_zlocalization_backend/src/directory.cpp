/**
 * @file directory.cpp
 * @brief 实现地图目录的同步落盘、原子安装及旧版本保留。
 */
#include "backend/directory.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace backend {
namespace {
/// 同步单个文件或目录，并在释放描述符后报告写回及关闭错误。
void synchronizePath(const std::filesystem::path& path, bool directory) {
    const int flags = O_RDONLY | O_CLOEXEC | (directory ? O_DIRECTORY : O_NOFOLLOW);
    const int descriptor = ::open(path.c_str(), flags);
    if (descriptor < 0)
        throw std::system_error(errno, std::generic_category(),
                                "cannot open for sync: " + path.string());
    int result;
    do {
        result = ::fsync(descriptor);
    } while (result < 0 && errno == EINTR);
    const int sync_error = result < 0 ? errno : 0;
    const int close_result = ::close(descriptor);
    const int close_error = close_result < 0 ? errno : 0;
    if (sync_error || close_error)
        throw std::system_error(sync_error ? sync_error : close_error, std::generic_category(),
                                "cannot synchronize: " + path.string());
}

/// 同步暂存树的普通文件及各级目录，拒绝符号链接和特殊文件。
void synchronizeTree(const std::filesystem::path& directory) {
    for (const auto& entry : std::filesystem::directory_iterator(directory)) {
        const auto status = entry.symlink_status();
        if (std::filesystem::is_regular_file(status))
            synchronizePath(entry.path(), false);
        else if (std::filesystem::is_directory(status))
            synchronizeTree(entry.path());
        else
            throw std::runtime_error("unsupported staging entry: " + entry.path().string());
    }
    synchronizePath(directory, true);
}

/// 自底向上同步输出父目录及本事务新建的祖先目录。
void synchronizeParents(std::filesystem::path directory, const std::filesystem::path& anchor) {
    for (;;) {
        synchronizePath(directory, true);
        if (directory == anchor)
            return;
        directory = directory.parent_path();
    }
}
}  // namespace

/// 返回路径的稳定绝对表示。
std::filesystem::path normalizedAbsolutePath(const std::string& path) {
    std::error_code error;
    auto absolute = std::filesystem::absolute(path, error);
    if (error)
        return std::filesystem::path(path).lexically_normal();
    auto canonical = std::filesystem::weakly_canonical(absolute, error);
    return error ? absolute.lexically_normal() : canonical;
}

/// 判断两个非空路径是否指向同一位置。
bool pathsReferToSameLocation(const std::string& lhs, const std::string& rhs) {
    return !lhs.empty() && !rhs.empty() &&
           normalizedAbsolutePath(lhs) == normalizedAbsolutePath(rhs);
}

/// 设置输出路径与已有目标的处理方式。
DirectoryTransaction::DirectoryTransaction(const std::string& target, Policy policy)
    : target_(policy == Policy::Create ? normalizedAbsolutePath(target)
                                       : std::filesystem::absolute(target).lexically_normal()),
      policy_(policy) {
}

/// 清理本事务拥有的暂存内容，保留持久化确认失败时的旧地图。
DirectoryTransaction::~DirectoryTransaction() {
    std::error_code ignored;
    if (!workspace_.empty() && !preserve_previous_)
        std::filesystem::remove_all(workspace_, ignored);
}

/// 创建独占工作目录及其待提交内容目录。
bool DirectoryTransaction::prepare(std::string* message) {
    if (!workspace_.empty())
        return prepared_;
    if (target_.empty() || target_ == target_.root_path() || target_.filename().empty())
        return fail(message, "unsafe output directory");
    if (policy_ == Policy::Create && !acceptEmptyTarget(message))
        return false;
    std::error_code error;
    parent_anchor_ = target_.parent_path();
    while (!std::filesystem::exists(parent_anchor_, error)) {
        if (error)
            return fail(message, error.message());
        const auto parent = parent_anchor_.parent_path();
        if (parent == parent_anchor_)
            return fail(message, "cannot locate an existing output ancestor");
        parent_anchor_ = parent;
    }
    if (error)
        return fail(message, error.message());
    std::filesystem::create_directories(target_.parent_path(), error);
    if (error)
        return fail(message, error.message());
    // 只有成功创建的独占目录才进入本事务的清理范围。
    const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
    for (int attempt = 0; attempt < 16; ++attempt) {
        auto candidate =
            target_.parent_path() / (target_.filename().string() + ".staging." +
                                     std::to_string(nonce) + "." + std::to_string(attempt));
        if (std::filesystem::create_directory(candidate, error)) {
            workspace_ = std::move(candidate);
            staging_ = workspace_ / "content";
            prepared_ = std::filesystem::create_directory(staging_, error);
            return prepared_ || fail(message, error.message());
        }
        if (error && error != std::errc::file_exists)
            return fail(message, error.message());
        error.clear();
    }
    return fail(message, "cannot allocate a staging directory");
}

/// 返回最终输出路径。
const std::filesystem::path& DirectoryTransaction::targetPath() const {
    return target_;
}

/// 返回本次写入使用的独占内容目录。
const std::filesystem::path& DirectoryTransaction::stagingPath() const {
    return staging_;
}

/// 同步暂存内容并原子安装；发布后的同步失败保留旧图并报告其位置。
bool DirectoryTransaction::commit(std::string* message) {
    if (!prepared_ || published_)
        return fail(message, "output transaction is not prepared");
    try {
        synchronizeTree(staging_);
        synchronizePath(workspace_, true);
        synchronizeParents(target_.parent_path(), parent_anchor_);

        const bool exists = std::filesystem::exists(target_);
        const bool exchange = policy_ == Policy::Replace && exists;
        if (policy_ == Policy::Create) {
            if (!acceptEmptyTarget(message))
                return false;
            if (exists && !std::filesystem::remove(target_))
                return fail(message, "cannot remove empty output");
        }
        const unsigned flags = exchange ? RENAME_EXCHANGE : RENAME_NOREPLACE;
        if (::renameat2(AT_FDCWD, staging_.c_str(), AT_FDCWD, target_.c_str(), flags) < 0)
            throw std::system_error(errno, std::generic_category(), "cannot publish map directory");
        published_ = true;
        preserve_previous_ = exchange;

        // 原子交换将旧图留在 content；两个父目录均同步后才允许清理旧版本。
        synchronizePath(workspace_, true);
        synchronizePath(target_.parent_path(), true);
        preserve_previous_ = false;
        return true;
    } catch (const std::exception& error) {
        if (!published_)
            return fail(message, error.what());
        return fail(message,
                    "map installed at " + target_.string() +
                        "; durability not confirmed: " + error.what() +
                        (preserve_previous_ ? "; previous map retained at " + staging_.string()
                                            : std::string{}));
    }
}

/// 检查新建模式下目标不存在或为空目录。
bool DirectoryTransaction::acceptEmptyTarget(std::string* message) const {
    std::error_code error;
    const bool exists = std::filesystem::exists(target_, error);
    if (error)
        return fail(message, error.message());
    if (exists && (!std::filesystem::is_directory(target_, error) ||
                   !std::filesystem::is_empty(target_, error)))
        return fail(message, error ? error.message() : "output directory is not empty");
    return !error || fail(message, error.message());
}

/// 写出失败原因并返回失败状态。
bool DirectoryTransaction::fail(std::string* destination, const std::string& message) {
    if (destination)
        *destination = message;
    return false;
}

}  // namespace backend
