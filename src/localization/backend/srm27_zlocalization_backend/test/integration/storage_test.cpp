// 持久化测试跨真实文件系统和重载接口验证一致性与失败边界。
#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <fstream>
#include <future>
#include <iterator>
#include <thread>

#include "backend/cache.hpp"
#include "backend/directory.hpp"
#include "backend/keyframe_map.hpp"
#include "backend/map_bundle.hpp"
#include "backend/scan_context.hpp"
#include "geometry.hpp"
#include "temporary_directory.hpp"

namespace {
enum class SyncFailure { None, File, PublishedDirectory, ExitAfterPublication };
thread_local SyncFailure sync_failure = SyncFailure::None;
thread_local const std::filesystem::path* publication_marker = nullptr;
thread_local const std::filesystem::path* failure_directory = nullptr;
thread_local int sync_failures = 0;

/// 将同步故障限定在当前测试线程和一次提交范围内。
class SyncFailureScope {
public:
    /// 指定当前提交的故障类型、发布标记和目录同步边界。
    explicit SyncFailureScope(SyncFailure mode, const std::filesystem::path& marker = {},
                              const std::filesystem::path& directory = {})
        : marker_(marker), directory_(directory) {
        sync_failure = mode;
        publication_marker = &marker_;
        failure_directory = &directory_;
        sync_failures = 0;
    }
    /// 恢复真实同步调用，避免影响后续测试。
    ~SyncFailureScope() {
        sync_failure = SyncFailure::None;
        publication_marker = nullptr;
        failure_directory = nullptr;
    }
    /// 禁止复制当前线程故障作用域及其成员地址借用。
    SyncFailureScope(const SyncFailureScope&) = delete;
    /// 禁止复制当前线程故障作用域的清理责任。
    SyncFailureScope& operator=(const SyncFailureScope&) = delete;

private:
    std::filesystem::path marker_, directory_;
};

/// 判断本次同步是否命中指定的文件或发布后目录故障阶段。
bool matchesSyncFailure(int descriptor) {
    if (sync_failure == SyncFailure::None)
        return false;
    struct stat opened {};
    if (::fstat(descriptor, &opened) != 0)
        return false;
    if (sync_failure == SyncFailure::File)
        return S_ISREG(opened.st_mode);
    std::error_code error;
    if (!S_ISDIR(opened.st_mode) || !std::filesystem::exists(*publication_marker, error) || error)
        return false;
    if (sync_failure == SyncFailure::ExitAfterPublication)
        return true;
    struct stat expected {};
    return ::stat(failure_directory->c_str(), &expected) == 0 && opened.st_dev == expected.st_dev &&
           opened.st_ino == expected.st_ino;
}
}  // namespace

/// 调用链接器保留的真实文件同步入口。
extern "C" int __real_fsync(int descriptor);

/// 在指定提交阶段注入写回错误或进程退出，其余调用使用真实文件系统。
extern "C" int __wrap_fsync(int descriptor) {
    if (matchesSyncFailure(descriptor)) {
        if (sync_failure == SyncFailure::ExitAfterPublication)
            ::_exit(73);
        ++sync_failures;
        errno = EIO;
        return -1;
    }
    return __real_fsync(descriptor);
}

namespace backend {
namespace {
namespace fs = std::filesystem;
/// 读取完整文件字节，用于比较地图提交前后的内容。
std::string read(const fs::path& path) {
    std::ifstream stream(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

/// 为每个持久化场景创建独立的临时地图目录。
class MapStorage : public ::testing::Test {
protected:
    testing::TemporaryDirectory temporary;
    const fs::path destination = temporary.path() / "bundle";
};

/// 同键并发请求共用一次加载，并观察相同的加载结果或异常。
TEST(MapCache, ConcurrentMissesShareOneLoadAndTheSameFailureOutcome) {
    using namespace std::chrono_literals;
    for (bool fail : {false, true}) {
        SCOPED_TRACE(fail);
        CloudCache<int> cache(1024U * 1024U);
        std::atomic<int> loads{0};
        std::promise<void> started, release;
        const auto ready = started.get_future();
        const auto released = release.get_future().share();
        const auto cloud = testing::planeCloud();
        auto first = std::async(std::launch::async, [&] {
            return cache.get(1, [&]() -> Cloud::ConstPtr {
                ++loads;
                started.set_value();
                released.wait();
                if (fail)
                    throw std::runtime_error("shared read failed");
                return cloud;
            });
        });
        const bool loader_started = ready.wait_for(2s) == std::future_status::ready;
        auto second = std::async(std::launch::async, [&] {
            return cache.get(1, [&] {
                ++loads;
                return testing::planeCloud();
            });
        });
        const auto deadline = std::chrono::steady_clock::now() + 2s;
        while (cache.stats().waits == 0 && std::chrono::steady_clock::now() < deadline)
            std::this_thread::yield();
        const bool waiter_joined = cache.stats().waits == 1;
        // 先解除阻塞再断言；即使调度或合同检查失败，也能汇合全部线程。
        release.set_value();
        EXPECT_TRUE(loader_started);
        EXPECT_TRUE(waiter_joined);
        if (fail) {
            EXPECT_THROW(first.get(), std::runtime_error);
            EXPECT_THROW(second.get(), std::runtime_error);
            EXPECT_EQ(cache.get(1, [&] { return cloud; }), cloud);
        } else {
            EXPECT_EQ(first.get(), cloud);
            EXPECT_EQ(second.get(), cloud);
        }
        EXPECT_EQ(loads.load(), 1);
    }
}

/// 加载期间清空缓存后，旧加载结果只能满足原请求，不能填回新代缓存。
TEST(MapCache, ClearDuringLoadCannotOverwriteNewGeneration) {
    CloudCache<int> cache(1024U * 1024U);
    const auto old_value = testing::planeCloud();
    const auto new_value = testing::planeCloud();
    new_value->front().intensity = 999;
    const auto old_lease = cache.get(1, [&] {
        cache.clear();
        cache.get(1, [&] { return new_value; });
        return old_value;
    });
    const auto current = cache.get(1, []() -> Cloud::ConstPtr {
        throw std::runtime_error("current generation should already be cached");
    });
    ASSERT_TRUE(old_lease);
    ASSERT_TRUE(current);
    EXPECT_FLOAT_EQ(old_lease->front().intensity, 0);
    EXPECT_FLOAT_EQ(current->front().intensity, 999);
    EXPECT_EQ(cache.stats().loads, 2U);
}

/// 加载失败后允许重试；缓存逐出后已有点云租约仍有效。
TEST(MapCache, FailedLoadCanBeRetriedAndEvictionPreservesBorrowedGeometry) {
    CloudCache<int> cache(16384, 1);
    EXPECT_THROW(cache.get(1, []() -> Cloud::ConstPtr { throw std::runtime_error("read failed"); }),
                 std::runtime_error);
    const auto first = cache.get(1, [] { return testing::planeCloud(); });
    const auto second = cache.get(2, [] { return testing::planeCloud(); });
    EXPECT_EQ(cache.stats().entries, 1U);
    EXPECT_EQ(cache.stats().evictions, 1U);
    cache.clear();
    EXPECT_EQ(cache.stats().bytes, 0U);
    ASSERT_TRUE(first);
    ASSERT_TRUE(second);
    EXPECT_FALSE(first->empty());
    EXPECT_FALSE(second->empty());
    EXPECT_EQ(cache.stats().loads, 3U);
}

/// 中止或失败的地图目录事务应保留原目标内容。
TEST_F(MapStorage, DirectoryAbortAndFailedReplacementKeepPreviousContents) {
    fs::create_directory(destination);
    std::ofstream(destination / "original") << "previous map";
    {
        DirectoryTransaction output(destination.string(), DirectoryTransaction::Policy::Replace);
        ASSERT_TRUE(output.prepare());
        std::ofstream(output.stagingPath() / "new") << "partial map";
    }
    EXPECT_EQ(read(destination / "original"), "previous map");
    {
        DirectoryTransaction output(destination.string(), DirectoryTransaction::Policy::Replace);
        ASSERT_TRUE(output.prepare());
        // 删除本测试拥有的暂存内容，验证提交失败时旧图不受影响。
        fs::remove_all(output.stagingPath());
        EXPECT_FALSE(output.commit());
        EXPECT_EQ(read(destination / "original"), "previous map");
    }
    EXPECT_EQ(std::distance(fs::directory_iterator(temporary.path()), fs::directory_iterator()), 1);
}

/// 提交时再次检查新建目标是否为空，移除冲突后同一事务可重试。
TEST_F(MapStorage, CreateChecksDestinationAgainAtCommitAndRetrySucceeds) {
    DirectoryTransaction output(destination.string());
    ASSERT_TRUE(output.prepare());
    std::ofstream(output.stagingPath() / "new") << "new map";
    fs::create_directory(destination);
    std::ofstream(destination / "concurrent") << "another writer";
    EXPECT_FALSE(output.commit());
    EXPECT_EQ(read(destination / "concurrent"), "another writer");
    fs::remove(destination / "concurrent");
    EXPECT_TRUE(output.commit());
    EXPECT_EQ(read(destination / "new"), "new map");
    EXPECT_FALSE(output.commit());
}

/// 文件同步失败保持旧图不变，解除故障后同一事务可重新提交。
TEST_F(MapStorage, FailedFileSyncPreservesPreviousMapAndAllowsRetry) {
    fs::create_directory(destination);
    std::ofstream(destination / "original") << "previous map";
    fs::path workspace;
    {
        DirectoryTransaction output(destination.string(), DirectoryTransaction::Policy::Replace);
        ASSERT_TRUE(output.prepare());
        workspace = output.stagingPath().parent_path();
        std::ofstream(output.stagingPath() / "new") << "replacement map";
        {
            SyncFailureScope failure(SyncFailure::File);
            std::string error;
            EXPECT_FALSE(output.commit(&error));
            EXPECT_GT(sync_failures, 0);
            EXPECT_FALSE(error.empty());
            EXPECT_EQ(read(destination / "original"), "previous map");
            EXPECT_FALSE(fs::exists(destination / "new"));
        }
        EXPECT_TRUE(output.commit());
        EXPECT_EQ(read(destination / "new"), "replacement map");
        EXPECT_FALSE(fs::exists(destination / "original"));
    }
    EXPECT_FALSE(fs::exists(workspace));
}

/// 任一父目录在发布后同步失败时，事务析构仍保留新旧两个完整版本。
TEST_F(MapStorage, PublicationSyncFailureRetainsBothVersionsAfterDestruction) {
    for (const bool workspace_sync : {false, true}) {
        SCOPED_TRACE(workspace_sync);
        const auto target = temporary.path() / (workspace_sync ? "workspace" : "parent");
        fs::create_directory(target);
        std::ofstream(target / "original") << "previous map";
        fs::path previous;
        {
            DirectoryTransaction output(target.string(), DirectoryTransaction::Policy::Replace);
            ASSERT_TRUE(output.prepare());
            previous = output.stagingPath();
            std::ofstream(previous / "new") << "replacement map";
            const auto directory = workspace_sync ? previous.parent_path() : target.parent_path();
            SyncFailureScope failure(SyncFailure::PublishedDirectory, target / "new", directory);
            std::string error;
            EXPECT_FALSE(output.commit(&error));
            EXPECT_GT(sync_failures, 0);
            EXPECT_NE(error.find("durability not confirmed"), std::string::npos);
            EXPECT_NE(error.find(previous.string()), std::string::npos);
            EXPECT_FALSE(output.commit());
        }
        EXPECT_EQ(read(target / "new"), "replacement map");
        EXPECT_EQ(read(previous / "original"), "previous map");
    }
}

/// 首次发布后的同步失败报告不确定持久性，同时保留已安装的新地图。
TEST_F(MapStorage, InitialPublicationSyncFailureDoesNotReportSuccessOrRemovePublishedMap) {
    {
        DirectoryTransaction output(destination.string());
        ASSERT_TRUE(output.prepare());
        std::ofstream(output.stagingPath() / "new") << "new map";
        SyncFailureScope failure(SyncFailure::PublishedDirectory, destination / "new",
                                 destination.parent_path());
        std::string error;
        EXPECT_FALSE(output.commit(&error));
        EXPECT_GT(sync_failures, 0);
        EXPECT_NE(error.find("durability not confirmed"), std::string::npos);
    }
    EXPECT_EQ(read(destination / "new"), "new map");
}

/// 原子发布后立即退出进程，目标路径及暂存旧版本均保持完整。
TEST_F(MapStorage, ProcessExitAfterPublicationLeavesACompleteTargetAndPreviousMap) {
    fs::create_directory(destination);
    std::ofstream(destination / "original") << "previous map";
    DirectoryTransaction output(destination.string(), DirectoryTransaction::Policy::Replace);
    ASSERT_TRUE(output.prepare());
    std::ofstream(output.stagingPath() / "new") << "replacement map";
    const pid_t child = ::fork();
    ASSERT_GE(child, 0);
    if (child == 0) {
        SyncFailureScope failure(SyncFailure::ExitAfterPublication, destination / "new");
        output.commit();
        ::_exit(74);
    }
    int status = 0;
    ASSERT_EQ(::waitpid(child, &status, 0), child);
    ASSERT_TRUE(WIFEXITED(status));
    EXPECT_EQ(WEXITSTATUS(status), 73);
    EXPECT_EQ(read(destination / "new"), "replacement map");
    EXPECT_EQ(read(output.stagingPath() / "original"), "previous map");
}

/// 地图快照保存并重载后，关键帧位姿、局部点云和描述子保持一致。
TEST_F(MapStorage, SnapshotRoundtripKeepsGeometryPoseAndDescriptorInAgreement) {
    KeyframeMap trajectory;
    trajectory.add(testing::frame(1));
    trajectory.add(testing::frame(2, 3));
    const auto before = trajectory.snapshot();
    ScanContextDescriptor descriptor;
    descriptor.configure({});
    ASSERT_TRUE(saveMapBundle(destination.string(), trajectory, descriptor, {true, 0}));
    KeyframeMap loaded;
    ASSERT_TRUE(loaded.load(destination.string(), 0));
    const auto after = loaded.snapshot();
    ASSERT_EQ(after.size(), before.size());
    ScanContextDescriptor restored;
    restored.configure(descriptor.config());
    ASSERT_TRUE(restored.load(destination.string()));
    EXPECT_EQ(restored.size(), after.size());
    Cloud global;
    ASSERT_EQ(pcl::io::loadPCDFile((destination / "map.pcd").string(), global), 0);
    const auto expected = KeyframeMap::buildOptimizedMapFromSnapshot(after);
    ASSERT_EQ(global.size(), expected->size());
    for (std::size_t index = 0; index < after.size(); ++index) {
        EXPECT_EQ(after[index].index, before[index].index);
        EXPECT_DOUBLE_EQ(after[index].timestamp, before[index].timestamp);
        EXPECT_LT((after[index].optimized_pose - before[index].optimized_pose).norm(), 1e-12);
        const auto source = before[index].cloud.load();
        const auto restored_cloud = after[index].cloud.load();
        ASSERT_EQ(source->size(), restored_cloud->size());
        for (std::size_t point = 0; point < source->size(); ++point) {
            EXPECT_FLOAT_EQ((*source)[point].x, (*restored_cloud)[point].x);
            EXPECT_FLOAT_EQ((*source)[point].intensity, (*restored_cloud)[point].intensity);
        }
    }
    for (std::size_t index = 0; index < global.size(); ++index)
        EXPECT_LT((global[index].getVector3fMap() - (*expected)[index].getVector3fMap()).norm(),
                  1e-6);
}

/// 地图加载失败保留已有记录，清空后已取得的点云租约仍可读取。
TEST_F(MapStorage, FailedLoadKeepsExistingMapAndLeasesSurviveClear) {
    KeyframeMap original;
    original.add(testing::frame(1));
    ASSERT_TRUE(original.save(destination.string()));
    KeyframeMap loaded;
    ASSERT_TRUE(loaded.load(destination.string(), 0));
    const auto lease = loaded.latest().cloud.load();
    const auto revision = loaded.revision();
    std::ofstream(destination / "keyframes.txt") << "# backend_map 999\n";
    EXPECT_FALSE(loaded.load(destination.string()));
    EXPECT_EQ(loaded.revision(), revision);
    EXPECT_EQ(loaded.size(), 1U);
    loaded.clear();
    EXPECT_FALSE(lease->empty());
    EXPECT_FLOAT_EQ(lease->front().intensity, 0);
}

/// 空地图保存失败，已有有效地图包保持完整。
TEST_F(MapStorage, EmptySaveCannotReplaceValidBundle) {
    KeyframeMap trajectory;
    trajectory.add(testing::frame(1));
    ScanContextDescriptor descriptor;
    descriptor.configure({});
    ASSERT_TRUE(saveMapBundle(destination.string(), trajectory, descriptor, {}));
    const auto index = read(destination / "keyframes.txt");
    trajectory.clear();
    EXPECT_FALSE(saveMapBundle(destination.string(), trajectory, descriptor, {}));
    EXPECT_EQ(read(destination / "keyframes.txt"), index);
}

/// 优化位姿更新空间查询索引，先前取得的地图快照保持原值。
TEST_F(MapStorage, SpatialIndexTracksOptimizedPoseAndSnapshotsRemainImmutable) {
    KeyframeMap trajectory;
    trajectory.add(testing::frame(1, 0));
    trajectory.add(testing::frame(2, 3));
    const auto snapshot = trajectory.snapshot();
    EXPECT_EQ(trajectory.nearestKeyframeIndices(Eigen::Vector3d::Zero(), 1), (std::vector<int>{0}));
    trajectory.updateOptimizedPoses({{0, testing::translation(10)}});
    EXPECT_EQ(trajectory.nearestKeyframeIndices(Eigen::Vector3d::Zero(), 1), (std::vector<int>{1}));
    EXPECT_DOUBLE_EQ(snapshot[0].optimized_pose(0, 3), 0);
    trajectory.retainLatest(1);
    EXPECT_EQ(trajectory.add(testing::frame(3, 4)).index, 2);
}
}  // namespace
}  // namespace backend
