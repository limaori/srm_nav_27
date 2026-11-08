// ROS 边界验证显式字段、物理单位及输入队列的关闭语义。
#include <gtest/gtest.h>

#include <array>
#include <cstring>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <stdexcept>

#include "apps/input.hpp"
#include "apps/ros.hpp"
#include "execution.hpp"

namespace zlio::ros {
namespace {
TEST(PointCloudDecoder, DecodesPointTimesAndRejectsTruncatedPayload) {
    sensor_msgs::msg::PointCloud2 message;
    message.header.stamp.sec = 2;
    sensor_msgs::PointCloud2Modifier modifier(message);
    modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                  sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                  sensor_msgs::msg::PointField::FLOAT32, "offset_time", 1,
                                  sensor_msgs::msg::PointField::UINT32);
    modifier.resize(2);
    for (std::size_t index = 0; index < 2; ++index) {
        const float xyz[] = {1.F + static_cast<float>(index), 2.F, 3.F};
        const std::uint32_t offset = static_cast<std::uint32_t>(index) * 1000U;
        std::memcpy(message.data.data() + index * message.point_step, xyz, sizeof(xyz));
        std::memcpy(message.data.data() + index * message.point_step + 12, &offset, sizeof(offset));
    }
    PointCloudDecoder decoder({});
    const auto points = decoder.decode(message);
    ASSERT_EQ(points.size(), 2U);
    EXPECT_EQ(points[0].time, nanoseconds(2));
    EXPECT_EQ(points[1].time, nanoseconds(2) + 1000);
    EXPECT_DOUBLE_EQ(points[1].position.x(), 2);
    message.data.pop_back();
    EXPECT_THROW(decoder.decode(message), std::exception);
}

TEST(InputQueue, DrainAcceptsAdmittedWorkAndRejectsNewCallbacks) {
    InputQueue queue(4, 10);
    {
        InputQueue::Admission admitted(queue);
        ASSERT_TRUE(admitted);
        queue.close();
        InputQueue::Admission denied(queue);
        EXPECT_FALSE(denied);
        Imu imu;
        imu.time = 123;
        queue.push(imu);
    }
    const auto event = queue.take();
    ASSERT_TRUE(event);
    EXPECT_EQ(std::get<Imu>(*event).time, 123);
    EXPECT_FALSE(queue.take());
    queue.finish();
}

TEST(InputQueue, CancelDiscardsQueuedAndInFlightResults) {
    InputQueue queue(4, 10);
    {
        InputQueue::Admission admission(queue);
        queue.push(Imu{});
        queue.close(InputQueue::CloseMode::Cancel);
        queue.push(Imu{});
    }
    EXPECT_TRUE(queue.failed());
    EXPECT_FALSE(queue.take());
    queue.finish();
}

TEST(InputQueue, CapacityFailureDoesNotSilentlyDropAcceptedEvent) {
    InputQueue queue(1, 2);
    {
        InputQueue::Admission admission(queue);
        queue.push(Imu{});
        EXPECT_THROW(queue.push(Imu{}), std::exception);
    }
    queue.close();
    EXPECT_TRUE(queue.take());
    EXPECT_FALSE(queue.take());
    queue.finish();
}

TEST(BatchExecutor, JoinsAllPartitionsAndCanBeReusedAfterException) {
    BatchExecutor executor(4);
    std::array<int, 4> visits{};
    EXPECT_THROW(executor.run(4,
                              [&](std::size_t index) {
                                  ++visits[index];
                                  if (index == 1)
                                      throw std::runtime_error("partition failed");
                              }),
                 std::runtime_error);
    EXPECT_EQ(visits, (std::array<int, 4>{1, 1, 1, 1}));
    executor.run(4, [&](std::size_t index) { ++visits[index]; });
    EXPECT_EQ(visits, (std::array<int, 4>{2, 2, 2, 2}));
}
}  // namespace
}  // namespace zlio::ros
