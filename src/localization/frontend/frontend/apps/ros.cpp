/**
 * @file ros.cpp
 * @brief 转换 ROS 传感器输入与里程计输出消息。
 */
#include "ros.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <stdexcept>
#include <utility>

#include "zlio/state.hpp"

namespace zlio::ros {
/// 将整数纳秒规范为 ROS 秒和非负纳秒，秒字段越界时拒绝。
builtin_interfaces::msg::Time makeTimeMessage(Time time) {
    constexpr Time kNanosecondsPerSecond = 1000000000LL;
    Time seconds = time / kNanosecondsPerSecond;
    Time remainder = time % kNanosecondsPerSecond;
    if (remainder < 0) {
        --seconds;
        remainder += kNanosecondsPerSecond;
    }
    if (seconds < std::numeric_limits<std::int32_t>::min() ||
        seconds > std::numeric_limits<std::int32_t>::max())
        throw std::out_of_range("odometry time exceeds the int32 range of ROS sec");
    builtin_interfaces::msg::Time result;
    result.sec = static_cast<std::int32_t>(seconds);
    result.nanosec = static_cast<std::uint32_t>(remainder);
    return result;
}

/// 将 ROS 时间字段转换为整数纳秒。
Time messageTime(const builtin_interfaces::msg::Time& time) {
    return static_cast<Time>(time.sec) * 1000000000LL + time.nanosec;
}

/// 保存点云格式，并拒绝非有限或非正的时间倍率。
PointCloudDecoder::PointCloudDecoder(PointCloudFormat format) : format_(std::move(format)) {
    if (!std::isfinite(format_.time_scale) || format_.time_scale <= 0)
        throw std::invalid_argument("point_time_scale must be finite and positive");
}

/// 构造世界系位姿和 IMU 系速度的里程计消息。
nav_msgs::msg::Odometry makeOdometryMessage(const Estimate& estimate, const Vec3& angular_velocity,
                                            const OutputFrames& frames, double gyro_variance) {
    nav_msgs::msg::Odometry output;
    const auto& x = estimate.state;
    const auto& p = x.pose.position;
    const auto& q = x.pose.rotation;
    output.header.stamp = makeTimeMessage(x.time);
    output.header.frame_id = frames.world;
    output.child_frame_id = frames.body;
    output.pose.pose.position.x = p.x();
    output.pose.pose.position.y = p.y();
    output.pose.pose.position.z = p.z();
    output.pose.pose.orientation.x = q.x();
    output.pose.pose.orientation.y = q.y();
    output.pose.pose.orientation.z = q.z();
    output.pose.pose.orientation.w = q.w();
    const Mat6 covariance = poseCovariance(x, estimate.covariance);
    for (int row = 0; row < 6; ++row)
        for (int column = 0; column < 6; ++column)
            output.pose.covariance[6 * row + column] = covariance(row, column);
    const Vec3 velocity = q.conjugate() * x.velocity, omega = angular_velocity - x.gyro_bias;
    output.twist.twist.linear.x = velocity.x();
    output.twist.twist.linear.y = velocity.y();
    output.twist.twist.linear.z = velocity.z();
    output.twist.twist.angular.x = omega.x();
    output.twist.twist.angular.y = omega.y();
    output.twist.twist.angular.z = omega.z();
    Eigen::Matrix<double, 6, 15> twist_jacobian = Eigen::Matrix<double, 6, 15>::Zero();
    const Mat3 inverse_rotation = q.conjugate().toRotationMatrix();
    twist_jacobian.block<3, 3>(0, 6) = inverse_rotation;
    twist_jacobian.block<3, 3>(3, 9) = -inverse_rotation;
    Mat6 twist_covariance = twist_jacobian * estimate.covariance * twist_jacobian.transpose();

    twist_covariance.bottomRightCorner<3, 3>() += gyro_variance * Mat3::Identity();
    for (int row = 0; row < 6; ++row)
        for (int column = 0; column < 6; ++column)
            output.twist.covariance[6 * row + column] = twist_covariance(row, column);
    return output;
}

/// 以原输入次序编码已去畸变点，保留后验时刻并指定机体坐标系。
sensor_msgs::msg::PointCloud2 makePointCloudMessage(const std::vector<Vec3>& points,
                                                    const std_msgs::msg::Header& header,
                                                    const std::string& body_frame) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    cloud.header.frame_id = body_frame;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (const Vec3& point : points) {
        *x = point.x();
        *y = point.y();
        *z = point.z();
        ++x;
        ++y;
        ++z;
    }
    return cloud;
}
namespace {
using Field = sensor_msgs::msg::PointField;
using Cloud = sensor_msgs::msg::PointCloud2;

/// 保存单点记录的坐标和时间字段布局。
struct CloudFields {
    const Field *x, *y, *z, *time;
};

/// 校验点云数据区、字节序和行步长。
void validateCloud(const Cloud& cloud) {
    if (cloud.is_bigendian)
        throw std::invalid_argument("big-endian PointCloud2 is not supported");
    if (cloud.data.size() < static_cast<std::size_t>(cloud.row_step) * cloud.height ||
        cloud.point_step == 0 ||
        cloud.row_step < static_cast<std::size_t>(cloud.point_step) * cloud.width)
        throw std::invalid_argument("invalid PointCloud2 data length or stride");
}

/// 按名称查找点字段。
const Field* findField(const Cloud& cloud, const std::string& name) {
    for (const auto& field : cloud.fields)
        if (field.name == name)
            return &field;
    return nullptr;
}

/// 从可能非对齐的地址读取一个字段值。
template <typename T>
double readValue(const std::uint8_t* data, const Field& field) {
    T value;
    std::memcpy(&value, data + field.offset, sizeof(value));
    return static_cast<double>(value);
}

/// 校验字段类型及其在单点记录中的范围。
void validateField(const Field& field, std::size_t point_step) {
    std::size_t width;
    switch (field.datatype) {
        case Field::FLOAT64:
            width = 8;
            break;
        case Field::FLOAT32:
        case Field::UINT32:
        case Field::INT32:
            width = 4;
            break;
        case Field::UINT16:
            width = 2;
            break;
        case Field::UINT8:
            width = 1;
            break;
        default:
            throw std::invalid_argument("unsupported PointCloud2 field datatype");
    }
    if (field.offset > point_step || width > point_step - field.offset)
        throw std::invalid_argument("point field exceeds point_step");
}

/// 按字段声明的数值类型读取标量。
double readField(const std::uint8_t* data, const Field* field) {
    switch (field->datatype) {
        case Field::FLOAT32:
            return readValue<float>(data, *field);
        case Field::FLOAT64:
            return readValue<double>(data, *field);
        case Field::UINT32:
            return readValue<std::uint32_t>(data, *field);
        case Field::INT32:
            return readValue<std::int32_t>(data, *field);
        case Field::UINT16:
            return readValue<std::uint16_t>(data, *field);
        case Field::UINT8:
            return readValue<std::uint8_t>(data, *field);
        default:
            throw std::invalid_argument("unsupported PointCloud2 field datatype");
    }
}

/// 将相对秒数转换为整数纳秒采样时刻。
Time acquisitionTime(Time base, double offset_seconds) {
    if (!std::isfinite(offset_seconds) || std::abs(offset_seconds) > 9e9)
        throw std::invalid_argument("invalid point timestamp");
    const Time offset = nanoseconds(offset_seconds);
    if ((offset > 0 && base > std::numeric_limits<Time>::max() - offset) ||
        (offset < 0 && base < std::numeric_limits<Time>::min() - offset))
        throw std::invalid_argument("point timestamp exceeds the integer nanosecond range");
    return base + offset;
}

/// 解码浮点坐标及指定类型的逐点时间。
template <class TimeValue>
Scan decodeFloatCloud(const Cloud& cloud, const CloudFields& fields, Time base, double time_scale) {
    Scan output;
    output.reserve(static_cast<std::size_t>(cloud.width) * cloud.height);
    for (std::size_t row = 0; row < cloud.height; ++row)
        for (std::size_t column = 0; column < cloud.width; ++column) {
            const auto* data = cloud.data.data() + row * cloud.row_step + column * cloud.point_step;
            const Time acquired =
                acquisitionTime(base, readValue<TimeValue>(data, *fields.time) * time_scale);
            output.push_back(
                {Vec3(readValue<float>(data, *fields.x), readValue<float>(data, *fields.y),
                      readValue<float>(data, *fields.z)),
                 acquired, 0});
        }
    return output;
}
}  // namespace

/// 校验点云布局并选择对应的解码路径。
Scan PointCloudDecoder::decode(const Cloud& cloud) const {
    validateCloud(cloud);
    const auto count = static_cast<std::size_t>(cloud.width) * cloud.height;
    if (count > format_.maximum_points)
        throw std::length_error("PointCloud2 packet point limit exceeded");
    const CloudFields fields{findField(cloud, "x"), findField(cloud, "y"), findField(cloud, "z"),
                             findField(cloud, format_.time_field)};
    if (!fields.x || !fields.y || !fields.z || !fields.time)
        throw std::invalid_argument(
            "PointCloud2 is missing coordinates or the configured per-point time field");
    const Time base = format_.absolute_time ? 0 : messageTime(cloud.header.stamp);
    if (count == 0)
        return {};
    for (const auto* field : {fields.x, fields.y, fields.z, fields.time})
        validateField(*field, cloud.point_step);
    // 按包选择常见布局，允许非对齐字段并逐点校验时间。
    if (fields.x->datatype == Field::FLOAT32 && fields.y->datatype == Field::FLOAT32 &&
        fields.z->datatype == Field::FLOAT32) {
        if (fields.time->datatype == Field::UINT32)
            return decodeFloatCloud<std::uint32_t>(cloud, fields, base, format_.time_scale);
        if (fields.time->datatype == Field::FLOAT64)
            return decodeFloatCloud<double>(cloud, fields, base, format_.time_scale);
    }
    Scan output;
    output.reserve(count);
    for (std::size_t row = 0; row < cloud.height; ++row) {
        for (std::size_t column = 0; column < cloud.width; ++column) {
            const auto* data = cloud.data.data() + row * cloud.row_step + column * cloud.point_step;
            const auto value = [&](const Field* field) { return readField(data, field); };
            const Time acquired = acquisitionTime(base, value(fields.time) * format_.time_scale);
            output.push_back(
                {Vec3(value(fields.x), value(fields.y), value(fields.z)), acquired, 0});
        }
    }
    return output;
}
}  // namespace zlio::ros
