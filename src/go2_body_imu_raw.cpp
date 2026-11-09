#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "unitree_go/msg/low_state.hpp"

// Clock-domain bridge: estimate LiDAR header time minus local receipt time.
// Maximum observed offset in a short window approximates the lowest-latency
// LiDAR message, reducing network scheduling jitter. This does NOT estimate
// physical sensor latency or guarantee hardware synchronization.
class BodyImuAdapter final : public rclcpp::Node {
public:
  BodyImuAdapter() : Node("go2_body_imu_raw_adapter") {
    lidar_topic_ = declare_parameter<std::string>("lidar_topic", "/utlidar/cloud");
    max_slew_seconds_per_second_ = declare_parameter<double>("max_slew_seconds_per_second", 0.05);
    // IMU publisher must be RELIABLE for Point-LIO's default subscription.
    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    pub_ = create_publisher<sensor_msgs::msg::Imu>("/go2/body_imu_raw", pub_qos);
    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(10);
    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic_, sensor_qos,
      std::bind(&BodyImuAdapter::on_lidar, this, std::placeholders::_1));
    imu_sub_ = create_subscription<unitree_go::msg::LowState>(
      "/lowstate", sensor_qos,
      std::bind(&BodyImuAdapter::on_imu, this, std::placeholders::_1));
    timer_ = create_wall_timer(std::chrono::seconds(5), [this]() {
      std::lock_guard<std::mutex> lock(mutex_);
      RCLCPP_INFO(get_logger(), "5s: lidar=%llu imu=%llu published=%llu offset=%.6f s target=%.6f s",
        static_cast<unsigned long long>(lidar_count_ - last_lidar_count_),
        static_cast<unsigned long long>(imu_count_ - last_imu_count_),
        static_cast<unsigned long long>(pub_count_ - last_pub_count_),
        offset_, target_offset_);
      last_lidar_count_ = lidar_count_; last_imu_count_ = imu_count_; last_pub_count_ = pub_count_;
    });
    RCLCPP_WARN(get_logger(), "LiDAR clock-domain bridge enabled; set Point-LIO time_lag_imu_to_lidar: 0.0. Not hardware sync.");
  }
private:
  static double to_seconds(const builtin_interfaces::msg::Time &t) {
    return static_cast<double>(t.sec) + 1e-9 * t.nanosec;
  }
  void on_lidar(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    const double receipt = now().seconds();
    const double stamp = to_seconds(msg->header.stamp);
    if (!std::isfinite(stamp) || stamp <= 0) return;
    std::lock_guard<std::mutex> lock(mutex_);
    ++lidar_count_;
    const double sample = stamp - receipt;
    // Retain ~10 s at 15 Hz. The maximum offset corresponds to minimum
    // receipt latency (assuming source clock advances normally).
    samples_.push_back({receipt, sample});
    while (!samples_.empty() && receipt - samples_.front().first > 10.0) samples_.pop_front();
    double best = sample;
    for (const auto &s : samples_) best = std::max(best, s.second);
    target_offset_ = best;
    if (!ready_) {
      offset_ = target_offset_;
      ready_ = true;
      last_adjust_time_ = receipt;
      RCLCPP_INFO(get_logger(), "Initial LiDAR clock offset: %.6f seconds", offset_);
    }
  }
  void on_imu(const unitree_go::msg::LowState::SharedPtr low) {
    const double receipt = now().seconds();
    sensor_msgs::msg::Imu out;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++imu_count_;
      if (!ready_) return; // Do not publish with a different clock domain.
      const double dt = std::max(0.0, std::min(1.0, receipt - last_adjust_time_));
      const double limit = std::max(0.0, max_slew_seconds_per_second_) * dt;
      const double delta = target_offset_ - offset_;
      offset_ += std::max(-limit, std::min(limit, delta));
      last_adjust_time_ = receipt;
      double stamp = receipt + offset_;
      // Keep timestamps strictly increasing even if the estimate shifts.
      if (stamp <= last_stamp_) stamp = last_stamp_ + 0.000001;
      last_stamp_ = stamp;
      const auto ns = static_cast<int64_t>(std::llround(stamp * 1e9));
      out.header.stamp = rclcpp::Time(ns, RCL_ROS_TIME);
      ++pub_count_;
    }
    out.header.frame_id = "go2_body_imu";
    out.orientation.w = 1.0;
    out.orientation_covariance[0] = -1.0;
    const auto &imu = low->imu_state;
    out.angular_velocity.x = imu.gyroscope[0];
    out.angular_velocity.y = imu.gyroscope[1];
    out.angular_velocity.z = imu.gyroscope[2];
    out.linear_acceleration.x = imu.accelerometer[0];
    out.linear_acceleration.y = imu.accelerometer[1];
    out.linear_acceleration.z = imu.accelerometer[2];
    pub_->publish(out);
  }
  std::string lidar_topic_;
  double max_slew_seconds_per_second_{};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;
  std::deque<std::pair<double,double>> samples_;
  bool ready_{false};
  double offset_{0}, target_offset_{0}, last_adjust_time_{0}, last_stamp_{0};
  std::uint64_t lidar_count_{0}, imu_count_{0}, pub_count_{0};
  std::uint64_t last_lidar_count_{0}, last_imu_count_{0}, last_pub_count_{0};
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BodyImuAdapter>());
  rclcpp::shutdown();
  return 0;
}
