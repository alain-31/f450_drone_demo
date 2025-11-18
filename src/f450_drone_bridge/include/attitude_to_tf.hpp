#ifndef F450_DRONE_BRIDGE__ATTITUDE_TO_TF_HPP_
#define F450_DRONE_BRIDGE__ATTITUDE_TO_TF_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace f450_drone_bridge
{

/**
 * @class AttitudeToTF
 * @brief Converts MAVROS IMU attitude data to TF transforms
 * 
 * This node subscribes to /mavros/imu/data and publishes a TF transform
 * from map frame to base_link frame, allowing visualization of drone
 * orientation in RViz2.
 */
class AttitudeToTF : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   */
  AttitudeToTF();

  /**
   * @brief Destructor
   */
  ~AttitudeToTF();

private:
  /**
   * @brief Callback function for IMU messages
   * @param msg Shared pointer to IMU message
   */
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  /**
   * @brief Publishes TF transform from map to base_link
   * @param msg IMU message containing orientation
   */
  void publishTransform(const sensor_msgs::msg::Imu::SharedPtr msg);

  // Subscriber to MAVROS IMU data
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;

  // TF broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Frame IDs
  std::string parent_frame_;
  std::string child_frame_;
};

}  // namespace f450_drone_bridge

#endif  // F450_DRONE_BRIDGE__ATTITUDE_TO_TF_HPP_