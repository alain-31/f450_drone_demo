#include "attitude_to_tf.hpp"

namespace f450_drone_bridge
{

AttitudeToTF::AttitudeToTF()
: Node("attitude_to_tf"),
  parent_frame_("map"),
  child_frame_("base_link")
{
  // Declare and get parameters
  this->declare_parameter<std::string>("parent_frame", "map");
  this->declare_parameter<std::string>("child_frame", "base_link");
  
  this->get_parameter("parent_frame", parent_frame_);
  this->get_parameter("child_frame", child_frame_);

  // Initialize TF broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Create QoS profile compatible with MAVROS (sensor data)
  auto qos = rclcpp::SensorDataQoS();  // BEST_EFFORT + other sensor defaults

  // Alternative: Manual QoS configuration
  // auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
  //   .reliability(rclcpp::ReliabilityPolicy::BestEffort)
  //   .durability(rclcpp::DurabilityPolicy::Volatile);

  // Create subscription to MAVROS IMU data with compatible QoS
  imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/mavros/imu/data",
    qos,  // ← Use sensor data QoS
    std::bind(&AttitudeToTF::imuCallback, this, std::placeholders::_1)
  );

  RCLCPP_INFO(this->get_logger(), "Attitude to TF node started");
  RCLCPP_INFO(this->get_logger(), "Publishing transform: %s -> %s",
              parent_frame_.c_str(), child_frame_.c_str());
}

AttitudeToTF::~AttitudeToTF()
{
  RCLCPP_INFO(this->get_logger(), "Attitude to TF node shutting down");
}

void AttitudeToTF::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  publishTransform(msg);
}

void AttitudeToTF::publishTransform(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // Create transform message
  geometry_msgs::msg::TransformStamped transform_stamped;

  // Set header
  transform_stamped.header.stamp = this->get_clock()->now();
  transform_stamped.header.frame_id = parent_frame_;
  transform_stamped.child_frame_id = child_frame_;

  // Set translation (position at origin for now)
  transform_stamped.transform.translation.x = 0.0;
  transform_stamped.transform.translation.y = 0.0;
  transform_stamped.transform.translation.z = 0.0;

  // Set rotation (orientation from IMU)
  transform_stamped.transform.rotation.x = msg->orientation.x;
  transform_stamped.transform.rotation.y = msg->orientation.y;
  transform_stamped.transform.rotation.z = msg->orientation.z;
  transform_stamped.transform.rotation.w = msg->orientation.w;

  // Broadcast transform
  tf_broadcaster_->sendTransform(transform_stamped);
}

}  // namespace f450_drone_bridge

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<f450_drone_bridge::AttitudeToTF>();
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}