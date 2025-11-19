#include "imu_roll_corrector.hpp"
#include <rclcpp/qos.hpp>

ImuRollCorrector::ImuRollCorrector() : Node("imu_roll_corrector")
{
    auto qos = rclcpp::QoS(10).best_effort();

    subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "mavros/imu/data", qos,
        std::bind(&ImuRollCorrector::imu_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "mavros/imu/data_corrected", 10);

    RCLCPP_INFO(this->get_logger(), "IMU Roll Corrector started");
}

void ImuRollCorrector::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    auto corrected_msg = *msg;

    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    roll = -roll;

    tf2::Quaternion q_corrected;
    q_corrected.setRPY(roll, pitch, yaw);

    corrected_msg.orientation.x = q_corrected.x();
    corrected_msg.orientation.y = q_corrected.y();
    corrected_msg.orientation.z = q_corrected.z();
    corrected_msg.orientation.w = q_corrected.w();

    publisher_->publish(corrected_msg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuRollCorrector>());
    rclcpp::shutdown();
    return 0;
}