#ifndef IMU_ROLL_CORRECTOR_HPP
#define IMU_ROLL_CORRECTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class ImuRollCorrector : public rclcpp::Node
{
public:
    ImuRollCorrector();

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};

#endif // IMU_ROLL_CORRECTOR_HPP