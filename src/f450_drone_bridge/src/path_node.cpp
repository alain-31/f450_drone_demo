#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

class PathNode : public rclcpp::Node {
public:
  PathNode() : Node("path_node") {
    path_.header.frame_id = declare_parameter<std::string>("frame_id", "map");
    min_dt_ = declare_parameter<double>("min_dt", 0.05);
    pub_ = create_publisher<nav_msgs::msg::Path>("trajectory/path", 10);

    rclcpp::QoS qos(10);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", qos,
      std::bind(&PathNode::poseCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "path_node: listening on /mavros/local_position/pose");
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // Check timestamp to avoid publishing too frequently
    const double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    if (last_t_ > 0.0 && (t - last_t_) < min_dt_) {
      return;
    }
    last_t_ = t;

    // Create pose with correct frame_id
    geometry_msgs::msg::PoseStamped p = *msg;
    p.header.frame_id = path_.header.frame_id;

    // Update and publish path
    path_.header.stamp = msg->header.stamp;
    path_.poses.push_back(p);
    pub_->publish(path_);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_;
  nav_msgs::msg::Path path_;
  double last_t_{-1.0};
  double min_dt_{0.05};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathNode>());
  rclcpp::shutdown();
  return 0;
}