#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

class PathNode : public rclcpp::Node {
public:
  PathNode() : Node("path_node") {
    path_.header.frame_id = declare_parameter<std::string>("frame_id", "map");
    min_dt_ = declare_parameter<double>("min_dt", 0.05);
    pub_ = create_publisher<nav_msgs::msg::Path>("trajectory/path", 10);

    sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        const double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        if (last_t_ > 0.0 && (t - last_t_) < min_dt_) return;
        last_t_ = t;

        geometry_msgs::msg::PoseStamped p = *msg;
        p.header.frame_id = path_.header.frame_id;

        path_.header.stamp = msg->header.stamp;
        path_.poses.push_back(p);
        pub_->publish(path_);
      });

    RCLCPP_INFO(get_logger(), "path_node: listening on /mavros/local_position/pose");
  }

private:
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
