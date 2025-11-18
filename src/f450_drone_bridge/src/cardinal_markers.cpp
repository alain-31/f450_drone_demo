#include "cardinal_markers.hpp"

namespace f450_drone_bridge
{

CardinalMarkers::CardinalMarkers()
: Node("cardinal_markers"),
  frame_id_("map"),
  distance_(3.0),      // Distance from origin in meters
  text_height_(0.0),   // Height above ground (Z)
  text_scale_(0.5)     // Text size
{
  // Declare parameters
  this->declare_parameter<std::string>("frame_id", "map");
  this->declare_parameter<double>("distance", 3.0);
  this->declare_parameter<double>("text_height", 0.0);
  this->declare_parameter<double>("text_scale", 0.5);
  
  // Get parameters
  this->get_parameter("frame_id", frame_id_);
  this->get_parameter("distance", distance_);
  this->get_parameter("text_height", text_height_);
  this->get_parameter("text_scale", text_scale_);

  // Create publisher
  marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/cardinal_markers", 10
  );

  // Create timer to publish markers periodically (1 Hz)
  timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&CardinalMarkers::publishMarkers, this)
  );

  RCLCPP_INFO(this->get_logger(), "Cardinal Markers node started");
  RCLCPP_INFO(this->get_logger(), "Frame: %s, Distance: %.1f m", 
              frame_id_.c_str(), distance_);
}

CardinalMarkers::~CardinalMarkers()
{
  RCLCPP_INFO(this->get_logger(), "Cardinal Markers node shutting down");
}

visualization_msgs::msg::Marker CardinalMarkers::createTextMarker(
  int id,
  const std::string& text,
  double x, double y, double z,
  float r, float g, float b)
{
  visualization_msgs::msg::Marker marker;
  
  marker.header.frame_id = frame_id_;
  marker.header.stamp = this->get_clock()->now();
  marker.ns = "cardinal_directions";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  
  // Position
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  
  // Orientation (identity - text faces camera)
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  
  // Scale (only z matters for TEXT_VIEW_FACING)
  marker.scale.x = text_scale_;
  marker.scale.y = text_scale_;
  marker.scale.z = text_scale_;
  
  // Color
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;
  
  // Text content
  marker.text = text;
  
  // Lifetime (0 = forever)
  marker.lifetime = rclcpp::Duration::from_seconds(0);
  
  return marker;
}

void CardinalMarkers::publishMarkers()
{
  visualization_msgs::msg::MarkerArray marker_array;
  
  // ENU Convention:
  // East  = +X
  // North = +Y
  // West  = -X
  // South = -Y
  
  // North (Green - like on a compass)
  marker_array.markers.push_back(
    createTextMarker(0, "NORTH", 0.0, distance_, text_height_, 0.0, 1.0, 0.0)
  );
  
  // South (Red)
  marker_array.markers.push_back(
    createTextMarker(1, "SOUTH", 0.0, -distance_, text_height_, 1.0, 0.0, 0.0)
  );
  
  // East (Yellow/Orange)
  marker_array.markers.push_back(
    createTextMarker(2, "EAST", distance_, 0.0, text_height_, 1.0, 0.5, 0.0)
  );
  
  // West (Blue)
  marker_array.markers.push_back(
    createTextMarker(3, "WEST", -distance_, 0.0, text_height_, 0.0, 0.5, 1.0)
  );
  
  // Publish
  marker_publisher_->publish(marker_array);
}

}  // namespace f450_drone_bridge

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<f450_drone_bridge::CardinalMarkers>();
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}