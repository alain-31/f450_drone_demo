#ifndef F450_DRONE_BRIDGE__CARDINAL_MARKERS_HPP_
#define F450_DRONE_BRIDGE__CARDINAL_MARKERS_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace f450_drone_bridge
{

/**
 * @class CardinalMarkers
 * @brief Publishes text markers for North, South, East, West directions
 */
class CardinalMarkers : public rclcpp::Node
{
public:
  CardinalMarkers();
  ~CardinalMarkers();

private:
  void publishMarkers();
  
  visualization_msgs::msg::Marker createTextMarker(
    int id,
    const std::string& text,
    double x, double y, double z,
    float r, float g, float b
  );

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Parameters
  std::string frame_id_;
  double distance_;
  double text_height_;
  double text_scale_;
};

}  // namespace f450_drone_bridge

#endif  // F450_DRONE_BRIDGE__CARDINAL_MARKERS_HPP_