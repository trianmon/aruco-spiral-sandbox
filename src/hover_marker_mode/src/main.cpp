#include "hover_mode.hpp"

#include <px4_ros2/components/node_with_mode.hpp>

using HoverNode = px4_ros2::NodeWithMode<HoverOverMarkerMode>;

static const std::string kNodeName = "hover_marker_node";
static const bool kEnableDebugOutput = true;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HoverNode>(kNodeName, kEnableDebugOutput));
    rclcpp::shutdown();
    return 0;
}