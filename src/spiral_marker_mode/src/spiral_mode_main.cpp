#include "spiral_mode.hpp"

#include <px4_ros2/components/node_with_mode.hpp>

using SpiralNode = px4_ros2::NodeWithMode<SpiralOverMarkerMode>;

static const std::string kNodeName = "spiral_marker_node";
static const bool kEnableDebugOutput = true;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpiralNode>(kNodeName, kEnableDebugOutput));
    rclcpp::shutdown();
    return 0;
}
