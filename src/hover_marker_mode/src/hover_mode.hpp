#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/goto.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <Eigen/Geometry>

/**
 * @brief PX4 flight mode for hovering above ArUco markers
 */
class HoverOverMarkerMode : public px4_ros2::ModeBase
{
public:
    /**
     * @brief Constructor
     * @param node ROS2 node reference
     */
    explicit HoverOverMarkerMode(rclcpp::Node & node);

    /**
     * @brief Mode activation callback
     */
    void onActivate() override;
    
    /**
     * @brief Mode deactivation callback
     */
    void onDeactivate() override;
    
    /**
     * @brief Setpoint update callback
     * @param dt_s Time step (unused)
     */
    void updateSetpoint(float dt_s) override;

private:
    std::shared_ptr<px4_ros2::GotoSetpointType> _goto_setpoint;
    std::shared_ptr<px4_ros2::OdometryLocalPosition> _odom;
    std::shared_ptr<px4_ros2::OdometryAttitude> _attitude;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _marker_sub;

    static constexpr float HOVER_HEIGHT = -10.0f;
    static constexpr float MARKER_TIMEOUT_S = 5.0f;
    static constexpr float MAX_VELOCITY = 2.0f;
    static constexpr float MAX_ACCELERATION = 1.0f;
    static constexpr float SMOOTHING_FACTOR = 0.15f;
    static constexpr float POSITION_TOLERANCE = 0.5f;
    static constexpr float RELATIVE_HEIGHT = 3.0f;
    static constexpr float MIN_FLIGHT_HEIGHT = -50.0f;
    static constexpr float MAX_FLIGHT_HEIGHT = -2.0f;
    static constexpr float MAX_HEIGHT_RATE = 2.0f;
    
    Eigen::Vector3f _target_pos{0.f, 0.f, HOVER_HEIGHT};
    Eigen::Vector3f _smoothed_target{0.f, 0.f, HOVER_HEIGHT};
    Eigen::Vector3f _velocity{0.f, 0.f, 0.f};
    bool _got_marker = false;
    bool _first_marker = true;
    rclcpp::Time _last_marker_time;
    rclcpp::Time _last_update_time;

    /**
     * @brief ArUco marker callback
     * @param msg Marker pose message
     */
    void onMarkerReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    
    /**
     * @brief Transform marker coordinates to NED frame
     * @param marker_x X coordinate in camera frame
     * @param marker_y Y coordinate in camera frame
     * @param marker_z Z coordinate in camera frame
     * @param should_log Enable debug logging
     * @return Position in NED frame
     */
    Eigen::Vector3f transformMarkerToNED(float marker_x, float marker_y, float marker_z, bool should_log = false);

    /**
     * @brief Declare ROS2 parameters
     */
    void declareParameters();

    float _max_velocity;
    float _max_acceleration;
    float _smoothing_factor;
    float _position_tolerance;
    float _relative_height;
    
    float _marker_height = 0.0f;
    bool _height_initialized = false;
};

