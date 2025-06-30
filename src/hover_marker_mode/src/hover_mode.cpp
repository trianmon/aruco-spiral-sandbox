#include "hover_mode.hpp"
#include <algorithm>

/**
 * @brief Constructor implementation
 */
HoverOverMarkerMode::HoverOverMarkerMode(rclcpp::Node & node)
: ModeBase(node, Settings{"Hover Marker Mode"})
{
    _goto_setpoint = std::make_shared<px4_ros2::GotoSetpointType>(*this);
    _odom = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
    _attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);

    _marker_sub = this->node().create_subscription<geometry_msgs::msg::PoseStamped>(
        "/aruco_pose", 10,
        std::bind(&HoverOverMarkerMode::onMarkerReceived, this, std::placeholders::_1)
    );
    
    declareParameters();
}

/**
 * @brief Declare and initialize ROS2 parameters
 */
void HoverOverMarkerMode::declareParameters()
{
    this->node().declare_parameter("hover_marker.max_velocity", static_cast<double>(MAX_VELOCITY));
    this->node().declare_parameter("hover_marker.max_acceleration", static_cast<double>(MAX_ACCELERATION));
    this->node().declare_parameter("hover_marker.smoothing_factor", static_cast<double>(SMOOTHING_FACTOR));
    this->node().declare_parameter("hover_marker.position_tolerance", static_cast<double>(POSITION_TOLERANCE));
    this->node().declare_parameter("hover_marker.relative_height", static_cast<double>(RELATIVE_HEIGHT));
    
    _max_velocity = static_cast<float>(this->node().get_parameter("hover_marker.max_velocity").as_double());
    _max_acceleration = static_cast<float>(this->node().get_parameter("hover_marker.max_acceleration").as_double());
    _smoothing_factor = static_cast<float>(this->node().get_parameter("hover_marker.smoothing_factor").as_double());
    _position_tolerance = static_cast<float>(this->node().get_parameter("hover_marker.position_tolerance").as_double());
    _relative_height = static_cast<float>(this->node().get_parameter("hover_marker.relative_height").as_double());
    
    RCLCPP_INFO(this->node().get_logger(), 
                "Параметры: макс_скорость=%.2f, макс_ускорение=%.2f, сглаживание=%.2f, точность=%.2f, высота=%.2f",
                _max_velocity, _max_acceleration, _smoothing_factor, _position_tolerance, _relative_height);
}

/**
 * @brief Mode activation implementation
 */
void HoverOverMarkerMode::onActivate()
{
    setSetpointUpdateRate(10.0f);
    RCLCPP_INFO(this->node().get_logger(), "Режим зависания над меткой активирован");
    
    _height_initialized = false;
    _marker_height = 0.0f;
    
    if (_odom->positionXYValid())
    {
        _target_pos = Eigen::Vector3f{
            _odom->positionNed().x(),
            _odom->positionNed().y(),
            HOVER_HEIGHT
        };
        _smoothed_target = _target_pos;
        _velocity = Eigen::Vector3f{0.f, 0.f, 0.f};
        RCLCPP_INFO(this->node().get_logger(), 
                    "Начальная позиция: [%.2f, %.2f, %.2f], высота над меткой: %.2fм", 
                    _target_pos.x(), _target_pos.y(), _target_pos.z(), _relative_height);
    }
    else
    {
        RCLCPP_WARN(this->node().get_logger(), "Нет корректных данных одометрии при активации!");
    }
    
    _last_update_time = this->node().now();
}

/**
 * @brief Mode deactivation implementation
 */
void HoverOverMarkerMode::onDeactivate()
{
    RCLCPP_INFO(this->node().get_logger(), "Режим зависания над меткой деактивирован");
}

/**
 * @brief Update flight setpoint
 */
void HoverOverMarkerMode::updateSetpoint(float /*dt_s*/)
{
    static int counter = 0;
    counter++;
    
    auto now = this->node().now();
    float dt = static_cast<float>((now - _last_update_time).seconds());
    _last_update_time = now;
    
    dt = std::clamp(dt, 0.01f, 0.2f);
    
    if (_got_marker)
    {
        if ((now - _last_marker_time).seconds() > MARKER_TIMEOUT_S)
        {
            _got_marker = false;
            _first_marker = true;
            RCLCPP_WARN(this->node().get_logger(), 
                        "Потеря сигнала метки - удержание позиции (таймаут %.1fs)", MARKER_TIMEOUT_S);
        }
    }
  
    Eigen::Vector3f desired_pos;
    
    if (_got_marker)
    {
        desired_pos = _target_pos;
    }
    else if (_odom->positionXYValid())
    {
        desired_pos = Eigen::Vector3f{
            _odom->positionNed().x(),
            _odom->positionNed().y(),
            HOVER_HEIGHT
        };
    }
    else
    {
        desired_pos = Eigen::Vector3f{0.0f, 0.0f, HOVER_HEIGHT};
    }
  
    Eigen::Vector3f position_error = desired_pos - _smoothed_target;
    float error_magnitude = position_error.norm();
    
    float speed_factor = std::min(1.0f, error_magnitude / _position_tolerance);
    Eigen::Vector3f desired_velocity = position_error.normalized() * (_max_velocity * speed_factor);
    
    if (std::abs(desired_velocity.z()) > MAX_HEIGHT_RATE)
    {
        desired_velocity.z() = std::copysign(MAX_HEIGHT_RATE, desired_velocity.z());
    }
    
    Eigen::Vector3f velocity_change = desired_velocity - _velocity;
    float max_velocity_change = _max_acceleration * dt;
    
    if (velocity_change.norm() > max_velocity_change)
    {
        velocity_change = velocity_change.normalized() * max_velocity_change;
    }
    
    _velocity += velocity_change;
    
    if (_velocity.norm() > _max_velocity)
    {
        _velocity = _velocity.normalized() * _max_velocity;
    }
    
    _smoothed_target += _velocity * dt;
    
    _goto_setpoint->update(_smoothed_target);

    if (counter % 10 == 0)
    {
        if (_got_marker)
        {
            RCLCPP_INFO(this->node().get_logger(), 
                        "Слежение - Цель: [%.2f, %.2f, %.2f], Текущая: [%.2f, %.2f, %.2f], Ошибка: %.2fм, Скорость: %.2fм/с (Z: %.2fм/с)", 
                        desired_pos.x(), desired_pos.y(), desired_pos.z(),
                        _smoothed_target.x(), _smoothed_target.y(), _smoothed_target.z(),
                        error_magnitude, _velocity.norm(), _velocity.z());
        }
        else
        {
            RCLCPP_WARN(this->node().get_logger(), 
                        "Метка не найдена, удержание позиции: [%.2f, %.2f, %.2f]", 
                        _smoothed_target.x(), _smoothed_target.y(), _smoothed_target.z());
        }
    }
}

/**
 * @brief Process received ArUco marker pose
 */
void HoverOverMarkerMode::onMarkerReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    static auto last_log_time = this->node().now();
    auto now = this->node().now();
    bool should_log = (now - last_log_time).seconds() > 1.0;
    
    if (should_log)
    {
        RCLCPP_INFO(this->node().get_logger(), 
                    "Данные метки: pos=[%.3f, %.3f, %.3f]м, frame=%s", 
                    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
                    msg->header.frame_id.c_str());
        last_log_time = now;
    }
    
    Eigen::Vector3f marker_ned = transformMarkerToNED(
        msg->pose.position.x, 
        msg->pose.position.y, 
        msg->pose.position.z,
        should_log
    );
  
    const float MAX_DISTANCE_FROM_DRONE = 8.0f;
    const float MIN_MOVEMENT = 0.03f;
    const float MAX_JUMP = 3.0f;

    if (_odom->positionXYValid())
    {
        Eigen::Vector3f drone_pos = _odom->positionNed();
        float distance_from_drone = std::sqrt(
            std::pow(marker_ned.x() - drone_pos.x(), 2) + 
            std::pow(marker_ned.y() - drone_pos.y(), 2)
        );
        
        if (distance_from_drone > MAX_DISTANCE_FROM_DRONE)
        {
            RCLCPP_WARN(this->node().get_logger(), 
                        "Метка слишком далеко от дрона: %.2fм в позиции [%.2f, %.2f] - игнорируется", 
                        distance_from_drone, marker_ned.x(), marker_ned.y());
            return;
        }
    }
    
    if (_got_marker)
    {
        Eigen::Vector2f movement(marker_ned.x() - _target_pos.x(), 
                               marker_ned.y() - _target_pos.y());
        float movement_distance = movement.norm();
        
        if (movement_distance < MIN_MOVEMENT)
        {
            return;
        }
        
        if (movement_distance > MAX_JUMP)
        {
            RCLCPP_WARN(this->node().get_logger(), 
                        "Обнаружен большой скачок метки: %.2fм - игнорируется", movement_distance);
            return;
        }
    }
  
    _target_pos = marker_ned;
    _got_marker = true;
    _last_marker_time = this->node().now();
    
    if (should_log)
    {
        RCLCPP_INFO(this->node().get_logger(), 
                    "Метка в NED: [%.2f, %.2f, %.2f], расстояние от камеры: %.2fм", 
                    _target_pos.x(), _target_pos.y(), _target_pos.z(),
                    msg->pose.position.z);
    }
}

/**
 * @brief Transform marker coordinates from camera to NED frame
 */
Eigen::Vector3f HoverOverMarkerMode::transformMarkerToNED(float marker_x, float marker_y, float marker_z, bool should_log)
{
    if (!_odom->positionXYValid())
    {
        RCLCPP_WARN(this->node().get_logger(), "Нет корректных данных одометрии для преобразования координат");
        return Eigen::Vector3f(0.0f, 0.0f, HOVER_HEIGHT);
    }
    
    Eigen::Vector3f drone_pos_ned = _odom->positionNed();
    Eigen::Quaternionf drone_quat = _attitude->attitude();
    
    Eigen::Vector3f marker_body(-marker_y, marker_x, marker_z);
    
    Eigen::Vector3f marker_ned_offset = drone_quat * marker_body;
    
    Eigen::Vector3f marker_ned_absolute(
        drone_pos_ned.x() + marker_ned_offset.x(),
        drone_pos_ned.y() + marker_ned_offset.y(),
        drone_pos_ned.z() + marker_ned_offset.z()
    );

    float target_height = marker_ned_absolute.z() - _relative_height;

    target_height = std::clamp(target_height, MIN_FLIGHT_HEIGHT, MAX_FLIGHT_HEIGHT);
    
    Eigen::Vector3f result(
        marker_ned_absolute.x(),
        marker_ned_absolute.y(),
        target_height
    );
    
    if (should_log)
    {
        RCLCPP_INFO(this->node().get_logger(), 
                     "Преобразование координат: камера[%.3f,%.3f,%.3f] -> body[%.3f,%.3f,%.3f] -> дрон_ned[%.3f,%.3f,%.3f] -> метка_abs[%.3f,%.3f,%.3f] -> цель[%.3f,%.3f,%.3f]",
                     marker_x, marker_y, marker_z,
                     marker_body.x(), marker_body.y(), marker_body.z(),
                     drone_pos_ned.x(), drone_pos_ned.y(), drone_pos_ned.z(),
                     marker_ned_absolute.x(), marker_ned_absolute.y(), marker_ned_absolute.z(),
                     result.x(), result.y(), result.z());
        RCLCPP_INFO(this->node().get_logger(), 
                     "Высота: метка_z=%.3fм, относ_высота=%.3fм, целевая_высота=%.3fм (ограничено с %.3fм)", 
                     marker_ned_absolute.z(), _relative_height, target_height, 
                     marker_ned_absolute.z() - _relative_height);
    }
    
    return result;
}
