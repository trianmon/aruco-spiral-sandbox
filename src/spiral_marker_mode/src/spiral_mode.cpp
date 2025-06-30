#include "spiral_mode.hpp"
#include <algorithm>

/**
 * @brief Constructor implementation
 */
SpiralOverMarkerMode::SpiralOverMarkerMode(rclcpp::Node & node)
: ModeBase(node, Settings{"Spiral Marker Mode"})
{
    _goto_setpoint = std::make_shared<px4_ros2::GotoSetpointType>(*this);
    _odom = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
    _attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);

    _marker_sub = this->node().create_subscription<geometry_msgs::msg::PoseStamped>(
        "/aruco_pose", 10,
        std::bind(&SpiralOverMarkerMode::onMarkerReceived, this, std::placeholders::_1)
    );
    
    declareParameters();
}

/**
 * @brief Declare and initialize ROS2 parameters
 */
void SpiralOverMarkerMode::declareParameters()
{
    this->node().declare_parameter("spiral_marker.max_velocity", static_cast<double>(MAX_VELOCITY));
    this->node().declare_parameter("spiral_marker.max_acceleration", static_cast<double>(MAX_ACCELERATION));
    this->node().declare_parameter("spiral_marker.position_tolerance", static_cast<double>(POSITION_TOLERANCE));
    this->node().declare_parameter("spiral_marker.relative_height", static_cast<double>(RELATIVE_HEIGHT));
    this->node().declare_parameter("spiral_marker.angular_velocity", static_cast<double>(SPIRAL_ANGULAR_VELOCITY));
    this->node().declare_parameter("spiral_marker.radius_rate", static_cast<double>(SPIRAL_RADIUS_RATE));
    
    _max_velocity = static_cast<float>(this->node().get_parameter("spiral_marker.max_velocity").as_double());
    _max_acceleration = static_cast<float>(this->node().get_parameter("spiral_marker.max_acceleration").as_double());
    _position_tolerance = static_cast<float>(this->node().get_parameter("spiral_marker.position_tolerance").as_double());
    _relative_height = static_cast<float>(this->node().get_parameter("spiral_marker.relative_height").as_double());
    _spiral_angular_velocity = static_cast<float>(this->node().get_parameter("spiral_marker.angular_velocity").as_double());
    _spiral_radius_rate = static_cast<float>(this->node().get_parameter("spiral_marker.radius_rate").as_double());
    
    RCLCPP_INFO(this->node().get_logger(), 
                "Параметры спирали: макс_скорость=%.2f, угл_скорость=%.2f рад/с, скорость_радиуса=%.2f м/с",
                _max_velocity, _spiral_angular_velocity, _spiral_radius_rate);
}

/**
 * @brief Mode activation implementation
 */
void SpiralOverMarkerMode::onActivate()
{
    setSetpointUpdateRate(20.0f);
    RCLCPP_INFO(this->node().get_logger(), "Режим спирального полета над меткой активирован");
    
    _spiral_angle = 0.0f;
    _spiral_radius = SPIRAL_RADIUS_MIN;
    _spiral_expanding = true;
    _height_initialized = false;
    _marker_height = 0.0f;
    
    if (_odom->positionXYValid())
    {
        _marker_center = Eigen::Vector3f{
            _odom->positionNed().x(),
            _odom->positionNed().y(),
            HOVER_HEIGHT
        };
        _target_pos = _marker_center;
        _smoothed_target = _target_pos;
        _velocity = Eigen::Vector3f{0.f, 0.f, 0.f};
        
        RCLCPP_INFO(this->node().get_logger(), 
                    "Начальный центр спирали: [%.2f, %.2f, %.2f], высота над меткой: %.2fм", 
                    _marker_center.x(), _marker_center.y(), _marker_center.z(), _relative_height);
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
void SpiralOverMarkerMode::onDeactivate()
{
    RCLCPP_INFO(this->node().get_logger(), "Режим спирального полета над меткой деактивирован");
}

/**
 * @brief Update flight setpoint
 */
void SpiralOverMarkerMode::updateSetpoint(float /*dt_s*/)
{
    static int counter = 0;
    counter++;
    
    auto now = this->node().now();
    float dt = static_cast<float>((now - _last_update_time).seconds());
    _last_update_time = now;
    
    dt = std::clamp(dt, 0.01f, 0.1f);
    
    if (_got_marker)
    {
        if ((now - _last_marker_time).seconds() > MARKER_TIMEOUT_S)
        {
            _got_marker = false;
            _first_marker = true;
            RCLCPP_WARN(this->node().get_logger(), 
                        "Потеря сигнала метки - продолжение спирали в последней известной позиции (таймаут %.1fs)", MARKER_TIMEOUT_S);
        }
    }
    
    Eigen::Vector3f desired_pos = calculateSpiralPosition(dt);
    
    Eigen::Vector3f position_error = desired_pos - _smoothed_target;
    float error_magnitude = position_error.norm();
    
    float speed_factor = std::min(1.0f, error_magnitude / _position_tolerance);
    Eigen::Vector3f desired_velocity = position_error.normalized() * (_max_velocity * speed_factor);
    
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
    
    if (counter % 40 == 0)
    {
        if (_got_marker)
        {
            RCLCPP_INFO(this->node().get_logger(), 
                        "Спираль слежение - Центр: [%.2f, %.2f, %.2f], Текущая: [%.2f, %.2f, %.2f], Радиус: %.2fм, Угол: %.1f°", 
                        _marker_center.x(), _marker_center.y(), _marker_center.z(),
                        _smoothed_target.x(), _smoothed_target.y(), _smoothed_target.z(),
                        _spiral_radius, _spiral_angle * 180.0f / M_PI);
        }
        else
        {
            RCLCPP_WARN(this->node().get_logger(), 
                        "Спираль без метки - Позиция: [%.2f, %.2f, %.2f], Радиус: %.2fм", 
                        _smoothed_target.x(), _smoothed_target.y(), _smoothed_target.z(), _spiral_radius);
        }
    }
}

/**
 * @brief Calculate next spiral position
 */
Eigen::Vector3f SpiralOverMarkerMode::calculateSpiralPosition(float dt)
{
    _spiral_angle += _spiral_angular_velocity * dt;
    
    while (_spiral_angle > 2 * M_PI)
    {
        _spiral_angle -= 2 * M_PI;
    }
    
    if (_spiral_expanding)
    {
        _spiral_radius += _spiral_radius_rate * dt;
        if (_spiral_radius >= SPIRAL_RADIUS_MAX)
        {
            _spiral_radius = SPIRAL_RADIUS_MAX;
            _spiral_expanding = false;
        }
    }
    else
    {
        _spiral_radius -= _spiral_radius_rate * dt;
        if (_spiral_radius <= SPIRAL_RADIUS_MIN)
        {
            _spiral_radius = SPIRAL_RADIUS_MIN;
            _spiral_expanding = true;
        }
    }
    
    float x_offset = _spiral_radius * cos(_spiral_angle);
    float y_offset = _spiral_radius * sin(_spiral_angle);
    
    float target_height = _marker_center.z() - _relative_height;
    
    target_height = std::clamp(target_height, MIN_FLIGHT_HEIGHT, MAX_FLIGHT_HEIGHT);
    
    return Eigen::Vector3f(
        _marker_center.x() + x_offset,
        _marker_center.y() + y_offset,
        target_height
    );
}

/**
 * @brief Process received ArUco marker pose
 */
void SpiralOverMarkerMode::onMarkerReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    static auto last_log_time = this->node().now();
    auto now = this->node().now();
    bool should_log = (now - last_log_time).seconds() > 2.0;
    
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
        Eigen::Vector2f movement(marker_ned.x() - _marker_center.x(), 
                               marker_ned.y() - _marker_center.y());
        float movement_distance = movement.norm();
        
        if (movement_distance < MIN_MOVEMENT)
        {
            return;
        }
        
        if (movement_distance > MAX_JUMP)
        {
            RCLCPP_WARN(this->node().get_logger(), 
                        "Большой скачок метки обнаружен: %.2fм - игнорируется", movement_distance);
            return;
        }
    }
    
    _marker_center = Eigen::Vector3f(marker_ned.x(), marker_ned.y(), marker_ned.z());
    _got_marker = true;
    _last_marker_time = this->node().now();
    
    if (should_log)
    {
        RCLCPP_INFO(this->node().get_logger(), 
                    "Обновлен центр спирали: [%.2f, %.2f, %.2f], расстояние от камеры: %.2fм", 
                    _marker_center.x(), _marker_center.y(), _marker_center.z(),
                    msg->pose.position.z);
    }
}

/**
 * @brief Transform marker coordinates from camera to NED frame
 */
Eigen::Vector3f SpiralOverMarkerMode::transformMarkerToNED(float marker_x, float marker_y, float marker_z, bool should_log)
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
    
    Eigen::Vector3f result = marker_ned_absolute;
    
    if (should_log)
    {
        RCLCPP_INFO(this->node().get_logger(), 
                     "Преобразование координат спирали: камера[%.3f,%.3f,%.3f] -> body[%.3f,%.3f,%.3f] -> дрон_ned[%.3f,%.3f,%.3f] -> метка_abs[%.3f,%.3f,%.3f]",
                     marker_x, marker_y, marker_z,
                     marker_body.x(), marker_body.y(), marker_body.z(),
                     drone_pos_ned.x(), drone_pos_ned.y(), drone_pos_ned.z(),
                     result.x(), result.y(), result.z());
    }
    
    return result;
}
