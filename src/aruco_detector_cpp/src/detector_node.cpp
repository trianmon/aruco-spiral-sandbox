#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <cmath>

/**
 * @brief ArUco marker detector node
 */
class ArucoDetectorNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor
     */
    ArucoDetectorNode() : Node("aruco_detector_node")
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image",
            10,
            std::bind(&ArucoDetectorNode::image_callback, this, std::placeholders::_1)
        );

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("aruco_pose", 10);

        marker_length_ = 0.5; 
        
        RCLCPP_INFO(this->get_logger(), 
                    "ArUco детектор инициализирован: размер_метки=%.2fм, публикация в /aruco_pose",
                    marker_length_);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    double marker_length_;

    /**
     * @brief Image callback for ArUco detection
     * @param msg Image message
     */
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // Преобразование ROS сообщения в формат OpenCV
            auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat image = cv_ptr->image;

            // Инициализация детектора ArUco
            cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            cv::aruco::detectMarkers(image, dictionary, corners, ids);

            if (!ids.empty())
            {
                for (size_t i = 0; i < ids.size(); ++i)
                {
                    auto& marker_corners = corners[i];
                    
                    // Ограничение частоты вывода логов
                    static auto last_log_time = this->get_clock()->now();
                    auto now = this->get_clock()->now();
                    bool should_log = (now - last_log_time).seconds() > 1.0;
          
                    // Вычисление среднего размера метки в пикселях
                    double side1 = cv::norm(marker_corners[1] - marker_corners[0]);
                    double side2 = cv::norm(marker_corners[2] - marker_corners[1]);
                    double side3 = cv::norm(marker_corners[3] - marker_corners[2]);
                    double side4 = cv::norm(marker_corners[0] - marker_corners[3]);
                    double marker_pixels = (side1 + side2 + side3 + side4) / 4.0;
                    
                    // Вычисление центра метки в пиксельных координатах
                    cv::Point2f center = (marker_corners[0] + marker_corners[1] + 
                                         marker_corners[2] + marker_corners[3]) / 4.0f;
                    
                    // Оценка расстояния
                    double fx = 1280.0 / (2.0 * tan(1.74 / 2.0));
                    double distance = (marker_length_ * fx) / marker_pixels;
                    
                    // Преобразование пиксельных смещений в метрические координаты
                    double cx = 1280.0 / 2.0;  // Центр изображения по X
                    double cy = 960.0 / 2.0;   // Центр изображения по Y
                    double x_offset_pixels = center.x - cx;
                    double y_offset_pixels = center.y - cy;
          
                    // Преобразование пиксельных координат в координаты системы камеры (метры)
                    double final_x = (x_offset_pixels * distance) / fx;
                    double final_y = (y_offset_pixels * distance) / fx;
                    double final_z = distance;
                    
                    if (should_log)
                    {
                        RCLCPP_INFO(this->get_logger(), 
                                    "Метка ID %d: пиксели=%.1f, расстояние=%.3fм, позиция=[%.3f, %.3f, %.3f]м",
                                    ids[i], marker_pixels, distance, final_x, final_y, final_z);
                        last_log_time = now;
                    }

                    // Публикация позы метки в системе координат камеры
                    geometry_msgs::msg::PoseStamped pose_msg;
                    pose_msg.header.stamp = this->get_clock()->now();
                    pose_msg.header.frame_id = "camera_link";
                    pose_msg.pose.position.x = final_x;
                    pose_msg.pose.position.y = final_y;
                    pose_msg.pose.position.z = final_z;

                    pose_msg.pose.orientation.x = 0.0;
                    pose_msg.pose.orientation.y = 0.0;
                    pose_msg.pose.orientation.z = 0.0;
                    pose_msg.pose.orientation.w = 1.0;

                    pose_pub_->publish(pose_msg);
                }
            }

        }
        catch (const cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Ошибка cv_bridge: %s", e.what());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoDetectorNode>());
    rclcpp::shutdown();
    return 0;
}

