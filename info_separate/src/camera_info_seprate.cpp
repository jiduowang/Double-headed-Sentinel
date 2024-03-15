#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>


class Camera_info_seprate : public rclcpp::Node
{
public:
    Camera_info_seprate() : Node("Camera_info_seprate")
    {
        // define quality of service: all messages that you want to receive must have the same
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera_info", rclcpp::SensorDataQoS(),
            std::bind(&Camera_info_seprate::camera_info_callback, this, std::placeholders::_1));
        camera_info_pub_first = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "/camera/camera_info_first", rclcpp::SensorDataQoS());
        camera_info_pub_second = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "/camera/camera_info_second", rclcpp::SensorDataQoS());
    }

private:
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info)
    {
        if (camera_info->header.frame_id == "camera_optical_frame_first")
        {
            // RCLCPP_INFO(this->get_logger(), "/image_raw_first");
            camera_info_pub_first->publish(*camera_info);
        }
        else if (camera_info->header.frame_id == "camera_optical_frame_second")
        {
            camera_info_pub_second->publish(*camera_info);
        }
    }

    sensor_msgs::msg::CameraInfo camera_info_pub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_first;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_second;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Camera_info_seprate>());
    rclcpp::shutdown();
    return 0;
}