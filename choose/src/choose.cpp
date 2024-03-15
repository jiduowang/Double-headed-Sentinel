#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include "auto_aim_interfaces/msg/armors.hpp"


rclcpp::Node::SharedPtr g_node = nullptr;


class CompareArmors : public rclcpp::Node{
    public:
    CompareArmors():Node("CompareArmors"){
        // define quality of service: all messages that you want to receive must have the same
        
        armors_sub_first = this->create_subscription<auto_aim_interfaces::msg::Armors>(
            "/detector/armors_first", rclcpp::SensorDataQoS(), 
            std::bind(&CompareArmors::armors_first_callback, this, std::placeholders::_1));
        armors_sub_second = this->create_subscription<auto_aim_interfaces::msg::Armors>(
            "/detector/armors_second", rclcpp::SensorDataQoS(), 
            std::bind(&CompareArmors::armors_second_callback, this, std::placeholders::_1));

        armors_pub_ = this->create_publisher<auto_aim_interfaces::msg::Armors>(
                    "/detector/armors", rclcpp::SensorDataQoS());


    }
    private:
        void armors_first_callback(const auto_aim_interfaces::msg::Armors::SharedPtr armors_first)
        {
            if(armors_first->armors.size()>0)
            {
                
                if (time_2 == 0)
                {
                    time_1 = 80;
                    armors_msg_.armors.clear();
                    armors_msg_.header=armors_first->header;
                    armors_msg_.armors=armors_first->armors;
                    
                    armors_pub_->publish(armors_msg_);
                    
                }
                
            }
            else
            {
                if (time_1>0){
                    time_1 -=1;
                }
                
            }
        }

        void armors_second_callback(const auto_aim_interfaces::msg::Armors::SharedPtr armors_second)
        {
            if(armors_second->armors.size()>0)
            {
                if (time_1 == 0)
                {
                    time_2 = 80;
                    armors_msg_.armors.clear();
                    armors_msg_.header=armors_second->header;
                    armors_msg_.armors=armors_second->armors;
                    //RCLCPP_INFO(rclcpp::get_logger("choose_node"), "Publishing armors_second");
                    armors_pub_->publish(armors_msg_);
                    
                }
            }
            else
            {
                if(time_2>0)
                {
                    time_2 -= 1 ;
                }
                
            }
        }

        rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr armors_sub_first;
        rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr armors_sub_second;
        rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;
        
        auto_aim_interfaces::msg::Armors armors_msg_;
        
        int flag = 0;
        int time_1 = 0;
        int time_2 = 0;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CompareArmors>());
    rclcpp::shutdown();
    return 0;
}