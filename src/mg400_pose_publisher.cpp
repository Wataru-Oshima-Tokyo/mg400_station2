#include "mg400_interface/tcp_interface/realtime_feedback_tcp_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
class PosePublisherNode : public rclcpp::Node
{
public:
    PosePublisherNode()
        : Node("pose_publisher_node"),
          pose_publisher_(this->create_publisher<geometry_msgs::msg::Pose>("pose", 10))
    {
    }

    void run()
    {
        auto pose_msg = std::make_shared<geometry_msgs::msg::Pose>();
        auto rt_tcp_if = std::make_unique<mg400_interface::RealtimeFeedbackTcpInterface>(ip);
        rt_tcp_if->init();

        rclcpp::Rate rate(10.0);  // 10 Hz

        while (rclcpp::ok())
        {
            rt_tcp_if->getCurrentEndPose(*pose_msg);
            pose_publisher_->publish(*pose_msg);
            rclcpp::spin_some(this->shared_from_this());
            rate.sleep();
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher_;
    
    std::string ip = "192.168.1.6";
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto pose_publisher_node = std::make_shared<PosePublisherNode>();
    pose_publisher_node->run();
    rclcpp::shutdown();

    return 0;
}