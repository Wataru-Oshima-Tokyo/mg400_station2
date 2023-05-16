#include "mg400_interface/tcp_interface/realtime_feedback_tcp_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "mg400_msgs/msg/tool_vector_actual.hpp"

class PosePublisherNode : public rclcpp::Node
{
public:
    PosePublisherNode()
        : Node("pose_publisher_node"),
          pose_publisher_(this->create_publisher<mg400_msgs::msg::ToolVectorActual>("ToolVectorActual", 100)),
          tool_vector_actual_msg(std::make_shared<mg400_msgs::msg::ToolVectorActual>())
    {
    }

    void run()
    {
        auto rt_tcp_if = std::make_unique<mg400_interface::RealtimeFeedbackTcpInterface>(ip);
        rt_tcp_if->init();

        rclcpp::Rate rate(100.0);  // 10 Hz
        double val[6];
        while (rclcpp::ok())
        {
            rt_tcp_if->getToolVectorActual(val);
            tool_vector_actual_msg->x = val[0];
            tool_vector_actual_msg->y = val[1];
            tool_vector_actual_msg->z = val[2];
            tool_vector_actual_msg->r = val[3];


            pose_publisher_->publish(*tool_vector_actual_msg);
            rclcpp::spin_some(this->shared_from_this());
            rate.sleep();
        }
    }

private:
    rclcpp::Publisher<mg400_msgs::msg::ToolVectorActual>::SharedPtr pose_publisher_;
    std::shared_ptr<mg400_msgs::msg::ToolVectorActual> tool_vector_actual_msg;
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