#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/empty.hpp"
#include "mg400_msgs/srv/clear_error.hpp"
#include "mg400_msgs/srv/enable_robot.hpp"
#include "mg400_msgs/srv/disable_robot.hpp"
#include "mg400_msgs/srv/speed_factor.hpp"
#include "mg400_msgs/action/mov_j.hpp"
#include "mg400_msgs/action/mov_l.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "techshare_ros_pkg2/action/empty.hpp"

struct Quaternion {
    double x, y, z, w;
};

class MG400ControlNode : public rclcpp::Node
{
public:
    MG400ControlNode();
    Quaternion YawToQuaternion(double);
private:
    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Client<mg400_msgs::srv::ClearError>::SharedPtr clear_error_client;
    rclcpp::Client<mg400_msgs::srv::EnableRobot>::SharedPtr enable_robot_client;
    rclcpp::Client<mg400_msgs::srv::DisableRobot>::SharedPtr disable_robot_client;
    rclcpp::Client<mg400_msgs::srv::SpeedFactor>::SharedPtr speed_factor_client;

    rclcpp_action::Client<mg400_msgs::action::MovJ>::SharedPtr mov_j_action_client;
    rclcpp_action::Client<mg400_msgs::action::MovL>::SharedPtr mov_l_action_client;
    rclcpp_action::Server<techshare_ros_pkg2::action::Empty>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
      const std::array<unsigned char, 16>& uuid,
      std::shared_ptr<const techshare_ros_pkg2::action::Empty::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<techshare_ros_pkg2::action::Empty>> goal_handle);

    void handle_accepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<techshare_ros_pkg2::action::Empty>> goal_handle);

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<techshare_ros_pkg2::action::Empty>> goal_handle);
};