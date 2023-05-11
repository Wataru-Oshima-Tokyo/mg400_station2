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

class MG400ControlNode : public rclcpp::Node
{
public:
    MG400ControlNode()
    : Node("mg400_control_node"),
      action_server_(rclcpp_action::create_server<techshare_ros_pkg2::action::Empty>(
          this->get_node_base_interface(),
          this->get_node_clock_interface(),
          this->get_node_logging_interface(),
          this->get_node_waitables_interface(),
          "mg400_server",
          std::bind(&MG400ControlNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
          std::bind(&MG400ControlNode::handle_cancel, this, std::placeholders::_1),
          std::bind(&MG400ControlNode::handle_accepted, this, std::placeholders::_1)))
    {


        clear_error_client = node->create_client<mg400_msgs::srv::ClearError>("/mg400/clear_error");
        enable_robot_client = node->create_client<mg400_msgs::srv::EnableRobot>("/mg400/enable_robot");
        disable_robot_client = node->create_client<mg400_msgs::srv::DisableRobot>("/mg400/disable_robot");
        speed_factor_client = node->create_client<mg400_msgs::srv::SpeedFactor>("/mg400/speed_factor");

        mov_j_action_client = rclcpp_action::create_client<mg400_msgs::action::MovJ>(node, "/mg400/mov_j");
        mov_l_action_client = rclcpp_action::create_client<mg400_msgs::action::MovL>(node, "/mg400/mov_l");

    }

private:
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("mg400_move_server");
    rclcpp::Client<mg400_msgs::srv::ClearError>::SharedPtr clear_error_client;
    rclcpp::Client<mg400_msgs::srv::EnableRobot>::SharedPtr enable_robot_client;
    rclcpp::Client<mg400_msgs::srv::DisableRobot>::SharedPtr disable_robot_client;
    rclcpp::Client<mg400_msgs::srv::SpeedFactor>::SharedPtr speed_factor_client;

    rclcpp_action::Client<mg400_msgs::action::MovJ>::SharedPtr mov_j_action_client;
    rclcpp_action::Client<mg400_msgs::action::MovL>::SharedPtr mov_l_action_client;
    rclcpp_action::Server<techshare_ros_pkg2::action::Empty>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
      const std::array<unsigned char, 16>& uuid,
      std::shared_ptr<const techshare_ros_pkg2::action::Empty::Goal> goal)
      {     
            RCLCPP_INFO(this->get_logger(), "Received goal request %d", goal->fake);
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

      }

    rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<techshare_ros_pkg2::action::Empty>> goal_handle)
      {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
      }

    void handle_accepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<techshare_ros_pkg2::action::Empty>> goal_handle)
      {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        RCLCPP_INFO(this->get_logger(), "Accepted a goal:)");
        std::thread{std::bind(&MG400ControlNode::execute, this, _1), goal_handle}.detach();
      };
    


    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<techshare_ros_pkg2::action::Empty>> goal_handle)
    {
        auto result = std::make_shared<techshare_ros_pkg2::action::Empty::Result>();
        // // Clear error
        auto clear_error_request = std::make_shared<mg400_msgs::srv::ClearError::Request>();
        auto clear_error_response_future = clear_error_client->async_send_request(clear_error_request);

        // Wait for the result
        if (rclcpp::spin_until_future_complete(node, clear_error_response_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call clear_error service");
            result->done = false;
            goal_handle->abort(result);
            RCLCPP_INFO(this->get_logger(), "Action failed");
            return;
        }

        auto clear_error_response = clear_error_response_future.get();
        RCLCPP_INFO(this->get_logger(), "clear_error_response: %d", clear_error_response);
        // Enable robot
        auto enable_robot_request = std::make_shared<mg400_msgs::srv::EnableRobot::Request>();
        auto enable_robot_response_future = enable_robot_client->async_send_request(enable_robot_request);

        if (rclcpp::spin_until_future_complete(node, enable_robot_response_future) !=
            rclcpp::executor::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call enable_robot service");
            result->done = false;
            goal_handle->abort(result);
            RCLCPP_INFO(this->get_logger(), "Action failed");
            return;
        }

        auto enable_robot_response = enable_robot_response_future.get();
        RCLCPP_INFO(this->get_logger(), "enable_robot_response: %d", enable_robot_response);

        // Send MovJ action
        auto mov_j_goal = mg400_msgs::action::MovJ::Goal();
        mov_j_goal.pose.header.frame_id = "mg400_origin_link";
        mov_j_goal.pose.pose.position.x = 0.34;
        mov_j_goal.pose.pose.orientation.w = 1.0;
        
        auto mov_j_goal_handle_future = mov_j_action_client->async_send_goal(mov_j_goal);

        if (rclcpp::spin_until_future_complete(node, mov_j_goal_handle_future) !=
            rclcpp::executor::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send goal to MovJ action");
            result->done = false;
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Action failed");
            return;
        }

        auto mov_j_goal_handle = mov_j_goal_handle_future.get();

        if (!mov_j_goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by MovJ action");
            result->done = false;
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Action failed");
            return;
        }

        auto mov_j_result_future = mov_j_action_client->async_get_result(mov_j_goal_handle);
        if (rclcpp::spin_until_future_complete(node, mov_j_result_future) !=
            rclcpp::executor::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get result from MovJ action");
            result->done = false;
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Action failed");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "MovJ action succeeded with result code: %d", mov_j_result_future);


        auto disable_robot_request = std::make_shared<mg400_msgs::srv::DisableRobot::Request>();
        auto disable_robot_response_future = disable_robot_client->async_send_request(disable_robot_request);

        if (rclcpp::spin_until_future_complete(node, disable_robot_response_future) !=
            rclcpp::executor::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call enable_robot service");
            result->done = false;
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Action failed");
            return;
        }

        //diable here
        auto disable_robot_response = disable_robot_response_future.get();
        RCLCPP_INFO(this->get_logger(), "disable_robot_response: %d", disable_robot_response);


        
        result->done = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Action completed");
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto control_node = std::make_shared<MG400ControlNode>();
    RCLCPP_INFO(control_node->get_logger(), "MG400 control node starting");
    rclcpp::spin(control_node);
    RCLCPP_INFO(control_node->get_logger(), "MG400 control node finished");
    rclcpp::shutdown();
    return 0;
}
