# mg400_station2

## Contents of this program

### mg400_station.cpp

This program defines a ROS2 node named `mg400_control_node` that provides a control interface for a MG400 robot. It communicates with the MG400 robot by calling services and actions provided by the robot.

When started, the node waits for goal requests on the `mg400_server` action server. When a goal request is received, it performs the following steps:

1. Calls the `/mg400/clear_error` service to clear any errors on the robot.

2. Calls the `/mg400/enable_robot` service to enable the robot.

3. Calls the `/mg400/speed_factor` service to set the speed factor of the robot to 70.

4. Performs a series of `MovJ` (Move Joint) actions. For each action, it sets a new goal pose with the `x` position offset by `0.01*i` (where `i` is the iteration index) from the base `x` position of `0.25`. The `w` component of the orientation quaternion is set to `1.0` when `i` is even and `-1.0` when `i` is odd. After sending each action goal, it waits for the action result and then sleeps for 2 seconds before the next action.

5. Calls the `/mg400/disable_robot` service to disable the robot after all actions are completed.

The node uses a separate thread to execute the actions so as not to block the main executor. This allows it to handle cancel requests promptly.

For error handling, if any service call or action fails, it aborts the goal with `result->done = false` and logs an error message.

The main function of the program starts the node and spins it to process callbacks, and then shuts down when the node finishes execution.

You can use this program to control a MG400 robot from another node by sending a goal request to the `mg400_server` action server.

Dependencies:
- ROS2 Foxy or above
- `geometry_msgs`
- `std_srvs`
- `mg400_msgs`
- `rclcpp_action`
- `techshare_ros_pkg2`


### mg400_pose_publisher.cpp

**Class definition and initialization**:
The `PosePublisherNode` is a class that inherits from the `rclcpp::Node` class, which is the main entry point to interact with the ROS2 system. It defines a constructor that initializes the node with the name `"pose_publisher_node"` and creates a publisher that will publish messages of type `geometry_msgs::msg::Pose` on the `"pose"` topic.

The `pose_publisher_` is a shared pointer to a `rclcpp::Publisher` object. It is initialized with `this->create_publisher<geometry_msgs::msg::Pose>("pose", 10)`, which creates a publisher that publishes on the "pose" topic with a queue size of 10.

**Running the node**:
The `run` function is the main loop of the node. It first creates a shared pointer to a `geometry_msgs::msg::Pose` message, which will hold the current pose of the end effector of the robot. It then creates a `mg400_interface::RealtimeFeedbackTcpInterface` object, which is used to get the current pose of the robot's end effector from the robot controller via a TCP/IP interface.

The loop then continuously gets the current pose of the robot's end effector, publishes it on the "pose" topic, and then sleeps for a short time to maintain a constant publishing rate of 10 Hz.

**Main function**:
The `main` function initializes the ROS2 system, creates an instance of the `PosePublisherNode`, runs the main loop of the node, and then shuts down the ROS2 system when the node is finished.

**Note**: The IP address `"192.168.1.6"` is hardcoded in this code. In practice, you would likely want to make this configurable, for example by passing it as a command-line argument or a ROS2 parameter.

Overall, this code provides a simple example of a ROS2 node that interfaces with a robot controller to get real-time feedback on the robot's state and publishes this information for other nodes to use.