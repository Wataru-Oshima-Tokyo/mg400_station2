#include <string>
#include <memory>
#include "mg400_interface/mg400_interface.hpp"

class CommanderCheckNode : public rclcpp::Node
{
public:
  std::unique_ptr<mg400_interface::MG400Interface> interface;

private:
  const std::string ip_address;

public:
  explicit CommanderCheckNode(const rclcpp::NodeOptions & options)
  : Node("commander_check_node", options),
    ip_address(this->declare_parameter("ip_address", "192.168.1.6"))
  {
    this->interface = std::make_unique<mg400_interface::MG400Interface>(
      this->ip_address);
  }

  bool configure()
  {
    return this->interface->configure();
  }

  bool activate()
  {
    return this->interface->activate();
  }

  bool deactivate()
  {
    return this->interface->deactivate();
  }

  double degreeToRadian(double degree) {
    return degree * (M_PI / 180.0);
  }
};


int main(int argc, char ** argv)
{
  using namespace std::chrono_literals;
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto ck_node = std::make_unique<CommanderCheckNode>(options);
  if (!ck_node->configure()) {
    return EXIT_FAILURE;
  }
  if (!ck_node->activate()) {
    return EXIT_FAILURE;
  }

  try {
    ck_node->interface->dashboard_commander->enableRobot();
    rclcpp::sleep_for(2s);

    ck_node->interface->dashboard_commander->tool(0);
    rclcpp::sleep_for(2s);

    ck_node->interface->motion_commander->mov_4axis(
      0.35, 0.0, 0.0, 0.0);
    rclcpp::sleep_for(2s);

    ck_node->interface->motion_commander->mov_4axis(
      0.35, 0.0, 0.0, ck_node->degreeToRadian(30));
    rclcpp::sleep_for(2s);

    ck_node->interface->dashboard_commander->disableRobot();

  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      ck_node->get_logger(),
      "%s", e.what());
    return EXIT_FAILURE;
  }

  ck_node->deactivate();

  return EXIT_SUCCESS;
}
