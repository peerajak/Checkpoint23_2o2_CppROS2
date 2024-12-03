#include <functional>
#include <memory>
#include <thread>
#include "tortoisebot_action_server.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include <math.h>

#define PI 3.1416

using WaypointAction = tortoisebot_waypoints::action::WaypointAction;


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<WaypointActionClass>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}