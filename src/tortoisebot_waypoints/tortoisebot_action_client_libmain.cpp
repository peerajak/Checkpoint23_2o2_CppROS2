#include "tortoisebot_action_client.h"

using WaypointAction = tortoisebot_waypoints::action::WaypointAction;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
  double goal_x = 1.8, goal_y = 0;

  auto action_client = std::make_shared<WaypointActionClient>(goal_x,goal_y);
  //action_client->send_goal(goal_x,goal_y);
  executor.add_node(action_client);
  while (!action_client->is_goal_done()) {
      executor.spin_some();
  }
  executor.remove_node(action_client);


  rclcpp::shutdown();
  return 0;
}