#include <ur_ros_rtde_simple_clients/simple_action_client_template.hpp>
#include <ur_ros_rtde_simple_clients/simple_service_client_template.hpp>

/* INCLUDE ACTIONS TO USE */
#include <ur_ros_rtde_msgs/action/move_l_relative.hpp>
#include <ur_ros_rtde_msgs/action/reset_force_torque_sensor.hpp>

/* INCLUDE SERVICES TO USE */
#include <ur_ros_rtde_msgs/srv/get_tcp_pose.hpp>

void print_pose(const geometry_msgs::msg::Pose &pose)
{
  std::cout << "Position: " << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << std::endl;
  std::cout << "Orientation: " << pose.orientation.x << ", " << pose.orientation.y << ", " << pose.orientation.z << ", " << pose.orientation.w << std::endl;
}

int main(int argc, char **argv)
{
  // INITIALIZATION
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("linear_movements_node");
  auto node_act = rclcpp::Node::make_shared("linear_movements_act_node");
  auto node_srv = rclcpp::Node::make_shared("linear_movements_srv_node");
  auto action_client = simple_action_client_template(node_act);
  auto service_client = simple_service_client_template(node_srv);

  using GetTcpPoseSrv = ur_ros_rtde_msgs::srv::GetTcpPose;
  using MoveLRelativeAction = ur_ros_rtde_msgs::action::MoveLRelative;

  // 1. Print initial robot tcp pose
  std::shared_ptr<GetTcpPoseSrv::Request> request_msg = std::make_shared<GetTcpPoseSrv::Request>();
  std::shared_ptr<GetTcpPoseSrv::Response> response_msg = std::make_shared<GetTcpPoseSrv::Response>();
  service_client.send_goal<GetTcpPoseSrv>("ur_ros_rtde/get_tcp_pose", request_msg, response_msg);
  print_pose(response_msg->pose);

  // 2. Execute small movements and print new pose
  auto goal_msg1 = MoveLRelativeAction::Goal();
  goal_msg1.position.x = 0.1;
  goal_msg1.speed = 0.1;
  goal_msg1.acceleration = 0.1;
  action_client.send_goal<MoveLRelativeAction>("ur_ros_rtde/move_l_relative_command", goal_msg1);
  service_client.send_goal<GetTcpPoseSrv>("ur_ros_rtde/get_tcp_pose", request_msg, response_msg);
  print_pose(response_msg->pose);

  auto goal_msg2 = MoveLRelativeAction::Goal();
  goal_msg2.position.x = -0.2;
  goal_msg2.speed = 0.1;
  goal_msg2.acceleration = 0.1;
  action_client.send_goal<MoveLRelativeAction>("ur_ros_rtde/move_l_relative_command", goal_msg2);
  service_client.send_goal<GetTcpPoseSrv>("ur_ros_rtde/get_tcp_pose", request_msg, response_msg);
  print_pose(response_msg->pose);

  auto goal_msg3 = MoveLRelativeAction::Goal();
  goal_msg3.position.x = 0.1;
  goal_msg3.speed = 0.01;
  goal_msg3.acceleration = 0.1;
  print_pose(response_msg->pose);

  // 3. Send an async goal and wait for termination
  std::shared_ptr<rclcpp_action::ClientGoalHandle<MoveLRelativeAction>> goal_handle;
  simple_action_client_template::async_goal_state<MoveLRelativeAction> goal_state;
  action_client.send_goal_async<MoveLRelativeAction>("ur_ros_rtde/move_l_relative_command", goal_msg3, goal_state);

  std::cout << "getting async action status" << std::endl;

  while (!action_client.get_async_action_status("ur_ros_rtde/move_l_relative_command", goal_state))
  {
    std::cout << "action not finished yet" << std::endl;
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  service_client.send_goal<GetTcpPoseSrv>("ur_ros_rtde/get_tcp_pose", request_msg, response_msg);
  print_pose(response_msg->pose);

  rclcpp::shutdown();
  return 0;
}
