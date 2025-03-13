#include <ur_ros_rtde_simple_clients/simple_action_client_template.hpp>
#include <ur_ros_rtde_simple_clients/simple_service_client_template.hpp>

/* INCLUDE ACTIONS TO USE */
#include <ur_ros_rtde_msgs/action/reset_force_torque_sensor.hpp>
#include <ur_ros_rtde_msgs/action/move_until_force.hpp>

/* INCLUDE SERVICES TO USE */
#include <ur_ros_rtde_msgs/srv/start_data_recording.hpp>
#include <std_srvs/srv/trigger.hpp>

#define SAVE_LOG true
const std::string log_filename = "/home/superdiodo/ros_ws/src/ur_ros_rtde/ur_ros_rtde/src/test/linear_moving_force_sensing_log";

int main(int argc, char **argv)
{

  // INITIALIZATION
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_server_node");
  auto node_act = rclcpp::Node::make_shared("test_server_act_node");
  auto node_srv = rclcpp::Node::make_shared("test_server_srv_node");
  auto action_client = simple_action_client_template(node_act);
  auto service_client = simple_service_client_template(node_srv);

  /* TEST SOMETHING HERE! */
  using ResetFTAction = ur_ros_rtde_msgs::action::ResetForceTorqueSensor;
  using MoveUntilForceAction = ur_ros_rtde_msgs::action::MoveUntilForce;

  // 1. Reset FT sensor
  auto goal_msg1 = ResetFTAction::Goal();
  action_client.send_goal<ResetFTAction>("ur_ros_rtde/reset_force_torque_sensor_command", goal_msg1);

  // (Optional) Start recording FT data
  if(SAVE_LOG){
    auto request = std::make_shared<ur_ros_rtde_msgs::srv::StartDataRecording::Request>();
    request->filename = log_filename;
    request->variables = {"actual_TCP_force", "target_qd"};
    auto response = std::make_shared<ur_ros_rtde_msgs::srv::StartDataRecording::Response>();
    service_client.send_goal<ur_ros_rtde_msgs::srv::StartDataRecording>("ur_ros_rtde/start_data_recording", request, response);
  }

  auto goal_msg2 = MoveUntilForceAction::Goal();
  goal_msg2.speed = 0.1;
  goal_msg2.acceleration = 0.1;
  goal_msg2.deceleration = 0.5;
  goal_msg2.force_th = 20;
  goal_msg2.forces_to_consider = {0.,0.,1.};
  goal_msg2.tool_position_movement = {0., 0., -1.};
  action_client.send_goal<MoveUntilForceAction>("ur_ros_rtde/move_until_force_command", goal_msg2);

  action_client.send_goal<ResetFTAction>("ur_ros_rtde/reset_force_torque_sensor_command", goal_msg1);

  if(SAVE_LOG){
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
    service_client.send_goal<std_srvs::srv::Trigger>("ur_ros_rtde/stop_data_recording", request, response);
  }

  rclcpp::shutdown();
  return 0;
}
