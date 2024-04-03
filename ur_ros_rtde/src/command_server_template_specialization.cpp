#include <ur_ros_rtde/command_server_template.hpp>

//---MESSAGES---//
#include <geometry_msgs/msg/pose.hpp>
#include <ur_ros_rtde_msgs/msg/joints_configuration.hpp>
#include <ur_ros_rtde_msgs/msg/vector.hpp>

//---ACTIONS---//
#include <ur_ros_rtde_msgs/action/move_l.hpp>
#include <ur_ros_rtde_msgs/action/move_j.hpp>
#include <ur_ros_rtde_msgs/action/set_speed_slider.hpp>
#include <ur_ros_rtde_msgs/action/set_digital_pin.hpp>
#include <ur_ros_rtde_msgs/action/set_suction.hpp>
#include <ur_ros_rtde_msgs/action/set_deposit.hpp>
#include <ur_ros_rtde_msgs/action/set_payload.hpp>
#include <ur_ros_rtde_msgs/action/execute_trajectory.hpp>
#include <ur_ros_rtde_msgs/action/execute_path.hpp>
#include <ur_ros_rtde_msgs/action/move_until_contact.hpp>
#include <ur_ros_rtde_msgs/action/move_until_force.hpp>
#include <ur_ros_rtde_msgs/action/move_until_torque.hpp>
#include <ur_ros_rtde_msgs/action/move_l_relative.hpp>
#include <ur_ros_rtde_msgs/action/move_j_relative.hpp>
#include <ur_ros_rtde_msgs/action/reset_force_torque_sensor.hpp>
#include <ur_ros_rtde_msgs/action/set_freedrive.hpp>
#include <ur_ros_rtde_msgs/action/soft_gripper_control.hpp>

//---SERVICES---//
#include <ur_ros_rtde_msgs/srv/get_internal_state.hpp>
#include <ur_ros_rtde_msgs/srv/get_tcp_pose.hpp>
#include <ur_ros_rtde_msgs/srv/get_robot_configuration.hpp>

using namespace std::chrono_literals;
using namespace std::chrono;

//---CHECKS---//

void check_control_interface_connection(ur_rtde::RTDEControlInterface *rtde_control, rclcpp::Node::SharedPtr node)
{
  while (!rtde_control->isConnected())
  {
    RCLCPP_INFO(node->get_logger(), "Control interface disconnected, reconnect..");
    try
    {
      rtde_control->reconnect();
    }
    catch (...)
    {
      RCLCPP_WARN(node->get_logger(), "Reconnection failed, waiting some time..");
      rclcpp::sleep_for(std::chrono::milliseconds(250));
      return;
    }

    RCLCPP_INFO(node->get_logger(), "Control interface reconnected!");
  }
};

void check_receive_interface_connection(ur_rtde::RTDEReceiveInterface *rtde_receive, rclcpp::Node::SharedPtr node)
{
  while (!rtde_receive->isConnected())
  {
    RCLCPP_INFO(node->get_logger(), "Receive interface disconnected, reconnect..");
    try
    {
      rtde_receive->reconnect();
    }
    catch (...)
    {
      RCLCPP_WARN(node->get_logger(), "Reconnection failed, waiting some time..");
      rclcpp::sleep_for(std::chrono::milliseconds(250));
      return;
    }

    RCLCPP_INFO(node->get_logger(), "Receive interface reconnected!");
  }
};

//---EXECUTE SPECIALIZATION---//

// MoveL
template <>
void command_server_template<ur_ros_rtde_msgs::action::MoveL>::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_ros_rtde_msgs::action::MoveL>> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ur_ros_rtde_msgs::action::MoveL::Result>();
  check_control_interface_connection(rtde_control_, node_);
  result->result = rtde_control_->moveL({goal->position.x, goal->position.y, goal->position.z, goal->orientation.x,
                                         goal->orientation.y, goal->orientation.z},
                                        goal->speed, goal->acceleration);
  RCLCPP_INFO(self::node_->get_logger(), (result->result ? "MoveL succeded" : "MoveL failed"));
  result->result ? goal_handle->succeed(result) : goal_handle->abort(result);
};

// MoveJ
template <>
void command_server_template<ur_ros_rtde_msgs::action::MoveJ>::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_ros_rtde_msgs::action::MoveJ>> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ur_ros_rtde_msgs::action::MoveJ::Result>();
  check_control_interface_connection(rtde_control_, node_);
  result->result = rtde_control_->moveJ(goal->joint_position, goal->speed, goal->acceleration);
  RCLCPP_INFO(self::node_->get_logger(), (result->result ? "MoveJ succeded" : "MoveJ failed"));
  result->result ? goal_handle->succeed(result) : goal_handle->abort(result);
};

// SetDigitalPin
template <>
void command_server_template<ur_ros_rtde_msgs::action::SetDigitalPin>::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_ros_rtde_msgs::action::SetDigitalPin>> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ur_ros_rtde_msgs::action::SetDigitalPin::Result>();
  result->result = rtde_io_->setStandardDigitalOut(goal->pin, goal->state);
  RCLCPP_INFO(self::node_->get_logger(), (result->result ? "SetDigitalPin succeded" : "SetDigitalPin failed"));
  result->result ? goal_handle->succeed(result) : goal_handle->abort(result);
};

// SetSuction
template <>
void command_server_template<ur_ros_rtde_msgs::action::SetSuction>::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_ros_rtde_msgs::action::SetSuction>> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ur_ros_rtde_msgs::action::SetSuction::Result>();
  result->result = rtde_io_->setStandardDigitalOut(params_.suction_pin, goal->state);
  RCLCPP_INFO(self::node_->get_logger(), (result->result ? "SetSuction succeded" : "SetSuction failed"));
  result->result ? goal_handle->succeed(result) : goal_handle->abort(result);
};

// SetDeposit
template <>
void command_server_template<ur_ros_rtde_msgs::action::SetDeposit>::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_ros_rtde_msgs::action::SetDeposit>> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ur_ros_rtde_msgs::action::SetDeposit::Result>();
  result->result = rtde_io_->setStandardDigitalOut(params_.deposit_pin, goal->state);
  RCLCPP_INFO(self::node_->get_logger(), (result->result ? "SetDeposit succeded" : "SetDeposit failed"));
  result->result ? goal_handle->succeed(result) : goal_handle->abort(result);
};

// SetPayload
template <>
void command_server_template<ur_ros_rtde_msgs::action::SetPayload>::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_ros_rtde_msgs::action::SetPayload>> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ur_ros_rtde_msgs::action::SetPayload::Result>();
  check_control_interface_connection(rtde_control_, node_);
  result->result = rtde_control_->setPayload(
      goal->mass, {goal->center_of_gravity.x, goal->center_of_gravity.y, goal->center_of_gravity.z});
  RCLCPP_INFO(self::node_->get_logger(), (result->result ? "SetPayload succeded" : "SetPayload failed"));
  result->result ? goal_handle->succeed(result) : goal_handle->abort(result);
};

// SetSpeedSlider
template <>
void command_server_template<ur_ros_rtde_msgs::action::SetSpeedSlider>::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_ros_rtde_msgs::action::SetSpeedSlider>> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ur_ros_rtde_msgs::action::SetSpeedSlider::Result>();
  result->result = rtde_io_->setSpeedSlider(goal->speedslider);
  RCLCPP_INFO(self::node_->get_logger(), (result->result ? "SetSpeedSlider succeded" : "SetSpeedSlider failed"));
  result->result ? goal_handle->succeed(result) : goal_handle->abort(result);
};

// SetFreedrive
template <>
void command_server_template<ur_ros_rtde_msgs::action::SetFreedrive>::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_ros_rtde_msgs::action::SetFreedrive>> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ur_ros_rtde_msgs::action::SetFreedrive::Result>();

  if (goal->activated)
  {
    result->result = rtde_control_->freedriveMode(goal->free_axes, goal->feature);
  }
  else
    result->result = rtde_control_->endFreedriveMode();
  RCLCPP_INFO(self::node_->get_logger(), (result->result ? "SetFreedrive succeded" : "SetFreedrive failed"));
  result->result ? goal_handle->succeed(result) : goal_handle->abort(result);
};

// SoftGripperControl
template <>
void command_server_template<ur_ros_rtde_msgs::action::SoftGripperControl>::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_ros_rtde_msgs::action::SoftGripperControl>> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ur_ros_rtde_msgs::action::SoftGripperControl::Result>();
  dashboard_client_->connect();
  dashboard_client_->loadURP("sg_control_program.urp");
  rclcpp::sleep_for(std::chrono::milliseconds(200));
  std::cout << "goal grip: " << goal->grip << std::endl;
  rtde_io_->setInputIntRegister(params_.grip_bool_input_register, goal->grip);

  int desired_width = goal->target_width;

  rtde_io_->setInputIntRegister(params_.desired_width_input_register, desired_width);
  try
  {

    rtde_control_->disconnect();
    dashboard_client_->play();

    rclcpp::sleep_for(std::chrono::milliseconds(300));
    while (dashboard_client_->programState() != "STOPPED sg_control_program.urp")
    {
      rclcpp::sleep_for(std::chrono::milliseconds(50));
      std::cout << dashboard_client_->programState() << std::endl;
    }

    int reached_width = rtde_receive_->getOutputIntRegister(params_.feedback_width_output_register);
    result->result = true;
    result->error = desired_width - reached_width;
  }
  catch (...)
  {
    result->result = false;
  }

  dashboard_client_->disconnect();
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  check_control_interface_connection(rtde_control_, node_);
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  rtde_control_->reuploadScript();
  RCLCPP_INFO(self::node_->get_logger(), (result->result ? "SoftGripperControl succeded" : "SoftGripperControl failed"));
  result->result ? goal_handle->succeed(result) : goal_handle->abort(result);
};

// ExecutePath
template <>
void command_server_template<ur_ros_rtde_msgs::action::ExecutePath>::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_ros_rtde_msgs::action::ExecutePath>> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ur_ros_rtde_msgs::action::ExecutePath::Result>();
  ur_rtde::Path path;
  std::vector<std::vector<double>> traj;

  for (std::size_t i = 0; i < goal->waypoints.size(); i++)
  {
    traj.push_back({goal->waypoints[i].vector[0], goal->waypoints[i].vector[1], goal->waypoints[i].vector[2],
                    goal->waypoints[i].vector[3], goal->waypoints[i].vector[4], goal->waypoints[i].vector[5],
                    goal->speed[i], goal->acceleration[i], goal->blend[i]});
    if (goal->move_type[i] == "L" || goal->move_type[i] == "l")
    {
      path.addEntry({ur_rtde::PathEntry::MoveL, ur_rtde::PathEntry::PositionTcpPose, traj.back()});
    }
    else if (goal->move_type[i] == "J" || goal->move_type[i] == "j")
    {
      path.addEntry({ur_rtde::PathEntry::MoveJ, ur_rtde::PathEntry::PositionJoints, traj.back()});
    }
  }
  check_control_interface_connection(rtde_control_, node_);
  result->result = rtde_control_->movePath(path, false);

  RCLCPP_INFO(self::node_->get_logger(),
              (result->result ? "ExecutePath succeded" : "ExecutePath failed"));

  result->result ? goal_handle->succeed(result) : goal_handle->abort(result);
};

// MoveUntilContact
template <>
void command_server_template<ur_ros_rtde_msgs::action::MoveUntilContact>::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_ros_rtde_msgs::action::MoveUntilContact>> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ur_ros_rtde_msgs::action::MoveUntilContact::Result>();
  check_control_interface_connection(rtde_control_, node_);
  result->result = rtde_control_->moveUntilContact(goal->toolspeed, goal->direction, goal->acceleration);

  RCLCPP_INFO(self::node_->get_logger(),
              (result->result ? "MoveUntilContact succeded" : "MoveUntilContact failed"));
  result->result ? goal_handle->succeed(result) : goal_handle->abort(result);
};

// MoveUntilForce
template <>
void command_server_template<ur_ros_rtde_msgs::action::MoveUntilForce>::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_ros_rtde_msgs::action::MoveUntilForce>> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ur_ros_rtde_msgs::action::MoveUntilForce::Result>();
  check_control_interface_connection(rtde_control_, node_);
  check_receive_interface_connection(rtde_receive_, node_);

  auto actual_pose = rtde_receive_->getActualTCPPose();
  actual_pose[0] += goal->tool_position_movement[0];
  actual_pose[1] += goal->tool_position_movement[1];
  actual_pose[2] += goal->tool_position_movement[2];

  rtde_control_->zeroFtSensor();
  rtde_control_->moveL(actual_pose, goal->speed, goal->acceleration, true);

  while (true)
  {
    auto f = rtde_receive_->getActualTCPForce();
    auto eigen_f =
        Eigen::Vector3d(double(goal->forces_to_consider[0]) * f[0], double(goal->forces_to_consider[1]) * f[1],
                        double(goal->forces_to_consider[2]) * f[2]);
    auto f_norm = eigen_f.norm();
    if (f_norm > goal->force_th)
    {
      rtde_control_->stopL(goal->deceleration);
      break;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(params_.move_until_force_collision_check_freq));
  }

  result->result = true;

  RCLCPP_INFO(self::node_->get_logger(),
              (result->result ? "MoveUntilForce succeded" : "MoveUntilForce failed"));
  result->result ? goal_handle->succeed(result) : goal_handle->abort(result);
};

// MoveUntilTorque
template <>
void command_server_template<ur_ros_rtde_msgs::action::MoveUntilTorque>::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_ros_rtde_msgs::action::MoveUntilTorque>> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ur_ros_rtde_msgs::action::MoveUntilTorque::Result>();
  check_control_interface_connection(rtde_control_, node_);
  check_receive_interface_connection(rtde_receive_, node_);

  auto actual_pose = rtde_receive_->getActualTCPPose();
  actual_pose[0] += goal->tool_position_movement[0];
  actual_pose[1] += goal->tool_position_movement[1];
  actual_pose[2] += goal->tool_position_movement[2];

  rtde_control_->zeroFtSensor();
  rtde_control_->moveL(actual_pose, goal->speed, goal->acceleration, true);

  while (true)
  {
    auto f = rtde_receive_->getActualTCPForce();
    auto eigen_f =
        Eigen::Vector3d(double(goal->torques_to_consider[0]) * f[3], double(goal->torques_to_consider[1]) * f[4],
                        double(goal->torques_to_consider[2]) * f[5]);
    auto f_norm = eigen_f.norm();
    if (f_norm > goal->torque_th)
    {
      rtde_control_->stopL(goal->deceleration);
      break;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(params_.move_until_force_collision_check_freq));
  }

  /* retraction
  
  actual_pose = rtde_receive_->getActualTCPPose();
  actual_pose[0] -= goal->tool_position_movement[0];
  actual_pose[1] -= goal->tool_position_movement[1];
  actual_pose[2] -= goal->tool_position_movement[2]; 
  rtde_control_->moveL(actual_pose, goal->speed, goal->acceleration, true);

  while (true)
  {
    auto f = rtde_receive_->getActualTCPForce();
    auto eigen_f = Eigen::Vector3d(f[3],f[4],f[5]);
    auto f_norm = eigen_f.norm();
    if (f_norm <= goal->torque_th/2)
    {
      rtde_control_->stopL(goal->deceleration);
      break;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(params_.move_until_force_collision_check_freq));
  }
  */

  result->result = true;

  RCLCPP_INFO(self::node_->get_logger(),
              (result->result ? "MoveUntilTorque succeded" : "MoveUntilTorque failed"));
  result->result ? goal_handle->succeed(result) : goal_handle->abort(result);
};

// MoveLRelative
template <>
void command_server_template<ur_ros_rtde_msgs::action::MoveLRelative>::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_ros_rtde_msgs::action::MoveLRelative>> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ur_ros_rtde_msgs::action::MoveLRelative::Result>();

  check_control_interface_connection(rtde_control_, node_);
  check_receive_interface_connection(rtde_receive_, node_);

  geometry_msgs::msg::Pose tcp_pose;

  std::vector<double> flange_pose = rtde_receive_->getActualTCPPose();
  tcp_pose.position.x = flange_pose[0];
  tcp_pose.position.y = flange_pose[1];
  tcp_pose.position.z = flange_pose[2];

  Eigen::Vector3d rotation_vector(flange_pose[3], flange_pose[4], flange_pose[5]);
  double angle = rotation_vector.norm();

  Eigen::Quaterniond flange_q;
  flange_q = Eigen::AngleAxisd(angle, rotation_vector / angle);

  Eigen::Quaterniond goal_q;
  goal_q = Eigen::AngleAxisd(goal->orientation.z, Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(goal->orientation.y, Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(goal->orientation.x, Eigen::Vector3d::UnitX());

  goal_q = goal_q * flange_q;
  goal_q.normalize();

  double w = 2 * acos(goal_q.w());
  double rx = (goal_q.x() / sqrt(1 - pow(goal_q.w(), 2))) * w;
  double ry = (goal_q.y() / sqrt(1 - pow(goal_q.w(), 2))) * w;
  double rz = (goal_q.z() / sqrt(1 - pow(goal_q.w(), 2))) * w;

  result->result =
      rtde_control_->moveL({goal->position.x + tcp_pose.position.x, goal->position.y + tcp_pose.position.y,
                            goal->position.z + tcp_pose.position.z, rx, ry, rz},
                           goal->speed, goal->acceleration);

  RCLCPP_INFO(self::node_->get_logger(),
              (result->result ? "MoveLRelative: succeded" : "MoveLRelative failed"));
  result->result ? goal_handle->succeed(result) : goal_handle->abort(result);
};

// MoveJRelative
template <>
void command_server_template<ur_ros_rtde_msgs::action::MoveJRelative>::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_ros_rtde_msgs::action::MoveJRelative>> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ur_ros_rtde_msgs::action::MoveJRelative::Result>();

  check_control_interface_connection(rtde_control_, node_);
  check_receive_interface_connection(rtde_receive_, node_);

  auto actual_robot_configuration = rtde_receive_->getActualQ();

  result->result =
      rtde_control_->moveJ({goal->joint_position[0] + actual_robot_configuration[0], goal->joint_position[1] + actual_robot_configuration[1],
                            goal->joint_position[2] + actual_robot_configuration[2], goal->joint_position[3] + actual_robot_configuration[3],
                            goal->joint_position[4] + actual_robot_configuration[4], goal->joint_position[5] + actual_robot_configuration[5]},
                           goal->speed, goal->acceleration);

  RCLCPP_INFO(self::node_->get_logger(),
              (result->result ? "MoveJRelative: succeded" : "MoveLRelative failed"));
  result->result ? goal_handle->succeed(result) : goal_handle->abort(result);
};

// ResetForceTorqueSensor
template <>
void command_server_template<ur_ros_rtde_msgs::action::ResetForceTorqueSensor>::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_ros_rtde_msgs::action::ResetForceTorqueSensor>>
        goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ur_ros_rtde_msgs::action::ResetForceTorqueSensor::Result>();
  check_control_interface_connection(rtde_control_, node_);

  result->result = rtde_control_->zeroFtSensor();
  RCLCPP_INFO(self::node_->get_logger(),
              (result->result ? "ResetForceTorqueSensor succeded" : "ResetForceTorqueSensor failed"));
  result->result ? goal_handle->succeed(result) : goal_handle->abort(result);
};

// ExecuteTrajectory

std::vector<double> get_times(const std::vector<ur_ros_rtde_msgs::msg::Vector> &trajectory, const double &a, const double &v)
{
  std::vector<std::vector<double>> velocities_forward;
  std::vector<double> old_diff = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> old_vel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  for (size_t i = 1; i < trajectory.size(); i++)
  {
    std::vector<double> motors_velocities;
    for (size_t j = 0; j < trajectory[i].vector.size(); j++)
    {
      double diff = trajectory[i - 1].vector[j] - trajectory[i].vector[j];
      double new_v = 0.0;
      if (diff * old_diff[j] >= 0)
        new_v = std::min(v, sqrt(2 * a * abs(diff) + pow(old_vel[j], 2)));
      else
        new_v = std::min(v, sqrt(a * abs(diff)));

      motors_velocities.push_back(new_v);
      old_diff[j] = diff;
      old_vel[j] = new_v;
    }

    velocities_forward.push_back(motors_velocities);
  }

  std::vector<std::vector<double>> velocities_backward;

  old_diff = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  old_vel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  for (int i = trajectory.size() - 2; i >= 0; i--)
  {
    std::vector<double> motors_velocities;

    for (size_t j = 0; j < trajectory[i].vector.size(); j++)
    {
      double diff = trajectory[i + 1].vector[j] - trajectory[i].vector[j];
      double new_v = 0.0;
      if (diff * old_diff[j] >= 0)
        new_v = std::min(v, sqrt(2 * a * abs(diff) + pow(old_vel[j], 2)));
      else
        new_v = std::min(v, sqrt(a * abs(diff)));

      motors_velocities.push_back(new_v);
      old_diff[j] = diff;
      old_vel[j] = new_v;
    }

    velocities_backward.push_back(motors_velocities);
  }

  std::reverse(velocities_backward.begin(), velocities_backward.end());

  std::vector<std::vector<double>> velocities_min;

  for (size_t i = 0; i < velocities_forward.size(); i++)
  {
    std::vector<double> motors_velocities;
    for (size_t j = 0; j < trajectory[i].vector.size(); j++)
    {
      motors_velocities.push_back(std::min(velocities_forward[i][j], velocities_backward[i][j]));
    }
    velocities_min.push_back(motors_velocities);
  }

  std::vector<double> times;
  times.push_back(0.0);
  for (size_t i = 1; i < trajectory.size(); i++)
  {
    double max_time = 0.0;

    for (size_t j = 0; j < trajectory[i].vector.size(); j++)
    {
      double diff = trajectory[i - 1].vector[j] - trajectory[i].vector[j];
      double time = abs(diff) / (velocities_min[i - 1][j]);

      if (time > max_time)
        max_time = time;
    }

    times.push_back(max_time);
  }

  return times;
};

std::vector<std::vector<double>> parametrize_traj(const double &dt, const double &acceleration, const double &velocity,
                                                  const std::vector<ur_ros_rtde_msgs::msg::Vector> &trajectory)
{
  std::cout << "Parametrizing trajectory with " << trajectory.size() << " waypoints, max speed: " << velocity << ", max acceleration: " << acceleration << std::endl;
  std::vector<double> times = get_times(trajectory, acceleration, velocity);
  std::vector<std::vector<double>> param_traj;

  for (size_t i = 1; i < trajectory.size(); i++)
  {

    if (times[i] > dt)
    {
      auto curr_q = trajectory[i].vector;
      auto prev_q = trajectory[i - 1].vector;

      double num_interpolation = times[i] / dt;

      std::cout << "waypoint " << i << " must be interpolated " << num_interpolation
                << " times (time diff: " << times[i] << " s)" << std::endl;

      for (double k = 0; k <= 1.0; k += 1.0 / num_interpolation)
      {
        std::vector<double> interpolated_q;
        for (size_t j = 0; j < curr_q.size(); j++)
          interpolated_q.push_back((1 - k) * prev_q[j] + k * curr_q[j]);

        param_traj.push_back(interpolated_q);
      }
    }

    param_traj.push_back(trajectory[i].vector);
  }

  return param_traj;
};

template <>
void command_server_template<ur_ros_rtde_msgs::action::ExecuteTrajectory>::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_ros_rtde_msgs::action::ExecuteTrajectory>> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ur_ros_rtde_msgs::action::ExecuteTrajectory::Result>();
  check_control_interface_connection(rtde_control_, node_);
  check_receive_interface_connection(rtde_receive_, node_);

  if (goal->trajectory.size() <= 1)
  {
    result->result = true;
    RCLCPP_INFO(self::node_->get_logger(), "Terminating execution, the passed trajectory has only %ld waypoints", goal->trajectory.size());
    return;
  }

  auto p_trajectory = parametrize_traj(params_.parametrization_timestep, goal->acceleration, goal->speed, goal->trajectory);

  int sleep_dt = params_.servoJ_timestep * 1000;

  rtde_control_->moveJ(p_trajectory[0], 0.2, 0.2);
  rtde_control_->stopJ();
  steady_clock::time_point t_start;

  double time = 0.0;
  for (auto p : p_trajectory)
  {
    t_start = rtde_control_->initPeriod();
    result->result = rtde_control_->servoJ(p, goal->speed, goal->acceleration, params_.servoJ_timestep,
                                           params_.servoJ_lookahead_time, params_.servoJ_gain);
    rtde_control_->waitPeriod(t_start);

    rclcpp::sleep_for(std::chrono::milliseconds(sleep_dt));
    if (!result->result)
      break;

    time += params_.servoJ_timestep;
    if (time >= 1.0 / (double)params_.deviation_check_freq)
    {
      time = 0;
      check_receive_interface_connection(rtde_receive_, node_);
      auto q = rtde_receive_->getActualQ();

      Eigen::VectorXd actual_conf = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q.data(), q.size());
      Eigen::VectorXd desired_conf = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(p.data(), p.size());
      auto deviation = (actual_conf - desired_conf).norm();
      if (deviation > params_.max_deviation)
      {
        result->result = false;
        RCLCPP_INFO(self::node_->get_logger(), "Aborting because the robot deviation is greater than %f rad", params_.max_deviation);
        break;
      }
    }
  }

  Eigen::VectorXd desired_conf = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(p_trajectory[p_trajectory.size() - 1].data(), p_trajectory[p_trajectory.size() - 1].size());

  while (true)
  {
    t_start = rtde_control_->initPeriod();
    result->result = rtde_control_->servoJ(p_trajectory[p_trajectory.size() - 1], goal->speed, goal->acceleration, params_.servoJ_timestep,
                                           params_.servoJ_lookahead_time, params_.servoJ_gain);
    rtde_control_->waitPeriod(t_start);
    rclcpp::sleep_for(std::chrono::milliseconds(sleep_dt));

    check_receive_interface_connection(rtde_receive_, node_);
    auto q = rtde_receive_->getActualQ();
    Eigen::VectorXd actual_conf = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q.data(), q.size());

    auto deviation = (actual_conf - desired_conf).norm();
    if (deviation < 0.0005)
    {
      result->result = true;
      RCLCPP_INFO(self::node_->get_logger(), "Reached a distance of %f rad from goal, terminating trajectory execution", deviation);
      break;
    }
  }

  rtde_control_->servoStop(goal->deceleration);
  rtde_control_->moveJ(p_trajectory[p_trajectory.size() - 1], 0.2, 0.2);
  rtde_control_->stopJ();

  RCLCPP_INFO(self::node_->get_logger(),
              (result->result ? "ExecuteTrajectory succeded" : "ExecuteTrajectory failed"));

  result->result ? goal_handle->succeed(result) : goal_handle->abort(result);
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ur_ros_rtde_command_server_node");
  auto robot_ip = node->declare_parameter<std::string>("robot_ip", "127.0.0.1");

  internal_params params;
  params.suction_pin = node->declare_parameter<int>("suction_pin", 0);
  params.deposit_pin = node->declare_parameter<int>("deposit_pin", 1);
  params.move_until_force_collision_check_freq =
      node->declare_parameter<int>("move_until_force_collision_check_freq", 10);
  params.servoJ_gain = node->declare_parameter<int>("servoJ_gain", 200);
  params.servoJ_lookahead_time = node->declare_parameter<double>("servoJ_lookahead_time", 0.2);
  params.servoJ_timestep = node->declare_parameter<double>("servoJ_timestep", 0.002);
  params.receiver_freq = node->declare_parameter<int>("receiver_freq", 100);
  params.deviation_check_freq = node->declare_parameter<int>("deviation_check_freq", 100);
  params.max_deviation = node->declare_parameter<double>("max_deviation", 0.15);
  params.parametrization_timestep = node->declare_parameter<double>("parametrization_timestep", 0.002);

  ur_rtde::RTDEControlInterface *rtde_control = new ur_rtde::RTDEControlInterface(robot_ip);
  ur_rtde::RTDEIOInterface *rtde_io = new ur_rtde::RTDEIOInterface(robot_ip);
  ur_rtde::RTDEReceiveInterface *rtde_receive = new ur_rtde::RTDEReceiveInterface(robot_ip, params.receiver_freq);
  ur_rtde::DashboardClient *dashboard_client = new ur_rtde::DashboardClient(robot_ip);

  auto server_SetDigitalPin = command_server_template<ur_ros_rtde_msgs::action::SetDigitalPin>(
      node, "set_digital_pin_command", params, rtde_control, rtde_io, rtde_receive, dashboard_client);
  auto server_SetDeposit = command_server_template<ur_ros_rtde_msgs::action::SetDeposit>(
      node, "set_deposit_command", params, rtde_control, rtde_io, rtde_receive, dashboard_client);
  auto server_SetSuction = command_server_template<ur_ros_rtde_msgs::action::SetSuction>(
      node, "set_suction_command", params, rtde_control, rtde_io, rtde_receive, dashboard_client);
  auto server_SetPayload = command_server_template<ur_ros_rtde_msgs::action::SetPayload>(
      node, "set_payload_command", params, rtde_control, rtde_io, rtde_receive, dashboard_client);
  auto server_SetSpeedslider = command_server_template<ur_ros_rtde_msgs::action::SetSpeedSlider>(
      node, "set_speed_slider_command", params, rtde_control, rtde_io, rtde_receive, dashboard_client);
  auto server_ResetForceTorqueSensor = command_server_template<ur_ros_rtde_msgs::action::ResetForceTorqueSensor>(
      node, "reset_force_torque_sensor_command", params, rtde_control, rtde_io, rtde_receive, dashboard_client);
  auto server_MoveL = command_server_template<ur_ros_rtde_msgs::action::MoveL>(
      node, "move_l_command", params, rtde_control, rtde_io, rtde_receive, dashboard_client);
  auto server_MoveJ = command_server_template<ur_ros_rtde_msgs::action::MoveJ>(
      node, "move_j_command", params, rtde_control, rtde_io, rtde_receive, dashboard_client);
  auto server_MoveUntilContact = command_server_template<ur_ros_rtde_msgs::action::MoveUntilContact>(
      node, "move_until_contact_command", params, rtde_control, rtde_io, rtde_receive, dashboard_client);
  auto server_MoveUntilForce = command_server_template<ur_ros_rtde_msgs::action::MoveUntilForce>(
      node, "move_until_force_command", params, rtde_control, rtde_io, rtde_receive, dashboard_client);
  auto server_MoveUntilTorque = command_server_template<ur_ros_rtde_msgs::action::MoveUntilTorque>(
      node, "move_until_torque_command", params, rtde_control, rtde_io, rtde_receive, dashboard_client);
  auto server_MoveLRelative = command_server_template<ur_ros_rtde_msgs::action::MoveLRelative>(
      node, "move_l_relative_command", params, rtde_control, rtde_io, rtde_receive, dashboard_client);
  auto server_MoveJRelative = command_server_template<ur_ros_rtde_msgs::action::MoveJRelative>(
      node, "move_j_relative_command", params, rtde_control, rtde_io, rtde_receive, dashboard_client);
  auto server_ExecuteTrajectory = command_server_template<ur_ros_rtde_msgs::action::ExecuteTrajectory>(
      node, "execute_trajectory_command", params, rtde_control, rtde_io, rtde_receive, dashboard_client);
  auto server_ExecutePath = command_server_template<ur_ros_rtde_msgs::action::ExecutePath>(
      node, "execute_path_command", params, rtde_control, rtde_io, rtde_receive, dashboard_client);
  auto server_ExecuteGrip = command_server_template<ur_ros_rtde_msgs::action::SoftGripperControl>(
      node, "soft_gripper_control_command", params, rtde_control, rtde_io, rtde_receive, dashboard_client);
  auto server_SetFreedrive = command_server_template<ur_ros_rtde_msgs::action::SetFreedrive>(
      node, "set_freedrive_command", params, rtde_control, rtde_io, rtde_receive, dashboard_client);

  RCLCPP_INFO(node->get_logger(), "\n< Action servers ready >\n");
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
