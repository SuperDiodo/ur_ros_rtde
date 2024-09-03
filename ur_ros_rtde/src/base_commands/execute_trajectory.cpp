#include <ur_ros_rtde/command_base_class.hpp>
#include <ur_ros_rtde_msgs/action/execute_trajectory.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "trajectory_time_planning.h"

// ---------- PLUGIN INFO ------------------
#define PLUGIN_NAME "execute_trajectory_command"
#define PLUGIN_CLASS_NAME ExecuteTrajectory
using action_type = ur_ros_rtde_msgs::action::ExecuteTrajectory;
// -----------------------------------------

using namespace std::chrono_literals;
using namespace std::chrono;

inline std::vector<Array6d> msg_to_Array6dVector(const std::vector<ur_ros_rtde_msgs::msg::Vector> &trajectory)
{
  std::vector<Array6d> traj_arr;
  traj_arr.reserve(trajectory.size());
  Array6d arr;
  for (size_t i = 0; i < trajectory.size(); i++)
  {
    for (size_t j = 0; j < NUM_JOINTS; j++)
      arr[j] = trajectory[i].vector[j];
    traj_arr.push_back(arr);
  }
  return traj_arr;
}

inline bool parameterize_traj(rclcpp::Node::SharedPtr node,
                              const double &acceleration, const double &speed,
                              const std::vector<ur_ros_rtde_msgs::msg::Vector> &trajectory,
                              TrajectoryFragments &fragments)
{
  std::vector<Array6d> traj_arr = msg_to_Array6dVector(trajectory);
  std::vector<double> times;
  std::vector<Array6d> velocities;
  uint64_t iterations;
  const bool success = trajectory_fragments_planning(traj_arr, acceleration, speed, times, velocities, iterations, fragments);
  if (!success)
  {
    RCLCPP_ERROR(node->get_logger(), "Trajectory parameterization failed!");
    return false;
  }

  RCLCPP_INFO(node->get_logger(), "Trajectory parameterization succeeded after %d iterations.", int(iterations));
  return true;
}

inline std::shared_ptr<ur_ros_rtde_msgs::action::ExecuteTrajectory::Feedback> create_feedback(const uint64_t reached_waypoint)
{
  auto feedback = std::make_shared<ur_ros_rtde_msgs::action::ExecuteTrajectory::Feedback>();
  feedback->reached_waypoint = reached_waypoint;
  return feedback;
}

void execute_function_impl(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<action_type>> goal_handle,
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<ur_rtde::RTDEControlInterface> rtde_control,
    std::shared_ptr<ur_rtde::RTDEIOInterface> rtde_io,
    std::shared_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive,
    std::shared_ptr<ur_rtde::DashboardClient> dashboard_client)
{

  (void)rtde_io;
  (void)dashboard_client;

  auto save_log = node->get_parameter(PLUGIN_NAME ".save_log").as_bool();
  auto log_filepath = node->get_parameter(PLUGIN_NAME ".log_filepath").as_string();
  auto servo_j_lookahead_time = node->get_parameter(PLUGIN_NAME ".servo_j.lookahead_time").as_double();
  auto servo_j_timestep = node->get_parameter(PLUGIN_NAME ".servo_j.timestep").as_double();
  auto servo_j_gain = node->get_parameter(PLUGIN_NAME ".servo_j.gain").as_int();
  auto max_allowed_deviation = node->get_parameter(PLUGIN_NAME ".max_allowed_deviation").as_int();

  RCLCPP_INFO(node->get_logger(), "Received traj. execution request");

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ur_ros_rtde_msgs::action::ExecuteTrajectory::Result>();
  check_control_interface_connection(rtde_control, node);
  check_receive_interface_connection(rtde_receive, node);

  if (goal->trajectory.size() <= 1)
  {
    RCLCPP_INFO(node->get_logger(), "Terminating execution, the passed trajectory has only %ld waypoints", goal->trajectory.size());
    result->result = false;
    goal_handle->abort(result);
    return;
  }

  // std::vector<std::array<std::shared_ptr<const std::string>, NUM_JOINTS>> comments;
  TrajectoryFragments fragments;
  if (!parameterize_traj(node, goal->acceleration, goal->speed, goal->trajectory, fragments))
  {
    RCLCPP_ERROR(node->get_logger(), "Terminating execution, trajectory parameterization failed!");
    result->result = false;
    goal_handle->abort(result);
    return;
  }

  const size_t num_trajectory_samples = trajectory_get_num_trajectory_samples(fragments, servo_j_timestep);
  Array6size p_cache = trajectory_create_cache();
  Array6size prev_p_cache = trajectory_create_cache();

  if (num_trajectory_samples == 0)
  {
    RCLCPP_ERROR(node->get_logger(), "Terminating execution, trajectory parameterization has no points!");
    result->result = false;
    goal_handle->abort(result);
    return;
  }

  const double expected_trajectory_duration = num_trajectory_samples * servo_j_timestep;
  RCLCPP_INFO(node->get_logger(), "Expected trajectory duration is %f seconds (%d samples).",
              float(expected_trajectory_duration), int(num_trajectory_samples));

  if (expected_trajectory_duration > 24.0f * 3600.0f)
  {
    RCLCPP_ERROR(node->get_logger(), "Terminating execution, trajectory duration is longer than a day! (sanity check)");
    result->result = false;
    goal_handle->abort(result);
    return;
  }

  const TrajectorySample start_p = trajectory_extract_sample(fragments, 0, p_cache, servo_j_timestep);
  rtde_control->moveJ(Array6dToVector(start_p.p), 0.2, 0.2);
  rtde_control->stopJ();

  goal_handle->publish_feedback(create_feedback(0));
  uint64_t feedback_prev_waypoint_id = 1;

  steady_clock::time_point t_start;

  std::ofstream file;
  if (save_log)
    file = std::ofstream(log_filepath);

  int curr_waypoint_id = -1, prev_waypoint_id = -1;
  std::string underscore_token;

  int waypoint_to_check = 0 - int(servo_j_lookahead_time / servo_j_timestep);
  for (size_t p_index = 0; p_index < num_trajectory_samples; p_index++)
  {
    const TrajectorySample p = trajectory_extract_sample(fragments, p_index, p_cache, servo_j_timestep);

    t_start = rtde_control->initPeriod();
    result->result = rtde_control->servoJ(Array6dToVector(p.p), goal->speed, goal->acceleration, servo_j_timestep,
                                          servo_j_lookahead_time, servo_j_gain);
    rtde_control->waitPeriod(t_start);

    if (!result->result)
      break;

    if (waypoint_to_check >= 0)
    {
      check_receive_interface_connection(rtde_receive, node);
      auto q = rtde_receive->getActualQ();

      const TrajectorySample prev_p = trajectory_extract_sample(fragments, waypoint_to_check, prev_p_cache, servo_j_timestep);

      const uint64_t feedback_waypoint_id = prev_p.comments[0]->waypoint;
      while (feedback_prev_waypoint_id < feedback_waypoint_id)
      {
        // we reached the previous waypoint, increase
        goal_handle->publish_feedback(create_feedback(feedback_prev_waypoint_id));
        feedback_prev_waypoint_id++;
      }

      if (save_log)
      {
        for (auto p_j : prev_p.p)
          file << p_j << ",";
        for (auto q_j : q)
          file << q_j << ",";
        curr_waypoint_id = (p.comments[0]->primitive == JointTrajFrag::Primitive::END) ?
                            prev_waypoint_id + 1 : p.comments[0]->waypoint;

        if (curr_waypoint_id != prev_waypoint_id)
        {
          prev_waypoint_id = curr_waypoint_id;
          file << 1 << "\n";
        }
        else
          file << 0 << "\n";
      }

      Eigen::VectorXd desired_conf = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(prev_p.p.data(), prev_p.p.size());
      Eigen::VectorXd actual_conf = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(q.data(), q.size());

      auto deviation = (actual_conf - desired_conf).norm();

      // RCLCPP_INFO(self::node_->get_logger(), "Trajectory execution, deviation at state %d of %f rad", int(p_index), deviation);

      if (deviation > max_allowed_deviation)
      {
        result->result = false;
        RCLCPP_INFO(node->get_logger(), "Aborting because the robot deviation is greater than %ld rad", (long int)max_allowed_deviation);
        result->result ? goal_handle->succeed(result) : goal_handle->abort(result);
        return;
      }
    }

    if (goal_handle->is_canceling())
    {
      rtde_control->servoStop(std::max(goal->deceleration, 1.0));
      result->result = false;
      RCLCPP_INFO(node->get_logger(), "Trajectory execution canceled");
      goal_handle->canceled(result);
      return;
    }

    if (waypoint_to_check < int(num_trajectory_samples) - 1)
      waypoint_to_check++;
  }

  const TrajectorySample end_p = trajectory_extract_sample(fragments, num_trajectory_samples - 1, p_cache, servo_j_timestep);
  Eigen::VectorXd desired_conf = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(end_p.p.data(), end_p.p.size());

  // if not aborted because of deviation execute the final step
  while (true)
  {
    t_start = rtde_control->initPeriod();
    result->result = rtde_control->servoJ(Array6dToVector(end_p.p), goal->speed, goal->acceleration, servo_j_timestep,
                                          servo_j_lookahead_time, servo_j_gain);
    rtde_control->waitPeriod(t_start);

    check_receive_interface_connection(rtde_receive, node);
    auto q = rtde_receive->getActualQ();

    const TrajectorySample prev_p = trajectory_extract_sample(fragments, waypoint_to_check, prev_p_cache, servo_j_timestep);

    if (save_log)
    {
      for (auto p_j : prev_p.p)
        file << p_j << ",";
      for (auto q_j : q)
        file << q_j << ",";
      file << "0\n";
    }

    Eigen::VectorXd actual_conf = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q.data(), q.size());

    auto deviation = (actual_conf - desired_conf).norm();
    if (deviation < 0.0005)
    {
      result->result = true;
      RCLCPP_INFO(node->get_logger(), "Reached a distance of %f rad from goal, terminating trajectory execution", deviation);
      break;
    }

    if (waypoint_to_check < int(num_trajectory_samples) - 1)
      waypoint_to_check++;
  }

  if (save_log)
    file.close();

  // Make sure that the final step is the goal
  rtde_control->servoStop(std::max(goal->deceleration, 1.0));
  rtde_control->moveJ(Array6dToVector(end_p.p), 0.2, 0.2);
  rtde_control->stopJ();

  RCLCPP_INFO(node->get_logger(),
              (result->result ? "%s succeeded" : "%s failed"), PLUGIN_NAME);

  result->result ? goal_handle->succeed(result) : goal_handle->abort(result);
  // -----------------------------------------
}

class PLUGIN_CLASS_NAME : public ur_ros_rtde_command
{
public:
  void start_action_server(
      rclcpp::Node::SharedPtr node,
      std::shared_ptr<ur_rtde::RTDEControlInterface> rtde_control,
      std::shared_ptr<ur_rtde::RTDEIOInterface> rtde_io,
      std::shared_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive,
      std::shared_ptr<ur_rtde::DashboardClient> dashboard_client) override
  {

    node->declare_parameter<bool>(PLUGIN_NAME ".save_log", false);
    node->declare_parameter<std::string>(PLUGIN_NAME ".log_filepath", "");
    node->declare_parameter<double>(PLUGIN_NAME ".servo_j.lookahead_time", 0.03);
    node->declare_parameter<double>(PLUGIN_NAME ".servo_j.timestep", 0.002);
    node->declare_parameter<int>(PLUGIN_NAME ".servo_j.gain", 2000);
    node->declare_parameter<int>(PLUGIN_NAME ".max_allowed_deviation", 10);

    auto bound_execute_function = std::bind(execute_function_impl, std::placeholders::_1, node, rtde_control, rtde_io, rtde_receive, dashboard_client);
    server_ = std::make_unique<command_server_template<action_type>>(
        node, PLUGIN_NAME, bound_execute_function);
  }

private:
  std::unique_ptr<command_server_template<action_type>> server_;
};

PLUGINLIB_EXPORT_CLASS(PLUGIN_CLASS_NAME, ur_ros_rtde_command)
