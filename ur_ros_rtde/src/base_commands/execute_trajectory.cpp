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

inline std::shared_ptr<action_type::Feedback> create_feedback(const uint64_t reached_waypoint)
{
  auto feedback = std::make_shared<action_type::Feedback>();
  feedback->reached_waypoint = reached_waypoint;
  return feedback;
}

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

  auto log_filepath = node->get_parameter(PLUGIN_NAME ".log_filepath").as_string();
  auto servo_j_lookahead_time = node->get_parameter(PLUGIN_NAME ".servo_j.lookahead_time").as_double();
  auto servo_j_timestep = node->get_parameter(PLUGIN_NAME ".servo_j.timestep").as_double();
  auto servo_j_gain = node->get_parameter(PLUGIN_NAME ".servo_j.gain").as_int();
  auto max_allowed_deviation = node->get_parameter(PLUGIN_NAME ".max_allowed_deviation").as_double();
  const double STOP_THRESHOLD = node->get_parameter(PLUGIN_NAME ".stop_threshold").as_double();

  RCLCPP_INFO(node->get_logger(), "Received traj. execution request");

  /* init */
  check_control_interface_connection(rtde_control, node);
  check_receive_interface_connection(rtde_receive, node);

  /* check trajectory size */
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<action_type::Result>();
  if (goal->trajectory.size() <= 1)
  {
    RCLCPP_INFO(node->get_logger(), "Terminating execution, the passed trajectory has only %ld waypoints", goal->trajectory.size());
    result->result = false;
    goal_handle->abort(result);
    return;
  }

  /* get joint speed limits */
  Array6d joint_speed_limits;
  std::copy(goal->joint_speed_limits.begin(), goal->joint_speed_limits.end(), joint_speed_limits.begin());

  /* parametrize trajectory */
  std::vector<double, std::allocator<double>> times;
  std::vector<Array6d> velocities;
  uint64_t iterations;
  std::vector<TrajectorySample> samples;
  bool parametrization_success = trajectory_time_planning(msg_to_Array6dVector(goal->trajectory), goal->acceleration, goal->speed, goal->speed_percentage, joint_speed_limits, servo_j_timestep, times, velocities, iterations, samples);
  if (!parametrization_success)
  {
    result->result = false;
    goal_handle->abort(result);
    return;
  }
  const int num_trajectory_samples = samples.size();
  const int last_sample_idx = num_trajectory_samples - 1;

  /* check if parametrized trajectory has valid samples */
  if (num_trajectory_samples == 0)
  {
    RCLCPP_ERROR(node->get_logger(), "Terminating execution, trajectory parameterization has no points!");
    result->result = false;
    goal_handle->abort(result);
    return;
  }

  /* check if parametrized trajectory can be executed in a reasonable time */
  const double expected_trajectory_duration = num_trajectory_samples * servo_j_timestep;
  RCLCPP_INFO(node->get_logger(), "Expected trajectory duration is %f seconds (%d samples).", float(expected_trajectory_duration), int(num_trajectory_samples));
  if (expected_trajectory_duration > 24.0f * 3600.0f)
  {
    RCLCPP_ERROR(node->get_logger(), "Terminating execution, trajectory duration is longer than a day! (sanity check)");
    result->result = false;
    goal_handle->abort(result);
    return;
  }
  
  auto start_time = node->now();

  /* move robot at the start of the trajectory */
  const TrajectorySample start_p = samples[0];
  rtde_control->moveJ(Array6dToVector(start_p.p), 0.2, 0.2);
  rtde_control->stopJ();

  goal_handle->publish_feedback(create_feedback(0));
  uint64_t feedback_prev_waypoint_id = 1;

  /* init logging variables */
  std::ofstream file;
  bool save_log = false;
  if (log_filepath != "")
  {
    file = std::ofstream(log_filepath);
    save_log = true;
  }

  /* macro for computing a trajectory waypoint based on lookahead_time */
  const int lookahead_delay = int((servo_j_lookahead_time) / (servo_j_timestep));
#define COMPUTE_WAYPOINT_TO_CHECK(x) (x - lookahead_delay)

  /* check which samples can be used for slow down trajectory execution in the past N samples based on lookahead */
  std::vector<bool> samples_ignore_slowdown(num_trajectory_samples, false);

  for (size_t idx = lookahead_delay + 1; idx < size_t(num_trajectory_samples); idx++)
  {
    bool any_positive[NUM_JOINTS] = {false};
    bool any_negative[NUM_JOINTS] = {false};
    // if at least one of the previous samples is positive and at least one is negative
    // there is a velocity inversion
    for (size_t prev_idx = idx; prev_idx >= (idx - lookahead_delay); prev_idx--)
    {
      const TrajectorySample prev_p = samples[prev_idx];
      for (size_t jj = 0; jj < NUM_JOINTS; jj++)
      {
        if (prev_p.v[jj] < 0.0)
          any_negative[jj] = true;
        if (prev_p.v[jj] > 0.0)
          any_positive[jj] = true;
      }
    }

    bool is_change_direction = false;
    for (size_t jj = 0; jj < NUM_JOINTS; jj++)
    {
      if (any_positive[jj] && any_negative[jj])
        is_change_direction = true;
    }

    if (is_change_direction)
      samples_ignore_slowdown[idx - lookahead_delay] = true;
  }

  /* MAIN CONTROL LOOP */
  steady_clock::time_point t_start, t_end;
  int sample_idx = -1;
  int delayed_waypoint_idx = -1;
  const TrajectorySample end_sample = samples.back();
  int control_loop_repetitions = 0;
  int control_loop_iterations = 0;

  /* check if RTDE frequency is appropirate for the requested cycle time */
  t_start = rtde_control->initPeriod();
  rtde_control->waitPeriod(t_start);
  t_end = std::chrono::steady_clock::now();
  const double rtde_frequency = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count() / 1000.0;
  if (rtde_frequency > servo_j_timestep)
  {
    RCLCPP_INFO(node->get_logger(), "Requested loop cycle time (%f s) is lower than RTDE frequency (%f s), aborting trajectory execution", servo_j_timestep, rtde_frequency);
    result->result = false;
    goal_handle->abort(result);
    return;
  }

  while (true)
  {
    control_loop_iterations++;
    auto start_cycle_time = node->now();

    /* step to the next sample */
    sample_idx = std::min(sample_idx + 1, num_trajectory_samples - 1);
    delayed_waypoint_idx = sample_idx == num_trajectory_samples - 1 ? std::min(delayed_waypoint_idx + 1, sample_idx) :
                           COMPUTE_WAYPOINT_TO_CHECK(sample_idx);
    //const double lookahead_time = (sample_idx - delayed_waypoint_idx - 1) * servo_j_timestep;       // what is this?
    const double lookahead_time = servo_j_lookahead_time;

    const TrajectorySample p = samples[sample_idx];

    /* request ServoJ command */
    {
      t_start = rtde_control->initPeriod();

      result->result = rtde_control->servoJ(Array6dToVector(p.p), goal->speed, goal->acceleration,
                                            servo_j_timestep, std::max(0.03, lookahead_time), servo_j_gain);

      if (!result->result)
      {
        RCLCPP_INFO(node->get_logger(), "Terminating execution, error in sending ServoJ command");
        result->result = false;
        goal_handle->abort(result);
        return;
      }

      double elapsed_time = 0.0;

      /* wait until at least <servo_j_timestep> seconds are passed */
      while (true)
      {
        rtde_control->waitPeriod(t_start);
        t_end = std::chrono::steady_clock::now();
        elapsed_time += std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count() / 1000.0;
        if ((double)elapsed_time >= servo_j_timestep)
          break;
        t_start = rtde_control->initPeriod();
      }
    }

    /* check if goal canceling has been requested */
    if (goal_handle->is_canceling())
    {
      rtde_control->servoStop(std::max(goal->deceleration, 1.0));
      result->result = false;
      RCLCPP_INFO(node->get_logger(), "Trajectory execution canceled");
      goal_handle->canceled(result);
      return;
    }

    /* check if enough time passed to perform execution validation */
    if (COMPUTE_WAYPOINT_TO_CHECK(sample_idx) < 1)
      continue;

    /* get where the robot should be and where it would go next */
    const TrajectorySample prev_prev_p = samples[delayed_waypoint_idx - 1];
    const TrajectorySample prev_p = samples[delayed_waypoint_idx];

    /* get where the robot is and the joint velocities */
    check_receive_interface_connection(rtde_receive, node);
    auto q = rtde_receive->getActualQ();
    auto v = rtde_receive->getActualQd();

    /* save log, if requested */
    if (save_log)
    {
      for (auto p_j : prev_p.p)
        file << p_j << ",";
      for (auto q_j : q)
        file << q_j << ",";
      for (auto v_desired_j : prev_p.v)
        file << v_desired_j << ",";
      for (auto v_j : v)
        file << v_j << ",";
      file << p.comments[0]->waypoint << "\n";
    }

    /* publish feedback */
    const uint64_t feedback_waypoint_id = prev_p.comments[0]->waypoint;
    while (feedback_prev_waypoint_id < feedback_waypoint_id)
    {
      goal_handle->publish_feedback(create_feedback(feedback_prev_waypoint_id));
      feedback_prev_waypoint_id++;
    }

    /* check if robot is deviating too much from trajectory (distance between actual robot state and where it should be)*/
    Eigen::VectorXd desired_conf = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(prev_p.p.data(), prev_p.p.size());
    Eigen::VectorXd actual_conf = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(q.data(), q.size());
    Eigen::VectorXd actual_v = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(v.data(), v.size());
    Eigen::VectorXd expected_v = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(prev_p.v.data(), prev_p.v.size());
    auto deviation_old_waypoint = (actual_conf - desired_conf).norm();
    
    if (deviation_old_waypoint > max_allowed_deviation)
    {
      result->result = false;
      RCLCPP_INFO(node->get_logger(), "Aborting because the deviation from trajectory is greater than %f rad: %f rad",
                  max_allowed_deviation, deviation_old_waypoint);
      result->result ? goal_handle->succeed(result) : goal_handle->abort(result);
      return;
    }

    /* check if robot has reached the end of trajectory */
    Eigen::VectorXd end_conf = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(end_sample.p.data(), end_sample.p.size());
    double distance_to_goal = (actual_conf - end_conf).norm();
    const double actual_speed = actual_v.norm();
    const double expected_speed = expected_v.norm();
    
    if (sample_idx == last_sample_idx && delayed_waypoint_idx == last_sample_idx && distance_to_goal < STOP_THRESHOLD)
    {
      RCLCPP_INFO(node->get_logger(), "Reached a distance of %f rad from goal, stopping robot.", float(distance_to_goal));
      rtde_control->servoStop(std::max(goal->deceleration, 1.0));
      auto last_q = rtde_receive->getActualQ();
      Eigen::VectorXd last_actual_conf = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(last_q.data(), last_q.size());
      double last_distance_to_goal = (last_actual_conf - end_conf).norm();
      RCLCPP_INFO(node->get_logger(), "Robot stopped at distance of %f rad from goal.", float(last_distance_to_goal));

      if (last_distance_to_goal > STOP_THRESHOLD)
      {
        RCLCPP_WARN(node->get_logger(), "Robot stopped too far from goal, moving to goal.");
        rtde_control->moveJ(Array6dToVector(end_sample.p), 0.2,0.2);
        rtde_control->stopJ();
      }
      result->result = true;
      RCLCPP_INFO(node->get_logger(), "Terminating trajectory execution");
      break;
    }

    if (sample_idx == last_sample_idx && delayed_waypoint_idx == last_sample_idx)
    {
      RCLCPP_WARN(node->get_logger(), "Robot is delayed and did not reach last sample yet. Waiting...");
    }

    /* check if robot has to spent more time on the previous waypoint, the check is performed only for targets that are not the same */
    Eigen::VectorXd prev_prev_conf = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(prev_prev_p.p.data(), prev_prev_p.p.size());
    auto dist_prev_prev = (actual_conf - prev_prev_conf).norm();

    if (!samples_ignore_slowdown[delayed_waypoint_idx - 1] && dist_prev_prev < deviation_old_waypoint && dist_prev_prev > 5.0E-2)
    {
      RCLCPP_INFO(node->get_logger(), "Sample %d robot is too far from the next goal (%f ~ %f rad, %f ~ %f rad/s), sending again this sample",
                  delayed_waypoint_idx, dist_prev_prev, deviation_old_waypoint, float(actual_speed), float(expected_speed));
      sample_idx--;
      delayed_waypoint_idx--;
      control_loop_repetitions++;
    }
    else{
      //if(sample_idx == num_trajectory_samples - 2) std::cout << deviation_old_waypoint << std::endl;
    }

    auto cycle_time = (node->now() - start_cycle_time).seconds();
    if (cycle_time > 2 * servo_j_timestep)
      RCLCPP_WARN(node->get_logger(), "Control loop cycle time is twice as desired (%f s instead of %f s), try increasing <servo_j_timestep> ros param", cycle_time, servo_j_timestep);
  }
  /* close log file */
  if (save_log)
    file.close();

  RCLCPP_INFO(node->get_logger(), "Control loop iterations: %d", control_loop_iterations);
  RCLCPP_INFO(node->get_logger(), "Control loop repetitions: %d", control_loop_repetitions);
  RCLCPP_INFO(node->get_logger(), "Expected Time: %.9f s", float(expected_trajectory_duration));
  RCLCPP_INFO(node->get_logger(), "Elapsed Time: %.9f s", (node->now() - start_time).seconds());
  RCLCPP_INFO(node->get_logger(), (result->result ? "%s succeeded" : "%s failed"), PLUGIN_NAME);
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
    node->declare_parameter<std::string>(PLUGIN_NAME ".log_filepath", "");
    node->declare_parameter<double>(PLUGIN_NAME ".servo_j.lookahead_time", 0.03);
    node->declare_parameter<double>(PLUGIN_NAME ".servo_j.timestep", 0.002);
    node->declare_parameter<int>(PLUGIN_NAME ".servo_j.gain", 500);
    node->declare_parameter<double>(PLUGIN_NAME ".max_allowed_deviation", 0.7);
    node->declare_parameter<double>(PLUGIN_NAME ".stop_threshold", 5.0E-3);

    auto bound_execute_function = std::bind(execute_function_impl, std::placeholders::_1, node, rtde_control, rtde_io, rtde_receive, dashboard_client);
    server_ = std::make_unique<command_server_template<action_type>>(
        node, PLUGIN_NAME, bound_execute_function);
  }

private:
  std::unique_ptr<command_server_template<action_type>> server_;
};

PLUGINLIB_EXPORT_CLASS(PLUGIN_CLASS_NAME, ur_ros_rtde_command)
