#include <ur_ros_rtde/command_base_class.hpp>
#include <ur_ros_rtde_msgs/action/execute_trajectory.hpp>
#include <pluginlib/class_list_macros.hpp>

using namespace std::chrono_literals;
using namespace std::chrono;

std::vector<double> joint_speed_limits = {2.09, 2.09, M_PI, M_PI, M_PI, M_PI};

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
      double clamped_v = std::min(v, joint_speed_limits[j]);
      if (diff * old_diff[j] >= 0)
        new_v = std::min(clamped_v, sqrt(2 * a * abs(diff) + pow(old_vel[j], 2)));
      else
        new_v = std::min(clamped_v, sqrt(a * abs(diff)));

      // if(j == 0) std::cout << (diff * old_diff[j] >= 0 ? "up" : "reset") << ", old: " << old_vel[j] << ", diff: " << diff <<  ", new: " << new_v << std::endl;

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
      double clamped_v = std::min(v, joint_speed_limits[j]);
      if (diff * old_diff[j] >= 0)
        new_v = std::min(clamped_v, sqrt(2 * a * abs(diff) + pow(old_vel[j], 2)));
      else
        new_v = std::min(clamped_v, sqrt(a * abs(diff)));

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

std::vector<std::vector<double>> parametrize_traj(const double &dt, const double &acceleration, const double &speed,
                                                  const std::vector<ur_ros_rtde_msgs::msg::Vector> &trajectory)
{
  std::cout << "Parametrizing trajectory with " << trajectory.size() << " waypoints, max speed: " << speed << ", max acceleration: " << acceleration << std::endl;

  std::vector<double> times = get_times(trajectory, acceleration, speed);
  std::vector<std::vector<double>> param_traj;

  for (size_t i = 1; i < trajectory.size(); i++)
  {

    if (times[i] > dt)
    {
      auto curr_q = trajectory[i].vector;
      auto prev_q = trajectory[i - 1].vector;

      double num_interpolation = times[i] / dt;

      //std::cout << "waypoint " << i << " must be interpolated " << num_interpolation << " times (time diff: " << times[i] << " s)" << std::endl;

      double interpolation_step = 1.0 / num_interpolation;
      for (int k = 0; k < num_interpolation; k++)
      {
        std::vector<double> interpolated_q;
        for (size_t j = 0; j < curr_q.size(); j++)
          interpolated_q.push_back((1.0 - k * interpolation_step) * prev_q[j] + k * interpolation_step * curr_q[j]);
        param_traj.push_back(interpolated_q);
      }
    }
  }

  param_traj.push_back(trajectory.back().vector);

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

  std::vector<ur_ros_rtde_msgs::msg::Vector> trajectory;

  if (goal->trajectory.size() <= 1)
  {
    result->result = true;
    RCLCPP_INFO(self::node_->get_logger(), "Terminating execution, the passed trajectory has only %ld waypoints", goal->trajectory.size());
    return;
  }
  else
  {

    for (size_t i = 1; i < goal->trajectory.size(); i++)
    {
      auto conf1 = goal->trajectory[i - 1].vector;
      auto conf2 = goal->trajectory[i].vector;

      double total_dist = 0.0;
      for (size_t j = 0; j < conf1.size(); ++j)
        total_dist += std::pow(conf2[j] - conf1[j], 2);
      total_dist = std::sqrt(total_dist);

      size_t num_waypoints = std::ceil(total_dist / (0.02));

      for (size_t j = 0; j < num_waypoints; ++j)
      {
        ur_ros_rtde_msgs::msg::Vector waypoint;
        double t = static_cast<double>(j) / num_waypoints;

        for (size_t k = 0; k < conf1.size(); ++k)
        {
          waypoint.vector.push_back((1 - t) * conf1[k] + t * conf2[k]);
        }

        trajectory.push_back(waypoint);
      }
    }

    trajectory.push_back(goal->trajectory.back());

    RCLCPP_INFO(self::node_->get_logger(), "Trajectory waypoints interpolated, old trajectory size is %ld, new trajectory size is %ld", goal->trajectory.size(), trajectory.size());
  }

  auto p_trajectory = parametrize_traj(params_.parametrization_timestep, goal->acceleration, goal->speed, trajectory);

  RCLCPP_INFO(self::node_->get_logger(), "Parametrized trajectory was divided into %ld waypoints, starting execution", p_trajectory.size());

  int sleep_dt = params_.servoJ_timestep * 1000;

  rtde_control_->moveJ(p_trajectory[0], 0.2, 0.2);
  rtde_control_->stopJ();
  steady_clock::time_point t_start;

  double time = 0.0;
  for(size_t p_index = 0; p_index < p_trajectory.size(); p_index++)
  {
    auto p = p_trajectory[p_index];
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

      std::cout << std::endl;
      std::cout << actual_conf << std::endl;
      std::cout << desired_conf << std::endl;

      auto deviation = (actual_conf - desired_conf).norm();

      RCLCPP_INFO(self::node_->get_logger(), "Trajectory execution, deviation at state %ld of %f rad", p_index, deviation);

      if (deviation > params_.max_deviation)
      {
        result->result = false;
        rtde_control_->servoStop(std::max(goal->deceleration, 1.0));
        RCLCPP_INFO(self::node_->get_logger(), "Aborting because the robot deviation is greater than %f rad", params_.max_deviation);
        result->result ? goal_handle->succeed(result) : goal_handle->abort(result);
        return;
      }
    }

    if (goal_handle->is_canceling())
    {
      rtde_control_->servoStop(std::max(goal->deceleration, 1.0));
      result->result = false;
      RCLCPP_INFO(self::node_->get_logger(), "Trajectory execution canceled");
      goal_handle->canceled(result);
      return;
    }
  }

  Eigen::VectorXd desired_conf = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(p_trajectory[p_trajectory.size() - 1].data(), p_trajectory[p_trajectory.size() - 1].size());

  // if not aborted because of deviation execute the final step
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

  // Make sure that the final step is the goal
  rtde_control_->servoStop(std::max(goal->deceleration, 1.0));
  rtde_control_->moveJ(p_trajectory[p_trajectory.size() - 1], 0.2, 0.2);
  rtde_control_->stopJ();

  RCLCPP_INFO(self::node_->get_logger(),
              (result->result ? "%s succeeded" : "%s failed"), action_name_.c_str());

  result->result ? goal_handle->succeed(result) : goal_handle->abort(result);
};

class ExecuteTrajectory : public ur_ros_rtde_command
{
public:
  void start_action_server(rclcpp::Node::SharedPtr node,
                           const internal_params &params,
                           std::shared_ptr<ur_rtde::RTDEControlInterface> rtde_control,
                           std::shared_ptr<ur_rtde::RTDEIOInterface> rtde_io,
                           std::shared_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive,
                           std::shared_ptr<ur_rtde::DashboardClient> dashboard_client) override
  {
    server_ = std::make_unique<command_server_template<ur_ros_rtde_msgs::action::ExecuteTrajectory>>(
        node, "execute_trajectory_command", params, rtde_control, rtde_io, rtde_receive, dashboard_client);
  };

private:
  std::unique_ptr<command_server_template<ur_ros_rtde_msgs::action::ExecuteTrajectory>> server_;
};

PLUGINLIB_EXPORT_CLASS(ExecuteTrajectory, ur_ros_rtde_command)
