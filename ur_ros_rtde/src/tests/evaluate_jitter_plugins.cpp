#include <ur_ros_rtde/utils.hpp>
#include <ur_ros_rtde_simple_clients/simple_action_client_template.hpp>
#include <ur_ros_rtde_msgs/action/move_j.hpp>
#include <ur_ros_rtde_msgs/action/servo_j.hpp>
#include <ur_ros_rtde_msgs/action/servo_stop.hpp>
#include <iomanip> 

void print_stats(std::vector<double> data, const std::string& label)
{
    if (data.size() <= 1) return;

    // Remove first sample (warm-up)
    data.erase(data.begin());

    double sum = std::accumulate(data.begin(), data.end(), 0.0);
    double avg = sum / data.size();

    auto [min_it, max_it] = std::minmax_element(data.begin(), data.end());
    double min_val = *min_it;
    double max_val = *max_it;

    double sq_sum = 0.0;
    for (double v : data)
        sq_sum += (v - avg) * (v - avg);

    double stddev = std::sqrt(sq_sum / data.size());

    std::cout << "\n=== " << label << " Stats ===\n";
    std::cout << "Num. samples: " << data.size() << "\n";

    // Print in ms with microsecond precision
    std::cout << std::fixed << std::setprecision(6);

    std::cout << "Avg: " << avg / 1000.0 << " ms\n";
    std::cout << "Min: " << min_val / 1000.0 << " ms\n";
    std::cout << "Max: " << max_val / 1000.0 << " ms\n";
    std::cout << "Std Dev: " << stddev / 1000.0 << " ms\n";
}

int main(int argc, char **argv)
{
  // INITIALIZATION
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("evaluate_jitter_node");
  auto node_act = rclcpp::Node::make_shared("evaluate_jitter_act_node");
  auto action_client = simple_action_client_template(node_act);
  node->declare_parameter<std::string>("robot_ip", "192.168.0.100");
  std::string robot_ip = node->get_parameter("robot_ip").as_string();
  auto rtde_receive = std::make_shared<ur_rtde::RTDEReceiveInterface>(robot_ip, 500.0);

  size_t num_it = 10001;
  double dt = 1.0/500.0;
  auto actual_conf = rtde_receive->getActualQ();

  using MoveJAction = ur_ros_rtde_msgs::action::MoveJ;
  auto goal_msg_movej = MoveJAction::Goal();
  goal_msg_movej.joint_position = actual_conf;
  goal_msg_movej.speed = 0.1;
  goal_msg_movej.acceleration = 0.1;

  using ServoJAction = ur_ros_rtde_msgs::action::ServoJ;
  auto goal_msg_servoj = ServoJAction::Goal();
  goal_msg_servoj.joint_positions = actual_conf;
  goal_msg_servoj.time = dt;
  goal_msg_servoj.lookahead_time = 0.03;
  goal_msg_servoj.gain = 500;

  using ServoStopAction = ur_ros_rtde_msgs::action::ServoStop;
  auto goal_msg_servostop = ServoStopAction::Goal();

  std::vector<double> movej_times;
  std::vector<double> servoj_times;

  for (size_t i = 0; i < num_it; i++)
  {
    auto start = std::chrono::high_resolution_clock::now();
    action_client.send_goal<MoveJAction>("ur_ros_rtde/move_j_command", goal_msg_movej);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    rclcpp::sleep_for(std::chrono::milliseconds(10));
    movej_times.push_back(duration.count());
  }

  for (size_t i = 0; i < num_it; i++)
  {
    auto start = std::chrono::high_resolution_clock::now();
    action_client.send_goal<ServoJAction>("ur_ros_rtde/servo_j_command", goal_msg_servoj);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    rclcpp::sleep_for(std::chrono::milliseconds(10));
    servoj_times.push_back(duration.count());
  }

  action_client.send_goal<ServoStopAction>("ur_ros_rtde/servo_stop_command", goal_msg_servostop);

  std::cout << "ROS 2 plugins stats" << std::endl;
  print_stats(movej_times, "MoveJ");
  print_stats(servoj_times, "ServoJ");

  rclcpp::shutdown();
  return 0;
}
