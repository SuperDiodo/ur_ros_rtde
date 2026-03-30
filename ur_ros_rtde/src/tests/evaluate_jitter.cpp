#include <ur_ros_rtde/utils.hpp>
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
  node->declare_parameter<std::string>("robot_ip", "192.168.0.100");
  std::string robot_ip = node->get_parameter("robot_ip").as_string();
  auto rtde_control = std::make_shared<ur_rtde::RTDEControlInterface>(robot_ip, 500.0);
  auto rtde_receive = std::make_shared<ur_rtde::RTDEReceiveInterface>(robot_ip, 500.0);

  size_t num_it = 10001;
  double dt = 1.0/500.0;
  auto actual_conf = rtde_receive->getActualQ();

  std::vector<double> movej_times;
  std::vector<double> servoj_times;

  for (size_t i = 0; i < num_it; i++)
  {
    auto start = std::chrono::high_resolution_clock::now();
    rtde_control->moveJ(actual_conf, 0.1, 0.1);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    rclcpp::sleep_for(std::chrono::milliseconds(10));
    movej_times.push_back(duration.count());
  }

  for (size_t i = 0; i < num_it; i++)
  {
    auto start = std::chrono::high_resolution_clock::now();
    rtde_control->servoJ(actual_conf, 0.0, 0.0, dt, 0.03, 500);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    rclcpp::sleep_for(std::chrono::milliseconds(10));
    servoj_times.push_back(duration.count());
  }

  rtde_control->servoStop();

  std::cout << "ur_rtde stats" << std::endl;
  print_stats(movej_times, "MoveJ");
  print_stats(servoj_times, "ServoJ");

  rclcpp::shutdown();
  return 0;
}
