#include <ament_index_cpp/get_packages_with_prefixes.hpp>
#include <filesystem>
#include <iostream>
#include <pluginlib/class_loader.hpp>
#include <ur_ros_rtde/command_base_class.hpp>

std::vector<std::shared_ptr<ur_ros_rtde_command>> commands;
std::vector<std::unique_ptr<pluginlib::ClassLoader<ur_ros_rtde_command>>> class_loaders;
std::shared_ptr<ur_rtde::RTDEControlInterface> rtde_control;
std::shared_ptr<ur_rtde::RTDEIOInterface> rtde_io;
std::shared_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive;
std::shared_ptr<ur_rtde::DashboardClient> dashboard_client;

void cleanup()
{
    commands.clear();
    class_loaders.clear();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ur_ros_rtde_command_server_node");
    auto robot_ip = node->declare_parameter<std::string>("robot_ip", "127.0.0.1");

    // change params with hardcoded numbers in each plugin
    internal_params params;
    params.suction_pin = node->declare_parameter<int>("suction_pin", 0);
    params.deposit_pin = node->declare_parameter<int>("deposit_pin", 1);
    params.move_until_force_collision_check_freq = node->declare_parameter<int>("move_until_force_collision_check_freq", 10);
    params.servoJ_gain = node->declare_parameter<int>("servoJ_gain", 200);
    params.servoJ_lookahead_time = node->declare_parameter<double>("servoJ_lookahead_time", 0.2);
    params.servoJ_timestep = node->declare_parameter<double>("servoJ_timestep", 0.002);
    params.receiver_freq = node->declare_parameter<int>("receiver_freq", 100);
    params.deviation_check_freq = node->declare_parameter<int>("deviation_check_freq", 100);
    params.max_deviation = node->declare_parameter<double>("max_deviation", 0.15);
    params.parametrization_timestep = node->declare_parameter<double>("parametrization_timestep", 0.002);
    params.grip_bool_input_register = node->declare_parameter<int>("grip_bool_input_register", 19);
    params.desired_width_input_register = node->declare_parameter<int>("desired_width_input_register", 18);
    params.feedback_width_output_register = node->declare_parameter<int>("feedback_width_output_register", 18);

    //rtde_control = std::make_shared<ur_rtde::RTDEControlInterface>(robot_ip, 500, ur_rtde::RTDEControlInterface::FLAG_USE_EXT_UR_CAP);
    rtde_control = std::make_shared<ur_rtde::RTDEControlInterface>(robot_ip);
    rtde_io = std::make_shared<ur_rtde::RTDEIOInterface>(robot_ip);
    rtde_receive = std::make_shared<ur_rtde::RTDEReceiveInterface>(robot_ip, params.receiver_freq);
    dashboard_client = std::make_shared<ur_rtde::DashboardClient>(robot_ip);

    dashboard_client->connect();

    RCLCPP_INFO(node->get_logger(), "Init done, discovering plugins..");

    auto packages = ament_index_cpp::get_packages_with_prefixes();
    for (const auto &package : packages)
    {
        try
        {
            auto class_loader = std::make_unique<pluginlib::ClassLoader<ur_ros_rtde_command>>(package.first, "ur_ros_rtde_command");
            class_loaders.push_back(std::move(class_loader));

            auto declared_classes = class_loaders.back()->getDeclaredClasses();
            if (declared_classes.size() > 0)
                std::cout << "Found ur_ros_rtde_command definitions in " << package.first << " package:" << std::endl;

            for (auto dc : declared_classes)
            {
                std::cout << "\t" << class_loaders.back()->getClassType(dc) << ": " << class_loaders.back()->getClassDescription(dc) << std::endl;
                commands.push_back(class_loaders.back()->createSharedInstance(dc));
                commands.back()->start_action_server(node, params, rtde_control, rtde_io, rtde_receive, dashboard_client);
            }
        }
        catch (pluginlib::PluginlibException &ex)
        {
            printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
        }
    }

    RCLCPP_INFO(node->get_logger(), "<Action servers ready>");
    rclcpp::on_shutdown(cleanup);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
