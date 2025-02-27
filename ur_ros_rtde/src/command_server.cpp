#include <ament_index_cpp/get_packages_with_prefixes.hpp>
#include <filesystem>
#include <iostream>
#include <pluginlib/class_loader.hpp>
#include <ur_ros_rtde/command_base_class.hpp>
#include <ur_ros_rtde/extension_base_class.hpp>
#include "control_script_str.h"

std::vector<std::shared_ptr<ur_ros_rtde_command>> commands;
std::vector<std::unique_ptr<pluginlib::ClassLoader<ur_ros_rtde_command>>> command_loaders;

std::vector<std::shared_ptr<ur_ros_rtde_extension>> extensions;
std::vector<std::unique_ptr<pluginlib::ClassLoader<ur_ros_rtde_extension>>> extension_loaders;

std::shared_ptr<ur_rtde::RTDEControlInterface> rtde_control;
std::shared_ptr<ur_rtde::RTDEIOInterface> rtde_io;
std::shared_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive;
std::shared_ptr<ur_rtde::DashboardClient> dashboard_client;

std::string temp_custom_control_script_filename;

void cleanup()
{
    commands.clear();
    command_loaders.clear();
    extensions.clear();
    extension_loaders.clear();

    std::filesystem::path temp_custom_control_script_file(temp_custom_control_script_filename); // Create a path object
    if (std::filesystem::exists(temp_custom_control_script_file))
    {
        std::filesystem::remove(temp_custom_control_script_file);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("command_server_node");
    auto robot_ip = node->declare_parameter<std::string>("robot_ip", "127.0.0.1");
    auto rtde_frequency = node->declare_parameter<double>("command_server"
                                                          ".rtde_frequency",
                                                          500.0);
    auto load_custom_control_script = node->declare_parameter<bool>("command_server"
                                                                    ".load_custom_control_script",
                                                                    true);

    auto plugins_blacklist = node->declare_parameter<std::vector<std::string>>("command_server"
                                                                                 ".plugins_blacklist",
                                                                                 std::vector<std::string>());

    std::unordered_set<std::string> plugins_blacklist_set;
    for (const auto& plugin_blacklisted : plugins_blacklist) {
        plugins_blacklist_set.insert(plugin_blacklisted);
    }

#ifndef UR_RTDE_LOAD_CUSTOM_CONTROL_SCRIPT_PATCH
    if (load_custom_control_script)
        RCLCPP_WARN(node->get_logger(), "The loading of custom control script is requested but the patch to UR_RTDE was not applied. Please apply the patch (see documentation).");
    load_custom_control_script = false;
#endif

    temp_custom_control_script_filename = "";

    rtde_receive = std::make_shared<ur_rtde::RTDEReceiveInterface>(robot_ip, rtde_frequency);
    rtde_io = std::make_shared<ur_rtde::RTDEIOInterface>(robot_ip);
    dashboard_client = std::make_shared<ur_rtde::DashboardClient>(robot_ip);
    dashboard_client->connect();
    RCLCPP_INFO(node->get_logger(), "Init done, discovering plugins..");

    auto packages = ament_index_cpp::get_packages_with_prefixes();
    for (const auto &package : packages)
    {
        try
        {
            if (load_custom_control_script)
            {
                auto extension_loader = std::make_unique<pluginlib::ClassLoader<ur_ros_rtde_extension>>(package.first, "ur_ros_rtde_extension");
                if (extension_loader->getDeclaredClasses().size() > 0)
                {
                    std::cout << "Found ur_ros_rtde_extension definitions in " << package.first << " package" << std::endl;
                    extension_loaders.push_back(std::move(extension_loader));
                }
            }

            auto command_loader = std::make_unique<pluginlib::ClassLoader<ur_ros_rtde_command>>(package.first, "ur_ros_rtde_command");
            if (command_loader->getDeclaredClasses().size() > 0)
            {
                std::cout << "Found ur_ros_rtde_command definitions in " << package.first << " package" << std::endl;
                command_loaders.push_back(std::move(command_loader));
            }
        }
        catch (pluginlib::PluginlibException &ex)
        {
            printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
        }
    }

    /* adapt control script with extension */
    if (extension_loaders.size() > 0)
    {
        // clear alive command requests
        rtde_io->setInputDoubleRegister(COMMAND_REQUEST_REGISTER, EXT_CMD_IDLE);
        std::cout << "\nur_ros_rtde_extensions found:" << std::endl;
        std::string control_script;
        control_script = CUSTOM_CONTROL_SCRIPT_STR;
        add_control_register_interaction(control_script);
        int extension_id = 0;
        std::unordered_set<std::string> loaded_preables;
        for (auto &ext : extension_loaders)
        {
            for (auto dc : ext->getDeclaredClasses())
            {

                if(!plugins_blacklist_set.insert(ext->getClassType(dc)).second){
                    std::cout << "\t" << ext->getClassType(dc) << ": " << ext->getClassDescription(dc) << "( skipped, blacklist )" << std::endl;
                    continue;
                }

                std::cout << "\t" << ext->getClassType(dc) << ": " << ext->getClassDescription(dc) << std::endl;
                auto extension_instance = ext->createSharedInstance(dc);
                extension_instance->extension_id += ++extension_id;
                control_script_extension str_extension;
                extension_instance->get_control_script_modifications(str_extension);
                if(loaded_preables.insert(str_extension.get_preamble().name).second) add_control_script_preamble(control_script, str_extension.get_preamble());
                apply_to_script(control_script, extension_instance->extension_id, str_extension);
                extensions.push_back(extension_instance);
            }
        }

        temp_custom_control_script_filename = (std::filesystem::temp_directory_path() / "temp_custom_control_script_XXXXXX").string();
        int fd = mkstemp(&temp_custom_control_script_filename[0]);
        if (fd == -1)
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to create custom control script temporary file");
            rclcpp::on_shutdown(cleanup);
            return 0;
        }
        write_control_script(temp_custom_control_script_filename, control_script);
    }

    if (extension_loaders.size() > 0)
        rtde_control = std::make_shared<ur_rtde::RTDEControlInterface>(robot_ip, rtde_frequency, ur_rtde::RTDEControlInterface::FLAG_UPLOAD_SCRIPT, 50002, RT_PRIORITY_UNDEFINED, temp_custom_control_script_filename);
    else
        rtde_control = std::make_shared<ur_rtde::RTDEControlInterface>(robot_ip, rtde_frequency);

    /* activate commands */
    if (command_loaders.size() > 0)
        std::cout << "\nur_ros_rtde_commands found:" << std::endl;
    for (auto &c : command_loaders)
    {
        for (auto dc : c->getDeclaredClasses())
        {

            if(!plugins_blacklist_set.insert(c->getClassType(dc)).second){
                std::cout << "\t" << c->getClassType(dc) << ": " << c->getClassDescription(dc) << " | skipped, blacklisted" << std::endl;
                continue;
            }

            std::cout << "\t" << c->getClassType(dc) << ": " << c->getClassDescription(dc) << std::endl;
            auto shared_instance = c->createSharedInstance(dc);
            shared_instance->start_action_server(node, rtde_control, rtde_io, rtde_receive, dashboard_client);
            commands.push_back(shared_instance);
        }
    }

    /* activate extensions */
    if (extensions.size() > 0){
        RCLCPP_INFO(node->get_logger(), "Temporary custom control script file written in %s", temp_custom_control_script_filename.c_str());
        for (auto ext : extensions)
            ext->start_action_server(node, rtde_control, rtde_io, rtde_receive, dashboard_client);
    }

    RCLCPP_INFO(node->get_logger(), "<Action servers ready>");
    rclcpp::on_shutdown(cleanup);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}