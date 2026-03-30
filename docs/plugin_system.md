# 🧩 **Plugin System**

In `ur_ros_rtde`, each available command that can be sent to the robot is implemented as a ROS 2
plugin and exposed as an action.\
This document provides an overview of the plugin system and how it
works.

------------------------------------------------------------------------

## **Plugin Discovery**

When either the `command_server` or `dashboard_server` ROS 2 node is started, all installed ROS 2 packages are scanned to
discover plugins of a specific type.\
The following pseudo-code illustrates how hypothetical `example_type` plugins are dynamically discovered and loaded:

``` cpp
auto packages = ament_index_cpp::get_packages_with_prefixes();
for (const auto &package : packages)
{
    try
    {
        auto class_loader = std::make_unique<pluginlib::ClassLoader<example_type>>(package.first, "example_type");
        class_loaders.push_back(std::move(class_loader));

        auto declared_classes = class_loaders.back()->getDeclaredClasses();
        for (auto dc : declared_classes)
        {
            commands.push_back(class_loaders.back()->createSharedInstance(dc));
            commands.back()->start_action_server(node, args);
        }
    }
    catch (pluginlib::PluginlibException &ex)
    {
        printf("Failed to load plugin. Error: %s\n", ex.what());
    }
}
```

Each invocation of `start_action_server` (implemented by each plugin)
creates and activates an action server.\
These action servers can then be used by ROS 2 client nodes by sending
goals.

The action server structure is generalized using C++ templates, as shown
in
[command_server_template.hpp](../ur_ros_rtde/include/ur_ros_rtde/command_server_template.hpp).\
This approach is shared by both the `command_server` and
`dashboard_server`, ensuring a consistent and reusable design.

------------------------------------------------------------------------

## **Plugin Types**

Three plugin types are available in `ur_ros_rtde`.  
They share a similar structure but differ in their interfaces and intended use.

Depending on the plugin type, different `ur_rtde` interface instances are provided:
- `rtde_control`: allows controlling the robot motion  
- `rtde_io`: allows interaction with the robot I/O interface (digital and analog pins, etc.)  
- `rtde_receive`: allows receiving data (e.g., joint positions, TCP pose, etc.)  
- `dashboard_client`: allows interaction with the teach pendant (e.g., powering on the robot, releasing motor brakes, etc.)

From a code perspective, the three plugin types are defined as follows:

**Command Plugins** ([`ur_ros_rtde_command`](../ur_ros_rtde/include/ur_ros_rtde/command_base_class.hpp), used by `command_server` ROS 2 node)



``` cpp
class ur_ros_rtde_command
{
public:
    virtual void start_action_server(
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<ur_rtde::RTDEControlInterface> rtde_control,
        std::shared_ptr<ur_rtde::RTDEIOInterface> rtde_io,
        std::shared_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive,
        std::shared_ptr<ur_rtde::DashboardClient> dashboard_client) = 0;

    virtual ~ur_ros_rtde_command() {}
};
```

**Extension Plugins** ([`ur_ros_rtde_extension`](../ur_ros_rtde/include/ur_ros_rtde/extension_base_class.hpp), used by `command_server` ROS 2 node)

``` cpp
class ur_ros_rtde_extension
{
public:
    virtual void get_control_script_modifications(control_script_extension &extension) = 0;

    virtual void start_action_server(
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<ur_rtde::RTDEControlInterface> rtde_control,
        std::shared_ptr<ur_rtde::RTDEIOInterface> rtde_io,
        std::shared_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive,
        std::shared_ptr<ur_rtde::DashboardClient> dashboard_client) = 0;

    virtual ~ur_ros_rtde_extension() {}

    int extension_id = COMMAND_ID_OFFSET;
    bool requires_external_urcap = false;
};
```

**Dashboard Command Plugins** ([`ur_ros_rtde_dashboard_command`](../ur_ros_rtde/include/ur_ros_rtde/dashboard_command_base_class.hpp),used by `dashboard_server` ROS 2 node)

``` cpp
class ur_ros_rtde_dashboard_command
{
public:
    virtual void start_action_server(
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<ur_rtde::DashboardClient> dashboard_client) = 0;

    virtual ~ur_ros_rtde_dashboard_command() {}
};
```

------------------------------------------------------------------------

## **Plugin Definition and Example**

In `ur_ros_rtde`, plugins are implemented as independent C++ components.\
Each plugin defines its behavior by implementing the
`start_action_server` method required by the templated action server.

The logic inside the plugin determines what happens when a goal is sent
to the corresponding action server.

Below is a minimal skeleton for implementing a `ur_ros_rtde_command`
plugin:

``` cpp
// ---------- PLUGIN INFO ------------------
#define PLUGIN_NAME /* FILL */
#define PLUGIN_CLASS_NAME /* FILL */
using action_type = /* FILL */
// -----------------------------------------

void execute_function_impl(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<action_type>> goal_handle,
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<ur_rtde::RTDEControlInterface> rtde_control,
    std::shared_ptr<ur_rtde::RTDEIOInterface> rtde_io,
    std::shared_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive,
    std::shared_ptr<ur_rtde::DashboardClient> dashboard_client)
{
    // ---------- PLUGIN BEHAVIOR ----------
    /* FILL */
    // ------------------------------------
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
        auto bound_execute_function = std::bind(
            execute_function_impl,
            std::placeholders::_1,
            node,
            rtde_control,
            rtde_io,
            rtde_receive,
            dashboard_client);

        server_ = std::make_unique<command_server_template<action_type>>(
            node,
            PLUGIN_NAME,
            bound_execute_function);
    }

private:
    std::unique_ptr<command_server_template<action_type>> server_;
};

PLUGINLIB_EXPORT_CLASS(PLUGIN_CLASS_NAME, ur_ros_rtde_command)
```

When implementing a plugin, you must define:

-   `PLUGIN_NAME`: Name of the action server
-   `PLUGIN_CLASS_NAME`: Name used to register the plugin in ROS 2
-   `action_type`: The ROS 2 action type handled by the plugin

A similar approach applies to `ur_ros_rtde_dashboard_command` plugins.\
For `ur_ros_rtde_extension`, the implementation is more advanced and
involves modifying the control script.
Refer to this dedicated [documentation file](extending_the_control_script.md) for
details.

A complete working example showing how to add a new plugin is available
in [tutorials](../ur_ros_rtde_tutorials/).
