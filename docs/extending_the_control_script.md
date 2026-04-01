# ➕ **Extend the `ur_rtde` control script via `ur_ros_rtde_extension` plugins**

Extension-type plugins are used to extend the robot control script, allowing custom URcap instructions to be executed via `ur_ros_rtde`.

## Changing the `ur_ros_rtde` control script

The main difference of this plugin type compared to the others is that, in addition to `start_action_server`, the method `get_control_script_modifications` must also be implemented.

This method is responsible for generating modifications to the control script dynamically.  
To support this, the utility header file [`control_script_utils.hpp`](../ur_ros_rtde/include/ur_ros_rtde/control_script_utils.hpp) is provided, simplifying the extension of the URScript control logic.

The header defines constants for interacting with RTDE registers and provides helper functions to:
- load and write control scripts,
- inject register-based communication into the control loop,
- integrate user-defined command extensions.

During plugin loading, the `command_server` adapts the control script as shown in the following example:

```cpp
std::string control_script;
control_script = CUSTOM_CONTROL_SCRIPT_STR;
add_control_register_interaction(control_script);

int extension_id = 0;
std::unordered_set<std::string> loaded_preambles;

// iterate over all the extension plugins found
for (auto &ext : extension_loaders)
{
    for (auto dc : ext->getDeclaredClasses())
    {
        auto extension_instance = ext->createSharedInstance(dc);
        extension_instance->extension_id += ++extension_id;

        // get extension definition
        control_script_extension str_extension;
        extension_instance->get_control_script_modifications(str_extension);
        
        // load preamble only once
        if (loaded_preambles.insert(str_extension.get_preamble().name).second)
            add_control_script_preamble(control_script, str_extension.get_preamble());

        // apply control script modification modifications
        apply_to_script(control_script, extension_instance->extension_id, str_extension);
    }
}

temp_custom_control_script_filename =
    (std::filesystem::temp_directory_path() / "temp_custom_control_script_XXXXXX").string();

int fd = mkstemp(&temp_custom_control_script_filename[0]);
write_control_script(temp_custom_control_script_filename, control_script);
```
Basically, the original control script (stored in the macro `CUSTOM_CONTROL_SCRIPT_STR`) is modified by each plugin and then sent to the robot.

It is important to highlight the following:

- With **`add_control_register_interaction`**, the `command_server` ROS 2 node injects the logic required for RTDE-based communication into the control script.  
The table below describes which registers are defined to interact with the control script from plugins.  
All registers can be reused for multiple purposes, for example to pass arguments to URCap methods or to retrieve return values.  
The only two exceptions are:
    - **Input integer register 18**, used to request the execution of a custom command added by a plugin  
    - **Output integer register 18**, used to retrieve the status of the control script (idle, busy, previous command done)

    | Type   | Direction | Address | Name                     | Description          |
    |--------|-----------|---------|--------------------------|----------------------|
    | DOUBLE    | Input     | 18      | COMMAND_REQUEST_REGISTER | Command request      |
    | DOUBLE    | Output    | 18      | COMMAND_STATUS_REGISTER  | Command status       |
    | INT    | Input     | 18      | INPUT_INTEGER_REG_1      | Integer input 1      |
    | INT    | Input     | 19      | INPUT_INTEGER_REG_2      | Integer input 2      |
    | INT    | Input     | 20      | INPUT_INTEGER_REG_3      | Integer input 3      |
    | INT    | Input     | 21      | INPUT_INTEGER_REG_4      | Integer input 4      |
    | INT    | Input     | 22      | INPUT_INTEGER_REG_5      | Integer input 5      |
    | DOUBLE | Input     | 19      | INPUT_DOUBLE_REG_1       | Double input 1       |
    | DOUBLE | Input     | 20      | INPUT_DOUBLE_REG_2       | Double input 2       |
    | DOUBLE | Input     | 21      | INPUT_DOUBLE_REG_3       | Double input 3       |
    | DOUBLE | Input     | 22      | INPUT_DOUBLE_REG_4       | Double input 4       |
    | INT    | Output    | 18      | OUTPUT_INTEGER_REG_1     | Integer output 1     |
    | INT    | Output    | 19      | OUTPUT_INTEGER_REG_2     | Integer output 2     |
    | INT    | Output    | 20      | OUTPUT_INTEGER_REG_3     | Integer output 3     |
    | INT    | Output    | 21      | OUTPUT_INTEGER_REG_4     | Integer output 4     |
    | INT    | Output    | 22      | OUTPUT_INTEGER_REG_5     | Integer output 5     |
    | DOUBLE | Output    | 19      | OUTPUT_DOUBLE_REG_1      | Double output 1      |
    | DOUBLE | Output    | 20      | OUTPUT_DOUBLE_REG_2      | Double output 2      |
    | DOUBLE | Output    | 21      | OUTPUT_DOUBLE_REG_3      | Double output 3      |
    | DOUBLE | Output    | 22      | OUTPUT_DOUBLE_REG_4      | Double output 4      |

- **`add_control_script_preamble`** adds URCap plugin-specific modifications required to execute URCap commands.
- **`apply_to_script`** modifies the control script based on the changes defined in the plugin, obtained with `get_control_script_modifications`. A practical example is discussed in the next section.
- The modified control script is written to a temporary file. This generated script is then used by `ur_rtde` in place of the original one, allowing all plugin-specific extensions to be executed at runtime.

## Example: use an OnRobot Soft Gripper via ROS 2

As a reference, the [ur_ros_rtde_gripper_commands](../ur_ros_rtde_gripper_commands/) ROS 2 package provides examples of control script extensions used to operate an OnRobot Soft Gripper via our driver.

Originally, the manufacturer provides a URCap for controlling the gripper from the teach pendant, but no software is available to operate it from an external application.

With our extension plugins, it is possible to define new methods in the control script that leverage the OnRobot URCap and communicate with `ur_ros_rtde`.  
Each command is also exposed as an action server, allowing it to be triggered by sending goals.

We were interested in using the following three methods from the OnRobot URCap:
- `sg_grip`: close the gripper  
- `sg_release`: open the gripper  
- `sg_get_width`: get the actual gripper width  

Here is a possible implementation of [sg_grip](../ur_ros_rtde_gripper_commands/src/sg_grip.cpp) using an extension plugin.  
The most crucial part is the implementation of the `generate_modifications` method, which defines how the control script must be modified.

```cpp
static control_script_extension generate_modifications()
{
  control_script_extension ext;
  ext.set_name("sg_grip");
  const std::string body = R"(
  textmsg("sg_grip")
  sg_grip($in18i,$in19i==1,$in20i,$in21i==1,$in22i==1)
  $out19f=get_sg_Width()
  )";
  ext.set_body(body);
  ext.set_preamble({"OnRobotSG", ON_ROBOT_SG_PREAMBLE_STR});
  return ext;
}

void get_control_script_modifications(control_script_extension &extension)
{
  extension = generate_modifications();
}
```

As shown in the snippet, it is possible to use the `control_script_extension` class from [`control_script_utils.hpp`](../ur_ros_rtde/include/ur_ros_rtde/control_script_utils.hpp) to define a new command that must be added to the control script.

This class allows you to define an extension by specifying:
- the name (i.e., the command name)  
- the body (i.e., the URScript methods to be called when the command execution is requested)

In this example, the name is set to `sg_grip`, and the body contains the URScript method that would be called from the teach pendant, except that arguments are passed via registers.   
Registers can be used in a compact form as `{in/out}{reg_number}{type}`, which will be translated into the correct URScript syntax.

The registers used in the extension must then be used to interact with the plugin when the plugin command is triggered.  
This interaction is defined by implementing `start_action_server`, as for the other plugin types.   
For reference, see the corresponding C++ implementation.