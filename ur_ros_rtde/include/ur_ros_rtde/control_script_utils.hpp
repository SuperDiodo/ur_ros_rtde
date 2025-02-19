#ifndef CONTROL_SCRIPT_UTILS_HPP
#define CONTROL_SCRIPT_UTILS_HPP

#include <ur_ros_rtde/utils.hpp>

#define COMMAND_REQUEST_REGISTER 18
#define COMMAND_STATUS_REGISTER 18

#define EXT_CMD_IDLE 0
#define EXT_CMD_BUSY 1
#define EXT_CMD_DONE 2

#define REGISTER_TYPE_INT 0
#define REGISTER_TYPE_BOOL 1
#define REGISTER_TYPE_DOUBLE 2

#define INPUT_INTEGER_REG_1 18
#define INPUT_INTEGER_REG_2 19
#define INPUT_INTEGER_REG_3 20
#define INPUT_INTEGER_REG_4 21
#define INPUT_INTEGER_REG_5 22

#define INPUT_DOUBLE_REG_1 19
#define INPUT_DOUBLE_REG_2 20
#define INPUT_DOUBLE_REG_3 21
#define INPUT_DOUBLE_REG_4 22

#define OUTPUT_INTEGER_REG_1 18
#define OUTPUT_INTEGER_REG_2 19
#define OUTPUT_INTEGER_REG_3 20
#define OUTPUT_INTEGER_REG_4 21
#define OUTPUT_INTEGER_REG_5 22

#define OUTPUT_DOUBLE_REG_1 19
#define OUTPUT_DOUBLE_REG_2 20
#define OUTPUT_DOUBLE_REG_3 21
#define OUTPUT_DOUBLE_REG_4 22

#define COMMAND_ID_OFFSET 255

static std::string load_control_script(const std::string &filename) __attribute__((unused));
inline std::string load_control_script(const std::string &filename)
{
    std::ifstream file(filename);
    if (!file)
    {
        std::cerr << "Could not open the file: " << filename << std::endl;
        return 0;
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

static void write_control_script(const std::string &filename, const std::string &script) __attribute__((unused));
inline void write_control_script(const std::string &filename, const std::string &script)
{
    std::ofstream outfile(filename);
    if (!outfile)
    {
        std::cerr << "Error creating file: " << filename << std::endl;
        return;
    }
    outfile << script;
    outfile.close();
}

struct register_arg
{
    int register_id;
    int register_type;
    register_arg(const int &RegisterId, const int &RegisterType) : register_id(RegisterId), register_type(RegisterType) {}
};

static void add_control_script_preamble(std::string &control_script) __attribute__((unused));
inline void add_control_script_preamble(std::string &control_script)
{
    const std::string before_while_ref_line = "while keep_running:\n";
    std::string before_ref = "";
    before_ref = before_ref + "textmsg(\"WARNING: this is a custom control script modified with ur_ros_rtde!\")\n\n";
    before_ref += "\t\twrite_output_float_reg(" + std::to_string(COMMAND_STATUS_REGISTER) + ", " + std::to_string(EXT_CMD_IDLE) + ")\n";
    before_ref += "\t\text_ready = True\n\t\t";
    size_t pos = control_script.find(before_while_ref_line);
    if (pos != std::string::npos)
        control_script.insert(pos, before_ref);

    const std::string after_while_ref_line = "cmd = rtde_cmd()\n";
    std::string after_ref = "";
    after_ref += "ext_cmd = floor(read_input_float_reg(" + std::to_string(COMMAND_REQUEST_REGISTER) + "))\n";
    after_ref += "\t    if ext_cmd == " + std::to_string(EXT_CMD_IDLE) + " and not ext_ready:\n";
    after_ref += "\t      ext_ready = True\n";
    after_ref += "\t      write_output_float_reg(" + std::to_string(COMMAND_STATUS_REGISTER) + ", " + std::to_string(EXT_CMD_IDLE) + ")\n";
    after_ref += "\t    end\n\n";

    after_ref += "\t    if ext_cmd != " + std::to_string(EXT_CMD_IDLE) + " and ext_cmd > " + std::to_string(COMMAND_ID_OFFSET) + " and ext_ready:\n";
    after_ref += "\t      ext_ready = False\n";
    after_ref += "\t      write_output_float_reg(" + std::to_string(COMMAND_STATUS_REGISTER) + ", " + std::to_string(EXT_CMD_BUSY) + ")\n";
    after_ref += "\t      process_cmd(ext_cmd)\n";
    after_ref += "\t      write_output_float_reg(" + std::to_string(COMMAND_STATUS_REGISTER) + ", " + std::to_string(EXT_CMD_DONE) + ")\n";
    after_ref += "\t    end\n\n\t    ";

    pos = control_script.find(after_while_ref_line);
    if (pos != std::string::npos)
        control_script.insert(pos, after_ref);
}

class control_script_extension
{

public:
    control_script_extension(const std::string &extension_name, const int &id) : extension_name_(extension_name), id_(id) {}

    void set_extension_id(const int &id) { id_ = id; }

    template <typename... Args>
    void add_method(const std::string &method_name, Args... args)
    {
        lines_.push_back(compose_method(method_name, args...));
    }

    template <typename... Args>
    void add_method_with_result(const std::string &method_name, const register_arg &result_reg, Args... args)
    {
        auto method = compose_method(method_name, args...);
        std::string method_with_result = "";
        if (result_reg.register_type == REGISTER_TYPE_DOUBLE)
            method_with_result = method_with_result + "write_output_float_reg(" + std::to_string(result_reg.register_id) + "," + method + ")";
        else
            method_with_result = method_with_result + "write_output_integer_reg(" + std::to_string(result_reg.register_id) + "," + method + ")";
        lines_.push_back(method_with_result);
    }

    void apply_to_script(std::string &script)
    {
        control_script_extension_ += "elif cmd == " + std::to_string(id_) + ":\n";
        for (auto m : lines_)
            control_script_extension_ += "\t      " + m + "\n\t";

        // add info about the extension before control loop
        const std::string control_loop_ref_line = "while keep_running:\n";
        const std::string info_txt = "textmsg(\"added " + extension_name_ + " as cmd " + std::to_string(id_) + "\")\n\n\t";
        size_t control_loop_pos = script.find(control_loop_ref_line);
        if (control_loop_pos != std::string::npos)
            script.insert(control_loop_pos, info_txt);

        // add extension to 'process_cmd'
        const std::string process_cmd_ref_line = "if cmd != 255:\n";
        size_t process_cmd_pos = script.find(process_cmd_ref_line);

        if (process_cmd_pos != std::string::npos)
        {
            size_t previousEndPos = script.rfind("end\n", process_cmd_pos);
            if (previousEndPos != std::string::npos)
                script.insert(previousEndPos, control_script_extension_);
        }
    }

private:
    std::string extension_name_;
    int id_ = 0;
    std::vector<std::string> lines_;
    std::string control_script_extension_ = "";

    template <typename... Args>
    std::string compose_method(const std::string &method_name, Args... args)
    {
        std::string method = "";
        method += method_name + "(";

        if constexpr (sizeof...(args) > 0)
        {
            auto process_arg = [&](auto &&arg, bool is_last)
            {
                using ArgType = std::decay_t<decltype(arg)>;
                if constexpr (std::is_same<ArgType, register_arg>::value)
                {
                    if (arg.register_type == REGISTER_TYPE_BOOL)
                        method += "read_input_integer_reg(" + std::to_string(arg.register_id) + ")==1";
                    else if (arg.register_type == REGISTER_TYPE_DOUBLE)
                        method += "read_input_float_reg(" + std::to_string(arg.register_id) + ")";
                    else
                        method += "read_input_integer_reg(" + std::to_string(arg.register_id) + ")";
                }
                else if constexpr (std::is_same<ArgType, bool>::value)
                    method += (arg ? "True" : "False");
                else if constexpr (std::is_arithmetic<ArgType>::value)
                    method += std::to_string(arg);
                else
                    method += arg;

                if (!is_last)
                    method += ", ";
                else
                    method += ")";
            };

            std::size_t num_args = sizeof...(args);
            std::size_t index = 0;
            ((process_arg(args, ++index == num_args)), ...);
        }
        else method += ")";

        return method;
    }
};

#endif