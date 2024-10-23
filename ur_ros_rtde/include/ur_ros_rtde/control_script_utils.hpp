#ifndef CONTROL_SCRIPT_UTILS_HPP
#define CONTROL_SCRIPT_UTILS_HPP

#include <ur_ros_rtde/utils.hpp>

#define COMMAND_ID_INTEGER_INPUT_REG 18
#define COMMAND_ID_INTEGER_OUTPUT_REG 18

#define INPUT_INTEGER_REG_1 19
#define INPUT_INTEGER_REG_2 20
#define INPUT_INTEGER_REG_3 21
#define INPUT_INTEGER_REG_4 22

#define INPUT_DOUBLE_REG_1 18
#define INPUT_DOUBLE_REG_2 19
#define INPUT_DOUBLE_REG_3 20
#define INPUT_DOUBLE_REG_4 21
#define INPUT_DOUBLE_REG_5 22

#define OUTPUT_INTEGER_REG_1 19
#define OUTPUT_INTEGER_REG_2 20
#define OUTPUT_INTEGER_REG_3 21
#define OUTPUT_INTEGER_REG_4 22

#define OUTPUT_DOUBLE_REG_1 18
#define OUTPUT_DOUBLE_REG_2 19
#define OUTPUT_DOUBLE_REG_3 20
#define OUTPUT_DOUBLE_REG_4 21
#define OUTPUT_DOUBLE_REG_5 22

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
    bool is_int = false;
    register_arg(const int &RegisterId, const bool &IsInt) : register_id(RegisterId), is_int(IsInt) {}
};

static void add_control_script_preamble(std::string &control_script) __attribute__((unused));
inline void add_control_script_preamble(std::string &control_script)
{
    const std::string ref_line = "while keep_running:\n";
    std::string before_ref = "";
    before_ref = before_ref + "textmsg(\"Custom control script\")\n";
    before_ref = before_ref + "\t    " + "write_output_integer_reg(" + std::to_string(COMMAND_ID_INTEGER_OUTPUT_REG) + ", 0)\n\t    ";
    size_t pos = control_script.find(ref_line);
    if (pos != std::string::npos)
        control_script.insert(pos, before_ref);
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
        if (result_reg.is_int)
            method_with_result = method_with_result + "write_input_integer_reg(" + std::to_string(result_reg.register_id) + "," + method + ")";
        else
            method_with_result = method_with_result + "write_input_float_reg(" + std::to_string(result_reg.register_id) + "," + method + ")";
        lines_.push_back(method_with_result);
    }

    void apply_to_script(std::string &script)
    {
        control_script_extension_ += "\n\t\t    if read_input_integer_reg(" + std::to_string(COMMAND_ID_INTEGER_INPUT_REG) + ") == " + std::to_string(id_) + ":\n\t";
        control_script_extension_ = control_script_extension_ + "\t\t    " + "write_output_integer_reg(" + std::to_string(COMMAND_ID_INTEGER_OUTPUT_REG) + ", 1)\n\t";
        for (auto m : lines_)
            control_script_extension_ += "\t\t    " + m + "\n\t";
        control_script_extension_ = control_script_extension_ + "\t\t    " + "write_output_integer_reg(" + std::to_string(COMMAND_ID_INTEGER_OUTPUT_REG) + ", 2)\n\t";

        control_script_extension_ = control_script_extension_ + "\t\t    while read_input_integer_reg(" + std::to_string(COMMAND_ID_INTEGER_INPUT_REG) + ") == " + std::to_string(id_) + ":\n\t";
        control_script_extension_ = control_script_extension_ + "\t\t\t    sleep(0.002)\n\t\t\t    end\n";
        control_script_extension_ += "\t\t    end\n";

        const std::string ref_line = "while keep_running:\n";
        const std::string before_ref = "textmsg(\"" + extension_name_ + " added to control script\")\n\t    ";

        size_t pos = script.find(ref_line);
        if (pos != std::string::npos)
        {
            script.insert(pos, before_ref);
            pos += before_ref.length() + ref_line.length();
            script.insert(pos, control_script_extension_);
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

        // Helper lambda to process each argument type
        auto process_arg = [&](auto &&arg, bool is_last)
        {
            using ArgType = std::decay_t<decltype(arg)>;
            if constexpr (std::is_same<ArgType, register_arg>::value)
            {
                // Check if it's a register_arg and add appropriate read method
                if (arg.is_int)
                    method += "read_input_integer_reg(" + std::to_string(arg.register_id) + ")";
                else
                    method += "read_input_float_reg(" + std::to_string(arg.register_id) + ")";
            }
            else if constexpr (std::is_arithmetic<ArgType>::value)
                method += std::to_string(arg);
            else
                method += arg;

            if (!is_last)
                method += ", ";
            else
                method += ")";
        };

        // Process each argument and determine if it's the last one
        std::size_t num_args = sizeof...(args);
        std::size_t index = 0;
        ((process_arg(args, ++index == num_args)), ...);
        return method;
    }
};

#endif