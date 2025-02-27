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

struct urscript_preamble {
    std::string name;
    std::string body;
};

class control_script_extension
{

public:
    control_script_extension() {};
    control_script_extension(const std::string &name, const std::string &body) : name_(name), body_(body) {}
    void set_name(const std::string name) { name_ = name; }
    void set_body(const std::string body) { body_ = body; }
    void set_preamble(const urscript_preamble preamble) { preamble_ = preamble; }
    std::string get_name() { return name_; }
    std::string get_body() { return body_; }
    urscript_preamble get_preamble() {return preamble_; }

private:
    std::string name_ = "";
    std::string body_ = "";
    urscript_preamble preamble_ = {"", ""};
};

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

static void add_control_register_interaction(std::string &control_script) __attribute__((unused));
inline void add_control_register_interaction(std::string &control_script)
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

static void add_control_script_preamble(std::string &control_script, const urscript_preamble &preamble) __attribute__((unused));
inline void add_control_script_preamble(std::string &control_script, const urscript_preamble &preamble)
{
    if(preamble.body == "") return;
    const std::string header_ref_line = "HEADER_BEGIN\n";
    std::string after_ref = "### " + preamble.name + " PREAMBLE BEGIN\n" + preamble.body + "\n### " + preamble.name + " PREAMBLE END\n";
    size_t pos = control_script.find(header_ref_line);
    if (pos != std::string::npos)
        control_script.insert(pos + header_ref_line.length(), after_ref);
}

static std::string inject_extension(const std::string &ext_name, const std::string &ext_body) __attribute__((unused));
std::string inject_extension(const std::string &ext_name, const std::string &ext_body)
{
    std::string body = ext_body;
    std::regex placeholder_pattern(R"(\$(out|in)(\d+)(i|f))"); // Regex pattern

    int temp_vars_id = 0;
    size_t search_pos = 0;

    while (true) {
        std::smatch match;
        std::string remaining_body = body.substr(search_pos);

        if (!std::regex_search(remaining_body, match, placeholder_pattern))
            break;

        size_t match_pos = search_pos + match.position(0);

        bool read_replacement = match[1].str() == "in";
        std::string reg_type = match[3].str() == "i" ? "integer_reg" : "float_reg";
        std::string replacement;

        if (read_replacement) {
            replacement = "read_input_" + reg_type + "(" + match[2].str() + ")";
        } else {
            replacement = "var_" + ext_name +  std::to_string(temp_vars_id++);
            size_t pos_end = body.find('\n', match_pos);
            pos_end = (pos_end == std::string::npos) ? body.size() : pos_end;
            std::string new_line = "write_output_" + reg_type + "(" + match[2].str() + ", " + replacement + ")\n";
            body.insert(pos_end + 1, new_line);
        }

        body.replace(match_pos, match.length(0), replacement);
        search_pos = match_pos + replacement.length();
    }

    return body;
}

static void apply_to_script(std::string &script, const int &id, control_script_extension &extension) __attribute__((unused));
void apply_to_script(std::string &script, const int &id, control_script_extension &extension)
{
    extension.set_body(inject_extension(extension.get_name(), extension.get_body()));
    const std::string modifications = "elif cmd == " + std::to_string(id) + ":\n\t" + extension.get_body();

    // add info about the extension before control loop
    const std::string control_loop_ref_line = "while keep_running:\n";
    const std::string info_txt = "textmsg(\"added " + extension.get_name() + " as cmd " + std::to_string(id) + "\")\n\n\t";
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
            script.insert(previousEndPos, modifications);
    }
}

#endif