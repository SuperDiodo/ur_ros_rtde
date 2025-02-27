#ifndef ROBOT_STATE_RECEIVER_HPP
#define ROBOT_STATE_RECEIVER_HPP

// UTILS
#include <rclcpp/rclcpp.hpp>
#include <ur_rtde/rtde_receive_interface.h>
#include <eigen3/Eigen/Dense>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

// MESSAGES
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <ur_ros_rtde_msgs/msg/io_state.hpp>

using WrenchMsg = geometry_msgs::msg::WrenchStamped;
using PoseMsg = geometry_msgs::msg::Pose;
using JointStateMsg = sensor_msgs::msg::JointState;
using TfMsg = tf2_msgs::msg::TFMessage;
using MarkerMsg = visualization_msgs::msg::Marker;
using IOStateMsg = ur_ros_rtde_msgs::msg::IOState;

// SERVICES
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <ur_ros_rtde_msgs/srv/get_io_state.hpp>
#include <ur_ros_rtde_msgs/srv/get_tcp_pose.hpp>
#include <ur_ros_rtde_msgs/srv/get_wrench.hpp>
#include <ur_ros_rtde_msgs/srv/get_joint_state.hpp>
#include <ur_ros_rtde_msgs/srv/start_data_recording.hpp>

using SwitchServiceType = std_srvs::srv::SetBool;
using IOStateServiceType = ur_ros_rtde_msgs::srv::GetIOState;
using TcpPoseServiceType = ur_ros_rtde_msgs::srv::GetTcpPose;
using WrenchServiceType = ur_ros_rtde_msgs::srv::GetWrench;
using JointStateServiceType = ur_ros_rtde_msgs::srv::GetJointState;
using StartDataRecordingServiceType = ur_ros_rtde_msgs::srv::StartDataRecording;
using StopDataRecordingServiceType = std_srvs::srv::Trigger;

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

typedef struct{
    std::vector<bool> digital_input_state;
    std::vector<bool> digital_output_state;
    double speed_slider_value;
    double payload_value;
} internal_state;

typedef struct{
    bool digital_pins_state;
    bool payload_value;
    bool joint_velocities;
    bool joint_positions;
    bool speed_slider_value;
    bool tcp_pose;
    bool wrench;
} data_to_record;

class robot_state_receiver
{
public:
    robot_state_receiver(rclcpp::Node::SharedPtr node);
    ~robot_state_receiver();

private:

    // ur_rtde stuff
    std::shared_ptr<ur_rtde::RTDEReceiveInterface> receiver_interface_;
    
    // ros2 stuff
    rclcpp::Node::SharedPtr node_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    tf2_ros::Buffer::SharedPtr tf_buffer_;
    rclcpp::Publisher<MarkerMsg>::SharedPtr marker_pub_;
    rclcpp::Publisher<WrenchMsg>::SharedPtr force_sensor_pub_;
    rclcpp::Publisher<JointStateMsg>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<JointStateMsg>::SharedPtr fake_joint_state_pub_;
    rclcpp::Publisher<JointStateMsg>::SharedPtr real_joint_state_pub_;
    rclcpp::Publisher<TfMsg>::SharedPtr calibrated_camera_tf_pub_;
    rclcpp::Publisher<IOStateMsg>::SharedPtr io_state_pub_;
    rclcpp::Publisher<PoseMsg>::SharedPtr tcp_pose_pub_;
    rclcpp::Subscription<JointStateMsg>::SharedPtr fake_joint_state_sub_;
    rclcpp::Service<SwitchServiceType>::SharedPtr switch_joint_state_type_service_;
    rclcpp::Service<IOStateServiceType>::SharedPtr get_io_state_service_;
    rclcpp::Service<TcpPoseServiceType>::SharedPtr get_tcp_pose_service_;
    rclcpp::Service<WrenchServiceType>::SharedPtr get_wrench_service_;
    rclcpp::Service<JointStateServiceType>::SharedPtr get_joint_state_service_;
    rclcpp::Service<StartDataRecordingServiceType>::SharedPtr start_data_recording_service_;
    rclcpp::Service<StopDataRecordingServiceType>::SharedPtr stop_data_recording_service_;
    JointStateMsg last_fake_joint_state_msg_;

    // receiver configuration
    bool publish_fake_joint_states = false;
    bool publish_force_sensor_markers_ = false;
    bool simulation_only_ = false;
    
    // robot configuration
    std::vector<std::string> joint_names_;
    std::string robot_base_link;
    std::string robot_flange_link;
    bool valid_camera_calibration_file_;
    geometry_msgs::msg::TransformStamped calibrated_camera_tf_;

    // exchanged data
    std::shared_ptr<PoseMsg> tcp_pose_;
    std::shared_ptr<JointStateMsg> joint_state_;
    std::shared_ptr<WrenchMsg> wrench_;
    std::shared_ptr<IOStateMsg> io_state_;

    // general variables
    int timer_iterations_ = 0;
    bool recording_data_ = false;
    std::string record_filename_ = "";
    
    void timer_callback();
    void switch_joint_state_type_cb(const std::shared_ptr<SwitchServiceType::Request> request, std::shared_ptr<SwitchServiceType::Response> response);
    void get_io_state_cb(const std::shared_ptr<IOStateServiceType::Request> request, std::shared_ptr<IOStateServiceType::Response> response);
    void get_tcp_pose_cb(const std::shared_ptr<TcpPoseServiceType::Request> request, std::shared_ptr<TcpPoseServiceType::Response> response);
    void get_wrench_cb(const std::shared_ptr<WrenchServiceType::Request> request, std::shared_ptr<WrenchServiceType::Response> response);
    void get_joint_state_cb(const std::shared_ptr<JointStateServiceType::Request> request, std::shared_ptr<JointStateServiceType::Response> response);
    void start_data_recording_cb(const std::shared_ptr<StartDataRecordingServiceType::Request> request, std::shared_ptr<StartDataRecordingServiceType::Response> response);
    void stop_data_recording_cb(const std::shared_ptr<StopDataRecordingServiceType::Request> request, std::shared_ptr<StopDataRecordingServiceType::Response> response);
    void subscriber_callback(const JointStateMsg::SharedPtr msg);
    void publish_wrench_markers(const WrenchMsg &wrench);
};

#endif
