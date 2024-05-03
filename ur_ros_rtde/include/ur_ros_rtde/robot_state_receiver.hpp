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

using WrenchMsg = geometry_msgs::msg::WrenchStamped;
using PoseMsg = geometry_msgs::msg::Pose;
using JointStateMsg = sensor_msgs::msg::JointState;
using TfMsg = tf2_msgs::msg::TFMessage;
using MarkerMsg = visualization_msgs::msg::Marker;

// SERVICE
#include <std_srvs/srv/set_bool.hpp>
#include <ur_ros_rtde_msgs/srv/get_internal_state.hpp>
#include <ur_ros_rtde_msgs/srv/get_tcp_pose.hpp>
#include <ur_ros_rtde_msgs/srv/get_wrench.hpp>
#include <ur_ros_rtde_msgs/srv/get_robot_configuration.hpp>

using SwitchServiceType = std_srvs::srv::SetBool;
using InternalStateServiceType = ur_ros_rtde_msgs::srv::GetInternalState;
using TcpPoseServiceType = ur_ros_rtde_msgs::srv::GetTcpPose;
using WrenchServiceType = ur_ros_rtde_msgs::srv::GetWrench;
using RobotConfigurationServiceType = ur_ros_rtde_msgs::srv::GetRobotConfiguration;

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

typedef struct{
    std::vector<bool> digital_input_state;
    std::vector<bool> digital_output_state;
    double speed_slider_value;
    double payload_value;
} internal_state;

class robot_state_receiver
{
public:
    robot_state_receiver(rclcpp::Node::SharedPtr node);
    ~robot_state_receiver();

private:

    // ur_rtde stuff
    ur_rtde::RTDEReceiveInterface *receiver_interface_;
    
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
    rclcpp::Subscription<JointStateMsg>::SharedPtr fake_joint_state_sub_;
    rclcpp::Service<SwitchServiceType>::SharedPtr switch_joint_state_type_service_;
    rclcpp::Service<InternalStateServiceType>::SharedPtr get_internal_state_service_;
    rclcpp::Service<TcpPoseServiceType>::SharedPtr get_tcp_pose_service_;
    rclcpp::Service<WrenchServiceType>::SharedPtr get_wrench_service_;
    rclcpp::Service<RobotConfigurationServiceType>::SharedPtr get_robot_configuration_service_;
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
    internal_state* robot_internal_state_ = nullptr;
    PoseMsg* tcp_pose_ = nullptr;
    JointStateMsg* robot_configuration_ = nullptr;
    WrenchMsg *wrench_ = nullptr;
    
    void timer_callback();
    void switch_joint_state_type_cb(const std::shared_ptr<SwitchServiceType::Request> request, std::shared_ptr<SwitchServiceType::Response> response);
    void get_internal_state_cb(const std::shared_ptr<InternalStateServiceType::Request> request, std::shared_ptr<InternalStateServiceType::Response> response);
    void get_tcp_pose_cb(const std::shared_ptr<TcpPoseServiceType::Request> request, std::shared_ptr<TcpPoseServiceType::Response> response);
    void get_wrench_cb(const std::shared_ptr<WrenchServiceType::Request> request, std::shared_ptr<WrenchServiceType::Response> response);
    void get_robot_configuration_cb(const std::shared_ptr<RobotConfigurationServiceType::Request> request, std::shared_ptr<RobotConfigurationServiceType::Response> response);
    void subscriber_callback(const JointStateMsg::SharedPtr msg);
    void publish_wrench_markers(const WrenchMsg &wrench);
};

#endif