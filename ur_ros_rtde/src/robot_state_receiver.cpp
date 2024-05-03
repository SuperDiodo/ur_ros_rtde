#include <ur_ros_rtde/robot_state_receiver.hpp>

robot_state_receiver::robot_state_receiver(rclcpp::Node::SharedPtr node) : node_(node)
{
    std::string param_string;
    int param_int;
    double param_double;

    joint_names_ = node_->declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>{
                                                                                         "shoulder_pan_joint",
                                                                                         "shoulder_lift_joint",
                                                                                         "elbow_joint",
                                                                                         "wrist_1_joint",
                                                                                         "wrist_2_joint",
                                                                                         "wrist_3_joint"});

    robot_base_link = node_->declare_parameter<std::string>("robot_base_link", "base_link_inertia");
    robot_flange_link = node_->declare_parameter<std::string>("robot_flange_link", "wrist_3_link");

    param_string = node_->declare_parameter<std::string>("joint_states_topic", "/joint_states");
    joint_state_pub_ = node_->create_publisher<JointStateMsg>("/joint_states", RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    RCLCPP_INFO(node->get_logger(), "Created joint state publisher..");

    simulation_only_ = node_->declare_parameter<bool>("simulation_only", false);

    param_string = node_->declare_parameter<std::string>("fake_joint_states_topic", "/fake_joint_states");
    fake_joint_state_pub_ = node_->create_publisher<JointStateMsg>(param_string, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    RCLCPP_INFO(node->get_logger(), "Created fake joint state publisher..");

    fake_joint_state_sub_ = node->create_subscription<JointStateMsg>(param_string, 10, std::bind(&robot_state_receiver::subscriber_callback, this, _1));
    RCLCPP_INFO(node->get_logger(), "Created subscriber to fake joint states..");

    param_string = node_->declare_parameter<std::string>("publish_fake_joint_state_service_name", "publish_fake_joint_states");
    switch_joint_state_type_service_ = node->create_service<SwitchServiceType>(param_string, std::bind(&robot_state_receiver::switch_joint_state_type_cb, this, _1, _2));
    RCLCPP_INFO(node->get_logger(), "Created service for switching between fake and real joint states..");

    if (!simulation_only_)
    {
        param_string = node_->declare_parameter<std::string>("robot_ip", "127.0.0.1");
        param_double = node_->declare_parameter<double>("rtde_frequency", 100.0);
        receiver_interface_ = new ur_rtde::RTDEReceiveInterface(param_string, param_double);

        param_string = node_->declare_parameter<std::string>("real_joint_states_topic", "/real_joint_states");
        real_joint_state_pub_ = node_->create_publisher<JointStateMsg>(param_string, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        RCLCPP_INFO(node->get_logger(), "Created real joint state publisher..");

        get_internal_state_service_ = node->create_service<InternalStateServiceType>("get_internal_state", std::bind(&robot_state_receiver::get_internal_state_cb, this, _1, _2));
        RCLCPP_INFO(node->get_logger(), "Created service for getting the internal robot state..");

        get_tcp_pose_service_ = node->create_service<TcpPoseServiceType>("get_tcp_pose", std::bind(&robot_state_receiver::get_tcp_pose_cb, this, _1, _2));
        RCLCPP_INFO(node->get_logger(), "Created service for getting the flange pose..");

        get_wrench_service_ = node->create_service<WrenchServiceType>("get_wrench", std::bind(&robot_state_receiver::get_wrench_cb, this, _1, _2));
        RCLCPP_INFO(node->get_logger(), "Created service for getting wrench..");

        param_string = node_->declare_parameter<std::string>("wrench_topic", "/wrench");
        force_sensor_pub_ = node_->create_publisher<WrenchMsg>(param_string, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        RCLCPP_INFO(node->get_logger(), "Created wrench publisher..");
    }
    else
    {
        publish_fake_joint_states = true;
        last_fake_joint_state_msg_.header.frame_id = "world";
        last_fake_joint_state_msg_.header.stamp = node_->now();
        last_fake_joint_state_msg_.name = joint_names_;
        last_fake_joint_state_msg_.position = node_->declare_parameter<std::vector<double>>("simulation_start_robot_state", std::vector<double>{0., 0., 0., 0., 0., 0.});
        last_fake_joint_state_msg_.effort = std::vector<double>(joint_names_.size(), 0);
        last_fake_joint_state_msg_.velocity = std::vector<double>(joint_names_.size(), 0);
    }

    get_robot_configuration_service_ = node->create_service<RobotConfigurationServiceType>("get_robot_configuration", std::bind(&robot_state_receiver::get_robot_configuration_cb, this, _1, _2));
    RCLCPP_INFO(node->get_logger(), "Created service for getting the robot configuration..");

    param_string = node_->declare_parameter<std::string>("camera_calibration_file", "");

    std::ifstream ifile(param_string.c_str());

    if (param_string.empty() || !ifile)
    {
        if (param_string.empty())
        {
            RCLCPP_INFO(node_->get_logger(), "Param <camera_calibration_file> is empty, no calibrated camera tf will be published..");
            valid_camera_calibration_file_ = false;
        }
        else
        {
            RCLCPP_FATAL(node_->get_logger(), "calibration filereader, invalid file: %s", param_string.c_str());
            exit(1);
        }
    }
    else
    {
        Eigen::Affine3d mat;
        float t;

        for (uint r = 0; r < 3; r++)
        {
            for (uint c = 0; c < 3; c++)
            {
                ifile >> t;
                mat.linear()(r, c) = t;
            }
            ifile >> t;
            mat.translation()[r] = t;
        }

        Eigen::Affine3d camera_offset;
        auto position_offset = node_->declare_parameter<std::vector<double>>("position_offset", std::vector<double>{0., 0., 0.});
        auto orientation_offset = node_->declare_parameter<std::vector<double>>("orientation_offset", std::vector<double>{0., 0., 0.});
        camera_offset.translation() << position_offset[0], position_offset[1], position_offset[2];

        Eigen::AngleAxisd rollAngle(orientation_offset[0], Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(orientation_offset[1], Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(orientation_offset[2], Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

        camera_offset.linear() = q.toRotationMatrix();

        mat = camera_offset * mat;

        calibrated_camera_tf_.header.frame_id = node_->declare_parameter<std::string>("calibrated_camera_parent_tf_name", "camera_aligned_tool_link");
        calibrated_camera_tf_.child_frame_id = node_->declare_parameter<std::string>("calibrated_camera_tf_name", "camera_color_optical_frame");
        calibrated_camera_tf_.transform.translation.x = mat.translation().x();
        calibrated_camera_tf_.transform.translation.y = mat.translation().y();
        calibrated_camera_tf_.transform.translation.z = mat.translation().z();
        Eigen::Quaterniond quat = (Eigen::Quaterniond)mat.linear();
        calibrated_camera_tf_.transform.rotation.x = quat.x();
        calibrated_camera_tf_.transform.rotation.y = quat.y();
        calibrated_camera_tf_.transform.rotation.z = quat.z();
        calibrated_camera_tf_.transform.rotation.w = quat.w();
        valid_camera_calibration_file_ = true;
        calibrated_camera_tf_pub_ = node_->create_publisher<TfMsg>("tf", RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        RCLCPP_INFO(node->get_logger(), "Computed camera calibration transform..");
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    param_int = node_->declare_parameter<int>("data_receiving_frequency", 100);
    int freq = (1000.0 / (double)param_int);
    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(freq),
        std::bind(&robot_state_receiver::timer_callback, this));
    RCLCPP_INFO(node->get_logger(), "Created timer at %d hz ( %d ms of period ) ..", param_int, freq);
}

robot_state_receiver::~robot_state_receiver()
{

    receiver_interface_->disconnect();

    free(receiver_interface_);
    delete (receiver_interface_);

    free(robot_internal_state_);
    delete (robot_internal_state_);

    free(tcp_pose_);
    delete (tcp_pose_);

    free(robot_configuration_);
    delete (robot_configuration_);
}

void robot_state_receiver::timer_callback()
{

    // Get receiver interface status
    if (!simulation_only_)
    {
        if (!receiver_interface_->isConnected())
        {
            RCLCPP_INFO(node_->get_logger(), "Receiver interface disconnected, reconnect..");
            try
            {
                receiver_interface_->reconnect();
            }
            catch (...)
            {
                RCLCPP_WARN(node_->get_logger(), "Reconnection failed, waiting some time..");
                return;
            }

            RCLCPP_INFO(node_->get_logger(), "Receiver interface reconnected!");
        }

        if (receiver_interface_->isEmergencyStopped())
        {
            RCLCPP_WARN(node_->get_logger(), "Receiver interface emergency stop enabled!");
        }

        if (receiver_interface_->isProtectiveStopped())
        {
            RCLCPP_WARN(node_->get_logger(), "Receiver interface protective stop enabled!");
        }
    }

    // Receive and forward robot configuration
    {
        if (simulation_only_)
        {
            last_fake_joint_state_msg_.header.stamp = node_->now();
            joint_state_pub_->publish(last_fake_joint_state_msg_);
        }
        else
        {

            if (robot_configuration_ == nullptr)
            {
                robot_configuration_ = new JointStateMsg;
            }

            std::vector<double> robot_configuration = receiver_interface_->getActualQ();
            JointStateMsg robot_configuration_msg;
            robot_configuration_msg.header.stamp = node_->now();
            robot_configuration_msg.name = joint_names_;
            robot_configuration_msg.header.frame_id = "world";
            robot_configuration_msg.position = robot_configuration;
            robot_configuration_msg.velocity = std::vector<double>(joint_names_.size(), 0.0);
            robot_configuration_msg.effort = std::vector<double>(joint_names_.size(), 0.0);
            real_joint_state_pub_->publish(robot_configuration_msg);
            if (!publish_fake_joint_states)
                last_fake_joint_state_msg_ = robot_configuration_msg;
            last_fake_joint_state_msg_.header.stamp = node_->now();
            joint_state_pub_->publish(publish_fake_joint_states ? last_fake_joint_state_msg_ : robot_configuration_msg);
            robot_configuration_->name = joint_names_;
            robot_configuration_->position = robot_configuration_msg.position;
        }
    }

    // Receive and update internal robot state
    if (!simulation_only_)
    {
        if (robot_internal_state_ == nullptr)
            robot_internal_state_ = new internal_state;

        robot_internal_state_->digital_input_state.clear();
        robot_internal_state_->digital_output_state.clear();
        for (int pin = 0; pin < 8; pin++)
        {
            robot_internal_state_->digital_input_state.push_back(receiver_interface_->getDigitalInState(pin));
            robot_internal_state_->digital_output_state.push_back(receiver_interface_->getDigitalOutState(pin));
        }

        robot_internal_state_->speed_slider_value = receiver_interface_->getSpeedScaling();
        robot_internal_state_->payload_value = receiver_interface_->getPayload();
        robot_internal_state_->payload_value = 2.0;
    }

    // Receive and update flange pose
    if (!simulation_only_)
    {
        if (tcp_pose_ == nullptr)
            tcp_pose_ = new PoseMsg;

        std::vector<double> flange_pose = receiver_interface_->getActualTCPPose();
        tcp_pose_->position.x = flange_pose[0];
        tcp_pose_->position.y = flange_pose[1];
        tcp_pose_->position.z = flange_pose[2];

        Eigen::Vector3d rotation_vector(flange_pose[3], flange_pose[4], flange_pose[5]);
        double angle = rotation_vector.norm();

        Eigen::Quaterniond quat;
        quat = Eigen::AngleAxisd(angle, rotation_vector / angle);

        tcp_pose_->orientation.x = quat.x();
        tcp_pose_->orientation.y = quat.y();
        tcp_pose_->orientation.z = quat.z();
        tcp_pose_->orientation.w = quat.w();
    }

    // Receive and forward force sensor data
    if (!simulation_only_)
    {
        std::vector<double> tcp_force_data = receiver_interface_->getActualTCPForce();

        Eigen::Vector3d forces(tcp_force_data[0], tcp_force_data[1], tcp_force_data[2]);
        Eigen::Vector3d torques(tcp_force_data[3], tcp_force_data[4], tcp_force_data[5]);

        Eigen::Quaterniond tcp_quat(tcp_pose_->orientation.w, tcp_pose_->orientation.x, tcp_pose_->orientation.y, tcp_pose_->orientation.z);

        forces = tcp_quat.inverse() * forces;
        torques = tcp_quat.inverse() * torques;

        if (wrench_ == nullptr)
            wrench_ = new WrenchMsg();

        wrench_->header.stamp = node_->now();
        wrench_->header.frame_id = "world";
        wrench_->wrench.force.x = forces.x();
        wrench_->wrench.force.y = forces.y();
        wrench_->wrench.force.z = forces.z();
        wrench_->wrench.torque.x = torques.x();
        wrench_->wrench.torque.y = torques.y();
        wrench_->wrench.torque.z = torques.z();

        force_sensor_pub_->publish(*wrench_);
    }

    // publish calibrated camera tf
    if (valid_camera_calibration_file_)
    {
        calibrated_camera_tf_.header.stamp = node_->now();

        TfMsg msg;
        msg.transforms.push_back(calibrated_camera_tf_);

        calibrated_camera_tf_pub_->publish(msg);
    }
}

void robot_state_receiver::subscriber_callback(const JointStateMsg::SharedPtr msg)
{
    last_fake_joint_state_msg_.position = msg->position;
}

void robot_state_receiver::switch_joint_state_type_cb(const std::shared_ptr<SwitchServiceType::Request> request, std::shared_ptr<SwitchServiceType::Response> response)
{
    publish_fake_joint_states = simulation_only_ ? true : request->data;
    response->success = true;
}

void robot_state_receiver::get_internal_state_cb(const std::shared_ptr<InternalStateServiceType::Request> request, std::shared_ptr<InternalStateServiceType::Response> response)
{

    (void)request;

    if (robot_internal_state_ == nullptr)
    {
        response->success = false;
        return;
    }

    response->digital_input_state = robot_internal_state_->digital_input_state;
    response->digital_output_state = robot_internal_state_->digital_output_state;
    response->speed_slider_value = robot_internal_state_->speed_slider_value;
    response->payload_value = robot_internal_state_->payload_value;
    response->success = true;
}

void robot_state_receiver::get_tcp_pose_cb(const std::shared_ptr<TcpPoseServiceType::Request> request, std::shared_ptr<TcpPoseServiceType::Response> response)
{
    (void)request;

    if (tcp_pose_ == nullptr)
    {
        response->success = false;
        return;
    }

    response->pose.position = tcp_pose_->position;
    response->pose.orientation = tcp_pose_->orientation;
    response->success = true;
}

void robot_state_receiver::get_wrench_cb(const std::shared_ptr<WrenchServiceType::Request> request, std::shared_ptr<WrenchServiceType::Response> response)
{
    (void)request;

    if (wrench_ == nullptr)
    {
        response->success = false;
        return;
    }

    response->wrench = wrench_->wrench;
    response->success = true;
}

void robot_state_receiver::get_robot_configuration_cb(const std::shared_ptr<RobotConfigurationServiceType::Request> request, std::shared_ptr<RobotConfigurationServiceType::Response> response)
{
    (void)request;

    if (simulation_only_)
    {
        response->names = last_fake_joint_state_msg_.name;
        response->values = last_fake_joint_state_msg_.position;
        response->success = true;
    }

    if (robot_configuration_ == nullptr)
    {
        response->success = false;
        return;
    }

    response->names = robot_configuration_->name;
    response->values = robot_configuration_->position;
    response->success = true;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("ur_ros_rtde_robot_state_receiver_node");
    std::shared_ptr<rclcpp::Node> node_srv = rclcpp::Node::make_shared("ur_ros_rtde_srv_node");
    auto rsr = robot_state_receiver(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
}