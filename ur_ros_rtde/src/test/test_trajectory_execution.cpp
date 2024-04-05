#include <ur_ros_rtde_simple_clients/simple_action_client_template.hpp>
#include <ur_ros_rtde_simple_clients/simple_service_client_template.hpp>

/* INCLUDE ACTIONS TO USE */
#include <ur_ros_rtde_msgs/action/execute_trajectory.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("evaluate_corrector_node");
    auto node_act = std::make_shared<rclcpp::Node>("evaluate_corrector_action_client_node");

    /* action client for communication with robot */
    auto action_client = simple_action_client_template(node_act);

    /*
        TRAJECTORY EXECUTION
        the trajectory was built with 3 robot configurations.
    */

    ur_ros_rtde_msgs::action::ExecuteTrajectory::Goal goal_msg;
    goal_msg.acceleration = 0.5;
    goal_msg.speed = 0.5;
    goal_msg.deceleration = 5.0;
    ur_ros_rtde_msgs::msg::Vector joint_vector;

    // initial state
    {
        joint_vector.vector = {-1.822409454976217, -1.5724393330016078, 1.772125546132223, -1.7732678852477015, -1.5799019972430628, -0.24573165575136358};
        goal_msg.trajectory.push_back(joint_vector);
    }

    {
        joint_vector.vector = {-1.726349178944723, -1.5776134930052699, 1.6113598982440394, -1.9500886402525843, -0.6720364729510706, -0.7341492811786097};
        goal_msg.trajectory.push_back(joint_vector);
    }

    {
        joint_vector.vector = {-2.1398356596576136, -2.0075184307494105, 2.0258382002459925, -1.5872675381102503, -1.5326102415667933, -0.5949648062335413};
        goal_msg.trajectory.push_back(joint_vector);
    }

    // repeat first conf. to move the robot in the initial state
    {
        joint_vector.vector = {-1.822409454976217, -1.5724393330016078, 1.772125546132223, -1.7732678852477015, -1.5799019972430628, -0.24573165575136358};
        goal_msg.trajectory.push_back(joint_vector);
    }

    action_client.send_goal<ur_ros_rtde_msgs::action::ExecuteTrajectory>("ur_ros_rtde/execute_trajectory_command", goal_msg);

    rclcpp::shutdown();
    return 0;
}