#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "simple_arm/GoToPosition.h"

// Global joint publishers (need to be accessible in all functions)
ros::Publisher joint1_pub, joint2_pub;

// This function clamps joint angles to their limits if outside joint limits
std::vector<float> clamp_joint_angles_at_limits(float requested_j1, float requested_j2)
{
    float clamped_j1 = requested_j1, clamped_j2 = requested_j2;

    // Retrieve joint limits from the parameter server
    float min_j1, max_j1, min_j2, max_j2;
    ros::NodeHandle n2;  // since n1 is not available in this function
    std::string node_name = ros::this_node::getName();   // Get node name

    // Acquiring joint limits
    n2.getParam(node_name+"/min_joint_1_angle", min_j1);
    n2.getParam(node_name+"/max_joint_1_angle", max_j1);
    n2.getParam(node_name+"/min_joint_2_angle", min_j2);
    n2.getParam(node_name+"/max_joint_2_angle", max_j2);

    // Check if joint 1 falls in the safe zone, otherwise clamp it
    if (requested_j1 < min_j1 || requested_j1 > max_j1) {
        clamped_j1 = std::min(std::max(requested_j1, min_j1), max_j1);
        ROS_WARN("j1 is out of bounds, valid range (%1.2f,%1.2f), clamping to: %1.2f", min_j1, max_j1, clamped_j1);
    }
    // Check if joint 2 falls in the safe zone, otherwise clamp it
    if (requested_j2 < min_j2 || requested_j2 > max_j2) {
        clamped_j2 = std::min(std::max(requested_j2, min_j2), max_j2);
        ROS_WARN("j2 is out of bounds, valid range (%1.2f,%1.2f), clamping to: %1.2f", min_j2, max_j2, clamped_j2);
    }

    std::vector<float> clamped_joint_angles = {clamped_j1, clamped_j2};

    return clamped_joint_angles;
}

// This callback function executes whenever a safe_move service is requested (callback function HAS TO return something)
bool handle_safe_move_request(simple_arm::GoToPosition::Request& req,
                              simple_arm::GoToPosition::Response& res)
{
    ROS_INFO("GoToPositionRequest received - j1:%1.2f, j2:%1.2f", (float)req.joint_1, (float)req.joint_2);

    // Check if requested joint angles are within limits, if not clamp them
    std::vector<float> joint_angles = clamp_joint_angles_at_limits(req.joint_1, req.joint_2);

    // Publish joint angles to arm (msg type std_msgs::Float64)
    std_msgs::Float64 joint1_angle, joint2_angle;
    joint1_angle.data = joint_angles[0];
    joint2_angle.data = joint_angles[1];
    joint1_pub.publish(joint1_angle);
    joint2_pub.publish(joint2_angle);

    // Wait for 3 seconds for arm to have enough time to move to requested position
    ros::Duration(3).sleep();

    // Return a response message
    res.msg_feedback = "Joint angles published - j1: " + std::to_string(joint_angles[0]) +
                       " , j2: " + std::to_string(joint_angles[1]);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}

int main(int argc, char** argv)
{
    // Initialize arm_mover node and create handle
    ros::init(argc, argv, "arm_mover");
    ros::NodeHandle n;

    // Create publishers to public std::msgs_Float64 for the two robot joints on their respective topics
    joint1_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command", 10);
    joint2_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command", 10);

    // safe_move service with a handle_safe_move_request callback function (called when service request is received)
    ros::ServiceServer server = n.advertiseService("/arm_mover/safe_move", handle_safe_move_request);
    ROS_INFO("Ready to set joint commands");

    // Handle ROS communication events
    ros::spin();

    return 0;
}
