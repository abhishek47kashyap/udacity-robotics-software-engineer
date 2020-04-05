#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

// Publisher for motor commands
ros::Publisher motor_command_publisher;

// This callback function executes whenever a handle_drive_request service is requested 
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req,
                          ball_chaser::DriveToTarget::Response& res)
{
    ROS_INFO("DriveToTargetRequest received - linearX:%1.2f, angularZ:%1.2f", (float)req.linear_x, (float)req.angular_z);
    
    // Create a motor_command object of type geometry_msgs::Twist
    geometry_msgs::Twist motor_command;

    // Set wheel velocities
    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;

    // Publish angles to drive the robot
    motor_command_publisher.publish(motor_command);
	
    // Return a response message
    res.msg_feedback = "Wheel velocities published - linearX: " + std::to_string(req.linear_x) +
                       " , angularZ: " + std::to_string(req.angular_z);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}

int main(int argc, char** argv)
{
    // Initialize arm_mover node and create handle
    ros::init(argc, argv, "drive_bot");
    ros::NodeHandle n;

    // Create publishers to public std::msgs_Float64 for the two robot joints on their respective topics
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    // Service with a handle_drive_request callback function (called when service request is received)
    ros::ServiceServer server = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
    ROS_INFO("Ready to set wheel velocities");

    // Handle ROS communication events
    ros::spin();

    return 0;
}
