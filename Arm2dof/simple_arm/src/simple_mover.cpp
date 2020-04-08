#include "ros/ros.h"
#include "std_msgs/Float64.h"

int main(int argc, char** argv)
{
    // Initialize arm_mover node
    ros::init(argc, argv, "simple_mover");

    // Create handle to arm_mover node
    ros::NodeHandle n;

    // Create publishers to public std::msgs_Float64 for the two robot joints on their respective topics
    ros::Publisher joint1_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command", 10);
    ros::Publisher joint2_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command", 10);

    // Set loop frequency of 10 Hz
    ros::Rate loop_rate(10);

    // Declare some variables
    int start_time, elapsed;
    std_msgs::Float64 joint1_angle, joint2_angle;


    while (not start_time)
        start_time = ros::Time::now().toSec();

    while (ros::ok())
    {
        // Get ROS elapsed time
        elapsed = ros::Time::now().toSec() - start_time;

        // Set arm joint angles
        joint1_angle.data = sin(2 * M_PI * 0.1 * elapsed) * (M_PI / 2);
        joint2_angle.data = sin(2 * M_PI * 0.1 * elapsed) * (M_PI / 2);

        // Publish the joint angles
        joint1_pub.publish(joint1_angle);
        joint2_pub.publish(joint2_angle);

        // Sleep for the remaining time until 10 Hz is reached
        loop_rate.sleep();
    }

    return 0;
}
