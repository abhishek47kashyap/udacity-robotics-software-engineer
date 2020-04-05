#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
  
  
  ball_chaser::DriveToTarget srv;
  srv.request.linear_x = lin_x;
  srv.request.angular_z = ang_z;
  
  if (client.call(srv))
        {
            ROS_INFO("Service called successfully!");
        }
        else
        {
            ROS_ERROR("Failed to call service ");
        }
  
  
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
  	bool ball_in_frame = false;
  	int column_index = -1;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
  
    // Loop through each pixel in the image and check if its equal to the first one
    for (int i = 0; i < (img.height * img.step); i=i+3)
    {
        if (img.data[i] == white_pixel && 
            img.data[i+1] == white_pixel && 
            img.data[i+2] == white_pixel)
        {
            ball_in_frame = true;
          	column_index = int(i/3) % img.width;
            break;
        }
    }
  
  	
    if (!ball_in_frame)  // ball not in frame
    	drive_robot(0.0, 0.0);
    else
    {
      // Determining region: left, center, or right
      if (column_index < int(img.width/3))
      {
        drive_robot(0.0, 0.5);   // turn left bc white_pixel in left region
        ROS_INFO("LEFT REGION");
      }
        
      
      else if (column_index >= int(img.width/3) && column_index <= 2*int(img.width/3))
      {
      	drive_robot(0.5, 0.0);   // go forward
        ROS_INFO("MOVING FORWARD");
      }
        
      
      else if (column_index > 2*int(img.width/3))
      {
        drive_robot(0.0, -0.5);   // turn right bc white_pixel in right region
        ROS_INFO("RIGHT REGION");
      }
        
    }
  
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}