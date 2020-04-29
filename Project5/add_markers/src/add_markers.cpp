#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
	ros::init(argc, argv, "add_markers");
	ros::NodeHandle n;
	ros::Rate r(1);
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

	bool robot_at_pickup_zone = false;
	bool robot_at_dropoff_zone = false;

	// Set shape type to be a cube
	visualization_msgs::Marker marker;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.header.frame_id = "map";

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "add_markers";
	marker.id = 0;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = 4;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 0.3;
	marker.scale.y = 0.3;
	marker.scale.z = 0.01;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.0f;
	marker.color.g = 0.0f;
	marker.color.b = 1.0f;
	marker.color.a = 1.0;

	while (ros::ok())
	{
		
		// Set the frame ID and timestamp
		marker.header.stamp = ros::Time::now();

		// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
		marker.action = visualization_msgs::Marker::ADD;

		
		marker.lifetime = ros::Duration();

		// Publish the marker
		while (marker_pub.getNumSubscribers() < 1)
		{
			if (!ros::ok())
			{
				return 0;
			}
			ROS_WARN_ONCE("Please create a subscriber to the marker");
			sleep(1);
		}
		marker_pub.publish(marker);
		
		r.sleep();

		//ros::Duration(5).sleep(); // sleep for 5 seconds
		n.getParam("robot_at_pickup_zone", robot_at_pickup_zone);
		if (robot_at_pickup_zone)
			break;
	}  // end of while




	// DELETE MARKER
	ROS_INFO("Deleting marker");
	marker.action = visualization_msgs::Marker::DELETE;
	marker_pub.publish(marker);
	// ros::Duration(5).sleep(); // sleep for 5 seconds

	while (true)   // blocks program execution until robot has reached drop-off zone
	{
		n.getParam("robot_at_dropoff_zone", robot_at_dropoff_zone);
		if (robot_at_dropoff_zone)
			break;
	}


	// ADD MARKER AT DROP OFF ZONE
	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = -2;
	marker.pose.position.y = 4;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	ROS_INFO("Adding marker at drop-off location");
	while (ros::ok())
	{
		
		// Set the frame ID and timestamp
		marker.header.stamp = ros::Time::now();

		// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
		marker.action = visualization_msgs::Marker::ADD;

		
		marker.lifetime = ros::Duration();

		// Publish the marker
		while (marker_pub.getNumSubscribers() < 1)
		{
			if (!ros::ok())
			{
				return 0;
			}
			ROS_WARN_ONCE("Please create a subscriber to the marker");
			sleep(1);
		}
		marker_pub.publish(marker);
		
		r.sleep();
	}  // end of while
}
