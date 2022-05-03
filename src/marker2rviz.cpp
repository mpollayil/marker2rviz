#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>

using namespace std;

/* ******************************************************************************************** */
void setObjectProperties(string name, vector<double> loc, vector<double> ori, vector<double> dim, 
		visualization_msgs::Marker& marker) {

	// Set the basic marker properties
    marker.header.frame_id = "marker_frame";
    marker.header.stamp = ros::Time::now();
	marker.ns = name;
    marker.id = 0;
    uint32_t shape = visualization_msgs::Marker::CUBE;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

	// Set the location
	marker.pose.position.x = loc.at(0);
	marker.pose.position.y = loc.at(1);
	marker.pose.position.z = loc.at(2);
	marker.pose.orientation.x = ori.at(0);
	marker.pose.orientation.y = ori.at(1);
	marker.pose.orientation.z = ori.at(2);
    marker.pose.orientation.z = ori.at(3);

	// Set the dimensions and color
	marker.scale.x = dim.at(0);
	marker.scale.y = dim.at(1);
	marker.scale.z = dim.at(2);
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    // Forever lasting
    marker.lifetime = ros::Duration();
}

/* ******************************************************************************************** */
int main(int argc, char **argv) {

	// Wait for other nodes to start (assuming launch file)
	// Start the ros node
	ros::init (argc, argv, "sdf2rviz");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::NodeHandle node_handle;

	// ------------------------------------------------------------
	
    // Advertise the planning scene message publisher
	ros::Publisher marker_pub = 
		node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	while(marker_pub.getNumSubscribers() < 1) {
		ROS_INFO("Waiting for a subscriber to the visualization_marker topic.\n");
	  ros::WallDuration sleep_t(0.5);
	  sleep_t.sleep();
	}

	// Spinning
	ROS_INFO("Putting specified boxes into RViz according to marker_frame!");

	while(ros::ok()) {

        // Declare marker
        visualization_msgs::Marker object;

		//----- Start part to repeat for more boxes -----//
		
		// Fill the object pose and dimensions into the object message
        string name = "box";
        vector<double> pose = {1, 2, 3};
		vector<double> orientation = {0.0, 0.0, 0.0, 1.0};
        vector<double> size = {1, 2, 3};
		setObjectProperties(name, pose, orientation, size, object);

		// Send the message
		marker_pub.publish(object);

		//----- End part to repeat for more boxes -----//

		sleep(0.001);
	}

	return EXIT_SUCCESS;
}