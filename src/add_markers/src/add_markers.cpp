#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <cmath>

// stores a 2d coordinate (an x and y)
class Coordinate {
	public:
		double x;
		double y;

		Coordinate(int x, int y) {
			this->x = x;
			this->y = y;
		}
};

// the robot's pickup and dropoff coordinates
Coordinate pickupCoordinate = Coordinate(-2.0, 4.5);
Coordinate dropoffCoordinate = Coordinate(5.0, 2.5);

// the robot's current state
enum RobotState {
	// the robot is driving to the pickup location
	FINDING_PICKUP,
	
	// the robot is picking up the item
	PICKING_UP,
	
	// the robot is driving to the dropoff location
	FINDING_DROPOFF,
	
	// the robot is dropping off the item
	DROPPING_OFF
};

// the robot's current state
RobotState currentState = FINDING_PICKUP;

// creates and returns a new marker
visualization_msgs::Marker createMarker(uint32_t shape) {
	visualization_msgs::Marker marker;

	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time::now();
	marker.ns = "add_markers";
	marker.id = 0;
	marker.type = shape;

	marker.pose.position.x = 0.0;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = 1.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	marker.scale.x = 1.0;
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;

	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	marker.action = visualization_msgs::Marker::ADD;
	marker.lifetime = ros::Duration();

	return marker;
}

// called when the robot state publisher sends an update
void stateCallback(const std_msgs::Int32::ConstPtr& msg){
	currentState = (RobotState) msg->data;
}

int main(int argc, char** argv){
	// initialize the add_markers node
	ros::init(argc, argv, "add_markers");

	// get a handle to the node
	ros::NodeHandle node;
	ros::Rate r(1.0);
	
	// create the marker publisher and robot state subscriber
	ros::Publisher markerPublisher =
		node.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	ros::Subscriber stateSubscriber =
		node.subscribe("home_service_robot_state", 1, stateCallback);

	// create a marker
	visualization_msgs::Marker marker =
		createMarker(visualization_msgs::Marker::CUBE);

	// continue on a loop
	while (ros::ok()) {
		// wait for subscribers
		while (markerPublisher.getNumSubscribers() < 1) {
			if (!ros::ok()) {
				return 0;
			}

			ROS_WARN_ONCE("Waiting for marker subscribers");
			sleep(1);
		}

		// act on the current robot state
		switch (currentState) {
			case FINDING_PICKUP:
				// show the pickup location marker
				marker.pose.position.x = pickupCoordinate.x;
				marker.pose.position.y = pickupCoordinate.y;
				marker.action = visualization_msgs::Marker::ADD;
				break;
			case PICKING_UP:
				// hide the pickup location marker
				marker.action = visualization_msgs::Marker::DELETE;
				break;
			case FINDING_DROPOFF:
				// show the dropoff location marker
				marker.pose.position.x = dropoffCoordinate.x;
				marker.pose.position.y = dropoffCoordinate.y;
				marker.action = visualization_msgs::Marker::ADD;
				break;
			case DROPPING_OFF:
				// hide the dropoff location marker
				marker.action = visualization_msgs::Marker::DELETE;
				break;
		}

		markerPublisher.publish(marker);
		ros::spinOnce();
	}

	return 0;
}
