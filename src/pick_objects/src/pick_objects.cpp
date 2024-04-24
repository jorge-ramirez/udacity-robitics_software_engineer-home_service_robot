#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <cmath>
#include <std_msgs/Int32.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

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

// returns a goal to the given coordinate.
move_base_msgs::MoveBaseGoal createTargetGoal(Coordinate coordinate) {
	move_base_msgs::MoveBaseGoal goal;

	// set up the frame parameters
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	// set the goal position and orientation
	goal.target_pose.pose.position.x = coordinate.x;
	goal.target_pose.pose.position.y = coordinate.y;
	goal.target_pose.pose.position.z = 1.0;
	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = 0.0;
	goal.target_pose.pose.orientation.w = 1.0;

	return goal;
}

// sends the goal to the move client and waits until complete
bool sendTargetGoal(MoveBaseClient &ac, move_base_msgs::MoveBaseGoal &goal) {
	// log and send the goal
	ROS_INFO(
		"Sending goal: {x: %0.4lf, y: %0.4lf}",
		goal.target_pose.pose.position.x,
		goal.target_pose.pose.position.y
	);
	ac.sendGoal(goal);

	// wait for the results
	ac.waitForResult();

	// return true if the robot reached the goal
	return (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

// moves the robot to the given coordinate
bool moveToTarget(MoveBaseClient &ac, Coordinate coordinate) {
	move_base_msgs::MoveBaseGoal goal = createTargetGoal(coordinate);
	return sendTargetGoal(ac, goal);
}

// publishes the robot's given state
void updateState(RobotState state, ros::Publisher &statePublisher) {
	std_msgs::Int32 stateMessage;
	stateMessage.data = state;
	statePublisher.publish(stateMessage);
}

// program entry point
int main(int argc, char** argv) {
	// initialize the pick_objects node
	ros::init(argc, argv, "pick_objects");

	// get a handle to the node
	ros::NodeHandle n;

	// create the state publisher
	ros::Publisher statePublisher = n.advertise<std_msgs::Int32>("home_service_robot_state", 0);

	// create the move base client
	MoveBaseClient ac("move_base", true);

	// wait 5 sec for move_base action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	// move to the pickup location
	ROS_INFO(
		"Moving to pickup location: {x: %0.4lf, y: %0.4lf}",
		pickupCoordinate.x,
		pickupCoordinate.y
	);
	updateState(FINDING_PICKUP, statePublisher);
	if (moveToTarget(ac, pickupCoordinate)) {
		ROS_INFO("Successfully moved to pickup location");
	} else {
		ROS_INFO("Failed to move to pickup location");
		return 0;
	}

	// pickup the item
	ROS_INFO("Picking up Item");
	updateState(PICKING_UP, statePublisher);
	ros::spinOnce();
	ros::Duration(5.0).sleep();

	// move to the dropoff location
	ROS_INFO(
		"Moving to dropoff location: {x: %0.4lf, y: %0.4lf}",
		dropoffCoordinate.x,
		dropoffCoordinate.y
	);
	updateState(FINDING_DROPOFF, statePublisher);
	if (moveToTarget(ac, dropoffCoordinate)) {
		ROS_INFO("Successfully moved to dropoff location");
	} else {
		ROS_INFO("Failed to move to dropoff location");
		return 0;
	}

	// dropoff the item
	updateState(DROPPING_OFF, statePublisher);

	return 0;
}

