#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <cmath>
#include <std_msgs/Int32.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Define a type to hold a coordinate
class Coordinate {
public:
  double x;
  double y;
  
  Coordinate(int x, int y) {
    this->x = x;
    this->y = y;
  }
};

// Define the robots coordinates to move to
Coordinate pickupCoordinate = Coordinate(-2.0, 4.5);
Coordinate dropoffCoordinate = Coordinate(5.0, 2.5);

// Define a type to advertise the robot state
enum RobotState {
  FINDING_PICKUP,
  PICKING_UP,
  FINDING_DROPOFF,
  DROPPING_OFF
};

move_base_msgs::MoveBaseGoal createTargetGoal(Coordinate coordinate) {
  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  
  // Define a position for the robot to reach
  goal.target_pose.pose.position.x = coordinate.x;
  goal.target_pose.pose.position.y = coordinate.y;
  goal.target_pose.pose.position.z = 1.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;
  
  return goal;
}

bool sendTargetGoal(MoveBaseClient &ac, move_base_msgs::MoveBaseGoal &goal) {
  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal: {x: %0.4lf, y: %0.4lf}", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
  ac.sendGoal(goal);
  
  // Wait an infinite time for the results
  ac.waitForResult();
  
  // Check if the robot reached its goal
  return (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

bool moveToTarget(MoveBaseClient &ac, Coordinate coordinate) {
  move_base_msgs::MoveBaseGoal goal = createTargetGoal(coordinate);
  return sendTargetGoal(ac, goal);
}

void updateState(RobotState newState, ros::Publisher &statePub) {
  std_msgs::Int32 stateMessage;
  stateMessage.data = newState;
  statePub.publish(stateMessage);
}

int main(int argc, char** argv) {
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;
  
  // Define the state publisher
  ros::Publisher statePub = n.advertise<std_msgs::Int32>("home_service_robot_state", 0);

  // tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ROS_INFO("Moving to pickupCoordinate: {x: %0.4lf, y: %0.4lf}", pickupCoordinate.x, pickupCoordinate.y);
  updateState(FINDING_PICKUP, statePub);
  if (moveToTarget(ac, pickupCoordinate)) {
    ROS_INFO("Successfully moved to pickupCoordinate");
  } else {
    ROS_INFO("Failed to move to pickupCoordinate");
    return 0;
  }
  
  ROS_INFO("Picking up Item");
  updateState(PICKING_UP, statePub);
  ros::spinOnce();
  ros::Duration(5.0).sleep();

  ROS_INFO("Moving to dropOffCoordinate: {x: %0.4lf, y: %0.4lf}", dropoffCoordinate.x, dropoffCoordinate.y);
  updateState(FINDING_DROPOFF, statePub);
  if (moveToTarget(ac, dropoffCoordinate)) {
    ROS_INFO("Successfully moved to dropOffCoordinate");
  } else {
    ROS_INFO("Failed to move to dropOffCoordinate");
    return 0;
  }
  
  updateState(DROPPING_OFF, statePub);

  return 0;
}

