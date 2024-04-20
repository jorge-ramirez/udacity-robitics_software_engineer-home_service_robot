#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
 
// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Define the robots positions to move to
std::vector<std::vector<float>> target_positions = {
    {1.0, 1.0},
    {20.0, 20.0},
};

move_base_msgs::MoveBaseGoal createTargetGoal(std::vector<float> &coordinates) {
  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  
  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = coordinates.at(0);
  goal.target_pose.pose.orientation.w = coordinates.at(1);
  
  return goal;
}

bool sendTargetGoal(MoveBaseClient &ac, move_base_msgs::MoveBaseGoal &goal) {
  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  
  // Wait an infinite time for the results
  ac.waitForResult();
  
  // Check if the robot reached its goal
  return (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

bool moveToTarget(MoveBaseClient &ac, std::vector<float> &coordinates) {
  move_base_msgs::MoveBaseGoal goal = createTargetGoal(coordinates);
  sendTargetGoal(ac, goal);
}

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  for(int i = 0; i < target_positions.size(); i++) {
    std::vector<float> coordinates = target_positions.at(i);
    ROS_INFO("Moving to position");
    
    if (moveToTarget(ac, coordinates)) {
      ROS_INFO("Successfully moved to position");
      ros::Duration(5.0).sleep(); // "pickup" object
    } else {
      ROS_INFO("Failed to move to position");
      break;
    }
  }

  return 0;
}

