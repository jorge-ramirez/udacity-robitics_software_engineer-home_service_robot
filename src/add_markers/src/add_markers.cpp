#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <cmath>

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

enum RobotState {
  FINDING_PICKUP,
  PICKING_UP,
  FINDING_DROPOFF,
  DROPPING_OFF
};

RobotState currentState = FINDING_PICKUP;

Coordinate pickupCoordinate = Coordinate(-2.0, 4.5);
Coordinate dropoffCoordinate = Coordinate(5.0, 2.5);

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

void state_callback(const std_msgs::Int32::ConstPtr& msg){
  currentState = (RobotState) msg->data;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "add_markers");
  
  ros::NodeHandle n;
  ros::Rate r(1.0);
  ros::Publisher markerPub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber stateSub = n.subscribe("home_service_robot_state", 1, state_callback);
  
  // create the marker
  visualization_msgs::Marker marker = createMarker(visualization_msgs::Marker::CUBE);
  
  while (ros::ok()) {
    // wait for subscribers
    while (markerPub.getNumSubscribers() < 1) {
      if (!ros::ok()) {
        return 0;
      }
      
      ROS_WARN_ONCE("Waiting for marker subscribers");
      sleep(1);
    }
    
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
    
    markerPub.publish(marker);
    ros::spinOnce();
  }

  return 0;
}

