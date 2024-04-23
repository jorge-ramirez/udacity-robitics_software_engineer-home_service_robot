#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

// Define a type to hold a coordinate
class Coordinate {
public:
  double x;
  double y;
  
  Coordinate(int x, int y) {
    this->x = x;
    this->y = y;
  }

  double distanceTo(Coordinate coord) {
    return sqrt(pow(this->x - coord.x, 2) + pow(this->y - coord.y, 2));
  }
};

// Define the robots coordinates to move to
Coordinate targetCoordinates[2] = {
  Coordinate(-2.0, 4.5),
  Coordinate(5.0, 2.5)
};

Coordinate odometryCoordinate = Coordinate(0.0, 0.0);

visualization_msgs::Marker createMarker(uint32_t shape, Coordinate coordinate) {
  visualization_msgs::Marker marker;
  
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "add_markers";
  marker.id = 0;
  marker.type = shape;

  marker.pose.position.x = coordinate.x;
  marker.pose.position.y = coordinate.y;
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

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odometryCoordinate = Coordinate(msg->pose.pose.position.x, msg->pose.pose.position.y);
  //ROS_INFO("received odometry: {x: %0.4lf, y: %0.4lf}", msg->pose.pose.position.x, msg->pose.pose.position.y);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "add_markers");
  
  double tolerance = 0.5;
  bool hasFoundFirstItem = false;
  ros::NodeHandle n;
  ros::Rate r(1.0);
  ros::Publisher markerPub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odometrySub = n.subscribe("/odom", 1000, odometryCallback);
  
  // create the marker
  visualization_msgs::Marker marker = createMarker(visualization_msgs::Marker::CUBE, targetCoordinates[0]);
  
  while(ros::ok()) {
    if (!hasFoundFirstItem) {
      // show the first item marker
      markerPub.publish(marker);
      
      ROS_INFO("distance from: {x: %0.4lf, y: %0.4lf} to {x: %0.4lf, y: %0.4lf} = %0.4lf", odometryCoordinate.x, odometryCoordinate.y, targetCoordinates[0].x, targetCoordinates[0].y, odometryCoordinate.distanceTo(targetCoordinates[0]));
      if (odometryCoordinate.distanceTo(targetCoordinates[0]) < tolerance) {
        hasFoundFirstItem = true;
        
        // hide the first item marker when it is found
        marker.action = visualization_msgs::Marker::DELETE;
        markerPub.publish(marker);
        
        //ros::spinOnce();
        //ros::Duration(5.0).sleep(); // "pickup" object
      }
    } else {
      // show the second item marker when it is found
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = targetCoordinates[1].x;
      marker.pose.position.y = targetCoordinates[1].y;
      markerPub.publish(marker);
    
      if (odometryCoordinate.distanceTo(targetCoordinates[1]) < tolerance) {
        // hide the second item marker when it is found
        marker.action = visualization_msgs::Marker::DELETE;
        markerPub.publish(marker);
      }
    }
    
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}

