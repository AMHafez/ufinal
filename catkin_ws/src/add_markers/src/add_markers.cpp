
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <nav_msgs/Odometry.h>   
#include <math.h>

using namespace std;

const double pickup_x = -3.38;
const double pickup_y = -1.8;
const double dropOff_x = -0.053;
const double dropOff_y =  -3.36;

visualization_msgs::Marker marker;
bool pickup = false;
bool dropoff = false;
double Distance = 0.3; 
bool Done = false;


void InitializeMarker(void){
  
  uint32_t shape = visualization_msgs::Marker::CUBE;
  
    
    marker.header.frame_id = "map"; 
    marker.header.stamp = ros::Time::now();
    
    marker.ns = "add_markers";
    marker.id = 0;
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE
    marker.action = visualization_msgs::Marker::ADD;

    // Set the SIZE of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.4;
}


void PickupMarker(void){
    marker.pose.position.x = pickup_x; 
    marker.pose.position.y = pickup_y; 
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0; 
    
    marker.color.r = 0.0f; marker.color.g = 1.0f; marker.color.b = 0.0f;
    marker.lifetime = ros::Duration();
}


void DropOffMarker(void){
 
    marker.pose.position.x = dropOff_x; 
    marker.pose.position.y = dropOff_y; 
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
   
    marker.color.r = 0.0f; marker.color.g = 1.0f; marker.color.b = 0.0f;
}

double distance(double Marker_x1, double Marker_y1, double Robot_x2, double Robot_y2)
{
  return sqrt(pow((Marker_x1 - Robot_x2), 2.0) + pow((Marker_y1 - Robot_y2), 2.0));
}


void OdomCheck(const nav_msgs::Odometry &odom_msg){
  double position_x = odom_msg.pose.pose.position.y;
  double position_y = -odom_msg.pose.pose.position.x; 
  double dist1, dist2;
  
  dist1 = distance(position_x, position_y, pickup_x, pickup_y);
  dist2 = distance(position_x, position_y, dropOff_x, dropOff_y);
  
  pickup = false;
  dropoff = false;
  if (dist1 < Distance){
   pickup = true;
   
  }
  if (dist2 < Distance){
    dropoff = true;
  
  }
}

int main( int argc, char** argv ){

  ros::init(argc, argv, "add_markers"); 
  ros::NodeHandle n;

  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  ros::Subscriber odom_sub = n.subscribe("odom", 2, OdomCheck);

  ROS_INFO("Pickup  : x=%5.2f , y=%5.2f", pickup_x, pickup_y);
  ROS_INFO("Dropoff : x=%5.2f , y=%5.2f", dropOff_x, dropOff_y);

  ros::Rate rate(2);

  while (ros::ok()){
    InitializeMarker();
    while (marker_pub.getNumSubscribers() < 1){
      if (!ros::ok()){
        return 0;
      }
     ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    
    ROS_INFO("---------------");
    ROS_INFO("Displaying pickup goal");
    PickupMarker();
    marker.color.a = 1.0;
    marker_pub.publish(marker);   
    ROS_INFO("Waiting for robot to reach pickup goal");
    while (!pickup){
       sleep(2);
      ros::spinOnce(); 
    }
    ROS_INFO("Robot reached pickup goal");
    marker.color.a = 0.0; marker_pub.publish(marker); 
    ros::Duration(5.0).sleep(); 
    
    while (!dropoff){
	
        ros::spinOnce();
    }
    ROS_INFO("------");
    ROS_INFO("Displaying drop off goal");
    DropOffMarker();
    marker.color.a = 1.0; marker_pub.publish(marker); 
    
    Done = true;        
    if (Done){
      ROS_INFO("Done");
      ros::Duration(5.0).sleep();
      break; 
    } 
    rate.sleep();
  } 
  return 0;
} 

