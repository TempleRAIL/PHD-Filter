#include "phd_msgs/Bearing.h"
#include "phd_msgs/BearingArray.h"
#include "ros/ros.h"
#include "ffld_ros/ObjectDetection.h"
#include <string>

struct Params {
  std::string frame_id;
} pr;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
	//std::cout << msg->pose << std::endl;
}

void detectionCallback(const ffld_ros::ObjectDetection::ConstPtr &msg, 
	const ros::Publisher &pub) {
  
  // initialize detections size
  int size;
  size = msg->score.size();

  // initialize focal lengths and center points
  double fx = 1474.041628032043;
  //double fy = 1479.139972160672;
  double cx = 1050.910712018828;
  //double cy = 770.7633865910121;

  // initialize Bearing and BearingArray
  phd_msgs::BearingArray bearing_array;
  phd_msgs::Bearing bearings[size];

  // initialize centers and bearing
  uint centersx[size];
  float bearing_xy[size];

  for (int i = 0; i < size; ++i) {
    /*std::cout << msg->score[i] << ' ' << msg->left[i] << ' '
        << msg->top[i] << ' ' << msg->right[i] << ' '
        << msg->bottom[i] << std::endl;*/
    
    // Set array to empty when no detections
    if (size == 0) {
    	//bearing_array.array[i] = null;
    }

    // calculate bearing
    centersx[i] = (msg->right[i] + msg->left[i]) / 2;
    bearing_xy[i] = atan((cx - centersx[i])/fx);

    // set conversions to bearing message
    bearings[i].header = msg->header;
    bearings[i].bearing = bearing_xy[i];
    //std::cout << bearing_xy[i] << ' ' << bearings[i].bearing << std::endl;
    
    // temporary values
    bearings[i].min_range = 0.2;
    bearings[i].max_range = 15;
    bearings[i].min_bearing = -135.0/180;
    bearings[i].max_bearing = 135.0/180;

    // add bearings to bearing array
    bearing_array.array.push_back(bearings[i]);
    bearing_array.child_frame_id = pr.frame_id;
    //std::cout << bearing_array.array[i] << std::endl;
  }

  //pub_pose.publish(bearing_array.pose);
  //bearing_array.pose = pose;
  pub.publish(bearing_array); // publish bearing array

}

int main(int argc, char **argv) {
  
  ros::init(argc, argv, "detection_converter");
  ros::NodeHandle n;

  // Set child_frame_id as param
  n.param<std::string>("child_frame_id", pr.frame_id, "/scarab0/base"); //laser_odom

  ros::Publisher pub_bearings = n.advertise<phd_msgs::BearingArray>("measurements", 1000);
  ros::Subscriber sub_pose = n.subscribe<geometry_msgs::PoseStamped>("pose", 1000, poseCallback);
  //ros::Publisher pub_pose = n.advertise<geometry_msgs::PoseStamped>("pub_pose", 1000);
  ros::Subscriber sub = n.subscribe<ffld_ros::ObjectDetection>("object_detections", 1000, 
  	boost::bind(detectionCallback, _1, pub_bearings));

  ROS_INFO("Converting detections to bearing...");
  ros::spin();
  return 0;
}