// -- ROS -- 
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>

#include "phd_msgs/BearingArray.h"
#include "ffld_ros/ObjectDetection.h"

//double degtorad(double deg){ return (M_PI / 180.0) * deg; }
//double radtodeg(double rad){ return (180.0 / M_PI) * rad; }

struct Params
{
  // initialize focal lengths and center points
  double fx = -1;//1474.041628032043;
  double fy = -1;//1479.139972160672;
  double cx = -1;//1050.910712018828;
  double cy = -1;//770.7633865910121;
  double min_rng = 0.2;
  double max_rng = 15;
  double min_ang = -0.609477; //-0.820305;
  double max_ang = 0.618911; //0.820305;
  
  std::string sensor_frame_id;
  geometry_msgs::PoseStamped currPose;
  bool pose_initialized=false;
  ros::Publisher bearing_pub;
} pr_;

void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg)
{
  pr_.fx = msg->K[0];
  pr_.cx = msg->K[2];
  pr_.fy = msg->K[4];
  pr_.cy = msg->K[5];
  pr_.min_ang = std::atan( (pr_.cx-msg->width) / pr_.fx );
  pr_.max_ang = std::atan( (pr_.cx-1) / pr_.fx );
  //std::cout << pr_.min_ang << ' ' << pr_.max_ang << std::endl;
}

void poseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  pr_.currPose.header = msg->header;
  pr_.currPose.pose = (msg->pose).pose; // save current pose
  pr_.pose_initialized = true;
}

void detectionCallback(const ffld_ros::ObjectDetection::ConstPtr &msg)
{
  if( pr_.fx == -1 || !pr_.pose_initialized )
    return; // we don't have camera info or pose yet
  
  phd_msgs::BearingArray bearing_array;
  bearing_array.child_frame_id = pr_.sensor_frame_id;
  bearing_array.pose = pr_.currPose;
  
  // Convert ObjectDetction to BearingArray
  unsigned int num_det = msg->left.size();
  bearing_array.array.resize(num_det);
  for(unsigned int k = 0; k < num_det; ++k)
  {
    bearing_array.array[k].header = msg->header;
    double centerx = (msg->right[k] + msg->left[k]) / 2;
    bearing_array.array[k].bearing = std::atan( (pr_.cx-centerx) / pr_.fx );
    bearing_array.array[k].min_range = pr_.min_rng;
    bearing_array.array[k].max_range = pr_.max_rng;
    bearing_array.array[k].min_bearing = pr_.min_ang;
    bearing_array.array[k].max_bearing = pr_.max_ang;
  }
  
  // Publish bearings
  pr_.bearing_pub.publish( bearing_array );
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detection_converter");
  
  // use this to read in parameters
  ros::NodeHandle private_nh("~"); 
  private_nh.param<std::string>("sensor_frame_id", pr_.sensor_frame_id, "/scarab0/base");
  
  ros::NodeHandle nh;
  ros::Subscriber camera_sub = nh.subscribe( "camera_info", 1, &cameraInfoCallback );
  ros::Subscriber pose_sub = nh.subscribe( "pose", 1, &poseCallback );
  ros::Subscriber det_sub = nh.subscribe( "object_detections", 1, &detectionCallback );
  pr_.bearing_pub = nh.advertise<phd_msgs::BearingArray>("measurements", 100);
  
  ROS_INFO("Converting object detections to bearings...");
  ros::spin();
  return 0;
}


