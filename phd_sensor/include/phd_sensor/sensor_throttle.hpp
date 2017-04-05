
#ifndef SENSOR_THROTTLE_HPP
#define SENSOR_THROTTLE_HPP

#include <cmath>    // hypot
#include <limits>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

namespace phd {

template <class MArray>
class SensorThrottle {
public:
  struct Params {
    double min_step;  // minimum distance to travel between measurements
    double min_angle; // minimum angle to rotate between measurements
    double min_time;  // minimum time between measurements
    double max_time;  // maximum time between measurements
  };

  SensorThrottle(const ros::NodeHandle& nh) {
    nh.param("min_step", p_.min_step, 0.0);
    nh.param("min_angle", p_.min_angle, std::numeric_limits<double>::max());
    nh.param("min_time", p_.min_time, 0.0);
    nh.param("max_time", p_.max_time, std::numeric_limits<double>::max());

    last_pose_.header.stamp = ros::Time(0.0);
    last_pose_.pose.position.x = std::numeric_limits<double>::max();
    last_pose_.pose.position.y = std::numeric_limits<double>::max();
    last_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    measurement_sub_ = n_.subscribe("measurements", 10, &SensorThrottle::measurementCallback, this);
    measurement_pub_ = n_.advertise<MArray>("measurements_throttle", 10);
  }

private:
  void measurementCallback(const MArray& measurements);

  static double dist(const geometry_msgs::PoseStamped& p0,
                     const geometry_msgs::PoseStamped& p1) {
    return hypot(p0.pose.position.x - p1.pose.position.x,
                p0.pose.position.y - p1.pose.position.y);
  }

  static double angle(const geometry_msgs::PoseStamped& p0,
                      const geometry_msgs::PoseStamped& p1) {
    double diff = tf::getYaw(p0.pose.orientation) - tf::getYaw(p1.pose.orientation);
    while (diff > 2 * M_PI) {
      diff -= 2 * M_PI;
    }
    while (diff < -2 * M_PI) {
      diff += 2 * M_PI;
    }
    return fabs(diff);
  }

  Params p_;
  geometry_msgs::PoseStamped last_pose_;

  ros::NodeHandle n_;
  ros::Subscriber measurement_sub_;
  ros::Publisher measurement_pub_;
};

} // end namespace

#endif
