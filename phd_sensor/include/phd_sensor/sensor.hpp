#ifndef SENSOR_HPP
#define SENSOR_HPP

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

namespace phd {

template <class Measurement>
class Sensor {
public:
  struct Params {};
  static Sensor* ROSInit(const ros::NodeHandle& n);
  virtual const Params& getParams(void) = 0;
  virtual double detProb(const geometry_msgs::Pose& x) const = 0;
  virtual double measurementLikelihood(const Measurement& z, const geometry_msgs::Pose& x) const = 0;
  virtual double clutterLikelihood(const Measurement& z) const = 0;
  virtual double getClutterRate(void) const = 0;
  virtual Measurement poseToMeasurement(const geometry_msgs::Pose& p, const std::string& frame_id) = 0;
};

} // end namespace

#endif
