#ifndef phd_sensor_HPP
#define phd_sensor_HPP

#include <utility>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <phd_sensor/sensor.hpp>
#include <sensor_msgs/Range.h>
#include <tf/transform_datatypes.h>

#define EPS (1.0e-9)

namespace phd {

template <>
class Sensor<sensor_msgs::Range> {
public:
  struct Params {
    double range_max; // sensor max range
    double range_min; // sensor min range
    double std_dev;   // measurement noise
    double kappa;     // expected number of false positives

    // probability of detection, in (range, probability) pairs
    // these points are linearly interpolated
    std::vector<std::pair<double, double> > det_prob;
  };
  typedef sensor_msgs::Range Measurement;

  Sensor(const Params& p) : p_(p),
    normalizer_(1.0 / (sqrt(2 * M_PI) * p.std_dev)) {
  }

  static Sensor* ROSInit(const ros::NodeHandle& n) {
    ROS_ASSERT(n.hasParam("range_min"));
    ROS_ASSERT(n.hasParam("range_max"));
    ROS_ASSERT(n.hasParam("std_dev"));
    ROS_ASSERT(n.hasParam("det_prob"));
    ROS_ASSERT(n.hasParam("kappa"));

    Sensor::Params p;
    n.getParam("range_max", p.range_max);
    n.getParam("range_min", p.range_min);
    n.getParam("std_dev", p.std_dev);
    n.getParam("kappa", p.kappa);

    ROS_ASSERT(p.range_max >= p.range_min);
    ROS_ASSERT(p.std_dev >= 0.0);
    ROS_ASSERT(p.kappa >= 0.0);

    XmlRpc::XmlRpcValue dp;
    n.getParam("det_prob", dp);
    ROS_ASSERT(dp.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(dp.size() > 1);
    p.det_prob.resize(dp.size());

    for (int i = 0; i < dp.size(); ++i) {
      ROS_ASSERT(dp[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
      ROS_ASSERT(dp[i].hasMember("range"));
      ROS_ASSERT(dp[i].hasMember("prob"));

      double range = static_cast<double>(dp[i]["range"]);
      double prob = static_cast<double>(dp[i]["prob"]);

      p.det_prob[i] = std::pair<double, double>(range, prob);
      if (i > 0) {
        ROS_ASSERT(p.det_prob[i-1].first < p.det_prob[i].first);
      }
      ROS_ASSERT(0.0 <= prob && prob <= 1.0);
    }
    ROS_ASSERT( fabs(p.det_prob.front().first - p.range_min) < EPS );
    ROS_ASSERT( fabs(p.det_prob.back().first - p.range_max) < EPS );

    Sensor* sensor = new Sensor(p);
    return sensor;
  }

  const Params& getParams(void) { return p_; }
  double getClutterRate(void) const { return p_.kappa; }

  double detProb(const geometry_msgs::Pose& x) const {
    double range = hypot(x.position.x, x.position.y);
    if (p_.range_min >= range || range >= p_.range_max) {
      return 0.0;
    }
    for (int i = 0; i < p_.det_prob.size()-1; ++i) {
      double r0 = p_.det_prob[i].first;
      double r1 = p_.det_prob[i+1].first;
      if (r0 <= range && range <= r1) {
        double frac = (range - r0) / (r1 - r0);
        double p0 = p_.det_prob[i].second;
        double p1 = p_.det_prob[i+1].second;
        return p0 + frac * (p1 - p0);
      }
    }
    ROS_WARN("%s: Range not specified", ros::this_node::getName().c_str());
    return 0.0;
  }

  double measurementLikelihood(const Measurement& z,
                                       const geometry_msgs::Pose& x) const {
    double z0 = static_cast<double>(z.range);
    double z1 = hypot(x.position.x, x.position.y);
    // Todo: implement this as a lookup table
    return normalizer_ * exp(-(z0 - z1)*(z0 - z1) / (2.0 * p_.std_dev*p_.std_dev) );
  }

  double clutterLikelihood(const Measurement& z) const {
    return p_.kappa / (p_.range_max - p_.range_min);
  }

  Measurement poseToMeasurement(const geometry_msgs::Pose& p,
      const std::string& frame_id) const {
    Measurement z;
    z.header.stamp = ros::Time::now();
    z.header.frame_id = frame_id;

    z.min_range = p_.range_min;
    z.max_range = p_.range_max;
    z.field_of_view = 2.0 * M_PI;

    z.range = hypot(p.position.x, p.position.y);

    return z;
  }

private:
  Params p_;
  double normalizer_;
};

typedef Sensor<sensor_msgs::Range> RangeSensor;

} // end namespace

#endif
