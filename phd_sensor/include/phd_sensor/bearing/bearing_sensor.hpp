#ifndef BEARING_SENSOR_HPP
#define BEARING_SENSOR_HPP

#include <utility>
#include <vector>

#include <boost/scoped_ptr.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <player_map/rosmap.hpp>
#include <phd_msgs/Bearing.h>
#include <phd_sensor/sensor.hpp>
#include <tf/transform_datatypes.h>

#define EPS (1.0e-9)

namespace phd {

template <>
class Sensor<phd_msgs::Bearing> {
public:
  struct Params {
    double bearing_max;         // sensor max bearing
    double bearing_min;         // sensor min bearing
    double range_max;           // sensor max range
    double range_min;           // sensor min range
    double std_dev;             // measurement noise
    double kappa;               // expected number of false positives
    double clutter_bin_width;   // width of peaks centered on +/- pi/2
    double clutter_bin_weight;  // fraction of weight of peaks (vs. uniform)

    // probability of detection, in (range, probability) pairs
    // these points are linearly interpolated
    std::vector<std::pair<double, double> > det_prob;
    std::string map_service;
  };
  typedef phd_msgs::Bearing Measurement;

  Sensor(const Params& p) : p_(p),
    normalizer_(1.0 / (sqrt(2.0 * M_PI) * p.std_dev)), map_(NULL) {
    map_.reset(scarab::OccupancyMap::FromMapServer(p_.map_service.c_str()));
    map_->updateCSpace(0.5, 0.2);
    pose_.orientation.w = 1.0;
  }

  static Sensor* ROSInit(const ros::NodeHandle& n) {
    ROS_ASSERT(n.hasParam("bearing_max"));
    ROS_ASSERT(n.hasParam("bearing_min"));
    ROS_ASSERT(n.hasParam("range_max"));
    ROS_ASSERT(n.hasParam("range_min"));
    ROS_ASSERT(n.hasParam("std_dev"));
    ROS_ASSERT(n.hasParam("det_prob"));
    ROS_ASSERT(n.hasParam("kappa"));
    ROS_ASSERT(n.hasParam("map_service"));

    Sensor::Params p;
    n.getParam("bearing_max", p.bearing_max);
    n.getParam("bearing_min", p.bearing_min);
    n.getParam("range_max", p.range_max);
    n.getParam("range_min", p.range_min);
    n.getParam("std_dev", p.std_dev);
    n.getParam("kappa", p.kappa);
    n.param("clutter_bin_width", p.clutter_bin_width, 0.0);
    n.param("clutter_bin_weight", p.clutter_bin_weight, 0.0);
    n.param("map_service", p.map_service, std::string("/static_map"));

    ROS_ASSERT(p.bearing_max >= p.bearing_min);
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

      double range = (double) dp[i]["range"];
      double prob = (double) dp[i]["prob"];

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

  const Params& getParams(void) const { return p_; }
  double getClutterRate(void) const { return p_.kappa; }

  double detProb(const geometry_msgs::Pose& x) const {
    // Check if within visible bounds of sensor
    double bearing = atan2(x.position.y, x.position.x);
    if (p_.bearing_min >= bearing || bearing >= p_.bearing_max) {
      return 0.0;
    }
    double range = hypot(x.position.x, x.position.y);
    if (p_.range_min >= range || range >= p_.range_max) {
      return 0.0;
    }
    // Check if line-of-sight
    double theta = tf::getYaw(pose_.orientation);
    double c = cos(theta), s = sin(theta);
    geometry_msgs::Pose p;
    p.position.x = c * x.position.x - s * x.position.y + pose_.position.x;
    p.position.y = s * x.position.x + c * x.position.y + pose_.position.y;
    if (!map_->lineOfSight(pose_.position.x, pose_.position.y,
                           p.position.x, p.position.y)) {
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
    ROS_WARN("%s: Bearing not specified", ros::this_node::getName().c_str());
    return 0.0;
  }

  double measurementLikelihood(const Measurement& z,
                                       const geometry_msgs::Pose& x) const {
    double z0 = static_cast<double>(z.bearing);
    double z1 = atan2(x.position.y, x.position.x);
    // Todo: implement this as a lookup table
    return normalizer_ * exp(-(z0 - z1)*(z0 - z1) / (2.0 * p_.std_dev*p_.std_dev) );
  }

  double clutterLikelihood(const Measurement& z) const {
    double bearing = static_cast<double>(z.bearing);

    double p = (1.0 - p_.clutter_bin_weight) / (p_.bearing_max - p_.bearing_min);
    if (fabs(fabs(bearing) - M_PI/2) < p_.clutter_bin_width / 2.0) {
      p += p_.clutter_bin_weight / (2.0 * p_.clutter_bin_width);
    }

    return p_.kappa * p;
  }

  Measurement poseToMeasurement(const geometry_msgs::Pose& p,
			const std::string& frame_id) const {
		Measurement z;
		z.header.stamp = ros::Time::now();
		z.header.frame_id = frame_id;

		z.min_range = p_.range_min;
		z.max_range = p_.range_max;
		z.min_bearing = p_.bearing_min;
		z.max_bearing = p_.bearing_max;

		z.bearing = atan2(p.position.y, p.position.x);

		return z;
	}

  void setPose(const geometry_msgs::Pose& p) { pose_ = p; }

  void setMap(const nav_msgs::OccupancyGrid& map) {
    map_->setMap(map);
    map_->updateCSpace(0.5, 0.2);
  }

  bool hasMap(void) {
    return map_.get() != NULL;
  }

private:
  Params p_;
  double normalizer_;
  boost::scoped_ptr<scarab::OccupancyMap> map_;
  geometry_msgs::Pose pose_;
};

typedef Sensor<phd_msgs::Bearing> BearingSensor;

} // end namespace

#endif
