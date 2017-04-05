#ifndef CAMERA_SENSOR_HPP
#define CAMERA_SENSOR_HPP

#include <cmath>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include <boost/scoped_ptr.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <phd_msgs/Camera.h>
#include <phd_sensor/sensor.hpp>
#include <tf/transform_datatypes.h>

#define EPS (1.0e-9)

namespace phd {

template <>
class Sensor<phd_msgs::Camera> {
public:
  struct Params {
    double fov_vertical;    // vertical field of view [rad]
    double fov_horizontal;  // horizontal field of view [rad]
    int width;              // image width in pixels
    int height;             // image height in pixels
    Eigen::Matrix2d cov;    // measurement covariance [pixels]
    double kappa;           // expected number of false positives

    // characterisitic length of the object
    // used to compute the detection probability
    double characteristic_length;
    // probability of detection as a function of the number of pixels
    // covering the object
    std::vector<std::pair<int, double> > det_prob;
  };
  typedef phd_msgs::Camera Measurement;

  Sensor(const Params& p) : p_(p),
      normalizer_(1.0 / (2 * M_PI * sqrt(p.cov.determinant()))),
      inv_cov_(p.cov.inverse()) { }

  static Sensor* ROSInit(const ros::NodeHandle& n) {
    ROS_ASSERT(n.hasParam("fov_vertical"));
    ROS_ASSERT(n.hasParam("fov_horizontal"));
    ROS_ASSERT(n.hasParam("width"));
    ROS_ASSERT(n.hasParam("height"));
    ROS_ASSERT(n.hasParam("std_dev_rr"));
    ROS_ASSERT(n.hasParam("std_dev_rc"));
    ROS_ASSERT(n.hasParam("std_dev_cc"));
    ROS_ASSERT(n.hasParam("kappa"));
    ROS_ASSERT(n.hasParam("characteristic_length"));
    ROS_ASSERT(n.hasParam("det_prob"));

    Sensor::Params p;
    n.getParam("fov_vertical", p.fov_vertical);
    n.getParam("fov_horizontal", p.fov_horizontal);
    n.getParam("width", p.width);
    n.getParam("height", p.height);
    n.getParam("std_dev_rr", p.cov(0,0)); p.cov(0,0) *= p.cov(0,0);
    n.getParam("std_dev_cc", p.cov(1,1)); p.cov(1,1) *= p.cov(1,1);
    n.getParam("std_dev_rc", p.cov(0,1)); p.cov(0,1) *= p.cov(0,1);
    p.cov(1,0) = p.cov(0,1);
    n.getParam("kappa", p.kappa);
    n.getParam("characteristic_length", p.characteristic_length);

    ROS_ASSERT(p.fov_vertical >= 0.0);
    ROS_ASSERT(p.fov_horizontal >= 0.0);
    ROS_ASSERT(p.width >= 0);
    ROS_ASSERT(p.height >= 0);
    ROS_ASSERT(p.cov.determinant() >= 0.0);
    ROS_ASSERT(p.kappa >= 0.0);

    XmlRpc::XmlRpcValue dp;
    n.getParam("det_prob", dp);
    ROS_ASSERT(dp.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(dp.size() > 1);
    p.det_prob.resize(dp.size());

    for (int i = 0; i < dp.size(); ++i) {
      ROS_ASSERT(dp[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
      ROS_ASSERT(dp[i].hasMember("pixels"));
      ROS_ASSERT(dp[i].hasMember("prob"));

      int pixels = (int) dp[i]["pixels"];
      double prob = (double) dp[i]["prob"];

      p.det_prob[i] = std::pair<double, double>(pixels, prob);
      if (i > 0) {
        ROS_ASSERT(p.det_prob[i-1].first < p.det_prob[i].first);
      }
      ROS_ASSERT(0.0 <= prob && prob <= 1.0);
    }

    Sensor* sensor = new Sensor(p);
    return sensor;
  }

  const Params& getParams(void) const { return p_; }
  double getClutterRate(void) const { return p_.kappa; }

  double detProb(const geometry_msgs::Pose& x) const {
    double ang_horizontal = atan2(x.position.x, x.position.z);
    double ang_vertical = atan2(x.position.y, x.position.z);

    // Check if within visible bounds of sensor
    if (-p_.fov_horizontal / 2.0 >= ang_horizontal ||
        ang_horizontal >= p_.fov_horizontal / 2.0) {
      return 0.0;
    }
    if (-p_.fov_vertical / 2.0 >= ang_vertical ||
        ang_vertical >= p_.fov_vertical / 2.0) {
      return 0.0;
    }

    // Find # pixels for object
    double pixel_width = 2.0 * x.position.z * tan(p_.fov_horizontal / 2.0) / static_cast<double>(p_.width);
    double pixel_height = 2.0 * x.position.z * tan(p_.fov_vertical / 2.0) / static_cast<double>(p_.height);
    double num_pixels = p_.characteristic_length / std::min(pixel_width, pixel_height);

    for (int i = 0; i < p_.det_prob.size()-1; ++i) {
      int n0 = p_.det_prob[i].first;
      int n1 = p_.det_prob[i+1].first;
      if (n0 <= num_pixels && num_pixels <= n1) {
        double frac = static_cast<double>(num_pixels - n0) / static_cast<double>(n1 - n0);
        double p0 = p_.det_prob[i].second;
        double p1 = p_.det_prob[i+1].second;
        return p0 + frac * (p1 - p0);
      }
    }
    // If larger than maximum pixels, return value at maximum pixels
    return p_.det_prob.back().second;
  }

  double measurementLikelihood(const Measurement& z,
                                       const geometry_msgs::Pose& x) const {
    Eigen::Vector2d z0, z1;
    z0(0) = z.row;
    z0(1) = z.col;
    poseToRowCol(x, &z1(0), &z1(1));
    double exponent = (z0 - z1).transpose() * inv_cov_ * (z0 - z1);
    return normalizer_ * exp(-exponent / 2.0);
  }

  double clutterLikelihood(const Measurement& z) const {
    return p_.kappa / static_cast<double>(p_.width * p_.height);
  }

  static geometry_msgs::Pose measurementToPose(const Measurement& m) {
    geometry_msgs::Pose p;

    double ang_horizontal = m.col * m.fov_horizontal / static_cast<double>(m.width);
    double ang_vertical = m.row * m.fov_vertical / static_cast<double>(m.height);

    p.position.x = tan(ang_horizontal - m.fov_horizontal / 2.0);
    p.position.y = tan(ang_vertical - m.fov_vertical / 2.0);
    p.position.z = 1.0;

    p.orientation = tf::createQuaternionMsgFromYaw(0.0);

    return p;
  }

  Measurement poseToMeasurement(const geometry_msgs::Pose& p, const std::string& frame_id) {
    Measurement z;

    z.header.stamp = ros::Time::now();
    z.header.frame_id = frame_id;
    z.height = p_.height;
    z.width = p_.width;
    z.fov_vertical = p_.fov_vertical;
    z.fov_horizontal = p_.fov_horizontal;

    double ang_horizontal = atan2(p.position.x, p.position.z) + z.fov_horizontal / 2.0;
    double ang_vertical = atan2(p.position.y, p.position.z) + z.fov_vertical / 2.0;

    z.row = ang_vertical / z.fov_vertical * static_cast<double>(z.height);
    z.col = ang_horizontal / z.fov_horizontal * static_cast<double>(z.width);

    return z;
  }

private:
  Params p_;
  double normalizer_;
  Eigen::Matrix2d inv_cov_;

  void poseToRowCol(const geometry_msgs::Pose& x, double* row, double* col) const {
    *col = xToCol(x.position.x, x.position.z);
    *row = yToRow(x.position.y, x.position.z);
  }

  double xToCol(double x, double z) const {
    double ang = atan2(x, z) + p_.fov_horizontal / 2.0;
    return ang / p_.fov_horizontal * static_cast<double>(p_.width);
  }

  double yToRow(double y, double z) const {
    double ang = atan2(y, z) + p_.fov_vertical / 2.0;
    return ang / p_.fov_vertical * static_cast<double>(p_.height);
  }
};

typedef Sensor<phd_msgs::Camera> CameraSensor;

} // end namespace

#endif
