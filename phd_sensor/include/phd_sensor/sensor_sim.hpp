#ifndef SENSOR_SIM_HPP
#define SENSOR_SIM_HPP

#include <limits>
#include <math.h>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <phd_msgs/TargetArray.h>
#include <phd_sensor/sensor.hpp>

#include <boost/random.hpp>
#include <boost/generator_iterator.hpp>
#include <boost/scoped_ptr.hpp>

namespace phd {

template <class Measurement, class MArray>
class SensorSim {

public:
  SensorSim(Sensor<Measurement>* sensor) : n_(), pn_("~"), rand_(0),
      uniform_generator_(rand_, uniform_),
      gaussian_generator_(rand_, gaussian_),
      sensor_(sensor) {
    target_sub_ = n_.subscribe("target_array", 5, &SensorSim::targetCallback, this);
    map_sub_ = n_.subscribe("map", 1, &SensorSim::mapCallback, this);
    pose_sub_ = n_.subscribe("pose", 5, &SensorSim::poseCallback, this);

    measurement_pub_ = n_.advertise<MArray>("measurements", 1);

    flags_.have_pose = false;
    flags_.have_map = false;

    pn_.param("frame_id", frame_id_, std::string("base_link"));
    name_ = pn_.getNamespace();
    int i = name_.find_last_of("/");
    name_ = name_.substr(0, i);
  }

  static SensorSim* ROSInit(const ros::NodeHandle& n) {
    Sensor<Measurement>* sensor = Sensor<Measurement>::ROSInit(n);
    SensorSim* sim = new SensorSim(sensor);
    return sim;
  }

  Sensor<Measurement>* getSensor(void) { return sensor_.get(); }

  void poseCallback(const geometry_msgs::PoseStamped& msg) {
    pose_ = msg;
    flags_.have_pose = true;
  }

  void targetCallback(const phd_msgs::TargetArray& msg) {
    targets_ = msg;
  }

  void mapCallback(const nav_msgs::OccupancyGrid& map) {
    map_ = map;
    flags_.have_map = true;
  }

  /* This implementation is based on the method by Knuth. Perhaps inverse
   * transform sampling can be more efficient if an efficient form of the
   * Poisson CDF is used */
  int samplePoisson(double lambda) {
    double L = exp(-lambda);
    double p = uniform_generator_();
    int k = 1;
    while (p > L) {
      ++k;
      p *= uniform_generator_();
    }
    return --k;
  }

  virtual void spin(void);
  virtual Measurement addNoiseToMeasurement(const Measurement& z);
  virtual Measurement drawClutterMeasurement(const std::string& frame_id);
  virtual geometry_msgs::Pose drawPoseFromMeasurement(const Measurement& m,
    const geometry_msgs::Pose& p);

private:
  typedef boost::mt19937 RANDType;
  typedef boost::uniform_01<> Uniform;
  typedef boost::normal_distribution<> Gaussian;

  bool ready(void) {
    return flags_.have_pose && flags_.have_map;
  }

  // Class members
  ros::NodeHandle n_, pn_;
  ros::Subscriber pose_sub_;
  ros::Subscriber target_sub_;
  ros::Subscriber map_sub_;
  ros::Publisher measurement_pub_;
  geometry_msgs::PoseStamped pose_;
  phd_msgs::TargetArray targets_;
  nav_msgs::OccupancyGrid map_;

  std::string name_;
  std::string frame_id_;

  struct {
    bool have_pose, have_map;
  } flags_;

  RANDType rand_;
  Uniform uniform_;
  Gaussian gaussian_;
  boost::variate_generator<RANDType, Uniform > uniform_generator_;
  boost::variate_generator<RANDType, Gaussian > gaussian_generator_;

  boost::scoped_ptr< Sensor<Measurement> > sensor_;
};

} // end namespace

#endif
