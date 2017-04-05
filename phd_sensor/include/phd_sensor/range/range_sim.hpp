#ifndef RANGE_SIM_HPP
#define RANGE_SIM_HPP

#include <phd_msgs/RangeArray.h>
#include <phd_sensor/sensor_sim.hpp>
#include <phd_sensor/range/range_sensor.hpp>

#include <tf/transform_datatypes.h>

namespace phd {

typedef SensorSim<sensor_msgs::Range, phd_msgs::RangeArray> RangeSim;

template<>
geometry_msgs::Pose RangeSim::drawPoseFromMeasurement(const sensor_msgs::Range& z,
    const geometry_msgs::Pose& p) {
  geometry_msgs::Pose x;

  double bearing = z.field_of_view / 2.0 * (uniform_generator_() - 0.5);

  x.position.x = z.range * cos(bearing);
  x.position.y = z.range * sin(bearing);
  x.position.z = 0.0;

  // Get transformation from sensor frame to global frame
  tf::Pose transform;
  tf::poseMsgToTF(p, transform);

  // Put pose in global frame
  tf::Vector3 v;
  tf::pointMsgToTF(x.position, v);
  v = transform * v;

  // Set marker pose
  tf::pointTFToMsg(v, x.position);
  x.orientation = tf::createQuaternionMsgFromYaw(0.0);

  return p;
}

template<>
sensor_msgs::Range RangeSim::addNoiseToMeasurement(const sensor_msgs::Range& z) {
  sensor_msgs::Range z_noisy(z);

  z_noisy.range += sensor_->getParams().std_dev * gaussian_generator_();

  return z_noisy;
}

template<>
sensor_msgs::Range RangeSim::drawClutterMeasurement(const std::string& frame_id) {
  sensor_msgs::Range z = sensor_->poseToMeasurement(pose_.pose, frame_id);

  z.range = z.min_range + (z.max_range - z.min_range) * uniform_generator_();

  return z;
}

template <>
void RangeSim::spin() {
  flags_.have_map = true;

  if (!ready()) {
    return;
  }

  phd_msgs::RangeArray msg;
  msg.pose = pose_;
  msg.child_frame_id = name_ + std::string("/") + frame_id_;

  tf::Pose transform;
  tf::poseMsgToTF(msg.pose.pose, transform);
  transform = transform.inverse();

  // Add true measurements
  for (std::vector<phd_msgs::Target>::iterator it = targets_.poses.begin();
       it != targets_.poses.end(); ++it) {
    tf::Pose p;
    tf::poseMsgToTF(it->pose, p);

    geometry_msgs::Pose x;
    tf::poseTFToMsg(transform * p, x);

    if (uniform_generator_() < sensor_->detProb(x)) {
      sensor_msgs::Range m = sensor_->poseToMeasurement(x, msg.child_frame_id);
      msg.array.push_back(addNoiseToMeasurement(m));
    }
  }

  // Add false positives
  for (int i = 0; i < samplePoisson(sensor_->getClutterRate()); ++i) {
    sensor_msgs::Range m = drawClutterMeasurement(msg.child_frame_id);
    msg.array.push_back(m);
  }

  measurement_pub_.publish(msg);
}

} // end namespace

#endif
