#ifndef RANGE_BEARING_SIM_HPP
#define RANGE_BEARING_SIM_HPP

#include <phd_msgs/RangeBearingArray.h>
#include <phd_sensor/range_bearing/range_bearing_sensor.hpp>
#include <phd_sensor/sensor_sim.hpp>

#include <tf/transform_datatypes.h>

#include <unsupported/Eigen/MatrixFunctions>  // matrix sqrt

namespace phd {

typedef SensorSim<phd_msgs::RangeBearing, phd_msgs::RangeBearingArray> RangeBearingSim;

template<>
geometry_msgs::Pose RangeBearingSim::drawPoseFromMeasurement(const phd_msgs::RangeBearing& z,
		const geometry_msgs::Pose& p) {
	geometry_msgs::Pose x;

  x.position.x = z.range * cos(z.bearing);
  x.position.y = z.range * sin(z.bearing);
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
phd_msgs::RangeBearing RangeBearingSim::addNoiseToMeasurement(const phd_msgs::RangeBearing& z) {
	phd_msgs::RangeBearing z_noisy(z);

	Eigen::Vector2d noise;
	noise(0) = gaussian_generator_();
	noise(1) = gaussian_generator_();
	noise = sensor_->getParams().cov.sqrt() * noise;
	z_noisy.range += noise(1);
	z_noisy.bearing += noise(0);

	return z_noisy;
}

template<>
phd_msgs::RangeBearing RangeBearingSim::drawClutterMeasurement(const std::string& frame_id) {
	phd_msgs::RangeBearing z = sensor_->poseToMeasurement(pose_.pose, frame_id);

	z.range = z.min_range + (z.max_range - z.min_range) * uniform_generator_();
	z.bearing = z.min_bearing + (z.max_bearing - z.min_bearing) * uniform_generator_();

	return z;
}

template <>
void RangeBearingSim::spin() {
  flags_.have_map = sensor_->hasMap();

  if (!ready()) {
    return;
  }

  phd_msgs::RangeBearingArray msg;
  msg.pose = pose_;
  msg.child_frame_id = name_ + std::string("/") + frame_id_;

  sensor_->setPose(pose_.pose);

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
			phd_msgs::RangeBearing m = sensor_->poseToMeasurement(x, msg.child_frame_id);
      msg.array.push_back(addNoiseToMeasurement(m));
    }
  }

  // Add false positives
  for (int i = 0; i < samplePoisson(sensor_->getClutterRate()); ++i) {
		phd_msgs::RangeBearing m = drawClutterMeasurement(msg.child_frame_id);
    msg.array.push_back(m);
  }

  measurement_pub_.publish(msg);
}

template <>
void RangeBearingSim::mapCallback(const nav_msgs::OccupancyGrid& map) {
  map_ = map;
  sensor_->setMap(map);
}

} // end namespace

#endif
