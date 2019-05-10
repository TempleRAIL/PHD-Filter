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
phd_msgs::Target RangeBearingSim::drawTargetFromMeasurement(const phd_msgs::RangeBearing& z,
		const geometry_msgs::Pose& p) {
	phd_msgs::Target t;

	double range = z.min_bearing + (z.max_bearing - z.min_bearing) * uniform_generator_();

	t.pose.position.x = range * cos(z.bearing);
	t.pose.position.y = range * sin(z.bearing);
	t.pose.position.z = 0.0;

	// Get transformation from sensor frame to global frame
	tf::Pose transform;
	tf::poseMsgToTF(p, transform);

	// Put pose in global frame
	tf::Vector3 v;
	tf::pointMsgToTF(t.pose.position, v);
	v = transform * v;

	// Set marker pose
	tf::pointTFToMsg(v, t.pose.position);
	t.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
	t.type = z.type;

	return t;
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
	
	// Add noise to class measurement
	const std::map<std::string, double>& cm = sensor_->getParams().confusion_matrix.at(z.type);
	
	double total = 0.0;
	double rand = uniform_generator_();
	for (const auto& c : cm) {
		total += c.second;
		if (rand < total) {
			z_noisy.type = c.first;
			break;
		}
	}

	return z_noisy;
}

template<>
phd_msgs::RangeBearing RangeBearingSim::drawClutterMeasurement(const std::string& frame_id) {
	phd_msgs::Target t;
	t.pose = pose_.pose;
	
	// Draw random class uniformly
	const Sensor<phd_msgs::RangeBearing>::Params& par = sensor_->getParams();
	std::map<std::string, std::map<std::string, double> >::const_iterator it = par.confusion_matrix.begin();
	std::advance(it, int_generator_(rand_));
	t.type = it->first;
	
	phd_msgs::RangeBearing z = sensor_->poseToMeasurement(t, frame_id);

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
  for (const auto& t: targets_.array) {
		phd_msgs::Target tar(t);
		
		tf::Pose p;
		tf::poseMsgToTF(tar.pose, p);
    tf::poseTFToMsg(transform * p, tar.pose);

    if (uniform_generator_() < sensor_->detProb(tar)) {
			phd_msgs::RangeBearing m = sensor_->poseToMeasurement(tar, msg.child_frame_id);
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
