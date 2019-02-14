#ifndef BEARING_SIM_HPP
#define BEARING_SIM_HPP
#include <phd_msgs/BearingArray.h>
#include <phd_sensor/bearing/bearing_sensor.hpp>
#include <phd_sensor/sensor_sim.hpp>

#include <tf/transform_datatypes.h>

namespace phd {

typedef SensorSim<phd_msgs::Bearing, phd_msgs::BearingArray> BearingSim;

template<>
phd_msgs::Target BearingSim::drawTargetFromMeasurement(const phd_msgs::Bearing& z,
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
phd_msgs::Bearing BearingSim::addNoiseToMeasurement(const phd_msgs::Bearing& z) {
	phd_msgs::Bearing z_noisy(z);

	// Add noise to bearing measurement
	z_noisy.bearing += sensor_->getParams().std_dev * gaussian_generator_();
	
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
phd_msgs::Bearing BearingSim::drawClutterMeasurement(const std::string& frame_id) {
	phd_msgs::Target t;
	t.pose = pose_.pose;
	
	// Draw random class uniformly
	const Sensor<phd_msgs::Bearing>::Params& par = sensor_->getParams();
	std::map<std::string, std::map<std::string, double> >::const_iterator it = par.confusion_matrix.begin();
	std::advance(it, int_generator_(rand_));
	t.type = it->first;
	
	phd_msgs::Bearing z = sensor_->targetToMeasurement(t, frame_id);

	double bin_width = sensor_->getParams().clutter_bin_width;
	double bin_weight = sensor_->getParams().clutter_bin_weight;

	double rand = uniform_generator_();
	if (rand < bin_weight / 2.0) {
		// Bin around -pi/2
		z.bearing = -M_PI / 2.0 - bin_width / 2.0 + uniform_generator_() * bin_width;
	} else if (rand < bin_weight) {
		// Bin around +pi/2
		z.bearing =  M_PI / 2.0 - bin_width / 2.0 + uniform_generator_() * bin_width;
	} else {
		// Uniform component
		z.bearing = z.min_bearing + (z.max_bearing - z.min_bearing) * uniform_generator_();
	}

	return z;
}

template <>
void BearingSim::spin() {
	flags_.have_map = sensor_->hasMap();

  if (!ready()) {
    return;
  }

  phd_msgs::BearingArray msg;
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
			phd_msgs::Bearing m = sensor_->targetToMeasurement(tar, msg.child_frame_id);
      msg.array.push_back(addNoiseToMeasurement(m));
    }
  }

  // Add false positives
  for (int i = 0; i < samplePoisson(sensor_->getClutterRate()); ++i) {
		phd_msgs::Bearing m = drawClutterMeasurement(msg.child_frame_id);
    msg.array.push_back(m);
  }

  measurement_pub_.publish(msg);
}

template <>
void BearingSim::mapCallback(const nav_msgs::OccupancyGrid& map) {
  map_ = map;
  sensor_->setMap(map);
}

} // end namespace

#endif
