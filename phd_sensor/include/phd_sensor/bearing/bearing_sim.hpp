#ifndef BEARING_SIM_HPP
#define BEARING_SIM_HPP
#include <phd_msgs/BearingArray.h>
#include <phd_sensor/bearing/bearing_sensor.hpp>
#include <phd_sensor/sensor_sim.hpp>

#include <tf/transform_datatypes.h>

namespace phd {

typedef SensorSim<phd_msgs::Bearing, phd_msgs::BearingArray> BearingSim;

template<>
geometry_msgs::Pose BearingSim::drawPoseFromMeasurement(const phd_msgs::Bearing& z,
		const geometry_msgs::Pose& p) {
	geometry_msgs::Pose x;

	double range = z.min_bearing + (z.max_bearing - z.min_bearing) * uniform_generator_();

  x.position.x = range * cos(z.bearing);
  x.position.y = range * sin(z.bearing);
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
phd_msgs::Bearing BearingSim::addNoiseToMeasurement(const phd_msgs::Bearing& z) {
	phd_msgs::Bearing z_noisy(z);

	z_noisy.bearing += sensor_->getParams().std_dev * gaussian_generator_();

	return z_noisy;
}

template<>
phd_msgs::Bearing BearingSim::drawClutterMeasurement(const std::string& frame_id) {
	phd_msgs::Bearing z = sensor_->poseToMeasurement(pose_.pose, frame_id);

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
  for (std::vector<phd_msgs::Target>::iterator it = targets_.poses.begin();
       it != targets_.poses.end(); ++it) {
		tf::Pose p;
		tf::poseMsgToTF(it->pose, p);

		geometry_msgs::Pose x;
    tf::poseTFToMsg(transform * p, x);

    if (uniform_generator_() < sensor_->detProb(x)) {
			phd_msgs::Bearing m = sensor_->poseToMeasurement(x, msg.child_frame_id);
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
