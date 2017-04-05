#ifndef CAMERA_SIM_HPP
#define CAMERA_SIM_HPP

#include <phd_msgs/CameraArray.h>
#include <phd_sensor/camera/camera_sensor.hpp>
#include <phd_sensor/sensor_sim.hpp>

#include <tf/transform_datatypes.h>

#include <unsupported/Eigen/MatrixFunctions>  // matrix sqrt

namespace phd {

typedef SensorSim<phd_msgs::Camera, phd_msgs::CameraArray> CameraSim;

template<>
geometry_msgs::Pose CameraSim::drawPoseFromMeasurement(const phd_msgs::Camera& z,
    const geometry_msgs::Pose& p) {
  geometry_msgs::Pose x;

  double ang_horizontal = z.col * z.fov_horizontal / static_cast<double>(z.width);
  double ang_vertical = z.row * z.fov_vertical / static_cast<double>(z.height);

  x.position.x = tan(ang_horizontal - z.fov_horizontal / 2.0);
  x.position.y = tan(ang_vertical - z.fov_vertical / 2.0);
  x.position.z = 1.0;

  // Get transformation from sensor frame to global frame
  tf::Pose transform;
  tf::poseMsgToTF(p, transform);

  // Put pose in global frame
  tf::Vector3 v;
  tf::pointMsgToTF(x.position, v);
  v = transform * v;

  // Put on ground plane
  double scale_factor = p.position.z / (p.position.z - v.z());
  v = v * scale_factor;

  // Set marker pose
  tf::pointTFToMsg(v, x.position);
  x.orientation = tf::createQuaternionMsgFromYaw(0.0);

  return p;
}

template<>
phd_msgs::Camera CameraSim::addNoiseToMeasurement(const phd_msgs::Camera& z) {
  phd_msgs::Camera z_noisy(z);

  Eigen::Vector2d noise;
  noise(0) = gaussian_generator_();
  noise(1) = gaussian_generator_();
  noise = sensor_->getParams().cov.sqrt() * noise;
  z_noisy.row += noise(0);
  z_noisy.col += noise(1);

  return z_noisy;
}

template<>
phd_msgs::Camera CameraSim::drawClutterMeasurement(const std::string& frame_id) {
  phd_msgs::Camera z = sensor_->poseToMeasurement(pose_.pose, frame_id);

  z.row = static_cast<double>(z.height) * uniform_generator_();
  z.col = static_cast<double>(z.width) * uniform_generator_();

  return z;
}

template <>
void CameraSim::spin() {
  flags_.have_map = true;

  if (!ready()) {
    return;
  }

  phd_msgs::CameraArray msg;
  msg.pose = pose_;
  // transform from base frame to sensor frame
  tf::Quaternion q;
  tf::quaternionMsgToTF(pose_.pose.orientation, q);
  q = q * tf::createQuaternionFromRPY(M_PI, 0.0, 0.0);
  tf::quaternionTFToMsg(q, msg.pose.pose.orientation);
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
      phd_msgs::Camera m = sensor_->poseToMeasurement(x, msg.child_frame_id);
      msg.array.push_back(addNoiseToMeasurement(m));
    }
  }

  // Add false positives
  for (int i = 0; i < samplePoisson(sensor_->getClutterRate()); ++i) {
    phd_msgs::Camera m = drawClutterMeasurement(msg.child_frame_id);
    msg.array.push_back(m);
  }

  measurement_pub_.publish(msg);
}

} // end namespace

#endif
