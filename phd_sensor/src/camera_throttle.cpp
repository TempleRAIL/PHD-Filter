#include <phd_msgs/CameraArray.h>
#include <phd_sensor/sensor_throttle.hpp>

namespace phd {

typedef SensorThrottle<phd_msgs::CameraArray> CameraThrottle;

template <>
void CameraThrottle::measurementCallback(const phd_msgs::CameraArray& msg) {
  double dt = (msg.pose.header.stamp - last_pose_.header.stamp).toSec();
  if ( dt > p_.max_time ||
      (dt > p_.min_time && (dist(msg.pose, last_pose_) > p_.min_step || angle(msg.pose, last_pose_) > p_.min_angle)) ) {
    last_pose_ = msg.pose;
    measurement_pub_.publish(msg);
  }
}

} // end namespace


int main(int argc, char **argv) {
  ros::init(argc, argv, "camera_throttle");
  ros::NodeHandle n("~");

  phd::CameraThrottle t(n);

  ros::spin();

  return 0;
}
