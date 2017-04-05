#include <phd_msgs/RangeBearingArray.h>
#include <phd_sensor/sensor_throttle.hpp>

namespace phd {

typedef SensorThrottle<phd_msgs::RangeBearingArray> RangeBearingThrottle;

template <>
void RangeBearingThrottle::measurementCallback(const phd_msgs::RangeBearingArray& msg) {
  double dt = (msg.pose.header.stamp - last_pose_.header.stamp).toSec();
  if ( dt > p_.max_time ||
      (dt > p_.min_time && (dist(msg.pose, last_pose_) > p_.min_step || angle(msg.pose, last_pose_) > p_.min_angle)) ) {
    last_pose_ = msg.pose;
    measurement_pub_.publish(msg);
  }
}

} // end namespace


int main(int argc, char **argv) {
  ros::init(argc, argv, "range_bearing_throttle");
  ros::NodeHandle n("~");

  phd::RangeBearingThrottle rbt(n);

  ros::spin();

  return 0;
}
