#include <phd_msgs/RangeBearingArray.h>
#include <phd_sensor/sensor_vis.hpp>

namespace phd {

typedef SensorVisualization<phd_msgs::RangeBearing, phd_msgs::RangeBearingArray> RangeBearingVisualization;

template <>
void RangeBearingVisualization::measurementToMarker(const phd_msgs::RangeBearing& z,
		const geometry_msgs::Pose& p, visualization_msgs::Marker* m) {
  m->header.stamp = ros::Time::now();
	m->type = visualization_msgs::Marker::SPHERE;

	m->pose.position.x = z.range * cos(z.bearing);
	m->pose.position.y = z.range * sin(z.bearing);
	m->pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

	m->scale.x = 0.15;
	m->scale.y = m->scale.x;
	m->scale.z = m->scale.x;

	m->color.r = RED;
	m->color.g = GREEN;
	m->color.b = BLUE;
	m->color.a = 0.9;
}

} // end namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "range_bearing_vis");

  phd::RangeBearingVisualization rbv;

  ros::spin();

  return 0;
}
