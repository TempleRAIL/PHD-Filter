#include <phd_msgs/BearingArray.h>
#include <phd_sensor/sensor_vis.hpp>

namespace phd {

typedef SensorVisualization<phd_msgs::Bearing, phd_msgs::BearingArray> BearingVisualization;

template <>
void BearingVisualization::measurementToMarker(const phd_msgs::Bearing& z,
																							 const geometry_msgs::Pose& p,
																							 visualization_msgs::Marker* m) {
	m->header.stamp = ros::Time::now();

	m->type = visualization_msgs::Marker::LINE_STRIP;

	m->pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

	m->scale.x = 0.05;

	m->color.r = RED;
	m->color.g = GREEN;
	m->color.b = BLUE;
	m->color.a = 0.9;

	m->points.resize(2);
	m->points[0].x = z.min_range * cos(z.bearing);
	m->points[0].y = z.min_range * sin(z.bearing);

	m->points[1].x = z.max_range * cos(z.bearing);
	m->points[1].y = z.max_range * sin(z.bearing);
}

} // end namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "bearing_vis");

  phd::BearingVisualization sv;

  ros::spin();

  return 0;
}
