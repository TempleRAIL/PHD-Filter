#include <phd_msgs/RangeArray.h>
#include <phd_sensor/sensor_vis.hpp>

namespace phd {

typedef SensorVisualization<sensor_msgs::Range, phd_msgs::RangeArray> RangeVisualization;

template <>
void RangeVisualization::measurementToMarker(const sensor_msgs::Range& z,
    const geometry_msgs::Pose& p, visualization_msgs::Marker* m) {
  m->header.stamp = ros::Time::now();
  m->type = visualization_msgs::Marker::LINE_STRIP;

  m->pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  m->scale.x = 0.02;

  m->color.r = RED;
  m->color.g = GREEN;
  m->color.b = BLUE;
  m->color.a = 0.9;

  int num_points = 32;
  m->points.resize(num_points + 1);
  for (unsigned int j = 0; j < num_points; ++j) {
    double f = 2 * M_PI * j / num_points;
    double x = z.range * cos(f);
    double y = z.range * sin(f);

    geometry_msgs::Point p;
    p.x = x;
    p.y = y;

    m->points[j] = p;
  }
  m->points[num_points] = m->points[0];
}

} // end namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "range_vis");

  phd::RangeVisualization rv;

  ros::spin();

  return 0;
}
