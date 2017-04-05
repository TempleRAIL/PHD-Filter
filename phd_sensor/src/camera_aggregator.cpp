#include <phd_msgs/CameraArray.h>
#include <phd_sensor/sensor_aggregator.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "SensorAggregator");
  ros::NodeHandle n("~");

  phd::SensorAggregator<phd_msgs::CameraArray> a(n);

  ros::spin();

  return 0;
}
