#include <phd_msgs/BearingArray.h>
#include <phd_sensor/sensor_aggregator.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "sensor_aggregator");
  ros::NodeHandle n("~");

  phd::SensorAggregator<phd_msgs::BearingArray> sa(n);

  ros::spin();

  return 0;
}
