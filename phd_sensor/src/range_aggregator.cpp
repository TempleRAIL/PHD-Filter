#include <phd_msgs/RangeArray.h>
#include <phd_sensor/sensor_aggregator.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "range_aggregator");
  ros::NodeHandle n("~");

  phd::SensorAggregator<phd_msgs::RangeArray> ra(n);

  ros::spin();

  return 0;
}
