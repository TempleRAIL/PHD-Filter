#include <phd_msgs/RangeBearingArray.h>
#include <phd_sensor/sensor_aggregator.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "range_bearing_aggregator");
  ros::NodeHandle n("~");

  phd::SensorAggregator<phd_msgs::RangeBearingArray> sa(n);

  ros::spin();

  return 0;
}
