#include <phd_msgs/RangeArray.h>
#include <phd_sensor/range/range_sim.hpp>

#include <nav_msgs/GetMap.h>

#include "phd_filter.hpp"

namespace phd {

typedef PHDFilter<sensor_msgs::Range, phd_msgs::RangeArray> PHDFilterRange;

template <>
void PHDFilterRange::update(const phd_msgs::RangeArray& Z) {
  update(Z.pose.pose, Z.array);
}

} // end namespace


int main(int argc, char **argv) {
  ros::init(argc, argv, "phd_filter");
  ros::NodeHandle node("~");

  bool initialize_map;
  node.param("initialize_map", initialize_map, true);

  boost::scoped_ptr<phd::PHDFilterRange> filter(phd::PHDFilterRange::ROSInit(node));

  if (initialize_map) {
    // get map via RPC
    nav_msgs::GetMap::Request  req;
    nav_msgs::GetMap::Response resp;
    ROS_INFO("Requesting the map...");
    while (!ros::service::call("/static_map", req, resp)) {
      ROS_WARN("Request for map '%s' failed; trying again...",
               ros::names::resolve(std::string("/static_map")).c_str());
      ros::Duration d(4.0);
      d.sleep();
      if (!node.ok()) {
        return 0;
      }
    }
    ROS_INFO_ONCE("Received a %d X %d map @ %.3f m/pix\n",
                  resp.map.info.width, resp.map.info.height,
                  resp.map.info.resolution);

    filter->setMap(resp.map);
    filter->trimToMap();
  }

  ros::spin();

  return 0;
}
