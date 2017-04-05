#include <phd_sensor/range/range_sim.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "RangeSim");
  ros::NodeHandle node("~");

  double publish_freq;
  node.param("publish_freq", publish_freq, 5.0);
  ros::Rate r(publish_freq);

  boost::scoped_ptr<phd::RangeSim> sim(phd::RangeSim::ROSInit(node));

  while (node.ok()) {
    sim->spin();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
