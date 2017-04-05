#include <phd_sensor/bearing/bearing_sim.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "bearing_sim");
  ros::NodeHandle node("~");

  double publish_freq;
  node.param("publish_freq", publish_freq, 5.0);
  ros::Rate r(publish_freq);

  boost::scoped_ptr<phd::BearingSim> sim(phd::BearingSim::ROSInit(node));

  while (node.ok()) {
    sim->spin();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
