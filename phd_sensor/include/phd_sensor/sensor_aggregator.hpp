#ifndef SENSOR_AGGREGATOR_HPP
#define SENSOR_AGGREGATOR_HPP

#include <map>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

namespace phd {

template <class MArray>
class SensorRepublisher {
public:
  SensorRepublisher(std::string name, ros::Publisher* measurement_pub)
      : pn_(name), measurement_pub_(measurement_pub) {
    measurement_sub_ = pn_.subscribe("measurements", 1, &SensorRepublisher::measurementCallback, this);
  }

private:
  void measurementCallback(const MArray& msg) {
    measurement_pub_->publish(msg);
  }

  ros::NodeHandle pn_;
  ros::Subscriber measurement_sub_;
  ros::Publisher* measurement_pub_;
};


template <class MArray>
class SensorAggregator {
public:
  SensorAggregator(const ros::NodeHandle& n) : n_() {
    measurement_pub_ = n_.advertise<MArray>("measurements", 10, true);

    std::string agent_name;
    int num_agents, num_agents_offset;
    n.param("agent_prefix", agent_name, std::string("scarab"));
    n.param("num_agents", num_agents, 0);
    n.param("num_agents_offset", num_agents_offset, 0);

    for (int i = 0; i < num_agents; ++i) {
      std::stringstream name_key;
      std::string name;
      name_key << agent_name << i + num_agents_offset;
      n.param(name_key.str(), name, name_key.str());

      republishers_[name] = new SensorRepublisher<MArray>(name, &measurement_pub_);
    }
  }

  ~SensorAggregator(void) {
    for (typename std::map<std::string, SensorRepublisher<MArray>*>::iterator it = republishers_.begin();
         it != republishers_.end(); ++it) {
      delete it->second;
    }
  }

private:
  ros::NodeHandle n_;
  ros::Publisher measurement_pub_;
  std::map<std::string, SensorRepublisher<MArray>*> republishers_;
};

} // end namespace

#endif
