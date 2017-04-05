#ifndef SENSOR_VIS_HPP
#define SENSOR_VIS_HPP

#include <algorithm>
#include <map>
#include <string>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>

#define RED   (0.80)  // 204
#define GREEN (0.80)  // 204
#define BLUE  (0.00)  // 0

namespace phd {

template <class Measurement, class MArray>
class SensorVisualization {
public:
  SensorVisualization() : n_(), pn_("~") {
    pub_ = n_.advertise<visualization_msgs::MarkerArray>("measurement_markers", 1);
    sub_ = n_.subscribe("measurements", 1, &SensorVisualization::callback, this);
  }

  ~SensorVisualization() {
    for (std::map<std::string, size_t>::iterator it = num_markers_last_.begin();
         it != num_markers_last_.end(); ++it) {
      visualization_msgs::MarkerArray ma;
      for (int i = 0; i < it->second; ++i) {
        visualization_msgs::Marker m;
        m.header.stamp = ros::Time::now();

        m.ns = it->first + std::string("/measurements");
        m.id = i;
        m.action = visualization_msgs::Marker::DELETE;

        ma.markers.push_back(m);
      }
      pub_.publish(ma);
    }
  }

  void callback(const MArray& msg) {
    visualization_msgs::MarkerArray ma;

    tf::Pose transform;
    tf::poseMsgToTF(msg.pose.pose, transform);

    std::string name = msg.child_frame_id;
    int i = name.find_last_of("/");
    name = name.substr(0, i);

    // Find number last published for that robot
    int num_last = 0;
    if (num_markers_last_.find(name) != num_markers_last_.end()) {
      num_last = num_markers_last_[name];
    }

    for (int i = 0; i < msg.array.size(); ++i) {
      visualization_msgs::Marker m;
      // Add marker in local sensor frame
      measurementToMarker(msg.array[i], msg.pose.pose, &m);

      m.header.frame_id = msg.pose.header.frame_id;
      m.ns = name + std::string("/measurements");
      m.id = i;
      m.action = visualization_msgs::Marker::ADD;

      // Put into map frame
      tf::Pose p;
      tf::poseMsgToTF(m.pose, p);
      tf::poseTFToMsg(transform * p, m.pose);

      ma.markers.push_back(m);
    }
    num_markers_last_[name] = ma.markers.size();

    // Delete any extra markers
    for (int i = ma.markers.size(); i < num_last; ++i) {
      visualization_msgs::Marker m;
      m.header.stamp = ros::Time::now();
      m.header.frame_id = msg.pose.header.frame_id;

      m.ns = name + std::string("/measurements");
      m.id = i;
      m.action = visualization_msgs::Marker::DELETE;

      ma.markers.push_back(m);
    }

    pub_.publish(ma);
  }

  // Measurement to marker
  // Create measurement marker in local sensor frame
  virtual void measurementToMarker(const Measurement& z,
    const geometry_msgs::Pose& p, visualization_msgs::Marker* m);

private:
  ros::NodeHandle n_, pn_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

  std::map<std::string, size_t> num_markers_last_;

};

} // end namespace

#endif
