#include <phd_msgs/CameraArray.h>
#include <phd_sensor/sensor_vis.hpp>
#include <phd_sensor/camera/camera_sensor.hpp>

namespace phd {

typedef SensorVisualization<phd_msgs::Camera, phd_msgs::CameraArray> CameraVisualization;

phd_msgs::Camera camera_params;

geometry_msgs::Point pointToGroundPlane(const geometry_msgs::Pose& pose,
                       const geometry_msgs::Point& p_in) {
  tf::Pose transform;
  tf::poseMsgToTF(pose, transform);
  // put into world frame
  tf::Vector3 v;
  tf::pointMsgToTF(p_in, v);
  tf::Vector3 v_world = transform * v;
  // find scale factor to place marker on ground
  double scale_factor = pose.position.z / (pose.position.z - v_world.z());
  v = v * scale_factor;
  // set marker pose
  geometry_msgs::Point p_out;
  tf::pointTFToMsg(v, p_out);
  return p_out;
}

template <>
void CameraVisualization::measurementToMarker(const phd_msgs::Camera& z,
                                              const geometry_msgs::Pose& p,
                                              visualization_msgs::Marker* m) {
  m->header.stamp = ros::Time::now();

  m->type = visualization_msgs::Marker::SPHERE;

  // Put marker on ground plane in world frame
  geometry_msgs::Pose x = CameraSensor::measurementToPose(z);
  m->pose.position = pointToGroundPlane(p, x.position);
  m->pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  m->scale.x = 0.15;
  m->scale.y = m->scale.x;
  m->scale.z = m->scale.x;

  m->color.r = RED;
  m->color.g = GREEN;
  m->color.b = BLUE;
  m->color.a = 0.9;
}

template <>
void CameraVisualization::callback(const phd_msgs::CameraArray& msg) {
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

    camera_params = msg.array[i];
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

  // Publish sensor footprint
  if (camera_params.height > 0) {
    visualization_msgs::Marker m;
    m.header.stamp = ros::Time::now();
    m.header.frame_id = msg.pose.header.frame_id;

    m.type = visualization_msgs::Marker::LINE_STRIP;

    m.ns = name + std::string("/footprint");
    m.id = 0;
    m.action = visualization_msgs::Marker::ADD;

    m.pose = msg.pose.pose;

    camera_params.row = 0;
    camera_params.col = 0;
    geometry_msgs::Pose x = CameraSensor::measurementToPose(camera_params);
    m.points.push_back(pointToGroundPlane(msg.pose.pose, x.position));

    camera_params.row = camera_params.height;
    camera_params.col = 0;
    x = CameraSensor::measurementToPose(camera_params);
    m.points.push_back(pointToGroundPlane(msg.pose.pose, x.position));

    camera_params.row = camera_params.height;
    camera_params.col = camera_params.width;
    x = CameraSensor::measurementToPose(camera_params);
    m.points.push_back(pointToGroundPlane(msg.pose.pose, x.position));

    camera_params.row = 0;
    camera_params.col = camera_params.width;
    x = CameraSensor::measurementToPose(camera_params);
    m.points.push_back(pointToGroundPlane(msg.pose.pose, x.position));

    m.points.push_back(m.points.front());

    m.scale.x = 0.15;

    m.color.r = RED;
    m.color.g = GREEN;
    m.color.b = BLUE;
    m.color.a = 0.9;

    ma.markers.push_back(m);
  }


  pub_.publish(ma);
}

} // end namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "camera_vis");

  phd::CameraVisualization v;

  ros::spin();

  return 0;
}
