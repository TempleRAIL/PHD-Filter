#include <fstream>
#include <iostream>
#include <map>
#include <math.h>
#include <string>
#include <time.h>
#include <unistd.h>

#include "ros/ros.h"

#include <boost/thread.hpp>

#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"

#include <phd_msgs/Particle.h>
#include <motion_model/motion_predictor.hpp>

using namespace std;

namespace phd {

////////////////////////////////////////////////////////////////////////////////
// The class for the driver
class MotionSimAgent {
private:
  ////////////////
  // Parameters
  ////////////////

  string name_; // agent name

  double publish_freq_; // frequency to publish pose

  string base_frame_id_;
  string odom_frame_id_;
  string global_frame_id_;

  bool pub_global_frame_;
  tf::TransformBroadcaster broadcaster_;

  ros::NodeHandle* node_;
  ros::Publisher pose_pub_;
  ros::Timer timer_;

  boost::recursive_mutex state_lock_;
  bool active_;
  phd_msgs::Particle p_;

public:

  // Constructor
  MotionSimAgent(ros::NodeHandle* node, const string& name, const phd_msgs::Particle p) :
      name_(name), node_(node), active_(true), p_(p) {
    node_->param("publish_freq", publish_freq_, 10.0);

    string pose_topic;
    node_->param("pose_topic", pose_topic, string("pose"));

    node_->param(string("base_frame_id"), base_frame_id_, string("/base_link"));
    node_->param(string("odom_frame_id"), odom_frame_id_, string("/odom"));
    node_->param(string("global_frame_id"), global_frame_id_, string("/map"));
    node_->param("pub_global_frame", pub_global_frame_, false);

    // ensure that frame id begins with / character
    if (base_frame_id_.compare(0, 1, "/", 1) != 0) {
      base_frame_id_ = string("/") + base_frame_id_;
    }
    if (odom_frame_id_.compare(0, 1, "/", 1) != 0) {
      odom_frame_id_ = string("/") + odom_frame_id_;
    }

    base_frame_id_ = "/" + name + base_frame_id_;
    odom_frame_id_ = "/" + name + odom_frame_id_;

    pose_pub_ =
      node_->advertise<geometry_msgs::PoseStamped>("/"+name+"/"+pose_topic, 100);
  }

  // spinPublish
  bool spinPublish() {
    ros::Rate rpub(this->publish_freq_);
    while (node_->ok() && active()) {
      publishPosition();
      rpub.sleep();
    }
    return true;
  }

  // PublishPosition
  void publishPosition() {
    boost::recursive_mutex::scoped_lock lock(state_lock_);

    if (!active_) {
      return;
    }

    geometry_msgs::PoseStamped p;
    p.header.stamp = ros::Time::now();
    p.header.frame_id = global_frame_id_;
    p.pose = p_.pose;

    pose_pub_.publish(p);

    tf::Transform transform;
    poseMsgToTF(p_.pose, transform);
    broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time(p.header.stamp),
                                                   odom_frame_id_, base_frame_id_));
    if (pub_global_frame_) {
      broadcaster_.sendTransform(tf::StampedTransform(tf::Transform::getIdentity(), ros::Time(p.header.stamp),
                                                     global_frame_id_, odom_frame_id_));
    }
  }

  // update state
  void updateState(phd_msgs::Particle p, bool active) {
    boost::recursive_mutex::scoped_lock lock(state_lock_);

    if (!active && active_) {
      ROS_INFO("Agent %s has stopped", name_.c_str());
    }

    p_ = p;
    active_ = active;
  }

  phd_msgs::Particle& getState(void) {
    boost::recursive_mutex::scoped_lock lock(state_lock_);
    return p_;
  }

  bool active(void) {
    boost::recursive_mutex::scoped_lock lock(state_lock_);
    return active_;
  }
};

void agentPublish(MotionSimAgent *agent) {
  agent->spinPublish();
}

class MotionSim {
private:
  int num_agents_;
  string agent_prefix_;
  double dt_;

  ros::NodeHandle* n_;
  map<string, MotionSimAgent*> agents_;
  map<string, boost::thread*> publish_threads_;
  ros::Timer prediction_timer_;
  boost::scoped_ptr<MotionPredictor> motion_predictor_;

public:
  ~MotionSim() {
    for(map<string, MotionSimAgent*>::iterator i = agents_.begin();
        i != agents_.end(); ++i) {
      delete publish_threads_[i->first];
      delete i->second;
    }
  }

  MotionSim(ros::NodeHandle* n) : n_(n) {
    // Read in parameters
    n_->param("agent_prefix", agent_prefix_, string("scarab"));
    n_->param("num_agents", num_agents_, 0);

    // Initialize all of the agents
    for (int i = 0; i < num_agents_; ++i) {
      stringstream name_key, initial_position_key, initial_orientation_key, initial_velocity_key;
      string initial_position, initial_orientation, initial_velocity;
      name_key << agent_prefix_ << i;
      initial_position_key << "initial_position" << i;
      initial_orientation_key << "initial_orientation" << i;
      initial_velocity_key << "initial_velocity" << i;

      string name = name_key.str();
      n_->param(initial_position_key.str(), initial_position, string("0.0 0.0 0.0"));
      n_->param(initial_orientation_key.str(), initial_orientation, string("0.0 0.0 0.0 1.0"));
      n_->param(initial_velocity_key.str(), initial_velocity, string("0.0 0.0 0.0 0.0 0.0 0.0"));

      phd_msgs::Particle p;
      stringstream initial_position_str(initial_position);
      initial_position_str >> p.pose.position.x >> p.pose.position.y >> p.pose.position.z;
      stringstream initial_orientation_str(initial_orientation);
      initial_orientation_str >> p.pose.orientation.x >> p.pose.orientation.y >> p.pose.orientation.z >> p.pose.orientation.w;
      stringstream initial_velocity_str(initial_velocity);
      initial_velocity_str >> p.velocity.linear.x >> p.velocity.linear.y >> p.velocity.linear.z
                           >> p.velocity.angular.x >> p.velocity.angular.y >> p.velocity.angular.z;

      agents_[name] = new MotionSimAgent(n_, name, p);
    }

    motion_predictor_.reset(MotionPredictor::ROSInit(*n_));

    n_->param("dt", dt_, -1.0);
  }

  // start
  void start() {
    // Start publishing and set start time for all agents
    for (map<string, MotionSimAgent*>::iterator i = agents_.begin();
         i != agents_.end(); ++i) {
      publish_threads_.insert(make_pair(i->first, new boost::thread(agentPublish, i->second)));
    }

    if (dt_ > 0.0) {
      prediction_timer_ = n_->createTimer(ros::Duration(dt_), &MotionSim::timerCallback, this);
    }
  }

  // stop
  void stop() {
    for (map<string, MotionSimAgent*>::iterator i = agents_.begin();
         i != agents_.end(); ++i) {
      publish_threads_[i->first]->join();
    }
  }

  void timerCallback(const ros::TimerEvent& event) {
    // Update each agent
    for (map<string, MotionSimAgent*>::iterator it = agents_.begin();
         it != agents_.end(); ++it) {
      if (it->second->active()) {
        phd_msgs::Particle p = it->second->getState();
        bool survive = motion_predictor_->updateParticle(&p);
        it->second->updateState(p, survive);
      } else {
        publish_threads_[it->first]->join();
        delete publish_threads_[it->first];
        publish_threads_.erase(it->first);

        delete agents_[it->first];
        agents_.erase(it->first);
      }
    }

    // Initialize new agents
    phd_msgs::ParticleArray pa;
    motion_predictor_->birthParticles(&pa);
    for (vector<phd_msgs::Particle>::iterator it = pa.particles.begin();
         it != pa.particles.end(); ++it) {
      stringstream name_key;
      name_key << agent_prefix_ << num_agents_;
      string name = name_key.str();

      ROS_INFO("Adding new agent %s", name.c_str());
      agents_[name] = new MotionSimAgent(n_, name, *it);
      publish_threads_[name] = new boost::thread(agentPublish, agents_[name]);

      ++num_agents_;
    }
  }
};

} // end namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "MotionSim");
  ros::NodeHandle n("~");
  phd::MotionSim d(&n);

  d.start();

  ros::spin();

  return(0);
}
