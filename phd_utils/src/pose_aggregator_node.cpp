#include <string>
#include <vector>
#include <map>

#include "ros/ros.h"

#include <boost/thread.hpp>

#include "geometry_msgs/PoseStamped.h"
#include "phd_msgs/TargetArray.h"

using namespace std;

class PoseSubscriber {
  public:
    PoseSubscriber(const string& agent_prefix, const string& pose_topic,
                   unsigned int id, const string& frame) : 
                   type_(agent_prefix), frame_(agent_prefix + to_string(id) + "/" + frame) {
      node_ = ros::NodeHandle(agent_prefix + to_string(id));
      sub_ = node_.subscribe(pose_topic, 5, &PoseSubscriber::poseCallback, this);
    }

    ~PoseSubscriber() { }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
			boost::recursive_mutex::scoped_lock lock(target_lock_);
			
      target_.header = msg->header;
      target_.pose = msg->pose;
      target_.child_frame_id = frame_;
      target_.type = type_;
    }

    phd_msgs::Target* getTarget() {
			boost::recursive_mutex::scoped_lock lock(target_lock_);
      return &target_;
    }

  private:
    ros::NodeHandle node_;
    ros::Subscriber sub_;
    
		boost::recursive_mutex target_lock_;

    string frame_;
    string type_;
    phd_msgs::Target target_;
};


class PoseAggregator {
  public:
    PoseAggregator(ros::NodeHandle& node) {
			int num_types;
			ROS_ASSERT(node.hasParam("num_types"));
			node.getParam("num_types", num_types);
			
			string pose_topic;
			node.param("pose_topic", pose_topic, string("pose"));

			string frame_id;
			node.param("frame_id", frame_id, string("base"));
			
			int count_total = 0;
			std::map<std::string, int> type_count;
			for (int i=0; i<num_types; ++i) {
				string type;
				int count;
				
				ROS_ASSERT(node.hasParam("type" + to_string(i)));
				node.getParam("type" + to_string(i), type);
			
				ROS_ASSERT(node.hasParam("count" + to_string(i)));
				node.getParam("count" + to_string(i), count);
				
				ROS_INFO("%d of type %s", count, type.c_str());
				type_count[type] = count;
				count_total += count;
			}

			subscribers_.resize(count_total);
			msg_.array.resize(count_total);
			int j = 0;
			for (const auto& tc : type_count) {
				for (int i = 0; i < tc.second; i++) {
					subscribers_[i+j].reset(new PoseSubscriber(tc.first, pose_topic, i, frame_id));
				}
				j += tc.second;
			}
			
			pub_ = node.advertise<phd_msgs::TargetArray>("pose_array", 5);
    }

    ~PoseAggregator() { }

    void publishTargetArray(const ros::TimerEvent& event) {
			unsigned int j = 0;
			for (auto& s : subscribers_) {
        phd_msgs::Target* t = s->getTarget();
        msg_.array[j++] = *t;
      }
			
      pub_.publish(msg_);
    }

  private:
    ros::Publisher pub_;
    vector<boost::shared_ptr<PoseSubscriber> > subscribers_;

    phd_msgs::TargetArray msg_;
};



int main(int argc, char **argv) {
  ros::init(argc, argv, "pose_aggregator");
  ros::NodeHandle node("~");
  
  PoseAggregator agg(node);	
	
  double publish_freq;
  node.param("publish_freq", publish_freq, 5.0);
  
  ros::Timer timer = node.createTimer(ros::Rate(publish_freq), &PoseAggregator::publishTargetArray, &agg);

	//~ ros::MultiThreadedSpinner spinner(0);
  //~ spinner.spin();
  ros::spin();

  return 0;
}
