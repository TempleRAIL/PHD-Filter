#include <ros/ros.h>
#include "phd_msgs/TargetArray.h"
#include "geometry_msgs/PoseStamped.h"

class InsideFoV{
public:
	InsideFoV(const ros::NodeHandle& pn, const double& fov) : pn_(pn), fov_(fov){		
		pose_sub_ = n_.subscribe("gt_pose", 5, &InsideFoV::poseCallback, this);
		target_sub_ = n_.subscribe("target_array", 5, &InsideFoV::targetCallback, this);
		target_fov_pub_ = n_.advertise<phd_msgs::TargetArray>("target_fov", 5, true);
	}
	
	~InsideFoV(){}
	
private:
	void poseCallback(const geometry_msgs::PoseStamped& P){
		P_ = P;
	}
	
	void targetCallback(const phd_msgs::TargetArray& Z){
		phd_msgs::TargetArray tar;
		double dist = 0.0;		
		for(int i = 0; i < Z.array.size(); i++)
		{
			dist = sqrt((Z.array.at(i).pose.position.x - P_.pose.position.x) * (Z.array.at(i).pose.position.x - P_.pose.position.x)
			+ (Z.array.at(i).pose.position.y - P_.pose.position.y) * (Z.array.at(i).pose.position.y - P_.pose.position.y));
			if(dist < fov_){				
				tar.array.push_back(Z.array.at(i));		
			}			
		}
		target_fov_pub_.publish(tar);
	}
	
	ros::NodeHandle pn_, n_;
	double fov_;
	ros::Subscriber pose_sub_, target_sub_;
	ros::Publisher target_fov_pub_;
	phd_msgs::TargetArray Z_;
	geometry_msgs::PoseStamped P_;
};

int main(int argc, char **argv){
	ros::init(argc, argv, "target_fov");
	ros::NodeHandle pn("~");
	
	double fov = 0.0;
	ROS_ASSERT(pn.hasParam("fov"));
	pn.getParam("fov", fov);

	InsideFoV inside_fov_(pn, fov);
	ros::spin();
}
