#ifndef DOWNWARD_SENSOR_HPP
#define DOWNWARD_SENSOR_HPP

#include <math.h>
#include <utility>
#include <vector>
#include <string>
#include <algorithm> //std::max, std::min

#include <boost/scoped_ptr.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <phd_msgs/Bearing.h>
#include <phd_sensor/sensor.hpp>
#include <tf/transform_datatypes.h>

namespace phd {
	
template<>
class Sensor<phd_msgs::Downward> {
public:
	struct Params {
		double std_dev;
		double kappa;
		double ele_max;
		double ele_min;
		double bearing_range; //widest half-angle
		
		std::vector<std::pair<double, double> > det_prob;
		
		std::map<std::string, std::map<std::string, double> > confusion_matrix;
		
	};
	typedef phd_msgs::Downward Measurement;
	
	
	Sensor(const Params& p) : p_(p), 
		normalizer_(1.0/(sqrt(2.0 * M_PI) * p.std_dev)) {}
	
	static Sensor* ROSInit(const ros::NodeHandle& n) {}
	
	double detProb(const phd_Msgs::Target& x) const {
		
		double footprint = pose_.position.z * tan(p_.bearing_range);
		double dist_x = pose_.position.x - x.pose.position.x;
		double dist_y = pose_.position.y - x.pose.position.y;
		double dist = sqrt(dist_x * dist_x + dist_y * dist_y);
		double pfn = std::max(std::min(0.2 * sqrt(footprint / 5.0), 0.99), 0.01);
		if (dist < footprint){
			return 0.0;
		}
		else {
			return (1 - pfn);
		}		
	}
	
	double measurementLikelihood(const Measurement& z, 
			const phd_msgs::Target& x) const {}
	
		
	
private:
	Params p_;
	double normalizer_;
	geometry_msgs::Pose pose_;
	
