#include <phd_sensor/range_bearing/range_bearing_sensor.hpp>
#include <phd_msgs/RangeBearingArray.h>

using namespace phd;

class Confusion {	
public:
	Confusion() : pn_("~"), sensor_(RangeBearingSensor::ROSInit(pn_))
	{
		//~ sensor.reset(phd::Sensor::ROSInit(node));
		measurement_sub_ = n_.subscribe("measurements", 100, &Confusion::confusionCallback, this);	
		confusion_pub_ = n_.advertise<phd_msgs::RangeBearingArray>("cm_measurements", 100, true);
	}
	
	~Confusion(){}

private:	
	void confusionCallback(const phd_msgs::RangeBearingArray& Z)
	{
		Z_ = Z;
		const std::vector<std::string>& target_types = sensor_->getParams().target_types; 
		const double& size = target_types.size();
		
		// normalize probability distribution for each true target set
		std::vector<double> normalizer(size);
		for(int i = 0; i < size; i++)
		{
			double sum = 0.0;
			for(int j = 0; j < size; j++)
			{
				sum += sensor_->getParams().confusion_matrix.at(target_types.at(j)).at(target_types.at(i));				
			}
			normalizer[i] = 1.0 / sum;
		}
		
		// add confusion to type measurements
		srand(Z.pose.header.seq);
		for(int k = 0; k < Z.array.size(); k++) 
		{ 					
			const double rn = ((double) rand() / (RAND_MAX)); 
			double p = 0.0, pt = 0.0;
			for(int i = 0; i < size; i++)  
			{
				pt = sensor_->getParams().confusion_matrix.at(target_types.at(i)).at(Z.array[k].type);
				if(p < rn && rn < p + pt * normalizer[i])
				{
					Z_.array[k].type = target_types[i];
					break;
				}
				p += pt * normalizer[i];			
			}
		}
		confusion_pub_.publish(Z_);
	}
	
	ros::NodeHandle n_, pn_;
	boost::scoped_ptr<RangeBearingSensor> sensor_;
	ros::Subscriber measurement_sub_;
	ros::Publisher confusion_pub_;
	phd_msgs::RangeBearingArray Z_;	
};	

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "confusion_matrix");
	ros::NodeHandle node("~");
	Confusion confusion_;
	ros::spin();
}

