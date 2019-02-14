#ifndef MOTION_PREDICTOR_HPP
#define MOTION_PREDICTOR_HPP

#include <map>
#include <string>

#include <boost/scoped_ptr.hpp>
#include <boost/random.hpp>

#include <phd_msgs/ParticleArray.h>

#include <motion_model/static_motion_model.hpp>
#include <motion_model/grw_motion_model.hpp>
#include <motion_model/const_vel_motion_model.hpp>

namespace phd {

class MotionPredictor {
public:
  struct Params {
    std::map<std::string, int> motion_models;         
			// type of motion model
			// 0 = static (default)
			// 1 = Gaussian random walk
			// 2 = Constant velocity
  };

  // Constructor
  MotionPredictor(const Params& p, std::map<std::string, MotionModel*> motion_models) : p_(p), rand_(0) {
		for (const auto& m : p.motion_models) {
			models_[m.first].model.reset(motion_models[m.first]);
			
			if (models_[m.first].model->birthRate() > 0.0) {
				models_[m.first].poisson.reset(new Poisson(models_[m.first].model->birthRate()));
				models_[m.first].poisson_generator.reset(
					new boost::variate_generator<RANDType, Poisson >(rand_, *(models_[m.first].poisson)));
			}
		}
  }

  // Destructor
  ~MotionPredictor() { }

  const Params& getParams(void) { return p_; }

  // ROSInit
  // Return a pointer to a new PHDFilter constructed from ROS parameters
  static MotionPredictor* ROSInit(const ros::NodeHandle& n) {
    MotionPredictor::Params p;
    
    n.getParam("motion_models", p.motion_models);

    std::map<std::string, MotionModel*> motion_models;
    for (auto m : p.motion_models) {
			switch (m.second) {
				case 0:
					motion_models[m.first] = StaticMotionModel::ROSInit(ros::NodeHandle(n, m.first));
					break;

				case 1:
					motion_models[m.first] = GRWMotionModel::ROSInit(ros::NodeHandle(n, m.first));
					break;

				case 2:
					motion_models[m.first] = ConstVelMotionModel::ROSInit(ros::NodeHandle(n, m.first));
					break;

				default:
					ROS_WARN("PHDFilter: No such motion model defined for %s, using static model", m.first.c_str());
					motion_models[m.first] = StaticMotionModel::ROSInit(ros::NodeHandle(n, m.first));
			}
		}

    MotionPredictor* motion_predictor = new MotionPredictor(p, motion_models);
    return motion_predictor;
  }

  // predict
  // PHD prediction function
  void predict(phd_msgs::ParticleArray* phd, int num_particles_birth = -1);

  // updateParticle
  // Update an individual particle
  bool updateParticle(phd_msgs::Particle* p);

  // birthParticles
  // Draw particles from birth PHD
  void birthParticles(phd_msgs::ParticleArray* pa, int num_particles = -1);

  MotionModel* getMotionModel(const std::string& s) { return models_[s].model.get(); }

private:
  typedef boost::mt19937 RANDType;
  typedef boost::poisson_distribution<> Poisson;
  
  struct MotionModelGen {
		boost::scoped_ptr<MotionModel> model;
		boost::scoped_ptr<Poisson> poisson;
		boost::scoped_ptr<boost::variate_generator<RANDType, Poisson > > poisson_generator;
	};

  // Class members
  Params p_;
  std::map<std::string, MotionModelGen> models_;

  RANDType rand_;
};

} // end namespace

#endif
