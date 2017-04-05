#ifndef MOTION_PREDICTOR_HPP
#define MOTION_PREDICTOR_HPP

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
    int motion_model;         // type of motion model
                              // 0 = static (default)
                              // 1 = Gaussian random walk
                              // 2 = Constant velocity
  };

  // Constructor
  MotionPredictor(const Params& p, MotionModel* motion_model) : p_(p), rand_(0) {
    motion_model_.reset(motion_model);
    if (motion_model_->birthRate() > 0.0) {
      poisson_.reset(new Poisson(motion_model_->birthRate()));
      poisson_generator_.reset(new boost::variate_generator<RANDType, Poisson >(rand_, *poisson_));
    }
  }

  // Destructor
  ~MotionPredictor() { }

  const Params& getParams(void) { return p_; }

  // ROSInit
  // Return a pointer to a new PHDFilter constructed from ROS parameters
  static MotionPredictor* ROSInit(const ros::NodeHandle& n) {
    MotionPredictor::Params p;

    n.param("motion_model", p.motion_model, -1);

    MotionModel* motion_model;
    switch (p.motion_model) {
      case 0:
         motion_model = StaticMotionModel::ROSInit(n);
        break;

      case 1:
        motion_model = GRWMotionModel::ROSInit(n);
        break;

      case 2:
        motion_model = ConstVelMotionModel::ROSInit(n);
        break;

      default:
        ROS_WARN("PHDFilter: No such motion model defined, using static model");
        motion_model = StaticMotionModel::ROSInit(n);
    }

    MotionPredictor* motion_predictor = new MotionPredictor(p, motion_model);
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

  MotionModel* getMotionModel(void) { return motion_model_.get(); }

private:
  // Class members
  Params p_;
  boost::scoped_ptr<MotionModel> motion_model_;

  typedef boost::mt19937 RANDType;
  typedef boost::poisson_distribution<> Poisson;

  RANDType rand_;
  boost::scoped_ptr<Poisson> poisson_;
  boost::scoped_ptr< boost::variate_generator<RANDType, Poisson > > poisson_generator_;
};

} // end namespace

#endif
