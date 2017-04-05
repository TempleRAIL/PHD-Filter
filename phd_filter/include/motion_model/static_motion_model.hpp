#ifndef STATIC_MOTION_MODEL_HPP
#define STATIC_MOTION_MODEL_HPP

#include "motion_model.hpp"

namespace phd {

class StaticMotionModel : public MotionModel {
public:
  StaticMotionModel(void) { }

  static StaticMotionModel* ROSInit(const ros::NodeHandle& n) {
    StaticMotionModel* motion_model = new StaticMotionModel();
    return motion_model;
  }

  virtual double survivalProb(const phd_msgs::Particle& x) const {
    return 1.0;
  }

  virtual double transitionLikelihood(const phd_msgs::Particle& x,
                                      const phd_msgs::Particle& y) const {
    if (x.pose.position.x == y.pose.position.x &&
        x.pose.position.y == y.pose.position.y &&
        x.pose.position.z == y.pose.position.z) {
      return 1.0;
    } else {
      return 0.0;
    }
  }

  virtual bool particleMotion(const phd_msgs::Particle& x,
                              phd_msgs::Particle* y) {
    *y = x;
    return true;
  }

  virtual double birthPHD(const phd_msgs::Particle& x) const {
    return 0.0;
  }

  virtual double birthRate(void) const {
    return 0.0;
  }

  virtual void particleBirth(phd_msgs::Particle* x) {
    x = NULL;
  }

private:

};

} // end namespace

#endif
