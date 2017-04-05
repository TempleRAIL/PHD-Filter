#ifndef MOTION_MODEL_HPP
#define MOTION_MODEL_HPP

#include <ros/ros.h>
#include <phd_msgs/Particle.h>

namespace phd {

class MotionModel {
public:
  // Likelihood of a target with state x surviving until the next time step
  virtual double survivalProb(const phd_msgs::Particle& x) const = 0;
  // Likelihood of a target with state x transitioning to state y
  virtual double transitionLikelihood(const phd_msgs::Particle& x, const phd_msgs::Particle& y) const = 0;
  // Draw a new target state y from current state x, return if particle survived
  virtual bool particleMotion(const phd_msgs::Particle& x, phd_msgs::Particle* y) = 0;
  // PHD of the birth process at state x
  virtual double birthPHD(const phd_msgs::Particle& x) const = 0;
  // Get total weight of the PHD of the birth process
  virtual double birthRate(void) const = 0;
  // Draw a new target state x from the birth process
  virtual void particleBirth(phd_msgs::Particle* x) = 0;
};

} // end namespace

#endif
