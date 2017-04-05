#ifndef CONST_VEL_MOTION_MODEL_HPP
#define CONST_VEL_MOTION_MODEL_HPP

#include <boost/random.hpp>
#include <boost/generator_iterator.hpp>
#include <boost/math/distributions/normal.hpp>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>  // matrix sqrt

#include <tf/transform_datatypes.h>

#include "motion_model.hpp"

namespace phd {

// Gaussian Random Walk
class ConstVelMotionModel : public MotionModel {
public:
  struct Params {
    double dt;                    // time step duration
    Eigen::Matrix4d cov;          // motion model covariance
    Eigen::Matrix4d cov_sqrt;     // sqrt of motion model covariance
    Eigen::Matrix4d cov_sqrt_inv; // sqrt of motion model covariance
    double survival_prob;         // probability of target survival
    double birth_rate;            // expected number of new targets
    double origin_x;              // origin of environment
    double origin_y;              // origin of environment
    double width;                 // width of environment
    double height;                // height of environment
  };

  ConstVelMotionModel(const Params& p) : p_(p), rand_(0), uniform_generator_(rand_, uniform_),
    gaussian_generator_(rand_, gaussian_) { }

  static ConstVelMotionModel* ROSInit(const ros::NodeHandle& n) {
    ConstVelMotionModel::Params p;

    n.param("dt", p.dt, 1.0);

    double cov_xx, cov_xy, cov_yy;
    n.param("cov_xx", cov_xx, 1.0);
    n.param("cov_xy", cov_xy, 0.0);
    n.param("cov_yy", cov_yy, 1.0);

    p.cov(0,0) = cov_xx;
    p.cov(0,1) = p.cov(1,0) = cov_xy;
    p.cov(1,1) = cov_yy;

    double cov_vxvx, cov_vxvy, cov_vyvy;
    n.param("cov_vxvx", cov_vxvx, 1.0);
    n.param("cov_vxvy", cov_vxvy, 0.0);
    n.param("cov_vyvy", cov_vyvy, 1.0);

    p.cov(2,2) = cov_vxvx;
    p.cov(2,3) = p.cov(3,2) = cov_vxvy;
    p.cov(3,3) = cov_vyvy;

    if (p.cov.determinant() <= 0.0) {
      ROS_ERROR("ConstVelMotionModel: Covariance must be positive definite");
    }

    p.cov_sqrt = p.cov.sqrt();
    p.cov_sqrt_inv = p.cov_sqrt.inverse();

    n.param("survival_prob", p.survival_prob, 1.0);
    if (p.survival_prob < 0.0) {
      ROS_ERROR("Survival probability must be in the range [0, 1]");
      p.survival_prob = 0.0;
    } else if (p.survival_prob > 1.0) {
      ROS_ERROR("Survival probability must be in the range [0, 1]");
      p.survival_prob = 1.0;
    }

    n.param("birth_rate", p.birth_rate, 0.0);

    n.param("origin_x", p.origin_x, 0.0);
    n.param("origin_y", p.origin_y, 0.0);
    n.param("width", p.width, 10.0);
    n.param("height", p.height, 10.0);

    ConstVelMotionModel* motion_model = new ConstVelMotionModel(p);
    return motion_model;
  }

  virtual double survivalProb(const phd_msgs::Particle& x) const {
    bool inside = p_.origin_x <= x.pose.position.x &&
                  x.pose.position.x <= p_.origin_x + p_.width &&
                  p_.origin_y <= x.pose.position.y &&
                  x.pose.position.y <= p_.origin_y + p_.height;

    if (inside) {
      return p_.survival_prob;
    } else {
      return 0.0;
    }
  }

  virtual double transitionLikelihood(const phd_msgs::Particle& x,
                                      const phd_msgs::Particle& y) const {
    boost::math::normal norm;

    Eigen::Vector4d delta;
    delta(0) = y.pose.position.x - (x.pose.position.x + x.velocity.linear.x * p_.dt);
    delta(1) = y.pose.position.y - (x.pose.position.y + x.velocity.linear.y * p_.dt);
    delta(2) = y.velocity.linear.x - x.velocity.linear.x;
    delta(3) = y.velocity.linear.y - x.velocity.linear.y;

    // Make the components independent via a linear transformation
    delta = p_.cov_sqrt_inv * delta;

    return pdf(norm, delta(0)) * pdf(norm, delta(1)) * pdf(norm, delta(2)) * pdf(norm, delta(3));
  }

  virtual bool particleMotion(const phd_msgs::Particle& x,
                              phd_msgs::Particle* y) {
    // Check if particle survives
    if (uniform_generator_() < survivalProb(x)) {
      // Perturb particle
      Eigen::Vector4d delta;
      delta(0) = gaussian_generator_();
      delta(1) = gaussian_generator_();
      delta(2) = gaussian_generator_();
      delta(3) = gaussian_generator_();
      delta = p_.cov_sqrt * delta;

      y->pose.position.x = x.pose.position.x + x.velocity.linear.x * p_.dt + delta(0);
      y->pose.position.y = x.pose.position.y + x.velocity.linear.y * p_.dt + delta(1);
      y->pose.orientation = x.pose.orientation;
      y->velocity.linear.x = x.velocity.linear.x + delta(2);
      y->velocity.linear.y = x.velocity.linear.y + delta(3);
      y->w = x.w;

      return true;
    } else {
      // Particle dies, so return false
      y = NULL;
      return false;
    }
  }

  virtual double birthPHD(const phd_msgs::Particle& x) const {
    boost::math::normal norm;

    bool inside = p_.origin_x <= x.pose.position.x &&
                  x.pose.position.x <= p_.origin_x + p_.width &&
                  p_.origin_y <= x.pose.position.y &&
                  x.pose.position.y <= p_.origin_y + p_.height;

    if (inside) {
      Eigen::Vector4d v;
      v(0) = 0.0;
      v(1) = 0.0;
      v(2) = x.velocity.linear.x;
      v(3) = x.velocity.linear.y;

      v = p_.cov_sqrt_inv * v;

      return birthRate() * 1.0 / (p_.width * p_.height) *
        pdf(norm, v(2)) * pdf(norm, v(3));
    } else {
      return 0.0;
    }
  }

  virtual double birthRate(void) const {
    return p_.birth_rate;
  }

  virtual void particleBirth(phd_msgs::Particle* x) {
    if (birthRate() > 0.0) {
      Eigen::Vector4d v;
      v(0) = 0.0;
      v(1) = 0.0;
      v(2) = gaussian_generator_();
      v(3) = gaussian_generator_();

      v = p_.cov_sqrt * v;

      x->pose.position.x = p_.origin_x + p_.width * uniform_generator_();
      x->pose.position.y = p_.origin_y + p_.height * uniform_generator_();
      x->pose.orientation = tf::createQuaternionMsgFromYaw(2.0 * M_PI * uniform_generator_());
      x->velocity.linear.x = v(2);
      x->velocity.linear.y = v(3);

    } else {
      x = NULL;
    }
  }

private:
  typedef boost::mt19937 RANDType;
  typedef boost::uniform_01<> Uniform;
  typedef boost::normal_distribution<> Gaussian;

  // Class members
  Params p_;

  RANDType rand_;
  Uniform uniform_;
  Gaussian gaussian_;
  boost::variate_generator<RANDType, Uniform > uniform_generator_;
  boost::variate_generator<RANDType, Gaussian > gaussian_generator_;
};

} // end namespace

#endif
