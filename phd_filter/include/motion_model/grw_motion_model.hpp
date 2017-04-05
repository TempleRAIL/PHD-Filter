#ifndef GRW_MOTION_MODEL_HPP
#define GRW_MOTION_MODEL_HPP

#include <boost/random.hpp>
#include <boost/generator_iterator.hpp>
#include <boost/math/distributions/normal.hpp>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>  // matrix sqrt

#include <tf/transform_datatypes.h>

#include "motion_model.hpp"

namespace phd {

// Gaussian Random Walk
class GRWMotionModel : public MotionModel {
public:
  struct Params {
    Eigen::Matrix2d cov;          // motion model covariance
    Eigen::Matrix2d cov_sqrt;     // sqrt of motion model covariance
    double survival_prob;         // probability of target survival
    double survival_dist;         // distance from boundary of maximum survival
    double birth_rate;            // expected number of new targets
    double origin_x;              // origin of environment
    double origin_y;              // origin of environment
    double width;                 // width of environment
    double height;                // height of environment
  };

  GRWMotionModel(const Params& p) : p_(p), rand_(0), uniform_generator_(rand_, uniform_),
    gaussian_generator_(rand_, gaussian_) { }

  static GRWMotionModel* ROSInit(const ros::NodeHandle& n) {
    GRWMotionModel::Params p;

    double cov_xx, cov_xy, cov_yy;
    n.param("cov_xx", cov_xx, 1.0);
    n.param("cov_xy", cov_xy, 0.0);
    n.param("cov_yy", cov_yy, 1.0);

    p.cov(0,0) = cov_xx;
    p.cov(0,1) = p.cov(1,0) = cov_xy;
    p.cov(1,1) = cov_yy;

    if (p.cov.determinant() <= 0.0) {
      ROS_ERROR("GRWMotionModel: Covariance must be positive definite");
    }

    p.cov_sqrt = p.cov.sqrt();

    n.param("survival_prob", p.survival_prob, 1.0);
    if (p.survival_prob < 0.0) {
      ROS_ERROR("Survival probability must be in the range [0, 1]");
      p.survival_prob = 0.0;
    } else if (p.survival_prob > 1.0) {
      ROS_ERROR("Survival probability must be in the range [0, 1]");
      p.survival_prob = 1.0;
    }
    n.param("survival_dist", p.survival_dist, 1.0);

    n.param("birth_rate", p.birth_rate, 0.0);

    n.param("origin_x", p.origin_x, 0.0);
    n.param("origin_y", p.origin_y, 0.0);
    n.param("width", p.width, 10.0);
    n.param("height", p.height, 10.0);

    GRWMotionModel* motion_model = new GRWMotionModel(p);
    return motion_model;
  }

  virtual double survivalProb(const phd_msgs::Particle& x) const {
    // Compute signed distance to the edge of the environment
    double dist = std::min( std::min(x.pose.position.x - p_.origin_x,
                                     p_.origin_x + p_.width - x.pose.position.x),
                            std::min(x.pose.position.y - p_.origin_y,
                                     p_.origin_y + p_.height - x.pose.position.y));

    if (dist < 0.0) {
      // Outisde of environment
      return 0.0;
    } else {
      return p_.survival_prob * std::min(1.0, dist / p_.survival_dist);
    }
  }

  virtual double transitionLikelihood(const phd_msgs::Particle& x,
                                      const phd_msgs::Particle& y) const {
    bool equal = true;
    if (x.pose.position.x == y.pose.position.x &&
        x.pose.position.y == y.pose.position.y) {
      return 1.0;
    } else {
      return 0.0;
    }
  }

  virtual bool particleMotion(const phd_msgs::Particle& x,
                              phd_msgs::Particle* y) {
    // Check if particle survives
    if (uniform_generator_() < survivalProb(x)) {
      // Perturb particle position
      Eigen::Vector2d pos;
      pos(0) = gaussian_generator_();
      pos(1) = gaussian_generator_();
      pos = p_.cov_sqrt * pos;

      y->pose.position.x = x.pose.position.x + pos(0);
      y->pose.position.y = x.pose.position.y + pos(1);
      y->pose.orientation = x.pose.orientation;
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
      return birthRate() * 1.0 / (p_.width * p_.height);
    } else {
      return 0.0;
    }
  }

  virtual double birthRate(void) const {
    return p_.birth_rate;
  }

  virtual void particleBirth(phd_msgs::Particle* x) {
    if (birthRate() > 0.0) {
      x->pose.position.x = p_.origin_x + p_.width * uniform_generator_();
      x->pose.position.y = p_.origin_y + p_.height * uniform_generator_();
      x->pose.orientation = tf::createQuaternionMsgFromYaw(2.0 * M_PI * uniform_generator_());
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
