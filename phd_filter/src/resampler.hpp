#ifndef RESAMPLER_HPP
#define RESAMPLER_HPP

#include <boost/random.hpp>
#include <boost/generator_iterator.hpp>

#include <ros/ros.h>
#include <phd_msgs/ParticleArray.h>

namespace phd {

class Resampler {
protected:
  // Class members
  typedef boost::mt19937 RANDType;
  typedef boost::uniform_01<> Uniform;
  typedef boost::normal_distribution<> Gaussian;

  RANDType rand_;
  Uniform uniform_;
  boost::variate_generator<RANDType, Uniform > uniform_generator_;

public:
  // Constructor
  Resampler(void) : rand_(0), uniform_generator_(rand_, uniform_) { }

  // resample
  virtual void resample(phd_msgs::ParticleArray* p,
    size_t num_particles_max = std::numeric_limits<size_t>::max()) = 0;
};


class ResamplerNull : public Resampler {
public:
  static ResamplerNull* ROSInit(const ros::NodeHandle& n) {
    ResamplerNull* resampler = new ResamplerNull();
    return resampler;
  }

  void resample(phd_msgs::ParticleArray* p,
      size_t num_particles_max = std::numeric_limits<size_t>::max()) {
  }
};


class ResamplerStandard : public Resampler {
public:
  static ResamplerStandard* ROSInit(const ros::NodeHandle& n) {
    ResamplerStandard* resampler = new ResamplerStandard();
    return resampler;
  }

  void resample(phd_msgs::ParticleArray* p,
      size_t num_particles_max = std::numeric_limits<size_t>::max()) {
    std::vector<phd_msgs::Particle> p_in(p->particles);
    p->particles.clear();

    double w_total = 0.0;
    for (size_t i = 0; i < p_in.size(); ++i) {
      w_total += p_in[i].w;
    }

    // Draw samples
    std::list<double> samples;
    size_t n = std::min(p_in.size(), num_particles_max);
    for (size_t i = 0; i < n; ++i) {
      samples.push_back(w_total * uniform_generator_());
    }
    samples.sort();

    // Find sample indices
    std::vector<phd_msgs::Particle>::const_iterator it = p_in.begin();
    double c = it->w;
    while (!samples.empty()) {
      while (samples.front() > c) {
        c += (++it)->w;
        ROS_ASSERT(it != p_in.end());
      }
      p->particles.push_back(*it);
      p->particles.back().w = w_total / static_cast<double>(n);
    }
  }
};


class ResamplerLowVar : public Resampler {
public:
  static ResamplerLowVar* ROSInit(const ros::NodeHandle& n) {
    ResamplerLowVar* resampler = new ResamplerLowVar();
    return resampler;
  }

  void resample(phd_msgs::ParticleArray* p,
      size_t num_particles_max = std::numeric_limits<size_t>::max()) {
    std::vector<phd_msgs::Particle> p_in(p->particles);
    p->particles.clear();

    double w_total = 0.0;
    for (size_t i = 0; i < p_in.size(); ++i) {
      w_total += p_in[i].w;
    }

    // Draw sample
    size_t n = std::min(p_in.size(), num_particles_max);
    double step = w_total / static_cast<double>(n);
    double r = uniform_generator_() * step;

    // Find sample indices
    std::vector<phd_msgs::Particle>::const_iterator it = p_in.begin();
    double c = it->w;
    for (size_t i = 0; i < n; ++i) {
      while (r > c) {
        c += (++it)->w;
        ROS_ASSERT(it != p_in.end());
      }
      p->particles.push_back(*it);
      p->particles.back().w = w_total / static_cast<double>(n);

      r += step;
    }
  }
};

} // end namespace

#endif
