#include <motion_model/motion_predictor.hpp>

namespace phd {

// predict
// PHD prediction function
void MotionPredictor::predict(phd_msgs::ParticleArray* phd,
    int num_particles_birth /* = -1 */) {
  // Draw new particles from motion model
  std::vector<phd_msgs::Particle>::iterator it = phd->particles.begin();
  while (it != phd->particles.end()) {
    phd_msgs::Particle p;
    if (updateParticle(&(*it))) {
      ++it;
    } else {
      it = phd->particles.erase(it);
    }
  }
  if (num_particles_birth > 0) {
    birthParticles(phd, num_particles_birth);
    std::vector<phd_msgs::Particle>::reverse_iterator it = phd->particles.rbegin();
    for (int i = 0; i < num_particles_birth; ++i, ++it) {
      it->w = motion_model_->birthRate() / static_cast<double>(num_particles_birth);
    }
  }
}

// updateParticle
// Update an individual particle
bool MotionPredictor::updateParticle(phd_msgs::Particle* p) {
  phd_msgs::Particle p_out;
  bool survival = motion_model_->particleMotion(*p, &p_out);
  *p = p_out;
  return survival;
}

// birthParticles
// Draw particles from birth PHD
void MotionPredictor::birthParticles(phd_msgs::ParticleArray* pa,
    int num_particles /* = -1 */) {
  if (motion_model_->birthRate() > 0.0) {
    int n = (num_particles == -1) ? (*poisson_generator_)() : num_particles;
    for (int i = 0; i < n; ++i) {
      phd_msgs::Particle p;
      motion_model_->particleBirth(&p);
      pa->particles.push_back(p);
    }
  }
}

} // end namespace
