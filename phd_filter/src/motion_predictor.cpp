#include <math.h>

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
	
	// Birth new particles
  if (num_particles_birth > 0) {
    birthParticles(phd, num_particles_birth);
  }
}

// updateParticle
// Update an individual particle
bool MotionPredictor::updateParticle(phd_msgs::Particle* p) {
  phd_msgs::Particle p_out;
  bool survival = models_[p->type].model->particleMotion(*p, &p_out);
  *p = p_out;
  return survival;
}

// birthParticles
// Draw particles from birth PHD
void MotionPredictor::birthParticles(phd_msgs::ParticleArray* pa,
	int num_particles /* = -1 */) {
	// Find total birth weight
	double total = 0.0;
	for (const auto& m : models_) {
		total += m.second.model->birthRate();
	}
	
	// Find number of particles for each type
	std::map<std::string, double> num;
	for (const auto& m : models_) {
		if (m.second.model->birthRate() > 0.0) {
			num[m.first] = (num_particles == -1) ? 
				(*m.second.poisson_generator)() : 
				round(num_particles * m.second.model->birthRate() / total);
		} else {
			num[m.first] = 0;
		}
	}
	
	// Create new particles with uniform weight
	for (const auto& m : models_) {
		for (int i = 0; i < static_cast<int>(num[m.first]); ++i) {
			phd_msgs::Particle p;
			m.second.model->particleBirth(&p);
			p.w = m.second.model->birthRate() / num[m.first];
			p.type = m.first;
			pa->particles.push_back(p);
		}
	}
}

} // end namespace
