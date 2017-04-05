#ifndef PHD_FILTER_HPP
#define PHD_FILTER_HPP

#include <algorithm>  // max, random_shuffle
#include <math.h>     // hypot
#include <string>
#include <vector>

#include <boost/scoped_ptr.hpp>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

#include <phd_msgs/ParticleArray.h>
#include <phd_sensor/sensor_sim.hpp>

#include <motion_model/motion_predictor.hpp>

#include <grid_map/grid_map.hpp>

#include "resampler.hpp"

#define EPS (1.0e-9)

namespace phd {

template <class Measurement, class MeasurementSet>
class PHDFilter {
public:
  struct Params {
    int resample_method;      // Method of resampling
                              // 0 = standard
                              // 1 = low variance
    int resample_every_n;     // resample every n steps
    int num_particles_max;    // Maximum number of particles in the filter
    int num_particles_birth;  // Number of birth particles per time step
    int num_particles_birth_per_measurement;
    double frac_birth_measurement;
    double num_targets;       // initial number of targets
    double origin_x;          // x value of map origin
    double width;             // width of environment
    double origin_y;          // y value of map origin
    double height;            // height of environment
    double grid_res;          // resolution of particle grid
    double max_value;         // value to saturate PHD particle weight
    std::string map_frame;    // map frame
    double dt;                // duration of prediction time step
  };

  // Constructor
  PHDFilter(const Params& p, SensorSim<Measurement, MeasurementSet>* sensor,
      MotionPredictor* motion_predictor, Resampler* resampler) :
      p_(p), n_(), pn_("~"), map_(NULL), sensor_(sensor),
      motion_predictor_(motion_predictor), resampler_(resampler),
      t_now_(0), t_last_(0), resample_count_(0) {
    phd_.header.frame_id = p_.map_frame;

    double num_particles = floor(p_.width / p_.grid_res) * floor(p_.height / p.grid_res);
    for (double x = p_.origin_x; x < p_.origin_x + p_.width; x += p_.grid_res) {
      for (double y = p_.origin_y; y < p_.origin_y + p_.height; y += p_.grid_res) {
        phd_msgs::Particle p;
        p.pose.position.x = x;
        p.pose.position.y = y;
        p.w = p_.num_targets / num_particles;
        phd_.particles.push_back(p);
      }
    }
    std::random_shuffle(phd_.particles.begin(), phd_.particles.end());

    // Initialize PHD grid
    phd_grid_.reset(new GridMap(p_.origin_x, p_.width, p_.origin_y, p_.height, p_.grid_res, p_.max_value));
    setPHDGrid();
    phd_grid_->setFrameId(p_.map_frame);

    measurement_sub_ = n_.subscribe("measurements", 100, &PHDFilter::measurementCallback, this);
    phd_pub_ = n_.advertise<phd_msgs::ParticleArray>("phd", 1, true);
    phd_grid_pub_ = n_.advertise<nav_msgs::OccupancyGrid>("phd_grid", 1, true);
    marker_pub_ = pn_.advertise<visualization_msgs::Marker>("markers", 1, true);

    if (p_.dt > 0.0) {
      prediction_timer_ = pn_.createTimer(ros::Duration(p.dt), &PHDFilter::predict, this);
    }
  }

  // Destructor
  ~PHDFilter() { }

  // ROSInit
  // Return a pointer to a new PHDFilter constructed from ROS parameters
  static PHDFilter* ROSInit(const ros::NodeHandle& n) {
    PHDFilter::Params p;

    n.param("resample_method", p.resample_method, 0);
    n.param("resample_every_n", p.resample_every_n, 1);
    n.param("num_particles_max", p.num_particles_max, 10000);
    n.param("num_particles_birth", p.num_particles_birth, 100);
    n.param("num_particles_birth_per_measurement", p.num_particles_birth_per_measurement, 20);
    n.param("frac_birth_measurement", p.frac_birth_measurement, 0.0);
    n.param("num_targets", p.num_targets, 1.0);
    n.param("origin_x", p.origin_x, 0.0);
    n.param("origin_y", p.origin_y, 0.0);
    n.param("width", p.width, 10.0);
    n.param("height", p.height, 10.0);
    n.param("grid_res", p.grid_res, 0.1);
    n.param("max_value", p.max_value, 1.0);
    n.param("map_frame", p.map_frame, std::string("/map"));
    n.param("motion/dt", p.dt, -1.0);

    SensorSim<Measurement, MeasurementSet>* sensor =
      SensorSim<Measurement, MeasurementSet>::ROSInit(ros::NodeHandle(n, "sensor"));
    MotionPredictor* motion_predictor = MotionPredictor::ROSInit(ros::NodeHandle(n, "motion"));

    Resampler* resampler;
    switch (p.resample_method) {
      case 0:
        resampler = ResamplerNull::ROSInit(ros::NodeHandle(n, "resampler"));
        break;

      case 1:
        resampler = ResamplerStandard::ROSInit(ros::NodeHandle(n, "resampler"));
        break;

      case 2:
        resampler = ResamplerLowVar::ROSInit(ros::NodeHandle(n, "resampler"));
        break;

      default:
        ROS_ERROR("Resampling method not defined");
    }

    PHDFilter* filter = new PHDFilter(p, sensor, motion_predictor, resampler);
    return filter;
  }

  // getSensor
  Sensor<Measurement>* getSensor(void) { return sensor_->getSensor(); }

  // pubParticles
  // Publish the particle representation of the PHD
  void pubParticles(void) {
    phd_.header.stamp = ros::Time::now();

    phd_pub_.publish(phd_);

    setPHDGrid();
    phd_grid_pub_.publish(phd_grid_->occGrid());
  }

  // pubVis
  // Publish the visualization of particle representation of the PHD
  void pubMarkers(void) {
    visualization_msgs::Marker m;

    m.header.stamp = ros::Time::now();
    m.header.frame_id = p_.map_frame;
    m.ns = "phd";
    m.id = 0;
    m.type = visualization_msgs::Marker::POINTS;
    m.action = visualization_msgs::Marker::ADD;

    m.scale.x = m.scale.y = p_.grid_res;

    for (int i = 0; i < phd_.particles.size(); ++i) {
      const phd_msgs::Particle& p = phd_.particles[i];
      m.points.push_back(p.pose.position);

      std_msgs::ColorRGBA c;
      //~ c.r = c.g = c.b = 1.0 - std::min((double) p.w, p_.max_value) / p_.max_value;
      c.r = 1.0;
      c.b = c.g = 1.0 - std::min((double) p.w, p_.max_value) / p_.max_value;
      c.a = 0.3;
      m.colors.push_back(c);
    }

    phd_.header.stamp = ros::Time::now();
    phd_.header.frame_id = p_.map_frame;

    marker_pub_.publish(m);
  }

  // update
  // Wrapper for generic update function
  // Must be implemented in class specializations
  virtual void update(const MeasurementSet& Z);

  // setMap
  // Set an occupancy grid map to use
  void setMap(const nav_msgs::OccupancyGrid& grid) {
    if (map_.get() != NULL) {
      ROS_WARN("PHDFilter: Map already set, resetting map");
    }
    map_.reset(new GridMap(grid));
  }

  // trimToMap
  // Removes all particles outside of map
  void trimToMap(bool keep_total_weight = true) {
    if (map_.get() == NULL) {
      ROS_ERROR("PHDFilter: Map not set");
      return;
    }

    // Find initial weight
    double w0 = 0.0;
    if (keep_total_weight) {
      for (int i = 0; i < phd_.particles.size(); ++i) {
        w0 += phd_.particles[i].w;
      }
    }

    // Remove particles outside map
    std::vector<phd_msgs::Particle>::iterator it = phd_.particles.begin();
    double w1 = 0.0;
    while (it != phd_.particles.end()) {
      if (!insideMap(it->pose.position.x, it->pose.position.y)) {
        it = phd_.particles.erase(it);
      } else {
        w1 += it->w;
        ++it;
      }
    }

    // Reweight particles, if necessary
    if (keep_total_weight) {
      for (int i = 0; i < phd_.particles.size(); ++i) {
        phd_.particles[i].w *= w0 / w1;
      }
    }
  }

private:
  bool insideMap(double x, double y) {
    if (map_.get() == NULL) {
      return true;
    } else {
      int xi, yi;
      map_->getSubscript(x, y, &xi, &yi);
      return (map_->valid(xi, yi) && map_->get(xi, yi) == 0);
    }
  }

  void setPHDGrid(void) {
    phd_grid_->fill(0.0);
    for (int i = 0; i < phd_.particles.size(); ++i) {
      const phd_msgs::Particle& p = phd_.particles[i];
      int xi, yi;
      phd_grid_->getSubscript(p.pose.position.x, p.pose.position.y, &xi, &yi);
      if (phd_grid_->valid(xi, yi)) {
        phd_grid_->set(xi, yi, phd_grid_->get(xi, yi) + p.w);
      }
    }
  }

  // measurementCallback
  // Callback function for incoming measurement sets
  void measurementCallback(const MeasurementSet& Z) {
    ROS_ASSERT(Z.pose.header.frame_id == p_.map_frame);
    t_last_ = t_now_;
    t_now_ = Z.pose.header.stamp;
    update(Z);
    pubParticles();
    pubMarkers();
  }

  void predict(const ros::TimerEvent& event) {
    motion_predictor_->predict(&phd_);

    if (p_.num_particles_birth > 0) {
      phd_msgs::ParticleArray phd_new;
      motion_predictor_->birthParticles(&phd_new, p_.num_particles_birth);

      // Set weight and add new particles to PHD
      double w_new = (1.0 - p_.frac_birth_measurement) *
        motion_predictor_->getMotionModel()->birthRate() / phd_new.particles.size();
      for (int i = 0; i < phd_new.particles.size(); ++i) {
        phd_new.particles[i].w = w_new;
        phd_.particles.push_back(phd_new.particles[i]);
      }
    }

    pubParticles();
    pubMarkers();
  }

  // update
  // PHD update function
  void update(const geometry_msgs::Pose& pose, const std::vector<Measurement>& Z) {
    int nZ = Z.size();
    int nX = phd_.particles.size();

    // Get transformation from map frame to measurement frame
    tf::Pose transform;
    tf::poseMsgToTF(pose, transform);
    transform = transform.inverse();

    // Pre-compute terms
    double *p_det;                  // detection likelihood
    p_det = new double[nX];
    double **psi = new double*[nX]; // psi(x, z) = p_d(x) * p(z | x)
    for (int i = 0; i < nX; ++i) {
      psi[i] = new double[nZ];
    }

    int footprintCount = 0;
    int *footprintIndices;          // cells in sensor footprint
    footprintIndices = new int[nX];
    double denominator[nZ]; // denominator(z) = kappa(z) + sum_x psi(x, z)
    for (int i = 0; i < nX; ++i) {
      // Get pose of particle in sensor frame
      const phd_msgs::Particle& p = phd_.particles[i];
      if (insideMap(p.pose.position.x, p.pose.position.y)) {
        tf::Pose x_tf;
        poseMsgToTF(p.pose, x_tf);
        x_tf = transform * x_tf;
        geometry_msgs::Pose x;
        poseTFToMsg(x_tf, x);

        p_det[i] = sensor_->getSensor()->detProb(x);
        if (p_det[i] > 0.0) {
          footprintIndices[footprintCount++] = i;
        }

        for (int j = 0; j < nZ; ++j) {
          psi[i][j] = 0.0;
        }
      }
    }
    for (int zi = 0; zi < nZ; ++zi) {
      denominator[zi] = sensor_->getSensor()->clutterLikelihood(Z[zi]) + EPS;
      if (isnan(denominator[zi])) {
        ROS_ERROR("Denominator is nan, shutting down");
        ros::shutdown();
      }
    }

    for (int i = 0; i < footprintCount; ++i) {
      const int& ind = footprintIndices[i];
      // Get pose of particle in sensor frame
      const phd_msgs::Particle& p = phd_.particles[ind];
      tf::Pose x_tf;
      tf::poseMsgToTF(p.pose, x_tf);
      x_tf = transform * x_tf;
      geometry_msgs::Pose x;
      tf::poseTFToMsg(x_tf, x);

      for (int zi = 0; zi < nZ; ++zi) {
        double p_z = sensor_->getSensor()->measurementLikelihood(Z[zi], x);
        psi[ind][zi] = p_det[ind] * p_z;
        if (isnan(psi[ind][zi])) {
          ROS_ERROR("Psi is nan, shutting down, p_z = %.3f, p_det = %.3f", p_z, p_det[ind]);
          ros::shutdown();
        }
        denominator[zi] += psi[ind][zi] * p.w;
      }
    }

    // Update PHD
    for (int i = 0; i < footprintCount; ++i) {
      const int& ind = footprintIndices[i];
      phd_msgs::Particle& p = phd_.particles[ind];

      double w = 1.0 - p_det[ind];
      for (int zi = 0; zi < nZ; ++zi) {
        w += psi[ind][zi] / denominator[zi];
      }
      p.w *= w;
      if (isnan(p.w)) {
        ROS_ERROR("PHD is nan, shutting down");
        ros::shutdown();
      }
    }

    // Clean up
    delete[] p_det;
    delete[] footprintIndices;
    for (int i = 0; i < nX; ++i) {
      delete[] psi[i];
    }
    delete[] psi;

    // Add new particles around measurements
    if (t_last_.toSec() != 0 && Z.size() > 0 &&
        p_.num_particles_birth_per_measurement > 0 &&
        p_.frac_birth_measurement > 0.0) {
      phd_msgs::ParticleArray phd_new;
      phd_new.particles.clear();
      for (int j = 0; j < Z.size(); ++j) {
        motion_predictor_->birthParticles(&phd_new, p_.num_particles_birth_per_measurement);
        std::vector<phd_msgs::Particle>::reverse_iterator it = phd_new.particles.rbegin();
        for (int i = 0; i < p_.num_particles_birth_per_measurement; ++i, ++it) {
          // Add noise to each measurement and convert to a pose
          Measurement z = sensor_->addNoiseToMeasurement(Z[j]);
          it->pose = sensor_->drawPoseFromMeasurement(z, pose);
        }
      }

      // Set weight and add new particles to PHD
      double w_new = p_.frac_birth_measurement *
        (t_now_ - t_last_).toSec() / p_.dt *
        motion_predictor_->getMotionModel()->birthRate() /
        phd_new.particles.size();
      for (int i = 0; i < phd_new.particles.size(); ++i) {
        phd_new.particles[i].w = w_new;
        phd_.particles.push_back(phd_new.particles[i]);
      }
    }

    if ((++resample_count_) % p_.resample_every_n == 0) {
      resampler_->resample(&phd_, p_.num_particles_max);
    }
  }


  // Class members
  Params p_;
  boost::scoped_ptr< SensorSim<Measurement, MeasurementSet> > sensor_;
  boost::scoped_ptr<MotionPredictor> motion_predictor_;
  phd_msgs::ParticleArray phd_;
  boost::scoped_ptr<GridMap> phd_grid_, map_;
  boost::scoped_ptr<Resampler> resampler_;
  ros::Time t_now_, t_last_;

  ros::NodeHandle n_, pn_;
  ros::Subscriber measurement_sub_;
  ros::Publisher phd_pub_, marker_pub_, phd_grid_pub_;
  ros::Timer prediction_timer_;
  int resample_count_;
};

} // end namespace

#endif
