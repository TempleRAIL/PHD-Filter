#ifndef PHD_FILTER_HPP
#define PHD_FILTER_HPP

#include <algorithm>  // max, random_shuffle
#include <cmath>     // hypot, isnan, etc.
#include <memory>			// shared_ptr
#include <string>
#include <vector>
#include <bits/stdc++.h> 
#include <fstream>

#include <boost/scoped_ptr.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

#include "phd_msgs/ParticleArray.h"
#include "phd_msgs/TargetArray.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include <phd_sensor/sensor_sim.hpp>

#include <motion_model/motion_predictor.hpp>

#include <grid_map/grid_map.hpp>

#include "resampler.hpp"
#include "OSPA.hpp"

// Data structure for OSPA calculation
class Pos2d{

public:
  
  Pos2d(double x, double y): x_(x), y_(y){}
  ~Pos2d(){}

  double operator-(const Pos2d& other){
    double dx = x_ - other.x_;
    double dy = y_ - other.y_;
    return sqrt(dx*dx + dy*dy);
  }
  
private:

  double x_;
  double y_;

};

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
    double num_targets;       // initial number of targets per type
    double origin_x;          // x value of map origin
    double width;             // width of environment
    double origin_y;          // y value of map origin
    double height;            // height of environment
    double grid_res;          // resolution of particle grid
    double max_value;         // value to saturate PHD particle weight
    std::string map_frame;    // map frame
    double dt;                // duration of prediction time step
    std::string ospa_filename;// path to save ospa.txt
  };

  // Constructor
  PHDFilter(const Params& p, SensorSim<Measurement, MeasurementSet>* sensor,
      MotionPredictor* motion_predictor, Resampler* resampler) :
      p_(p), n_(), pn_("~"), map_(NULL), sensor_(sensor),
      motion_predictor_(motion_predictor), resampler_(resampler),
      t_now_(0), t_last_(0), resample_count_(0) {
    phd_.header.frame_id = p_.map_frame;

    double num_particles = floor(p_.width / p_.grid_res) * floor(p_.height / p.grid_res);
    //ROS_WARN("Width %f, Res %f, Number of particle is %f", p_.width, p_.grid_res, num_particles);  
    for (double x = p_.origin_x; x < p_.origin_x + p_.width; x += p_.grid_res) {
      for (double y = p_.origin_y; y < p_.origin_y + p_.height; y += p_.grid_res) {
        phd_msgs::Particle p;
        p.pose.position.x = x;
        p.pose.position.y = y;
        p.w = p_.num_targets / num_particles;
        for (const auto& t : sensor_->getSensor()->getParams().confusion_matrix) {
					p.type = t.first;
					phd_.particles.push_back(p);
				}
      }
    }
    //ROS_INFO("PHD NUM = %lu", phd_.particles.size());
    std::random_shuffle(phd_.particles.begin(), phd_.particles.end());

    // Initialize PHD grids and publishers
    for (const auto& t : sensor_->getSensor()->getParams().confusion_matrix) {
			phd_grid_[t.first] = std::shared_ptr<GridMap>(new GridMap(p_.origin_x, p_.width, p_.origin_y, p_.height, p_.grid_res, p_.max_value));
			phd_grid_[t.first]->setFrameId(p_.map_frame);
			
			phd_grid_pub_[t.first] = n_.advertise<nav_msgs::OccupancyGrid>("phd_grid_"+t.first, 1, true);
			ROS_INFO("PHD Filter:: Created grid map for type %s", t.first.c_str());
		}
		setPHDGrid();

    measurement_sub_ = n_.subscribe("measurements", 100, &PHDFilter::measurementCallback, this);    
    phd_pub_ = n_.advertise<phd_msgs::ParticleArray>("phd", 1, true);
    peak_marker_pub_ = pn_.advertise<visualization_msgs::Marker>("peak_markers", 1, true);
    marker_pub_ = pn_.advertise<visualization_msgs::Marker>("markers", 1, true);
    //target_sub_ = n_.subscribe("target_fov", 5, &PHDFilter::OSPA, this);  // use only when dynamic targets inside fov
    target_sub_ = n_.subscribe("target_array", 5, &PHDFilter::OSPA, this);
    click_sub_ = n_.subscribe("clicked_point", 1, &PHDFilter::click, this);

    if (p_.dt > 0.0) {
      prediction_timer_ = pn_.createTimer(ros::Duration(p.dt), &PHDFilter::predict, this);
    }
    
    //std::ofstream save_ospa(path);
    save_ospa.open(p_.ospa_filename);
	if (~save_ospa.is_open())
	{
		ROS_WARN("Failed to create file.");
		ROS_WARN("%s", p_.ospa_filename.c_str());
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
    n.param("ospa_filename", p.ospa_filename, std::string(""));

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
    for (const auto& pub : phd_grid_pub_) {
			pub.second.publish(phd_grid_[pub.first]->occGrid());
		}
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
		for (const auto& g : phd_grid_) {
			g.second->fill(0.0);
		}
		for (int i = 0; i < phd_.particles.size(); ++i) {
			const phd_msgs::Particle& p = phd_.particles[i];
			int xi, yi;
			if (phd_grid_.count(p.type) > 0) {
				phd_grid_[p.type]->getSubscript(p.pose.position.x, p.pose.position.y, &xi, &yi);
				if (phd_grid_[p.type]->valid(xi, yi)) {
					phd_grid_[p.type]->set(xi, yi, phd_grid_[p.type]->get(xi, yi) + p.w);
				}
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
        
        phd_msgs::Target t;
        poseTFToMsg(x_tf, t.pose);
        t.type = p.type;

        p_det[i] = sensor_->getSensor()->detProb(t);
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
      if (std::isnan(denominator[zi])) {
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
        
			phd_msgs::Target t;
			poseTFToMsg(x_tf, t.pose);
			t.type = p.type;

      for (int zi = 0; zi < nZ; ++zi) {
        double p_z = sensor_->getSensor()->measurementLikelihood(Z[zi], t);
        psi[ind][zi] = p_det[ind] * p_z;
        if (std::isnan(psi[ind][zi])) {
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
      if (p.w < EPS) p.w = EPS;
      if (std::isnan(p.w)) {
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
          it->pose = sensor_->drawTargetFromMeasurement(z, pose).pose;
        }
      }

      // Update weight and add new particles to PHD
      for (int i = 0; i < phd_new.particles.size(); ++i) {
        phd_new.particles[i].w *= p_.frac_birth_measurement *
					(t_now_ - t_last_).toSec() / p_.dt;
		if (phd_new.particles[i].w < EPS) {
			phd_new.particles[i].w = EPS;
		}
        phd_.particles.push_back(phd_new.particles[i]);
      }
    }

    if ((++resample_count_) % p_.resample_every_n == 0) {
      resampler_->resample(&phd_, p_.num_particles_max);
    }

  }
  
    // Find local maxima and calculate OSPA  
    ros::Time time = ros::Time::now(); 
	class weight_type
    {
	public:
		weight_type():weight(0.0), type(""){}
		
		~weight_type(){}

		double weight;
		std::string type;
	};
	
	void OSPA(const phd_msgs::TargetArray& tar)
	{		
		int rows = floor(p_.width / p_.grid_res), cols = floor(p_.height / p_.grid_res);
		const std::vector<std::string>& type_name = sensor_->getSensor()->getParams().target_types;     // type names
		int type_num = type_name.size();
				
		std::vector<std::vector<weight_type>> particle(rows);    // particle matrix of all types 
		for (int i = 0; i < rows; i++)
			particle[i].resize(cols);
		
		std::vector<std::vector<std::vector<weight_type>>> particle_t(type_num);  // particle matrix of each type
		for (int i = 0; i < type_num; i++)
		{
			particle_t[i].resize(rows);
			for (int j = 0; j < rows; j++)
				particle_t[i][j].resize(cols);		
		}
		
		int ind_x, ind_y;
		std::vector<Pos2d> maxima;			// all local maxima		
		std::vector<std::vector<Pos2d>> type_array(type_num);   // local maxima of each type
		for (int i = 0; i < phd_.particles.size(); i++) 
        {			
			ind_x = (phd_.particles[i].pose.position.x - p_.origin_x) / p_.grid_res ;
			ind_y = (phd_.particles[i].pose.position.y - p_.origin_y) / p_.grid_res ;
			particle[ind_x][ind_y].weight += phd_.particles[i].w;
			
			for (int j = 0; j < type_num; j++)
			{
				particle_t[j][ind_x][ind_y].type = type_name[j];	
				if(phd_.particles[i].type == type_name[j])
				{
					particle_t[j][ind_x][ind_y].weight = phd_.particles[i].w;

				}				
			}
		}

		visualization_msgs::Marker m;
		m.header.stamp = ros::Time::now();
		m.header.frame_id = p_.map_frame;
		m.ns = "peak";
		m.id = 1;
		m.type = visualization_msgs::Marker::POINTS;
		m.action = visualization_msgs::Marker::ADD;
		m.scale.x = m.scale.y = p_.grid_res;
		const double THRESHOLD = 0.0005; //0.000003
		for(int row = 0; row < rows; row++)
			for(int col = 0 ; col < cols; col++)
			{				
				// Peak of all classes
				int cnt_high = 0;
				int cnt_valid = 0;								
				for(int i = -2; i < 3; i++) 
					for(int j = -2; j < 3; j++)
						if(row+i >= 0 && row+i < rows && col+j >= 0 && col+j < cols) 
						{
							cnt_high += particle[row+i][col+j].weight < particle[row][col].weight 
							&& particle[row][col].weight > THRESHOLD ? 1 : 0 ;
							++cnt_valid;
						}
				if(cnt_high == cnt_valid-1) 
				{
					double x = p_.origin_x + row * p_.grid_res;
					double y = p_.origin_y + col * p_.grid_res;

					Pos2d pos_(x, y);
					maxima.push_back(pos_);										
				}
				
				// Peak of each class
				for(int k = 0; k < type_num; k++)
				{
					int cnt_high_t = 0;
					int cnt_valid_t = 0;
					for(int i = -2; i < 3; i++) 
						for(int j = -2; j < 3; j++)
							if(row+i >= 0 && row+i < rows && col+j >= 0 && col+j < cols) 
							{
								cnt_high_t += particle_t[k][row+i][col+j].weight < particle_t[k][row][col].weight 
								&& particle_t[k][row][col].weight > THRESHOLD ? 1 : 0 ;
								++cnt_valid_t;
							}
					if(cnt_high_t == cnt_valid_t-1) 
					{
						double x = p_.origin_x + row * p_.grid_res;
						double y = p_.origin_y + col * p_.grid_res;	
						Pos2d pos_(x, y);
						
						geometry_msgs::Point p;
						p.x = x;
						p.y = y;						
						std_msgs::ColorRGBA c;

						//ROS_INFO("A %s is found", particle_t[k][row][col].type.c_str());
						type_array[k].push_back(pos_);
						
						if(k == 0)
						{
							c.r = 1.0;
							c.g = 0.0;
							c.b = 0.0;
						}
						else if(k == 1)
						{
							c.r = 0.0;
							c.g = 1.0;
							c.b = 0.0;
						}
						else if(k == 2)
						{
							c.r = 0.0;
							c.g = 0.0;
							c.b = 1.0;
						}
						else                    // Colors can only support up to 4 different classes
						{
							c.r = 0.0;
							c.g = 0.0;
							c.b = 0.0;
						}
						c.a = 1.0;
						m.points.push_back(p);				
						m.colors.push_back(c);										
					}	
				}						
			}
		peak_marker_pub_.publish(m);		
		ROS_INFO("Estimation number is %lu", maxima.size());  // for debugging
			
		// OSPA calculating fuction. https://github.com/kykleung/RFS-SLAM	
		double cutoff = 10.0;
		double order = 1.0;	
		std::vector<Pos2d> target_set;
		std::vector<std::vector<Pos2d>> tar_type(type_num);
		for (int i = 0; i < tar.array.size(); i++)
		{
			if(tar.array[i].pose.position.x > p_.origin_x && 
			tar.array[i].pose.position.x < p_.origin_x + p_.width &&
			tar.array[i].pose.position.y > p_.origin_y &&
			tar.array[i].pose.position.y < p_.origin_y + p_.height) 
			{
				Pos2d tar_pos_(tar.array[i].pose.position.x, tar.array[i].pose.position.y);
				target_set.push_back(tar_pos_);
				for (int j = 0; j < type_num; j++)
				{
					if(tar.array[i].type == type_name[j]) tar_type[j].push_back(tar_pos_);					
				}				
			}
		}
		ROS_INFO("Target number is %lu", target_set.size());       // for debugging

		rfs::OSPA<Pos2d> ospa(maxima, target_set, cutoff, order);
		std::vector<double> e(type_num+1);
		double e_d, e_c;
		e[0] = ospa.calcError(&e_d, &e_c, true);
		/*ospa.reportSoln();
		std::cout << "OSPA error:        " << e << std::endl;
	    std::cout << "distance error:    " << e_d << std::endl;
		std::cout << "cardinality error: " << e_c << std::endl;*/
		ROS_INFO("OSPA is %f",e[0]);    
					
		for(int i = 0; i < type_num; i++)
		{
			rfs::OSPA<Pos2d> ospa_type(type_array[i], tar_type[i], cutoff, order);
			e[i+1] = ospa_type.calcError(&e_d, &e_c, true);
			ROS_INFO("OSPA of %s is %f", type_name[i].c_str(), e[i+1]);    
		}
		save_ospa << ros::Time::now() << ' ' << e[0] << ' ' << e[1] << ' ' << e[2] << ' ' << e[3] << '\n';
		
	}

	// Show PHD weight at clicked point on rviz
	void click(const geometry_msgs::PointStamped& pt)
	{
		//~ double click = 0.0;
		std::string s = "";
		for (int i = 0; i < phd_.particles.size(); i++)
		{
			if (fabs(pt.point.x - phd_.particles[i].pose.position.x + p_.grid_res/2.0) < p_.grid_res/2.0 && 
			fabs(pt.point.y - phd_.particles[i].pose.position.y + p_.grid_res/2.0) < p_.grid_res/2.0)
			{
				//~ click += phd_.particles[i].w;
				s += '\n' + " type: " + phd_.particles[i].type + " w: " + std::to_string(phd_.particles[i].w);				
			}			
		}
		ROS_WARN("Weight %s", s.c_str());
		ros::Duration(3.0).sleep();
	}
	

  // Class members
  Params p_;
  boost::scoped_ptr< SensorSim<Measurement, MeasurementSet> > sensor_;
  boost::scoped_ptr<MotionPredictor> motion_predictor_;
  phd_msgs::ParticleArray phd_;
  boost::scoped_ptr<GridMap> map_;
  std::map<std::string, std::shared_ptr<GridMap> > phd_grid_;
  boost::scoped_ptr<Resampler> resampler_;
  ros::Time t_now_, t_last_;

  ros::NodeHandle n_, pn_;
  ros::Subscriber measurement_sub_, target_sub_, click_sub_;
  ros::Publisher phd_pub_, marker_pub_, peak_marker_pub_;
  std::map<std::string, ros::Publisher> phd_grid_pub_;
  ros::Timer prediction_timer_;
  int resample_count_;

  std::ofstream save_ospa;
};

} // end namespace

#endif
