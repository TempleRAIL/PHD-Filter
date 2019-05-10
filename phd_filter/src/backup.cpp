	void OSPA(const phd_msgs::TargetArray& tar)
	{		
		int rows = floor(p_.width / p_.grid_res), cols = floor(p_.height / p_.grid_res);
		const std::vector<std::string>& type_name = sensor_->getSensor()->getParams().target_types;     // store type names
		int type_num = type_name.size();		
		std::vector<std::vector<weight_type>> particle(rows);    // store indexed particle matrix with types 
		for (int i = 0; i < rows; i++)
		{
			particle[i].resize(cols);
		}
		std::vector<std::vector<std::vector<weight_type>>> particle_t(type_num); 
		for (int i = 0; i < type_num; i++)
		{
			particle_t[i].resize(rows);
			for (int j = 0; j < rows; j++)
			{
				particle_t[i][j].resize(cols);
			}
			
		}
		int ind_x, ind_y;
		std::vector<Pos2d> maxima;			// store all local maxima		

		std::vector<std::vector<Pos2d>> type_array(type_num);

		for (int i = 0; i < rows*cols; i++) 
        {
			ind_x = (phd_.particles[3*i].pose.position.x - p_.origin_x) / p_.grid_res ;
			ind_y = (phd_.particles[3*i].pose.position.y - p_.origin_y) / p_.grid_res ;
			particle[ind_x][ind_y].weight = phd_.particles[3*i].w+phd_.particles[3*i+1].w+phd_.particles[3*i].w;
			
			for (int j = 0; j < type_num; j++)
			{
				
				
				particle_t[j][ind_x][ind_y].weight = phd_.particles[3*i+j].w;
				particle_t[j][ind_x][ind_y].type = phd_.particles[3*i+j].type;
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
		const double THRESHOLD = 0.01;
		for(int row = 0; row < rows; row++)
			for(int col = 0 ; col < cols; col++)
			{
				int cnt_high = 0;
				int cnt_valid = 0;				
				// Peak of all classes
				for(int i = -2; i < 3; i++) 
					for(int j = -2; j < 3; j++)
						if(row+i >= 0 && row+i < rows && col+j >= 0 && col+j < cols) 
						{
							cnt_high += (particle[row+i][col+j].weight+EPS) < particle[row][col].weight 
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
				
				
				int cnt_high_t = 0;
				int cnt_valid_t = 0;
				// Peak of each class
				for(int i = -2; i < 3; i++) 
					for(int j = -2; j < 3; j++)
						for(int k = 0; k < type_num; k++)
							if(row+i >= 0 && row+i < rows && col+j >= 0 && col+j < cols) 
							{
								cnt_high_t += (particle_t[k][row+i][col+j].weight+EPS) < particle_t[k][row][col].weight 
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

					for(int i = 0; i < type_num; i++)
					{
						if(particle_t[i][row][col].type == type_name[i])
						{
							ROS_INFO("A %s is found", particle_t[i][row][col].type.c_str());
							type_array[i].push_back(pos_);
							
							if(i == 0)
							{
								c.r = 1.0;
								c.g = 0.0;
								c.b = 0.0;
							}
							else if(i == 1)
							{
								c.r = 0.0;
								c.g = 1.0;
								c.b = 0.0;
							}
							else if(i == 2)
							{
								c.r = 0.0;
								c.g = 0.0;
								c.b = 1.0;
							}
							else
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
			}
		peak_marker_pub_.publish(m);		
		ROS_INFO("Estimation number is %lu", maxima.size());  // for debugging
			
		// https://github.com/kykleung/RFS-SLAM	
		double cutoff = 10.0;
		double order = 1.0;	
		std::vector<Pos2d> target_set;
		std::vector<std::vector<Pos2d>> tar_type(type_num);
		for (int i = 0; i < tar.array.size(); i++)
		{
			Pos2d tar_pos_(tar.array[i].pose.position.x, tar.array[i].pose.position.y);
			target_set.push_back(tar_pos_);
			for (int j = 0; j < type_num; j++)
			{
				if(tar.array[i].type == type_name[j]) 
					tar_type[j].push_back(tar_pos_);
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
	
	    // Find local maxima and calculate OSPA  
    ros::Time time = ros::Time::now(); 
    //~ class weight_type
    //~ {
	//~ public:
		//~ weight_type():weight(0.0), type(""){}
		//~ ~weight_type(){}

		//~ double weight;
		//~ std::string type;
	//~ };
	
	//~ void OSPA(const phd_msgs::TargetArray& tar)
	//~ {		
		//~ int rows = floor(p_.width / p_.grid_res), cols = floor(p_.height / p_.grid_res);
		
		//~ std::vector<std::vector<weight_type>> particle(rows);    // store indexed particle matrix with types 
		//~ for (int i = 0; i < rows; i++)
		//~ {
			//~ particle[i].resize(cols);
		//~ }
		//~ int ind_x, ind_y;
		//~ std::vector<Pos2d> maxima;			// store all local maxima
		//~ const std::vector<std::string>& type_name = sensor_->getSensor()->getParams().target_types;     // store type names
		//~ int type_num = type_name.size();
		//~ std::vector<std::vector<Pos2d>> type_array(type_num);
		
		//~ for (int i = 0; i < phd_.particles.size(); i++) 
        //~ {
			//~ ind_x = (phd_.particles[i].pose.position.x - p_.origin_x) / p_.grid_res ;
			//~ ind_y = (phd_.particles[i].pose.position.y - p_.origin_y) / p_.grid_res ;
			//~ particle[ind_x][ind_y].weight = phd_.particles[i].w;
			//~ particle[ind_x][ind_y].type = phd_.particles[i].type;
		//~ }

		//~ visualization_msgs::Marker m;
		//~ m.header.stamp = ros::Time::now();
		//~ m.header.frame_id = p_.map_frame;
		//~ m.ns = "peak";
		//~ m.id = 1;
		//~ m.type = visualization_msgs::Marker::POINTS;
		//~ m.action = visualization_msgs::Marker::ADD;
		//~ m.scale.x = m.scale.y = p_.grid_res;
		//~ const double THRESHOLD = EPS;
		//~ for(int row = 0; row < rows; row++)
			//~ for(int col = 0 ; col < cols; col++)
			//~ {
				//~ int cnt_high = 0;
				//~ int cnt_valid = 0;
				//~ for(int i = -2; i < 3; i++) 
					//~ for(int j = -2; j < 3; j++)
						//~ if(row+i >= 0 && row+i < rows && col+j >= 0 && col+j < cols) 
						//~ {
							//~ cnt_high += (particle[row+i][col+j].weight+EPS) < particle[row][col].weight 
							//~ && particle[row][col].weight > THRESHOLD ? 1 : 0 ;
							//~ ++cnt_valid;
						//~ }
				//~ if(cnt_high == cnt_valid-1) 
				//~ {
					//~ double x = p_.origin_x + row * p_.grid_res;
					//~ double y = p_.origin_y + col * p_.grid_res;

					//~ Pos2d pos_(x, y);
					//~ maxima.push_back(pos_);	
					
					//~ geometry_msgs::Point p;
					//~ p.x = x;
					//~ p.y = y;						
					//~ std_msgs::ColorRGBA c;

					//~ for(int i = 0; i < type_num; i++)
					//~ {
						//~ if(particle[row][col].type == type_name[i])
						//~ {
							//~ ROS_INFO("A %s is found", particle[row][col].type.c_str());
							//~ type_array[i].push_back(pos_);
							
							//~ if(i == 0)
							//~ {
								//~ c.r = 1.0;
								//~ c.g = 0.0;
								//~ c.b = 0.0;
							//~ }
							//~ else if(i == 1)
							//~ {
								//~ c.r = 0.0;
								//~ c.g = 1.0;
								//~ c.b = 0.0;
							//~ }
							//~ else if(i == 2)
							//~ {
								//~ c.r = 0.0;
								//~ c.g = 0.0;
								//~ c.b = 1.0;
							//~ }
							//~ else
							//~ {
								//~ c.r = 0.0;
								//~ c.g = 0.0;
								//~ c.b = 0.0;
							//~ }
							//~ c.a = 1.0;
							//~ m.points.push_back(p);				
							//~ m.colors.push_back(c);							
						//~ }												
					//~ }
				//~ }				
			//~ }
		//~ peak_marker_pub_.publish(m);
		//~ ROS_INFO("Estimation number is %lu", maxima.size());  // for debugging
			
		//~ // https://github.com/kykleung/RFS-SLAM	
		//~ double cutoff = 10.0;
		//~ double order = 1.0;	
		//~ std::vector<Pos2d> target_set;
		//~ std::vector<std::vector<Pos2d>> tar_type(type_num);
		//~ for (int i = 0; i < tar.array.size(); i++)
		//~ {
			//~ Pos2d tar_pos_(tar.array[i].pose.position.x, tar.array[i].pose.position.y);
			//~ target_set.push_back(tar_pos_);
			//~ for (int j = 0; j < type_num; j++)
			//~ {
				//~ if(tar.array[i].type == type_name[j]) 
					//~ tar_type[j].push_back(tar_pos_);
			//~ }
		//~ }
		//~ ROS_INFO("Target number is %lu", target_set.size());       // for debugging

		//~ rfs::OSPA<Pos2d> ospa(maxima, target_set, cutoff, order);
		//~ std::vector<double> e(type_num+1);
		//~ double e_d, e_c;
		//~ e[0] = ospa.calcError(&e_d, &e_c, true);
		//~ /*ospa.reportSoln();
		//~ std::cout << "OSPA error:        " << e << std::endl;
	    //~ std::cout << "distance error:    " << e_d << std::endl;
		//~ std::cout << "cardinality error: " << e_c << std::endl;*/
		//~ ROS_INFO("OSPA is %f",e[0]);    
					
		//~ for(int i = 0; i < type_num; i++)
		//~ {
			//~ rfs::OSPA<Pos2d> ospa_type(type_array[i], tar_type[i], cutoff, order);
			//~ e[i+1] = ospa_type.calcError(&e_d, &e_c, true);
			//~ ROS_INFO("OSPA of %s is %f", type_name[i].c_str(), e[i+1]);    
		//~ }
		//~ save_ospa << ros::Time::now() << ' ' << e[0] << ' ' << e[1] << ' ' << e[2] << ' ' << e[3] << '\n';
		
	//~ }
