Header header           # timestamp in the header is the time the sensor
                        # returned the camera reading

int32 height						# image height, that is, number of rows
int32 width							# image width, that is, number of columns

float32 fov_vertical    # vertical field of view [rad], that is, over rows
float32 fov_horizontal  # horizontal field of view [rad], that is, over columns

float32 row         		# pixel reading
                        # (Note: values < 0 or > height should be discarded)
float32 col         		# pixel reading
                        # (Note: values < 0 or > width should be discarded)
