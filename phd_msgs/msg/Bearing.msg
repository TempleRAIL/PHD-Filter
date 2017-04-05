Header header           # timestamp in the header is the time the sensor
                        # returned the bearing reading

float32 min_range       # the minimum range that the bearing reading is
                        # valid for [m]
float32 max_range       # the maximum range that the bearing reading is
                        # valid for [m]
                        # the object causing the bearing reading may have
                        # been anywhere within min_range and
                        # max_range at the measured bearing.

float32 min_bearing     # minimum bearing value [rad]
float32 max_bearing     # maximum bearing value [rad]

float32 bearing         # bearing data [rad]
                        # (Note: values < bearing_min or > bearing_max
                        # should be discarded)
