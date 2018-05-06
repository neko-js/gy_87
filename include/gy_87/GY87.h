// ROS libraries
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"

// Sensor libraries
#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "MadgwickAHRS.h"

namespace gy_87 {

class GY87 {
    private:
		ros::Publisher pub_imu_;
		ros::Publisher pub_mag_;
		MPU6050 mpu_;
		HMC5883L hmc_;
		float ax, ay, az, gx, gy, gz, mx, my, mz;
  		int16_t axi, ayi, azi, gxi, gyi, gzi, mxi, myi, mzi;
  		sensor_msgs::Imu msg_imu_;
  		sensor_msgs::MagneticField msg_mag_;

		float getAccelerationFloat(int16_t a, int range_in_g = 2);
		float getAngularVelocityFloat(int16_t g, int range_in_deg = 250);
		float getMagneticFieldFloat(int16_t m, float range_in_T = 1.3);
        
	public:
        GY87(ros::NodeHandle nh);
        ~GY87();
		void publish();
};

}
