#include "GY87.h"

namespace gy_87 {

// Constructor
GY87::GY87(ros::NodeHandle nh){
	// Init publishers
	ROS_INFO("GY87: Init Publishers...");
    pub_imu_ = nh.advertise<sensor_msgs::Imu>("imu_data", 1000);
	pub_mag_ = nh.advertise<sensor_msgs::MagneticField>("magnetic_field", 1000);
	
	// Init I2C
	ROS_INFO("GY87: Init I2C...");
	I2Cdev::initialize();

	// Init sensors
	ROS_INFO("GY87: Init sensors...");
	mpu_ = MPU6050(0x68);
	mpu_.initialize();

	// Enable I2C bypass to be able to connect to magnet sensor
	// (connected to auxilary I2C of MPU)
	mpu_.setI2CMasterModeEnabled(false);
	mpu_.setI2CBypassEnabled(true) ;
	mpu_.setSleepEnabled(false);

	hmc_ = HMC5883L(0x1E);
	hmc_.initialize();
}

// Destructor
GY87::~GY87(){}

void GY87::publish(){
	// Read sensor data
	mpu_.getMotion6(&axi, &ayi, &azi, &gxi, &gyi, &gzi);
	hmc_.getHeading(&mxi, &myi, &mzi);
	
	// Convert sensor data
	ax = GY87::getAccelerationFloat(axi);
	ay = GY87::getAccelerationFloat(ayi);
	az = GY87::getAccelerationFloat(azi);
	gx = GY87::getAngularVelocityFloat(gxi);
	gy = GY87::getAngularVelocityFloat(gyi);
	gz = GY87::getAngularVelocityFloat(gzi);
	mx = GY87::getMagneticFieldFloat(mxi);
	my = GY87::getMagneticFieldFloat(myi);
	mz = GY87::getMagneticFieldFloat(mzi);
	
	// Calculate attitude quaternion
	MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz);

	// Create message
	msg_imu_.orientation.x = q0;
	msg_imu_.orientation.y = q1;
	msg_imu_.orientation.z = q2;
	msg_imu_.orientation.w = q3;
	msg_imu_.linear_acceleration.x = ax;
	msg_imu_.linear_acceleration.y = ay;
	msg_imu_.linear_acceleration.z = az;
	msg_imu_.angular_velocity.x = gx;
	msg_imu_.angular_velocity.y = gy;
	msg_imu_.angular_velocity.z = gz;
	msg_mag_.magnetic_field.x = mx;
	msg_mag_.magnetic_field.y = my;
	msg_mag_.magnetic_field.z = mz;
	
	// Publish messages
	pub_imu_.publish(msg_imu_);
	pub_mag_.publish(msg_mag_);
}

// Returns the linear acceleration a in m/sec² as float.
float GY87::getAccelerationFloat(int16_t a, int range_in_g) {
  return ((float)a)*range_in_g/32768*9.80665;
}

// Returns the angular velocity g in rad/sec as float.
float GY87::getAngularVelocityFloat(int16_t g, int range_in_deg) {
  return ((float)g)*range_in_deg/32768 * 3.14159265357989/180;
}

// Returns the magnetic field in Tesla as float.
float GY87::getMagneticFieldFloat(int16_t m, float range_in_T){
  return ((float)m)*((4.35 - 0.73)/(8.1 - 0.88)*(range_in_T - 0.88) + 0.73);

}

}
