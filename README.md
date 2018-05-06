# GY-87 for ROS on Raspberry Pi 3
Implementation of the sensors on the GY-87 breakout board in ROS for Raspberry Pi. It is reading out the sensor MPU6050 and HMC5883L which is connected to the auxilary I2C port of the former sensor.

The pressure sensor BMP180 and the core temperature of the MPU6050 are not read out.

This package is using libraries developed for Arduino and utilizing them with an I2Cdev wrapper for the Raspberry Pi. This wrapper is using [bcm2835](http://www.airspayce.com/mikem/bcm2835/index.html) instead of [wiringPi](http://wiringpi.com/download-and-install/).

## Topics

Two topics are being published with 50 Hz:

* `/gy_87/imu_data`: Linear acceleration, angular velocity read from the MPU6050. Quaternion of the orientation calculated with Madgwick's AHRS algorithm.

* `/gy_87/magnetic_field`: Magnetic field read from HMC5883L.

All data is without covariance matrices.

## Installation

An installation of ROS is required on the Raspberry Pi. See [ROS wiki](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi) for this. An additional package required for this package is `sensor_msgs`.

An installation of [bcm2835](http://www.airspayce.com/mikem/bcm2835/index.html) is required.

Clone this package to your catkin workspace source folder:

```
cd ~/catkin_ws/src/
git clone https://github.com/smcgit/gy_87.git
```

Build the package:

```
cd ~/catkin_ws/
catkin_make --pkg=gy_87 && . devel/setup.bash
```


## Running the Node

Since the node needs `sudo` to use I2C, the node cannot be run directly (it will end with a `Segmentation fault` message). It needs to be run with the included launch file instead. Type the following to run the node:

```
roslaunch gy_87 gy_87.launch
```

## Stopping the Node

There might be a bug in ROS where a node run with sudo is not exiting completely. Check with `ps aux` if the node is still is running after stopping it with `CTRL+C` and kill these processes.

## Performance

`ps aux` reports around 4% CPU and 1% memory usage when publishing topics with 50 Hz. A similar approach in Python took about 30% CPU usage.

## ToDo

* The integer values read from the sensors are currently interpolated from two values found somewhere online. Definitely need to check the values for the magnetic field. However, the length of the magnetic field vector is not important since it gets normalized during quaternion calculation.

* Add covariance matrices.

## Credits

This package is utilising work from the creators of the following libraries:

* [bcm2835](http://www.airspayce.com/mikem/bcm2835/index.html)
* [I2Cdev](https://github.com/jrowberg/i2cdevlib/tree/master/RaspberryPi_bcm2835/I2Cdev), [MPU6050](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050) and [HMC5883L](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/HMC5883L)
* [Madgwick's AHRS algorithm](http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/)

