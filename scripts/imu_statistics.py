#!/usr/bin/env python

# ROS node to compute the covariance matrix of a IMU sensor. The covariance matrix is stored in an yaml file.

__author__ = "henriquejsfj@poli.ufrj.br (Henrique Ferreira Jr.)"

import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import numpy as np
import yaml
import rospkg
import os


def callback(data):
    """
    Stores the IMU data in global variables to be processed later.
    :param data: IMU data.
    """
    orien = data.orientation
    euler = euler_from_quaternion([orien.x, orien.y, orien.z, orien.w])
    euler_x[i] = euler[0]
    euler_y[i] = euler[1]
    euler_z[i] = euler[2]
    angular_velocity_x[i] = data.angular_velocity.x
    angular_velocity_y[i] = data.angular_velocity.y
    angular_velocity_z[i] = data.angular_velocity.z
    linear_acceleration_x[i] = data.linear_acceleration.x
    linear_acceleration_y[i] = data.linear_acceleration.y
    linear_acceleration_z[i] = data.linear_acceleration.z


def imu_statistics():
    """
    Init the ROS Node, receive the IMU data and calls the callback to process it.

    Note: This doesn't set a subscriber.
    """
    global i

    rospy.init_node('imu_statistics')

    rate = rospy.Rate(5)  # 5hz
    for i in range(samples):
        data = rospy.wait_for_message(topic, Imu, timeout=None)
        callback(data)
        rate.sleep()


def manage_output_directory(pkg_name):
    """
    Change the path to `$pkg_name/params`. If params folder doesn't exists creates it.
    :param pkg_name: String with the package name.
    """
    r = rospkg.RosPack()
    new_path = r.get_path(pkg_name)
    os.chdir(new_path)
    if 'params' not in os.listdir("."):
        os.mkdir('params')
    os.chdir(new_path + '/params')


def compute_variance(measures):
    """
    Compute the sample variance of the input measurements.
    :param measures: A numpy array containing the measurements.
    :return: The variance.
    """
    avg = np.sum(measures) / samples
    errors = measures - avg
    variance = np.sum(np.multiply(errors, errors)) / (samples - 1)
    return float(variance)


if __name__ == '__main__':
    actual_path = os.getcwd()
    pkg_name = rospy.get_param('~output_pkg', rospkg.get_package_name(actual_path))
    file_name = rospy.get_param('~output_file', 'imu_covariance.yaml')
    samples = rospy.get_param('~samples', 20)
    topic = rospy.get_param('~imu_topic', '/gy_87/imu_data')

    manage_output_directory(pkg_name)

    euler_x = np.zeros(samples)
    euler_y = np.zeros(samples)
    euler_z = np.zeros(samples)
    angular_velocity_x = np.zeros(samples)
    angular_velocity_y = np.zeros(samples)
    angular_velocity_z = np.zeros(samples)
    linear_acceleration_x = np.zeros(samples)
    linear_acceleration_y = np.zeros(samples)
    linear_acceleration_z = np.zeros(samples)

    imu_statistics()

    dict_covariances = {
        'orientation_cov': [compute_variance(euler_x), 0.0, 0.0,
                            0.0, compute_variance(euler_y), 0.0,
                            0.0, 0.0, compute_variance(euler_z)],
        'angular_vel_cov': [compute_variance(angular_velocity_x), 0.0, 0.0,
                            0.0, compute_variance(angular_velocity_y), 0.0,
                            0.0, 0.0, compute_variance(angular_velocity_z)],
        'linear_accel_cov': [compute_variance(linear_acceleration_x), 0.0, 0.0,
                             0.0, compute_variance(linear_acceleration_y), 0.0,
                             0.0, 0.0, compute_variance(linear_acceleration_z)]
    }

    print dict_covariances
    yaml.dump(dict_covariances)

    with open(file_name, 'w') as out_file:
        out_file.write(yaml.dump(dict_covariances))
