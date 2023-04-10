from UAV.rover import rover

import datetime
import numpy as np
# import rospy

from UAV.sensor_msg import Imu


def thread_imu(Imu_message):
    print('IMU: thread starting ..')

    # rospy.Subscriber('uav_imu', Imu, rover.ros_imu_callback)
    # rate = rospy.Rate(100) # 100 hz


    freq = 100.0
    t = datetime.datetime.now()
    t_pre = datetime.datetime.now()
    avg_number = 100


    # while not rospy.is_shutdown() and rover.on:
    while rover.on:
        rover.ros_imu_callback(Imu_message)
        t = datetime.datetime.now()
        dt = (t - t_pre).total_seconds()
        if dt < 1e-6:
            continue
        # print(Imu_message.angular_velocity.x)
        freq = (freq * (avg_number - 1) + (1 / dt)) / avg_number
        t_pre = t
        rover.freq_imu = freq

        # rate.sleep()
    
    print('IMU: thread closed!')
