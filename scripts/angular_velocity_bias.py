#!/usr/bin/env python
"""
Go through the IMU calibration rosbag and output angular velocity bias
"""
import exceptions
import sys

import rosbag

if len(sys.argv) <= 1:
    raise exceptions.ImportError("Missing bagfile name")

bag = rosbag.Bag(sys.argv[1])

angular_velocities = [0.0, 0.0, 0.0]
ctr = 0
for topic, msg, t in bag.read_messages(topics=['/alphasense/imu']):
    ctr += 1
    temp = msg.angular_velocity
    angular_velocities[0] += temp.x
    angular_velocities[1] += temp.y
    angular_velocities[2] += temp.z

angular_velocities = [x/ctr for x in angular_velocities]
print(angular_velocities)
