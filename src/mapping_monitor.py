#!/usr/bin/env python
from __future__ import print_function

import os
import time

import rosbag
import rospy
from sensor_msgs.msg import PointCloud2

"""
Monitor /map publish rate
For offline mapping
"""


class MappingMonitor:

    def __init__(self):
        rospy.init_node('mapping_monitor', anonymous=True)
        bag_file = rospy.get_param('~bagfile', '/home/mbo/norlab_data/hilti/construction_upper_level/bagfiles/exp_04_construction_upper_level_easy_2_2022-03-03-11-48-59.bag')
        lidar_topic = rospy.get_param('~points_in', '/hesai/pandar')
        self.file_name = os.path.basename(bag_file)
        with rosbag.Bag(bag_file) as bag:
            self.msg_count = int(bag.get_message_count(lidar_topic))
        rospy.loginfo("Expecting total number of %d messages" % (self.msg_count))
        self.time_prev_mes = 0
        self.rostime_prev_mes = 0
        self.first_msg = True
        rospy.Subscriber("map", PointCloud2, self.map_msg_callback)
        rospy.spin()

    def map_msg_callback(self, ptc):
        seq = int(ptc.header.seq)

        if self.first_msg:
            self.first_msg = False
            rospy.loginfo("Received first map message: %d/%d." % (seq, self.msg_count))
        else:
            rostime_diff = (ptc.header.stamp - self.rostime_prev_mes).to_sec()
            rospy.loginfo("Received a new map message: %d/%d. Time from last msg: %.2f s. Rostime from last msg: %.2f s. Msg rate: %.2f" % (
                seq, self.msg_count, time.time() - self.time_prev_mes, rostime_diff, 1/rostime_diff))
        self.time_prev_mes = time.time()
        self.rostime_prev_mes = ptc.header.stamp


if __name__ == '__main__':
    try:
        MappingMonitor()
    except rospy.ROSInterruptException:
        pass
