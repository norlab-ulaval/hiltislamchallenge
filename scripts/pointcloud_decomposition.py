#!/usr/bin/env python
from __future__ import print_function, division
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2
import rospy
import ros_numpy


def pcloud_cback(msg):
    pcloud = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
    idxs = (0, len(pcloud) // 2, -1)
    print(pcloud.shape)
    pts = {i: pcloud[i] for i in idxs}

    for idx, pt in pts.items():
        info = {"i": idx, "p": pt, "r": pt[-1]}
        summary = "{i}: {r} / {p}".format(**info)
        rospy.loginfo(summary)


if __name__ == "__main__":
    rospy.init_node("PointCloudDecomposition", anonymous=True)
    rospy.Subscriber("pcloud_topic", PointCloud2, pcloud_cback)
    rospy.spin()
