#!/usr/bin/env python
import rospy
import tf2_ros
from tf2_ros import TransformListener
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import do_transform_pose

tf2_excepts = (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException)


class OdometryCorrector:
    def __init__(self):
        self.buffer = tf2_ros.Buffer()
        self.tf = TransformListener(self.buffer)
        self.imu_pose_pub = rospy.Publisher("tf_odom_topic", Odometry, queue_size=10, latch=True)
        self.transf = None

    def get_transf(self):
        """Get transform if frames exists"""
        try:
            return self.buffer.lookup_transform("imu_sensor_frame", "base_link", rospy.Time(0))
        except tf2_excepts as e:
            print(e)
            rospy.logerr('FAILED TO GET TRANSFORM BETWEEN base_link and imu')
            return None

    def odom_cback(self, msg):
        # Get pose from the message
        odompose = msg.pose.pose

        # Pass into a PoseStamped for applying tf
        base_pose = PoseStamped()
        base_pose.header = msg.header
        base_pose.pose = odompose

        if self.transf is None:
            self.transf = self.get_transf()


        imu_pose = do_transform_pose(base_pose, self.transf)

        # Publish a odometry message with the pose
        # No `child_frame_id` inside a PoseStamped message
        corr_odom = Odometry()
        corr_odom.header = msg.header
        corr_odom.child_frame_id = "imu_sensor_frame"

        corr_odom.pose.pose = imu_pose.pose
        self.imu_pose_pub.publish(corr_odom)



if __name__ == "__main__":
    rospy.init_node("odom_corrector", anonymous=True)

    # Instantiate odometry corrector
    odom_crrctr = OdometryCorrector()

    rospy.Subscriber("odom_topic", Odometry, odom_crrctr.odom_cback)

    rospy.spin()





