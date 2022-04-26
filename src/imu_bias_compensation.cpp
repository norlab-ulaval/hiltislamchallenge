//
// Created by mbo on 26.4.22.
// Laval University, NORLAB, 2022
//

#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include <cmath>
#include <sstream>

std::vector<double> linear_velocity_bias_(3, 0.0);

// output imu stuff
ros::Publisher *imu_pub_;

void imuMsgCallback(const sensor_msgs::Imu &imu_msg) {

	sensor_msgs::Imu imu_msg_out = imu_msg;
	imu_msg_out.angular_velocity.x -= linear_velocity_bias_[0];
	imu_msg_out.angular_velocity.y -= linear_velocity_bias_[1];
	imu_msg_out.angular_velocity.z -= linear_velocity_bias_[2];

	imu_pub_->publish(imu_msg_out);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, ROS_PACKAGE_NAME);

	ros::NodeHandle n;
	ros::NodeHandle pn("~");

	// Quaternion for IMU alignment
	if (pn.hasParam("linear_velocity_bias")) {
		if (!pn.getParam("linear_velocity_bias", linear_velocity_bias_)) {
			ROS_WARN("Parameter linear_velocity_bias is not a list of three numbers, setting default 0,0,0");
		} else {
			if (linear_velocity_bias_.size() != 3) {
				ROS_WARN("Parameter linear_velocity_bias is not a list of three numbers, setting default 0,0,0");
				linear_velocity_bias_.assign(3, 0.0);
			} else {
				ROS_INFO("Found linear_velocity_bias values");
			}
		}
	}

	// Subscribe the IMU and start the loop
	imu_pub_ = new ros::Publisher(n.advertise<sensor_msgs::Imu>("/imu/data_out", 10));

	ros::Subscriber imu_subscriber = n.subscribe("/imu/data_in", 10, imuMsgCallback);

	ros::spin();

	delete imu_pub_;

	return 0;
}
