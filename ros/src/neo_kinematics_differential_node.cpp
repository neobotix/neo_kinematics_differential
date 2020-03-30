/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <Kinematics.h>
#include <DiffDrive2WKinematics.h>
#include <tf/transform_broadcaster.h>


class PlatformCtrlNode
{
public:
	virtual ~PlatformCtrlNode();

	ros::NodeHandle nh;
	ros::Publisher topicPub_Odometry;
	ros::Subscriber topicSub_DriveState;
	ros::Publisher topicPub_DriveCommands;
	ros::Subscriber topicSub_ComVel;
	tf::TransformBroadcaster odom_broadcaster;

	bool init();
	void receiveCmd(const geometry_msgs::Twist& twist);
	void receiveOdo(const sensor_msgs::JointState& js);

private:
	boost::mutex mutex;
	Kinematics* kin = 0;
	bool sendTransform = false;

};

PlatformCtrlNode::~PlatformCtrlNode()
{
	delete kin;
}

bool PlatformCtrlNode::init()
{
	nh.param<bool>("sendTransform", sendTransform, false);

	{
		double wheelDiameter = 0;
		double axisLength = 0;

		if(!nh.getParam("wheelDiameter", wheelDiameter)) {
			ROS_ERROR_STREAM("wheelDiameter param missing");
			return false;
		}
		if(!nh.getParam("robotWidth", axisLength)) {
			ROS_ERROR_STREAM("robotWidth param missing");
			return false;
		}

		DiffDrive2WKinematics* diffKin = new DiffDrive2WKinematics();
		diffKin->setWheelDiameter(wheelDiameter);
		diffKin->setAxisLength(axisLength);
		kin = diffKin;
	}

	topicPub_Odometry = nh.advertise<nav_msgs::Odometry>("/odom", 1);
	topicSub_DriveState = nh.subscribe("/drives/joint_states",1,&PlatformCtrlNode::receiveOdo, this);
	topicPub_DriveCommands = nh.advertise<trajectory_msgs::JointTrajectory>("/drives/joint_trajectory", 1);
	topicSub_ComVel = nh.subscribe("/cmd_vel", 1, &PlatformCtrlNode::receiveCmd, this);
	return true;
}

void PlatformCtrlNode::receiveCmd(const geometry_msgs::Twist& twist)
{
	boost::mutex::scoped_lock lock(mutex);

	trajectory_msgs::JointTrajectory traj;
	kin->execInvKin(twist, traj);
	topicPub_DriveCommands.publish(traj);
}

void PlatformCtrlNode::receiveOdo(const sensor_msgs::JointState& js)
{
	boost::mutex::scoped_lock lock(mutex);

	// odometry msgs
	nav_msgs::Odometry odom;
	kin->execForwKin(js, odom);
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";
	topicPub_Odometry.publish(odom);

	// odometry transform:
	if(sendTransform)
	{
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = odom.header.stamp;
		odom_trans.header.frame_id = odom.header.frame_id;
		odom_trans.child_frame_id = odom.child_frame_id;
		odom_trans.transform.translation.x = odom.pose.pose.position.x;
		odom_trans.transform.translation.y = odom.pose.pose.position.y;
		odom_trans.transform.translation.z = odom.pose.pose.position.z;
		odom_trans.transform.rotation = odom.pose.pose.orientation;
		odom_broadcaster.sendTransform(odom_trans);
	}
}


int main (int argc, char** argv)
{
    ros::init(argc, argv, "neo_kinematics_differential_node");

	PlatformCtrlNode node;

	if(!node.init()) {
		return -1;
	}

	ros::spin();

	return 0;
}


