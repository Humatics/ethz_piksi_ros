#pragma once
#include "ros/ros.h"
#include <boost/array.hpp>
#include <geometry_msgs/PointStamped.h>
#include <gnss_msgs/BaselinePosition.h>
#include <gnss_msgs/BaselineVelocity.h>
#include <geometry_msgs/PointStamped.h>
#include <piksi_rtk_msgs/BaselineNed.h>
#include <piksi_rtk_msgs/VelNedCov.h>
#include <libsbp_ros_msgs/MsgBaselineNed.h>
#include <libsbp_ros_msgs/MsgVelNedCov.h>

class SwiftNavRover{
public:
	SwiftNavRover(ros::NodeHandle* node_handle);
	ros::NodeHandle nh;
	
	// ros::NodeHandle rover;
	ros::Subscriber vel_sub;
	ros::Subscriber pos_sub;
	ros::Publisher ned_point_fix;
	ros::Publisher ned_baseline_position_fix;
	ros::Publisher ned_vel_cov_fix;
	void init();
	void publishBaselinePosition(ros::Time t, float n, float e, float d, int tow, boost::array<double, 9> covariance, int n_sats, int fixed_mode);
	void baselinePositionCallback(const libsbp_ros_msgs::MsgBaselineNed::ConstPtr & msg);
	void publishBaselineVelocity(ros::Time t, int tow,float n,float e,float d,boost::array<double, 9> covariance,int n_sats,int vel_mode,int ins_mode);
	void baselineVelocityCallback(const libsbp_ros_msgs::MsgVelNedCov::ConstPtr & msg);

	// ~SwiftNavRover();
};