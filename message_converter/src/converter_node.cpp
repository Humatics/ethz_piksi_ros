// #include </home/ssonawane/Desktop/catkin_ws/src/ethz_piksi_ros/message_converter/include/message_converter/converter_node.hpp>
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
};

SwiftNavRover::SwiftNavRover(ros::NodeHandle* node_handle):nh(*node_handle){
    SwiftNavRover::init();
}

void SwiftNavRover::init()
{
    vel_sub = nh.subscribe("/rover/piksi/position_receiver_0/sbp/vel_ned_cov", 1000, &SwiftNavRover::baselineVelocityCallback, this);
    pos_sub = nh.subscribe("/rover/piksi/position_receiver_0/sbp/baseline_ned", 1000, &SwiftNavRover::baselinePositionCallback, this);
}

void SwiftNavRover::baselinePositionCallback(const libsbp_ros_msgs::MsgBaselineNed::ConstPtr & msg)
{
    float n = 1e-3*(msg->n);
    float e = 1e-3*(msg->e);
    float d = 1e-3*(msg->d);
    int tow = msg->tow;
    int h_accuracy = msg->h_accuracy;
    int v_accuracy = msg->v_accuracy;
    boost::array<double, 9> covariance = {{h_accuracy, 0, 0, 0, h_accuracy, 0, 0, 0, v_accuracy}};
    int n_sats = msg->n_sats;
    int fixed_mode = (msg->flags) & (0b00000111);
    ros::Time t = ros::Time::now();
    SwiftNavRover::publishBaselinePosition(t,n,e,d,tow,covariance,n_sats,fixed_mode);

}

void SwiftNavRover::baselineVelocityCallback(const libsbp_ros_msgs::MsgVelNedCov::ConstPtr & msg)
{
    float n = 1e-3*(msg->n);
    float e = 1e-3*(msg->e);
    float d = 1e-3*(msg->d);
    int tow = msg->tow;
    float cov_n_n = msg->cov_n_n;
    float cov_n_e = msg->cov_n_e;
    float cov_n_d = msg->cov_n_d;
    float cov_e_e = msg->cov_e_e;
    float cov_e_d = msg->cov_e_d;
    float cov_d_d = msg->cov_d_d;
    int n_sats = msg->n_sats;
    int ins_mode = ((msg->flags) & (0b00011000)) >> 3;
    int vel_mode = (msg->flags) & (0b00000111);
    boost::array<double, 9> covariance = {{cov_n_n,cov_n_e,cov_n_d,cov_n_e,cov_e_e,cov_e_d,cov_n_d,cov_e_d,cov_d_d}};
    ros::Time t = ros::Time::now();
    SwiftNavRover::publishBaselineVelocity(t, tow, n, e, d, covariance, n_sats, vel_mode, ins_mode);
}

void SwiftNavRover::publishBaselinePosition(ros::Time t,float n,float e,float d,int tow,boost::array<double, 9> covariance,int n_sats,int fixed_mode)
{
    geometry_msgs::PointStamped msgPointStamped;
    msgPointStamped.header.frame_id = "ned";
    msgPointStamped.header.stamp = t;
    msgPointStamped.point.x = n;
    msgPointStamped.point.y = e;
    msgPointStamped.point.z = d;
    ned_point_fix = nh.advertise<geometry_msgs::PointStamped>("/rover/ned_point_fix", 1000);
    ned_point_fix.publish(msgPointStamped);
    gnss_msgs::BaselinePosition msg;
    msg.header.frame_id = "ned";
    msg.header.stamp = t;
    msg.gps_tow = tow;
    msg.n = n;
    msg.e = e;
    msg.d = d;
    msg.n_sats = n_sats;
    msg.covariance = covariance;
    msg.mode = fixed_mode;
    ned_baseline_position_fix = nh.advertise<gnss_msgs::BaselinePosition>("/rover/ned_baseline_position_fix", 1000);
    ned_baseline_position_fix.publish(msg);
}

void SwiftNavRover::publishBaselineVelocity(ros::Time t, int tow,float n,float e,float d,boost::array<double, 9> covariance,int n_sats,int vel_mode,int ins_mode)
{
    gnss_msgs::BaselineVelocity msg;
    msg.header.frame_id = "ned";
    msg.header.stamp = t;
    msg.gps_tow = tow;
    msg.n = n;
    msg.e = e;
    msg.d = d;
    msg.n_sats = n_sats;
    msg.covariance = covariance;
    msg.vel_mode = vel_mode;
    msg.ins_mode = ins_mode;
    ned_vel_cov_fix = nh.advertise<gnss_msgs::BaselineVelocity>("/rover/ned_vel_cov_fix", 1000);
    ned_vel_cov_fix.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rover");
    ros::NodeHandle n;
    SwiftNavRover obj(&n);
    ros::spin();
    return 0;
}