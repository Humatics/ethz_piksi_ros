// #include </home/ssonawane/Desktop/catkin_ws/src/ethz_piksi_ros/message_converter/include/message_converter/converter_node.hpp>
#include "ros/ros.h"
#include <ros/console.h>
#include <boost/array.hpp>
#include <geometry_msgs/PointStamped.h>
#include <gnss_msgs/BaselinePosition.h>
#include <gnss_msgs/BaselineVelocity.h>
#include <geometry_msgs/PointStamped.h>
#include <piksi_rtk_msgs/BaselineNed.h>
#include <piksi_rtk_msgs/VelNedCov.h>
#include <libsbp_ros_msgs/MsgBaselineNed.h>
#include <libsbp_ros_msgs/MsgVelNedCov.h>
#include <libsbp_ros_msgs/MsgPosLlh.h>
#include <eigen3/Eigen/Dense>
#include <math.h>

class SwiftNavRover{
public:
    SwiftNavRover(ros::NodeHandle* node_handle, double la, double lo, double al);

    const double kSemimajorAxis = 6378137.0;
    const double kSemiminorAxis = 6356752.3142;
    const double kFirstEccentricitySquared = 6.69437999014 * 0.001;
    const double kSecondEccentricitySquared = 6.73949674228 * 0.001;
    const double kFlattening = 1/298.257223563;

    bool satelliteCheck = true;

    float home_lat, home_lon, home_alt;
    float home_lat_rad, home_lon_rad;
    typedef Eigen::Matrix<double,3,3> Matrix3x3d;
    Matrix3x3d ecef_to_ned_matrix;

    ros::NodeHandle nh;
    
    ros::Subscriber vel_sub;
    ros::Subscriber pos_sub;
    ros::Subscriber pos_llh_sub;
    ros::Publisher ned_point_fix;
    ros::Publisher ned_baseline_position_fix;
    ros::Publisher ned_vel_cov_fix;
    void init();
    void publishBaselinePosition(ros::Time t, double n, double e, double d, int tow, boost::array<double, 9> covariance, int n_sats, int fixed_mode);
    void baselinePositionCallback(const libsbp_ros_msgs::MsgBaselineNed::ConstPtr & msg);
    void publishBaselineVelocity(ros::Time t, int tow,double n,double e,double d,boost::array<double, 9> covariance,int n_sats,int vel_mode,int ins_mode);
    void baselineVelocityCallback(const libsbp_ros_msgs::MsgVelNedCov::ConstPtr & msg);
    void posLLHCallback(const libsbp_ros_msgs::MsgPosLlh::ConstPtr & msg);
    void geodetic2ned(double latitude, double longitude, double altitude, double* north, double* east, double* down);
    void geodetic2ecef(double latitude, double longitude, double altitude, double *x, double *y, double *z);
    void ecef2ned(double x_t, double y_t, double z_t, double* north, double* east, double* down);
    double deg2rad(double degrees);
    inline Matrix3x3d nRe(const double lat_radians, const double lon_radians)
    {
        const double sLat = sin(lat_radians);
        const double sLon = sin(lon_radians);
        const double cLat = cos(lat_radians);
        const double cLon = cos(lon_radians);        
        Matrix3x3d ret;
        ret(0, 0) = -sLat * cLon;
        ret(0, 1) = -sLat * sLon;
        ret(0, 2) = cLat;
        ret(1, 0) = -sLon;
        ret(1, 1) = cLon;
        ret(1, 2) = 0.0;
        ret(2, 0) = cLat * cLon;
        ret(2, 1) = cLat * sLon;
        ret(2, 2) = sLat;
        return ret;
    }
};

SwiftNavRover::SwiftNavRover(ros::NodeHandle* node_handle, double hla = 0.0, double hlo = 0.0, double hal = 0.0):nh(*node_handle), 
home_lat(hla), home_lon(hlo), home_alt(hal){
    SwiftNavRover::init();
    SwiftNavRover::home_lon_rad = SwiftNavRover::deg2rad(home_lon);
    SwiftNavRover::home_lat_rad = SwiftNavRover::deg2rad(home_lat);
    double x,y,z;
    SwiftNavRover::geodetic2ecef(home_lat, home_lon, home_alt, &x, &y, &z);
    double phip = atan2(z, sqrt(pow(x,2)+pow(y,2)));
    SwiftNavRover::ecef_to_ned_matrix = SwiftNavRover::nRe(phip, SwiftNavRover::home_lon_rad);
}

void SwiftNavRover::init()
{
    vel_sub = nh.subscribe("/rover/piksi/position_receiver_0/sbp/vel_ned_cov", 1000, &SwiftNavRover::baselineVelocityCallback, this);
    pos_sub = nh.subscribe("/rover/piksi/position_receiver_0/sbp/baseline_ned", 1000, &SwiftNavRover::baselinePositionCallback, this);
    pos_llh_sub = nh.subscribe("/rover/piksi/position_receiver_0/sbp/pos_llh", 1000, &SwiftNavRover::posLLHCallback, this);
    ned_point_fix = nh.advertise<geometry_msgs::PointStamped>("/rover/ned_point_fix", 1000);
    ned_baseline_position_fix = nh.advertise<gnss_msgs::BaselinePosition>("/rover/ned_baseline_position_fix", 1000);
    ned_vel_cov_fix = nh.advertise<gnss_msgs::BaselineVelocity>("/rover/ned_vel_cov_fix", 1000);
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
    if(n_sats == 0){
        SwiftNavRover::satelliteCheck = false;
    }
    else
    {
        SwiftNavRover::satelliteCheck = true;
    }
    if(SwiftNavRover::satelliteCheck == true)
    {
        SwiftNavRover::publishBaselinePosition(t,n,e,d,tow,covariance,n_sats,fixed_mode);
    }
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

void SwiftNavRover::posLLHCallback(const libsbp_ros_msgs::MsgPosLlh::ConstPtr & msg)
{
    double n = 0.0;
    double e = 0.0;
    double d = 0.0;
    double lat = msg->lat;
    double lon = msg->lon;
    double height = msg->height;
    int n_sats = msg->n_sats;
    int h_accuracy = msg->h_accuracy;
    int v_accuracy = msg->v_accuracy;
    int tow = msg->tow;
    int fixed_mode = (msg->flags) & (0b00000111);
    boost::array<double, 9> covariance = {{h_accuracy, 0, 0, 0, h_accuracy, 0, 0, 0, v_accuracy}};
    ros::Time t = ros::Time::now();
    SwiftNavRover::geodetic2ned(lat, lon, height, &n, &e, &d);
    if(SwiftNavRover::satelliteCheck == false)
    {
        // ROS_INFO_STREAM("Publishing lat-lon converted position");
        SwiftNavRover::publishBaselinePosition(t,n,e,d,tow,covariance,n_sats,fixed_mode);
    }    
}

void SwiftNavRover::publishBaselinePosition(ros::Time t,double n,double e,double d,int tow,boost::array<double, 9> covariance,int n_sats,int fixed_mode)
{
    geometry_msgs::PointStamped msgPointStamped;
    msgPointStamped.header.frame_id = "ned";
    msgPointStamped.header.stamp = t;
    msgPointStamped.point.x = n;
    msgPointStamped.point.y = e;
    msgPointStamped.point.z = d;
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
    ned_baseline_position_fix.publish(msg);
}

void SwiftNavRover::publishBaselineVelocity(ros::Time t, int tow,double n,double e,double d,boost::array<double, 9> covariance,int n_sats,int vel_mode,int ins_mode)
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
    ned_vel_cov_fix.publish(msg);
}

void SwiftNavRover::geodetic2ned(double latitude, double longitude, double altitude, double* north, double* east, double* down)
{
    double x_temp, y_temp, z_temp;
    SwiftNavRover::geodetic2ecef(latitude, longitude, altitude, &x_temp, &y_temp, &z_temp);
    SwiftNavRover::ecef2ned(x_temp, y_temp, z_temp, north, east, down);    
}

void SwiftNavRover::geodetic2ecef(double latitude, double longitude, double altitude, double *x, double *y, double *z)
{
    double lat_rad = SwiftNavRover::deg2rad(latitude);
    double lon_rad = SwiftNavRover::deg2rad(longitude);
    double xi = sqrt(1 - kFirstEccentricitySquared * sin(lat_rad) * sin(lat_rad));
    *x = (kSemimajorAxis / xi + altitude) * cos(lat_rad) * cos(lon_rad);
    *y = (kSemimajorAxis / xi + altitude) * cos(lat_rad) * sin(lon_rad);
    *z = (kSemimajorAxis / xi * (1 - kFirstEccentricitySquared) + altitude) * sin(lat_rad);
}

void SwiftNavRover::ecef2ned(double x_t, double y_t, double z_t, double* north, double* east, double* down)
{
    Eigen::Vector3d vect, ret;
    double home_ecef_x, home_ecef_y, home_ecef_z;
    SwiftNavRover::geodetic2ecef(SwiftNavRover::home_lat, SwiftNavRover::home_lon, SwiftNavRover::home_alt, &home_ecef_x, &home_ecef_y, &home_ecef_z);
    vect(0) = x_t - home_ecef_x;
    vect(1) = y_t - home_ecef_y;
    vect(2) = z_t - home_ecef_z;   
    ret = SwiftNavRover::ecef_to_ned_matrix * vect;
    *north = ret(0);
    *east = ret(1);
    *down = -ret(2);
    std::cout << "ecef2ned: " << ret(0)<< " " << ret(1) << " " << -ret(2) << std::endl;    
}

double SwiftNavRover::deg2rad(double degrees)
{
    return (degrees / 180.0) * M_PI;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rover");
    ros::NodeHandle n;
    double home_latitude, home_longitude, home_altitude;
    n.getParam("/home_latitude", home_latitude);
    n.getParam("/home_longitude", home_longitude);
    n.getParam("/home_altitude", home_altitude);
    // Values in comments are from the parking lot test
    // double home_latitude = 42.3711071377; 
    // double home_longitude = -71.2168400352; 
    // double home_altitude = -15.3775837652; 
    SwiftNavRover obj(&n, home_latitude, home_longitude, home_altitude);
    ros::spin();
    return 0;
}
