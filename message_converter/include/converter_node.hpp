#pragma once
#include "ros/ros.h"
#include <ros/console.h>
#include <boost/array.hpp>
#include <geometry_msgs/PointStamped.h>
#include <gnss_msgs/BaselinePosition.h>
#include <gnss_msgs/BaselineVelocity.h>
#include <gnss_msgs/RtkAvailable.h>
#include <geometry_msgs/PointStamped.h>
#include <piksi_rtk_msgs/BaselineNed.h>
#include <piksi_rtk_msgs/VelNedCov.h>
#include <libsbp_ros_msgs/MsgBaselineNed.h>
#include <libsbp_ros_msgs/MsgVelNedCov.h>
#include <libsbp_ros_msgs/MsgPosLlh.h>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <std_msgs/Bool.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include "libsbp_ros_msgs/ros_conversion.h"

class SwiftNavRover{
public:
    SwiftNavRover(ros::NodeHandle* node_handle, double la, double lo, double al);

    // WGS 84 derived geometric constants
    const double kSemimajorAxis = 6378137.0;
    const double kSemiminorAxis = 6356752.3142;
    const double kFirstEccentricitySquared = 6.69437999014 * 0.001;
    const double kSecondEccentricitySquared = 6.73949674228 * 0.001;
    const double kFlattening = 1/298.257223563;

    typedef Eigen::Matrix<double,3,3> Matrix3x3d;
    Matrix3x3d ecef_to_ned_matrix;

    ros::NodeHandle nh;
    
    void init();

    // Publisher functions
    void publishBaselinePosition(ros::Time t, double n, double e, double d, int tow, boost::array<double, 9> covariance, int n_sats, int fixed_mode);
    
    void publishBaselineVelocity(ros::Time t, int tow,double n,double e,double d,boost::array<double, 9> covariance,int n_sats,int vel_mode,int ins_mode);
    void publishRTKAvailable(ros::Time t);
    void publishStatus(const ros::TimerEvent& event);

    // Callback functions
    void baselineVelocityCallback(const libsbp_ros_msgs::MsgVelNedCov::ConstPtr & msg);
    void posLLHCallback(const libsbp_ros_msgs::MsgPosLlh::ConstPtr & msg);
    void baselinePositionCallback(const libsbp_ros_msgs::MsgBaselineNed::ConstPtr & msg);
    // void baselinePositionCallback(const piksi_rtk_msgs::PositionWithCovarianceStamped msg);

    // Co-ordinate conversions
    void geodetic2ned(double latitude, double longitude, double altitude, double* north, double* east, double* down);
    void geodetic2ecef(double latitude, double longitude, double altitude, double *x, double *y, double *z);
    void ecef2ned(double x_t, double y_t, double z_t, double* north, double* east, double* down);

    // Setters
    void setHomeECEF(double* x, double* y, double* z)
    {
        home_ecef_x_ = *x;
        home_ecef_y_ = *y;
        home_ecef_z_ = *z;
    }

    inline double deg2rad(double degrees)
    {
        return (degrees / 180.0) * M_PI;
    }

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

    private:
    // Timer to publish status messages at 1Hz
    ros::Timer status_timer_;

    double home_lat_, home_lon_, home_alt_;
    double home_ecef_x_, home_ecef_y_, home_ecef_z_;
    float home_lat_rad_, home_lon_rad_;

    // Declare subscribers
    ros::Subscriber vel_sub_;
    ros::Subscriber pos_sub_;
    ros::Subscriber pos_llh_sub_;

    // Declare publishers
    ros::Publisher ned_point_fix_;
    ros::Publisher ned_baseline_position_fix_;
    ros::Publisher ned_vel_cov_fix_;
    ros::Publisher rtk_mode_available_;
    ros::Publisher diagnostic_publisher_;

    // Flag to check if satellites are visible - if this drops to 0, 
    // RTK blackout has occured. If false, it means that RTK blackout 
    // has occured
    bool satellite_check_ = true;
    
    // Variable to store the operation mode of the rover - float RTK,
    // fix-RTK, SBAS, etc
    int positioning_mode_;


};