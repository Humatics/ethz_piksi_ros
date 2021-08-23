#if available and synchronized to ROS (UTC) time, time of actual sensor measurement
#otherwise, use ros time now() when sensor measurement is first received by ROS
Header header

string child_frame_id

#name of track/line corresponding to along track component
string track_name

#TODO add segment ID to support track_geometry/curved track

string mobile_name

#position along track [m]
float64 along_track_component_m

#variance of along_track_component_m
float64 along_track_variance

#velocity along track [m/s]
float64 along_track_velocity

#variance of along_track_component_m
float64 along_track_velocity_variance

#use if along_track_velocity is not known
float64 UNKNOWN_ALONG_TRACK_VELOCITY = -1000000.0

#gnss data point in the frame specified by the header
geometry_msgs/PointStamped gnss_data_point

#gnss data point projected (closest point) on track in the frame specified by the header
geometry_msgs/PointStamped gnss_data_point_on_track

#distance between gnss data point and its closest point on track
float64 gnss_track_distance

#Components of the unit vector from the gnss projected point on track (the base) to the gnss data point (the tip)
geometry_msgs/PointStamped track_point_to_gnss_normal
