#if available and synchronized to ROS (UTC) time, time of actual sensor measurement
#otherwise, use ros time now() when sensor measurement is first received by ROS
Header header

# flag to indicate RTK availability
bool rtk_available