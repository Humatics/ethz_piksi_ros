#if available and synchronized to ROS (UTC) time, time of actual sensor measurement
#otherwise, use ros time now() when sensor measurement is first received by ROS
Header header

#GPS time of week, in ms. 
#may be used for data association by matching different message types with the same time of week
#this contains incomplete information to specify atomic clock time
#also requires leap second data to convert to UTC
uint32 gps_tow

# north, east, down components of position in NED frame, in m/s
float64 n
float64 e
float64 d

#row major order of position covariance in NED frame, in m^2
float64[9] covariance

#number of satellites used in the fix.
uint8 n_sats

#Fix mode of rover
uint8 mode

#note distinction between SBP and ROS message values
uint8 FIX_MODE_INVALID = 0
uint8 FIX_MODE_SPP = 1 #single point precision
uint8 FIX_MODE_DGNSS = 2 #differential GNSS mode
uint8 FIX_MODE_FLOAT_RTK = 3
uint8 FIX_MODE_FIXED_RTK = 4
