#if available and synchronized to ROS (UTC) time, time of actual sensor measurement
#otherwise, use ros time now() when sensor measurement is first received by ROS
Header header

#GPS time of week, in ms. 
#may be used for data association by matching different message types with the same time of week
#this contains incomplete information to specify atomic clock time
#also requires leap second data to convert to UTC
uint32 gps_tow

# north, east, down components of velocity in NED frame, in m/s
float64 n
float64 e
float64 d

#row major order of velocity covariance in NED frame, in m^2/s^2
float64[9] covariance

#number of satellites used in the fix.
uint8 n_sats

#INS MODE of rover
uint8 ins_mode

uint8 INS_MODE_NONE = 0
uint8 INS_MODE_INS = 1

#Velocity mode of rover
uint8 vel_mode

uint8 VEL_MODE_INVALID = 0
uint8 VEL_MODE_MEASURED_DOPPLER_DERIVED = 1
uint8 VEL_MODE_COMPUTED_DOPPLER_DERIVED = 2
uint8 VEL_MODE_DEAD_RECKONING = 3
