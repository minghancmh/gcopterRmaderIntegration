# VioFilterState.msg
std_msgs/Header header

geometry_msgs/Pose pose
geometry_msgs/Twist twist  # includes gyro measurement for rates
geometry_msgs/Vector3 bw
geometry_msgs/Vector3 ba
geometry_msgs/Pose extrinsics  # camera-to-IMU transform

geometry_msgs/Vector3 accel_meas  # (IMU-frame) raw accel measurement

int32 N  # error state dimension
float32[] error_cov  # NxN error covariance.
                     # For consistency, leading 21 terms are ordered [pos, vel, Rwb, bw, ba, Rcb, tcb].
