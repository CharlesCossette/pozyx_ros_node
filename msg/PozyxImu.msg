Header header
uint16 id

geometry_msgs/Quaternion orientation
float64[9] orientation_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance # Row major x, y z 

geometry_msgs/Vector3 magnetic_field
float64[9] magnetic_field_covariance # Row major x, y z 

float64 roll
float64 pitch
float64 yaw
float64 pressure 
float64 pressure_variance
float64 temperature
float64 temperature_variance