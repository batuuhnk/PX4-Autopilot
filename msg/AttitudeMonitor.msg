# Output of the attitude_monitor module (learning project).
# Euler angles, tilt, gyro rate and callback timing derived from vehicle_attitude.

uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample   # timestamp_sample of the vehicle_attitude that triggered this update

float32 roll              # roll angle (rad)
float32 pitch             # pitch angle (rad)
float32 yaw               # yaw angle (rad)
float32 tilt              # angle between body z axis and earth z axis (rad)
float32 gyro_norm         # magnitude of the body angular rate from sensor_combined (rad/s)

uint32 dt_us              # time between the last two attitude samples (us)
uint32 dt_max_us          # largest dt seen since start (us)

bool tilt_warning         # true while tilt is above AM_TILT_WARN
