/**
 * Tilt warning threshold
 *
 * attitude_monitor sets tilt_warning and prints a warning when the angle
 * between the body z axis and the vertical is above this value.
 *
 * @unit deg
 * @min 5
 * @max 90
 * @decimal 1
 * @group Attitude Monitor
 */
PARAM_DEFINE_FLOAT(AM_TILT_WARN, 30.0f);

/**
 * Start attitude_monitor with SIH
 *
 * When enabled, the SIH startup script starts attitude_monitor.
 *
 * @boolean
 * @reboot_required true
 * @group Attitude Monitor
 */
PARAM_DEFINE_INT32(AM_EN, 1);
