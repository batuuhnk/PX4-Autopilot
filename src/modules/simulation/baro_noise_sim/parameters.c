/**
 * Simulated baro white noise (Pa)
 *
 * Standard deviation of the Gaussian noise added to each pressure sample.
 * 12 Pa is roughly 1 m of altitude near sea level.
 *
 * @min 0
 * @max 200
 * @decimal 2
 * @group Baro Noise Sim
 */
PARAM_DEFINE_FLOAT(BNS_NOISE, 3.0f);

/**
 * Simulated baro bias drift (Pa/s)
 *
 * Rate at which a pressure bias builds up over time, in Pa per second.
 *
 * @min -50
 * @max 50
 * @decimal 3
 * @group Baro Noise Sim
 */
PARAM_DEFINE_FLOAT(BNS_DRIFT, 0.0f);

/**
 * Simulated baro fault mode
 *
 * @value 0 None
 * @value 1 Stuck output
 * @value 2 Step offset (BNS_STEP)
 * @value 3 Dropout
 * @group Baro Noise Sim
 */
PARAM_DEFINE_INT32(BNS_FAULT, 0);

/**
 * Simulated baro step offset (Pa)
 *
 * Pressure offset applied when BNS_FAULT is 2. Positive pressure means lower altitude.
 *
 * @min -2000
 * @max 2000
 * @decimal 1
 * @group Baro Noise Sim
 */
PARAM_DEFINE_FLOAT(BNS_STEP, 120.0f);

/**
 * Simulated baro output rate
 *
 * @unit Hz
 * @min 1
 * @max 100
 * @group Baro Noise Sim
 */
PARAM_DEFINE_INT32(BNS_RATE, 20);

/**
 * Start baro_noise_sim with SIH
 *
 * When enabled, the SIH startup script starts baro_noise_sim.
 * Set SENS_EN_BAROSIM to 0 at the same time so this is the only barometer.
 * Takes effect after reboot.
 *
 * @boolean
 * @reboot_required true
 * @group Baro Noise Sim
 */
PARAM_DEFINE_INT32(BNS_EN, 0);
