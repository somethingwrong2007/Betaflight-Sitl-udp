/**
 * In-process Betaflight SITL API for simulators (SITL_LINK_MODE=LOCAL).
 *
 * Instead of exchanging fdm_packet / motor packets over UDP, the host engine
 * (e.g. Unreal's async physics tick at 1000 Hz) calls sitl_local_step()
 * synchronously: the flight controller receives the simulator state, runs one
 * scheduler pass on its virtual clock, and returns the motor outputs for that
 * exact state with zero network latency.
 *
 * The web configurator still works: the DLL keeps the TCP/WebSocket proxy
 * alive on 127.0.0.1:5761 / 6761, and a background thread services MSP.
 */

#ifndef SRC_SITL_LOCAL_H
#define SRC_SITL_LOCAL_H

#include <stdint.h>
#include <stdbool.h>

#if defined(_WIN32) && defined(SITL_LOCAL)
#define SITL_LOCAL_API __declspec(dllexport)
#else
#define SITL_LOCAL_API
#endif

#define SITL_LOCAL_MAX_RC_CHANNELS 16
#define SITL_LOCAL_MAX_MOTORS      16
#define SITL_LOCAL_MAX_SERVOS      8

typedef struct {
    double timestamp;                   // seconds, monotonic (logging only)
    double angular_velocity_rpy[3];     // rad/s, FRD body frame
    double linear_acceleration_xyz[3];  // m/s^2, FRD body specific force
    double orientation_quat[4];         // w,x,y,z, same convention as the UDP fdm_packet
    double velocity_xyz[3];             // m/s, earth frame ENU (east, north, up)
    double position_xyz[3];             // longitude, latitude, altitude (like UDP mode)
    double pressure;                    // Pa (ignored: derived from altitude)
    double battery_voltage;             // V
    double battery_current;             // A
    double motor_rpm[4];                // per-motor RPM (telemetry)
    uint16_t rc_channels[SITL_LOCAL_MAX_RC_CHANNELS]; // 1000..2000, AETR + aux
} sitl_local_input_t;

typedef struct {
    uint8_t motor_count;
    float pwm_output_raw[SITL_LOCAL_MAX_MOTORS]; // 1000..2000 (raw PWM)
    bool armed;
    // Fixed-wing / unusual mixers also produce servo outputs (after the
    // motors in the raw PWM stream). servo_count is 0 for multicopters.
    uint8_t servo_count;
    float servo_output_raw[SITL_LOCAL_MAX_SERVOS]; // 1000..2000 (raw PWM)
} sitl_local_output_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Boot the flight controller (equivalent to starting betaflight_SITL.exe).
 * Call once from the host process before the first step.
 */
SITL_LOCAL_API int sitl_local_init(void);

/**
 * Advance the virtual clock by dtUs and run one scheduler pass.
 * dtUs is normally 1000 (1 ms) for a 1000 Hz host tick.
 * Sensors/RC are taken from `in`, motor outputs are returned in `out`.
 */
SITL_LOCAL_API void sitl_local_step(const sitl_local_input_t *in, uint32_t dtUs,
                                    sitl_local_output_t *out);

/** Current virtual time in microseconds (the FC clock). */
SITL_LOCAL_API uint64_t sitl_local_time_us(void);

/**
 * Synchronous access to the in-memory flight-controller state, for hosts that
 * want to read/write the same structures the configurator sees over MSP
 * without touching the serial stream. All values below are the exact same
 * memory the MSP handlers read/write, so both sides stay consistent.
 */

/** Bitmask of arming-disable reasons; same value as MSP_STATUS_EX. 0 = may arm. */
SITL_LOCAL_API uint32_t sitl_local_get_arming_flags(void);

/** True while arming is currently blocked by any arming-disable reason. */
SITL_LOCAL_API bool sitl_local_is_arming_disabled(void);

/** True when the FC is armed (same flag as the configurator's status icon). */
SITL_LOCAL_API bool sitl_local_get_armed(void);

/** flightModeFlags bitmask (ANGLE/HORIZON/MAG/... active modes). */
SITL_LOCAL_API uint32_t sitl_local_get_flight_modes(void);

/**
 * Read a rate profile. index in [0, CONTROL_RATE_PROFILE_COUNT) selects that
 * profile; any other value reads the current profile. Each output is a
 * 3-element array in axis order ROLL, PITCH, YAW (same columns as the
 * configurator's Rates tab). Units always match the Rates tab for the current
 * rate mode: rcRate is the RC Rate column, rcExpo the Expo column and
 * superRate the Super Rate / max-velocity column (e.g. deg/s in ACTUAL mode).
 * Any output pointer may be NULL to skip that group.
 */
SITL_LOCAL_API void sitl_local_get_rate(int index,
                                        float rcRate[3], float rcExpo[3],
                                        float superRate[3]);

/**
 * Write the current rate profile per axis (arrays in ROLL, PITCH, YAW order,
 * same display units as get_rate) and persist it to the virtual EEPROM via
 * the background thread. Each pointer may be NULL to leave that axis group
 * unchanged.
 */
SITL_LOCAL_API void sitl_local_set_rate(const float rcRate[3],
                                        const float rcExpo[3],
                                        const float superRate[3]);

/**
 * Rate mode of the current profile (rates_type): 0 = BETAFLIGHT,
 * 1 = RACEFLIGHT, 2 = KISS, 3 = ACTUAL, 4 = QUICK - the same values the
 * configurator's Rates tab stores. Returns -1 before sitl_local_init().
 */
SITL_LOCAL_API int sitl_local_get_rate_mode(void);

/**
 * Set the rate mode of the current profile and persist it. Accepts the same
 * values as get_rate_mode (0..RATES_TYPE_COUNT-1). Returns 0 on success,
 * -1 for an invalid mode.
 */
SITL_LOCAL_API int sitl_local_set_rate_mode(int mode);

/**
 * Locate the ARM switch in the mode-activation conditions. On success
 * auxChannel is the RC channel index (4 = AUX1) and startStep/endStep the
 * 25us-step range (900..2100us). When no ARM condition is configured,
 * auxChannel is set to 0xFF and the steps to 0.
 */
SITL_LOCAL_API void sitl_local_get_arm_switch(uint8_t *auxChannel,
                                              uint8_t *startStep,
                                              uint8_t *endStep);

/**
 * Redirect the blackbox log directory at runtime (e.g. one folder per
 * aircraft in UE). The directory is created if missing, and the log-number
 * scan is re-run for that folder so LOG00001.BFL numbering is correct and
 * existing logs are never overwritten. Returns 0 on success, -1 for a NULL,
 * empty or too-long path. Call it before arming / logging starts; a log that
 * is already in progress keeps writing to its original file.
 */
SITL_LOCAL_API int sitl_local_set_blackbox_dir(const char *path);

/** Stop the background MSP thread. Does not exit the host process. */
SITL_LOCAL_API void sitl_local_shutdown(void);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // SRC_SITL_LOCAL_H
